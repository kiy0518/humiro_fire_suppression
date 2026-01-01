#include "http_server.h"
#include "../../thermal/src/utils.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>

HTTPServer::HTTPServer() 
    : daemon_(nullptr), frame_queue_(nullptr), is_running_(false) {
}

HTTPServer::~HTTPServer() {
    stop();
}

bool HTTPServer::initialize(ThreadSafeQueue<cv::Mat>* frame_queue) {
    frame_queue_ = frame_queue;
    return true;
}

void HTTPServer::start() {
    if (daemon_) {
        return;
    }
    
    daemon_ = MHD_start_daemon(
        MHD_USE_SELECT_INTERNALLY,
        HTTP_PORT,
        nullptr, nullptr,
        &handle_request, this,
        MHD_OPTION_END
    );
    
    if (daemon_) {
        is_running_ = true;
        std::cout << "  ✓ HTTP 웹 서버 시작: http://" << get_local_ip() << ":" << HTTP_PORT << std::endl;
    } else {
        std::cerr << "  ⚠ HTTP 웹 서버 시작 실패" << std::endl;
    }
}

void HTTPServer::stop() {
    if (daemon_) {
        MHD_stop_daemon(daemon_);
        daemon_ = nullptr;
        is_running_ = false;
    }
}

std::string HTTPServer::get_http_url() const {
    std::string ip = get_local_ip();
    return "http://" + ip + ":" + std::to_string(HTTP_PORT);
}

MHD_Result HTTPServer::handle_request(void* cls, struct MHD_Connection* connection,
                               const char* url, const char* method,
                               const char* version, const char* upload_data,
                               size_t* upload_data_size, void** con_cls) {
    HTTPServer* server = static_cast<HTTPServer*>(cls);
    
    if (std::strcmp(method, "GET") != 0) {
        return MHD_NO;
    }
    
    if (std::strcmp(url, "/") == 0 || std::strcmp(url, "/index.html") == 0) {
        return handle_index(server, connection);
    } else if (std::strcmp(url, "/video_feed") == 0) {
        return handle_video_feed(server, connection);
    }
    
    return MHD_NO;
}

MHD_Result HTTPServer::handle_index(void* cls, struct MHD_Connection* connection) {
    HTTPServer* server = static_cast<HTTPServer*>(cls);
    std::string ip = get_local_ip();
    
    std::ostringstream html;
    html << "<!DOCTYPE html>\n"
         << "<html>\n"
         << "<head>\n"
         << "  <title>RGB + 열화상 스트림</title>\n"
         << "  <meta charset=\"utf-8\">\n"
         << "  <style>\n"
         << "    body { margin: 0; padding: 20px; background: #1a1a1a; color: #fff; font-family: Arial, sans-serif; }\n"
         << "    h1 { text-align: center; color: #4CAF50; }\n"
         << "    .container { max-width: 800px; margin: 0 auto; }\n"
         << "    img { width: 100%; height: auto; border: 2px solid #4CAF50; border-radius: 5px; }\n"
         << "    .info { margin-top: 10px; padding: 10px; background: #2a2a2a; border-radius: 5px; }\n"
         << "  </style>\n"
         << "</head>\n"
         << "<body>\n"
         << "  <div class=\"container\">\n"
         << "    <h1>RGB + 열화상 스트림</h1>\n"
         << "    <img src=\"/video_feed\" alt=\"Video Stream\">\n"
         << "    <div class=\"info\">\n"
         << "      <p><strong>RTSP URL:</strong> rtsp://" << ip << ":" << RTSP_PORT << RTSP_MOUNT_POINT << "</p>\n"
         << "      <p><strong>HTTP 스트림:</strong> http://" << ip << ":" << HTTP_PORT << "/video_feed</p>\n"
         << "    </div>\n"
         << "  </div>\n"
         << "</body>\n"
         << "</html>\n";
    
    std::string html_str = html.str();
    struct MHD_Response* response = MHD_create_response_from_buffer(
        html_str.length(),
        const_cast<char*>(html_str.c_str()),
        MHD_RESPMEM_MUST_COPY
    );
    
    MHD_add_response_header(response, "Content-Type", "text/html; charset=utf-8");
    MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);
    
    return ret;
}

// MJPEG 스트림을 위한 구조체
struct StreamData {
    ThreadSafeQueue<cv::Mat>* frame_queue;
    std::atomic<bool>* is_running;
    std::vector<uchar> current_frame_data;
    size_t current_pos;
    
    StreamData() : frame_queue(nullptr), is_running(nullptr), current_pos(0) {}
};

static ssize_t stream_reader(void* cls, uint64_t pos, char* buf, size_t max) {
    StreamData* stream_data = static_cast<StreamData*>(cls);
    
    if (!stream_data->frame_queue || !(*stream_data->is_running)) {
        return 0;
    }
    
    // 새로운 프레임이 필요하면 가져오기
    if (stream_data->current_pos >= stream_data->current_frame_data.size()) {
        cv::Mat frame;
        if (!stream_data->frame_queue->try_pop(frame, 100)) {
            // 프레임이 없으면 작은 지연 후 재시도
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return 0;
        }
        
        if (frame.empty()) {
            return 0;
        }
        
        // RGB → BGR 변환
        cv::Mat frame_bgr;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, frame_bgr, cv::COLOR_RGB2BGR);
        } else {
            frame_bgr = frame;
        }
        
        // JPEG 인코딩
        std::vector<uchar> jpeg_data;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
        if (!cv::imencode(".jpg", frame_bgr, jpeg_data, params)) {
            return 0;
        }
        
        // MJPEG 형식: boundary + headers + data
        std::ostringstream mjpeg_frame;
        mjpeg_frame << "--frame\r\n"
                    << "Content-Type: image/jpeg\r\n"
                    << "Content-Length: " << jpeg_data.size() << "\r\n\r\n";
        
        std::string header = mjpeg_frame.str();
        stream_data->current_frame_data.clear();
        stream_data->current_frame_data.reserve(header.size() + jpeg_data.size());
        
        // 헤더 추가
        for (char c : header) {
            stream_data->current_frame_data.push_back(static_cast<uchar>(c));
        }
        // JPEG 데이터 추가
        stream_data->current_frame_data.insert(
            stream_data->current_frame_data.end(),
            jpeg_data.begin(),
            jpeg_data.end()
        );
        
        stream_data->current_pos = 0;
    }
    
    // 버퍼에 데이터 복사
    size_t remaining = stream_data->current_frame_data.size() - stream_data->current_pos;
    size_t to_copy = std::min(remaining, max);
    
    if (to_copy > 0) {
        std::memcpy(buf, 
                   stream_data->current_frame_data.data() + stream_data->current_pos, 
                   to_copy);
        stream_data->current_pos += to_copy;
        
        // 프레임을 모두 전송했으면 다음 프레임을 위해 리셋
        if (stream_data->current_pos >= stream_data->current_frame_data.size()) {
            stream_data->current_frame_data.clear();
            stream_data->current_pos = 0;
        }
        
        return to_copy;
    }
    
    return 0;
}

static void stream_free(void* cls) {
    delete static_cast<StreamData*>(cls);
}

MHD_Result HTTPServer::handle_video_feed(void* cls, struct MHD_Connection* connection) {
    HTTPServer* server = static_cast<HTTPServer*>(cls);
    
    if (!server->frame_queue_) {
        return MHD_NO;
    }
    
    // 스트림 데이터 구조체 생성
    StreamData* stream_data = new StreamData();
    stream_data->frame_queue = server->frame_queue_;
    stream_data->is_running = &server->is_running_;
    
    // MJPEG 스트림 응답 생성
    struct MHD_Response* response = MHD_create_response_from_callback(
        MHD_SIZE_UNKNOWN,
        8192,  // 버퍼 크기 증가
        stream_reader,
        stream_data,
        stream_free
    );
    
    if (!response) {
        delete stream_data;
        return MHD_NO;
    }
    
    MHD_add_response_header(response, "Content-Type", "multipart/x-mixed-replace; boundary=frame");
    MHD_add_response_header(response, "Cache-Control", "no-cache, no-store, must-revalidate");
    MHD_add_response_header(response, "Pragma", "no-cache");
    MHD_add_response_header(response, "Expires", "0");
    
    MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);
    
    return ret;
}

void HTTPServer::request_completed(void* cls, struct MHD_Connection* connection,
                                  void** con_cls, enum MHD_RequestTerminationCode toe) {
    // 정리 작업 (필요시)
}
