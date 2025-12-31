#include "rtsp_server.h"
#include "utils.h"
#include <iostream>
#include <sstream>
#include <cstring>

// GStreamer 콜백: 미디어 요소 생성
GstElement* RTSPServer::create_media_element(GstRTSPMediaFactory* factory, const GstRTSPUrl* url) {
    RTSPData* data = static_cast<RTSPData*>(g_object_get_data(G_OBJECT(factory), "rtsp-data"));
    if (!data) {
        return nullptr;
    }
    
    std::string launch_string = static_cast<const char*>(g_object_get_data(G_OBJECT(factory), "launch-string"));
    if (launch_string.empty()) {
        return nullptr;
    }
    
    GstElement* pipeline = gst_parse_launch(launch_string.c_str(), nullptr);
    if (!pipeline) {
        return nullptr;
    }
    
    data->appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    if (data->appsrc) {
        g_object_set(data->appsrc,
                   "format", GST_FORMAT_TIME,
                   "is-live", TRUE,
                   "do-timestamp", TRUE,
                   nullptr);
        
        if (!data->push_thread || 
            (data->push_thread && !data->push_thread->joinable())) {
            data->is_running = true;
            data->push_thread = new std::thread(push_frames_thread, data);
        }
    }
    
    return pipeline;
}

// 프레임 푸시 스레드
void RTSPServer::push_frames_thread(RTSPData* data) {
    GstClockTime frame_duration = GST_SECOND / OUTPUT_FPS;
    GstClockTime timestamp = 0;
    
    while (data->is_running && data->frame_queue) {
        cv::Mat frame;
        if (!data->frame_queue->try_pop(frame, 200)) {
            continue;
        }
        
        if (frame.empty() || !data->appsrc) {
            continue;
        }
        
        // OpenCV Mat을 GStreamer 버퍼로 변환
        size_t data_size = frame.total() * frame.elemSize();
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, data_size, nullptr);
        if (!buffer) {
            continue;
        }
        
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
            std::memcpy(map.data, frame.data, data_size);
            gst_buffer_unmap(buffer, &map);
        } else {
            gst_buffer_unref(buffer);
            continue;
        }
        
        GST_BUFFER_PTS(buffer) = timestamp;
        GST_BUFFER_DTS(buffer) = timestamp;
        GST_BUFFER_DURATION(buffer) = frame_duration;
        
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data->appsrc), buffer);
        if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
            gst_buffer_unref(buffer);
            break;
        }
        
        timestamp += frame_duration;
    }
}

RTSPServer::RTSPServer() 
    : server_(nullptr), factory_(nullptr), loop_(nullptr), rtsp_data_(nullptr), is_running_(false) {
}

RTSPServer::~RTSPServer() {
    stop();
}

bool RTSPServer::initialize(ThreadSafeQueue<cv::Mat>* frame_queue) {
    server_ = gst_rtsp_server_new();
    if (!server_) {
        std::cerr << "  ✗ RTSP 서버 생성 실패" << std::endl;
        return false;
    }
    
    g_object_set(server_, "service", RTSP_PORT, nullptr);
    
    // 팩토리 생성
    factory_ = gst_rtsp_media_factory_new();
    if (!factory_) {
        std::cerr << "  ✗ RTSP 팩토리 생성 실패" << std::endl;
        g_object_unref(server_);
        server_ = nullptr;
        return false;
    }
    
    std::string launch_string = get_launch_string();
    
    // launch_string을 팩토리에 저장
    gst_rtsp_media_factory_set_launch(factory_, launch_string.c_str());
    gst_rtsp_media_factory_set_shared(factory_, TRUE);
    
    // RTSP 데이터 구조체 생성 및 저장
    rtsp_data_ = new RTSPData();
    rtsp_data_->frame_queue = frame_queue;
    
    g_object_set_data_full(G_OBJECT(factory_), "rtsp-data", rtsp_data_, 
                          [](gpointer data) { delete static_cast<RTSPData*>(data); });
    g_object_set_data_full(G_OBJECT(factory_), "launch-string", 
                          g_strdup(launch_string.c_str()), g_free);
    
    // create_element 콜백 설정
    GstRTSPMediaFactoryClass* klass = GST_RTSP_MEDIA_FACTORY_GET_CLASS(factory_);
    klass->create_element = create_media_element;
    
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server_);
    gst_rtsp_mount_points_add_factory(mounts, RTSP_MOUNT_POINT, factory_);
    g_object_unref(mounts);
    
    return true;
}

void RTSPServer::start() {
    if (!server_) {
        return;
    }
    
    gst_rtsp_server_attach(server_, nullptr);
    
    loop_ = g_main_loop_new(nullptr, FALSE);
    is_running_ = true;
    server_thread_ = std::thread([this]() {
        g_main_loop_run(loop_);
    });
}

void RTSPServer::stop() {
    is_running_ = false;
    
    // RTSP 데이터 정리 (스레드 먼저 종료)
    if (rtsp_data_) {
        rtsp_data_->is_running = false;
        if (rtsp_data_->push_thread && rtsp_data_->push_thread->joinable()) {
            rtsp_data_->push_thread->join();
            delete rtsp_data_->push_thread;
            rtsp_data_->push_thread = nullptr;
        }
        // rtsp_data_는 factory_의 데이터로 관리되므로 여기서 delete하지 않음
    }
    
    if (loop_) {
        g_main_loop_quit(loop_);
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        g_main_loop_unref(loop_);
        loop_ = nullptr;
    }
    
    // factory_는 server_가 소유하므로 server_ 해제 전에 해제하지 않음
    // factory_는 server_ 해제 시 자동으로 해제됨
    
    if (server_) {
        g_object_unref(server_);
        server_ = nullptr;
        // factory_는 server_와 함께 해제됨
        factory_ = nullptr;
    }
    
    // rtsp_data_는 factory_의 데이터로 관리되므로 자동 해제됨
    rtsp_data_ = nullptr;
}

std::string RTSPServer::get_rtsp_url() const {
    std::string ip = get_local_ip();
    return "rtsp://" + ip + ":" + RTSP_PORT + RTSP_MOUNT_POINT;
}

std::string RTSPServer::get_launch_string() {
    std::ostringstream oss;
    oss << "appsrc name=src is-live=true format=time do-timestamp=true "
        << "caps=video/x-raw,format=RGB,width=" << OUTPUT_WIDTH 
        << ",height=" << OUTPUT_HEIGHT 
        << ",framerate=" << OUTPUT_FPS << "/1 ! "
        << "videoconvert ! ";
    
    if (USE_HARDWARE_ENCODER && check_encoder("amlvenc")) {
        oss << "video/x-raw,format=NV12 ! amlvenc bitrate=" << BITRATE_KBPS 
            << " gop=" << OUTPUT_FPS << " ! ";
    } else if (check_encoder("openh264enc")) {
        oss << "video/x-raw,format=I420 ! openh264enc bitrate=" << (BITRATE_KBPS * 1000) 
            << " complexity=0 ! ";
    } else if (check_encoder("x264enc")) {
        oss << "video/x-raw,format=I420 ! x264enc tune=zerolatency bitrate=" << BITRATE_KBPS 
            << " speed-preset=ultrafast ! ";
    } else {
        oss << "video/x-raw,format=I420 ! openh264enc bitrate=" << (BITRATE_KBPS * 1000) << " ! ";
    }
    
    oss << "video/x-h264,profile=baseline ! h264parse config-interval=1 ! rtph264pay name=pay0 pt=96";
    
    return oss.str();
}

bool RTSPServer::check_encoder(const std::string& encoder_name) {
    return ::check_encoder(encoder_name);
}
