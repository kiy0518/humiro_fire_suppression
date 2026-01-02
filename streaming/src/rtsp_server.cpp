#include "rtsp_server.h"
#include "../../thermal/src/utils.h"
#include "../../thermal/src/config.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

// GStreamer 콜백: 미디어 요소 생성
GstElement* RTSPServer::create_media_element(GstRTSPMediaFactory* factory, const GstRTSPUrl* url) {
    // 팩토리에서 공유 데이터 가져오기 (frame_queue는 공유)
    RTSPData* shared_data = static_cast<RTSPData*>(g_object_get_data(G_OBJECT(factory), "rtsp-data"));
    if (!shared_data || !shared_data->frame_queue) {
        return nullptr;
    }
    
    std::string launch_string = static_cast<const char*>(g_object_get_data(G_OBJECT(factory), "launch-string"));
    if (launch_string.empty()) {
        return nullptr;
    }
    
    // 각 클라이언트마다 독립적인 파이프라인 생성
    GstElement* pipeline = gst_parse_launch(launch_string.c_str(), nullptr);
    if (!pipeline) {
        return nullptr;
    }
    
    // 각 파이프라인마다 독립적인 RTSPData 생성
    RTSPData* pipeline_data = new RTSPData();
    pipeline_data->frame_queue = shared_data->frame_queue;  // frame_queue는 공유
    pipeline_data->appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    
    if (pipeline_data->appsrc) {
        g_object_set(pipeline_data->appsrc,
                   "format", GST_FORMAT_TIME,
                   "is-live", TRUE,
                   "do-timestamp", TRUE,
                   nullptr);
        
        // 각 파이프라인마다 독립적인 push_thread 시작
        pipeline_data->is_running = true;
        pipeline_data->push_thread = new std::thread(push_frames_thread, pipeline_data);
    }
    
    // 파이프라인에 RTSPData 연결 (정리 시 사용)
    g_object_set_data_full(G_OBJECT(pipeline), "rtsp-data", pipeline_data,
                          [](gpointer data) {
                              RTSPData* d = static_cast<RTSPData*>(data);
                              if (d) {
                                  d->is_running = false;
                                  if (d->push_thread && d->push_thread->joinable()) {
                                      d->push_thread->join();
                                      delete d->push_thread;
                                      d->push_thread = nullptr;
                                  }
                                  delete d;
                              }
                          });
    
    return pipeline;
}

// 미디어 구성 완료 콜백
void RTSPServer::media_configured(GstRTSPMediaFactory* factory, GstRTSPMedia* media) {
    // 미디어가 구성되었을 때 호출됨
    // 새로운 클라이언트 연결 시 호출됨
    std::cout << "  ✓ RTSP: New client connected (pipeline created)" << std::endl;
}

// 미디어 해제 콜백 (클라이언트 연결 해제 시 호출)
void RTSPServer::media_unprepared(GstRTSPMedia* media) {
    // 클라이언트 연결이 끊어졌을 때 호출됨
    // shared=FALSE로 설정했으므로 각 클라이언트의 파이프라인이 독립적으로 정리됨
    // 파이프라인의 RTSPData는 g_object_set_data_full의 destroy 함수에서 자동 정리됨
    std::cout << "  ✓ RTSP: Client disconnected, pipeline and thread cleaned up" << std::endl;
}

// 프레임 푸시 스레드
void RTSPServer::push_frames_thread(RTSPData* data) {
    GstClockTime frame_duration = GST_SECOND / OUTPUT_FPS;
    GstClockTime timestamp = 0;
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 10;
    
    while (data->is_running && data->frame_queue) {
        cv::Mat frame;
        if (!data->frame_queue->try_pop(frame, 200)) {
            continue;
        }
        
        if (frame.empty()) {
            continue;
        }
        
        // appsrc 유효성 검사
        if (!data->appsrc) {
            // appsrc가 없으면 잠시 대기 후 재시도
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // appsrc 상태 확인
        GstState state;
        GstState pending;
        GstElement* parent = GST_ELEMENT_PARENT(data->appsrc);
        if (parent) {
            gst_element_get_state(parent, &state, &pending, GST_CLOCK_TIME_NONE);
            if (state == GST_STATE_NULL || state == GST_STATE_READY) {
                // 파이프라인이 정지 상태면 대기
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
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
        if (ret == GST_FLOW_OK) {
            consecutive_errors = 0;
            timestamp += frame_duration;
        } else if (ret == GST_FLOW_FLUSHING) {
            // 정상적인 종료 중
            gst_buffer_unref(buffer);
            break;
        } else {
            // 오류 발생
            gst_buffer_unref(buffer);
            consecutive_errors++;
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                std::cerr << "  ⚠ RTSP push thread: Too many consecutive errors, stopping" << std::endl;
                break;
            }
            // 오류 시 잠시 대기 후 재시도
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
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
    // shared=TRUE: 여러 클라이언트가 같은 파이프라인 공유 (메모리 효율적)
    // shared=FALSE: 각 클라이언트마다 독립적인 파이프라인 생성 (재연결 시 더 안정적)
    // QGC 재연결 문제 해결을 위해 FALSE로 변경
    gst_rtsp_media_factory_set_shared(factory_, FALSE);
    
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
    
    // 미디어 상태 변경 콜백 등록
    g_signal_connect(factory_, "media-configured", G_CALLBACK(media_configured), nullptr);
    g_signal_connect(factory_, "media-unprepared", G_CALLBACK(media_unprepared), nullptr);
    
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
    
    // 팩토리 레벨의 rtsp_data_는 frame_queue만 저장하므로
    // 각 파이프라인의 독립적인 RTSPData는 파이프라인 정리 시 자동으로 정리됨
    // (g_object_set_data_full의 destroy 함수에서 처리)
    
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
    std::string ip;
    
    // config.h에서 RTSP_IP가 정의되어 있으면 사용, 없으면 자동 감지
    #ifdef RTSP_IP
    const char* rtsp_ip = RTSP_IP;
    if (rtsp_ip && strlen(rtsp_ip) > 0) {
        ip = rtsp_ip;
        
    } else {
        ip = get_local_ip();
    }
    #else
    ip = get_local_ip();
    #endif
    
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

