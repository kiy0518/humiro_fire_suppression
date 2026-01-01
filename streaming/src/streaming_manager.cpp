#include "streaming_manager.h"
#include "../../thermal/src/config.h"
#include <iostream>

StreamingManager::StreamingManager()
    : rtsp_server_(nullptr)
    , http_server_(nullptr)
    , rtsp_queue_(nullptr)
    , http_queue_(nullptr)
    , http_enabled_(false)
    , initialized_(false) {
}

StreamingManager::~StreamingManager() {
    stop();
    if (rtsp_server_) {
        delete rtsp_server_;
        rtsp_server_ = nullptr;
    }
    if (http_server_) {
        delete http_server_;
        http_server_ = nullptr;
    }
}

bool StreamingManager::initialize(ThreadSafeQueue<cv::Mat>* rtsp_queue, 
                                   ThreadSafeQueue<cv::Mat>* http_queue) {
    if (initialized_) {
        std::cerr << "  ⚠ StreamingManager already initialized" << std::endl;
        return false;
    }
    
    rtsp_queue_ = rtsp_queue;
    http_queue_ = http_queue;
    
    if (!rtsp_queue_) {
        std::cerr << "  ✗ RTSP queue is required" << std::endl;
        return false;
    }
    
    // RTSP 서버 초기화
    rtsp_server_ = new RTSPServer();
    if (!rtsp_server_->initialize(rtsp_queue_)) {
        std::cerr << "  ✗ RTSP 서버 초기화 실패" << std::endl;
        delete rtsp_server_;
        rtsp_server_ = nullptr;
        return false;
    }
    
    // HTTP 서버 초기화 (옵션)
    if (http_queue_ && ENABLE_HTTP_SERVER) {
        http_server_ = new HTTPServer();
        if (http_server_->initialize(http_queue_)) {
            http_enabled_ = true;
            std::cout << "  ✓ HTTP 서버 초기화 완료" << std::endl;
        } else {
            std::cout << "  ⚠ HTTP 서버 초기화 실패 (계속 진행)" << std::endl;
            delete http_server_;
            http_server_ = nullptr;
        }
    }
    
    initialized_ = true;
    return true;
}

void StreamingManager::start() {
    if (!initialized_) {
        std::cerr << "  ⚠ StreamingManager not initialized" << std::endl;
        return;
    }
    
    // RTSP 서버 시작
    if (rtsp_server_) {
        rtsp_server_->start();
        std::cout << "  ✓ RTSP 서버 시작: " << rtsp_server_->get_rtsp_url() << std::endl;
    }
    
    // HTTP 서버 시작
    if (http_server_ && http_enabled_) {
        http_server_->start();
        std::cout << "  ✓ HTTP 서버 시작: " << http_server_->get_http_url() << std::endl;
    }
}

void StreamingManager::stop() {
    if (rtsp_server_) {
        rtsp_server_->stop();
    }
    if (http_server_) {
        http_server_->stop();
    }
}

std::string StreamingManager::getRTSPUrl() const {
    if (rtsp_server_) {
        return rtsp_server_->get_rtsp_url();
    }
    return "";
}

std::string StreamingManager::getHTTPUrl() const {
    if (http_server_) {
        return http_server_->get_http_url();
    }
    return "";
}

bool StreamingManager::isHTTPServerEnabled() const {
    return http_enabled_;
}

