#ifndef STREAMING_MANAGER_H
#define STREAMING_MANAGER_H

#include "rtsp_server.h"
#include "http_server.h"
#include "../../thermal/src/thread_safe_queue.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <atomic>

/**
 * 스트리밍 관리자 클래스
 * RTSP, HTTP 등 모든 스트리밍 서버를 통합 관리
 */
class StreamingManager {
public:
    StreamingManager();
    ~StreamingManager();
    
    /**
     * 초기화
     * @param rtsp_queue RTSP용 프레임 큐
     * @param http_queue HTTP용 프레임 큐 (nullptr이면 HTTP 서버 비활성화)
     */
    bool initialize(ThreadSafeQueue<cv::Mat>* rtsp_queue, 
                    ThreadSafeQueue<cv::Mat>* http_queue = nullptr);
    
    /**
     * 시작
     */
    void start();
    
    /**
     * 중지
     */
    void stop();
    
    /**
     * RTSP URL 가져오기
     */
    std::string getRTSPUrl() const;
    
    /**
     * HTTP URL 가져오기
     */
    std::string getHTTPUrl() const;
    
    /**
     * HTTP 서버 활성화 여부
     */
    bool isHTTPServerEnabled() const;
    
private:
    RTSPServer* rtsp_server_;
    HTTPServer* http_server_;
    ThreadSafeQueue<cv::Mat>* rtsp_queue_;
    ThreadSafeQueue<cv::Mat>* http_queue_;
    std::atomic<bool> http_enabled_;
    bool initialized_;
};

#endif // STREAMING_MANAGER_H

