#ifndef RTSP_SERVER_H
#define RTSP_SERVER_H

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>
#include "../../thermal/src/thread_safe_queue.h"
#include "../../thermal/src/config.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <string>

// RTSP 서버 데이터 구조체
struct RTSPData {
    ThreadSafeQueue<cv::Mat>* frame_queue;
    GstElement* appsrc;
    std::thread* push_thread;
    std::atomic<bool> is_running;
    
    RTSPData() : frame_queue(nullptr), appsrc(nullptr), push_thread(nullptr), is_running(false) {}
};

class RTSPServer {
public:
    RTSPServer();
    ~RTSPServer();
    
    bool initialize(ThreadSafeQueue<cv::Mat>* frame_queue);
    void start();
    void stop();
    
    std::string get_rtsp_url() const;
    
private:
    GstRTSPServer* server_;
    GstRTSPMediaFactory* factory_;
    GMainLoop* loop_;
    RTSPData* rtsp_data_;
    std::thread server_thread_;
    std::atomic<bool> is_running_;
    
    static GstElement* create_media_element(GstRTSPMediaFactory* factory, const GstRTSPUrl* url);
    // 주의: 최신 GStreamer에서는 GstRTSPMediaFactory에 시그널이 없으므로 사용하지 않음
    // static void media_configured(GstRTSPMediaFactory* factory, GstRTSPMedia* media);
    // static void media_unprepared(GstRTSPMedia* media);
    static void* server_thread_func(void* arg);
    std::string get_launch_string();
    bool check_encoder(const std::string& encoder_name);
    static void push_frames_thread(RTSPData* data);
};

#endif // RTSP_SERVER_H
