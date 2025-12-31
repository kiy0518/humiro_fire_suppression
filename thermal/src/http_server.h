#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <microhttpd.h>
#include "thread_safe_queue.h"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <atomic>
#include <thread>

class HTTPServer {
public:
    HTTPServer();
    ~HTTPServer();
    
    bool initialize(ThreadSafeQueue<cv::Mat>* frame_queue);
    void start();
    void stop();
    
    std::string get_http_url() const;
    
private:
    struct MHD_Daemon* daemon_;
    ThreadSafeQueue<cv::Mat>* frame_queue_;
    std::atomic<bool> is_running_;
    
    static MHD_Result handle_request(void* cls, struct MHD_Connection* connection,
                            const char* url, const char* method,
                            const char* version, const char* upload_data,
                            size_t* upload_data_size, void** con_cls);
    
    static MHD_Result handle_video_feed(void* cls, struct MHD_Connection* connection);
    static MHD_Result handle_index(void* cls, struct MHD_Connection* connection);
    
    static void request_completed(void* cls, struct MHD_Connection* connection,
                                 void** con_cls, enum MHD_RequestTerminationCode toe);
};

#endif // HTTP_SERVER_H
