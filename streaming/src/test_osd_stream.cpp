#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include "rtsp_server.h"
#include "../../thermal/src/thread_safe_queue.h"
#include "../../osd/src/status/status_overlay.h"
#include "../../osd/src/lidar/distance_overlay.h"

// 더미 프레임 생성
cv::Mat createDummyFrame(int width, int height, int frameCount) {
    cv::Mat frame = cv::Mat::zeros(height, width, CV_8UC3);
    
    // 그리드
    cv::Scalar gridColor(30, 30, 30);
    for (int y = 0; y < height; y += 50) {
        cv::line(frame, cv::Point(0, y), cv::Point(width, y), gridColor, 1);
    }
    for (int x = 0; x < width; x += 50) {
        cv::line(frame, cv::Point(x, 0), cv::Point(x, height), gridColor, 1);
    }
    
    // 프레임 카운터
    std::string text = "OSD Test - Frame: " + std::to_string(frameCount);
    cv::putText(frame, text, cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 100, 100), 2);
    
    // 중앙 십자선
    cv::line(frame, cv::Point(width/2 - 20, height/2), 
             cv::Point(width/2 + 20, height/2), cv::Scalar(50, 50, 50), 2);
    cv::line(frame, cv::Point(width/2, height/2 - 20), 
             cv::Point(width/2, height/2 + 20), cv::Scalar(50, 50, 50), 2);
    
    return frame;
}

int main() {
    std::cout << "=================================" << std::endl;
    std::cout << "  OSD-Only Streaming Test" << std::endl;
    std::cout << "=================================" << std::endl;
    
    const int width = 1280;
    const int height = 720;
    const int fps = 30;
    
    // 프레임 큐
    ThreadSafeQueue<cv::Mat> frame_queue;
    
    // RTSP 서버
    RTSPServer rtsp_server;
    if (!rtsp_server.initialize(&frame_queue)) {
        std::cerr << "Failed to initialize RTSP server" << std::endl;
        return 1;
    }
    
    std::cout << "\n[RTSP URL]" << std::endl;
    std::cout << rtsp_server.get_rtsp_url() << std::endl;
    std::cout << "\nConnect from QGC with this URL\n" << std::endl;
    
    rtsp_server.start();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // OSD 오버레이
    StatusOverlay status_overlay;
    DistanceOverlay distance_overlay;
    
    // 테스트 데이터
    std::cout << "Setting up test data..." << std::endl;
    
    status_overlay.updatePx4State("OFFBOARD", true);
    status_overlay.updateOffboardStatus(StatusOverlay::DroneStatus::NAVIGATING);
    status_overlay.setBattery(87);
    status_overlay.setAmmunition(5, 6);
    status_overlay.setFormation(1, 3);
    
    // LiDAR 테스트 데이터
    std::vector<LidarPoint> test_points;
    for (int angle = 0; angle < 360; angle += 5) {
        float distance = 5.0f + 3.0f * std::sin(angle * M_PI / 180.0);
        test_points.push_back({(float)angle, distance, 255});
    }
    distance_overlay.setLidarData(test_points);
    
    std::cout << "Starting frame generation..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    int frameCount = 0;
    auto startTime = std::chrono::steady_clock::now();
    
    while (true) {
        // 더미 프레임 생성
        cv::Mat frame = createDummyFrame(width, height, frameCount);
        
        // LiDAR 오버레이
        distance_overlay.drawOverlay(frame);
        
        // 상태 오버레이
        status_overlay.draw(frame);
        
        // 타임스탬프
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
        std::string timeText = "Running: " + std::to_string(elapsed) + "s";
        cv::putText(frame, timeText, cv::Point(width - 200, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);
        
        // 프레임 큐에 추가
        frame_queue.push(frame);
        
        // FPS 제어
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
        
        frameCount++;
        
        // 10초마다 상태 업데이트
        if (frameCount % (fps * 10) == 0) {
            int battery = 87 - (frameCount / (fps * 10)) * 5;
            if (battery < 0) battery = 0;
            status_overlay.setBattery(battery);
            
            // LiDAR 데이터 변화
            test_points.clear();
            float offset = frameCount * 0.01f;
            for (int angle = 0; angle < 360; angle += 5) {
                float distance = 5.0f + 3.0f * std::sin((angle + offset) * M_PI / 180.0);
                test_points.push_back({(float)angle, distance, 255});
            }
            distance_overlay.setLidarData(test_points);
            
            std::cout << "Frame: " << frameCount 
                      << ", Battery: " << battery << "%" << std::endl;
        }
    }
    
    rtsp_server.stop();
    return 0;
}
