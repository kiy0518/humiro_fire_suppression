#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "config.h"

class CameraManager {
public:
    CameraManager();
    ~CameraManager();
    
    bool initialize();
    
    // 개별 카메라 초기화 (비동기식 스트리밍용)
    bool initialize_rgb_camera();
    bool initialize_thermal_camera();
    
    // 준비 상태 확인
    bool is_rgb_ready() const { return cap_rgb_.isOpened(); }
    bool is_thermal_ready() const { return cap_thermal_.isOpened(); }
    
    bool read_rgb_frame(cv::Mat& frame);
    bool read_thermal_frame(cv::Mat& frame);
    
    // 재연결 메서드
    bool reconnect_rgb_camera();
    bool reconnect_thermal_camera();
    
    int get_thermal_camera_id() const { return thermal_camera_id_; }
    int get_rgb_camera_id() const { return rgb_camera_id_; }
    
private:
    cv::VideoCapture cap_thermal_;
    cv::VideoCapture cap_rgb_;
    int thermal_camera_id_;
    int rgb_camera_id_;
    
    bool find_thermal_camera();
    bool find_rgb_camera();
    bool test_camera(int camera_id, bool is_thermal);
    
    // 예외 처리 강화: 디바이스 존재 확인
    bool device_exists(const std::string& device_path);
    bool safe_open_camera(int camera_id, cv::VideoCapture& cap);
};

#endif // CAMERA_MANAGER_H
