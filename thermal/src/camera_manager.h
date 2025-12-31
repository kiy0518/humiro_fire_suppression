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
    bool read_rgb_frame(cv::Mat& frame);
    bool read_thermal_frame(cv::Mat& frame);
    
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
};

#endif // CAMERA_MANAGER_H
