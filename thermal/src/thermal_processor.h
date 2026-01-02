#ifndef THERMAL_PROCESSOR_H
#define THERMAL_PROCESSOR_H

#include <opencv2/opencv.hpp>
#include "thermal_data.h"
#include "config.h"

class ThermalProcessor {
public:
    ThermalProcessor();
    ~ThermalProcessor();
    
    bool extract_thermal_data(const cv::Mat& thermal_frame, ThermalData& data);
    
private:
    cv::Point2i find_hotspot(const cv::Mat& thermal_frame);
    cv::Vec3b determine_color(double distance);
    
    // Lepton 3.5 온도 변환 함수
    // pixel_value: 0~255 픽셀 값
    // 반환: 섭씨 온도
    double pixel_to_temperature(double pixel_value) const;
};

#endif // THERMAL_PROCESSOR_H
