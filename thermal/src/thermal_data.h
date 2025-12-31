#ifndef THERMAL_DATA_H
#define THERMAL_DATA_H

#include <opencv2/opencv.hpp>
#include <mutex>
#include <ctime>

struct ThermalData {
    int hotspot_x = -1;
    int hotspot_y = -1;
    int center_x = -1;
    int center_y = -1;
    double max_val = 0.0;
    double min_val = 0.0;
    double distance = 0.0;
    cv::Vec3b color = cv::Vec3b(255, 255, 255);
    cv::Vec3b max_color = cv::Vec3b(0, 0, 255);
    int rel_x = 0;
    int rel_y = 0;
    double timestamp = 0.0;
    bool valid = false;
    cv::Mat frame;  // 열화상 프레임 (오버레이용)
    
    // 스레드 안전 접근을 위한 락
    mutable std::mutex mutex;
    
    // 복사 생성자 (mutex는 새로 생성)
    ThermalData(const ThermalData& other) 
        : hotspot_x(other.hotspot_x),
          hotspot_y(other.hotspot_y),
          center_x(other.center_x),
          center_y(other.center_y),
          max_val(other.max_val),
          min_val(other.min_val),
          distance(other.distance),
          color(other.color),
          max_color(other.max_color),
          rel_x(other.rel_x),
          rel_y(other.rel_y),
          timestamp(other.timestamp),
          valid(other.valid) {
        if (!other.frame.empty()) {
            frame = other.frame.clone();
        }
        // mutex는 기본 생성자로 새로 생성됨
    }
    
    // 기본 생성자
    ThermalData() = default;
    
    // 데이터 복사 (스레드 안전)
    ThermalData copy() const {
        std::lock_guard<std::mutex> lock(mutex);
        return ThermalData(*this);
    }
};

#endif // THERMAL_DATA_H
