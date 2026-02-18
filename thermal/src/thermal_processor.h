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
    cv::Vec3b determine_color(double distance);
    double pixel_to_temperature(double pixel_value) const;

    // Y16/BGR 공통 후처리 (EMA, 좌표 변환, 데이터 업데이트)
    bool process_hotspot(const ThermalConfig& tc,
                         const cv::Point& max_loc, const cv::Point& min_loc,
                         double max_val, double min_val,
                         double max_temp, double min_temp,
                         const cv::Mat& thermal_color,
                         ThermalData& data);

    // EMA 시간 필터 상태 (최고점 좌표 떨림 방지)
    float ema_x_ = -1.0f;
    float ema_y_ = -1.0f;
    static constexpr float EMA_ALPHA = 0.4f;  // 0.3=부드러움, 0.7=반응빠름
};

#endif // THERMAL_PROCESSOR_H
