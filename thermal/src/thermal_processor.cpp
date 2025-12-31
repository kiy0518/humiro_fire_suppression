#include "thermal_processor.h"
#include <cmath>
#include <algorithm>

ThermalProcessor::ThermalProcessor() {
}

ThermalProcessor::~ThermalProcessor() {
}

bool ThermalProcessor::extract_thermal_data(const cv::Mat& thermal_frame, ThermalData& data) {
    if (thermal_frame.empty() || thermal_frame.rows < 2 || thermal_frame.cols < 2) {
        return false;
    }
    
    try {
        // 열화상 데이터 리사이즈
        cv::Mat frame_small;
        cv::resize(thermal_frame, frame_small, cv::Size(THERMAL_WIDTH, THERMAL_HEIGHT), 0, 0, cv::INTER_NEAREST);
        
        // 하단 픽셀 제거
        cv::Mat frame_cropped = frame_small(cv::Rect(0, 0, THERMAL_WIDTH, THERMAL_CROPPED_HEIGHT));
        
        // 화점 검출 (녹색 채널)
        if (frame_cropped.channels() < 2) {
            return false;
        }
        
        std::vector<cv::Mat> channels;
        cv::split(frame_cropped, channels);
        cv::Mat green = channels[1];
        
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(green, &min_val, &max_val, &min_loc, &max_loc);
        
        // RGB 좌표계로 변환
        int rgb_center_x = RGB_CROP_X + CENTER_X;
        int rgb_center_y = RGB_CROP_Y + CENTER_Y;
        int rgb_max_x = RGB_CROP_X + max_loc.x;
        int rgb_max_y = RGB_CROP_Y + max_loc.y;
        
        // 경계 체크
        rgb_center_x = std::max(0, std::min(OUTPUT_WIDTH - 1, rgb_center_x));
        rgb_center_y = std::max(0, std::min(OUTPUT_HEIGHT - 1, rgb_center_y));
        rgb_max_x = std::max(0, std::min(OUTPUT_WIDTH - 1, rgb_max_x));
        rgb_max_y = std::max(0, std::min(OUTPUT_HEIGHT - 1, rgb_max_y));
        
        // 거리 계산
        int dx = rgb_center_x - rgb_max_x;
        int dy = rgb_center_y - rgb_max_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // 색상 결정
        cv::Vec3b color = determine_color(distance);
        cv::Vec3b max_color;
        if (distance < RADIUS_INNER) {
            max_color = cv::Vec3b(0, 255, 0);  // 녹색
        } else if (distance < RADIUS_OUTER) {
            max_color = cv::Vec3b(0, 255, 255);  // 노란색
        } else {
            max_color = cv::Vec3b(0, 0, 255);  // 빨간색
        }
        
        int rel_x = max_loc.x - CENTER_X;
        int rel_y = max_loc.y - CENTER_Y;
        
        // 열화상 프레임 준비 (오버레이용)
        cv::Mat thermal_frame_for_overlay;
        if (OVERLAY_THERMAL) {
            cv::resize(thermal_frame, thermal_frame_for_overlay, 
                      cv::Size(THERMAL_WIDTH, THERMAL_CROPPED_HEIGHT), 0, 0, cv::INTER_LINEAR);
        }
        
        // 데이터 업데이트 (스레드 안전)
        {
            std::lock_guard<std::mutex> lock(data.mutex);
            data.hotspot_x = rgb_max_x;
            data.hotspot_y = rgb_max_y;
            data.center_x = rgb_center_x;
            data.center_y = rgb_center_y;
            data.max_val = max_val;
            data.min_val = min_val;
            data.distance = distance;
            data.color = color;
            data.max_color = max_color;
            data.rel_x = rel_x;
            data.rel_y = rel_y;
            data.timestamp = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
            data.valid = true;
            if (!thermal_frame_for_overlay.empty()) {
                data.frame = thermal_frame_for_overlay.clone();
            }
        }
        
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 열화상 데이터 추출 오류: " << e.what() << std::endl;
        return false;
    }
}

cv::Vec3b ThermalProcessor::determine_color(double distance) {
    if (distance < RADIUS_INNER) {
        return cv::Vec3b(0, 255, 0);  // 녹색
    } else if (distance < RADIUS_OUTER) {
        return cv::Vec3b(0, 255, 255);  // 노란색
    } else {
        return cv::Vec3b(255, 255, 255);  // 흰색
    }
}
