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
        
        // 화점 검출 (녹색 채널 또는 그레이스케일)
        cv::Mat thermal_gray;
        if (frame_cropped.channels() >= 3) {
            // 컬러 이미지인 경우 녹색 채널 사용
            std::vector<cv::Mat> channels;
            cv::split(frame_cropped, channels);
            thermal_gray = channels[1];  // 녹색 채널
        } else if (frame_cropped.channels() == 1) {
            // 그레이스케일 이미지인 경우 직접 사용
            thermal_gray = frame_cropped;
        } else {
            return false;
        }
        
        // 픽셀 값 범위 확인
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(thermal_gray, &min_val, &max_val, &min_loc, &max_loc);
        
        // 픽셀 값을 0~255 범위로 클리핑
        // PureThermal 드라이버는 보통 0~255 범위의 픽셀 값을 제공
        // 프레임 내 정규화를 하지 않고, 픽셀 값을 직접 사용
        // (프레임 내 정규화는 실제 온도와 무관하게 상대적 값만 보여주게 됨)
        max_val = std::max(0.0, std::min(255.0, max_val));
        min_val = std::max(0.0, std::min(255.0, min_val));
        
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
        
        // 온도 변환 (픽셀 값 → 섭씨 온도)
        double max_temp = pixel_to_temperature(max_val);
        double min_temp = pixel_to_temperature(min_val);
        
        // 데이터 업데이트 (스레드 안전)
        {
            std::lock_guard<std::mutex> lock(data.mutex);
            data.hotspot_x = rgb_max_x;
            data.hotspot_y = rgb_max_y;
            data.center_x = rgb_center_x;
            data.center_y = rgb_center_y;
            data.max_val = max_val;
            data.min_val = min_val;
            data.max_temp_celsius = max_temp;
            data.min_temp_celsius = min_temp;
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

// Lepton 3.5 온도 변환 함수
// Lepton 3.5는 radiometric 센서로, 픽셀 값을 온도로 변환 가능
// PureThermal 드라이버를 통해 받은 0~255 값을 온도로 변환
// 
// Lepton 3.5 Gain Mode별 온도 범위:
// - High Gain Mode: -10°C ~ 140°C (높은 정밀도, 낮은 온도 범위)
// - Low Gain Mode: -10°C ~ 400°C (실온 기준) 또는 -10°C ~ 450°C (일반적)
//                   (낮은 정밀도, 높은 온도 범위)
// 
// 주의: PureThermal 드라이버는 픽셀 값을 온도로 변환하여 제공할 수도 있음
// 실제 온도 변환 공식은 드라이버 구현에 따라 다를 수 있음
// 
// 픽셀 값 0 = 최소 온도, 255 = 최대 온도 (일반적인 가정)
double ThermalProcessor::pixel_to_temperature(double pixel_value) const {
    // 픽셀 값을 0~255 범위로 클리핑
    pixel_value = std::max(0.0, std::min(255.0, pixel_value));
    
    // 픽셀 값을 0~1 범위로 정규화
    double normalized = pixel_value / 255.0;
    
    // Lepton 3.5 온도 범위 (Gain Mode에 따라 다름)
    const double MIN_TEMP = -10.0;  // 최소 온도 (섭씨) - 모든 모드 동일
    
    double MAX_TEMP;
    
    #if LEPTON_GAIN_MODE == 1
        // Low Gain Mode: -10°C ~ 400°C (실온 기준) 또는 -10°C ~ 450°C (일반적)
        // 일반적으로 400°C를 사용하지만, 더 높은 온도 측정이 필요하면 450°C 사용
        MAX_TEMP = 400.0;  // 실온 기준 최대 온도
        // MAX_TEMP = 450.0;  // 일반적 최대 온도 (더 높은 온도 측정 시)
    #else
        // High Gain Mode: -10°C ~ 140°C (기본값)
        MAX_TEMP = 140.0;
    #endif
    
    // 선형 변환: normalized * (MAX - MIN) + MIN
    double temperature = normalized * (MAX_TEMP - MIN_TEMP) + MIN_TEMP;
    
    // 온도 보정 적용 (config.h의 TEMP_OFFSET, TEMP_SCALE 사용)
    temperature = temperature * TEMP_SCALE + TEMP_OFFSET;
    
    return temperature;
}
