#include "thermal_processor.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

ThermalProcessor::ThermalProcessor() {
}

ThermalProcessor::~ThermalProcessor() {
}

bool ThermalProcessor::extract_thermal_data(const cv::Mat& thermal_frame, ThermalData& data) {
    if (thermal_frame.empty() || thermal_frame.rows < 2 || thermal_frame.cols < 2) {
        return false;
    }

    try {
        const auto& tc = ThermalConfig::get();

        // Y16 (16-bit) vs BGR (8-bit) 분기
        bool is_y16 = (thermal_frame.type() == CV_16UC1);

        // 최초 1회만 프레임 타입 로깅
        static bool logged_type = false;
        if (!logged_type) {
            std::cout << "[ThermalProcessor] frame type=" << thermal_frame.type()
                      << " (" << (is_y16 ? "Y16/16UC1" : "BGR/8UC3") << ")"
                      << " size=" << thermal_frame.cols << "x" << thermal_frame.rows
                      << " channels=" << thermal_frame.channels() << std::endl;
            logged_type = true;
        }

        // 열화상 분석용 그레이 프레임
        cv::Mat thermal_gray;
        // 오버레이용 컬러 프레임
        cv::Mat thermal_color;

        if (is_y16) {
            // === Y16 모드: raw radiometric (센티켈빈) ===
            cv::Mat frame_small_16;
            cv::resize(thermal_frame, frame_small_16,
                       cv::Size(tc.thermal_width, tc.thermal_height), 0, 0, cv::INTER_NEAREST);
            cv::Mat frame_cropped_16 = frame_small_16(
                cv::Rect(0, 0, tc.thermal_width, tc.thermal_cropped_height));

            // 공간 필터 (16-bit에서 직접 적용)
            cv::GaussianBlur(frame_cropped_16, frame_cropped_16, cv::Size(5, 5), 0);

            // 최고점 검출 (16-bit 값 그대로)
            double min_val_raw, max_val_raw;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(frame_cropped_16, &min_val_raw, &max_val_raw, &min_loc, &max_loc);

            // 센티켈빈 → 섭씨 변환
            double max_temp = pixel_to_temperature(max_val_raw);
            double min_temp = pixel_to_temperature(min_val_raw);

            // 주기적 온도 로깅 (5초마다)
            static auto last_log = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
                std::cout << "[Y16] max=" << std::fixed << std::setprecision(1)
                          << max_temp << "°C min=" << min_temp << "°C"
                          << " raw_max=" << (int)max_val_raw
                          << " at(" << max_loc.x << "," << max_loc.y << ")" << std::endl;
                last_log = now;
            }

            // 오버레이용 컬러맵 생성 (프레임 min-max 정규화)
            // 실제 프레임의 온도 범위에 맞춰 전체 컬러맵 활용
            double viz_min_raw, viz_max_raw;
            cv::minMaxLoc(frame_cropped_16, &viz_min_raw, &viz_max_raw);
            // 최소 범위 보장 (너무 좁으면 노이즈가 증폭됨)
            if (viz_max_raw - viz_min_raw < 200.0) {
                double mid = (viz_max_raw + viz_min_raw) / 2.0;
                viz_min_raw = mid - 100.0;
                viz_max_raw = mid + 100.0;
            }
            const double VIZ_RANGE = viz_max_raw - viz_min_raw;

            cv::Mat normalized_8bit;
            frame_cropped_16.convertTo(normalized_8bit, CV_8UC1,
                                       255.0 / VIZ_RANGE,
                                       -viz_min_raw * 255.0 / VIZ_RANGE);
            cv::applyColorMap(normalized_8bit, thermal_color, cv::COLORMAP_HOT);

            // 공통 처리로 전달
            return process_hotspot(tc, max_loc, min_loc, max_val_raw, min_val_raw,
                                   max_temp, min_temp, thermal_color, data);
        } else {
            // === BGR/8-bit 모드 (폴백: AGC 적용된 시각화 출력) ===
            cv::Mat frame_small;
            cv::resize(thermal_frame, frame_small,
                       cv::Size(tc.thermal_width, tc.thermal_height), 0, 0, cv::INTER_NEAREST);
            cv::Mat frame_cropped = frame_small(
                cv::Rect(0, 0, tc.thermal_width, tc.thermal_cropped_height));

            if (frame_cropped.channels() >= 3) {
                std::vector<cv::Mat> channels;
                cv::split(frame_cropped, channels);
                thermal_gray = channels[1];  // 녹색 채널
            } else if (frame_cropped.channels() == 1) {
                thermal_gray = frame_cropped;
            } else {
                return false;
            }

            cv::GaussianBlur(thermal_gray, thermal_gray, cv::Size(5, 5), 0);

            double min_val, max_val;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(thermal_gray, &min_val, &max_val, &min_loc, &max_loc);

            max_val = std::max(0.0, std::min(255.0, max_val));
            min_val = std::max(0.0, std::min(255.0, min_val));

            // 8-bit AGC 모드에서는 정확한 온도 변환 불가 (상대값만 표시)
            double max_temp = pixel_to_temperature(max_val);
            double min_temp = pixel_to_temperature(min_val);

            // 오버레이용: 원본 컬러 프레임 사용
            if (frame_cropped.channels() >= 3) {
                thermal_color = frame_cropped;
            } else {
                cv::applyColorMap(frame_cropped, thermal_color, cv::COLORMAP_HOT);
            }

            return process_hotspot(tc, max_loc, min_loc, max_val, min_val,
                                   max_temp, min_temp, thermal_color, data);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 열화상 데이터 추출 오류: " << e.what() << std::endl;
        return false;
    }
}

bool ThermalProcessor::process_hotspot(
    const ThermalConfig& tc,
    const cv::Point& max_loc, const cv::Point& min_loc,
    double max_val, double min_val,
    double max_temp, double min_temp,
    const cv::Mat& thermal_color,
    ThermalData& data)
{
    // RGB 좌표계로 변환
    int rgb_center_x = tc.rgb_crop_x + tc.center_x;
    int rgb_center_y = tc.rgb_crop_y + tc.center_y;
    int raw_max_x = tc.rgb_crop_x + max_loc.x;
    int raw_max_y = tc.rgb_crop_y + max_loc.y;

    // EMA 시간 필터: 최고점 좌표 떨림 방지
    if (ema_x_ < 0) {
        ema_x_ = static_cast<float>(raw_max_x);
        ema_y_ = static_cast<float>(raw_max_y);
    } else {
        ema_x_ = EMA_ALPHA * raw_max_x + (1.0f - EMA_ALPHA) * ema_x_;
        ema_y_ = EMA_ALPHA * raw_max_y + (1.0f - EMA_ALPHA) * ema_y_;
    }
    int rgb_max_x = static_cast<int>(std::round(ema_x_));
    int rgb_max_y = static_cast<int>(std::round(ema_y_));

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
        max_color = cv::Vec3b(0, 255, 0);
    } else if (distance < RADIUS_OUTER) {
        max_color = cv::Vec3b(0, 255, 255);
    } else {
        max_color = cv::Vec3b(0, 0, 255);
    }

    int rel_x = max_loc.x - tc.center_x;
    int rel_y = max_loc.y - tc.center_y;

    // 데이터 업데이트
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
        data.fire_detected = (max_temp >= FIRE_DETECT_MIN_TEMP);
        if (!thermal_color.empty()) {
            data.frame = thermal_color.clone();
        }
    }

    return true;
}

cv::Vec3b ThermalProcessor::determine_color(double distance) {
    if (distance < RADIUS_INNER) {
        return cv::Vec3b(0, 255, 0);
    } else if (distance < RADIUS_OUTER) {
        return cv::Vec3b(0, 255, 255);
    } else {
        return cv::Vec3b(255, 255, 255);
    }
}

// Y16 센티켈빈 → 섭씨 변환
// Lepton 3.5 Y16 raw radiometric: 각 픽셀 = 0.01K (센티켈빈)
// 8-bit 폴백: 0~255를 -10~400°C (Low Gain) 범위로 선형 매핑
double ThermalProcessor::pixel_to_temperature(double pixel_value) const {
    double temperature;

    if (pixel_value > 1000) {
        // Y16 radiometric: 센티켈빈 → 섭씨
        temperature = pixel_value * 0.01 - 273.15;
    } else {
        // 8-bit 폴백 (AGC): 상대값이므로 근사치만 가능
        double normalized = std::max(0.0, std::min(255.0, pixel_value)) / 255.0;
        const double MIN_TEMP = -10.0;
        const double MAX_TEMP = 400.0;  // Low Gain
        temperature = normalized * (MAX_TEMP - MIN_TEMP) + MIN_TEMP;
    }

    // 온도 보정 적용
    temperature = temperature * TEMP_SCALE + TEMP_OFFSET;

    return temperature;
}
