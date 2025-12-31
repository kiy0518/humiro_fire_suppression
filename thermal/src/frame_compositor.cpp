#include "frame_compositor.h"
#include "../../lidar/src/lidar_interface.h"  // LidarPoint 정의 포함
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <string>
#include <mutex>

FrameCompositor::FrameCompositor() : logo_loaded_(false), lidar_angle_offset_(0.0f) {
    // 로고 이미지 로드 (실행 파일 기준 상대 경로 또는 절대 경로)
    std::string logo_path = LOGO_PATH;
    // 상대 경로인 경우 실행 파일 위치 기준으로 변환
    if (logo_path[0] != '/') {
        char exe_path[1024];
        ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
        if (len != -1) {
            exe_path[len] = '\0';
            std::string exe_dir = std::string(exe_path);
            size_t last_slash = exe_dir.find_last_of('/');
            if (last_slash != std::string::npos) {
                logo_path = exe_dir.substr(0, last_slash) + "/" + logo_path;
            }
        }
    }
    logo_image_ = cv::imread(logo_path, cv::IMREAD_UNCHANGED);
    if (!logo_image_.empty()) {
        // 로고 크기 조정
        if (logo_image_.cols > LOGO_WIDTH) {
            double scale = static_cast<double>(LOGO_WIDTH) / logo_image_.cols;
            int new_height = static_cast<int>(logo_image_.rows * scale);
            cv::resize(logo_image_, logo_image_, cv::Size(LOGO_WIDTH, new_height), 0, 0, cv::INTER_LINEAR);
        }
        logo_loaded_ = true;
        std::cout << "  ✓ 로고 이미지 로드: " << LOGO_PATH << " (" 
                  << logo_image_.cols << "x" << logo_image_.rows << ")" << std::endl;
    } else {
        std::cerr << "  ⚠ 로고 이미지 로드 실패: " << LOGO_PATH << std::endl;
    }
}

FrameCompositor::~FrameCompositor() {
}

cv::Mat FrameCompositor::composite_frames(const cv::Mat& rgb_frame, const ThermalData& thermal_data) {
    if (rgb_frame.empty()) {
        return cv::Mat();
    }
    
    // RGB 영상이 메인 - 복사
    cv::Mat output;
    if (rgb_frame.rows != OUTPUT_HEIGHT || rgb_frame.cols != OUTPUT_WIDTH) {
        cv::resize(rgb_frame, output, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
    } else {
        output = rgb_frame.clone();
    }
    
    // 열화상 데이터 가져오기 (스레드 안전 복사)
    ThermalData data = thermal_data.copy();
    
    // 열화상 영상 오버레이 (옵션)
    if (OVERLAY_THERMAL && data.valid && !data.frame.empty()) {
        overlay_thermal(output, data.frame);
    }
    
    // 열화상 데이터가 유효하면 마커 그리기
    if (data.valid && data.center_x >= 0 && data.center_y >= 0) {
        draw_marker(output, data);
        draw_text_info(output, data);
    }
    
    // 로고 오버레이 (오른쪽 상단)
    overlay_logo(output);
    
    // 라이다 레이더 오버레이 (센터 중심 원형)
    overlay_lidar_radar(output);
    
    return output;
}

void FrameCompositor::draw_marker(cv::Mat& frame, const ThermalData& data) {
    int center_x = data.center_x;
    int center_y = data.center_y;
    int max_x = data.hotspot_x;
    int max_y = data.hotspot_y;
    
    cv::Vec3b color = data.color;
    cv::Vec3b max_color = data.max_color;
    
    // 타겟팅 마커 그리기
    cv::circle(frame, cv::Point(center_x, center_y), 10, cv::Scalar(color[0], color[1], color[2]), 1);
    cv::circle(frame, cv::Point(center_x, center_y), RADIUS_OUTER, cv::Scalar(color[0], color[1], color[2]), 1);
    cv::circle(frame, cv::Point(center_x, center_y), RADIUS_INNER, cv::Scalar(color[0], color[1], color[2]), 1);
    
    int extend_radius = RADIUS_OUTER + 20;
    cv::line(frame, cv::Point(center_x + 10, center_y), 
             cv::Point(center_x + extend_radius, center_y), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    cv::line(frame, cv::Point(center_x - extend_radius, center_y), 
             cv::Point(center_x - 10, center_y), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    cv::line(frame, cv::Point(center_x, center_y + 10), 
             cv::Point(center_x, center_y + extend_radius), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    cv::line(frame, cv::Point(center_x, center_y - extend_radius), 
             cv::Point(center_x, center_y - 10), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    
    // 화점 마커
    cv::circle(frame, cv::Point(max_x, max_y), 20, 
               cv::Scalar(max_color[0], max_color[1], max_color[2]), 2);
}

void FrameCompositor::draw_text_info(cv::Mat& frame, const ThermalData& data) {
    int text_y = 20;
    int text_spacing = 25;
    
    // 전체 정보 영역의 배경 투명도 적용
    int info_start_y = text_y - 15;
    int info_end_y = text_y + text_spacing * 4 + 10;  // 5개 라인 (Offset, Max, Min, Dist, Status)
    int info_width = 200;
    int info_height = info_end_y - info_start_y;
    
    // 정보 배경 영역 ROI
    cv::Rect info_bg_rect(5, info_start_y, info_width, info_height);
    if (info_bg_rect.x >= 0 && info_bg_rect.y >= 0 && 
        info_bg_rect.x + info_bg_rect.width <= frame.cols &&
        info_bg_rect.y + info_bg_rect.height <= frame.rows) {
        
        cv::Mat roi = frame(info_bg_rect);
        cv::Mat roi_float;
        roi.convertTo(roi_float, CV_32F);
        
        // 검은색 배경과 블렌딩 (투명도 0.7)
        cv::Mat black_bg = cv::Mat::zeros(roi_float.size(), CV_32FC3);
        cv::Mat blended = roi_float * (1.0f - INFO_BACKGROUND_ALPHA) + 
                         black_bg * INFO_BACKGROUND_ALPHA;
        blended.convertTo(roi, CV_8U);
    }
    
    // 좌표 정보
    std::string offset_text = "Offset: (" + std::to_string(data.rel_x) + ", " + 
                              std::to_string(data.rel_y) + ")";
    cv::putText(frame, offset_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 온도값 정보
    text_y += text_spacing;
    std::string max_text = "Max: " + std::to_string(data.max_val);
    cv::putText(frame, max_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 최소값 정보
    text_y += text_spacing;
    std::string min_text = "Min: " + std::to_string(data.min_val);
    cv::putText(frame, min_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 거리 정보
    text_y += text_spacing;
    std::string dist_text = "Dist: " + std::to_string(data.distance) + "px";
    cv::putText(frame, dist_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 상태 정보
    text_y += text_spacing;
    std::string status;
    cv::Scalar status_color;
    if (data.distance < RADIUS_INNER) {
        status = "INNER";
        status_color = cv::Scalar(0, 255, 0);
    } else if (data.distance < RADIUS_OUTER) {
        status = "OUTER";
        status_color = cv::Scalar(0, 255, 255);
    } else {
        status = "OUT";
        status_color = cv::Scalar(0, 0, 255);
    }
    cv::putText(frame, "Status: " + status, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1);
}

void FrameCompositor::overlay_thermal(cv::Mat& rgb_frame, const cv::Mat& thermal_frame) {
    if (thermal_frame.empty()) {
        return;
    }
    
    try {
        int overlay_x = RGB_CROP_X;
        int overlay_y = RGB_CROP_Y;
        
        int thermal_h = thermal_frame.rows;
        int thermal_w = thermal_frame.cols;
        
        int overlay_w = std::min({THERMAL_WIDTH, OUTPUT_WIDTH - overlay_x, thermal_w});
        int overlay_h = std::min({THERMAL_CROPPED_HEIGHT, OUTPUT_HEIGHT - overlay_y, thermal_h});
        
        if (overlay_w <= 0 || overlay_h <= 0) {
            return;
        }
        
        cv::Mat thermal_resized;
        if (thermal_w != overlay_w || thermal_h != overlay_h) {
            cv::resize(thermal_frame, thermal_resized, cv::Size(overlay_w, overlay_h), 0, 0, cv::INTER_LINEAR);
        } else {
            thermal_resized = thermal_frame;
        }
        
        if (overlay_y + overlay_h <= rgb_frame.rows && overlay_x + overlay_w <= rgb_frame.cols) {
            cv::Rect roi_rect(overlay_x, overlay_y, overlay_w, overlay_h);
            cv::Mat roi = rgb_frame(roi_rect);
            
            if (thermal_resized.channels() == 3 && roi.channels() == 3) {
                // 크기 확인 및 필요시 리사이즈
                if (roi.size() != thermal_resized.size()) {
                    cv::resize(thermal_resized, thermal_resized, roi.size(), 0, 0, cv::INTER_LINEAR);
                }
                
                // 크기 재확인
                if (roi.rows == overlay_h && roi.cols == overlay_w &&
                    thermal_resized.rows == overlay_h && thermal_resized.cols == overlay_w) {
                    // 단순 투명도 블렌딩 (그라데이션/블러 제거)
                    // roi와 thermal_resized를 float로 변환
                    cv::Mat roi_float, thermal_float;
                    roi.convertTo(roi_float, CV_32F);  // (overlay_h, overlay_w, 3) -> CV_32FC3
                    thermal_resized.convertTo(thermal_float, CV_32F);  // (overlay_h, overlay_w, 3) -> CV_32FC3
                    
                    // 크기 및 채널 재확인
                    if (roi_float.rows == overlay_h && roi_float.cols == overlay_w &&
                        thermal_float.rows == overlay_h && thermal_float.cols == overlay_w &&
                        roi_float.channels() == 3 && thermal_float.channels() == 3) {
                        // 단순 알파 블렌딩: roi * (1 - alpha) + thermal * alpha
                        // 그라데이션 없이 일정한 투명도만 적용
                        float alpha = ALPHA_THERMAL_LAYER;
                        cv::Mat blended = roi_float * (1.0f - alpha) + thermal_float * alpha;
                        blended.convertTo(roi, CV_8U);
                    } else {
                        std::cerr << "  ⚠ 블렌딩 크기/채널 불일치:" << std::endl;
                        std::cerr << "     roi_float: " << roi_float.size() << " ch=" << roi_float.channels() << std::endl;
                        std::cerr << "     thermal_float: " << thermal_float.size() << " ch=" << thermal_float.channels() << std::endl;
                        std::cerr << "     overlay: " << overlay_w << "x" << overlay_h << std::endl;
                    }
                } else {
                    std::cerr << "  ⚠ 크기 불일치: ROI " << roi.size() 
                              << " vs Thermal " << thermal_resized.size()
                              << " vs overlay " << overlay_w << "x" << overlay_h << std::endl;
                }
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 열화상 오버레이 오류: " << e.what() << std::endl;
    }
}

cv::Mat FrameCompositor::create_gradient_mask(int width, int height) {
    cv::Mat mask = cv::Mat::ones(height, width, CV_32F);
    
    int grad_size = std::min({GRADIENT_BORDER_SIZE, width / 4, height / 4});
    
    if (grad_size > 0) {
        // Python 버전과 동일한 로직: 각 방향에서 그라데이션 적용
        // 상단 그라데이션
        for (int y = 0; y < grad_size; ++y) {
            float alpha = apply_curve(static_cast<float>(y) / grad_size);
            for (int x = 0; x < width; ++x) {
                mask.at<float>(y, x) *= alpha;
            }
        }
        
        // 하단 그라데이션
        for (int y = height - grad_size; y < height; ++y) {
            float alpha = apply_curve(static_cast<float>(height - y) / grad_size);
            for (int x = 0; x < width; ++x) {
                mask.at<float>(y, x) *= alpha;
            }
        }
        
        // 좌측 그라데이션
        for (int x = 0; x < grad_size; ++x) {
            float alpha = apply_curve(static_cast<float>(x) / grad_size);
            for (int y = 0; y < height; ++y) {
                // 이미 상단/하단 그라데이션이 적용된 경우 곱하기
                mask.at<float>(y, x) *= alpha;
            }
        }
        
        // 우측 그라데이션
        for (int x = width - grad_size; x < width; ++x) {
            float alpha = apply_curve(static_cast<float>(width - x) / grad_size);
            for (int y = 0; y < height; ++y) {
                // 이미 상단/하단 그라데이션이 적용된 경우 곱하기
                mask.at<float>(y, x) *= alpha;
            }
        }
    }
    
    return mask;
}

float FrameCompositor::apply_curve(float value) {
    if (GRADIENT_CURVE == 1.0f) {
        return value;  // 선형
    } else if (GRADIENT_CURVE > 1.0f) {
        return std::pow(value, GRADIENT_CURVE);  // 제곱
    } else {
        return std::pow(value, GRADIENT_CURVE);  // 제곱근
    }
}

void FrameCompositor::overlay_logo(cv::Mat& frame) {
    if (!logo_loaded_ || logo_image_.empty()) {
        return;
    }
    
    try {
        int logo_h = logo_image_.rows;
        int logo_w = logo_image_.cols;
        int pos_x = LOGO_POS_X;
        int pos_y = LOGO_POS_Y;
        
        // 경계 체크
        if (pos_x < 0 || pos_y < 0 || 
            pos_x + logo_w > frame.cols || 
            pos_y + logo_h > frame.rows) {
            return;
        }
        
        // ROI 추출
        cv::Rect roi_rect(pos_x, pos_y, logo_w, logo_h);
        cv::Mat roi = frame(roi_rect);
        
        // 알파 채널이 있는 경우 (RGBA)
        if (logo_image_.channels() == 4) {
            // 알파 채널 분리
            std::vector<cv::Mat> logo_channels;
            cv::split(logo_image_, logo_channels);
            cv::Mat logo_bgr = cv::Mat(logo_h, logo_w, CV_8UC3);
            cv::Mat alpha = logo_channels[3];
            
            // BGR 채널만 추출
            std::vector<cv::Mat> bgr_channels = {logo_channels[0], logo_channels[1], logo_channels[2]};
            cv::merge(bgr_channels, logo_bgr);
            
            // 알파 블렌딩
            alpha.convertTo(alpha, CV_32F, 1.0 / 255.0);
            std::vector<cv::Mat> alpha_channels = {alpha, alpha, alpha};
            cv::Mat alpha_3d;
            cv::merge(alpha_channels, alpha_3d);
            
            cv::Mat roi_float, logo_float;
            roi.convertTo(roi_float, CV_32F);
            logo_bgr.convertTo(logo_float, CV_32F);
            
            cv::Mat blended = roi_float.mul(1.0f - alpha_3d) + logo_float.mul(alpha_3d);
            blended.convertTo(roi, CV_8U);
        } else if (logo_image_.channels() == 3) {
            // 알파 채널이 없는 경우 (BGR)
            logo_image_.copyTo(roi);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 로고 오버레이 오류: " << e.what() << std::endl;
    }
}

void FrameCompositor::setLidarData(const std::vector<LidarPoint>& lidar_points) {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_points_ = lidar_points;
}

void FrameCompositor::setLidarOrientation(float offset_degrees) {
    std::lock_guard<std::mutex> lock(orientation_mutex_);
    lidar_angle_offset_ = offset_degrees;
    // 각도를 0~360도 범위로 정규화
    while (lidar_angle_offset_ >= 360.0f) lidar_angle_offset_ -= 360.0f;
    while (lidar_angle_offset_ < 0.0f) lidar_angle_offset_ += 360.0f;
}

void FrameCompositor::overlay_lidar_radar(cv::Mat& frame) {
    // 라이다 데이터 가져오기 (스레드 안전)
    std::vector<LidarPoint> points;
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        points = lidar_points_;
    }
    
    if (points.empty()) {
        return;
    }
    
    try {
        // 실제 프레임 크기에 맞춰 중심 좌표 계산
        int center_x = frame.cols / 2;
        int center_y = frame.rows / 2;
        int radius_pixels = LIDAR_RADIUS_PIXELS;
        float max_range = LIDAR_MAX_RANGE;
        
        // 반경이 프레임 크기를 초과하지 않도록 제한
        int max_radius = std::min({center_x, center_y, frame.cols - center_x, frame.rows - center_y});
        if (radius_pixels > max_radius) {
            radius_pixels = max_radius;
        }
        
        // 원형 그리드 그리기 (거리 링)
        // 3m, 6m, 9m, 12m 링
        std::vector<float> distance_rings = {3.0f, 6.0f, 9.0f, 12.0f};
        for (float dist : distance_rings) {
            int ring_radius = static_cast<int>((dist / max_range) * radius_pixels);
            cv::circle(frame, cv::Point(center_x, center_y), ring_radius, 
                      cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
        }
        
        // 중심 십자선
        cv::line(frame, cv::Point(center_x - 10, center_y), 
                cv::Point(center_x + 10, center_y), 
                cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
        cv::line(frame, cv::Point(center_x, center_y - 10), 
                cv::Point(center_x, center_y + 10), 
                cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
        
        // 라이다 오프셋 가져오기 (스레드 안전)
        float angle_offset = 0.0f;
        {
            std::lock_guard<std::mutex> lock(orientation_mutex_);
            angle_offset = lidar_angle_offset_;
        }
        
        const float PI = 3.14159265358979323846f;

        // 방향선 표시 (옵션)
        if (LIDAR_SHOW_DIRECTION_LINES) {
            // 디스플레이 모드에 따라 방향선 그리기
            std::string display_mode = LIDAR_DISPLAY_MODE;

            if (display_mode == "FRONT_3DIR") {
                // 3방향 모드: -90°, 0°, 90°
                std::vector<int> base_angles = {270, 0, 90};  // LD19 각도
                std::vector<std::string> labels = {"-90° (L)", "0° (F)", "90° (R)"};

                for (size_t i = 0; i < base_angles.size(); i++) {
                    int base_angle = base_angles[i];
                    float adjusted_angle = base_angle + angle_offset;
                    while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
                    while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;

                    // 180도 회전 보정
                    float screen_angle = (90.0f - adjusted_angle) * PI / 180.0f;
                    int end_x = center_x + static_cast<int>(radius_pixels * std::cos(screen_angle));
                    int end_y = center_y - static_cast<int>(radius_pixels * std::sin(screen_angle));

                    cv::Scalar line_color = (base_angle == 0) ? cv::Scalar(150, 150, 150) : cv::Scalar(100, 100, 100);
                    int line_thickness = (base_angle == 0) ? 2 : 1;
                    cv::line(frame, cv::Point(center_x, center_y),
                            cv::Point(end_x, end_y),
                            line_color, line_thickness, cv::LINE_AA);
                }
            } else {
                // 360도 전체 모드: 0°, 90°, 180°, 270°
                for (int base_angle = 0; base_angle < 360; base_angle += 90) {
                    float adjusted_angle = base_angle + angle_offset;
                    while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
                    while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;

                    // 180도 회전 보정
                    float screen_angle = (90.0f - adjusted_angle) * PI / 180.0f;
                    int end_x = center_x + static_cast<int>(radius_pixels * std::cos(screen_angle));
                    int end_y = center_y - static_cast<int>(radius_pixels * std::sin(screen_angle));

                    cv::Scalar line_color = (base_angle == 0) ? cv::Scalar(150, 150, 150) : cv::Scalar(100, 100, 100);
                    int line_thickness = (base_angle == 0) ? 2 : 1;
                    cv::line(frame, cv::Point(center_x, center_y),
                            cv::Point(end_x, end_y),
                            line_color, line_thickness, cv::LINE_AA);
                }
            }
        }
        
        // -2~2도 범위의 포인트들을 평균내어 상단에 텍스트로 표시
        std::vector<float> center_distances;
        for (const auto& point : points) {
            // 오프셋 적용하여 실제 각도 계산
            float adjusted_angle = point.angle + angle_offset;
            // 각도를 0~360도 범위로 정규화
            while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
            while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;
            
            // -2~2도 범위 체크 (0도 ± 2도)
            float angle_diff = std::abs(adjusted_angle);
            if (angle_diff > 180.0f) {
                angle_diff = 360.0f - angle_diff;
            }
            
            if (angle_diff <= LIDAR_CENTER_ANGLE_RANGE / 2.0f) {
                // 유효 범위 내의 거리만 사용
                if (point.distance >= 0.05f && point.distance <= 12.0f) {
                    center_distances.push_back(point.distance);
                }
            }
        }
        
        // 평균 거리 계산 및 상단에 텍스트로만 표시
        if (!center_distances.empty()) {
            float avg_distance = 0.0f;
            for (float dist : center_distances) {
                avg_distance += dist;
            }
            avg_distance /= center_distances.size();
            
            // 상단 중앙에 텍스트로 표시
            std::string dist_text = std::to_string(avg_distance).substr(0, 4) + "m";
            cv::Scalar text_color = getLidarColor(avg_distance);
            cv::putText(frame, dist_text, cv::Point(center_x - 30, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);
        }
        
        // 디스플레이 모드 가져오기
        std::string display_mode = LIDAR_DISPLAY_MODE;

        // THREE_POINTS 모드: 3방향 각 1포인트씩만 표시
        if (display_mode == "THREE_POINTS") {
            // 3방향: 270° (좌측, -90°), 0° (정면), 90° (우측)
            std::vector<float> target_angles = {270.0f, 0.0f, 90.0f};
            float tolerance = LIDAR_THREE_POINT_TOLERANCE;  // ±5도

            for (float target_angle : target_angles) {
                // 해당 각도 범위의 포인트들 수집
                std::vector<float> distances;
                for (const auto& point : points) {
                    float adjusted_angle = point.angle + angle_offset;
                    while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
                    while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;

                    // 유효 범위 체크
                    if (point.distance < 0.05f || point.distance > 12.0f) {
                        continue;
                    }

                    // 목표 각도와의 차이 계산
                    float angle_diff = std::abs(adjusted_angle - target_angle);
                    if (angle_diff > 180.0f) {
                        angle_diff = 360.0f - angle_diff;
                    }

                    // 허용 범위 내의 포인트만 수집
                    if (angle_diff <= tolerance) {
                        distances.push_back(point.distance);
                    }
                }

                // 평균 거리 계산 및 1개 포인트 표시
                if (!distances.empty()) {
                    float avg_distance = 0.0f;
                    for (float dist : distances) {
                        avg_distance += dist;
                    }
                    avg_distance /= distances.size();

                    // 거리를 픽셀 반경으로 변환
                    float normalized_dist = std::min(avg_distance / max_range, 1.0f);
                    int pixel_radius = static_cast<int>(normalized_dist * radius_pixels);

                    // 각도 변환 (오프셋 적용)
                    float display_angle = target_angle + angle_offset;
                    while (display_angle >= 360.0f) display_angle -= 360.0f;
                    while (display_angle < 0.0f) display_angle += 360.0f;

                    // 180도 회전 보정: 270 - angle → 90 - angle
                    // LiDAR 0도(전방) → 화면 상단(90도)
                    // LiDAR 90도(우측) → 화면 우측(0도)
                    // LiDAR 180도(후방) → 화면 하단(270도)
                    // LiDAR 270도(좌측) → 화면 좌측(180도)
                    float angle_rad = (90.0f - display_angle) * PI / 180.0f;
                    int x = center_x + static_cast<int>(pixel_radius * std::cos(angle_rad));
                    int y = center_y - static_cast<int>(pixel_radius * std::sin(angle_rad));

                    // 경계 체크
                    if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                        cv::Scalar color = getLidarColor(avg_distance);
                        // 큰 원으로 표시 (눈에 잘 띄게)
                        cv::circle(frame, cv::Point(x, y), 5, color, -1, cv::LINE_AA);
                        cv::circle(frame, cv::Point(x, y), 7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

                        // 거리 텍스트 표시
                        std::string dist_text = std::to_string(avg_distance).substr(0, 4) + "m";
                        cv::putText(frame, dist_text, cv::Point(x + 10, y - 10),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
                    }
                }
            }
        } else {
            // FULL_360 또는 FRONT_3DIR 모드: 모든 포인트 표시
            for (const auto& point : points) {
                // 오프셋 적용하여 실제 각도 계산
                float adjusted_angle = point.angle + angle_offset;
                // 각도를 0~360도 범위로 정규화
                while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
                while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;

                // 유효 범위 체크
                if (point.distance < 0.05f || point.distance > 12.0f) {
                    continue;  // 유효 범위를 벗어나면 스킵
                }

                // 디스플레이 모드에 따른 각도 필터링
                if (display_mode == "FRONT_3DIR") {
                    // 3방향 모드: -90°~90° 범위만 표시 (정면 중심)
                    // adjusted_angle이 0도 기준으로 ±90도 범위인지 체크
                    // 270°~360°(=-90°~0°) 또는 0°~90°
                    if (adjusted_angle > 90.0f && adjusted_angle < 270.0f) {
                        continue;  // 후방 180도는 스킵
                    }
                }
                // FULL_360 모드는 모든 각도 표시 (필터링 없음)
            
            // 거리를 픽셀 반경으로 변환 (12m = 200픽셀)
            float normalized_dist = std::min(point.distance / max_range, 1.0f);
            int pixel_radius = static_cast<int>(normalized_dist * radius_pixels);
            
            // 각도를 라디안으로 변환 (180도 회전 보정)
            // LiDAR: 0도=전방, 90도=우측, 180도=후방, 270도=좌측
            // 화면: 90도=상단, 0도=우측, 270도=하단, 180도=좌측
            // 변환: 90 - adjusted_angle
            // 검증:
            //   0도 → 90도 → cos(90°)=0, sin(90°)=1 → (0, -1*radius) = 상단 ✓
            //   90도 → 0도 → cos(0°)=1, sin(0°)=0 → (radius, 0) = 우측 ✓
            //   180도 → -90°=270도 → cos(270°)=0, sin(270°)=-1 → (0, +radius) = 하단 ✓
            //   270도 → -180°=180도 → cos(180°)=-1, sin(180°)=0 → (-radius, 0) = 좌측 ✓
            float angle_rad = (90.0f - adjusted_angle) * PI / 180.0f;
            int x = center_x + static_cast<int>(pixel_radius * std::cos(angle_rad));
            int y = center_y - static_cast<int>(pixel_radius * std::sin(angle_rad));  // Y축 반전
            
            // 경계 체크
            if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                // 거리에 따른 색상 (그라데이션)
                cv::Scalar color = getLidarColor(point.distance);
                
                // 포인트 그리기 (작은 원)
                cv::circle(frame, cv::Point(x, y), 2, color, -1, cv::LINE_AA);
            }
            }  // for loop 끝
        }  // else 블록 끝

        // -2~2도 범위 포인트 개수 계산 (오프셋 적용)
        int center_point_count = 0;
        float center_avg_distance = -1.0f;
        for (const auto& point : points) {
            float adjusted_angle = point.angle + angle_offset;
            while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
            while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;
            
            float angle_diff = std::abs(adjusted_angle);
            if (angle_diff > 180.0f) {
                angle_diff = 360.0f - angle_diff;
            }
            if (angle_diff <= LIDAR_CENTER_ANGLE_RANGE / 2.0f) {
                center_point_count++;
                if (point.distance >= 0.05f && point.distance <= 12.0f) {
                    if (center_avg_distance < 0.0f) {
                        center_avg_distance = point.distance;
                    } else {
                        center_avg_distance = (center_avg_distance + point.distance) / 2.0f;
                    }
                }
            }
        }
        
        // 거리 정보 텍스트 (우측 하단)
        std::string lidar_text = "LiDAR: " + std::to_string(points.size()) + " pts";
        if (center_avg_distance > 0.0f) {
            lidar_text += " | Center: " + std::to_string(center_avg_distance).substr(0, 4) + "m";
        }
        cv::putText(frame, lidar_text, cv::Point(frame.cols - 250, frame.rows - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 라이다 오버레이 오류: " << e.what() << std::endl;
    }
}

cv::Scalar FrameCompositor::getLidarColor(float distance) {
    // 거리에 따른 색상 그라데이션
    // 0m ~ 3m: 빨간색 → 주황색
    // 3m ~ 6m: 주황색 → 노란색
    // 6m ~ 10m: 노란색 → 초록색
    // 10m ~ 12m: 초록색 → 노란색
    
    if (distance < 0.05f) {
        return cv::Scalar(0, 0, 255);  // 빨간색 (매우 가까움) - BGR
    }
    
    if (distance <= 3.0f) {
        // 0m ~ 3m: 빨간색 → 주황색
        float ratio = distance / 3.0f;
        int r = 255;
        int g = static_cast<int>(ratio * 165);  // 0 → 165
        int b = 0;
        return cv::Scalar(b, g, r);  // BGR 형식
    } else if (distance <= 6.0f) {
        // 3m ~ 6m: 주황색 → 노란색
        float ratio = (distance - 3.0f) / 3.0f;
        int r = 255;
        int g = static_cast<int>(165 + ratio * 90);  // 165 → 255
        int b = 0;
        return cv::Scalar(b, g, r);  // BGR 형식
    } else if (distance <= 10.0f) {
        // 6m ~ 10m: 노란색 → 초록색
        float ratio = (distance - 6.0f) / 4.0f;
        int r = static_cast<int>(255 * (1.0f - ratio));  // 255 → 0
        int g = 255;
        int b = 0;
        return cv::Scalar(b, g, r);  // BGR 형식
    } else {
        // 10m ~ 12m: 초록색 → 노란색
        float ratio = (distance - 10.0f) / 2.0f;
        int r = static_cast<int>(255 * ratio);  // 0 → 255
        int g = 255;
        int b = 0;
        return cv::Scalar(b, g, r);  // BGR 형식
    }
}
