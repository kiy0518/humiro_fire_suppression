#include "distance_overlay.h"
#include "../../lidar/src/lidar_interface.h"
#include "../../thermal/src/config.h"
#include <cmath>
#include <algorithm>
#include <iostream>

DistanceOverlay::DistanceOverlay() 
    : lidar_angle_offset_(0.0f)
    , display_mode_("FULL_360")
    , show_direction_lines_(false)
    , three_point_tolerance_(5.0f) {
}

DistanceOverlay::~DistanceOverlay() {
}

void DistanceOverlay::setLidarData(const std::vector<LidarPoint>& lidar_points) {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_points_ = lidar_points;
}

void DistanceOverlay::setLidarOrientation(float offset_degrees) {
    std::lock_guard<std::mutex> lock(orientation_mutex_);
    lidar_angle_offset_ = offset_degrees;
    // 각도를 0~360도 범위로 정규화
    while (lidar_angle_offset_ >= 360.0f) lidar_angle_offset_ -= 360.0f;
    while (lidar_angle_offset_ < 0.0f) lidar_angle_offset_ += 360.0f;
}

void DistanceOverlay::setDisplayMode(const std::string& mode) {
    display_mode_ = mode;
}

void DistanceOverlay::setShowDirectionLines(bool show) {
    show_direction_lines_ = show;
}

void DistanceOverlay::setThreePointTolerance(float tolerance) {
    three_point_tolerance_ = tolerance;
}

void DistanceOverlay::drawOverlay(cv::Mat& frame) {
    // 라이다 데이터 가져오기 (스레드 안전)
    std::vector<LidarPoint> points;
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        points = lidar_points_;
    }
    
    if (points.empty()) {
        return;
    }
    
    // 디버깅: 각도별 포인트 개수 카운트
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 30 == 0) {  // 1초에 한 번 출력 (30fps 기준)
        int quadrant[4] = {0};  // 0~90, 90~180, 180~270, 270~360
        for (const auto& point : points) {
            float angle = point.angle;
            while (angle >= 360.0f) angle -= 360.0f;
            while (angle < 0.0f) angle += 360.0f;
            
            if (angle < 90.0f) quadrant[0]++;
            else if (angle < 180.0f) quadrant[1]++;
            else if (angle < 270.0f) quadrant[2]++;
            else quadrant[3]++;
        }
        std::cout << "[LiDAR Debug] 각도별 포인트: 0-90도:" << quadrant[0] 
                  << " | 90-180도:" << quadrant[1]
                  << " | 180-270도:" << quadrant[2]
                  << " | 270-360도:" << quadrant[3]
                  << " | 전체:" << points.size() << std::endl;
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
        
        // 라이다 오프셋 가져오기 (스레드 안전)
        float angle_offset = 0.0f;
        {
            std::lock_guard<std::mutex> lock(orientation_mutex_);
            angle_offset = lidar_angle_offset_;
        }
        
        // 거리 링 그리기
        drawDistanceRings(frame, center_x, center_y, radius_pixels, max_range);
        
        // 중심 십자선
        cv::line(frame, cv::Point(center_x - 10, center_y), 
                cv::Point(center_x + 10, center_y), 
                cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
        cv::line(frame, cv::Point(center_x, center_y - 10), 
                cv::Point(center_x, center_y + 10), 
                cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
        
        // 방향선 표시 (옵션)
        if (show_direction_lines_) {
            drawDirectionLines(frame, center_x, center_y, radius_pixels, angle_offset, display_mode_);
        }
        
        // 중심 거리 텍스트 표시
        drawCenterDistanceText(frame, center_x, points, angle_offset);
        
        // 포인트 표시
        if (display_mode_ == "THREE_POINTS") {
            drawThreePoints(frame, center_x, center_y, radius_pixels, max_range, points, angle_offset);
        } else {
            drawAllPoints(frame, center_x, center_y, radius_pixels, max_range, points, angle_offset, display_mode_);
        }
        
        // 거리 정보 텍스트 (우측 하단)
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

void DistanceOverlay::drawDistanceRings(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float max_range) {
    // 원형 그리드 그리기 (거리 링)
    // 3m, 6m, 9m, 12m 링
    std::vector<float> distance_rings = {3.0f, 6.0f, 9.0f, 12.0f};
    for (float dist : distance_rings) {
        int ring_radius = static_cast<int>((dist / max_range) * radius_pixels);
        cv::circle(frame, cv::Point(center_x, center_y), ring_radius, 
                  cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
    }
}

void DistanceOverlay::drawDirectionLines(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float angle_offset, const std::string& display_mode) {
    const float PI = 3.14159265358979323846f;
    
    if (display_mode == "FRONT_3DIR") {
        // 3방향 모드: -90°, 0°, 90°
        std::vector<int> base_angles = {270, 0, 90};  // LD19 각도
        
        for (int base_angle : base_angles) {
            float adjusted_angle = base_angle + angle_offset;
            while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
            while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;
            
            // LD19 시계방향 회전 보정
            float screen_angle = (adjusted_angle - 90.0f) * PI / 180.0f;
            int end_x = center_x + static_cast<int>(std::round(radius_pixels * std::cos(screen_angle)));
            int end_y = center_y + static_cast<int>(std::round(radius_pixels * std::sin(screen_angle)));
            
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
            
            // LD19 시계방향 회전 보정
            float screen_angle = (adjusted_angle - 90.0f) * PI / 180.0f;
            int end_x = center_x + static_cast<int>(std::round(radius_pixels * std::cos(screen_angle)));
            int end_y = center_y + static_cast<int>(std::round(radius_pixels * std::sin(screen_angle)));
            
            cv::Scalar line_color = (base_angle == 0) ? cv::Scalar(150, 150, 150) : cv::Scalar(100, 100, 100);
            int line_thickness = (base_angle == 0) ? 2 : 1;
            cv::line(frame, cv::Point(center_x, center_y),
                    cv::Point(end_x, end_y),
                    line_color, line_thickness, cv::LINE_AA);
        }
    }
}

void DistanceOverlay::drawCenterDistanceText(cv::Mat& frame, int center_x, const std::vector<LidarPoint>& points, float angle_offset) {
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
}

void DistanceOverlay::drawThreePoints(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float max_range, const std::vector<LidarPoint>& points, float angle_offset) {
    const float PI = 3.14159265358979323846f;
    
    // 3방향: 270° (좌측, -90°), 0° (정면), 90° (우측)
    std::vector<float> target_angles = {270.0f, 0.0f, 90.0f};
    
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
            if (angle_diff <= three_point_tolerance_) {
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
            
            // LD19 시계방향 회전 보정
            float angle_rad = (display_angle - 90.0f) * PI / 180.0f;
            int x = center_x + static_cast<int>(std::round(pixel_radius * std::cos(angle_rad)));
            int y = center_y + static_cast<int>(std::round(pixel_radius * std::sin(angle_rad)));
            
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
}

void DistanceOverlay::drawAllPoints(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float max_range, const std::vector<LidarPoint>& points, float angle_offset, const std::string& display_mode) {
    const float PI = 3.14159265358979323846f;
    
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
        float pixel_radius_float = normalized_dist * radius_pixels;
        
        // 각도를 라디안으로 변환 (LD19 시계방향 회전 보정)
        float angle_rad = (adjusted_angle - 90.0f) * PI / 180.0f;
        
        // 반올림을 사용하여 원형 정밀도 향상
        int x = center_x + static_cast<int>(std::round(pixel_radius_float * std::cos(angle_rad)));
        int y = center_y + static_cast<int>(std::round(pixel_radius_float * std::sin(angle_rad)));
        
        // 경계 체크
        if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
            // 거리에 따른 색상 (그라데이션)
            cv::Scalar color = getLidarColor(point.distance);
            
            // 포인트 그리기 (작은 원)
            cv::circle(frame, cv::Point(x, y), 2, color, -1, cv::LINE_AA);
        }
    }
}

cv::Scalar DistanceOverlay::getLidarColor(float distance) {
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

