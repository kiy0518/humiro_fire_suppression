#include "distance_overlay.h"
#include <algorithm>
#include <cmath>

DistanceOverlay::DistanceOverlay(float camera_fov, int image_width, int image_height)
    : camera_fov_(camera_fov)
    , image_width_(image_width)
    , image_height_(image_height)
    , min_distance_(0.05f)   // LD19 최소 거리
    , max_distance_(12.0f)   // LD19 최대 거리 (12m)
    , color_optimal_(cv::Scalar(0, 255, 0))    // 초록색 (BGR)
    , color_close_(cv::Scalar(0, 0, 255))      // 빨간색
    , color_far_(cv::Scalar(255, 0, 0))        // 파란색
{
}

int DistanceOverlay::angleToScreenX(float angle) {
    // 각도를 -camera_fov_/2 ~ +camera_fov_/2 범위로 정규화
    // 0도 = 정면 = 화면 중앙
    
    // 각도를 -180 ~ +180 범위로 변환
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    
    // FOV 범위를 벗어나면 -1 반환
    float half_fov = camera_fov_ / 2.0f;
    if (angle < -half_fov || angle > half_fov) {
        return -1;
    }
    
    // 각도를 X 좌표로 변환
    // -half_fov -> 0, 0 -> width/2, +half_fov -> width
    float normalized = (angle + half_fov) / camera_fov_;
    int x = static_cast<int>(normalized * image_width_);
    
    return std::max(0, std::min(image_width_ - 1, x));
}

int DistanceOverlay::distanceToScreenY(float distance) {
    // 거리를 Y 좌표로 변환
    // 가까운 거리 -> 아래(큰 Y), 먼 거리 -> 위(작은 Y)
    
    if (distance < min_distance_) distance = min_distance_;
    if (distance > max_distance_) distance = max_distance_;
    
    // 선형 매핑 (역방향)
    float normalized = (distance - min_distance_) / (max_distance_ - min_distance_);
    
    // 화면 하단 1/3 영역 사용
    int y_min = image_height_ * 2 / 3;
    int y_max = image_height_ - 20;
    
    int y = y_max - static_cast<int>(normalized * (y_max - y_min));
    
    return std::max(y_min, std::min(y_max, y));
}

cv::Scalar DistanceOverlay::getColorForDistance(float distance) {
    // 거리에 따른 색상 반환 (12m 범위 기준)
    // 9m ~ 11m: 초록색 (최적)
    // < 9m: 빨간색 (너무 가까움)
    // > 11m: 파란색 (너무 멀음, 최대 12m)
    
    // 12m 범위를 벗어나면 무효 데이터로 처리
    if (distance > max_distance_) {
        return cv::Scalar(128, 128, 128);  // 회색 (무효)
    }
    
    if (distance >= 9.0f && distance <= 11.0f) {
        // 최적 거리 - 초록색
        return color_optimal_;
    } else if (distance < 9.0f) {
        // 가까운 거리 - 빨간색
        // 거리가 가까울수록 밝은 빨간색 (0.05m ~ 9m)
        float intensity = std::max(0.0f, std::min(1.0f, (9.0f - distance) / 9.0f));
        return cv::Scalar(0, 0, 128 + static_cast<int>(127 * intensity));
    } else {
        // 먼 거리 - 파란색 (11m ~ 12m)
        // 거리가 멀수록 밝은 파란색
        float intensity = std::max(0.0f, std::min(1.0f, (distance - 11.0f) / 1.0f));
        return cv::Scalar(128 + static_cast<int>(127 * intensity), 0, 0);
    }
}

void DistanceOverlay::drawDistanceLine(cv::Mat& frame, 
                                       const std::vector<LidarPoint>& lidar_data) {
    if (lidar_data.empty()) {
        return;
    }
    
    // 각도별로 정렬
    std::vector<LidarPoint> sorted_data = lidar_data;
    std::sort(sorted_data.begin(), sorted_data.end(), 
              [](const LidarPoint& a, const LidarPoint& b) {
                  return a.angle < b.angle;
              });
    
    // 라인 그리기
    cv::Point prev_point(-1, -1);
    
    for (const auto& point : sorted_data) {
        int x = angleToScreenX(point.angle);
        if (x < 0) continue;  // FOV 범위 밖
        
        int y = distanceToScreenY(point.distance);
        cv::Scalar color = getColorForDistance(point.distance);
        
        cv::Point current_point(x, y);
        
        // 포인트 그리기
        cv::circle(frame, current_point, 3, color, -1);
        
        // 이전 포인트와 연결
        if (prev_point.x >= 0) {
            // 거리가 너무 크면 연결하지 않음
            if (std::abs(current_point.x - prev_point.x) < 50) {
                cv::line(frame, prev_point, current_point, color, 2);
            }
        }
        
        prev_point = current_point;
    }
    
    // 기준선 그리기 (10m 위치)
    int y_10m = distanceToScreenY(10.0f);
    cv::line(frame, cv::Point(0, y_10m), 
             cv::Point(image_width_, y_10m), 
             cv::Scalar(0, 200, 0), 1, cv::LINE_AA);
    
    // 거리 눈금 표시 (12m 범위에 맞게 조정)
    std::vector<float> distance_marks = {3.0f, 6.0f, 9.0f, 12.0f};
    for (float dist : distance_marks) {
        int y = distanceToScreenY(dist);
        cv::putText(frame, std::to_string(static_cast<int>(dist)) + "m",
                   cv::Point(5, y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4,
                   cv::Scalar(255, 255, 255), 1);
    }
}

float DistanceOverlay::calculateAverageDistance(const std::vector<LidarPoint>& lidar_data) {
    if (lidar_data.empty()) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    for (const auto& point : lidar_data) {
        sum += point.distance;
    }
    
    return sum / lidar_data.size();
}

void DistanceOverlay::drawDistanceInfo(cv::Mat& frame,
                                       const std::vector<LidarPoint>& lidar_data) {
    if (lidar_data.empty()) {
        cv::putText(frame, "LiDAR: No Data",
                   cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   cv::Scalar(0, 0, 255), 2);
        return;
    }

    // 평균 거리 계산
    float avg_distance = calculateAverageDistance(lidar_data);

    // 최소/최대 거리
    float min_dist = lidar_data[0].distance;
    float max_dist = lidar_data[0].distance;

    for (const auto& point : lidar_data) {
        if (point.distance < min_dist) min_dist = point.distance;
        if (point.distance > max_dist) max_dist = point.distance;
    }

    // 정보 패널 그리기
    int panel_x = 10;
    int panel_y = 10;
    int panel_width = 250;
    int panel_height = 100;

    // 반투명 배경
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay,
                 cv::Point(panel_x, panel_y),
                 cv::Point(panel_x + panel_width, panel_y + panel_height),
                 cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.5, frame, 0.5, 0, frame);

    // 텍스트 정보
    int text_y = panel_y + 25;
    cv::putText(frame, "LiDAR Distance Info",
               cv::Point(panel_x + 10, text_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.5,
               cv::Scalar(255, 255, 255), 1);

    text_y += 25;
    char avg_text[50];
    snprintf(avg_text, sizeof(avg_text), "Avg: %.2f m", avg_distance);
    cv::Scalar avg_color = getColorForDistance(avg_distance);
    cv::putText(frame, avg_text,
               cv::Point(panel_x + 10, text_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.5,
               avg_color, 1);

    text_y += 20;
    char range_text[50];
    snprintf(range_text, sizeof(range_text), "Range: %.2f ~ %.2f m", min_dist, max_dist);
    cv::putText(frame, range_text,
               cv::Point(panel_x + 10, text_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(200, 200, 200), 1);

    text_y += 20;
    const char* status_text;
    if (avg_distance >= 9.0f && avg_distance <= 11.0f) {
        status_text = "Status: OPTIMAL";
        cv::putText(frame, status_text,
                   cv::Point(panel_x + 10, text_y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(0, 255, 0), 1);
    } else if (avg_distance < 9.0f) {
        status_text = "Status: TOO CLOSE";
        cv::putText(frame, status_text,
                   cv::Point(panel_x + 10, text_y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(0, 0, 255), 1);
    } else {
        status_text = "Status: TOO FAR";
        cv::putText(frame, status_text,
                   cv::Point(panel_x + 10, text_y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(255, 0, 0), 1);
    }
}

void DistanceOverlay::drawRadarView(cv::Mat& frame,
                                    const std::vector<LidarPoint>& lidar_data) {
    // 화면 중앙에 360도 원형 레이더 뷰 그리기

    // 중심점 및 반지름 계산
    cv::Point center(image_width_ / 2, image_height_ / 2);
    int max_radius = std::min(image_width_, image_height_) / 2 - 40;

    // 배경: 검은색 반투명
    cv::Mat overlay = frame.clone();
    cv::circle(overlay, center, max_radius + 10, cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.7, frame, 0.3, 0, frame);

    // 거리 원 그리기 (3m, 6m, 9m, 12m)
    std::vector<float> distance_rings = {3.0f, 6.0f, 9.0f, 12.0f};
    for (float dist : distance_rings) {
        int radius = static_cast<int>((dist / max_distance_) * max_radius);
        cv::Scalar ring_color;

        if (dist == 9.0f || dist == 12.0f) {
            // 9m(최적 시작), 12m(최대) - 밝은 회색
            ring_color = cv::Scalar(100, 100, 100);
        } else {
            // 3m, 6m - 어두운 회색
            ring_color = cv::Scalar(60, 60, 60);
        }

        cv::circle(frame, center, radius, ring_color, 1, cv::LINE_AA);

        // 거리 레이블
        char label[10];
        snprintf(label, sizeof(label), "%.0fm", dist);
        cv::putText(frame, label,
                   cv::Point(center.x + 5, center.y - radius + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4,
                   cv::Scalar(150, 150, 150), 1);
    }

    // 방향 라인 및 레이블 (3방향: -90°, 0°, 90°)
    // 화면상 위쪽=0°(정면), 오른쪽=90°, 왼쪽=-90°(270°)
    std::vector<std::pair<int, std::string>> direction_labels = {
        {180, "0° (Front)"},      // 정면 - 위쪽
        {270, "90° (Right)"},     // 우측 - 오른쪽
        {90, "-90° (Left)"}       // 좌측 - 왼쪽
    };

    for (const auto& dir : direction_labels) {
        int angle_deg = dir.first;
        // OpenCV 좌표계: 0도=위, 시계방향
        // LiDAR 좌표계: 0도=정면, 반시계방향
        // 변환: 270 - angle (180도 회전 보정)
        float angle_rad = (270 - angle_deg) * M_PI / 180.0f;

        int x1 = center.x + static_cast<int>(max_radius * std::cos(angle_rad));
        int y1 = center.y - static_cast<int>(max_radius * std::sin(angle_rad));

        cv::line(frame, center, cv::Point(x1, y1),
                cv::Scalar(80, 80, 80), 1, cv::LINE_AA);

        // 방향 레이블
        int label_x = center.x + static_cast<int>((max_radius + 20) * std::cos(angle_rad));
        int label_y = center.y - static_cast<int>((max_radius + 20) * std::sin(angle_rad));

        cv::putText(frame, dir.second,
                   cv::Point(label_x - 20, label_y + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(200, 200, 200), 1);
    }

    // LiDAR 데이터 포인트 그리기
    if (!lidar_data.empty()) {
        for (const auto& point : lidar_data) {
            // 거리를 반지름으로 변환
            float normalized_dist = std::min(point.distance / max_distance_, 1.0f);
            int radius = static_cast<int>(normalized_dist * max_radius);

            // LiDAR 각도를 OpenCV 각도로 변환 (180도 보정)
            // LiDAR: 0도=정면, 반시계방향
            // OpenCV: 0도=오른쪽, 반시계방향
            // 변환: 270 - angle (180도 회전 보정)
            float angle_rad = (270 - point.angle) * M_PI / 180.0f;

            int x = center.x + static_cast<int>(radius * std::cos(angle_rad));
            int y = center.y - static_cast<int>(radius * std::sin(angle_rad));

            // 색상 결정
            cv::Scalar color = getColorForDistance(point.distance);

            // 포인트 그리기
            cv::circle(frame, cv::Point(x, y), 3, color, -1, cv::LINE_AA);
        }

        // 중심점 표시 (드론 위치)
        cv::circle(frame, center, 8, cv::Scalar(255, 255, 255), -1);
        cv::circle(frame, center, 6, cv::Scalar(0, 0, 0), -1);
        cv::putText(frame, "DRONE",
                   cv::Point(center.x - 25, center.y + 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4,
                   cv::Scalar(255, 255, 255), 1);
    }

    // 레이더 외곽 원
    cv::circle(frame, center, max_radius, cv::Scalar(150, 150, 150), 2, cv::LINE_AA);

    // 타이틀
    cv::putText(frame, "360° LiDAR Radar View",
               cv::Point(center.x - 100, 25),
               cv::FONT_HERSHEY_SIMPLEX, 0.7,
               cv::Scalar(255, 255, 255), 2);

    // 데이터 포인트 개수 표시
    char count_text[50];
    snprintf(count_text, sizeof(count_text), "Points: %zu", lidar_data.size());
    cv::putText(frame, count_text,
               cv::Point(10, image_height_ - 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.5,
               cv::Scalar(200, 200, 200), 1);

    // 범례 (우측 상단)
    int legend_x = image_width_ - 120;
    int legend_y = 20;

    cv::putText(frame, "Distance:",
               cv::Point(legend_x, legend_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(255, 255, 255), 1);

    legend_y += 20;
    cv::rectangle(frame, cv::Point(legend_x, legend_y - 10),
                 cv::Point(legend_x + 15, legend_y + 5),
                 color_optimal_, -1);
    cv::putText(frame, "9-11m",
               cv::Point(legend_x + 20, legend_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(200, 200, 200), 1);

    legend_y += 20;
    cv::rectangle(frame, cv::Point(legend_x, legend_y - 10),
                 cv::Point(legend_x + 15, legend_y + 5),
                 color_close_, -1);
    cv::putText(frame, "<9m",
               cv::Point(legend_x + 20, legend_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(200, 200, 200), 1);

    legend_y += 20;
    cv::rectangle(frame, cv::Point(legend_x, legend_y - 10),
                 cv::Point(legend_x + 15, legend_y + 5),
                 color_far_, -1);
    cv::putText(frame, ">11m",
               cv::Point(legend_x + 20, legend_y),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(200, 200, 200), 1);
}
