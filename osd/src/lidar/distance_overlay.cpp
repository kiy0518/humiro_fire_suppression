#include "distance_overlay.h"
#include "../../lidar/src/lidar_interface.h"
#include "../../thermal/src/config.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

DistanceOverlay::DistanceOverlay() 
    : lidar_angle_offset_(0.0f)
    , cached_center_distance_(-1.0f)
    , last_confidence_update_(std::chrono::steady_clock::now()) {
}

DistanceOverlay::~DistanceOverlay() {
}

void DistanceOverlay::setLidarData(const std::vector<LidarPoint>& lidar_points) {
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        lidar_points_ = lidar_points;
    }
    
    // SLAM 방식 포인트 캐시 업데이트
    auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(point_cache_mutex_);
        
        // 주기적으로 신뢰도 감소 (업데이트되지 않은 포인트)
        auto cycle_duration = std::chrono::milliseconds(UPDATE_CYCLE_MS);
        if (now - last_confidence_update_ >= cycle_duration) {
            for (auto& entry : point_cache_) {
                // 신뢰도 점진적 감소 (SLAM 방식)
                entry.second.confidence *= CONFIDENCE_DECAY;
            }
            last_confidence_update_ = now;
        }
        
        // 새 데이터로 포인트 캐시 업데이트 (SLAM 방식: Exponential Smoothing)
        for (const auto& point : lidar_points) {
            // 각도를 1도 단위 버킷으로 변환 (0~359)
            int angle_bucket = static_cast<int>(std::round(point.angle)) % 360;
            if (angle_bucket < 0) angle_bucket += 360;
            
            auto it = point_cache_.find(angle_bucket);
            if (it != point_cache_.end()) {
                // 기존 포인트가 있으면 SLAM 방식으로 업데이트
                PointEntry& entry = it->second;
                
                // 거리 변화 계산
                float distance_diff = std::abs(point.distance - entry.point.distance);
                
                // 변화가 크면 노이즈 가능성 (낮은 가중치로 업데이트)
                // 변화가 작으면 일관성 있는 관측 (높은 가중치로 업데이트)
                float update_alpha = SMOOTHING_ALPHA;
                if (distance_diff > CHANGE_THRESHOLD) {
                    // 갑작스러운 변화는 노이즈 가능성이 높으므로 낮은 가중치
                    update_alpha = SMOOTHING_ALPHA * 0.5f;
                }
                
                // Exponential smoothing: 거리 업데이트
                entry.point.distance = update_alpha * point.distance + (1.0f - update_alpha) * entry.point.distance;
                
                // 각도와 품질은 새 값으로 업데이트 (거리가 주요 관심사)
                entry.point.angle = point.angle;
                entry.point.quality = point.quality;
                
                // 신뢰도 증가 (새 관측이 있으므로)
                entry.confidence = std::min(1.0f, entry.confidence + 0.3f);
                entry.last_update = now;
            } else {
                // 새로운 포인트 추가 (초기 신뢰도 1.0)
                PointEntry new_entry;
                new_entry.point = point;
                new_entry.confidence = 1.0f;
                new_entry.last_update = now;
                point_cache_[angle_bucket] = new_entry;
            }
        }
        
        // 신뢰도가 너무 낮은 포인트 제거
        for (auto it = point_cache_.begin(); it != point_cache_.end();) {
            if (it->second.confidence < MIN_CONFIDENCE) {
                it = point_cache_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    // 전방 거리 계산 및 캐시 업데이트 (비동기 처리)
    if (!lidar_points.empty()) {
        float angle_offset = 0.0f;
        {
            std::lock_guard<std::mutex> lock(orientation_mutex_);
            angle_offset = lidar_angle_offset_;
        }
        
        // -2~2도 범위의 포인트들을 평균내어 전방 거리 계산
        std::vector<float> center_distances;
        for (const auto& point : lidar_points) {
            float adjusted_angle = point.angle + angle_offset;
            while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
            while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;
            
            float angle_diff = std::abs(adjusted_angle);
            if (angle_diff > 180.0f) {
                angle_diff = 360.0f - angle_diff;
            }
            
            if (angle_diff <= LIDAR_CENTER_ANGLE_RANGE / 2.0f) {
                if (point.distance >= 0.05f && point.distance <= 12.0f) {
                    center_distances.push_back(point.distance);
                }
            }
        }
        
        // 평균 거리 계산 및 캐시 업데이트
        // center_distances가 비어있으면 캐시를 업데이트하지 않음 (마지막 유효한 값 유지)
        // 이렇게 하면 데이터가 일시적으로 없을 때도 깜빡임이 발생하지 않음
        if (!center_distances.empty()) {
            float avg_distance = 0.0f;
            for (float dist : center_distances) {
                avg_distance += dist;
            }
            avg_distance /= center_distances.size();
            
            std::lock_guard<std::mutex> lock(distance_cache_mutex_);
            cached_center_distance_ = avg_distance;
        }
        // else: center_distances가 비어있으면 캐시를 업데이트하지 않아
        //       마지막 유효한 값이 계속 유지됨 (깜빡임 방지)
    }
}

void DistanceOverlay::setLidarOrientation(float offset_degrees) {
    std::lock_guard<std::mutex> lock(orientation_mutex_);
    lidar_angle_offset_ = offset_degrees;
    // 각도를 0~360도 범위로 정규화
    while (lidar_angle_offset_ >= 360.0f) lidar_angle_offset_ -= 360.0f;
    while (lidar_angle_offset_ < 0.0f) lidar_angle_offset_ += 360.0f;
}

void DistanceOverlay::drawOverlay(cv::Mat& frame) {
    // 포인트 캐시에서 포인트 가져오기 (SLAM 방식, 신뢰도 기반)
    std::vector<LidarPoint> points;
    {
        std::lock_guard<std::mutex> lock(point_cache_mutex_);
        points.reserve(point_cache_.size());
        for (const auto& entry : point_cache_) {
            // 신뢰도가 충분한 포인트만 표시 (선택적: 모든 포인트 표시도 가능)
            // if (entry.second.confidence > 0.3f) {
                points.push_back(entry.second.point);
            // }
        }
    }
    
    try {
        // 프레임 중심 X 좌표 계산
        int center_x = frame.cols / 2;
        
        // 라이다 오프셋 가져오기 (스레드 안전)
        float angle_offset = 0.0f;
        {
            std::lock_guard<std::mutex> lock(orientation_mutex_);
            angle_offset = lidar_angle_offset_;
        }
        
        // 중심 거리 텍스트 표시 (캐시된 값 사용, points가 비어있어도 표시)
        drawCenterDistanceText(frame, center_x);
        
        // 미니맵 표시 (오른쪽 상단, 항상 표시 - 데이터가 없어도 미니맵 자체는 그려야 함)
        drawMinimap(frame, points, angle_offset);
        
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 라이다 오버레이 오류: " << e.what() << std::endl;
    }
}

void DistanceOverlay::drawMinimap(cv::Mat& frame, const std::vector<LidarPoint>& points, float angle_offset) {
    const float PI = 3.14159265358979323846f;
    const int MINIMAP_SIZE = 100;  // 100x100 픽셀
    const int MINIMAP_RADIUS = MINIMAP_SIZE / 2;  // 미니맵 반경 (50픽셀)
    const int MINIMAP_MARGIN = 5;  // 프레임 가장자리 여백
    const float MAX_RANGE = LIDAR_MAX_RANGE;  // 12m
    const float BACKGROUND_ALPHA = 0.8f;  // 미니맵 원 내부 배경 투명도
    
    // 미니맵 중심 위치 계산 (오른쪽 상단)
    int minimap_center_x = frame.cols - MINIMAP_MARGIN - MINIMAP_RADIUS;
    int minimap_center_y = MINIMAP_MARGIN + MINIMAP_RADIUS;
    
    // 미니맵 영역이 프레임을 벗어나지 않도록 체크
    if (minimap_center_x < MINIMAP_RADIUS || minimap_center_y < MINIMAP_RADIUS ||
        minimap_center_x + MINIMAP_RADIUS >= frame.cols ||
        minimap_center_y + MINIMAP_RADIUS >= frame.rows) {
        return;  // 프레임 범위를 벗어나면 그리지 않음
    }
    
    // 미니맵 원 내부를 검정색으로 채우기 (투명도 0.8)
    // 원형 영역 내부만 처리
    for (int y = minimap_center_y - MINIMAP_RADIUS; y < minimap_center_y + MINIMAP_RADIUS; y++) {
        for (int x = minimap_center_x - MINIMAP_RADIUS; x < minimap_center_x + MINIMAP_RADIUS; x++) {
            int dx = x - minimap_center_x;
            int dy = y - minimap_center_y;
            // 원형 영역 내부인지 확인
            if (dx * dx + dy * dy <= MINIMAP_RADIUS * MINIMAP_RADIUS) {
                if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                    cv::Vec3b& pixel = frame.at<cv::Vec3b>(y, x);
                    // 검정색과 블렌딩 (투명도 0.8)
                    pixel[0] = static_cast<uchar>(pixel[0] * (1.0f - BACKGROUND_ALPHA));  // B
                    pixel[1] = static_cast<uchar>(pixel[1] * (1.0f - BACKGROUND_ALPHA));  // G
                    pixel[2] = static_cast<uchar>(pixel[2] * (1.0f - BACKGROUND_ALPHA));  // R
                }
            }
        }
    }
    
    // 미니맵 테두리 그리기 (흰색 원)
    cv::circle(frame, cv::Point(minimap_center_x, minimap_center_y), MINIMAP_RADIUS,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    
    // 거리 링 그리기 (12m 원, 회색)
    int ring_radius = static_cast<int>((12.0f / MAX_RANGE) * MINIMAP_RADIUS);
    cv::circle(frame, cv::Point(minimap_center_x, minimap_center_y), ring_radius,
              cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
    
    // 센터에 점 그리기 (흰색)
    cv::circle(frame, cv::Point(minimap_center_x, minimap_center_y), 2,
              cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    
    // 라이다 포인트 그리기
    for (const auto& point : points) {
        // 오프셋 적용하여 실제 각도 계산
        float adjusted_angle = point.angle + angle_offset;
        while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
        while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;
        
        // 유효 범위 체크
        if (point.distance < 0.05f || point.distance > 12.0f) {
            continue;  // 유효 범위를 벗어나면 스킵
        }
        
        // 거리를 픽셀 반경으로 변환 (12m = MINIMAP_RADIUS 픽셀)
        float normalized_dist = std::min(point.distance / MAX_RANGE, 1.0f);
        float pixel_radius_float = normalized_dist * MINIMAP_RADIUS;
        
        // 각도를 라디안으로 변환
        // LD19: 0° = 전방 (X축), 시계방향 증가
        // 화면: 0° = 위쪽, 시계방향 증가
        // OpenCV 좌표계: Y축이 아래로 증가하므로 각도 보정 필요
        // 수학 좌표계에서 0°는 오른쪽, 시계방향은 음수
        // 화면에서 0°를 위쪽으로 표시하려면: angle_rad = (adjusted_angle - 90.0f) * PI / 180.0f
        float angle_rad = (adjusted_angle - 90.0f) * PI / 180.0f;
        
        // 미니맵 좌표 계산
        int x = minimap_center_x + static_cast<int>(std::round(pixel_radius_float * std::cos(angle_rad)));
        int y = minimap_center_y + static_cast<int>(std::round(pixel_radius_float * std::sin(angle_rad)));
        
        // 미니맵 영역 내에 있는지 체크 (원형 영역)
        int dx = x - minimap_center_x;
        int dy = y - minimap_center_y;
        if (dx * dx + dy * dy <= MINIMAP_RADIUS * MINIMAP_RADIUS) {
            // 프레임 경계 체크
            if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                // 거리에 따른 색상 (그라데이션)
                cv::Scalar color = getLidarColor(point.distance);
                
                // 포인트 그리기 (1픽셀)
                frame.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    static_cast<uchar>(color[0]),
                    static_cast<uchar>(color[1]),
                    static_cast<uchar>(color[2])
                );
            }
        }
    }
}

void DistanceOverlay::drawCenterDistanceText(cv::Mat& frame, int center_x) {
    // 캐시된 전방 거리 값 사용 (깜빡임 방지)
    float center_distance = -1.0f;
    {
        std::lock_guard<std::mutex> lock(distance_cache_mutex_);
        center_distance = cached_center_distance_;
    }
    
    // 유효한 거리 값이 있으면 표시
    if (center_distance > 0.0f) {
        std::string dist_text = std::to_string(center_distance).substr(0, 4) + "m";
        cv::Scalar text_color = getLidarColor(center_distance);
        cv::putText(frame, dist_text, cv::Point(center_x - 30, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);
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

