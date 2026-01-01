#include "distance_overlay.h"
#include "../../lidar/src/lidar_interface.h"
#include "../../thermal/src/config.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>

DistanceOverlay::DistanceOverlay()
    : lidar_angle_offset_(0.0f)
    , cached_center_distance_(-1.0f)
    , show_lidar_points_(true)  // 기본값: 포인트 표시
    , last_confidence_update_(std::chrono::steady_clock::now())
    , smoothing_alpha_(0.2f)       // 기본값
    , confidence_decay_(0.55f)     // 기본값: 0.65 (더 빠른 사라짐, 작을수록 빠름, 0.75→0.65)
    , min_confidence_(0.45f)        // 기본값: 0.5 (더 빠른 제거, 클수록 빠름, 0.4→0.5)
    , change_threshold_(0.5f)      // 기본값
    , update_cycle_ms_(40) {       // 기본값: 40ms (더 자주 업데이트, 50→40)
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

    // SLAM 파라미터 가져오기 (스레드 안전)
    float smoothing_alpha, confidence_decay, min_confidence, change_threshold;
    int update_cycle_ms;
    {
        std::lock_guard<std::mutex> lock(slam_params_mutex_);
        smoothing_alpha = smoothing_alpha_;
        confidence_decay = confidence_decay_;
        min_confidence = min_confidence_;
        change_threshold = change_threshold_;
        update_cycle_ms = update_cycle_ms_;
    }

    {
        std::lock_guard<std::mutex> lock(point_cache_mutex_);

        // 주기적으로 신뢰도 감소 (업데이트되지 않은 포인트)
        auto cycle_duration = std::chrono::milliseconds(update_cycle_ms);
        if (now - last_confidence_update_ >= cycle_duration) {
            for (auto& entry : point_cache_) {
                // 신뢰도 점진적 감소 (SLAM 방식)
                entry.second.confidence *= confidence_decay;
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
                float update_alpha = smoothing_alpha;
                if (distance_diff > change_threshold) {
                    // 갑작스러운 변화는 노이즈 가능성이 높으므로 낮은 가중치
                    update_alpha = smoothing_alpha * 0.5f;
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
            if (it->second.confidence < min_confidence) {
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
        
        // 메디안 필터 적용: 평균 대신 메디안 사용 (이상치 제거 효과)
        // center_distances가 비어있으면 캐시를 업데이트하지 않음 (마지막 유효한 값 유지)
        // 이렇게 하면 데이터가 일시적으로 없을 때도 깜빡임이 발생하지 않음
        if (!center_distances.empty()) {
            // 현재 프레임의 평균 거리 계산
            float current_avg = 0.0f;
            for (float dist : center_distances) {
                current_avg += dist;
            }
            current_avg /= center_distances.size();
            
            // 메디안 필터 적용
            std::lock_guard<std::mutex> lock(distance_cache_mutex_);
            
            // 히스토리에 현재 값 추가
            distance_history_.push_back(current_avg);
            
            // 히스토리 크기 제한
            if (distance_history_.size() > MEDIAN_FILTER_SIZE) {
                distance_history_.erase(distance_history_.begin());
            }
            
            // 메디안 값 계산
            if (distance_history_.size() >= 3) {
                // 최소 3개 이상의 값이 있을 때만 메디안 계산
                std::vector<float> sorted_history = distance_history_;
                std::sort(sorted_history.begin(), sorted_history.end());
                
                size_t size = sorted_history.size();
                if (size % 2 == 0) {
                    // 짝수 개: 중간 두 값의 평균
                    cached_center_distance_ = (sorted_history[size / 2 - 1] + sorted_history[size / 2]) / 2.0f;
                } else {
                    // 홀수 개: 중간 값
                    cached_center_distance_ = sorted_history[size / 2];
                }
            } else {
                // 값이 부족하면 평균 사용
                float sum = 0.0f;
                for (float dist : distance_history_) {
                    sum += dist;
                }
                cached_center_distance_ = sum / distance_history_.size();
            }
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

void DistanceOverlay::setShowLidarPoints(bool show) {
    std::lock_guard<std::mutex> lock(display_mutex_);
    show_lidar_points_ = show;
}

void DistanceOverlay::setPointDecayRate(float decay) {
    std::lock_guard<std::mutex> lock(slam_params_mutex_);
    // 유효 범위 체크 (0.0~1.0)
    if (decay >= 0.0f && decay <= 1.0f) {
        confidence_decay_ = decay;
    }
}

void DistanceOverlay::setPointMinConfidence(float threshold) {
    std::lock_guard<std::mutex> lock(slam_params_mutex_);
    // 유효 범위 체크 (0.0~1.0)
    if (threshold >= 0.0f && threshold <= 1.0f) {
        min_confidence_ = threshold;
    }
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
    const int MINIMAP_SIZE = 100;  // 100x100 픽셀
    const int MINIMAP_RADIUS = MINIMAP_SIZE / 2;  // 미니맵 반경 (50픽셀)
    const int MINIMAP_MARGIN = 10;  // 프레임 가장자리 여백
    const float BACKGROUND_ALPHA = 0.7f;  // 배경 투명도

    // 미니맵 중심 위치 계산 (오른쪽 하단)
    int minimap_center_x = frame.cols - MINIMAP_MARGIN - MINIMAP_RADIUS;
    int minimap_center_y = frame.rows - MINIMAP_MARGIN - MINIMAP_RADIUS;

    // 미니맵 영역이 프레임을 벗어나지 않도록 체크
    if (minimap_center_x < MINIMAP_RADIUS || minimap_center_y < MINIMAP_RADIUS ||
        minimap_center_x + MINIMAP_RADIUS >= frame.cols ||
        minimap_center_y + MINIMAP_RADIUS >= frame.rows) {
        return;  // 프레임 범위를 벗어나면 그리지 않음
    }

    // 반투명 원형 배경 그리기 (먼저 그리기 - 테두리 아래 레이어)
    for (int y = minimap_center_y - MINIMAP_RADIUS; y <= minimap_center_y + MINIMAP_RADIUS; y++) {
        for (int x = minimap_center_x - MINIMAP_RADIUS; x <= minimap_center_x + MINIMAP_RADIUS; x++) {
            int dx = x - minimap_center_x;
            int dy = y - minimap_center_y;
            int dist_sq = dx * dx + dy * dy;
            // 원형 영역 내부인지 확인 (테두리 제외, 약간 안쪽)
            int border_thickness = 3;
            int inner_radius_sq = (MINIMAP_RADIUS - border_thickness) * (MINIMAP_RADIUS - border_thickness);
            if (dist_sq <= inner_radius_sq) {
                if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                    cv::Vec3b& pixel = frame.at<cv::Vec3b>(y, x);
                    // 검정색과 블렌딩 (반투명)
                    pixel[0] = static_cast<uchar>(pixel[0] * (1.0f - BACKGROUND_ALPHA));  // B
                    pixel[1] = static_cast<uchar>(pixel[1] * (1.0f - BACKGROUND_ALPHA));  // G
                    pixel[2] = static_cast<uchar>(pixel[2] * (1.0f - BACKGROUND_ALPHA));  // R
                }
            }
        }
    }

    // 미니맵 테두리 그리기 (흰색, 1픽셀) - 배경 위에 그리기 (명확하게 보이도록)
    // cv::circle의 thickness 파라미터는 선 두께 (양수면 외곽선, 음수면 채움)
    cv::circle(frame, cv::Point(minimap_center_x, minimap_center_y), MINIMAP_RADIUS,
              cv::Scalar(138, 138, 138), 1, cv::LINE_AA);
    
    // 미니맵 내부에 작은 원 그리기 (중심 원)
    // int inner_circle_radius = MINIMAP_RADIUS / 3;  // 외부 원의 1/3 크기
    // cv::circle(frame, cv::Point(minimap_center_x, minimap_center_y), inner_circle_radius,
    //           cv::Scalar(138, 138, 138), 1, cv::LINE_AA);

    // 전방 거리 값 가져오기 (필요 시 사용)
    float center_distance = -1.0f;
    {
        std::lock_guard<std::mutex> lock(distance_cache_mutex_);
        center_distance = cached_center_distance_;
    }

    // 부채꼴 그리기 제거 (필요 시 주석 해제)
    // drawDistanceSectors(frame, points, angle_offset, minimap_center_x, minimap_center_y, MINIMAP_RADIUS, center_distance);

    // LiDAR 포인트 그리기 (setShowLidarPoints()로 제어 가능)
    bool show_points = false;
    {
        std::lock_guard<std::mutex> lock(display_mutex_);
        show_points = show_lidar_points_;
    }

    if (show_points) {
        const float PI = 3.14159265358979323846f;
        const float MAX_RANGE = LIDAR_MAX_RANGE;
        for (const auto& point : points) {
            float adjusted_angle = point.angle + angle_offset;
            while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
            while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;

            if (point.distance < 0.05f || point.distance > 12.0f) continue;

            float normalized_dist = std::min(point.distance / MAX_RANGE, 1.0f);
            float pixel_radius_float = normalized_dist * MINIMAP_RADIUS;
            float angle_rad = (adjusted_angle - 90.0f) * PI / 180.0f;

            int x = minimap_center_x + static_cast<int>(std::round(pixel_radius_float * std::cos(angle_rad)));
            int y = minimap_center_y + static_cast<int>(std::round(pixel_radius_float * std::sin(angle_rad)));

            int dx = x - minimap_center_x;
            int dy = y - minimap_center_y;
            if (dx * dx + dy * dy <= MINIMAP_RADIUS * MINIMAP_RADIUS) {
                if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                    cv::Scalar color = getLidarColor(point.distance);
                    frame.at<cv::Vec3b>(y, x) = cv::Vec3b(
                        static_cast<uchar>(color[0]),
                        static_cast<uchar>(color[1]),
                        static_cast<uchar>(color[2])
                    );
                }
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
        // 소수점 아래 2자리로 포맷팅
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << center_distance;
        std::string dist_text = oss.str() + "m";
        cv::Scalar text_color = getLidarColor(center_distance);
        
        // 텍스트 크기 및 위치 설정
        double font_scale = 1.2;  // 글자 크기 증가 (0.7 → 1.2)
        int thickness = 2;
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(dist_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
        
        // 하단 중앙 위치 계산
        int text_x = center_x - text_size.width / 2;
        int text_y = frame.rows - 20;  // 하단 여백 20픽셀 (베이스라인 위치)
        
        // 텍스트 위치를 5픽셀 위로 조정
        text_y -= 5;
        
        // 진회색 배경 그리기 (텍스트 주변에 패딩 추가 - 배경 크기 증가)
        int padding = 15;  // 10 → 15로 증가
        // 텍스트의 실제 위치: text_y는 베이스라인, 텍스트는 text_y - text_size.height부터 시작
        int bg_x = text_x - padding;
        int bg_y = text_y - text_size.height - baseline - padding;  // 텍스트 상단부터 배경 시작
        int bg_width = text_size.width + padding * 2;
        int bg_height = text_size.height + baseline + padding * 2;  // 텍스트 높이 + baseline + 패딩
        
        // 배경 영역이 프레임 범위 내인지 확인 및 조정
        if (bg_x < 0) {
            bg_width += bg_x;  // 너비 조정
            bg_x = 0;
        }
        if (bg_y < 0) {
            bg_height += bg_y;  // 높이 조정
            bg_y = 0;
        }
        if (bg_x + bg_width > frame.cols) {
            bg_width = frame.cols - bg_x;
        }
        if (bg_y + bg_height > frame.rows) {
            bg_height = frame.rows - bg_y;
        }
        
        // 배경 그리기 (텍스트보다 먼저 그려야 함) - 둥근 모서리
        if (bg_width > 0 && bg_height > 0) {
            // 진회색 배경 (BGR: 34, 34, 34)
            cv::Scalar dark_gray(34, 34, 34);
            int corner_radius = 3;  // 둥근 모서리 반경
            
            // 둥근 모서리 사각형 그리기
            // OpenCV 4.5.0+ 버전에서는 cv::RoundedRectangle 사용 가능
            // 호환성을 위해 직접 그리기
            cv::Rect bg_rect(bg_x, bg_y, bg_width, bg_height);
            
            // 중앙 사각형 영역 채우기
            cv::Rect inner_rect(bg_x + corner_radius, bg_y, 
                               bg_width - corner_radius * 2, bg_height);
            cv::rectangle(frame, inner_rect, dark_gray, -1);
            
            // 상단/하단 가로 영역 채우기
            cv::Rect top_rect(bg_x, bg_y + corner_radius, 
                             bg_width, bg_height - corner_radius * 2);
            cv::rectangle(frame, top_rect, dark_gray, -1);
            
            // 네 모서리에 원 그리기 (둥근 모서리 효과)
            cv::Point corners[4] = {
                cv::Point(bg_x + corner_radius, bg_y + corner_radius),  // 좌상
                cv::Point(bg_x + bg_width - corner_radius, bg_y + corner_radius),  // 우상
                cv::Point(bg_x + corner_radius, bg_y + bg_height - corner_radius),  // 좌하
                cv::Point(bg_x + bg_width - corner_radius, bg_y + bg_height - corner_radius)  // 우하
            };
            
            for (int i = 0; i < 4; i++) {
                cv::circle(frame, corners[i], corner_radius, dark_gray, -1);
            }
        }
        
        // 텍스트 그리기 (배경 위에, 고딕체)
        cv::putText(frame, dist_text, cv::Point(text_x, text_y),
                   cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness, cv::LINE_AA);
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

// 부채꼴 그리기 함수 (필요 시 복원 가능)
/*
void DistanceOverlay::drawDistanceSectors(cv::Mat& frame, const std::vector<LidarPoint>& points,
                                          float angle_offset, int center_x, int center_y, int radius, float center_distance) {
    const float PI = 3.14159265358979323846f;
    const float MAX_RANGE = LIDAR_MAX_RANGE;  // 12m
    
    // 5단계 거리 구간 정의
    // Level 0: 0-5m (빨간색)
    // Level 1: 5-7.5m (주황색)
    // Level 2: 7.5-9.8m (노란색)
    // Level 3: 9.8-10.2m (녹색) - 타겟 거리 확보 지점 (가장 중요!)
    // Level 4: 10.2-12m (노란색)
    const float SECTOR_RANGES[6] = {0.0f, 5.0f, 7.5f, 9.8f, 10.2f, 12.0f};
    const int NUM_SECTORS = 5;
    
    // 전방 거리 값이 유효하지 않으면 기본값 사용 (항상 표시)
    if (center_distance < 0.05f || center_distance > 12.0f) {
        center_distance = 10.0f;  // 기본값 (Level 3 - 녹색)
    }
    
    // 전방 거리 값에 해당하는 거리 구간 결정
    int target_level = -1;
    for (int level = 0; level < NUM_SECTORS; level++) {
        if (center_distance >= SECTOR_RANGES[level] && center_distance < SECTOR_RANGES[level + 1]) {
            target_level = level;
            break;
        }
    }
    
    if (target_level < 0) {
        target_level = 3;  // 기본값: Level 3 (녹색)
    }
    
    // 해당 구간의 색상 가져오기
    cv::Scalar sector_color = getSectorColor(target_level);
    
    // 거리 범위에 따른 반경 계산 (전방 거리 값에 해당하는 구간의 반경)
    float min_dist = SECTOR_RANGES[target_level];
    float max_dist = SECTOR_RANGES[target_level + 1];
    
    int inner_radius = static_cast<int>((min_dist / MAX_RANGE) * radius);
    int outer_radius = static_cast<int>((max_dist / MAX_RANGE) * radius);
    
    // 최소 반경 보장 (너무 작으면 보이지 않음)
    if (inner_radius == outer_radius && inner_radius > 0) {
        outer_radius = inner_radius + 1;
    }
    if (outer_radius <= 0) {
        inner_radius = 5;
        outer_radius = radius / 2;  // 최소 반경 보장
    }
    
    // 전방 방향에 부채꼴 그리기 (0도 근처, ±45도 범위)
    const float FRONT_ANGLE_RANGE = 45.0f;  // 전방 각도 범위 (±45도)

    // 부채꼴 영역 채우기 (픽셀 단위로 직접 그리기)
    for (int y = center_y - outer_radius; y <= center_y + outer_radius; y++) {
        for (int x = center_x - outer_radius; x <= center_x + outer_radius; x++) {
            int dx = x - center_x;
            int dy = y - center_y;
            int dist_sq = dx * dx + dy * dy;

            // 원형 영역 내부인지 확인 (내부 반경 ~ 외부 반경)
            if (dist_sq >= inner_radius * inner_radius && dist_sq <= outer_radius * outer_radius) {
                // 각도 계산 (화면 좌표계: 0도 = 위쪽)
                float angle_rad = std::atan2(dy, dx);
                float angle_deg = angle_rad * 180.0f / PI;

                // 화면 좌표계 변환: 0도 = 위쪽
                float screen_angle = angle_deg - 90.0f;
                if (screen_angle < 0.0f) screen_angle += 360.0f;
                if (screen_angle >= 360.0f) screen_angle -= 360.0f;

                // 전방 방향 범위 체크 (±45도, 0도 = 위쪽)
                bool in_front_range = false;
                if (screen_angle <= FRONT_ANGLE_RANGE) {
                    in_front_range = true;  // 0~45도
                } else if (screen_angle >= (360.0f - FRONT_ANGLE_RANGE)) {
                    in_front_range = true;  // 315~360도 (0도 근처)
                }

                if (in_front_range && x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                    // 알파 블렌딩 (깔끔하고 선명하게)
                    float alpha = 0.75f;  // 부채꼴 투명도 (더 선명하게)
                    cv::Vec3b& pixel = frame.at<cv::Vec3b>(y, x);
                    pixel[0] = static_cast<uchar>(pixel[0] * (1.0f - alpha) + sector_color[0] * alpha);  // B
                    pixel[1] = static_cast<uchar>(pixel[1] * (1.0f - alpha) + sector_color[1] * alpha);  // G
                    pixel[2] = static_cast<uchar>(pixel[2] * (1.0f - alpha) + sector_color[2] * alpha);  // R
                }
            }
        }
    }

    // 부채꼴 테두리 그리기 (깔끔하고 얇게)
    // 외부 호 그리기 (부채꼴 외곽선)
    cv::ellipse(frame, cv::Point(center_x, center_y),
               cv::Size(outer_radius, outer_radius), 0,
               -90.0f - FRONT_ANGLE_RANGE, -90.0f + FRONT_ANGLE_RANGE,
               sector_color, 2, cv::LINE_AA);

    // 내부 호 그리기 (부채꼴 내곽선)
    if (inner_radius > 5) {  // 내부 반경이 충분히 클 때만 그리기
        cv::ellipse(frame, cv::Point(center_x, center_y),
                   cv::Size(inner_radius, inner_radius), 0,
                   -90.0f - FRONT_ANGLE_RANGE, -90.0f + FRONT_ANGLE_RANGE,
                   sector_color, 2, cv::LINE_AA);
    }

    // 부채꼴 양쪽 경계선 (±45도)
    for (int side = -1; side <= 1; side += 2) {
        float border_angle = side * FRONT_ANGLE_RANGE;
        float border_rad = (border_angle - 90.0f) * PI / 180.0f;
        int bx1 = center_x + static_cast<int>(inner_radius * std::cos(border_rad));
        int by1 = center_y + static_cast<int>(inner_radius * std::sin(border_rad));
        int bx2 = center_x + static_cast<int>(outer_radius * std::cos(border_rad));
        int by2 = center_y + static_cast<int>(outer_radius * std::sin(border_rad));
        if (bx1 >= 0 && bx1 < frame.cols && by1 >= 0 && by1 < frame.rows &&
            bx2 >= 0 && bx2 < frame.cols && by2 >= 0 && by2 < frame.rows) {
            cv::line(frame, cv::Point(bx1, by1), cv::Point(bx2, by2), sector_color, 2, cv::LINE_AA);
        }
    }
}

cv::Scalar DistanceOverlay::getSectorColor(int level) {
    // 5단계 색상 정의 (BGR 형식)
    switch (level) {
        case 0:  // 0-5m: 빨간색
            return cv::Scalar(0, 0, 255);
        case 1:  // 5-7.5m: 주황색
            return cv::Scalar(0, 165, 255);
        case 2:  // 7.5-9.8m: 노란색
            return cv::Scalar(0, 255, 255);
        case 3:  // 9.8-10.2m: 녹색 (타겟 거리 확보 지점 - 가장 중요!)
            return cv::Scalar(0, 255, 0);
        case 4:  // 10.2-12m: 노란색
            return cv::Scalar(0, 255, 255);
        default:
            return cv::Scalar(255, 255, 255);  // 흰색 (기본값)
    }
}
*/

