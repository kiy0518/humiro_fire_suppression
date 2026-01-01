#ifndef DISTANCE_OVERLAY_H
#define DISTANCE_OVERLAY_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <map>
#include <chrono>
#include "../../lidar/src/lidar_interface.h"  // LidarPoint 완전한 정의 필요

/**
 * 거리 오버레이 클래스
 * 라이다 데이터를 기반으로 거리 정보를 화면에 표시
 */
class DistanceOverlay {
public:
    DistanceOverlay();
    ~DistanceOverlay();
    
    /**
     * 라이다 데이터 설정 (스레드 안전)
     */
    void setLidarData(const std::vector<LidarPoint>& lidar_points);
    
    /**
     * 라이다 오리엔테이션 설정 (각도 오프셋)
     */
    void setLidarOrientation(float offset_degrees);
    
    /**
     * 거리 오버레이 그리기
     * @param frame 출력 프레임
     */
    void drawOverlay(cv::Mat& frame);
    
private:
    cv::Scalar getLidarColor(float distance);
    void drawMinimap(cv::Mat& frame, const std::vector<LidarPoint>& points, float angle_offset);
    void drawCenterDistanceText(cv::Mat& frame, int center_x);
    
    // 라이다 데이터 (스레드 안전 접근)
    std::vector<LidarPoint> lidar_points_;
    std::mutex lidar_mutex_;
    
    // 라이다 오리엔테이션 (각도 오프셋)
    float lidar_angle_offset_;
    std::mutex orientation_mutex_;
    
    // 전방 거리 캐시 (깜빡임 방지용)
    float cached_center_distance_;
    std::mutex distance_cache_mutex_;
    
    // SLAM 방식 포인트 캐시 구조
    struct PointEntry {
        LidarPoint point;           // 포인트 데이터
        float confidence;           // 신뢰도 (0.0 ~ 1.0)
        std::chrono::steady_clock::time_point last_update;
    };
    
    // 각도 버킷(1도 단위)별로 포인트 엔트리 저장 (SLAM 방식)
    // int: 각도 버킷 (0~359)
    std::map<int, PointEntry> point_cache_;
    std::mutex point_cache_mutex_;
    
    // 마지막 신뢰도 업데이트 시간 (주기적 감소용)
    std::chrono::steady_clock::time_point last_confidence_update_;
    
    // SLAM 방식 파라미터
    static constexpr float SMOOTHING_ALPHA = 0.5f;        // Exponential smoothing 가중치 (0.0~1.0)
    static constexpr float CONFIDENCE_DECAY = 0.95f;      // 신뢰도 감소율 (매 업데이트 주기마다)
    static constexpr float MIN_CONFIDENCE = 0.1f;         // 최소 신뢰도 (이하일 때 제거)
    static constexpr float CHANGE_THRESHOLD = 0.5f;       // 변화 임계값 (m, 이보다 크면 노이즈로 간주)
    static constexpr int UPDATE_CYCLE_MS = 50;            // 신뢰도 감소 업데이트 주기 (ms)
    
    // 상수
    static constexpr float LIDAR_MAX_RANGE = 12.0f;  // 12m 최대 거리
    static constexpr float LIDAR_CENTER_ANGLE_RANGE = 4.0f;  // 중심 각도 범위 (±2도)
};

#endif // DISTANCE_OVERLAY_H

