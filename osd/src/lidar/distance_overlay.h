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
     * LiDAR 포인트 표시 ON/OFF
     * @param show true: 포인트 표시, false: 포인트 숨김 (기본값)
     */
    void setShowLidarPoints(bool show);

    /**
     * 포인트 페이드아웃 속도 설정
     * @param decay 신뢰도 감소율 (0.0~1.0, 작을수록 빠르게 사라짐)
     *              기본값: 0.95 (느림), 권장: 0.85 (보통), 0.7 (빠름)
     */
    void setPointDecayRate(float decay);

    /**
     * 포인트 제거 임계값 설정
     * @param threshold 최소 신뢰도 (0.0~1.0, 클수록 빠르게 제거)
     *                  기본값: 0.1 (느림), 권장: 0.3 (보통), 0.5 (빠름)
     */
    void setPointMinConfidence(float threshold);

    /**
     * 거리 오버레이 그리기
     * @param frame 출력 프레임
     */
    void drawOverlay(cv::Mat& frame);
    
private:
    cv::Scalar getLidarColor(float distance);
    void drawMinimap(cv::Mat& frame, const std::vector<LidarPoint>& points, float angle_offset);
    void drawCenterDistanceText(cv::Mat& frame, int center_x);
    // 부채꼴 관련 함수 제거 (필요 시 복원 가능)
    // void drawDistanceSectors(cv::Mat& frame, const std::vector<LidarPoint>& points, float angle_offset,
    //                         int center_x, int center_y, int radius, float center_distance);
    // cv::Scalar getSectorColor(int level);
    
    // 라이다 데이터 (스레드 안전 접근)
    std::vector<LidarPoint> lidar_points_;
    std::mutex lidar_mutex_;
    
    // 라이다 오리엔테이션 (각도 오프셋)
    float lidar_angle_offset_;
    std::mutex orientation_mutex_;
    
    // 전방 거리 캐시 (깜빡임 방지용)
    float cached_center_distance_;
    std::mutex distance_cache_mutex_;
    
    // LiDAR 포인트 표시 플래그
    bool show_lidar_points_;
    std::mutex display_mutex_;
    
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

    // SLAM 방식 파라미터 (런타임 조정 가능)
    float smoothing_alpha_;       // Exponential smoothing 가중치 (0.0~1.0)
    float confidence_decay_;      // 신뢰도 감소율 (매 업데이트 주기마다)
    float min_confidence_;        // 최소 신뢰도 (이하일 때 제거)
    float change_threshold_;      // 변화 임계값 (m, 이보다 크면 노이즈로 간주)
    int update_cycle_ms_;         // 신뢰도 감소 업데이트 주기 (ms)
    std::mutex slam_params_mutex_;  // SLAM 파라미터 보호용 뮤텍스

    // 상수
    static constexpr float LIDAR_MAX_RANGE = 12.0f;  // 12m 최대 거리
    static constexpr float LIDAR_CENTER_ANGLE_RANGE = 4.0f;  // 중심 각도 범위 (±2도)
};

#endif // DISTANCE_OVERLAY_H
