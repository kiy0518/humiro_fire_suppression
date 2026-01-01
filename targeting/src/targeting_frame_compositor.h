#ifndef TARGETING_FRAME_COMPOSITOR_H
#define TARGETING_FRAME_COMPOSITOR_H

#include <opencv2/opencv.hpp>
#include "../../thermal/src/thermal_data.h"
#include <vector>
#include <mutex>

// Forward declarations
struct LidarPoint;

// 전방 선언
class DistanceOverlay;
class AimIndicator;
class HotspotTracker;

/**
 * 타겟팅 프레임 합성 클래스
 * 모든 타겟팅 정보를 프레임에 합성
 */
class TargetingFrameCompositor {
public:
    TargetingFrameCompositor();
    ~TargetingFrameCompositor();
    
    /**
     * 타겟팅 정보를 프레임에 합성
     * @param frame 원본 프레임 (수정됨)
     * @param thermal_data 열화상 데이터
     */
    void compositeTargeting(cv::Mat& frame, const ThermalData& thermal_data);
    
    /**
     * 라이다 데이터 설정 (스레드 안전)
     */
    void setLidarData(const std::vector<LidarPoint>& lidar_points);
    
    /**
     * 라이다 오리엔테이션 설정
     */
    void setLidarOrientation(float offset_degrees);
    
    
private:
    DistanceOverlay* distance_overlay_;
    AimIndicator* aim_indicator_;
    HotspotTracker* hotspot_tracker_;
};

#endif // TARGETING_FRAME_COMPOSITOR_H

