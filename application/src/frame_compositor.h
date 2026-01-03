#ifndef FRAME_COMPOSITOR_H
#define FRAME_COMPOSITOR_H

#include <opencv2/opencv.hpp>
#include "thermal_data.h"
#include "camera_manager.h"

// Forward declarations
class StatusOverlay;
class ThermalOverlay;
class TargetingFrameCompositor;

/**
 * @brief 프레임 합성 클래스
 * 
 * OSD, RGB, 열화상, 타겟팅 오버레이를 합성합니다.
 * 스트리밍 룰에 따라 우선순위를 적용합니다.
 */
class FrameCompositor {
public:
    FrameCompositor(
        StatusOverlay* status_overlay,
        ThermalOverlay* thermal_overlay,
        TargetingFrameCompositor* targeting_compositor,
        CameraManager* camera_manager
    );
    
    /**
     * @brief 프레임 합성
     * 
     * @param rgb_frame RGB 프레임 (있으면 사용)
     * @param thermal_data 열화상 데이터 (있으면 사용)
     * @param has_rgb RGB 프레임 존재 여부
     * @param has_thermal 열화상 데이터 존재 여부
     * @return 합성된 프레임 (BGR 형식)
     */
    cv::Mat compose(
        const cv::Mat& rgb_frame,
        const ThermalData& thermal_data,
        bool has_rgb,
        bool has_thermal
    );
    
private:
    StatusOverlay* status_overlay_;
    ThermalOverlay* thermal_overlay_;
    TargetingFrameCompositor* targeting_compositor_;
    CameraManager* camera_manager_;
    
    /**
     * @brief 카메라 상태 메시지 그리기
     * 
     * @param frame 출력 프레임
     * @param rgb_ready RGB 카메라 준비 여부
     * @param thermal_ready 열화상 카메라 준비 여부
     * @param has_rgb RGB 프레임 존재 여부
     * @param has_thermal 열화상 데이터 존재 여부
     */
    void drawCameraStatus(
        cv::Mat& frame,
        bool rgb_ready,
        bool thermal_ready,
        bool has_rgb,
        bool has_thermal
    );
};

#endif // FRAME_COMPOSITOR_H

