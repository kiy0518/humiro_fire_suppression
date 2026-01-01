#ifndef THERMAL_OVERLAY_H
#define THERMAL_OVERLAY_H

#include <opencv2/opencv.hpp>

/**
 * 열화상 기본 오버레이 클래스
 * 열화상 레이어와 로고만 처리 (타겟팅 관련 기능 없음)
 */
class ThermalOverlay {
public:
    ThermalOverlay();
    ~ThermalOverlay();
    
    /**
     * 열화상 레이어 오버레이
     * @param rgb_frame RGB 프레임 (수정됨)
     * @param thermal_frame 열화상 프레임
     */
    void overlayThermal(cv::Mat& rgb_frame, const cv::Mat& thermal_frame);
    
    /**
     * 로고 오버레이
     * @param frame 출력 프레임 (수정됨)
     */
    void overlayLogo(cv::Mat& frame);
    
private:
    cv::Mat create_gradient_mask(int width, int height);
    float apply_curve(float value);
    
    cv::Mat logo_image_;  // 로고 이미지 (캐시)
    bool logo_loaded_;    // 로고 로드 여부
};

#endif // THERMAL_OVERLAY_H

