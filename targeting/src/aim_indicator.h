#ifndef AIM_INDICATOR_H
#define AIM_INDICATOR_H

#include <opencv2/opencv.hpp>
#include "../../thermal/src/thermal_data.h"

/**
 * 조준 표시 클래스
 * 타겟 위치에 조준 마커와 정보를 표시
 */
class AimIndicator {
public:
    AimIndicator();
    ~AimIndicator();
    
    /**
     * 조준 마커 그리기
     * @param frame 출력 프레임
     * @param data 열화상 데이터 (타겟 위치 포함)
     */
    void drawAimMarker(cv::Mat& frame, const ThermalData& data);
    
    /**
     * 조준 정보 텍스트 표시
     * @param frame 출력 프레임
     * @param data 열화상 데이터
     */
    void drawAimInfo(cv::Mat& frame, const ThermalData& data);
    
private:
    // 상수 (thermal/src/config.h에서 가져옴)
    static constexpr int RADIUS_OUTER = 100;
    static constexpr int RADIUS_INNER = 50;
    static constexpr float INFO_BACKGROUND_ALPHA = 0.7f;
};

#endif // AIM_INDICATOR_H

