#ifndef HOTSPOT_TRACKER_H
#define HOTSPOT_TRACKER_H

#include <opencv2/opencv.hpp>
#include "../../thermal/src/thermal_data.h"
#include <vector>

/**
 * Hotspot 추적 클래스
 * 열화상 데이터에서 Hotspot을 추적하고 표시
 */
class HotspotTracker {
public:
    HotspotTracker();
    ~HotspotTracker();
    
    /**
     * Hotspot 추적 및 표시
     * @param frame 출력 프레임
     * @param data 열화상 데이터
     */
    void trackAndDraw(cv::Mat& frame, const ThermalData& data);
    
    /**
     * 추적 히스토리 표시
     * @param frame 출력 프레임
     */
    void drawTrackingHistory(cv::Mat& frame);
    
private:
    struct HotspotHistory {
        int x;
        int y;
        float temperature;
        uint64_t timestamp;
    };
    
    std::vector<HotspotHistory> history_;
    static constexpr size_t MAX_HISTORY_SIZE = 30;  // 최근 30프레임 추적
};

#endif // HOTSPOT_TRACKER_H

