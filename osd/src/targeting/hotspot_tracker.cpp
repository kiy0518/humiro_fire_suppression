#include "hotspot_tracker.h"
#include <chrono>
#include <algorithm>

HotspotTracker::HotspotTracker() {
}

HotspotTracker::~HotspotTracker() {
}

void HotspotTracker::trackAndDraw(cv::Mat& frame, const ThermalData& data) {
    if (!data.valid || data.hotspot_x < 0 || data.hotspot_y < 0) {
        return;
    }
    
    // 히스토리에 추가
    auto now = std::chrono::steady_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    HotspotHistory entry;
    entry.x = data.hotspot_x;
    entry.y = data.hotspot_y;
    entry.temperature = data.max_val;
    entry.timestamp = timestamp;
    
    history_.push_back(entry);
    
    // 히스토리 크기 제한
    if (history_.size() > MAX_HISTORY_SIZE) {
        history_.erase(history_.begin());
    }
    
    // 현재 Hotspot 표시
    cv::Vec3b max_color = data.max_color;
    cv::circle(frame, cv::Point(data.hotspot_x, data.hotspot_y), 20, 
               cv::Scalar(max_color[0], max_color[1], max_color[2]), 2);
    
    // 온도 텍스트 표시
    std::string temp_text = std::to_string(data.max_val);
    cv::putText(frame, temp_text, 
                cv::Point(data.hotspot_x + 25, data.hotspot_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(max_color[0], max_color[1], max_color[2]), 1);
}

void HotspotTracker::drawTrackingHistory(cv::Mat& frame) {
    // 추적 경로 선 그리기 제거 (사용자 요청)
    // 히스토리는 유지하되 화면에 선을 그리지 않음
    if (history_.size() < 2) {
        return;
    }
    
    // 선 그리기 코드 제거됨
    // 히스토리 데이터는 유지 (필요시 다른 용도로 사용 가능)
}

