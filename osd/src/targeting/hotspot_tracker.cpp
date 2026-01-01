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
    if (history_.size() < 2) {
        return;
    }
    
    // 추적 경로 그리기 (최근 10개만)
    size_t start_idx = (history_.size() > 10) ? history_.size() - 10 : 0;
    
    for (size_t i = start_idx; i < history_.size() - 1; i++) {
        const auto& p1 = history_[i];
        const auto& p2 = history_[i + 1];
        
        // 시간에 따른 색상 변화 (최근일수록 밝게)
        float ratio = static_cast<float>(i - start_idx) / (history_.size() - start_idx - 1);
        int alpha = static_cast<int>(255 * (1.0f - ratio * 0.5f));  // 255 → 127
        
        cv::line(frame, 
                cv::Point(p1.x, p1.y), 
                cv::Point(p2.x, p2.y),
                cv::Scalar(0, alpha, alpha), 2, cv::LINE_AA);
    }
}

