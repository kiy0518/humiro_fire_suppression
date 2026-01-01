#include "targeting_frame_compositor.h"
#include "distance_overlay.h"
#include "aim_indicator.h"
#include "hotspot_tracker.h"
#include "../../lidar/src/lidar_interface.h"

TargetingFrameCompositor::TargetingFrameCompositor() {
    distance_overlay_ = new DistanceOverlay();
    aim_indicator_ = new AimIndicator();
    hotspot_tracker_ = new HotspotTracker();
}

TargetingFrameCompositor::~TargetingFrameCompositor() {
    delete distance_overlay_;
    delete aim_indicator_;
    delete hotspot_tracker_;
}

void TargetingFrameCompositor::compositeTargeting(cv::Mat& frame, const ThermalData& thermal_data) {
    if (frame.empty()) {
        return;
    }
    
    // 열화상 데이터가 유효하면 조준 표시
    if (thermal_data.valid && thermal_data.center_x >= 0 && thermal_data.center_y >= 0) {
        aim_indicator_->drawAimMarker(frame, thermal_data);
        aim_indicator_->drawAimInfo(frame, thermal_data);
    }
    
    // Hotspot 추적 및 표시
    if (thermal_data.valid) {
        hotspot_tracker_->trackAndDraw(frame, thermal_data);
        hotspot_tracker_->drawTrackingHistory(frame);
    }
    
    // 라이다 거리 오버레이
    distance_overlay_->drawOverlay(frame);
}

void TargetingFrameCompositor::setLidarData(const std::vector<LidarPoint>& lidar_points) {
    if (distance_overlay_) {
        distance_overlay_->setLidarData(lidar_points);
    }
}

void TargetingFrameCompositor::setLidarOrientation(float offset_degrees) {
    if (distance_overlay_) {
        distance_overlay_->setLidarOrientation(offset_degrees);
    }
}

void TargetingFrameCompositor::setLidarDisplayMode(const std::string& mode) {
    if (distance_overlay_) {
        distance_overlay_->setDisplayMode(mode);
    }
}

void TargetingFrameCompositor::setLidarShowDirectionLines(bool show) {
    if (distance_overlay_) {
        distance_overlay_->setShowDirectionLines(show);
    }
}

void TargetingFrameCompositor::setLidarThreePointTolerance(float tolerance) {
    if (distance_overlay_) {
        distance_overlay_->setThreePointTolerance(tolerance);
    }
}

