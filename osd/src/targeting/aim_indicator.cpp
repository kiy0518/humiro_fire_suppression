#include "aim_indicator.h"
#include "../../thermal/src/config.h"
#include <string>

AimIndicator::AimIndicator() {
}

AimIndicator::~AimIndicator() {
}

void AimIndicator::drawAimMarker(cv::Mat& frame, const ThermalData& data) {
    int center_x = data.center_x;
    int center_y = data.center_y;
    int max_x = data.hotspot_x;
    int max_y = data.hotspot_y;
    
    cv::Vec3b color = data.color;
    cv::Vec3b max_color = data.max_color;
    
    // 타겟팅 마커 그리기
    cv::circle(frame, cv::Point(center_x, center_y), 10, cv::Scalar(color[0], color[1], color[2]), 1);
    cv::circle(frame, cv::Point(center_x, center_y), RADIUS_OUTER, cv::Scalar(color[0], color[1], color[2]), 1);
    cv::circle(frame, cv::Point(center_x, center_y), RADIUS_INNER, cv::Scalar(color[0], color[1], color[2]), 1);
    
    int extend_radius = RADIUS_OUTER + 20;
    cv::line(frame, cv::Point(center_x + 10, center_y), 
             cv::Point(center_x + extend_radius, center_y), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    cv::line(frame, cv::Point(center_x - extend_radius, center_y), 
             cv::Point(center_x - 10, center_y), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    cv::line(frame, cv::Point(center_x, center_y + 10), 
             cv::Point(center_x, center_y + extend_radius), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    cv::line(frame, cv::Point(center_x, center_y - extend_radius), 
             cv::Point(center_x, center_y - 10), 
             cv::Scalar(color[0], color[1], color[2]), 2);
    
    // 화점 마커
    cv::circle(frame, cv::Point(max_x, max_y), 20, 
               cv::Scalar(max_color[0], max_color[1], max_color[2]), 2);
}

void AimIndicator::drawAimInfo(cv::Mat& frame, const ThermalData& data) {
    // 열화상 디버그 정보 제거 (StatusOverlay로 대체됨)
    // 이 함수는 빈 함수로 유지 (호출은 하지만 아무것도 그리지 않음)
}

