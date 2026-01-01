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
    int text_y = 20;
    int text_spacing = 25;
    
    // 전체 정보 영역의 배경 투명도 적용
    int info_start_y = text_y - 15;
    int info_end_y = text_y + text_spacing * 4 + 10;  // 5개 라인 (Offset, Max, Min, Dist, Status)
    int info_width = 200;
    int info_height = info_end_y - info_start_y;
    
    // 정보 배경 영역 ROI
    cv::Rect info_bg_rect(5, info_start_y, info_width, info_height);
    if (info_bg_rect.x >= 0 && info_bg_rect.y >= 0 && 
        info_bg_rect.x + info_bg_rect.width <= frame.cols &&
        info_bg_rect.y + info_bg_rect.height <= frame.rows) {
        
        cv::Mat roi = frame(info_bg_rect);
        cv::Mat roi_float;
        roi.convertTo(roi_float, CV_32F);
        
        // 검은색 배경과 블렌딩 (투명도 0.7)
        cv::Mat black_bg = cv::Mat::zeros(roi_float.size(), CV_32FC3);
        cv::Mat blended = roi_float * (1.0f - INFO_BACKGROUND_ALPHA) + 
                         black_bg * INFO_BACKGROUND_ALPHA;
        blended.convertTo(roi, CV_8U);
    }
    
    // 좌표 정보
    std::string offset_text = "Offset: (" + std::to_string(data.rel_x) + ", " + 
                              std::to_string(data.rel_y) + ")";
    cv::putText(frame, offset_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 온도값 정보
    text_y += text_spacing;
    std::string max_text = "Max: " + std::to_string(data.max_val);
    cv::putText(frame, max_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 최소값 정보
    text_y += text_spacing;
    std::string min_text = "Min: " + std::to_string(data.min_val);
    cv::putText(frame, min_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 거리 정보
    text_y += text_spacing;
    std::string dist_text = "Dist: " + std::to_string(data.distance) + "px";
    cv::putText(frame, dist_text, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // 상태 정보
    text_y += text_spacing;
    std::string status;
    cv::Scalar status_color;
    if (data.distance < RADIUS_INNER) {
        status = "INNER";
        status_color = cv::Scalar(0, 255, 0);
    } else if (data.distance < RADIUS_OUTER) {
        status = "OUTER";
        status_color = cv::Scalar(0, 255, 255);
    } else {
        status = "OUT";
        status_color = cv::Scalar(0, 0, 255);
    }
    cv::putText(frame, "Status: " + status, cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1);
}

