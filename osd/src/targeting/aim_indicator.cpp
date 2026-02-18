#include "aim_indicator.h"
#include "../../thermal/src/config.h"
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

AimIndicator::AimIndicator() {
}

AimIndicator::~AimIndicator() {
}

void AimIndicator::drawAimMarker(cv::Mat& frame, const ThermalData& data) {
    // 십자선/원은 항상 화면 정중앙
    int center_x = frame.cols / 2;
    int center_y = frame.rows / 2;
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
    if (!data.valid) return;

    // 최고점 마커 옆에 온도 표시
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << data.max_temp_celsius << "C";

    int text_x = data.hotspot_x + 25;
    int text_y = data.hotspot_y - 5;

    // 화면 경계 체크 (오른쪽 넘어가면 왼쪽에 표시)
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(oss.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    if (text_x + text_size.width > frame.cols) {
        text_x = data.hotspot_x - text_size.width - 25;
    }
    if (text_y < text_size.height) {
        text_y = data.hotspot_y + 25;
    }

    // 배경 (라운드 엣지, fillPoly + LINE_AA)
    {
        cv::Scalar bg_color(0, 0, 0);
        const int pad = 3;
        const int r = 6;
        const int step = 15;
        int bx = text_x - pad;
        int by = text_y - text_size.height - pad;
        int bw = text_size.width + pad * 2;
        int bh = text_size.height + pad * 2;

        std::vector<cv::Point> pts;
        for (int a = 180; a <= 270; a += step)
            pts.emplace_back(bx + r + (int)(r * std::cos(a * M_PI / 180)),
                             by + r + (int)(r * std::sin(a * M_PI / 180)));
        for (int a = 270; a <= 360; a += step)
            pts.emplace_back(bx + bw - r + (int)(r * std::cos(a * M_PI / 180)),
                             by + r + (int)(r * std::sin(a * M_PI / 180)));
        for (int a = 0; a <= 90; a += step)
            pts.emplace_back(bx + bw - r + (int)(r * std::cos(a * M_PI / 180)),
                             by + bh - r + (int)(r * std::sin(a * M_PI / 180)));
        for (int a = 90; a <= 180; a += step)
            pts.emplace_back(bx + r + (int)(r * std::cos(a * M_PI / 180)),
                             by + bh - r + (int)(r * std::sin(a * M_PI / 180)));

        std::vector<std::vector<cv::Point>> poly = {pts};
        cv::fillPoly(frame, poly, bg_color, cv::LINE_AA);
    }

    // 온도 텍스트 (최고점 색상과 동일)
    cv::Scalar text_color(data.max_color[0], data.max_color[1], data.max_color[2]);
    cv::putText(frame, oss.str(), cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1, cv::LINE_AA);
}

