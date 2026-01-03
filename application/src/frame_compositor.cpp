#include "frame_compositor.h"
#include "status_overlay.h"
#include "thermal_overlay.h"
#include "targeting_frame_compositor.h"
#include "camera_manager.h"
#include "config.h"
#include <iostream>

FrameCompositor::FrameCompositor(
    StatusOverlay* status_overlay,
    ThermalOverlay* thermal_overlay,
    TargetingFrameCompositor* targeting_compositor,
    CameraManager* camera_manager
) : status_overlay_(status_overlay),
     thermal_overlay_(thermal_overlay),
     targeting_compositor_(targeting_compositor),
     camera_manager_(camera_manager) {
}

cv::Mat FrameCompositor::compose(
    const cv::Mat& rgb_frame,
    const ThermalData& thermal_data,
    bool has_rgb,
    bool has_thermal
) {
    // 출력 프레임 준비 (검은 배경으로 시작)
    cv::Mat output = cv::Mat(OUTPUT_HEIGHT, OUTPUT_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    
    // 우선순위 1: OSD 먼저 그리기 (카메라 상태와 무관하게 항상 표시)
    if (status_overlay_) {
        status_overlay_->draw(output);
    }
    
    // 카메라 상태 확인
    bool rgb_ready = camera_manager_ && camera_manager_->is_rgb_ready();
    bool thermal_ready = camera_manager_ && camera_manager_->is_thermal_ready();
    
    // 우선순위 2: RGB 카메라 (RGB 프레임이 있으면 배경으로 사용)
    if (has_rgb && !rgb_frame.empty()) {
        if (rgb_frame.rows != OUTPUT_HEIGHT || rgb_frame.cols != OUTPUT_WIDTH) {
            cv::resize(rgb_frame, output, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
        } else {
            output = rgb_frame.clone();
        }
        // OSD 다시 그리기 (프레임 위에)
        if (status_overlay_) {
            status_overlay_->draw(output);
        }
    } else if (has_thermal && !thermal_data.frame.empty()) {
        // 우선순위 3: 열화상 카메라 (RGB가 없을 때만 배경으로 사용)
        cv::Mat thermal_resized;
        cv::resize(thermal_data.frame, thermal_resized, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
        // 열화상은 그레이스케일이므로 BGR로 변환
        if (thermal_resized.channels() == 1) {
            cv::cvtColor(thermal_resized, output, cv::COLOR_GRAY2BGR);
        } else {
            output = thermal_resized.clone();
        }
        // OSD 다시 그리기 (프레임 위에)
        if (status_overlay_) {
            status_overlay_->draw(output);
        }
    }
    // 둘 다 없으면 검은 배경 유지 (이미 OSD는 그려짐)
    
    // 열화상 오버레이 (열화상 데이터가 있고 RGB가 있을 때만)
    if (has_rgb && has_thermal && OVERLAY_THERMAL && thermal_overlay_) {
        thermal_overlay_->overlayThermal(output, thermal_data.frame);
    }
    
    // 우선순위 4: 타겟팅 오버레이 (조준, 라이다, hotspot)
    // 열화상 데이터가 없어도 라이다는 표시 가능
    if (targeting_compositor_) {
        targeting_compositor_->compositeTargeting(output, thermal_data);
    }
    
    // 우선순위 5: 카메라 상태 메시지 (왼쪽 하단에 표시)
    drawCameraStatus(output, rgb_ready, thermal_ready, has_rgb, has_thermal);
    
    return output;
}

void FrameCompositor::drawCameraStatus(
    cv::Mat& frame,
    bool rgb_ready,
    bool thermal_ready,
    bool has_rgb,
    bool has_thermal
) {
    if (frame.empty()) return;
    
    std::string rgb_status, thermal_status;
    
    if (!rgb_ready) {
        if (camera_manager_) {
            rgb_status = "Not Connected";
        } else {
            rgb_status = "Initializing...";
        }
    } else if (!has_rgb) {
        rgb_status = "Loading...";
    }
    
    if (!thermal_ready) {
        if (camera_manager_) {
            thermal_status = "Not Connected";
        } else {
            thermal_status = "Initializing...";
        }
    } else if (!has_thermal) {
        thermal_status = "Loading...";
    }
    
    if (rgb_status.empty() && thermal_status.empty()) {
        return;  // 상태 메시지가 없으면 표시하지 않음
    }
    
    const int FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
    const double FONT_SCALE = 0.5;
    const int FONT_THICKNESS = 1;
    const cv::Scalar TEXT_COLOR(255, 255, 255);  // 흰색
    const cv::Scalar BG_COLOR(0, 0, 0);  // 검은색 배경
    const int PADDING = 8;
    const int LINE_SPACING = 4;
    const int MARGIN_LEFT = 10;
    const int MARGIN_BOTTOM = 10;
    
    std::vector<std::string> lines;
    if (!rgb_status.empty()) {
        lines.push_back("RGB: " + rgb_status);
    }
    if (!thermal_status.empty()) {
        lines.push_back("Thermal: " + thermal_status);
    }
    
    if (lines.empty()) return;
    
    // 텍스트 크기 계산
    int max_width = 0;
    int total_height = 0;
    std::vector<cv::Size> text_sizes;
    for (const auto& line : lines) {
        int baseline = 0;
        cv::Size size = cv::getTextSize(line, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
        text_sizes.push_back(size);
        max_width = std::max(max_width, size.width);
        total_height += size.height + LINE_SPACING;
    }
    total_height -= LINE_SPACING;  // 마지막 줄 간격 제거
    
    // 배경 그리기
    int bg_x = MARGIN_LEFT;
    int bg_y = frame.rows - MARGIN_BOTTOM - total_height - PADDING * 2;
    int bg_width = max_width + PADDING * 2;
    int bg_height = total_height + PADDING * 2;
    
    cv::rectangle(frame, 
                  cv::Point(bg_x, bg_y),
                  cv::Point(bg_x + bg_width, bg_y + bg_height),
                  BG_COLOR, -1);
    
    // 텍스트 그리기
    int y = bg_y + PADDING;
    for (size_t i = 0; i < lines.size(); ++i) {
        int baseline = 0;
        cv::Size size = text_sizes[i];
        y += size.height;
        cv::putText(frame, lines[i],
                    cv::Point(bg_x + PADDING, y),
                    FONT_FACE, FONT_SCALE,
                    TEXT_COLOR, FONT_THICKNESS, cv::LINE_AA);
        y += LINE_SPACING;
    }
}

