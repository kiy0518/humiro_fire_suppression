#include "thermal_overlay.h"
#include "../../thermal/src/config.h"
#include <unistd.h>
#include <cmath>

ThermalOverlay::ThermalOverlay() : logo_loaded_(false) {
    // 로고 이미지 로드 (osd/src/ 폴더 기준)
    std::string logo_path = LOGO_PATH;
    // 상대 경로인 경우 osd/src/ 폴더 기준으로 변환
    if (logo_path[0] != '/') {
        // osd/src/ 폴더 경로 찾기 (현재 파일 위치 기준)
        char file_path[1024];
        ssize_t len = readlink("/proc/self/exe", file_path, sizeof(file_path) - 1);
        if (len != -1) {
            file_path[len] = '\0';
            std::string exe_dir = std::string(file_path);
            size_t last_slash = exe_dir.find_last_of('/');
            if (last_slash != std::string::npos) {
                // application/build/ -> application/ -> .. -> osd/src/
                logo_path = exe_dir.substr(0, last_slash) + "/../../osd/src/" + logo_path;
            }
        }
    }
    logo_image_ = cv::imread(logo_path, cv::IMREAD_UNCHANGED);
    if (!logo_image_.empty()) {
        // 로고 크기 조정
        if (logo_image_.cols > LOGO_WIDTH) {
            double scale = static_cast<double>(LOGO_WIDTH) / logo_image_.cols;
            int new_height = static_cast<int>(logo_image_.rows * scale);
            cv::resize(logo_image_, logo_image_, cv::Size(LOGO_WIDTH, new_height), 0, 0, cv::INTER_LINEAR);
        }
        logo_loaded_ = true;
        std::cout << "  ✓ 로고 이미지 로드: " << LOGO_PATH << " (" 
                  << logo_image_.cols << "x" << logo_image_.rows << ")" << std::endl;
    } else {
        std::cerr << "  ⚠ 로고 이미지 로드 실패: " << LOGO_PATH << std::endl;
    }
}

ThermalOverlay::~ThermalOverlay() {
}

void ThermalOverlay::overlayThermal(cv::Mat& rgb_frame, const cv::Mat& thermal_frame) {
    if (thermal_frame.empty()) {
        return;
    }
    
    try {
        int overlay_x = RGB_CROP_X;
        int overlay_y = RGB_CROP_Y;
        
        int thermal_h = thermal_frame.rows;
        int thermal_w = thermal_frame.cols;
        
        int overlay_w = std::min({THERMAL_WIDTH, OUTPUT_WIDTH - overlay_x, thermal_w});
        int overlay_h = std::min({THERMAL_CROPPED_HEIGHT, OUTPUT_HEIGHT - overlay_y, thermal_h});
        
        if (overlay_w <= 0 || overlay_h <= 0) {
            return;
        }
        
        cv::Mat thermal_resized;
        if (thermal_w != overlay_w || thermal_h != overlay_h) {
            cv::resize(thermal_frame, thermal_resized, cv::Size(overlay_w, overlay_h), 0, 0, cv::INTER_LINEAR);
        } else {
            thermal_resized = thermal_frame;
        }
        
        if (overlay_y + overlay_h <= rgb_frame.rows && overlay_x + overlay_w <= rgb_frame.cols) {
            cv::Rect roi_rect(overlay_x, overlay_y, overlay_w, overlay_h);
            cv::Mat roi = rgb_frame(roi_rect);
            
            if (thermal_resized.channels() == 3 && roi.channels() == 3) {
                // 크기 확인 및 필요시 리사이즈
                if (roi.size() != thermal_resized.size()) {
                    cv::resize(thermal_resized, thermal_resized, roi.size(), 0, 0, cv::INTER_LINEAR);
                }
                
                // 크기 재확인
                if (roi.rows == overlay_h && roi.cols == overlay_w &&
                    thermal_resized.rows == overlay_h && thermal_resized.cols == overlay_w) {
                    // 단순 투명도 블렌딩
                    cv::Mat roi_float, thermal_float;
                    roi.convertTo(roi_float, CV_32F);
                    thermal_resized.convertTo(thermal_float, CV_32F);
                    
                    // 크기 및 채널 재확인
                    if (roi_float.rows == overlay_h && roi_float.cols == overlay_w &&
                        thermal_float.rows == overlay_h && thermal_float.cols == overlay_w &&
                        roi_float.channels() == 3 && thermal_float.channels() == 3) {
                        // 단순 알파 블렌딩: roi * (1 - alpha) + thermal * alpha
                        float alpha = ALPHA_THERMAL_LAYER;
                        cv::Mat blended = roi_float * (1.0f - alpha) + thermal_float * alpha;
                        blended.convertTo(roi, CV_8U);
                    } else {
                        std::cerr << "  ⚠ 블렌딩 크기/채널 불일치:" << std::endl;
                        std::cerr << "     roi_float: " << roi_float.size() << " ch=" << roi_float.channels() << std::endl;
                        std::cerr << "     thermal_float: " << thermal_float.size() << " ch=" << thermal_float.channels() << std::endl;
                        std::cerr << "     overlay: " << overlay_w << "x" << overlay_h << std::endl;
                    }
                } else {
                    std::cerr << "  ⚠ 크기 불일치: ROI " << roi.size() 
                              << " vs Thermal " << thermal_resized.size()
                              << " vs overlay " << overlay_w << "x" << overlay_h << std::endl;
                }
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 열화상 오버레이 오류: " << e.what() << std::endl;
    }
}

void ThermalOverlay::overlayLogo(cv::Mat& frame) {
    if (!logo_loaded_ || logo_image_.empty()) {
        return;
    }
    
    try {
        int logo_h = logo_image_.rows;
        int logo_w = logo_image_.cols;
        int pos_x = LOGO_POS_X;  // 왼쪽 여백
        int pos_y = frame.rows - logo_h - 5;  // 하단 여백 (5픽셀)
        
        // 경계 체크
        if (pos_x < 0 || pos_y < 0 || 
            pos_x + logo_w > frame.cols || 
            pos_y + logo_h > frame.rows) {
            return;
        }
        
        // ROI 추출
        cv::Rect roi_rect(pos_x, pos_y, logo_w, logo_h);
        cv::Mat roi = frame(roi_rect);
        
        // 알파 채널이 있는 경우 (RGBA)
        if (logo_image_.channels() == 4) {
            // 알파 채널 분리
            std::vector<cv::Mat> logo_channels;
            cv::split(logo_image_, logo_channels);
            cv::Mat logo_bgr = cv::Mat(logo_h, logo_w, CV_8UC3);
            cv::Mat alpha = logo_channels[3];
            
            // BGR 채널만 추출
            std::vector<cv::Mat> bgr_channels = {logo_channels[0], logo_channels[1], logo_channels[2]};
            cv::merge(bgr_channels, logo_bgr);
            
            // 알파 블렌딩
            alpha.convertTo(alpha, CV_32F, 1.0 / 255.0);
            std::vector<cv::Mat> alpha_channels = {alpha, alpha, alpha};
            cv::Mat alpha_3d;
            cv::merge(alpha_channels, alpha_3d);
            
            cv::Mat roi_float, logo_float;
            roi.convertTo(roi_float, CV_32F);
            logo_bgr.convertTo(logo_float, CV_32F);
            
            cv::Mat blended = roi_float.mul(1.0f - alpha_3d) + logo_float.mul(alpha_3d);
            blended.convertTo(roi, CV_8U);
        } else if (logo_image_.channels() == 3) {
            // 알파 채널이 없는 경우 (BGR)
            logo_image_.copyTo(roi);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "  ⚠ 로고 오버레이 오류: " << e.what() << std::endl;
    }
}

cv::Mat ThermalOverlay::create_gradient_mask(int width, int height) {
    cv::Mat mask = cv::Mat::ones(height, width, CV_32F);
    
    int grad_size = std::min({GRADIENT_BORDER_SIZE, width / 4, height / 4});
    
    if (grad_size > 0) {
        // Python 버전과 동일한 로직: 각 방향에서 그라데이션 적용
        // 상단 그라데이션
        for (int y = 0; y < grad_size; ++y) {
            float alpha = apply_curve(static_cast<float>(y) / grad_size);
            for (int x = 0; x < width; ++x) {
                mask.at<float>(y, x) *= alpha;
            }
        }
        
        // 하단 그라데이션
        for (int y = height - grad_size; y < height; ++y) {
            float alpha = apply_curve(static_cast<float>(height - y) / grad_size);
            for (int x = 0; x < width; ++x) {
                mask.at<float>(y, x) *= alpha;
            }
        }
        
        // 좌측 그라데이션
        for (int x = 0; x < grad_size; ++x) {
            float alpha = apply_curve(static_cast<float>(x) / grad_size);
            for (int y = 0; y < height; ++y) {
                // 이미 상단/하단 그라데이션이 적용된 경우 곱하기
                mask.at<float>(y, x) *= alpha;
            }
        }
        
        // 우측 그라데이션
        for (int x = width - grad_size; x < width; ++x) {
            float alpha = apply_curve(static_cast<float>(width - x) / grad_size);
            for (int y = 0; y < height; ++y) {
                // 이미 상단/하단 그라데이션이 적용된 경우 곱하기
                mask.at<float>(y, x) *= alpha;
            }
        }
    }
    
    return mask;
}

float ThermalOverlay::apply_curve(float value) {
    if (GRADIENT_CURVE == 1.0f) {
        return value;  // 선형
    } else if (GRADIENT_CURVE > 1.0f) {
        return std::pow(value, GRADIENT_CURVE);  // 제곱
    } else {
        return std::pow(value, GRADIENT_CURVE);  // 제곱근
    }
}

