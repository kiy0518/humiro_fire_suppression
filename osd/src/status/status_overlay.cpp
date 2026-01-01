#include "status_overlay.h"
#include "../../thermal/src/config.h"
#include <sstream>
#include <algorithm>

StatusOverlay::StatusOverlay()
    : current_status_(DroneStatus::IDLE)
    , px4_mode_("UNKNOWN")
    , is_armed_(false)
    , is_offboard_(false)
    , is_offboard_custom_status_(false)  // VIM4 커스텀 상태 사용 여부
    , ammo_current_(6)
    , ammo_max_(6)
    , drone_name_("1")
    , formation_current_(1)
    , formation_total_(3)  // 기본 삼각편대
    , battery_percentage_(-1)
    , gps_satellites_(-1)
    , show_battery_(false)
    , show_gps_(false)
{
}

StatusOverlay::~StatusOverlay() {
}

void StatusOverlay::updatePx4State(const std::string& px4_mode, bool is_armed) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 상태 변경 감지 (디버깅용)
    std::string old_mode = px4_mode_;
    bool old_armed = is_armed_;
    bool old_offboard = is_offboard_;
    
    px4_mode_ = px4_mode;
    is_armed_ = is_armed;
    bool was_offboard = is_offboard_;
    is_offboard_ = (px4_mode == "OFFBOARD");
    
    // 디버깅: 모드 변경 시 출력
    if (old_mode != px4_mode || old_armed != is_armed) {
        std::cout << "  [StatusOverlay::updatePx4State] 모드 변경: \"" << old_mode 
                  << "\" → \"" << px4_mode << "\", 시동: " << (old_armed ? "ON" : "OFF") 
                  << " → " << (is_armed ? "ON" : "OFF") << std::endl;
    }
    
    // OFFBOARD 모드가 아니면 PX4 모드로 상태 결정 (PX4 토픽 기반)
    if (!is_offboard_) {
        DroneStatus new_status = convertPx4ModeToStatus(px4_mode, is_armed);
        if (current_status_ != new_status || old_mode != px4_mode) {
            std::cout << "  [StatusOverlay] 상태 변경: " << static_cast<int>(current_status_) 
                      << " → " << static_cast<int>(new_status) << " (모드: \"" << px4_mode << "\")" << std::endl;
        }
        current_status_ = new_status;
        is_offboard_custom_status_ = false;  // 일반 모드에서는 VIM4 커스텀 상태 아님
    }
    // OFFBOARD 모드일 때: 공유 상태(시동, 이륙, 이동, 착륙, RTL 등)는 VIM4에서 /offboard/status로 전송
    // PX4의 nav_state는 OFFBOARD 모드일 때 항상 9(OFFBOARD)이므로 세부 상태를 알 수 없음
    // 따라서 VIM4 커스텀 상태가 없으면 기본 NAVIGATING 상태 사용
    else if (is_offboard_) {
        // VIM4 커스텀 상태가 없으면 기본 NAVIGATING 상태 유지
        // VIM4 커스텀 상태는 updateOffboardStatus()에서 처리됨
        if (!is_offboard_custom_status_) {
            // VIM4 커스텀 상태가 아직 수신되지 않았으면 기본 상태 유지
            // (이미 NAVIGATING으로 설정되어 있음)
        }
    }
}

void StatusOverlay::updateOffboardStatus(DroneStatus status) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // OFFBOARD 모드일 때만 VIM4 커스텀 상태 사용
    if (is_offboard_) {
        // VIM4에서 /offboard/status 토픽으로 상태 전송
        // 공유 상태(ARMING, TAKEOFF, NAVIGATING, LANDING, RETURNING, DISARMED)와
        // 특수 상태(DESTINATION_REACHED, FIRE_READY, FIRING_AUTO_TARGETING, AUTO_FIRING, MISSION_COMPLETE) 모두
        // VIM4 커스텀 상태로 처리 (PX4 상태보다 우선)
        current_status_ = status;
        is_offboard_custom_status_ = true;  // VIM4 커스텀 상태 사용 중
    }
    // OFFBOARD 모드가 아닐 때는 무시 (PX4 상태가 우선)
}

void StatusOverlay::setAmmunition(int current, int max) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ammo_current_ = current;
    ammo_max_ = max;
}

void StatusOverlay::setDroneName(const std::string& name) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    drone_name_ = name;
}

void StatusOverlay::setFormation(int current, int total) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    formation_current_ = current;
    formation_total_ = total;
}

void StatusOverlay::setBattery(int percentage) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    battery_percentage_ = percentage;
    show_battery_ = (percentage >= 0);
}

void StatusOverlay::setGpsSatellites(int count) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    gps_satellites_ = count;
    show_gps_ = (count >= 0);
}

StatusOverlay::DroneStatus StatusOverlay::convertPx4ModeToStatus(const std::string& px4_mode, bool is_armed) {
    if (!is_armed) {
        return DroneStatus::DISARMED;
    }
    
    // PX4 모드 문자열 파싱
    std::string mode_upper = px4_mode;
    std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), ::toupper);
    
    if (mode_upper.find("AUTO_TAKEOFF") != std::string::npos || 
        mode_upper.find("TAKEOFF") != std::string::npos) {
        return DroneStatus::TAKEOFF;
    }
    else if (mode_upper.find("AUTO_MISSION") != std::string::npos || 
             mode_upper.find("MISSION") != std::string::npos) {
        // Mission 모드에서는 waypoint 도착 여부를 추가로 확인해야 함
        // 여기서는 기본적으로 이동중으로 설정
        return DroneStatus::NAVIGATING;
    }
    else if (mode_upper.find("AUTO_RTL") != std::string::npos || 
             mode_upper.find("RTL") != std::string::npos) {
        return DroneStatus::RETURNING;
    }
    else if (mode_upper.find("AUTO_LAND") != std::string::npos || 
             mode_upper.find("LAND") != std::string::npos) {
        return DroneStatus::LANDING;
    }
    else if (mode_upper.find("AUTO_LOITER") != std::string::npos || 
             mode_upper.find("LOITER") != std::string::npos) {
        return DroneStatus::IDLE;  // 대기 (호버링)
    }
    else if (mode_upper.find("MANUAL") != std::string::npos ||
             mode_upper.find("STABILIZED") != std::string::npos ||
             mode_upper.find("POSCTL") != std::string::npos) {
        return DroneStatus::IDLE;  // 수동 비행 모드
    }
    else if (mode_upper.find("OFFBOARD") != std::string::npos) {
        // OFFBOARD 모드는 updateOffboardStatus()에서 처리
        return DroneStatus::IDLE;
    }
    
    // 기본값
    return is_armed ? DroneStatus::IDLE : DroneStatus::DISARMED;
}

cv::Scalar StatusOverlay::getStatusColor(DroneStatus status) {
    switch (status) {
        case DroneStatus::IDLE:
        case DroneStatus::DISARMED:
            return cv::Scalar(128, 128, 128);  // 회색
        case DroneStatus::ARMING:
        case DroneStatus::TAKEOFF:
        case DroneStatus::LANDING:
            return cv::Scalar(0, 255, 255);  // 노란색
        case DroneStatus::NAVIGATING:
        case DroneStatus::RETURNING:
            return cv::Scalar(255, 0, 0);  // 파란색
        case DroneStatus::DESTINATION_REACHED:
        case DroneStatus::FIRE_READY:
            return cv::Scalar(0, 255, 0);  // 초록색
        case DroneStatus::FIRING_AUTO_TARGETING:
        case DroneStatus::AUTO_FIRING:
            return cv::Scalar(0, 0, 255);  // 빨간색
        case DroneStatus::MISSION_COMPLETE:
            return cv::Scalar(255, 255, 0);  // 하늘색
        default:
            return cv::Scalar(255, 255, 255);  // 흰색
    }
}

std::string StatusOverlay::getStatusText(DroneStatus status) {
    switch (status) {
        case DroneStatus::IDLE:
            return "IDLE";
        case DroneStatus::ARMING:
            return "ARMING";
        case DroneStatus::TAKEOFF:
            return "TAKEOFF";
        case DroneStatus::NAVIGATING:
            return "NAVIGATING";
        case DroneStatus::DESTINATION_REACHED:
            return "DEST_REACHED";
        case DroneStatus::FIRE_READY:
            return "FIRE_READY";
        case DroneStatus::FIRING_AUTO_TARGETING:
            return "FIRING";
        case DroneStatus::AUTO_FIRING:
            return "AUTO_FIRING";
        case DroneStatus::MISSION_COMPLETE:
            return "MISSION_COMPLETE";
        case DroneStatus::RETURNING:
            return "RETURNING";
        case DroneStatus::LANDING:
            return "LANDING";
        case DroneStatus::DISARMED:
            return "DISARMED";
        default:
            return "UNKNOWN";
    }
}

void StatusOverlay::drawBackground(cv::Mat& frame, int x, int y, int width, int height) {
    const int corner_radius = 5;
    cv::Scalar bg_color(0, 0, 0);  // 검은색
    int alpha = 180;  // 반투명 (0-255)
    
    // 중앙 사각형 영역
    cv::Rect center_rect(x + corner_radius, y, width - 2 * corner_radius, height);
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay, center_rect, bg_color, -1);
    cv::addWeighted(overlay, alpha / 255.0, frame, 1.0 - alpha / 255.0, 0, frame);
    
    // 좌측/우측 세로 영역
    cv::Rect left_rect(x, y + corner_radius, corner_radius, height - 2 * corner_radius);
    overlay = frame.clone();
    cv::rectangle(overlay, left_rect, bg_color, -1);
    cv::addWeighted(overlay, alpha / 255.0, frame, 1.0 - alpha / 255.0, 0, frame);
    
    cv::Rect right_rect(x + width - corner_radius, y + corner_radius, corner_radius, height - 2 * corner_radius);
    overlay = frame.clone();
    cv::rectangle(overlay, right_rect, bg_color, -1);
    cv::addWeighted(overlay, alpha / 255.0, frame, 1.0 - alpha / 255.0, 0, frame);
    
    // 네 모서리에 원 그리기
    cv::circle(frame, cv::Point(x + corner_radius, y + corner_radius), corner_radius, bg_color, -1);  // Top-left
    cv::circle(frame, cv::Point(x + width - corner_radius, y + corner_radius), corner_radius, bg_color, -1);  // Top-right
    cv::circle(frame, cv::Point(x + corner_radius, y + height - corner_radius), corner_radius, bg_color, -1);  // Bottom-left
    cv::circle(frame, cv::Point(x + width - corner_radius, y + height - corner_radius), corner_radius, bg_color, -1);  // Bottom-right
}

void StatusOverlay::draw(cv::Mat& frame) {
    if (frame.empty()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 상단 상태바 그리기 (0,0) ~ (OUTPUT_WIDTH, RGB_CROP_Y)
    int status_bar_height = RGB_CROP_Y;
    cv::Scalar status_bar_color(34, 34, 34);  // 진회색
    cv::Rect status_bar_rect(0, 0, frame.cols, status_bar_height);
    cv::rectangle(frame, status_bar_rect, status_bar_color, -1);  // -1 = 채우기
    
    const int MARGIN_LEFT = 10;
    const int MARGIN_TOP = 10;
    const int ITEM_SPACING = 15;  // 항목 간 간격
    const double FONT_SCALE = 0.6;
    const int FONT_THICKNESS = 1;  // 두께 줄임 (2 → 1)
    const cv::Scalar TEXT_COLOR(166, 166, 166);  // 회색 글자색
    const int FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;  // Arial 스타일 (선명하고 깔끔함)
    
    int x = MARGIN_LEFT;
    int y = MARGIN_TOP;
    int baseline = 0;
    
    // 1. 드론 이름 (라운드 사각형 배경, 번호만)
    std::string name_text = drone_name_;
    cv::Size name_size = cv::getTextSize(name_text, FONT_FACE, 
                                         FONT_SCALE, FONT_THICKNESS, &baseline);
    
    // 라운드 사각형 배경 크기 계산
    const int name_padding = 6;
    const int name_corner_radius = 4;
    int name_bg_width = name_size.width + name_padding * 2;
    int name_bg_height = name_size.height + name_padding * 2;
    int name_bg_x = x;
    int name_bg_y = y;
    
    // 라운드 사각형 배경 그리기
    cv::Scalar name_bg_color(50, 50, 50);  // 어두운 회색 배경
    // 중앙 사각형
    cv::Rect center_rect(name_bg_x + name_corner_radius, name_bg_y, 
                         name_bg_width - 2 * name_corner_radius, name_bg_height);
    cv::rectangle(frame, center_rect, name_bg_color, -1);
    // 좌측/우측 세로 영역
    cv::Rect left_rect(name_bg_x, name_bg_y + name_corner_radius, 
                       name_corner_radius, name_bg_height - 2 * name_corner_radius);
    cv::rectangle(frame, left_rect, name_bg_color, -1);
    cv::Rect right_rect(name_bg_x + name_bg_width - name_corner_radius, name_bg_y + name_corner_radius, 
                        name_corner_radius, name_bg_height - 2 * name_corner_radius);
    cv::rectangle(frame, right_rect, name_bg_color, -1);
    // 네 모서리 원
    cv::circle(frame, cv::Point(name_bg_x + name_corner_radius, name_bg_y + name_corner_radius), 
               name_corner_radius, name_bg_color, -1);
    cv::circle(frame, cv::Point(name_bg_x + name_bg_width - name_corner_radius, name_bg_y + name_corner_radius), 
               name_corner_radius, name_bg_color, -1);
    cv::circle(frame, cv::Point(name_bg_x + name_corner_radius, name_bg_y + name_bg_height - name_corner_radius), 
               name_corner_radius, name_bg_color, -1);
    cv::circle(frame, cv::Point(name_bg_x + name_bg_width - name_corner_radius, name_bg_y + name_bg_height - name_corner_radius), 
               name_corner_radius, name_bg_color, -1);
    
    // 드론 이름 텍스트
    cv::putText(frame, name_text,
                cv::Point(name_bg_x + name_padding, name_bg_y + name_size.height + name_padding),
                FONT_FACE, FONT_SCALE, 
                cv::Scalar(255, 255, 255), FONT_THICKNESS, cv::LINE_AA);
    
    // 다음 항목 위치 (가로 배열)
    int current_x = name_bg_x + name_bg_width + ITEM_SPACING;
    int text_y = y + name_size.height + name_padding;
    
    // 2. 비행모드 (아이콘 + 값)
    std::string mode_icon = "M";
    std::string mode_text = mode_icon + ":" + px4_mode_;
    cv::Size mode_size = cv::getTextSize(mode_text, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
    cv::putText(frame, mode_text,
                cv::Point(current_x, text_y),
                FONT_FACE, FONT_SCALE, 
                TEXT_COLOR, FONT_THICKNESS, cv::LINE_AA);
    current_x += mode_size.width + ITEM_SPACING;
    
    // 3. 소화탄 (아이콘 + 값) - 커스텀 토픽이므로 녹색
    std::string ammo_icon = "*";
    std::ostringstream ammo_oss;
    ammo_oss << ammo_icon << ":" << ammo_current_ << "/" << ammo_max_;
    cv::Scalar ammo_color = cv::Scalar(0, 255, 0);  // 녹색 (커스텀 토픽)
    cv::Size ammo_size = cv::getTextSize(ammo_oss.str(), FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
    cv::putText(frame, ammo_oss.str(),
                cv::Point(current_x, text_y),
                FONT_FACE, FONT_SCALE, 
                ammo_color, FONT_THICKNESS, cv::LINE_AA);
    current_x += ammo_size.width + ITEM_SPACING;
    
    // 4. GPS (아이콘 + 값, 선택적)
    if (show_gps_) {
        std::string gps_icon = "G";
        std::ostringstream gps_oss;
        gps_oss << gps_icon << ":" << gps_satellites_;
        cv::Scalar gps_color = TEXT_COLOR;
        if (gps_satellites_ >= 6) {
            gps_color = cv::Scalar(0, 255, 0);  // 초록색
        } else if (gps_satellites_ >= 3) {
            gps_color = cv::Scalar(0, 255, 255);  // 노란색
        } else {
            gps_color = cv::Scalar(0, 0, 255);  // 빨간색
        }
        cv::Size gps_size = cv::getTextSize(gps_oss.str(), FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
        cv::putText(frame, gps_oss.str(),
                    cv::Point(current_x, text_y),
                    FONT_FACE, FONT_SCALE, 
                    gps_color, FONT_THICKNESS, cv::LINE_AA);
        current_x += gps_size.width + ITEM_SPACING;
    }
    
    // 5. 배터리 (아이콘 + 값, 선택적)
    if (show_battery_) {
        std::string battery_icon = "B";
        std::ostringstream battery_oss;
        battery_oss << battery_icon << ":" << battery_percentage_ << "%";
        cv::Scalar battery_color = TEXT_COLOR;
        if (battery_percentage_ > 50) {
            battery_color = cv::Scalar(0, 255, 0);  // 초록색
        } else if (battery_percentage_ > 20) {
            battery_color = cv::Scalar(0, 255, 255);  // 노란색
        } else {
            battery_color = cv::Scalar(0, 0, 255);  // 빨간색
        }
        cv::Size battery_size = cv::getTextSize(battery_oss.str(), FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
        cv::putText(frame, battery_oss.str(),
                    cv::Point(current_x, text_y),
                    FONT_FACE, FONT_SCALE, 
                    battery_color, FONT_THICKNESS, cv::LINE_AA);
        current_x += battery_size.width + ITEM_SPACING;
    }
    
    // 6. 편대 (아이콘 + 값) - 커스텀 토픽이므로 녹색
    if (formation_total_ > 1) {
        std::string form_icon = "F";
        std::ostringstream formation_oss;
        formation_oss << form_icon << ":" << formation_current_ << "/" << formation_total_;
        cv::Scalar form_color = cv::Scalar(0, 255, 0);  // 녹색 (커스텀 토픽)
        cv::Size form_size = cv::getTextSize(formation_oss.str(), FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
        cv::putText(frame, formation_oss.str(),
                    cv::Point(current_x, text_y),
                    FONT_FACE, FONT_SCALE, 
                    form_color, FONT_THICKNESS, cv::LINE_AA);
        current_x += form_size.width + ITEM_SPACING;
    }
    
    // 7. 상태 (마지막)
    // OFFBOARD 모드 + VIM4 커스텀 상태면 초록색, 그 외(일반 모드)면 흰색
    std::string status_text = getStatusText(current_status_);
    cv::Scalar status_color;
    if (is_offboard_ && is_offboard_custom_status_) {
        status_color = cv::Scalar(0, 255, 0);  // 초록색 (OFFBOARD 모드 + VIM4 커스텀 상태)
    } else {
        status_color = cv::Scalar(255, 255, 255);  // 흰색 (일반 모드 또는 PX4 기본 상태)
    }
    cv::putText(frame, status_text,
                cv::Point(current_x, text_y),
                FONT_FACE, FONT_SCALE, 
                status_color, FONT_THICKNESS, cv::LINE_AA);
}

