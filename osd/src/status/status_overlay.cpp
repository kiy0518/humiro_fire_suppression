#include "status_overlay.h"
#include "../../thermal/src/config.h"
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <cmath>

StatusOverlay::StatusOverlay()
    : current_status_(DroneStatus::IDLE)
    , px4_mode_("UNKNOWN")
    , is_armed_(false)
    , is_offboard_(false)
    , is_offboard_custom_status_(false)  // VIM4 커스텀 상태 사용 여부
    , ammo_current_(6)
    , ammo_max_(6)
    , drone_name_("1")
    , wifi_ssid_("")
    , wifi_rssi_(0)
    , show_wifi_(false)
    , battery_percentage_(-1)
    , gps_satellites_(-1)
    , gps_hdop_(-1.0f)
    , max_temperature_(-1.0)
    , temp_threshold_(50.0)
    , altitude_(0.0f)
    , dist_bottom_(-1.0f)
    , dist_bottom_valid_(false)
    , vel_horizontal_(0.0f)
    , vel_vertical_(0.0f)
    , show_velocity_(false)
    , show_battery_(false)
    , show_gps_(false)
    , show_temperature_(false)
    , show_altitude_(false)
    , show_dist_bottom_(false)
    , custom_message_("")
    , custom_message_timeout_(0.0)
    , show_custom_message_(false)
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

void StatusOverlay::setWifiInfo(const std::string& ssid, int rssi) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    wifi_ssid_ = ssid;
    wifi_rssi_ = rssi;
    show_wifi_ = !ssid.empty();
}

void StatusOverlay::setBattery(int percentage) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    battery_percentage_ = percentage;
    show_battery_ = (percentage >= 0);
}

void StatusOverlay::setGpsInfo(int satellites, float hdop) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    gps_satellites_ = satellites;
    gps_hdop_ = hdop;
    show_gps_ = (satellites >= 0 && hdop >= 0.0f);
}

void StatusOverlay::setAltitude(float altitude) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    altitude_ = altitude;
    show_altitude_ = true;
}

void StatusOverlay::setDistBottom(float distance, bool valid) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    dist_bottom_ = distance;
    dist_bottom_valid_ = valid;
    show_dist_bottom_ = valid;
}

void StatusOverlay::setVelocity(float vx, float vy, float vz) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    vel_horizontal_ = std::sqrt(vx * vx + vy * vy);
    vel_vertical_ = -vz;  // NED: vz 양수=하강, 부호 반전하여 양수=상승
    show_velocity_ = true;
}

void StatusOverlay::setMaxTemperature(double temperature) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    max_temperature_ = temperature;
    show_temperature_ = (temperature >= 0.0);
}

void StatusOverlay::setTempThreshold(double threshold) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    temp_threshold_ = threshold;
}

void StatusOverlay::setCustomMessage(const std::string& message, double timeout_seconds) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::cout << "[StatusOverlay] [STEP 5] setCustomMessage 호출: \"" << message 
              << "\", timeout=" << timeout_seconds << "초" << std::endl;
    
    custom_message_ = message;
    custom_message_timeout_ = timeout_seconds;
    custom_message_time_ = std::chrono::steady_clock::now();
    show_custom_message_ = true;
    
    std::cout << "[StatusOverlay] [STEP 5] ✓ 커스텀 메시지 설정 완료 (OSD 표시 활성화)" << std::endl;
}

void StatusOverlay::clearCustomMessage() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    custom_message_ = "";
    show_custom_message_ = false;
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
        case DroneStatus::HOVERING:
        case DroneStatus::ROTATING:
            return cv::Scalar(0, 255, 255);  // 노란색
        case DroneStatus::NAVIGATING:
        case DroneStatus::RETURNING:
            return cv::Scalar(255, 0, 0);  // 파란색
        case DroneStatus::DISTANCE_ADJUST:
        case DroneStatus::TRACKING:
            return cv::Scalar(0, 255, 0);  // 초록색
        case DroneStatus::DESTINATION_REACHED:
        case DroneStatus::FIRE_READY:
            return cv::Scalar(0, 255, 0);  // 초록색
        case DroneStatus::FIRING:
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
        case DroneStatus::HOVERING:
            return "HOVERING";
        case DroneStatus::ROTATING:
            return "ROTATING";
        case DroneStatus::NAVIGATING:
            return "NAVIGATING";
        case DroneStatus::DISTANCE_ADJUST:
            return "DIST_ADJUST";
        case DroneStatus::DESTINATION_REACHED:
            return "DEST_REACHED";
        case DroneStatus::TRACKING:
            return "TRACKING";
        case DroneStatus::FIRE_READY:
            return "FIRE_READY";
        case DroneStatus::FIRING:
            return "FIRING";
        case DroneStatus::FIRING_AUTO_TARGETING:
            return "AUTO_AIMING";
        case DroneStatus::AUTO_FIRING:
            return "AUTO_FIRING";
        case DroneStatus::MISSION_COMPLETE:
            return "COMPLETE";
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
    
    // 상단 상태바 그리기 (device_config.env의 STATUS_BAR_HEIGHT)
    int status_bar_height = ThermalConfig::get().status_bar_height;
    cv::Scalar status_bar_color(34, 34, 34);  // 진회색
    cv::Rect status_bar_rect(0, 0, frame.cols, status_bar_height);
    cv::rectangle(frame, status_bar_rect, status_bar_color, -1);  // -1 = 채우기
    
    const int MARGIN_LEFT = 6;
    const int MARGIN_TOP = 6;     // 상단 여백 (작을수록 글자가 위로)
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
    int mode_x_start = current_x;  // 모드 텍스트 시작 X 위치 저장
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
    
    // 4. GPS (아이콘 + 값, 선택적) - 형식: G:위성수(hdop)
    if (show_gps_) {
        std::string gps_icon = "G";
        std::ostringstream gps_oss;
        gps_oss << gps_icon << ":" << gps_satellites_;
        if (gps_hdop_ >= 0.0f) {
            gps_oss << "(" << std::fixed << std::setprecision(1) << gps_hdop_ << ")";
        }
        cv::Scalar gps_color = TEXT_COLOR;
        if (gps_satellites_ >= 6 && gps_hdop_ < 2.0f) {
            gps_color = cv::Scalar(0, 255, 0);  // 초록색 (좋은 신호)
        } else if (gps_satellites_ >= 3 && gps_hdop_ < 5.0f) {
            gps_color = cv::Scalar(0, 255, 255);  // 노란색 (보통)
        } else {
            gps_color = cv::Scalar(0, 0, 255);  // 빨간색 (나쁨)
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

    // 5.5. 온도 → 상태바에서 제거 (핫스팟 마커 옆에 표시)

    // 6. 상태 표시 (비행모드 아래, 라운드 진회색 배경)
    std::string status_text = getStatusText(current_status_);
    // 텍스트 색상: OFFBOARD 모드 = 초록색, 표준 모드 = 연회색
    cv::Scalar status_text_color = (is_offboard_ && is_offboard_custom_status_)
        ? cv::Scalar(0, 255, 0) : cv::Scalar(200, 200, 200);

    {
        int status_y_top = status_bar_height + 4;
        cv::Size status_size = cv::getTextSize(status_text, FONT_FACE,
                                               FONT_SCALE, FONT_THICKNESS, &baseline);
        const int pad = 5;
        const int r = 4;  // 모서리 반경
        cv::Scalar bg_color(50, 50, 50);

        int box_x = mode_x_start - pad;
        int box_y = status_y_top;
        int box_w = status_size.width + pad * 2;
        int box_h = status_size.height + pad * 2;

        if (box_x >= 0 && box_y >= 0 &&
            box_x + box_w <= frame.cols && box_y + box_h <= frame.rows) {
            // 라운드 사각형: fillPoly (단일 폴리곤, LINE_AA)
            std::vector<cv::Point> pts;
            const int step = 15;  // 원호 분할 각도
            // 좌상단 원호 (180°→270°)
            for (int a = 180; a <= 270; a += step)
                pts.emplace_back(box_x + r + (int)(r * std::cos(a * M_PI / 180)),
                                 box_y + r + (int)(r * std::sin(a * M_PI / 180)));
            // 우상단 원호 (270°→360°)
            for (int a = 270; a <= 360; a += step)
                pts.emplace_back(box_x + box_w - r + (int)(r * std::cos(a * M_PI / 180)),
                                 box_y + r + (int)(r * std::sin(a * M_PI / 180)));
            // 우하단 원호 (0°→90°)
            for (int a = 0; a <= 90; a += step)
                pts.emplace_back(box_x + box_w - r + (int)(r * std::cos(a * M_PI / 180)),
                                 box_y + box_h - r + (int)(r * std::sin(a * M_PI / 180)));
            // 좌하단 원호 (90°→180°)
            for (int a = 90; a <= 180; a += step)
                pts.emplace_back(box_x + r + (int)(r * std::cos(a * M_PI / 180)),
                                 box_y + box_h - r + (int)(r * std::sin(a * M_PI / 180)));

            std::vector<std::vector<cv::Point>> poly = {pts};
            cv::fillPoly(frame, poly, bg_color, cv::LINE_AA);
        }

        // 텍스트 수직 중앙 배치 (baseline 보정)
        int text_y = box_y + pad + status_size.height;
        cv::putText(frame, status_text,
                    cv::Point(mode_x_start, text_y),
                    FONT_FACE, FONT_SCALE,
                    status_text_color, FONT_THICKNESS, cv::LINE_AA);

        // 6.5. 고도 + 하방거리 (상태 뱃지 옆, 라운드 배경)
        {
            std::string alt_str = show_altitude_
                ? "A:" + ([&]{ std::ostringstream o; o << std::fixed << std::setprecision(2) << altitude_; return o.str(); })() + "m"
                : "A:--m";
            std::string dist_str = show_dist_bottom_
                ? "H:" + ([&]{ std::ostringstream o; o << std::fixed << std::setprecision(2) << dist_bottom_; return o.str(); })() + "m"
                : "H:--m";
            std::string info_str = alt_str + "  " + dist_str;

            cv::Size info_size = cv::getTextSize(info_str, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);

            // 라운드 배경
            int ib_x = box_x + box_w + ITEM_SPACING - pad;
            int ib_y = box_y;
            int ib_w = info_size.width + pad * 2;
            int ib_h = box_h;

            if (ib_x + ib_w <= frame.cols && ib_y + ib_h <= frame.rows) {
                std::vector<cv::Point> ipts;
                const int istep = 15;
                for (int a = 180; a <= 270; a += istep)
                    ipts.emplace_back(ib_x + r + (int)(r * std::cos(a * M_PI / 180)),
                                      ib_y + r + (int)(r * std::sin(a * M_PI / 180)));
                for (int a = 270; a <= 360; a += istep)
                    ipts.emplace_back(ib_x + ib_w - r + (int)(r * std::cos(a * M_PI / 180)),
                                      ib_y + r + (int)(r * std::sin(a * M_PI / 180)));
                for (int a = 0; a <= 90; a += istep)
                    ipts.emplace_back(ib_x + ib_w - r + (int)(r * std::cos(a * M_PI / 180)),
                                      ib_y + ib_h - r + (int)(r * std::sin(a * M_PI / 180)));
                for (int a = 90; a <= 180; a += istep)
                    ipts.emplace_back(ib_x + r + (int)(r * std::cos(a * M_PI / 180)),
                                      ib_y + ib_h - r + (int)(r * std::sin(a * M_PI / 180)));
                std::vector<std::vector<cv::Point>> ipoly = {ipts};
                cv::fillPoly(frame, ipoly, bg_color, cv::LINE_AA);
            }

            // 텍스트: 고도 부분
            int info_tx = ib_x + pad;
            cv::Scalar alt_color = show_altitude_ ? cv::Scalar(255, 255, 255) : cv::Scalar(128, 128, 128);
            cv::Size alt_size = cv::getTextSize(alt_str, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
            cv::putText(frame, alt_str,
                        cv::Point(info_tx, text_y),
                        FONT_FACE, FONT_SCALE,
                        alt_color, FONT_THICKNESS, cv::LINE_AA);
            info_tx += alt_size.width;

            // 구분 공백
            cv::Size sp_size = cv::getTextSize("  ", FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
            info_tx += sp_size.width;

            // 텍스트: 하방거리 부분
            cv::Scalar dist_color = cv::Scalar(128, 128, 128);
            if (show_dist_bottom_) {
                if (dist_bottom_ > 5.0f) dist_color = cv::Scalar(0, 255, 0);
                else if (dist_bottom_ > 2.0f) dist_color = cv::Scalar(0, 255, 255);
                else dist_color = cv::Scalar(0, 0, 255);
            }
            cv::putText(frame, dist_str,
                        cv::Point(info_tx, text_y),
                        FONT_FACE, FONT_SCALE,
                        dist_color, FONT_THICKNESS, cv::LINE_AA);

            // 6.6. 수직/수평 속도 (고도 뱃지 옆, 라운드 배경, arrowedLine 화살표)
            {
                // 속도 문자열 (화살표 없이 값만)
                std::string vspd_str = show_velocity_
                    ? ([&]{ std::ostringstream o; o << std::fixed << std::setprecision(1) << std::abs(vel_vertical_); return o.str(); })() + "m/s"
                    : "--m/s";
                std::string hspd_str = show_velocity_
                    ? ([&]{ std::ostringstream o; o << std::fixed << std::setprecision(1) << vel_horizontal_; return o.str(); })() + "m/s"
                    : "--m/s";

                // 화살표 크기 (텍스트 높이에 맞춤)
                const int arrow_len = 10;   // 화살표 길이 (px)
                const int arrow_gap = 3;    // 화살표와 텍스트 사이 간격
                const int arrow_w = arrow_len + arrow_gap;  // 화살표가 차지하는 폭

                // 배경 크기 계산 (화살표 + 텍스트)
                cv::Size vspd_sz = cv::getTextSize(vspd_str, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
                cv::Size hspd_sz = cv::getTextSize(hspd_str, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
                cv::Size gap_sz = cv::getTextSize("  ", FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
                int total_w = arrow_w + vspd_sz.width + gap_sz.width + arrow_w + hspd_sz.width;

                int sb_x = ib_x + ib_w + ITEM_SPACING - pad;
                int sb_y = box_y;
                int sb_w = total_w + pad * 2;
                int sb_h = box_h;

                if (sb_x + sb_w <= frame.cols && sb_y + sb_h <= frame.rows) {
                    std::vector<cv::Point> spts;
                    const int sstep = 15;
                    for (int a = 180; a <= 270; a += sstep)
                        spts.emplace_back(sb_x + r + (int)(r * std::cos(a * M_PI / 180)),
                                          sb_y + r + (int)(r * std::sin(a * M_PI / 180)));
                    for (int a = 270; a <= 360; a += sstep)
                        spts.emplace_back(sb_x + sb_w - r + (int)(r * std::cos(a * M_PI / 180)),
                                          sb_y + r + (int)(r * std::sin(a * M_PI / 180)));
                    for (int a = 0; a <= 90; a += sstep)
                        spts.emplace_back(sb_x + sb_w - r + (int)(r * std::cos(a * M_PI / 180)),
                                          sb_y + sb_h - r + (int)(r * std::sin(a * M_PI / 180)));
                    for (int a = 90; a <= 180; a += sstep)
                        spts.emplace_back(sb_x + r + (int)(r * std::cos(a * M_PI / 180)),
                                          sb_y + sb_h - r + (int)(r * std::sin(a * M_PI / 180)));
                    std::vector<std::vector<cv::Point>> spoly = {spts};
                    cv::fillPoly(frame, spoly, bg_color, cv::LINE_AA);
                }

                int spd_tx = sb_x + pad;
                int arrow_cy = sb_y + sb_h / 2;  // 화살표 수직 중심 = 뱃지 중심

                // 수직 속도 화살표 (arrowedLine)
                cv::Scalar vspd_color = show_velocity_ ? cv::Scalar(255, 255, 255) : cv::Scalar(128, 128, 128);
                {
                    int ax = spd_tx + arrow_len / 2;  // 화살표 수평 중심
                    cv::Point pt_from, pt_to;
                    if (!show_velocity_ || vel_vertical_ >= 0) {
                        // 위 화살표 (상승 또는 기본)
                        pt_from = cv::Point(ax, arrow_cy + arrow_len / 2);
                        pt_to   = cv::Point(ax, arrow_cy - arrow_len / 2);
                    } else {
                        // 아래 화살표 (하강)
                        pt_from = cv::Point(ax, arrow_cy - arrow_len / 2);
                        pt_to   = cv::Point(ax, arrow_cy + arrow_len / 2);
                    }
                    cv::arrowedLine(frame, pt_from, pt_to, vspd_color, 1, cv::LINE_AA, 0, 0.35);
                }
                spd_tx += arrow_w;

                // 수직 속도 텍스트
                cv::putText(frame, vspd_str, cv::Point(spd_tx, text_y),
                            FONT_FACE, FONT_SCALE, vspd_color, FONT_THICKNESS, cv::LINE_AA);
                spd_tx += vspd_sz.width + gap_sz.width;

                // 수평 속도 화살표 (arrowedLine, 오른쪽 →)
                cv::Scalar hspd_color = show_velocity_ ? cv::Scalar(0, 255, 255) : cv::Scalar(128, 128, 128);
                {
                    int ax_start = spd_tx;
                    cv::Point pt_from(ax_start, arrow_cy);
                    cv::Point pt_to(ax_start + arrow_len, arrow_cy);
                    cv::arrowedLine(frame, pt_from, pt_to, hspd_color, 1, cv::LINE_AA, 0, 0.35);
                }
                spd_tx += arrow_w;

                // 수평 속도 텍스트
                cv::putText(frame, hspd_str, cv::Point(spd_tx, text_y),
                            FONT_FACE, FONT_SCALE, hspd_color, FONT_THICKNESS, cv::LINE_AA);
            }
        }
    }

    // 7. WiFi 정보 (오른쪽 정렬) - SSID + 안테나 바 아이콘
    if (show_wifi_) {
        // 신호 강도에 따른 바 수 결정 (4단계)
        int bars = 0;
        cv::Scalar bar_color;
        if (wifi_rssi_ >= -50) {
            bars = 4; bar_color = cv::Scalar(0, 255, 0);     // 초록 (강함)
        } else if (wifi_rssi_ >= -60) {
            bars = 3; bar_color = cv::Scalar(0, 255, 0);     // 초록 (양호)
        } else if (wifi_rssi_ >= -70) {
            bars = 2; bar_color = cv::Scalar(0, 255, 255);   // 노랑 (보통)
        } else if (wifi_rssi_ >= -80) {
            bars = 1; bar_color = cv::Scalar(0, 0, 255);     // 빨강 (약함)
        } else {
            bars = 0; bar_color = cv::Scalar(0, 0, 255);     // 빨강 (매우 약함)
        }

        // 바 아이콘 크기
        const int BAR_WIDTH = 4;
        const int BAR_GAP = 2;
        const int BAR_MAX_HEIGHT = 16;
        const int BAR_MIN_HEIGHT = 4;
        const int NUM_BARS = 4;
        int total_bar_width = NUM_BARS * BAR_WIDTH + (NUM_BARS - 1) * BAR_GAP;

        // SSID 텍스트 크기
        cv::Size ssid_size = cv::getTextSize(wifi_ssid_, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
        int ssid_bar_gap = 6;
        int total_wifi_width = ssid_size.width + ssid_bar_gap + total_bar_width;

        int wifi_x = frame.cols - total_wifi_width - MARGIN_LEFT;

        // SSID 텍스트
        cv::putText(frame, wifi_ssid_,
                    cv::Point(wifi_x, text_y),
                    FONT_FACE, FONT_SCALE,
                    bar_color, FONT_THICKNESS, cv::LINE_AA);

        // 안테나 바 그리기 (왼쪽부터 짧→긴)
        int bar_base_y = text_y + 1;  // 텍스트 baseline 기준
        int bar_x = wifi_x + ssid_size.width + ssid_bar_gap;
        cv::Scalar inactive_color(80, 80, 80);  // 비활성 바 색상

        for (int i = 0; i < NUM_BARS; i++) {
            int bar_height = BAR_MIN_HEIGHT + (BAR_MAX_HEIGHT - BAR_MIN_HEIGHT) * (i + 1) / NUM_BARS;
            int bx = bar_x + i * (BAR_WIDTH + BAR_GAP);
            int by = bar_base_y - bar_height;
            cv::Scalar color = (i < bars) ? bar_color : inactive_color;
            cv::rectangle(frame, cv::Rect(bx, by, BAR_WIDTH, bar_height), color, -1);
        }
    }

    // QGC 커스텀 메시지 표시 (왼쪽 하단)
    if (show_custom_message_ && !custom_message_.empty()) {
        // 타임아웃 확인
        if (custom_message_timeout_ > 0.0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - custom_message_time_).count() / 1000.0;
            if (elapsed >= custom_message_timeout_) {
                static bool timeout_logged = false;
                if (!timeout_logged) {
                    std::cout << "[StatusOverlay] [STEP 6] 커스텀 메시지 타임아웃 (표시 중지)" << std::endl;
                    timeout_logged = true;
                }
                show_custom_message_ = false;
            }
        }
        
        if (show_custom_message_) {
            static int render_count = 0;
            render_count++;
            if (render_count <= 5 || render_count % 60 == 0) {  // 처음 5번 + 60프레임마다
                std::cout << "[StatusOverlay] [STEP 6] OSD 렌더링: \"" << custom_message_ 
                          << "\" (프레임 #" << render_count << ")" << std::endl;
            }
            const double MSG_FONT_SCALE = 0.6;
            const int MSG_FONT_THICKNESS = 1;
            const cv::Scalar MSG_TEXT_COLOR(0, 255, 255);  // 노란색 텍스트 (경고 메시지)
            const int MSG_PADDING = 8;

            // 텍스트 크기 계산
            int msg_baseline = 0;
            cv::Size msg_size = cv::getTextSize(custom_message_, FONT_FACE,
                                                MSG_FONT_SCALE, MSG_FONT_THICKNESS, &msg_baseline);

            // 상단 3번째 열 중앙 배치 (1열: 상태바 내부, 2열: 비행모드/고도/속도 뱃지)
            int status_bar_h = ThermalConfig::get().status_bar_height;
            int row2_height = msg_size.height + 10 + 4;  // 2열 뱃지 높이(pad*2) + 간격
            int msg_bg_width = msg_size.width + MSG_PADDING * 2;
            int msg_bg_height = msg_size.height + MSG_PADDING * 2;
            int msg_bg_x = (frame.cols - msg_bg_width) / 2;
            int msg_bg_y = status_bar_h + 4 + row2_height;

            // 반투명 라운드 배경 (테두리 없음)
            {
                const int r = 5;
                const int step = 15;
                std::vector<cv::Point> pts;
                int bx = msg_bg_x, by = msg_bg_y, bw = msg_bg_width, bh = msg_bg_height;
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

                cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
                std::vector<std::vector<cv::Point>> poly = {pts};
                cv::fillPoly(mask, poly, cv::Scalar(255), cv::LINE_AA);

                cv::Mat darkened;
                frame.copyTo(darkened);
                darkened.setTo(cv::Scalar(0, 0, 0), mask);
                cv::addWeighted(frame, 0.5, darkened, 0.5, 0, frame);
            }

            // 텍스트 그리기
            cv::putText(frame, custom_message_,
                        cv::Point(msg_bg_x + MSG_PADDING, msg_bg_y + msg_size.height + MSG_PADDING),
                        FONT_FACE, MSG_FONT_SCALE,
                        MSG_TEXT_COLOR, MSG_FONT_THICKNESS, cv::LINE_AA);
        }
    }
}

