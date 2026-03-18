#ifdef ENABLE_ROS2

#include "status_ros2_subscriber.h"
#include "../../../navigation/src/offboard/bridge/fc_bridge_client.h"
#include <iostream>
#include <chrono>
#include <map>

using namespace std::chrono_literals;

StatusROS2Subscriber::StatusROS2Subscriber(rclcpp::Node::SharedPtr node,
                                           StatusOverlay* status_overlay,
                                           FCBridgeClient* fc_bridge)
    : node_(node)
    , status_overlay_(status_overlay)
    , fc_bridge_(fc_bridge)
    , last_state_update_(std::chrono::steady_clock::now())
    , last_battery_update_(std::chrono::steady_clock::now())
    , last_gps_update_(std::chrono::steady_clock::now())
{
    if (!node_ || !status_overlay_) {
        std::cerr << "  [StatusROS2Subscriber] node 또는 status_overlay가 nullptr입니다" << std::endl;
        return;
    }

    std::cout << "  [StatusROS2Subscriber] v2.0 DDS Domain 분리" << std::endl;
    std::cout << "  → FC 상태: FCBridgeClient (UDP IPC, 10Hz 폴링)" << std::endl;
    std::cout << "  → 커스텀 토픽: ROS2 Domain 1 (WiFi)" << std::endl;

    // ========== FC 상태 폴링 타이머 (10Hz) ==========
    if (fc_bridge_) {
        fc_poll_timer_ = node_->create_wall_timer(
            100ms, std::bind(&StatusROS2Subscriber::pollFCState, this));
        std::cout << "  [OK] FC 상태 폴링 타이머 시작 (10Hz, FCBridgeClient)" << std::endl;
    } else {
        std::cerr << "  [WARN] FCBridgeClient가 nullptr → FC 상태 폴링 비활성화" << std::endl;
    }

    // ========== Domain 1 커스텀 토픽 구독 ==========

    // OFFBOARD 모드 상태 구독 (/offboard/status) - VIM4 커스텀 토픽
    offboard_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/offboard/status", 10,
        std::bind(&StatusROS2Subscriber::offboardStatusCallback, this, std::placeholders::_1));
    std::cout << "  [OK] ROS2 구독: /offboard/status" << std::endl;

    // 소화탄 갯수 구독 (/ammunition/current) - 커스텀 토픽
    ammunition_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/ammunition/current", 10,
        std::bind(&StatusROS2Subscriber::ammunitionCallback, this, std::placeholders::_1));
    std::cout << "  [OK] ROS2 구독: /ammunition/current" << std::endl;

    // 편대 정보 구독 (/formation/current) - 커스텀 토픽
    formation_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/formation/current", 10,
        std::bind(&StatusROS2Subscriber::formationCallback, this, std::placeholders::_1));
    std::cout << "  [OK] ROS2 구독: /formation/current" << std::endl;
}

StatusROS2Subscriber::~StatusROS2Subscriber() {
    if (fc_poll_timer_) {
        fc_poll_timer_->cancel();
    }
}

void StatusROS2Subscriber::setModeChangeCallback(ModeChangeCallback callback) {
    mode_change_callback_ = callback;
}

void StatusROS2Subscriber::spin() {
    if (node_) {
        rclcpp::spin_some(node_);
    }
}

// ========== FC 상태 폴링 (FCBridgeClient, 10Hz) ==========

void StatusROS2Subscriber::pollFCState() {
    if (!fc_bridge_ || !status_overlay_) return;

    FCState state = fc_bridge_->getLatestState();

    // FC 연결 확인
    if (!fc_bridge_->isConnected() || !state.fc_connected) {
        return;
    }

    // 첫 수신 확인
    static bool first_received = false;
    if (!first_received) {
        std::cout << "  [FC Bridge] 첫 FC 상태 수신 (nav=" << (int)state.nav_state
                  << ", arm=" << (int)state.arming_state << ")" << std::endl;
        first_received = true;
    }

    // ========== nav_state / arming_state 처리 ==========
    std::string mode = navStateToModeString(state.nav_state);
    bool is_armed = (state.arming_state == 2);

    // 내부 상태 업데이트 (항상)
    current_nav_state_.store(state.nav_state);
    current_arming_state_.store(state.arming_state);

    // 외부 컴포넌트에 상태 전달
    if (vehicle_status_callback_) {
        vehicle_status_callback_(state.nav_state, state.arming_state);
    }

    // OSD 모드 업데이트
    bool mode_changed = (mode != confirmed_mode_);
    bool armed_changed = (is_armed != confirmed_armed_);

    if (mode_changed || armed_changed) {
        status_overlay_->updatePx4State(mode, is_armed);

        std::cout << "  [비행 모드 변경] " << confirmed_mode_ << " → " << mode
                  << " (nav_state=" << (int)state.nav_state << ")"
                  << ", 시동: " << (confirmed_armed_ ? "ON" : "OFF")
                  << " → " << (is_armed ? "ON" : "OFF") << std::endl;

        // OFFBOARD(14)에서 다른 모드로 변경 시 콜백 호출
        if (last_nav_state_ == 14 && state.nav_state != 14) {
            std::cout << "    OFFBOARD 모드 종료 감지 (nav: 14 → " << (int)state.nav_state << ")" << std::endl;
            if (mode_change_callback_) {
                mode_change_callback_(last_nav_state_, state.nav_state);
            }
        }

        confirmed_mode_ = mode;
        confirmed_armed_ = is_armed;
        last_nav_state_ = state.nav_state;
        last_state_update_ = std::chrono::steady_clock::now();
    } else {
        // 주기적 상태 확인 (30초마다)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_state_update_).count();
        if (elapsed >= 30) {
            std::cout << "  [상태] 모드: " << mode << " (nav=" << (int)state.nav_state
                      << "), 시동: " << (is_armed ? "ON" : "OFF") << std::endl;
            last_state_update_ = now;
        }
    }

    // ========== 배터리 처리 ==========
    float remaining = state.battery_remaining;
    if (remaining >= 0.0f && remaining <= 1.0f) {
        battery_remaining_.store(remaining);

        int percentage = static_cast<int>(remaining * 100.0f);
        if (percentage < 0) percentage = 0;
        if (percentage > 100) percentage = 100;

        status_overlay_->setBattery(percentage);

        // 주기적 배터리 출력 (5초마다)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_battery_update_).count();
        if (elapsed >= 5) {
            std::cout << "  [배터리] " << percentage << "% (remaining=" << remaining << ")" << std::endl;
            last_battery_update_ = now;
        }
    }

    // ========== 고도 처리 ==========
    // MAVLink GLOBAL_POSITION_INT.relative_alt 사용 (QGC alt(rel)과 동일)
    // application_manager.cpp의 altitude_callback_에서 처리 (DDS 덮어쓰기 제거)

    // ========== 속도 처리 ==========
    status_overlay_->setVelocity(state.vx, state.vy, state.vz);

    // 하방 거리: MAVLink DISTANCE_SENSOR 콜백에서 처리 (application_manager.cpp)
    // FC Bridge의 dist_bottom은 vehicle_local_position 기반이라 거리센서 미장착 시 항상 invalid


    // ========== GPS 처리 ==========
    gps_fix_type_.store(state.gps_fix_type);

    int satellites = static_cast<int>(state.gps_satellites);
    if (satellites < 0) satellites = 0;
    if (satellites > 50) satellites = 50;

    float hdop = state.gps_hdop;
    if (hdop < 0.0f) hdop = 0.0f;

    status_overlay_->setGpsInfo(satellites, hdop);

    // 주기적 GPS 출력 (5초마다)
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_gps_update_).count();
        if (elapsed >= 5) {
            std::cout << "  [GPS] 위성: " << satellites << ", HDOP: " << hdop
                      << ", fix_type: " << (int)state.gps_fix_type << std::endl;
            last_gps_update_ = now;
        }
    }
}

// PX4 nav_state를 모드 문자열로 변환
std::string StatusROS2Subscriber::navStateToModeString(uint8_t nav_state) {
    static std::map<uint8_t, std::string> nav_state_map = {
        {0, "MANUAL"},
        {1, "Altitude"},
        {2, "Position"},
        {3, "Mission"},
        {4, "Hold"},
        {5, "AUTO_RTL"},
        {6, "Position Slow"},
        {7, "FREE5"},
        {8, "ALTITUDE_CRUISE"},
        {9, "FREE3"},
        {10, "Acro"},
        {11, "FREE2"},
        {12, "DESCEND"},
        {13, "TERMINATION"},
        {14, "OFFBOARD"},
        {15, "Stabilized"},
        {16, "FREE1"},
        {17, "AUTO_TAKEOFF"},
        {18, "AUTO_LAND"},
        {19, "Follow Target"},
        {20, "Precision Landing"},
        {21, "ORBIT"},
        {22, "VTOL Takeoff"},
        {23, "EXTERNAL1"},
        {24, "EXTERNAL2"},
    };

    auto it = nav_state_map.find(nav_state);
    if (it != nav_state_map.end()) {
        return it->second;
    }
    return "UNKNOWN";
}

void StatusROS2Subscriber::offboardStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!status_overlay_) return;

    std::string status_str = msg->data;

    StatusOverlay::DroneStatus status = StatusOverlay::DroneStatus::IDLE;

    if (status_str == "IDLE") {
        status = StatusOverlay::DroneStatus::IDLE;
    } else if (status_str == "ARMING") {
        status = StatusOverlay::DroneStatus::ARMING;
    } else if (status_str == "TAKEOFF") {
        status = StatusOverlay::DroneStatus::TAKEOFF;
    } else if (status_str == "HOVERING") {
        status = StatusOverlay::DroneStatus::HOVERING;
    } else if (status_str == "ROTATING") {
        status = StatusOverlay::DroneStatus::ROTATING;
    } else if (status_str == "NAVIGATING") {
        status = StatusOverlay::DroneStatus::NAVIGATING;
    } else if (status_str == "DISTANCE_ADJUST") {
        status = StatusOverlay::DroneStatus::DISTANCE_ADJUST;
    } else if (status_str == "DESTINATION_REACHED") {
        status = StatusOverlay::DroneStatus::DESTINATION_REACHED;
    } else if (status_str == "TRACKING") {
        status = StatusOverlay::DroneStatus::TRACKING;
    } else if (status_str == "FIRE_READY") {
        status = StatusOverlay::DroneStatus::FIRE_READY;
    } else if (status_str == "FIRING") {
        status = StatusOverlay::DroneStatus::FIRING;
    } else if (status_str == "FIRING_AUTO_TARGETING") {
        status = StatusOverlay::DroneStatus::FIRING_AUTO_TARGETING;
    } else if (status_str == "AUTO_FIRING") {
        status = StatusOverlay::DroneStatus::AUTO_FIRING;
    } else if (status_str == "MISSION_COMPLETE") {
        status = StatusOverlay::DroneStatus::MISSION_COMPLETE;
    } else if (status_str == "RETURNING") {
        status = StatusOverlay::DroneStatus::RETURNING;
    } else if (status_str == "LANDING") {
        status = StatusOverlay::DroneStatus::LANDING;
    } else if (status_str == "DISARMED") {
        status = StatusOverlay::DroneStatus::DISARMED;
    }

    status_overlay_->updateOffboardStatus(status);

    static std::string last_status = "";
    if (status_str != last_status) {
        std::cout << "  [OFFBOARD 모드] 상태: " << status_str << std::endl;
        last_status = status_str;
    }
}

void StatusROS2Subscriber::ammunitionCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!status_overlay_) return;

    int current = msg->data;
    if (current < 0) current = 0;

    status_overlay_->setAmmunition(current, 6);

    static int last_ammo = -1;
    if (current != last_ammo) {
        std::cout << "  [소화탄 업데이트] " << current << "/6발" << std::endl;
        last_ammo = current;
    }
}

void StatusROS2Subscriber::formationCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // 편대 OSD 표시 제거됨 (v0.18.1+)
    (void)msg;
}

bool StatusROS2Subscriber::isFCConnected() const {
    if (!fc_bridge_) return false;
    return fc_bridge_->isConnected();
}

bool StatusROS2Subscriber::isGPSFixed() const {
    return gps_fix_type_.load() >= 3;  // 3D Fix 이상
}

#endif // ENABLE_ROS2
