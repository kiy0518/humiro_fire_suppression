#include "rtl_handler.h"
#include <thread>
#include <cmath>

RTLHandler::RTLHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Publishers 초기화
    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // Subscribers 초기화
    vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", 10,
        std::bind(&RTLHandler::vehicleStatusCallback, this, std::placeholders::_1));

    vehicle_global_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", 10,
        std::bind(&RTLHandler::vehicleGlobalPositionCallback, this, std::placeholders::_1));

    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", 10,
        std::bind(&RTLHandler::vehicleLocalPositionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "RTLHandler initialized");
}

bool RTLHandler::returnToLaunch(int timeout_ms)
{
    RCLCPP_INFO(node_->get_logger(), "Executing RTL (Return To Launch)...");

    // GPS 위치 정보 수신 대기
    auto start_time = std::chrono::steady_clock::now();
    while (!global_position_received_) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > 5000) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to receive GPS data for RTL");
            return false;
        }
    }

    // Home position이 설정되지 않았으면 현재 위치를 home으로 설정
    if (!home_position_set_) {
        RCLCPP_WARN(node_->get_logger(), "Home position not set. Using current position as home.");
        home_latitude_ = current_latitude_.load();
        home_longitude_ = current_longitude_.load();
        home_altitude_amsl_ = current_altitude_amsl_.load();
        home_position_set_ = true;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Home position: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                home_latitude_, home_longitude_, home_altitude_amsl_);

    RCLCPP_INFO(node_->get_logger(),
                "Current position: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                current_latitude_.load(), current_longitude_.load(),
                current_altitude_amsl_.load());

    double initial_distance = getDistanceFromHome();
    RCLCPP_INFO(node_->get_logger(),
                "Distance from home: %.2f meters", initial_distance);

    // RTL 명령 전송 (여러 번 전송하여 확실하게)
    for (int i = 0; i < 5; i++) {
        publishVehicleCommand(VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        rclcpp::spin_some(node_);
    }

    RCLCPP_INFO(node_->get_logger(), "RTL command sent, waiting for RTL mode activation...");

    // RTL 모드 활성화 대기 (5초)
    start_time = std::chrono::steady_clock::now();
    bool rtl_activated = false;

    while (!rtl_activated) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (nav_state_ == NAV_STATE_AUTO_RTL) {
            rtl_activated = true;
            RCLCPP_INFO(node_->get_logger(), "RTL mode activated");
            break;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > 5000) {
            RCLCPP_WARN(node_->get_logger(),
                       "RTL mode not confirmed, but continuing (nav_state=%d)",
                       nav_state_.load());
            break;
        }
    }

    // Home 도달 및 착륙 대기
    start_time = std::chrono::steady_clock::now();

    while (true) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double distance = getDistanceFromHome();

        // 착륙 확인
        if (isLanded()) {
            RCLCPP_INFO(node_->get_logger(),
                       "RTL complete! Vehicle landed at home. Distance: %.2f m",
                       distance);
            return true;
        }

        // 타임아웃 확인
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_ms) {
            RCLCPP_ERROR(node_->get_logger(),
                        "RTL timeout! Distance from home: %.2f m",
                        distance);
            return false;
        }

        // 진행 상황 로깅 (5초마다)
        if (static_cast<int>(elapsed) % 5000 < 500) {
            RCLCPP_INFO(node_->get_logger(),
                       "RTL in progress... Distance from home: %.2f m, Altitude: %.2f m",
                       distance, current_altitude_amsl_.load());
        }
    }

    return false;
}

bool RTLHandler::land(int timeout_ms)
{
    RCLCPP_INFO(node_->get_logger(), "Executing LAND...");

    // LAND 명령 전송
    for (int i = 0; i < 5; i++) {
        publishVehicleCommand(VEHICLE_CMD_NAV_LAND);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        rclcpp::spin_some(node_);
    }

    RCLCPP_INFO(node_->get_logger(), "LAND command sent");

    // 착륙 대기
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (isLanded()) {
            RCLCPP_INFO(node_->get_logger(), "Landing complete!");
            return true;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_ms) {
            RCLCPP_ERROR(node_->get_logger(), "Landing timeout!");
            return false;
        }

        // 진행 상황 로깅 (2초마다)
        if (static_cast<int>(elapsed) % 2000 < 500) {
            RCLCPP_INFO(node_->get_logger(),
                       "Landing... Altitude: %.2f m (NED Z: %.2f)",
                       current_altitude_amsl_.load(), -current_local_z_.load());
        }
    }

    return false;
}

double RTLHandler::getDistanceFromHome() const
{
    if (!home_position_set_) {
        return 0.0;
    }

    double horizontal_distance = haversineDistance(
        current_latitude_.load(), current_longitude_.load(),
        home_latitude_, home_longitude_
    );

    double altitude_diff = current_altitude_amsl_.load() - home_altitude_amsl_;

    // 3D 거리 계산
    return std::sqrt(horizontal_distance * horizontal_distance +
                    altitude_diff * altitude_diff);
}

bool RTLHandler::isLanded() const
{
    // 착륙 판정: Arming state가 STANDBY (disarmed)이거나
    // 고도가 매우 낮은 경우
    bool is_disarmed = (arming_state_ == ARMING_STATE_STANDBY);
    bool is_on_ground = (std::abs(current_local_z_.load()) < LANDED_THRESHOLD);

    return is_disarmed || (landed_ && is_on_ground);
}

bool RTLHandler::isRTLActive() const
{
    return nav_state_ == NAV_STATE_AUTO_RTL;
}

void RTLHandler::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    nav_state_ = msg->nav_state;
    arming_state_ = msg->arming_state;

    // PX4 landed state flag (bit field)
    landed_ = (msg->nav_state_timestamp > 0);  // Simple check
}

void RTLHandler::vehicleGlobalPositionCallback(
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    current_latitude_ = msg->lat;
    current_longitude_ = msg->lon;
    current_altitude_amsl_ = msg->alt;

    // 첫 GPS 수신 시 home position 설정
    if (!home_position_set_) {
        home_latitude_ = msg->lat;
        home_longitude_ = msg->lon;
        home_altitude_amsl_ = msg->alt;
        home_position_set_ = true;

        RCLCPP_INFO(node_->get_logger(),
                    "Home position set: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                    home_latitude_, home_longitude_, home_altitude_amsl_);
    }

    global_position_received_ = true;
}

void RTLHandler::vehicleLocalPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_local_z_ = msg->z;
}

void RTLHandler::publishVehicleCommand(
    uint16_t command, float param1, float param2, float param3,
    float param4, float param5, float param6, float param7)
{
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
}

double RTLHandler::haversineDistance(double lat1, double lon1,
                                     double lat2, double lon2) const
{
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;

    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return EARTH_RADIUS * c;
}
