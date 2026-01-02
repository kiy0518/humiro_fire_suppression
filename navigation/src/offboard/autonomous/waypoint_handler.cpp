#include "waypoint_handler.h"
#include <thread>

WaypointHandler::WaypointHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Publishers 초기화
    trajectory_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    // Subscribers 초기화
    vehicle_global_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", 10,
        std::bind(&WaypointHandler::vehicleGlobalPositionCallback, this, std::placeholders::_1));

    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", 10,
        std::bind(&WaypointHandler::vehicleLocalPositionCallback, this, std::placeholders::_1));

    // OFFBOARD 모드 heartbeat 타이머 (2Hz)
    offboard_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&WaypointHandler::publishOffboardControlMode, this));

    RCLCPP_INFO(node_->get_logger(), "WaypointHandler initialized");
}

bool WaypointHandler::goToWaypoint(const GPSCoordinate& target, int timeout_ms)
{
    RCLCPP_INFO(node_->get_logger(),
                "Going to waypoint: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                target.latitude, target.longitude, target.altitude);

    // GPS 위치 정보 수신 대기
    auto start_time = std::chrono::steady_clock::now();
    while (!global_position_received_ || !local_position_received_) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > 5000) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to receive GPS data");
            return false;
        }
    }

    // 목표 설정
    target_waypoint_ = target;

    // 현재 위치 로깅
    RCLCPP_INFO(node_->get_logger(),
                "Current position: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                current_latitude_.load(), current_longitude_.load(),
                current_altitude_amsl_.load());

    // 초기 거리 계산
    double initial_distance = getDistanceToTarget(target);
    RCLCPP_INFO(node_->get_logger(),
                "Distance to target: %.2f meters", initial_distance);

    // GPS를 Local NED로 변환
    float target_x, target_y, target_z;
    gpsToLocalNED(target, target_x, target_y, target_z);

    RCLCPP_INFO(node_->get_logger(),
                "Target in NED: X=%.2f, Y=%.2f, Z=%.2f",
                target_x, target_y, target_z);

    // 목표 위치로 이동
    start_time = std::chrono::steady_clock::now();

    while (true) {
        // TrajectorySetpoint 발행
        publishTrajectorySetpoint(target_x, target_y, target_z, current_yaw_);

        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 거리 확인
        double distance = getDistanceToTarget(target);

        if (distance < WAYPOINT_THRESHOLD) {
            RCLCPP_INFO(node_->get_logger(),
                        "Waypoint reached! Distance: %.2f m", distance);
            return true;
        }

        // 타임아웃 확인
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_ms) {
            RCLCPP_ERROR(node_->get_logger(),
                        "Waypoint navigation timeout! Distance: %.2f m", distance);
            return false;
        }

        // 진행 상황 로깅 (5초마다)
        if (static_cast<int>(elapsed) % 5000 < 100) {
            RCLCPP_INFO(node_->get_logger(),
                        "Navigating... Distance: %.2f m, Position: (%.7f, %.7f)",
                        distance, current_latitude_.load(), current_longitude_.load());
        }
    }

    return false;
}

GPSCoordinate WaypointHandler::getCurrentPosition() const
{
    GPSCoordinate pos;
    pos.latitude = current_latitude_.load();
    pos.longitude = current_longitude_.load();
    pos.altitude = current_altitude_amsl_.load();
    return pos;
}

double WaypointHandler::getDistanceToTarget(const GPSCoordinate& target) const
{
    double horizontal_distance = haversineDistance(
        current_latitude_.load(), current_longitude_.load(),
        target.latitude, target.longitude
    );

    double altitude_diff = target.altitude - current_altitude_amsl_.load();

    // 3D 거리 계산
    return std::sqrt(horizontal_distance * horizontal_distance +
                    altitude_diff * altitude_diff);
}

bool WaypointHandler::isWaypointReached() const
{
    double distance = getDistanceToTarget(target_waypoint_);
    return distance < WAYPOINT_THRESHOLD;
}

void WaypointHandler::vehicleGlobalPositionCallback(
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    current_latitude_ = msg->lat;
    current_longitude_ = msg->lon;
    current_altitude_amsl_ = msg->alt;

    // 첫 GPS 수신 시 home position 설정
    if (!global_position_received_) {
        home_latitude_ = msg->lat;
        home_longitude_ = msg->lon;
        home_altitude_amsl_ = msg->alt;

        RCLCPP_INFO(node_->get_logger(),
                    "Home position set: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                    home_latitude_, home_longitude_, home_altitude_amsl_);
    }

    global_position_received_ = true;
}

void WaypointHandler::vehicleLocalPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_local_x_ = msg->x;
    current_local_y_ = msg->y;
    current_local_z_ = msg->z;
    current_yaw_ = msg->heading;
    local_position_received_ = true;
}

void WaypointHandler::publishTrajectorySetpoint(float x, float y, float z, float yaw)
{
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;

    // Position setpoint (NED 좌표계)
    msg.position[0] = x;
    msg.position[1] = y;
    msg.position[2] = z;

    msg.yaw = yaw;

    // NaN으로 설정하여 사용하지 않는 필드 표시
    msg.velocity[0] = std::nanf("");
    msg.velocity[1] = std::nanf("");
    msg.velocity[2] = std::nanf("");

    msg.acceleration[0] = std::nanf("");
    msg.acceleration[1] = std::nanf("");
    msg.acceleration[2] = std::nanf("");

    msg.jerk[0] = std::nanf("");
    msg.jerk[1] = std::nanf("");
    msg.jerk[2] = std::nanf("");

    msg.yawspeed = std::nanf("");

    trajectory_setpoint_pub_->publish(msg);
}

void WaypointHandler::publishOffboardControlMode()
{
    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);
}

void WaypointHandler::gpsToLocalNED(const GPSCoordinate& target,
                                     float& local_x, float& local_y, float& local_z)
{
    // 위도 차이 (북쪽 방향)
    double lat_diff = target.latitude - home_latitude_;
    local_x = lat_diff * DEG_TO_RAD * EARTH_RADIUS;

    // 경도 차이 (동쪽 방향)
    // cos(위도)로 보정 (위도가 높을수록 경도 간격이 좁아짐)
    double lon_diff = target.longitude - home_longitude_;
    double lat_avg = (target.latitude + home_latitude_) / 2.0;
    local_y = lon_diff * DEG_TO_RAD * EARTH_RADIUS * std::cos(lat_avg * DEG_TO_RAD);

    // 고도 차이 (NED에서는 아래 방향이 양수)
    local_z = -(target.altitude - home_altitude_amsl_);
}

double WaypointHandler::haversineDistance(double lat1, double lon1,
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
