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
    // PX4 uXRCE-DDS QoS 설정 (BestEffort + TransientLocal)
    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    vehicle_global_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", px4_qos,
        std::bind(&WaypointHandler::vehicleGlobalPositionCallback, this, std::placeholders::_1));

    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_qos,
        std::bind(&WaypointHandler::vehicleLocalPositionCallback, this, std::placeholders::_1));

    // OFFBOARD 모드 heartbeat 타이머 (2Hz)
    try {
        offboard_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WaypointHandler::publishOffboardControlMode, this));
    } catch (const std::runtime_error& e) {
        // executor 관련 예외는 특별히 처리
        std::string error_msg = e.what();
        if (error_msg.find("already been added to an executor") != std::string::npos) {
            RCLCPP_WARN(node_->get_logger(), "타이머 생성 실패 (executor 충돌): %s", e.what());
            RCLCPP_WARN(node_->get_logger(), "메인 스레드의 executor와 충돌했습니다. 타이머 없이 계속 진행합니다.");
            // 타이머 없이도 계속 진행 가능 (수동으로 heartbeat 전송)
        } else {
            RCLCPP_ERROR(node_->get_logger(), "타이머 생성 실패: %s", e.what());
            throw;  // 다른 예외는 다시 throw
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "타이머 생성 실패: %s", e.what());
        throw;  // 예외를 다시 throw하여 호출자에게 전달
    }

    RCLCPP_INFO(node_->get_logger(), "WaypointHandler initialized");
}

bool WaypointHandler::goToWaypoint(const GPSCoordinate& target, int timeout_ms)
{
    RCLCPP_INFO(node_->get_logger(),
                "Going to waypoint: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                target.latitude, target.longitude, target.altitude);

    // GPS 위치 정보 수신 대기
    // 중요: spin_some을 호출하지 않음 (메인 executor에서 콜백 처리)
    auto start_time = std::chrono::steady_clock::now();
    while (!global_position_received_ || !local_position_received_) {
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
        // 중단 요청 확인
        if (abort_flag_ && abort_flag_->load()) {
            RCLCPP_WARN(node_->get_logger(), "[WaypointHandler] ★ Navigation aborted by external request");
            return false;
        }

        // TrajectorySetpoint 발행
        publishTrajectorySetpoint(target_x, target_y, target_z, current_yaw_);

        // 중요: spin_some을 호출하지 않음 (메인 executor에서 콜백 처리)
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
    double horizontal_distance = gps_utils::haversineDistance(
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
    // ★ target.altitude는 이륙 지점 기준 상대 고도로 해석
    // 해발 고도(AMSL) = home_altitude_amsl + 상대 고도
    float target_altitude_amsl = home_altitude_amsl_ + target.altitude;

    RCLCPP_INFO(node_->get_logger(),
                "[GPS->NED] Target altitude: %.2fm (relative) + %.2fm (home) = %.2fm (AMSL)",
                target.altitude, home_altitude_amsl_, target_altitude_amsl);

    // gps_utils 사용하여 GPS -> Local NED 변환
    gps_utils::gpsToLocalNED(home_latitude_, home_longitude_, home_altitude_amsl_,
                             target.latitude, target.longitude, target_altitude_amsl,
                             local_x, local_y, local_z);
}

