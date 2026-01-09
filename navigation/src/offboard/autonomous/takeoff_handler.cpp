#include "takeoff_handler.h"
#include <thread>
#include <cmath>
#include <cstdlib>

TakeoffHandler::TakeoffHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // DRONE_ID 환경 변수에서 target_system 읽기 (device_config.env에서 설정됨)
    target_system_ = 1;  // 기본값
    const char* drone_id_env = std::getenv("DRONE_ID");
    if (drone_id_env) {
        try {
            int drone_id = std::stoi(drone_id_env);
            if (drone_id >= 1 && drone_id <= 255) {
                target_system_ = static_cast<uint8_t>(drone_id);
                RCLCPP_INFO(node_->get_logger(), "TakeoffHandler: DRONE_ID 환경 변수 사용: target_system=%d", 
                          static_cast<int>(target_system_));
            } else {
                RCLCPP_WARN(node_->get_logger(), "TakeoffHandler: 잘못된 DRONE_ID 값: %d (1-255 범위 필요), 기본값 1 사용", drone_id);
            }
        } catch (...) {
            RCLCPP_WARN(node_->get_logger(), "TakeoffHandler: DRONE_ID 환경 변수 파싱 실패, 기본값 1 사용");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "TakeoffHandler: DRONE_ID 환경 변수 없음, 기본값 1 사용");
    }
    // Publishers 초기화
    trajectory_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // Subscribers 초기화
    // PX4 uXRCE-DDS QoS 설정 (BestEffort + TransientLocal)
    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_qos,
        std::bind(&TakeoffHandler::vehicleLocalPositionCallback, this, std::placeholders::_1));

    // OFFBOARD 모드 heartbeat 타이머 (2Hz)
    offboard_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TakeoffHandler::publishOffboardControlMode, this));

    RCLCPP_INFO(node_->get_logger(), "TakeoffHandler initialized");
}

bool TakeoffHandler::takeoff(float altitude_m, int timeout_ms)
{
    RCLCPP_INFO(node_->get_logger(), "Starting takeoff to %.2f meters", altitude_m);

    // 위치 정보 수신 대기
    auto start_time = std::chrono::steady_clock::now();
    while (!position_received_) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > 5000) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to receive position data");
            return false;
        }
    }

    // 이륙 시작 위치 저장
    takeoff_start_altitude_ = current_altitude_;
    target_altitude_ = -altitude_m;  // NED 좌표계: 위쪽이 음수

    RCLCPP_INFO(node_->get_logger(),
                "Current altitude: %.2f m (NED), Target: %.2f m (NED)",
                -current_altitude_.load(), -target_altitude_);

    // OFFBOARD 모드로 이륙 (TrajectorySetpoint 지속 발행)
    start_time = std::chrono::steady_clock::now();

    while (true) {
        // 현재 XY 위치 유지, Z만 목표 고도로
        publishTrajectorySetpoint(
            current_x_,
            current_y_,
            target_altitude_,
            current_yaw_
        );

        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 고도 도달 확인
        float altitude_error = std::abs(current_altitude_ - target_altitude_);

        if (altitude_error < ALTITUDE_THRESHOLD) {
            RCLCPP_INFO(node_->get_logger(),
                        "Takeoff complete! Altitude: %.2f m, Error: %.2f m",
                        -current_altitude_.load(), altitude_error);
            return true;
        }

        // 타임아웃 확인
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_ms) {
            RCLCPP_ERROR(node_->get_logger(),
                        "Takeoff timeout! Current altitude: %.2f m, Target: %.2f m",
                        -current_altitude_.load(), -target_altitude_);
            return false;
        }

        // 진행 상황 로깅 (2초마다)
        if (static_cast<int>(elapsed) % 2000 < 100) {
            RCLCPP_INFO(node_->get_logger(),
                        "Climbing... Current: %.2f m, Target: %.2f m, Error: %.2f m",
                        -current_altitude_.load(), -target_altitude_, altitude_error);
        }
    }

    return false;
}

float TakeoffHandler::getCurrentAltitude() const
{
    return -current_altitude_.load();  // NED를 일반 고도로 변환 (양수 = 위)
}

bool TakeoffHandler::isTakeoffComplete() const
{
    float altitude_error = std::abs(current_altitude_ - target_altitude_);
    return altitude_error < ALTITUDE_THRESHOLD;
}

void TakeoffHandler::hover()
{
    RCLCPP_INFO(node_->get_logger(), "Hovering at current position");

    publishTrajectorySetpoint(
        current_x_,
        current_y_,
        current_altitude_,  // 현재 고도 유지
        current_yaw_
    );
}

void TakeoffHandler::vehicleLocalPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_altitude_ = msg->z;  // NED: 아래가 양수
    current_yaw_ = msg->heading;
    position_received_ = true;
}

void TakeoffHandler::publishTrajectorySetpoint(float x, float y, float z, float yaw)
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

void TakeoffHandler::publishOffboardControlMode()
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

void TakeoffHandler::publishVehicleCommand(
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
    msg.target_system = target_system_;
    msg.target_component = 1;
    msg.source_system = target_system_;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
}
