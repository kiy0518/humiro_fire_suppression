#include "distance_adjuster.h"
#include <thread>
#include <cmath>

DistanceAdjuster::DistanceAdjuster(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Publishers 초기화
    trajectory_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    // Subscribers 초기화
    front_distance_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/lidar/front_distance", 10,
        std::bind(&DistanceAdjuster::frontDistanceCallback, this, std::placeholders::_1));

    // PX4 uXRCE-DDS QoS 설정 (BestEffort + TransientLocal)
    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_qos,
        std::bind(&DistanceAdjuster::vehicleLocalPositionCallback, this, std::placeholders::_1));

    // OFFBOARD 모드 heartbeat 타이머 (2Hz)
    try {
        offboard_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DistanceAdjuster::publishOffboardControlMode, this));
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

    RCLCPP_INFO(node_->get_logger(), "DistanceAdjuster initialized");
}

bool DistanceAdjuster::adjustDistance(float target_distance, float tolerance, int timeout_ms)
{
    target_distance_ = target_distance;
    distance_tolerance_ = tolerance;

    RCLCPP_INFO(node_->get_logger(),
                "Adjusting distance to %.2f m (tolerance: ±%.2f m)",
                target_distance, tolerance);

    // LiDAR 데이터 및 위치 정보 수신 대기
    auto start_time = std::chrono::steady_clock::now();
    while (!distance_received_ || !position_received_) {
        try {
            rclcpp::spin_some(node_);
        } catch (const std::runtime_error& e) {
            // executor 관련 예외는 무시 (이미 메인 executor에 추가된 경우)
            std::string error_msg = e.what();
            if (error_msg.find("already been added to an executor") == std::string::npos) {
                RCLCPP_WARN(node_->get_logger(), "spin_some runtime_error (무시): %s", e.what());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "spin_some 예외 (무시): %s", e.what());
        } catch (...) {
            RCLCPP_WARN(node_->get_logger(), "spin_some 알 수 없는 예외 (무시)");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > 5000) {
            if (!distance_received_) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to receive LiDAR distance data");
            }
            if (!position_received_) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to receive position data");
            }
            return false;
        }
    }

    float initial_distance = front_distance_.load();
    RCLCPP_INFO(node_->get_logger(),
                "Current front distance: %.2f m", initial_distance);

    // 시작 위치 저장
    start_x_ = current_local_x_.load();

    // 거리 조정 루프
    start_time = std::chrono::steady_clock::now();

    while (true) {
        float current_distance = front_distance_.load();
        float distance_error = current_distance - target_distance;

        // 목표 거리 도달 확인
        if (std::abs(distance_error) < tolerance) {
            RCLCPP_INFO(node_->get_logger(),
                       "Distance adjusted! Front distance: %.2f m (error: %.2f m)",
                       current_distance, distance_error);
            return true;
        }

        // 이동 거리 계산 (전방 거리가 목표보다 크면 전진, 작으면 후진)
        // NED 좌표계에서 X는 북쪽(전방)
        float movement = distance_error;  // 양수면 전진, 음수면 후진

        // 안전 제한
        if (std::abs(movement) > MAX_DISTANCE_ERROR) {
            movement = (movement > 0) ? MAX_DISTANCE_ERROR : -MAX_DISTANCE_ERROR;
            RCLCPP_WARN(node_->get_logger(),
                       "Movement limited to %.2f m for safety", MAX_DISTANCE_ERROR);
        }

        // 목표 위치 계산
        float target_x = start_x_ + movement;

        // TrajectorySetpoint 발행
        publishTrajectorySetpoint(
            target_x,
            current_local_y_,
            current_local_z_,
            current_yaw_
        );

        try {
            rclcpp::spin_some(node_);
        } catch (const std::runtime_error& e) {
            // executor 관련 예외는 무시 (이미 메인 executor에 추가된 경우)
            std::string error_msg = e.what();
            if (error_msg.find("already been added to an executor") == std::string::npos) {
                RCLCPP_WARN(node_->get_logger(), "spin_some runtime_error (무시): %s", e.what());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "spin_some 예외 (무시): %s", e.what());
        } catch (...) {
            RCLCPP_WARN(node_->get_logger(), "spin_some 알 수 없는 예외 (무시)");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 타임아웃 확인
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_ms) {
            RCLCPP_ERROR(node_->get_logger(),
                        "Distance adjustment timeout! Front distance: %.2f m (target: %.2f m)",
                        current_distance, target_distance);
            return false;
        }

        // 진행 상황 로깅 (2초마다)
        if (static_cast<int>(elapsed) % 2000 < 100) {
            RCLCPP_INFO(node_->get_logger(),
                       "Adjusting... Distance: %.2f m, Error: %.2f m, Movement: %.2f m",
                       current_distance, distance_error, movement);
        }
    }

    return false;
}

float DistanceAdjuster::getFrontDistance() const
{
    return front_distance_.load();
}

bool DistanceAdjuster::isDistanceAdjusted() const
{
    float distance_error = std::abs(front_distance_.load() - target_distance_);
    return distance_error < distance_tolerance_;
}

void DistanceAdjuster::hover()
{
    publishTrajectorySetpoint(
        current_local_x_,
        current_local_y_,
        current_local_z_,
        current_yaw_
    );
}

void DistanceAdjuster::frontDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    front_distance_ = msg->data;
    distance_received_ = true;
}

void DistanceAdjuster::vehicleLocalPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_local_x_ = msg->x;
    current_local_y_ = msg->y;
    current_local_z_ = msg->z;
    current_yaw_ = msg->heading;
    position_received_ = true;
}

void DistanceAdjuster::publishTrajectorySetpoint(float x, float y, float z, float yaw)
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

void DistanceAdjuster::publishOffboardControlMode()
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
