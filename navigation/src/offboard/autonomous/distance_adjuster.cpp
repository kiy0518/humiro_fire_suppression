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
    // 메인 executor가 백그라운드에서 콜백을 처리하므로 단순히 대기만 함
    auto start_time = std::chrono::steady_clock::now();
    while (!distance_received_ || !position_received_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

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

    // 거리 조정 루프 (Position Control 사용 - 작은 증분으로 이동)
    start_time = std::chrono::steady_clock::now();
    auto last_heartbeat_time = start_time;
    auto last_position_update = start_time;

    while (true) {
        // 중단 플래그 확인 (즉시 종료)
        if (abort_flag_ && abort_flag_->load()) {
            RCLCPP_WARN(node_->get_logger(), "[DistanceAdjuster] ★ ABORT requested - exiting loop immediately!");
            return false;
        }

        float current_distance = front_distance_.load();
        float distance_error = current_distance - target_distance;

        // 현재 시간 (루프 전체에서 사용)
        auto now = std::chrono::steady_clock::now();

        // Velocity control update (100ms마다)
        auto velocity_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_position_update).count();

        if (velocity_elapsed >= 100) {
            // 현재 상태 가져오기
            float current_yaw = current_yaw_.load();
            float current_z = current_local_z_.load();
            float current_distance = front_distance_.load();

            // Body Frame 속도 계산 (비례 제어로 부드러운 거리 유지)
            // distance_error가 클수록 빠르게 이동, 작을수록 천천히 이동
            float body_velocity_x;

            if (std::abs(distance_error) < tolerance) {
                // 목표 범위 내: 미세 조정 (비례 제어, P gain = 0.5)
                body_velocity_x = distance_error * 0.5f;

                // 속도 제한 (±0.05 m/s, 부드러운 유지)
                body_velocity_x = std::max(-0.05f, std::min(0.05f, body_velocity_x));
            } else {
                // 목표 범위 밖: 일정 속도로 접근
                body_velocity_x = (distance_error > 0) ? APPROACH_SPEED : -APPROACH_SPEED;
            }

            // Body Frame → NED Frame 변환
            // NED에서: X=North, Y=East, Yaw=0이 북쪽
            float ned_velocity_x = body_velocity_x * std::cos(current_yaw);  // 북쪽 성분
            float ned_velocity_y = body_velocity_x * std::sin(current_yaw);  // 동쪽 성분

            // 고도 유지 (PID 제어)
            // 목표 고도: -1.0m (NED), 현재 고도: current_z
            float altitude_error = -1.0f - current_z;  // 양수: 상승 필요, 음수: 하강 필요
            float ned_velocity_z = altitude_error * 0.5f;  // 비례 제어 (P gain = 0.5)

            // 속도 제한 (안전)
            ned_velocity_z = std::max(-0.5f, std::min(0.5f, ned_velocity_z));  // ±0.5 m/s

            // 디버그 출력 (500ms마다)
            static auto last_debug_time = std::chrono::steady_clock::now();
            auto debug_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debug_time).count();
            if (debug_elapsed >= 500) {
                RCLCPP_INFO(node_->get_logger(),
                           "[OFFBOARD] Dist: %.2fm | Alt: %.2fm (err: %.2fm) | Yaw: %.1f° | "
                           "Body Vel: %.2fm/s | NED Vel: [%.2f, %.2f, %.2f]",
                           current_distance,
                           -current_z,  // NED Z는 음수이므로 양수로 변환
                           altitude_error,
                           current_yaw * 180.0f / M_PI,  // 라디안 → 도
                           body_velocity_x,
                           ned_velocity_x, ned_velocity_y, ned_velocity_z);
                last_debug_time = now;
            }

            // Velocity setpoint 발행 (NED Frame 기준, 고도 제어 포함)
            publishVelocitySetpoint(ned_velocity_x, ned_velocity_y, ned_velocity_z, 0.0f);

            last_position_update = now;
        }

        // OFFBOARD heartbeat 수동 전송 (500ms마다)
        auto heartbeat_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_time).count();
        if (heartbeat_elapsed >= 500) {
            publishOffboardControlMode();
            last_heartbeat_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 타임아웃 확인
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_ms) {
            // 타임아웃 도달 - 거리 유지 완료
            RCLCPP_INFO(node_->get_logger(),
                        "Distance maintain completed! Front distance: %.2f m (target: %.2f m)",
                        current_distance, target_distance);
            return true;  // 성공으로 처리
        }

        // 진행 상황 로깅 (2초마다)
        if (static_cast<int>(elapsed) % 2000 < 100) {
            RCLCPP_INFO(node_->get_logger(),
                       "Maintaining distance... Current: %.2f m, Target: %.2f m, Error: %.2f m",
                       current_distance, target_distance, distance_error);
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
    // ★★★ OFFBOARD 모드 유지를 위한 heartbeat 발행 (2Hz 이상 필수) ★★★
    publishOffboardControlMode();

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

void DistanceAdjuster::publishVelocitySetpoint(float vx, float vy, float vz, float yaw_rate)
{
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;

    // Velocity setpoint (NED Frame 기준)
    msg.velocity[0] = vx;  // 북쪽 속도 (North)
    msg.velocity[1] = vy;  // 동쪽 속도 (East)
    msg.velocity[2] = vz;  // 하방 속도 (Down)

    msg.yawspeed = yaw_rate;

    // Position은 NaN으로 설정 (사용 안 함)
    msg.position[0] = std::nanf("");
    msg.position[1] = std::nanf("");
    msg.position[2] = std::nanf("");

    msg.yaw = std::nanf("");

    // Acceleration, Jerk도 NaN
    msg.acceleration[0] = std::nanf("");
    msg.acceleration[1] = std::nanf("");
    msg.acceleration[2] = std::nanf("");

    msg.jerk[0] = std::nanf("");
    msg.jerk[1] = std::nanf("");
    msg.jerk[2] = std::nanf("");

    trajectory_setpoint_pub_->publish(msg);
}

void DistanceAdjuster::publishOffboardControlMode()
{
    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = true;   // Velocity control 활성화
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);
}

// ========== 비동기 제어 함수 (상태 머신용) ==========

bool DistanceAdjuster::publishDistanceSetpoint(float target_distance)
{
    if (!distance_received_ || !position_received_) {
        // 데이터 없으면 현재 위치 유지
        hover();
        return false;
    }

    float current_distance = front_distance_.load();
    float distance_error = current_distance - target_distance;

    // 목표 도달 확인 (0.3m 이내면 완료)
    if (std::abs(distance_error) < 0.3f) {
        hover();  // 위치 유지
        return true;  // 목표 도달
    }

    // ★ 부드러운 속도 제어 (낮은 비례 게인 + 속도 제한)
    // 비례 게인: 0.05 (부드러운 접근)
    // 최대 속도: 0.08 m/s (실내 안전 속도)
    const float SMOOTH_GAIN = 0.05f;
    const float MAX_SPEED = 0.08f;
    const float DEAD_ZONE = 0.2f;  // 데드존 (이 이내면 정지)

    float body_velocity_x = 0.0f;

    if (std::abs(distance_error) > DEAD_ZONE) {
        // 비례 제어로 속도 계산
        body_velocity_x = distance_error * SMOOTH_GAIN;

        // 속도 제한
        if (body_velocity_x > MAX_SPEED) body_velocity_x = MAX_SPEED;
        if (body_velocity_x < -MAX_SPEED) body_velocity_x = -MAX_SPEED;
    }

    // Body Frame → NED Frame 변환
    float current_yaw = current_yaw_.load();
    float ned_velocity_x = body_velocity_x * std::cos(current_yaw);
    float ned_velocity_y = body_velocity_x * std::sin(current_yaw);

    // 고도 유지 (현재 고도 유지 - 속도 0)
    float ned_velocity_z = 0.0f;

    // OFFBOARD 모드 발행
    publishOffboardControlMode();

    // Velocity setpoint 발행
    publishVelocitySetpoint(ned_velocity_x, ned_velocity_y, ned_velocity_z, 0.0f);

    return false;  // 아직 조절 중
}
