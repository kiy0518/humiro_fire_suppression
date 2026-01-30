#include "takeoff_handler.h"
#include <thread>
#define _USE_MATH_DEFINES
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
    try {
        offboard_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TakeoffHandler::publishOffboardControlMode, this));
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

    RCLCPP_INFO(node_->get_logger(), "TakeoffHandler initialized");
}

bool TakeoffHandler::takeoff(float altitude_m, int timeout_ms)
{
    RCLCPP_INFO(node_->get_logger(), "Starting takeoff to %.2f meters", altitude_m);

    // 위치 정보 수신 대기 (메인 executor가 자동으로 콜백 처리)
    auto start_time = std::chrono::steady_clock::now();
    while (!position_received_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > 5000) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to receive position data");
            return false;
        }
    }

    // 이륙 시작 위치 저장 (X, Y, Yaw 고정하여 드리프트 방지)
    takeoff_start_altitude_ = current_altitude_;
    takeoff_start_x_ = current_x_;
    takeoff_start_y_ = current_y_;
    takeoff_start_yaw_ = current_yaw_;
    target_altitude_ = -altitude_m;  // NED 좌표계: 위쪽이 음수

    RCLCPP_INFO(node_->get_logger(),
                "Current altitude: %.2f m (NED), Target: %.2f m (NED)",
                -current_altitude_.load(), -target_altitude_);
    RCLCPP_INFO(node_->get_logger(),
                "Starting position fixed: X=%.2f, Y=%.2f, Yaw=%.2f",
                takeoff_start_x_, takeoff_start_y_, takeoff_start_yaw_);

    // OFFBOARD 모드로 이륙 (TrajectorySetpoint 지속 발행)
    start_time = std::chrono::steady_clock::now();

    while (true) {
        // 중단 요청 확인
        if (abort_flag_ && abort_flag_->load()) {
            RCLCPP_WARN(node_->get_logger(), "[TakeoffHandler] ★ Takeoff aborted by external request");
            return false;
        }

        // 이륙 시작 위치 고정 (X, Y, Yaw), Z만 목표 고도로
        publishTrajectorySetpoint(
            takeoff_start_x_,      // ← 고정된 X 위치
            takeoff_start_y_,      // ← 고정된 Y 위치
            target_altitude_,
            takeoff_start_yaw_     // ← 고정된 헤딩
        );

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

void TakeoffHandler::stopOffboardHeartbeat()
{
    if (offboard_timer_) {
        offboard_timer_->cancel();
        offboard_timer_.reset();
        RCLCPP_INFO(node_->get_logger(), "[TakeoffHandler] OFFBOARD heartbeat stopped");
    }
}

void TakeoffHandler::startHoldPositionSetpoint()
{
    RCLCPP_INFO(node_->get_logger(), "[TakeoffHandler] Starting hold-position setpoint publishing...");

    // 위치 정보 수신 대기 (최대 5초)
    auto start_time = std::chrono::steady_clock::now();
    while (!position_received_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > 5000) {
            RCLCPP_WARN(node_->get_logger(), "[TakeoffHandler] Timeout waiting for position data, using default values");
            break;
        }
    }

    // 현재 위치를 이륙 시작 위치로 저장 (45도 회전 방지)
    takeoff_start_x_ = current_x_.load();
    takeoff_start_y_ = current_y_.load();
    takeoff_start_altitude_ = current_altitude_.load();
    takeoff_start_yaw_ = current_yaw_.load();
    target_altitude_ = takeoff_start_altitude_;  // 현재 고도 유지

    RCLCPP_INFO(node_->get_logger(),
                "[TakeoffHandler] Hold position captured: X=%.2f, Y=%.2f, Z=%.2f, Yaw=%.2f (%.1f deg)",
                takeoff_start_x_, takeoff_start_y_, takeoff_start_altitude_, takeoff_start_yaw_,
                takeoff_start_yaw_ * 180.0f / M_PI);

    // 첫 번째 setpoint 즉시 발행 (OFFBOARD 모드 진입 전 필수)
    // ★ OffboardControlMode + TrajectorySetpoint 둘 다 발행해야 함
    for (int i = 0; i < 10; i++) {
        publishOffboardControlMode();  // ★ 추가: OFFBOARD heartbeat
        publishTrajectorySetpoint(
            takeoff_start_x_,
            takeoff_start_y_,
            takeoff_start_altitude_,
            takeoff_start_yaw_
        );
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node_->get_logger(), "[TakeoffHandler] Hold-position setpoint publishing started");
}

bool TakeoffHandler::returnToTakeoffPosition(int timeout_ms)
{
    RCLCPP_INFO(node_->get_logger(), "[TakeoffHandler] Returning to takeoff position...");
    RCLCPP_INFO(node_->get_logger(),
                "  Target: X=%.2f, Y=%.2f, Z=%.2f, Yaw=%.2f",
                takeoff_start_x_, takeoff_start_y_, target_altitude_, takeoff_start_yaw_);

    auto start_time = std::chrono::steady_clock::now();
    const float POSITION_THRESHOLD = 0.5f;  // 50cm 오차 허용
    const float YAW_THRESHOLD = 0.17f;      // 약 10도 오차 허용

    while (true) {
        // 중단 요청 확인
        if (abort_flag_ && abort_flag_->load()) {
            RCLCPP_WARN(node_->get_logger(), "[TakeoffHandler] ★ Return aborted by external request");
            return false;
        }

        // 이륙 시작 위치로 setpoint 발행
        publishTrajectorySetpoint(
            takeoff_start_x_,
            takeoff_start_y_,
            target_altitude_,    // 현재 고도 유지
            takeoff_start_yaw_
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 현재 위치와 목표 위치 비교
        float dx = current_x_.load() - takeoff_start_x_;
        float dy = current_y_.load() - takeoff_start_y_;
        float distance_error = std::sqrt(dx * dx + dy * dy);

        float yaw_error = std::abs(current_yaw_.load() - takeoff_start_yaw_);
        // Yaw는 순환 값이므로 2π를 고려
        if (yaw_error > M_PI) {
            yaw_error = 2.0f * M_PI - yaw_error;
        }

        // 위치 도달 확인
        if (distance_error < POSITION_THRESHOLD && yaw_error < YAW_THRESHOLD) {
            RCLCPP_INFO(node_->get_logger(),
                        "[TakeoffHandler] Returned to takeoff position! Distance error: %.2f m, Yaw error: %.1f deg",
                        distance_error, yaw_error * 180.0f / M_PI);
            return true;
        }

        // 타임아웃 확인
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_ms) {
            RCLCPP_ERROR(node_->get_logger(),
                        "[TakeoffHandler] Return to takeoff position timeout! Distance error: %.2f m",
                        distance_error);
            return false;
        }

        // 진행 상황 로깅 (2초마다)
        if (static_cast<int>(elapsed) % 2000 < 100) {
            RCLCPP_INFO(node_->get_logger(),
                        "  Returning... Distance error: %.2f m, Yaw error: %.1f deg",
                        distance_error, yaw_error * 180.0f / M_PI);
        }
    }

    return false;
}

bool TakeoffHandler::isTakeoffComplete() const
{
    float altitude_error = std::abs(current_altitude_ - target_altitude_);
    return altitude_error < ALTITUDE_THRESHOLD;
}

void TakeoffHandler::hover()
{
    // ★★★ OFFBOARD 모드 유지를 위한 heartbeat 발행 (2Hz 이상 필수) ★★★
    publishOffboardControlMode();

    publishTrajectorySetpoint(
        takeoff_start_x_,      // 이륙 시작 위치 고정
        takeoff_start_y_,      // 이륙 시작 위치 고정
        target_altitude_,      // 목표 고도 유지 (이륙 완료 고도)
        takeoff_start_yaw_     // 이륙 시작 헤딩 고정
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

// ========== 비동기 제어 함수 (상태 머신용) ==========

void TakeoffHandler::startTakeoff(float altitude_m)
{
    // 목표 고도 설정 (NED 좌표계: 아래가 양수이므로 음수로 변환)
    target_altitude_ = -std::abs(altitude_m);

    // 현재 위치를 이륙 시작점으로 저장
    if (position_received_) {
        takeoff_start_x_ = current_x_.load();
        takeoff_start_y_ = current_y_.load();
        takeoff_start_altitude_ = current_altitude_.load();
        takeoff_start_yaw_ = current_yaw_.load();
    }

    RCLCPP_INFO(node_->get_logger(),
                "[TakeoffHandler] Takeoff started - target altitude: %.1fm (NED Z: %.1f)",
                altitude_m, target_altitude_);
}

void TakeoffHandler::publishTakeoffSetpoint()
{
    // OFFBOARD 제어 모드 발행
    publishOffboardControlMode();

    // 이륙 시작 위치(X, Y)를 유지하면서 목표 고도로 상승
    publishTrajectorySetpoint(
        takeoff_start_x_,
        takeoff_start_y_,
        target_altitude_,
        takeoff_start_yaw_
    );
}

void TakeoffHandler::publishLandingSetpoint()
{
    // OFFBOARD 제어 모드 발행
    publishOffboardControlMode();

    // ★ 현재 위치, 현재 헤딩 유지하면서 하강
    float current_x = current_x_.load();
    float current_y = current_y_.load();
    float current_z = current_altitude_.load();
    float current_yaw = current_yaw_.load();

    // 하강 속도: 0.02m/tick * 20Hz = 0.4m/s
    float target_z = current_z + 0.02f;  // NED: 양수가 아래 방향

    // 지면 아래로 가지 않도록 제한
    if (target_z > 0.0f) {
        target_z = 0.0f;
    }

    publishTrajectorySetpoint(
        current_x,      // 현재 X 위치
        current_y,      // 현재 Y 위치
        target_z,       // 하강 목표 고도
        current_yaw     // 현재 헤딩 유지
    );
}
