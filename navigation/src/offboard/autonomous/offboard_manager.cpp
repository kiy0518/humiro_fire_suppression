#include "offboard_manager.h"
#include <thread>
#include <chrono>

OffboardManager::OffboardManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Handlers 초기화
    arm_handler_ = std::make_unique<ArmHandler>(node);
    takeoff_handler_ = std::make_unique<TakeoffHandler>(node);
    waypoint_handler_ = std::make_unique<WaypointHandler>(node);
    distance_adjuster_ = std::make_unique<DistanceAdjuster>(node);
    rtl_handler_ = std::make_unique<RTLHandler>(node);

    // 중단 플래그 연결 (DistanceAdjuster가 루프 중에 확인)
    distance_adjuster_->setAbortFlag(&abort_requested_);

    // testMission3용 구독자 초기화
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    
    gps_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position",
        qos,
        std::bind(&OffboardManager::onGpsPosition, this, std::placeholders::_1));
    
    attitude_sub_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        qos,
        std::bind(&OffboardManager::onAttitude, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "OffboardManager initialized");
}

bool OffboardManager::executeMission(const MissionConfig& config)
{
    try {
        // 이미 미션이 실행 중이면 중복 실행 방지
        if (current_state_ != MissionState::IDLE) {
            RCLCPP_WARN(node_->get_logger(), 
                       "Mission already running (current state: %s). Ignoring new mission request.",
                       getStateName(current_state_).c_str());
            return false;
        }

        mission_config_ = config;

        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  Starting Autonomous Mission");
        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "Mission config:");
        RCLCPP_INFO(node_->get_logger(), "  Takeoff altitude: %.2f m", config.takeoff_altitude);
        RCLCPP_INFO(node_->get_logger(), "  Target waypoint: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                    config.target_waypoint.latitude,
                    config.target_waypoint.longitude,
                    config.target_waypoint.altitude);
        RCLCPP_INFO(node_->get_logger(), "  Target distance: %.2f m (±%.2f m)",
                    config.target_distance, config.distance_tolerance);
        RCLCPP_INFO(node_->get_logger(), "======================================\n");

        // === State: ARMING ===
        if (!transitionToState(MissionState::ARMING)) {
            handleError("Failed to transition to ARMING state");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[ARMING] Enabling OFFBOARD mode...");
        try {
            // 중요: OFFBOARD 모드 활성화 전에 현재 위치 setpoint 발행 시작
            // 이렇게 하지 않으면 OFFBOARD 모드 진입 시 yaw가 정의되지 않아 45도 회전 발생
            RCLCPP_INFO(node_->get_logger(), "[ARMING] Starting hold-position setpoint before OFFBOARD mode...");
            takeoff_handler_->startHoldPositionSetpoint();

            if (!arm_handler_->enableOffboardMode()) {
                handleError("Failed to enable OFFBOARD mode");
                return false;
            }
        } catch (const std::runtime_error& e) {
            std::string error_msg = e.what();
            if (error_msg.find("already been added to an executor") != std::string::npos) {
                RCLCPP_WARN(node_->get_logger(), "[ARMING] enableOffboardMode executor 충돌 (계속 진행): %s", e.what());
                // executor 충돌은 치명적이지 않으므로 계속 진행
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[ARMING] enableOffboardMode 실패: %s", e.what());
                handleError("Failed to enable OFFBOARD mode");
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[ARMING] enableOffboardMode 예외: %s", e.what());
            handleError("Failed to enable OFFBOARD mode");
            return false;
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "[ARMING] enableOffboardMode 알 수 없는 예외 발생");
            handleError("Failed to enable OFFBOARD mode");
            return false;
        }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node_->get_logger(), "[ARMING] Arming vehicle...");
    if (!arm_handler_->arm()) {
        handleError("Failed to arm vehicle");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[ARMING] Vehicle armed successfully\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // === State: TAKEOFF ===
    if (!transitionToState(MissionState::TAKEOFF)) {
        handleError("Failed to transition to TAKEOFF state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[TAKEOFF] Taking off to %.2f m...", config.takeoff_altitude);
    if (!takeoff_handler_->takeoff(config.takeoff_altitude)) {
        handleError("Takeoff failed");
        emergencyRTL();
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[TAKEOFF] Takeoff successful! Altitude: %.2f m\n",
                takeoff_handler_->getCurrentAltitude());
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // === State: NAVIGATE ===
    if (!transitionToState(MissionState::NAVIGATE)) {
        handleError("Failed to transition to NAVIGATE state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[NAVIGATE] Navigating to waypoint...");
    if (!waypoint_handler_->goToWaypoint(config.target_waypoint)) {
        handleError("Waypoint navigation failed");
        emergencyRTL();
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[NAVIGATE] Waypoint reached!\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // === State: ADJUST_DISTANCE ===
    if (!transitionToState(MissionState::ADJUST_DISTANCE)) {
        handleError("Failed to transition to ADJUST_DISTANCE state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[ADJUST_DISTANCE] Adjusting distance to %.2f m...",
                config.target_distance);
    if (!distance_adjuster_->adjustDistance(config.target_distance, config.distance_tolerance)) {
        handleError("Distance adjustment failed");
        emergencyRTL();
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[ADJUST_DISTANCE] Distance adjusted! Front distance: %.2f m\n",
                distance_adjuster_->getFrontDistance());
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // === State: HOVER ===
    if (!transitionToState(MissionState::HOVER)) {
        handleError("Failed to transition to HOVER state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[HOVER] Hovering for %.1f seconds...", config.hover_duration_sec);
    int hover_iterations = static_cast<int>(config.hover_duration_sec * 2);  // 2Hz
    for (int i = 0; i < hover_iterations; i++) {
        distance_adjuster_->hover();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (i % 4 == 0) {  // 2초마다 로깅
            RCLCPP_INFO(node_->get_logger(), "[HOVER] Distance: %.2f m",
                       distance_adjuster_->getFrontDistance());
        }
    }
    RCLCPP_INFO(node_->get_logger(), "[HOVER] Hover complete\n");

    // === State: RTL ===
    if (!transitionToState(MissionState::RTL)) {
        handleError("Failed to transition to RTL state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[RTL] Returning to launch...");
    if (!rtl_handler_->returnToLaunch()) {
        handleError("RTL failed");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[RTL] RTL complete! Vehicle landed\n");
    
    // OFFBOARD 모드 비활성화 (heartbeat 중단)
    // 중요: heartbeat 중단 시 PX4가 자동으로 다른 모드로 전환되어
    // QGC나 다른 GCS에서 비행 모드 변경 가능
    RCLCPP_INFO(node_->get_logger(), "[RTL] Disabling OFFBOARD mode (heartbeat stop)...");
    disableOffboardMode();

    // === State: LANDED ===
    transitionToState(MissionState::LANDED);

    RCLCPP_INFO(node_->get_logger(), "======================================");
    RCLCPP_INFO(node_->get_logger(), "  Mission Complete!");
    RCLCPP_INFO(node_->get_logger(), "======================================");

    return true;
    } catch (const std::runtime_error& e) {
        std::string error_msg = e.what();
        if (error_msg.find("already been added to an executor") != std::string::npos) {
            RCLCPP_WARN(node_->get_logger(), "[executeMission] executor 충돌 (계속 진행): %s", e.what());
            // executor 충돌은 치명적이지 않으므로 계속 진행
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "[executeMission] runtime_error: %s", e.what());
            handleError("Mission execution failed");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[executeMission] 예외: %s", e.what());
        handleError("Mission execution failed");
        return false;
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "[executeMission] 알 수 없는 예외 발생");
        handleError("Mission execution failed");
        return false;
    }
}

bool OffboardManager::testMission(float takeoff_altitude, float hover_duration)
{
    try {
        // 이미 미션이 실행 중이면 중복 실행 방지
        if (current_state_ != MissionState::IDLE) {
            RCLCPP_WARN(node_->get_logger(),
                       "[testMission] Mission already running (current state: %s). Ignoring new mission request.",
                       getStateName(current_state_).c_str());
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  실내 테스트 미션 시작 (GPS 불필요)");
        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  시퀀스: OFFBOARD -> ARM -> TAKEOFF %.1fm -> HOVER %.0fs -> LAND",
                    takeoff_altitude, hover_duration);
        RCLCPP_INFO(node_->get_logger(), "======================================\n");

        // === State: ARMING ===
        if (!transitionToState(MissionState::ARMING)) {
            handleError("Failed to transition to ARMING state");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission] Step 1: OFFBOARD 모드 활성화");

        // 중요: OFFBOARD 모드 활성화 전에 현재 위치 setpoint 발행 시작 (45도 회전 방지)
        RCLCPP_INFO(node_->get_logger(), "[testMission] Starting hold-position setpoint before OFFBOARD mode...");
        takeoff_handler_->startHoldPositionSetpoint();

        if (!arm_handler_->enableOffboardMode()) {
            handleError("Failed to enable OFFBOARD mode");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission] ✓ OFFBOARD 모드 활성화 성공\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(node_->get_logger(), "[testMission] Step 2: ARM 시동 걸기");
        if (!arm_handler_->arm()) {
            handleError("Failed to arm vehicle");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission] ✓ ARM 성공\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // === State: TAKEOFF ===
        if (!transitionToState(MissionState::TAKEOFF)) {
            handleError("Failed to transition to TAKEOFF state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission] Step 3: 이륙 시작, 목표 고도: %.1fm", takeoff_altitude);
        if (!takeoff_handler_->takeoff(takeoff_altitude)) {
            handleError("Takeoff failed");
            emergencyRTL();
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission] ✓ 이륙 완료, 고도: %.2fm\n",
                    takeoff_handler_->getCurrentAltitude());

        // === State: HOVER ===
        if (!transitionToState(MissionState::HOVER)) {
            handleError("Failed to transition to HOVER state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission] Step 4: 호버링 %.0f초", hover_duration);
        int hover_iterations = static_cast<int>(hover_duration);
        for (int i = 1; i <= hover_iterations; i++) {
            RCLCPP_INFO(node_->get_logger(), "[testMission]   호버링 중... %d/%d초", i, hover_iterations);
            takeoff_handler_->hover();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission] ✓ 호버링 완료\n");

        // === State: RTL (착륙) ===
        if (!transitionToState(MissionState::RTL)) {
            handleError("Failed to transition to RTL state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission] Step 5: 착륙 시작");
        if (!rtl_handler_->land()) {
            handleError("Landing failed");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission] ✓ 착륙 완료\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // === State: IDLE ===
        RCLCPP_INFO(node_->get_logger(), "[testMission] Step 6: OFFBOARD 모드 비활성화");
        disableOffboardMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!transitionToState(MissionState::IDLE)) {
            RCLCPP_WARN(node_->get_logger(), "[testMission] Failed to transition to IDLE");
        }

        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  테스트 미션 완료!");
        RCLCPP_INFO(node_->get_logger(), "======================================\n");

        return true;

    } catch (const std::runtime_error& e) {
        std::string error_msg = e.what();
        if (error_msg.find("already been added to an executor") != std::string::npos) {
            RCLCPP_WARN(node_->get_logger(), "[testMission] executor 충돌 (계속 진행): %s", e.what());
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "[testMission] runtime_error: %s", e.what());
            handleError("Test mission failed");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[testMission] 예외: %s", e.what());
        handleError("Test mission failed");
        return false;
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "[testMission] 알 수 없는 예외 발생");
        handleError("Test mission failed");
        return false;
    }
}

bool OffboardManager::testMission2(float takeoff_altitude, float hover_duration,
                                    float target_distance, float movement_speed)
{
    try {
        // 이미 미션이 실행 중이면 중복 실행 방지
        if (current_state_ != MissionState::IDLE) {
            RCLCPP_WARN(node_->get_logger(),
                       "[testMission2] Mission already running (current state: %s). Ignoring new mission request.",
                       getStateName(current_state_).c_str());
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  실내 테스트 미션 2 - LiDAR 기반 전진/후진");
        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  시퀀스: OFFBOARD -> ARM -> TAKEOFF %.1fm -> HOVER %.0fs",
                    takeoff_altitude, hover_duration);
        RCLCPP_INFO(node_->get_logger(), "         -> FORWARD(%.1fm, %.2fm/s) -> HOVER %.0fs",
                    target_distance, movement_speed, hover_duration);
        RCLCPP_INFO(node_->get_logger(), "         -> BACKWARD(원래위치, %.2fm/s) -> HOVER %.0fs -> LAND",
                    movement_speed, hover_duration);
        RCLCPP_INFO(node_->get_logger(), "======================================\n");

        // === State: ARMING ===
        if (!transitionToState(MissionState::ARMING)) {
            handleError("Failed to transition to ARMING state");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 1: OFFBOARD 모드 활성화");

        // 중요: OFFBOARD 모드 활성화 전에 현재 위치 setpoint 발행 시작 (45도 회전 방지)
        RCLCPP_INFO(node_->get_logger(), "[testMission2] Starting hold-position setpoint before OFFBOARD mode...");
        takeoff_handler_->startHoldPositionSetpoint();

        if (!arm_handler_->enableOffboardMode()) {
            handleError("Failed to enable OFFBOARD mode");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ OFFBOARD 모드 활성화 성공\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 2: ARM 시동 걸기");
        if (!arm_handler_->arm()) {
            handleError("Failed to arm vehicle");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ ARM 성공\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // === State: TAKEOFF ===
        if (!transitionToState(MissionState::TAKEOFF)) {
            handleError("Failed to transition to TAKEOFF state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 3: 이륙 시작, 목표 고도: %.1fm", takeoff_altitude);
        if (!takeoff_handler_->takeoff(takeoff_altitude)) {
            handleError("Takeoff failed");
            emergencyRTL();
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ 이륙 완료, 고도: %.2fm\n",
                    takeoff_handler_->getCurrentAltitude());

        // === State: HOVER (첫 번째) ===
        if (!transitionToState(MissionState::HOVER)) {
            handleError("Failed to transition to HOVER state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 4: 호버링 %.0f초 (이륙 후)", hover_duration);
        int hover_iterations = static_cast<int>(hover_duration);
        for (int i = 1; i <= hover_iterations; i++) {
            RCLCPP_INFO(node_->get_logger(), "[testMission2]   호버링 중... %d/%d초", i, hover_iterations);
            takeoff_handler_->hover();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ 호버링 완료\n");

        // === State: ADJUST_DISTANCE (전진) ===
        if (!transitionToState(MissionState::ADJUST_DISTANCE)) {
            handleError("Failed to transition to ADJUST_DISTANCE state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 5: LiDAR 기반 전진 - 목표 거리 %.1fm (속도: %.2fm/s)",
                    target_distance, movement_speed);

        // 현재 전방 거리 저장 (복귀용)
        float initial_front_distance = distance_adjuster_->getFrontDistance();
        RCLCPP_INFO(node_->get_logger(), "[testMission2]   초기 전방 거리: %.2fm", initial_front_distance);

        // 거리 허용 오차: 0.2m
        float distance_tolerance = 0.2f;

        if (!distance_adjuster_->adjustDistance(target_distance, distance_tolerance, 30000)) {
            RCLCPP_WARN(node_->get_logger(), "[testMission2] ⚠ 전진 실패 또는 타임아웃");
            emergencyRTL();
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ 전진 완료, 현재 전방 거리: %.2fm\n",
                    distance_adjuster_->getFrontDistance());

        // === State: HOVER (두 번째) ===
        if (!transitionToState(MissionState::HOVER)) {
            handleError("Failed to transition to HOVER state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 6: 호버링 %.0f초 (전진 후)", hover_duration);
        for (int i = 1; i <= hover_iterations; i++) {
            RCLCPP_INFO(node_->get_logger(), "[testMission2]   호버링 중... %d/%d초", i, hover_iterations);
            takeoff_handler_->hover();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ 호버링 완료\n");

        // === State: ADJUST_DISTANCE (후진 - 원래 위치) ===
        if (!transitionToState(MissionState::ADJUST_DISTANCE)) {
            handleError("Failed to transition to ADJUST_DISTANCE state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 7: LiDAR 기반 후진 - 원래 거리 %.2fm로 복귀 (속도: %.2fm/s)",
                    initial_front_distance, movement_speed);

        if (!distance_adjuster_->adjustDistance(initial_front_distance, distance_tolerance, 30000)) {
            RCLCPP_WARN(node_->get_logger(), "[testMission2] ⚠ 후진 실패 또는 타임아웃");
            emergencyRTL();
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ 후진 완료, 현재 전방 거리: %.2fm\n",
                    distance_adjuster_->getFrontDistance());

        // === State: HOVER (세 번째) ===
        if (!transitionToState(MissionState::HOVER)) {
            handleError("Failed to transition to HOVER state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 8: 호버링 %.0f초 (후진 후)", hover_duration);
        for (int i = 1; i <= hover_iterations; i++) {
            RCLCPP_INFO(node_->get_logger(), "[testMission2]   호버링 중... %d/%d초", i, hover_iterations);
            takeoff_handler_->hover();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ 호버링 완료\n");

        // === State: RTL (착륙) ===
        if (!transitionToState(MissionState::RTL)) {
            handleError("Failed to transition to RTL state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 9: 착륙 시작");
        if (!rtl_handler_->land()) {
            handleError("Landing failed");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission2] ✓ 착륙 완료\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // === State: IDLE ===
        RCLCPP_INFO(node_->get_logger(), "[testMission2] Step 10: OFFBOARD 모드 비활성화");
        disableOffboardMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!transitionToState(MissionState::IDLE)) {
            RCLCPP_WARN(node_->get_logger(), "[testMission2] Failed to transition to IDLE");
        }

        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  테스트 미션 2 완료!");
        RCLCPP_INFO(node_->get_logger(), "======================================\n");

        return true;

    } catch (const std::runtime_error& e) {
        std::string error_msg = e.what();
        if (error_msg.find("already been added to an executor") != std::string::npos) {
            RCLCPP_WARN(node_->get_logger(), "[testMission2] executor 충돌 (계속 진행): %s", e.what());
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "[testMission2] runtime_error: %s", e.what());
            handleError("Test mission 2 failed");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[testMission2] 예외: %s", e.what());
        handleError("Test mission 2 failed");
        return false;
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "[testMission2] 알 수 없는 예외 발생");
        handleError("Test mission 2 failed");
        return false;
    }
}

void OffboardManager::emergencyRTL()
{
    RCLCPP_ERROR(node_->get_logger(), "\n!!! EMERGENCY RTL TRIGGERED !!!\n");
    
    // OFFBOARD 모드 비활성화 (heartbeat 중단)
    // 중요: 긴급 상황 시 다른 모드로 전환 가능하도록 heartbeat 중단
    RCLCPP_INFO(node_->get_logger(), "[EMERGENCY] Disabling OFFBOARD mode (heartbeat stop)...");
    disableOffboardMode();

    transitionToState(MissionState::RTL);

    RCLCPP_INFO(node_->get_logger(), "[EMERGENCY] Executing RTL...");
    if (!rtl_handler_->returnToLaunch(120000)) {
        RCLCPP_ERROR(node_->get_logger(), "[EMERGENCY] RTL failed!");
        transitionToState(MissionState::ERROR);
    } else {
        RCLCPP_INFO(node_->get_logger(), "[EMERGENCY] RTL complete");
        transitionToState(MissionState::LANDED);
    }
}

void OffboardManager::abortMission()
{
    RCLCPP_WARN(node_->get_logger(), "\n!!! MISSION ABORT REQUESTED !!!\n");

    // 중단 플래그 설정 (모든 루프가 이 플래그를 확인하고 종료)
    abort_requested_.store(true);

    // ★ 비동기 미션이 실행 중이면 requestLanding() 사용
    if (mission_loop_running_.load()) {
        RCLCPP_INFO(node_->get_logger(), "[ABORT] Async mission running - requesting landing via state machine");
        requestLanding();
        return;
    }

    // ★ 동기식 미션인 경우 기존 방식 사용
    // OFFBOARD 모드가 활성 상태에서 먼저 LAND 명령 전송
    RCLCPP_INFO(node_->get_logger(), "[ABORT] Sending LAND command while in OFFBOARD mode...");
    if (rtl_handler_) {
        rtl_handler_->sendLandCommandOnly();
    }

    // 모드 전환 시간 대기 (PX4가 AUTO.LAND로 전환할 시간)
    RCLCPP_INFO(node_->get_logger(), "[ABORT] Waiting for mode transition...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // OFFBOARD 모드 비활성화 (heartbeat 중단)
    RCLCPP_INFO(node_->get_logger(), "[ABORT] Disabling OFFBOARD heartbeat...");
    disableOffboardMode();

    // 상태를 IDLE로 리셋
    transitionToState(MissionState::IDLE);

    RCLCPP_INFO(node_->get_logger(), "[ABORT] ✓ Mission abort complete - aircraft will land");
}

bool OffboardManager::transitionToState(MissionState new_state)
{
    std::string old_state_name = getStateName(current_state_);
    std::string new_state_name = getStateName(new_state);

    RCLCPP_INFO(node_->get_logger(), "State transition: %s -> %s",
                old_state_name.c_str(), new_state_name.c_str());

    current_state_ = new_state;
    return true;
}

void OffboardManager::handleError(const std::string& error_msg)
{
    RCLCPP_ERROR(node_->get_logger(), "ERROR: %s", error_msg.c_str());
    transitionToState(MissionState::ERROR);
}

uint8_t OffboardManager::getCurrentNavState() const
{
    if (arm_handler_) {
        return arm_handler_->getNavState();
    }
    return 0;  // 기본값 (MANUAL)
}

bool OffboardManager::isOffboardMode() const
{
    return arm_handler_ && arm_handler_->isOffboardMode();
}

void OffboardManager::resetToIdle()
{
    RCLCPP_INFO(node_->get_logger(), "Resetting state from %s to IDLE", 
                getStateName(current_state_).c_str());
    
    // OFFBOARD 모드 비활성화 (heartbeat 중단)
    // 상태 리셋 시 OFFBOARD 모드를 종료하여 다른 모드로 전환 가능하도록 함
    disableOffboardMode();
    
    transitionToState(MissionState::IDLE);
}

void OffboardManager::disableOffboardMode()
{
    RCLCPP_INFO(node_->get_logger(), "[OFFBOARD] Disabling OFFBOARD mode (heartbeat stop)...");

    // TakeoffHandler의 heartbeat 중지
    if (takeoff_handler_) {
        takeoff_handler_->stopOffboardHeartbeat();
        RCLCPP_INFO(node_->get_logger(), "[OFFBOARD] ✓ TakeoffHandler heartbeat stopped");
    }

    // ArmHandler의 heartbeat 중지
    if (arm_handler_) {
        if (arm_handler_->disableOffboardMode()) {
            RCLCPP_INFO(node_->get_logger(), "[OFFBOARD] ✓ ArmHandler heartbeat stopped");
            RCLCPP_INFO(node_->get_logger(), "[OFFBOARD] ✓ OFFBOARD 모드 비활성화 성공");
            RCLCPP_INFO(node_->get_logger(), "[OFFBOARD]   → PX4가 자동으로 다른 모드로 전환됩니다");
            RCLCPP_INFO(node_->get_logger(), "[OFFBOARD]   → QGC나 다른 GCS에서 비행 모드 변경이 가능합니다");
        } else {
            RCLCPP_WARN(node_->get_logger(), "[OFFBOARD] ⚠ ArmHandler 비활성화 실패");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "[OFFBOARD] ⚠ ArmHandler가 초기화되지 않음");
    }
}

std::string OffboardManager::getStateName(MissionState state)
{
    switch (state) {
        case MissionState::IDLE:              return "IDLE";
        case MissionState::ARMING:            return "ARMING";
        case MissionState::TAKEOFF:           return "TAKEOFF";
        case MissionState::HOVER:             return "HOVER";
        case MissionState::NAVIGATE:          return "NAVIGATE";
        case MissionState::ADJUST_DISTANCE:   return "ADJUST_DISTANCE";
        case MissionState::WAIT_GPS:          return "WAIT_GPS";
        case MissionState::WAIT_LEADER_INFO:  return "WAIT_LEADER_INFO";
        case MissionState::MOVE_TO_FORMATION: return "MOVE_TO_FORMATION";
        case MissionState::THERMAL_AIMING:    return "THERMAL_AIMING";
        case MissionState::RETURNING_HOME:    return "RETURNING_HOME";
        case MissionState::LANDING:           return "LANDING";
        case MissionState::RTL:               return "RTL";
        case MissionState::LANDED:            return "LANDED";
        case MissionState::ERROR:             return "ERROR";
        default:                              return "UNKNOWN";
    }
}
// ========== GPS 계산 헬퍼 함수 구현 ==========

std::pair<double, double> OffboardManager::calculateOffsetPosition(
    double origin_lat, double origin_lon,
    float distance_m, float bearing_deg)
{
    const double R = 6378137.0;  // 지구 반지름 (m)
    
    // 도 → 라디안 변환
    double lat_rad = origin_lat * M_PI / 180.0;
    double lon_rad = origin_lon * M_PI / 180.0;
    double bearing_rad = bearing_deg * M_PI / 180.0;
    
    // 새 위도 계산
    double new_lat_rad = asin(
        sin(lat_rad) * cos(distance_m / R) +
        cos(lat_rad) * sin(distance_m / R) * cos(bearing_rad)
    );
    
    // 새 경도 계산
    double new_lon_rad = lon_rad + atan2(
        sin(bearing_rad) * sin(distance_m / R) * cos(lat_rad),
        cos(distance_m / R) - sin(lat_rad) * sin(new_lat_rad)
    );
    
    // 라디안 → 도 변환
    return {
        new_lat_rad * 180.0 / M_PI, 
        new_lon_rad * 180.0 / M_PI
    };
}

float OffboardManager::calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6378137.0;  // 지구 반지름 (m)
    
    // 도 → 라디안
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    // 위도/경도 차이
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    
    // Haversine 공식
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2) * sin(dlon / 2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    float distance = R * c;  // 미터 단위
    
    return distance;
}

float OffboardManager::calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    // 도 → 라디안
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    double dlon = lon2_rad - lon1_rad;
    
    // 방위각 계산
    double y = sin(dlon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon);
    
    double bearing_rad = atan2(y, x);
    
    // 라디안 → 도 변환 (0~360)
    float bearing_deg = fmod((bearing_rad * 180.0 / M_PI) + 360.0, 360.0);
    
    return bearing_deg;
}

std::pair<double, double> OffboardManager::calculateTargetPosition(
    double leader_lat, double leader_lon,
    float distance, float heading)
{
    // 리더의 헤딩 방향으로 distance만큼 떨어진 목표물 좌표 계산
    return calculateOffsetPosition(leader_lat, leader_lon, distance, heading);
}
// ========== testMission3 구현 (1단계: 기본 골격) ==========

bool OffboardManager::testMission3(uint8_t vehicle_id,
                                    float takeoff_altitude,
                                    float target_distance)
{
    try {
        // 이미 미션이 실행 중이면 중복 실행 방지
        if (current_state_ != MissionState::IDLE) {
            RCLCPP_WARN(node_->get_logger(),
                       "[testMission3] Mission already running (current state: %s). Ignoring new mission request.",
                       getStateName(current_state_).c_str());
            return false;
        }

        // 중단 플래그 리셋 (새 미션 시작)
        clearAbortFlag();

        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  테스트 미션 3 - 삼각 포메이션 자동 조준");
        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  기체 번호: %d (1=리더, 2=좌측, 3=우측)", vehicle_id);
        RCLCPP_INFO(node_->get_logger(), "  이륙 고도: %.1fm", takeoff_altitude);
        RCLCPP_INFO(node_->get_logger(), "  목표 거리: %.1fm", target_distance);
        RCLCPP_INFO(node_->get_logger(), "======================================\n");

        // 기체 번호 검증
        if (vehicle_id < 1 || vehicle_id > 3) {
            handleError("Invalid vehicle_id. Must be 1, 2, or 3.");
            return false;
        }

        // === State: ARMING ===
        if (!transitionToState(MissionState::ARMING)) {
            handleError("Failed to transition to ARMING state");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 1: OFFBOARD 모드 활성화");

        // 중요: OFFBOARD 모드 활성화 전에 현재 위치 setpoint 발행 시작 (45도 회전 방지)
        RCLCPP_INFO(node_->get_logger(), "[testMission3] Starting hold-position setpoint before OFFBOARD mode...");
        takeoff_handler_->startHoldPositionSetpoint();

        if (!arm_handler_->enableOffboardMode()) {
            handleError("Failed to enable OFFBOARD mode");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission3] ✓ OFFBOARD 모드 활성화 성공\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 2: ARM 시동 걸기");
        if (!arm_handler_->arm()) {
            handleError("Failed to arm vehicle");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission3] ✓ ARM 성공\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // === State: TAKEOFF ===
        if (!transitionToState(MissionState::TAKEOFF)) {
            handleError("Failed to transition to TAKEOFF state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 3: 이륙 시작, 목표 고도: %.1fm", takeoff_altitude);
        if (!takeoff_handler_->takeoff(takeoff_altitude)) {
            handleError("Takeoff failed");
            emergencyRTL();
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission3] ✓ 이륙 완료, 고도: %.2fm\n",
                    takeoff_handler_->getCurrentAltitude());

        // === State: HOVER ===
        if (!transitionToState(MissionState::HOVER)) {
            handleError("Failed to transition to HOVER state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 4: 호버링 5초 (이륙 후)");
        for (int i = 1; i <= 5; i++) {
            RCLCPP_INFO(node_->get_logger(), "[testMission3]   호버링 중... %d/5초", i);
            takeoff_handler_->hover();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission3] ✓ 호버링 완료\n");

                // ========== 리더/팔로워 분기 로직 ==========
        if (vehicle_id == 1) {
            // ===== 리더 모드 =====
            RCLCPP_INFO(node_->get_logger(), "[testMission3] === 리더 모드 시작 ===\n");
            
            // Step 1: 거리 제어 (10m 유지)
            if (!transitionToState(MissionState::ADJUST_DISTANCE)) {
                handleError("Failed to transition to ADJUST_DISTANCE");
                emergencyRTL();
                return false;
            }
            
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 5: LiDAR 거리 제어 (%.1fm)", target_distance);
            if (!leaderDistanceControl(target_distance, 0.5f)) {
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] 거리 제어 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Distance control OK\n");
            
            // Step 2: GPS/헤딩 데이터 확인
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 6: GPS/Attitude 데이터 확인");
            int wait_count = 0;
            while ((!gps_valid_ || !attitude_valid_) && wait_count < 50) {
                // 중단 플래그 확인
                if (abort_requested_.load()) {
                    RCLCPP_WARN(node_->get_logger(), "[testMission3] ★ ABORT requested during GPS wait");
                    return false;
                }
                RCLCPP_WARN(node_->get_logger(), "Waiting for GPS/Attitude data... (%d/50)", wait_count);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // 참고: spin_some은 메인 스레드에서 호출됨 (여기서 호출하면 executor 충돌)
                wait_count++;
            }
            
            if (!gps_valid_ || !attitude_valid_) {
                RCLCPP_ERROR(node_->get_logger(), "GPS or Attitude data not available");
                emergencyRTL();
                return false;
            }
            
            RCLCPP_INFO(node_->get_logger(), "Current GPS: (%.7f, %.7f), Heading: %.1f deg\n",
                       current_lat_, current_lon_, current_heading_);
            
            // Step 3: 목표물 GPS 좌표 계산
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 7: 목표물 좌표 계산");
            float lidar_distance = distance_adjuster_->getFrontDistance();
            auto [target_lat, target_lon] = calculateTargetPosition(
                current_lat_, current_lon_, lidar_distance, current_heading_);
            
            RCLCPP_INFO(node_->get_logger(), "Target position: (%.7f, %.7f)\n", target_lat, target_lon);
            
            // Step 4: 열화상 조준 (시뮬레이션)
            if (!transitionToState(MissionState::HOVER)) {
                handleError("Failed to transition to HOVER for aiming");
                emergencyRTL();
                return false;
            }
            
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 8: 열화상 조준");
            if (!leaderThermalAiming()) {
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] 조준 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Thermal aiming OK\n");
            
            // Step 5: ROS2 토픽 발행 (팔로워에게 정보 전달)
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 9: 리더 정보 발행");
            publishLeaderAimPose(target_lat, target_lon);
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Leader data published\n");
            
            // 호버링 (팔로워가 포메이션 형성할 시간)
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 10: 호버링 (팔로워 대기, 5초)");
            for (int i = 1; i <= 5; i++) {
                RCLCPP_INFO(node_->get_logger(), "  Hovering... %d/5 sec", i);
                takeoff_handler_->hover();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // Step 11: 원래 위치로 복귀 (착륙 전)
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 11: 이륙 지점으로 복귀");
            if (!takeoff_handler_->returnToTakeoffPosition()) {
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] 복귀 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] ✓ 이륙 지점 복귀 완료\n");

            RCLCPP_INFO(node_->get_logger(), "[testMission3] === 리더 모드 완료 ===\n");

        } else {
            // ===== 팔로워 모드 =====
            RCLCPP_INFO(node_->get_logger(), "[testMission3] === 팔로워 모드 시작 ===");
            RCLCPP_INFO(node_->get_logger(), "  기체 번호: %d (%s)\n", 
                       vehicle_id, vehicle_id == 2 ? "좌측" : "우측");
            
            // Step 1: 리더 정보 대기
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 5: 리더 정보 대기");
            if (!waitForLeaderInfo(10)) {
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] 리더 정보 수신 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Leader info received\n");
            
            // Step 2: GPS 데이터 확인
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 6: GPS 데이터 확인");
            int wait_count = 0;
            while (!gps_valid_ && wait_count < 50) {
                // 중단 플래그 확인
                if (abort_requested_.load()) {
                    RCLCPP_WARN(node_->get_logger(), "[testMission3] ★ ABORT requested during GPS wait");
                    return false;
                }
                RCLCPP_WARN(node_->get_logger(), "Waiting for GPS data... (%d/50)", wait_count);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // 참고: spin_some은 메인 스레드에서 호출됨 (여기서 호출하면 executor 충돌)
                wait_count++;
            }
            
            if (!gps_valid_) {
                RCLCPP_ERROR(node_->get_logger(), "GPS data not available");
                emergencyRTL();
                return false;
            }
            
            RCLCPP_INFO(node_->get_logger(), "Current GPS: (%.7f, %.7f)\n",
                       current_lat_, current_lon_);
            
            // Step 3: 포메이션 위치 계산
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 7: 포메이션 위치 계산");
            double my_target_lat, my_target_lon;
            float my_target_heading;
            
            if (!calculateFormationPosition(vehicle_id, my_target_lat, my_target_lon, my_target_heading)) {
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] 포메이션 계산 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Formation position calculated\n");
            
            // Step 4: GPS 이동
            if (!transitionToState(MissionState::NAVIGATE)) {
                handleError("Failed to transition to NAVIGATE");
                emergencyRTL();
                return false;
            }
            
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 8: GPS 이동");
            if (!moveToPosition(my_target_lat, my_target_lon, takeoff_altitude, my_target_heading, 1.0f)) {
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] GPS 이동 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] GPS navigation complete\n");
            
            // Step 5: (옵션) LiDAR 거리 제어
            bool use_lidar_control = true;  // 설정 가능
            
            if (use_lidar_control) {
                if (!transitionToState(MissionState::ADJUST_DISTANCE)) {
                    handleError("Failed to transition to ADJUST_DISTANCE");
                    emergencyRTL();
                    return false;
                }
                
                RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 9: LiDAR 거리 제어 (옵션)");
                followerDistanceControl(target_distance, 0.5f);  // 실패해도 계속 진행
                RCLCPP_INFO(node_->get_logger(), "[testMission3] Distance control done\n");
            }
            
            // Step 6: 열화상 조준 (시뮬레이션)
            if (!transitionToState(MissionState::HOVER)) {
                handleError("Failed to transition to HOVER for aiming");
                emergencyRTL();
                return false;
            }
            
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 10: 열화상 조준");
            if (!leaderThermalAiming()) {  // 리더와 같은 함수 재사용
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] 조준 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Thermal aiming OK\n");
            
            // 호버링 (격발 준비)
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 11: 호버링 (격발 대기, 5초)");
            for (int i = 1; i <= 5; i++) {
                RCLCPP_INFO(node_->get_logger(), "  Hovering... %d/5 sec", i);
                takeoff_handler_->hover();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            // Step 12: 원래 위치로 복귀 (착륙 전)
            RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 12: 이륙 지점으로 복귀");
            if (!takeoff_handler_->returnToTakeoffPosition()) {
                RCLCPP_ERROR(node_->get_logger(), "[testMission3] 복귀 실패");
                emergencyRTL();
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "[testMission3] ✓ 이륙 지점 복귀 완료\n");

            RCLCPP_INFO(node_->get_logger(), "[testMission3] === 팔로워 모드 완료 ===\n");
        }


        // === State: RTL (착륙) ===
        if (!transitionToState(MissionState::RTL)) {
            handleError("Failed to transition to RTL state");
            emergencyRTL();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 5: 착륙 시작");
        if (!rtl_handler_->land()) {
            handleError("Landing failed");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "[testMission3] ✓ 착륙 완료\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // === State: IDLE ===
        RCLCPP_INFO(node_->get_logger(), "[testMission3] Step 6: OFFBOARD 모드 비활성화");
        disableOffboardMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!transitionToState(MissionState::IDLE)) {
            RCLCPP_WARN(node_->get_logger(), "[testMission3] Failed to transition to IDLE");
        }

        RCLCPP_INFO(node_->get_logger(), "======================================");
        RCLCPP_INFO(node_->get_logger(), "  테스트 미션 3 완료!");
        RCLCPP_INFO(node_->get_logger(), "======================================\n");

        return true;

    } catch (const std::runtime_error& e) {
        std::string error_msg = e.what();
        if (error_msg.find("already been added to an executor") != std::string::npos) {
            RCLCPP_WARN(node_->get_logger(), "[testMission3] executor 충돌 (계속 진행): %s", e.what());
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "[testMission3] runtime_error: %s", e.what());
            handleError("Test mission 3 failed");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[testMission3] 예외: %s", e.what());
        handleError("Test mission 3 failed");
        return false;
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "[testMission3] 알 수 없는 예외 발생");
        handleError("Test mission 3 failed");
        return false;
    }
}
// ========== testMission3 콜백 함수 구현 ==========

void OffboardManager::onGpsPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    current_lat_ = msg->lat;
    current_lon_ = msg->lon;
    current_alt_ = msg->alt;
    gps_valid_ = true;
}

void OffboardManager::onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    // 쿼터니언을 오일러 각도로 변환 (Yaw만 필요)
    float qw = msg->q[0];
    float qx = msg->q[1];
    float qy = msg->q[2];
    float qz = msg->q[3];
    
    // Yaw 계산 (라디안)
    float yaw_rad = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    
    // 라디안을 도로 변환 (0~360)
    current_heading_ = fmod((yaw_rad * 180.0f / M_PI) + 360.0f, 360.0f);
    
    attitude_valid_ = true;
}

// ========== 리더 전용 함수 구현 ==========

bool OffboardManager::leaderDistanceControl(float target_distance, float tolerance)
{
    RCLCPP_INFO(node_->get_logger(), "[Leader] Distance control start: target %.1fm", target_distance);
    
    // DistanceAdjuster 사용하여 거리 제어
    if (!distance_adjuster_->adjustDistance(target_distance, tolerance, 30000)) {
        RCLCPP_ERROR(node_->get_logger(), "[Leader] Distance control failed");
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "[Leader] Distance control OK: %.2fm", 
                distance_adjuster_->getFrontDistance());
    return true;
}

bool OffboardManager::leaderThermalAiming()
{
    RCLCPP_INFO(node_->get_logger(), "[Leader] Thermal aiming start");
    
    // TODO: Read thermal camera data
    // TODO: Adjust Yaw until hotspot center is at screen center
    // TODO: Verify aiming stability
    
    // Simulation: wait 3 seconds
    RCLCPP_INFO(node_->get_logger(), "[Leader] Aiming simulation...");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    RCLCPP_INFO(node_->get_logger(), "[Leader] Aiming complete");
    return true;
}

void OffboardManager::publishLeaderAimPose(double target_lat, double target_lon)
{
    RCLCPP_INFO(node_->get_logger(), "[Leader] Publishing leader aim pose");
    RCLCPP_INFO(node_->get_logger(), "  Leader pos: (%.7f, %.7f), alt: %.2fm, hdg: %.1f deg",
                current_lat_, current_lon_, current_alt_, current_heading_);
    RCLCPP_INFO(node_->get_logger(), "  Target pos: (%.7f, %.7f)", target_lat, target_lon);
    RCLCPP_INFO(node_->get_logger(), "  Distance: %.2fm", distance_adjuster_->getFrontDistance());
    
    // TODO: Publish ROS2 message
    // Need to define humiro_msgs::msg::LeaderAimPose
    
    RCLCPP_INFO(node_->get_logger(), "[Leader] Topic published");
}
// ========== 팔로워 전용 함수 구현 ==========

bool OffboardManager::waitForLeaderInfo(int timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "[Follower] Waiting for leader info (timeout: %d sec)...", timeout_sec);
    
    // 현재는 시뮬레이션: publishLeaderAimPose에서 설정한 값 사용
    // 실제로는 ROS2 토픽 구독으로 받아야 함
    
    // 시뮬레이션을 위해 대기
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // TODO: 실제 ROS2 토픽 구독 구현
    // 현재는 더미 데이터로 설정
    RCLCPP_WARN(node_->get_logger(), "[Follower] Using simulated leader info (TODO: implement ROS2 subscription)");
    
    return true;
}

bool OffboardManager::calculateFormationPosition(uint8_t vehicle_id,
                                                 double& my_target_lat,
                                                 double& my_target_lon,
                                                 float& my_target_heading)
{
    RCLCPP_INFO(node_->get_logger(), "[Follower] Calculating formation position for vehicle %d", vehicle_id);
    
    // 리더 정보 확인
    if (!gps_valid_) {
        RCLCPP_ERROR(node_->get_logger(), "[Follower] Leader GPS data not available");
        return false;
    }
    
    // 정삼각형 포메이션 각도
    float formation_angle_offset = 60.0f;  // 정삼각형: 60도
    
    // 현재 리더의 위치를 목표물로 간주 (시뮬레이션)
    // 실제로는 leader_info_.target_lat/lon 사용
    double target_lat = current_lat_;
    double target_lon = current_lon_;
    float target_heading = current_heading_;
    
    RCLCPP_INFO(node_->get_logger(), "  Target (simulated): (%.7f, %.7f), heading: %.1f deg",
                target_lat, target_lon, target_heading);
    
    // 포메이션 각도 계산
    float formation_bearing;
    if (vehicle_id == 2) {
        // 2번 기체: 목표물 기준 +60도 (좌측)
        formation_bearing = target_heading + formation_angle_offset;
    } else if (vehicle_id == 3) {
        // 3번 기체: 목표물 기준 -60도 (우측)
        formation_bearing = target_heading - formation_angle_offset;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "[Follower] Invalid vehicle_id: %d", vehicle_id);
        return false;
    }
    
    // 포메이션 위치 계산 (목표물로부터 10m 떨어진 위치)
    float formation_distance = 10.0f;  // 미터
    auto [calc_lat, calc_lon] = calculateOffsetPosition(
        target_lat, target_lon,
        formation_distance,
        formation_bearing
    );
    
    my_target_lat = calc_lat;
    my_target_lon = calc_lon;
    
    // 목표물을 향한 헤딩 계산
    my_target_heading = calculateBearing(
        my_target_lat, my_target_lon,
        target_lat, target_lon
    );
    
    RCLCPP_INFO(node_->get_logger(), "  My target position: (%.7f, %.7f), heading: %.1f deg",
                my_target_lat, my_target_lon, my_target_heading);
    
    return true;
}

bool OffboardManager::moveToPosition(double target_lat, double target_lon,
                                     float target_alt, float target_heading,
                                     float tolerance_m)
{
    RCLCPP_INFO(node_->get_logger(), "[Follower] Moving to position (%.7f, %.7f), alt: %.1fm",
                target_lat, target_lon, target_alt);
    
    // WaypointHandler 사용하여 GPS 이동
    // 현재는 간단히 시뮬레이션
    RCLCPP_INFO(node_->get_logger(), "[Follower] GPS navigation simulation (5 seconds)...");
    
    for (int i = 1; i <= 5; i++) {
        RCLCPP_INFO(node_->get_logger(), "  Moving... %d/5 sec", i);
        
        // 현재 위치 확인
        if (gps_valid_) {
            float distance = calculateDistance(current_lat_, current_lon_, target_lat, target_lon);
            RCLCPP_INFO(node_->get_logger(), "  Current distance to target: %.2f m", distance);
        }
        
        // 호버링 (실제로는 위치 제어)
        takeoff_handler_->hover();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    RCLCPP_INFO(node_->get_logger(), "[Follower] Position reached (simulated)");
    
    // TODO: 실제 GPS 이동 구현
    // waypoint_handler_->moveToPosition(target_lat, target_lon, target_alt);
    
    return true;
}

bool OffboardManager::followerDistanceControl(float target_distance, float tolerance)
{
    RCLCPP_INFO(node_->get_logger(), "[Follower] Distance control (optional): target %.1fm", target_distance);

    // 팔로워도 LiDAR 거리 제어 수행 (옵션)
    // 리더와 동일한 로직 사용
    if (!distance_adjuster_->adjustDistance(target_distance, tolerance, 30000)) {
        RCLCPP_WARN(node_->get_logger(), "[Follower] Distance control failed (continuing anyway)");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[Follower] Distance control OK: %.2fm",
                distance_adjuster_->getFrontDistance());
    return true;
}

// ========== 비동기 상태 머신 구현 ==========

bool OffboardManager::startAsyncMission3(uint8_t vehicle_id,
                                          float takeoff_altitude,
                                          float target_distance)
{
    // 이미 미션이 실행 중이면 거부
    if (mission_loop_running_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[AsyncMission] Mission already running!");
        return false;
    }

    if (current_state_ != MissionState::IDLE) {
        RCLCPP_WARN(node_->get_logger(), "[AsyncMission] Not in IDLE state (current: %s)",
                    getStateName(current_state_).c_str());
        return false;
    }

    // 미션 파라미터 저장
    mission_vehicle_id_ = vehicle_id;
    mission_takeoff_altitude_ = takeoff_altitude;
    mission_target_distance_ = target_distance;

    // 플래그 리셋
    clearAbortFlag();
    landing_requested_.store(false);
    return_home_requested_.store(false);

    RCLCPP_INFO(node_->get_logger(), "======================================");
    RCLCPP_INFO(node_->get_logger(), "  비동기 미션 3 시작");
    RCLCPP_INFO(node_->get_logger(), "======================================");
    RCLCPP_INFO(node_->get_logger(), "  기체 번호: %d", vehicle_id);
    RCLCPP_INFO(node_->get_logger(), "  이륙 고도: %.1fm", takeoff_altitude);
    RCLCPP_INFO(node_->get_logger(), "  목표 거리: %.1fm", target_distance);
    RCLCPP_INFO(node_->get_logger(), "======================================\n");

    // 기존 스레드 정리
    if (mission_thread_.joinable()) {
        mission_thread_.join();
    }

    // 미션 루프 시작 (별도 스레드)
    mission_loop_running_.store(true);
    mission_thread_ = std::thread(&OffboardManager::missionLoop, this);

    return true;
}

void OffboardManager::requestLanding()
{
    RCLCPP_WARN(node_->get_logger(), "\n★★★ LANDING REQUESTED ★★★\n");
    landing_requested_.store(true);
}

void OffboardManager::requestReturnHome()
{
    RCLCPP_WARN(node_->get_logger(), "\n★★★ RETURN HOME REQUESTED ★★★\n");
    return_home_requested_.store(true);
}

void OffboardManager::saveHomePosition()
{
    if (gps_valid_) {
        home_lat_ = current_lat_;
        home_lon_ = current_lon_;
        home_alt_ = current_alt_;
        home_position_set_ = true;
        RCLCPP_INFO(node_->get_logger(), "[Home] Saved: (%.7f, %.7f, %.1fm)",
                    home_lat_, home_lon_, home_alt_);
    }
}

bool OffboardManager::checkStateTimeout(int timeout_ms)
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - state_start_time_).count();
    return elapsed > timeout_ms;
}

void OffboardManager::missionLoop()
{
    RCLCPP_INFO(node_->get_logger(), "[MissionLoop] Started");

    const int LOOP_RATE_HZ = 20;  // 20Hz = 50ms
    const auto loop_duration = std::chrono::milliseconds(1000 / LOOP_RATE_HZ);

    // 초기 상태: ARMING
    transitionToState(MissionState::ARMING);
    state_start_time_ = std::chrono::steady_clock::now();
    state_step_counter_ = 0;

    while (mission_loop_running_.load() && rclcpp::ok()) {
        auto loop_start = std::chrono::steady_clock::now();

        // ★ 즉시 착륙 요청 체크 (최우선)
        if (landing_requested_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[MissionLoop] ★ Landing requested - transitioning to LANDING");
            landing_requested_.store(false);
            transitionToState(MissionState::LANDING);
            state_start_time_ = std::chrono::steady_clock::now();
            state_step_counter_ = 0;
        }

        // ★ Home 복귀 요청 체크
        if (return_home_requested_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[MissionLoop] ★ Return home requested - transitioning to RETURNING_HOME");
            return_home_requested_.store(false);
            transitionToState(MissionState::RETURNING_HOME);
            state_start_time_ = std::chrono::steady_clock::now();
            state_step_counter_ = 0;
        }

        // 상태 업데이트
        updateState();

        // 종료 조건 체크
        if (current_state_ == MissionState::IDLE ||
            current_state_ == MissionState::LANDED ||
            current_state_ == MissionState::ERROR) {
            RCLCPP_INFO(node_->get_logger(), "[MissionLoop] Mission ended (state: %s)",
                        getStateName(current_state_).c_str());
            break;
        }

        // 루프 주기 유지
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        }
    }

    mission_loop_running_.store(false);
    RCLCPP_INFO(node_->get_logger(), "[MissionLoop] Ended");
}

void OffboardManager::updateState()
{
    switch (current_state_) {
        case MissionState::ARMING:
            updateArming();
            break;
        case MissionState::TAKEOFF:
            updateTakeoff();
            break;
        case MissionState::HOVER:
            updateHover();
            break;
        case MissionState::ADJUST_DISTANCE:
            updateAdjustDistance();
            break;
        case MissionState::WAIT_GPS:
            updateWaitGps();
            break;
        case MissionState::WAIT_LEADER_INFO:
            updateWaitLeaderInfo();
            break;
        case MissionState::MOVE_TO_FORMATION:
            updateMoveToFormation();
            break;
        case MissionState::THERMAL_AIMING:
            updateThermalAiming();
            break;
        case MissionState::RETURNING_HOME:
            updateReturningHome();
            break;
        case MissionState::LANDING:
            updateLanding();
            break;
        default:
            break;
    }
}

void OffboardManager::updateArming()
{
    // ★★★ 매 틱마다 heartbeat (setpoint) 발행 ★★★
    // OFFBOARD 모드 유지를 위해 필수 (2Hz 이상 필요, 우리는 20Hz)
    if (state_step_counter_ > 0) {
        takeoff_handler_->publishTakeoffSetpoint();
    }

    // Step 0: Hold position setpoint 시작 (초기 위치 캡처)
    if (state_step_counter_ == 0) {
        RCLCPP_INFO(node_->get_logger(), "[ARMING] Step 0: Starting hold-position setpoint");
        takeoff_handler_->startHoldPositionSetpoint();
        state_step_counter_ = 1;
        return;
    }

    // Step 1: OFFBOARD 모드 활성화
    if (state_step_counter_ == 1) {
        static bool logged = false;
        if (!logged) {
            RCLCPP_INFO(node_->get_logger(), "[ARMING] Step 1: Enabling OFFBOARD mode");
            logged = true;
        }
        if (arm_handler_->enableOffboardMode()) {
            RCLCPP_INFO(node_->get_logger(), "[ARMING] ✓ OFFBOARD mode enabled");
            state_step_counter_ = 2;
            state_start_time_ = std::chrono::steady_clock::now();  // 대기 시간 리셋
            logged = false;
        } else if (checkStateTimeout(10000)) {
            RCLCPP_ERROR(node_->get_logger(), "[ARMING] OFFBOARD mode timeout");
            transitionToState(MissionState::ERROR);
            logged = false;
        }
        return;
    }

    // Step 2: 2초 대기 (heartbeat 유지하면서)
    if (state_step_counter_ == 2) {
        if (checkStateTimeout(2000)) {
            state_step_counter_ = 3;
        }
        return;
    }

    // Step 3: ARM
    if (state_step_counter_ == 3) {
        static bool logged = false;
        if (!logged) {
            RCLCPP_INFO(node_->get_logger(), "[ARMING] Step 3: Arming vehicle");
            logged = true;
        }
        if (arm_handler_->arm()) {
            RCLCPP_INFO(node_->get_logger(), "[ARMING] ✓ Vehicle armed");

            // Home 위치 저장
            saveHomePosition();

            // 다음 상태: TAKEOFF
            transitionToState(MissionState::TAKEOFF);
            state_start_time_ = std::chrono::steady_clock::now();
            state_step_counter_ = 0;
            logged = false;
        } else if (checkStateTimeout(10000)) {
            RCLCPP_ERROR(node_->get_logger(), "[ARMING] ARM timeout");
            transitionToState(MissionState::ERROR);
            logged = false;
        }
        return;
    }
}

void OffboardManager::updateTakeoff()
{
    // Step 0: 이륙 시작
    if (state_step_counter_ == 0) {
        RCLCPP_INFO(node_->get_logger(), "[TAKEOFF] Starting takeoff to %.1fm", mission_takeoff_altitude_);
        takeoff_handler_->startTakeoff(mission_takeoff_altitude_);
        state_step_counter_ = 1;
        return;
    }

    // Step 1: 이륙 진행 중 - setpoint 발행 & 고도 체크
    if (state_step_counter_ == 1) {
        takeoff_handler_->publishTakeoffSetpoint();

        float current_alt = takeoff_handler_->getCurrentAltitude();
        float target_alt = mission_takeoff_altitude_;

        // 10초마다 로그
        static int log_counter = 0;
        if (++log_counter % 200 == 0) {  // 20Hz * 10초 = 200
            RCLCPP_INFO(node_->get_logger(), "[TAKEOFF] Current altitude: %.2fm / %.2fm",
                        current_alt, target_alt);
        }

        // 목표 고도 도달 확인 (90%)
        if (current_alt >= target_alt * 0.9f) {
            RCLCPP_INFO(node_->get_logger(), "[TAKEOFF] ✓ Takeoff complete at %.2fm", current_alt);
            transitionToState(MissionState::HOVER);
            state_start_time_ = std::chrono::steady_clock::now();
            state_step_counter_ = 0;
        } else if (checkStateTimeout(30000)) {
            RCLCPP_ERROR(node_->get_logger(), "[TAKEOFF] Timeout!");
            transitionToState(MissionState::LANDING);
            state_step_counter_ = 0;
        }
    }
}

void OffboardManager::updateHover()
{
    // 호버링 setpoint 발행
    takeoff_handler_->hover();

    // 5초 호버링 후 다음 상태
    if (checkStateTimeout(5000)) {
        RCLCPP_INFO(node_->get_logger(), "[HOVER] ✓ Hover complete (5 sec)");

        // 리더/팔로워 분기
        if (mission_vehicle_id_ == 1) {
            // 리더: 거리 조절
            transitionToState(MissionState::ADJUST_DISTANCE);
        } else {
            // 팔로워: 리더 정보 대기
            transitionToState(MissionState::WAIT_LEADER_INFO);
        }
        state_start_time_ = std::chrono::steady_clock::now();
        state_step_counter_ = 0;
    }
}

void OffboardManager::updateAdjustDistance()
{
    // 비동기 거리 조절
    // Step 0: 시작
    if (state_step_counter_ == 0) {
        RCLCPP_INFO(node_->get_logger(), "[ADJUST_DISTANCE] Starting distance control (target: %.1fm)",
                    mission_target_distance_);
        state_step_counter_ = 1;
        return;
    }

    // Step 1: 거리 조절 진행
    float current_dist = distance_adjuster_->getFrontDistance();
    float error = current_dist - mission_target_distance_;

    // 거리 조절 setpoint 발행
    distance_adjuster_->publishDistanceSetpoint(mission_target_distance_);

    // 5초마다 로그
    static int log_counter = 0;
    if (++log_counter % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[ADJUST_DISTANCE] Current: %.2fm, Target: %.1fm, Error: %.2fm",
                    current_dist, mission_target_distance_, error);
    }

    // 목표 거리 도달 확인
    if (std::abs(error) < 0.5f) {
        RCLCPP_INFO(node_->get_logger(), "[ADJUST_DISTANCE] ✓ Distance OK: %.2fm", current_dist);
        transitionToState(MissionState::WAIT_GPS);
        state_start_time_ = std::chrono::steady_clock::now();
        state_step_counter_ = 0;
    } else if (checkStateTimeout(30000)) {
        RCLCPP_WARN(node_->get_logger(), "[ADJUST_DISTANCE] Timeout - continuing anyway");
        transitionToState(MissionState::WAIT_GPS);
        state_start_time_ = std::chrono::steady_clock::now();
        state_step_counter_ = 0;
    }
}

void OffboardManager::updateWaitGps()
{
    // 호버링 유지
    takeoff_handler_->hover();

    // GPS/Attitude 데이터 확인
    if (gps_valid_ && attitude_valid_) {
        RCLCPP_INFO(node_->get_logger(), "[WAIT_GPS] ✓ GPS/Attitude valid");
        RCLCPP_INFO(node_->get_logger(), "  Position: (%.7f, %.7f), Heading: %.1f deg",
                    current_lat_, current_lon_, current_heading_);

        // 목표물 좌표 계산
        float lidar_distance = distance_adjuster_->getFrontDistance();
        auto [target_lat, target_lon] = calculateTargetPosition(
            current_lat_, current_lon_, lidar_distance, current_heading_);
        RCLCPP_INFO(node_->get_logger(), "  Target: (%.7f, %.7f)", target_lat, target_lon);

        // 리더 정보 발행
        publishLeaderAimPose(target_lat, target_lon);

        // ★ 현재 위치에서 바로 착륙 (RETURNING_HOME 생략)
        RCLCPP_INFO(node_->get_logger(), "[WAIT_GPS] → LANDING (현재 위치에서 착륙)");
        transitionToState(MissionState::LANDING);
        state_start_time_ = std::chrono::steady_clock::now();
        state_step_counter_ = 0;
    } else if (checkStateTimeout(10000)) {
        RCLCPP_WARN(node_->get_logger(), "[WAIT_GPS] Timeout waiting for GPS/Attitude");
        transitionToState(MissionState::LANDING);
        state_step_counter_ = 0;
    }
}

void OffboardManager::updateWaitLeaderInfo()
{
    // 호버링 유지
    takeoff_handler_->hover();

    // 팔로워: 리더 정보 대기 (시뮬레이션)
    if (checkStateTimeout(5000)) {
        RCLCPP_INFO(node_->get_logger(), "[WAIT_LEADER_INFO] ✓ Leader info received (simulated)");
        transitionToState(MissionState::MOVE_TO_FORMATION);
        state_start_time_ = std::chrono::steady_clock::now();
        state_step_counter_ = 0;
    }
}

void OffboardManager::updateMoveToFormation()
{
    // 호버링 유지 (실제로는 GPS 이동)
    takeoff_handler_->hover();

    // 포메이션 위치 이동 (시뮬레이션)
    if (checkStateTimeout(5000)) {
        RCLCPP_INFO(node_->get_logger(), "[MOVE_TO_FORMATION] ✓ Formation position reached (simulated)");
        transitionToState(MissionState::ADJUST_DISTANCE);
        state_start_time_ = std::chrono::steady_clock::now();
        state_step_counter_ = 0;
    }
}

void OffboardManager::updateThermalAiming()
{
    // 호버링 유지
    takeoff_handler_->hover();

    // 열화상 조준 (시뮬레이션)
    if (state_step_counter_ == 0) {
        RCLCPP_INFO(node_->get_logger(), "[THERMAL_AIMING] Starting thermal aiming...");
        state_step_counter_ = 1;
    }

    // 5초 후 완료
    if (checkStateTimeout(5000)) {
        RCLCPP_INFO(node_->get_logger(), "[THERMAL_AIMING] ✓ Aiming complete");

        // ★ 현재 위치에서 바로 착륙
        RCLCPP_INFO(node_->get_logger(), "[THERMAL_AIMING] → LANDING (현재 위치에서 착륙)");
        transitionToState(MissionState::LANDING);
        state_start_time_ = std::chrono::steady_clock::now();
        state_step_counter_ = 0;
    }
}

void OffboardManager::updateReturningHome()
{
    // Step 0: Home 위치 확인 및 이동 시작
    if (state_step_counter_ == 0) {
        if (!home_position_set_) {
            RCLCPP_WARN(node_->get_logger(), "[RETURNING_HOME] Home position not set - landing in place");
            transitionToState(MissionState::LANDING);
            state_step_counter_ = 0;
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[RETURNING_HOME] Returning to home (%.7f, %.7f)",
                    home_lat_, home_lon_);
        state_step_counter_ = 1;
        return;
    }

    // Step 1: Home으로 이동 (TakeoffHandler의 returnToTakeoffPosition 사용)
    // 현재는 간단히 호버링
    takeoff_handler_->hover();

    // 거리 체크
    if (gps_valid_) {
        float distance = calculateDistance(current_lat_, current_lon_, home_lat_, home_lon_);

        static int log_counter = 0;
        if (++log_counter % 100 == 0) {
            RCLCPP_INFO(node_->get_logger(), "[RETURNING_HOME] Distance to home: %.1fm", distance);
        }

        if (distance < 2.0f) {
            RCLCPP_INFO(node_->get_logger(), "[RETURNING_HOME] ✓ Home reached");
            transitionToState(MissionState::LANDING);
            state_start_time_ = std::chrono::steady_clock::now();
            state_step_counter_ = 0;
        }
    }

    // 타임아웃
    if (checkStateTimeout(60000)) {
        RCLCPP_WARN(node_->get_logger(), "[RETURNING_HOME] Timeout - landing in place");
        transitionToState(MissionState::LANDING);
        state_step_counter_ = 0;
    }
}

void OffboardManager::updateLanding()
{
    // ★★★ 즉시 NAV_LAND 착륙 ★★★
    // OFFBOARD 해제 없이 바로 NAV_LAND 명령 전송
    // → 현재 위치/헤딩 유지, Failsafe 없음

    // Step 0: NAV_LAND 명령 전송
    if (state_step_counter_ == 0) {
        float current_alt = takeoff_handler_->getCurrentAltitude();
        RCLCPP_INFO(node_->get_logger(), "[LANDING] ★ 즉시 NAV_LAND 착륙 시작 (현재 고도: %.2fm)", current_alt);
        RCLCPP_INFO(node_->get_logger(), "[LANDING]   → 현재 위치/헤딩 유지 (파라미터 0)");
        RCLCPP_INFO(node_->get_logger(), "[LANDING]   → FC MPC_LAND_SPEED 파라미터로 착륙 속도 제어");
        RCLCPP_INFO(node_->get_logger(), "[LANDING]   ※ 착륙 속도 변경: QGC에서 MPC_LAND_SPEED 설정 (기본 0.7m/s → 0.35m/s 권장)");

        // NAV_LAND 명령 전송 (OFFBOARD 해제 안 함)
        rtl_handler_->sendLandCommandOnly();

        state_step_counter_ = 1;
        state_start_time_ = std::chrono::steady_clock::now();
        return;
    }

    // Step 1: NAV_LAND 완료 대기 (FC가 접지 감지 및 DISARM 처리)
    float current_alt = takeoff_handler_->getCurrentAltitude();

    static int log_counter = 0;
    if (++log_counter % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[LANDING] AUTO.LAND 진행 중... 고도: %.2fm", current_alt);
    }

    // 착륙 완료 확인 (FC의 Land Detector가 판단)
    if (rtl_handler_->isLanded() || current_alt < 0.1f) {
        RCLCPP_INFO(node_->get_logger(), "[LANDING] ✓ Landed! (FC Land Detector)");
        transitionToState(MissionState::LANDED);
        state_step_counter_ = 0;
    } else if (checkStateTimeout(60000)) {
        RCLCPP_WARN(node_->get_logger(), "[LANDING] Timeout (60s) - FC가 착륙 처리 중으로 간주");
        transitionToState(MissionState::LANDED);
    }
}
