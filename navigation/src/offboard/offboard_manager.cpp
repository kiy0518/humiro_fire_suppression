/**
 * @file offboard_manager.cpp
 * @brief PX4 Offboard 제어 - 단순화된 버전
 *
 * 제공된 offboard_control.cpp 기반으로 작성
 * - 단일 타이머에서 heartbeat + setpoint 동시 발행
 * - PX4 호환 QoS (sensor_data 프로파일)
 * - 상태 카운터 기반 단계 진행
 */

#include "offboard_manager.h"
#include <cstdlib>

using namespace std::chrono_literals;

OffboardManager::OffboardManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // ========== PX4 토픽 네임스페이스 (멀티 드론 충돌 방지) ==========
    // PX4 uXRCE-DDS client가 -n droneN 옵션으로 시작되면
    // 토픽이 /droneN/fmu/in/... 형태로 발행됨
    const char* ns_env = getenv("ROS_NAMESPACE");
    std::string px4_ns = ns_env ? ("/" + std::string(ns_env)) : "";
    if (!px4_ns.empty()) {
        RCLCPP_INFO(node_->get_logger(), "PX4 topic namespace: %s", px4_ns.c_str());
    }

    // ========== PX4 호환 QoS 설정 (매우 중요!) ==========
    // PX4는 Best Effort + Volatile 조합을 요구
    // sensor_data 프로파일 = BEST_EFFORT + VOLATILE + KEEP_LAST
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    // ========== Publishers ==========
    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_ns + "/fmu/in/offboard_control_mode", qos);
    trajectory_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_ns + "/fmu/in/trajectory_setpoint", qos);
    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        px4_ns + "/fmu/in/vehicle_command", qos);

    // ========== Subscribers ==========
    vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        px4_ns + "/fmu/out/vehicle_status_v1", qos,
        std::bind(&OffboardManager::vehicleStatusCallback, this, std::placeholders::_1));

    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        px4_ns + "/fmu/out/vehicle_local_position", qos,
        std::bind(&OffboardManager::vehicleLocalPositionCallback, this, std::placeholders::_1));

    vehicle_global_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        px4_ns + "/fmu/out/vehicle_global_position", qos,
        std::bind(&OffboardManager::vehicleGlobalPositionCallback, this, std::placeholders::_1));

    // ========== 드론 ID (MAV_SYS_ID) ==========
    const char* drone_id_env = getenv("DRONE_ID");
    if (drone_id_env) {
        target_system_ = static_cast<uint8_t>(atoi(drone_id_env));
    }

    RCLCPP_INFO(node_->get_logger(), "OffboardManager initialized (ns=%s, target_sys=%d)",
                px4_ns.empty() ? "none" : px4_ns.c_str(), (int)target_system_);
}

OffboardManager::~OffboardManager()
{
    if (timer_) {
        timer_->cancel();
    }
}

bool OffboardManager::executeMission3(const MissionConfig& config)
{
    // 이미 미션 실행 중이면 좌표만 업데이트 (경로 변경)
    if (mission_running_.load()) {
        // PX4가 DISARMED인데 미션 진행 중이면 → 이전 미션 비정상 종료, 상태 리셋
        if (arming_state_.load() != 2) {  // 2 = ARMED
            RCLCPP_WARN(node_->get_logger(),
                "[MISSION] Stale mission state detected! (mission_running=true, armed=OFF, state=%s) → Resetting",
                getStateName(current_state_.load()).c_str());
            mission_running_.store(false);
            current_state_.store(MissionState::IDLE);
            abort_requested_.store(false);
            if (timer_) timer_->cancel();
            // 아래로 흘러서 새 미션 시작
        } else {
            RCLCPP_INFO(node_->get_logger(), "[MISSION] Mission already running, updating target...");
            return updateMissionTarget(config.target_waypoint);
        }
    }

    // 미션 설정 저장
    mission_config_ = config;
    mission_running_.store(true);
    abort_requested_.store(false);
    setpoint_counter_.store(0);
    formation_ready_to_rotate_.store(false);
    formation_ready_to_navigate_.store(false);
    prev_vx_ = 0.0f;
    prev_vy_ = 0.0f;

    // Home 위치 리셋 (현재 위치를 새 Home으로 사용)
    home_set_ = false;

    // 시작 위치 저장 (PX4 로컬 NED 좌표)
    start_local_x_ = current_local_x_.load();
    start_local_y_ = current_local_y_.load();
    start_local_z_ = current_local_z_.load();

    RCLCPP_INFO(node_->get_logger(), "==============================================");
    RCLCPP_INFO(node_->get_logger(), "  Starting Mission (executeMission3)");
    RCLCPP_INFO(node_->get_logger(), "  Start position (NED): (%.1f, %.1f, %.1f)",
                start_local_x_, start_local_y_, start_local_z_);
    RCLCPP_INFO(node_->get_logger(), "==============================================");
    RCLCPP_INFO(node_->get_logger(), "  Takeoff altitude: %.1f m", config.takeoff_altitude);
    RCLCPP_INFO(node_->get_logger(), "  Target: Lat=%.7f, Lon=%.7f",
                config.target_waypoint.latitude, config.target_waypoint.longitude);
    RCLCPP_INFO(node_->get_logger(), "  Hover duration: %.1f sec", config.hover_duration_sec);
    RCLCPP_INFO(node_->get_logger(), "==============================================");
    RCLCPP_INFO(node_->get_logger(), "Mission sequence:");
    RCLCPP_INFO(node_->get_logger(), "  1. Heartbeat 발행 (2초)");
    RCLCPP_INFO(node_->get_logger(), "  2. OFFBOARD 모드 전환");
    RCLCPP_INFO(node_->get_logger(), "  3. ARM (시동)");
    RCLCPP_INFO(node_->get_logger(), "  4. 이륙 및 호버링");
    RCLCPP_INFO(node_->get_logger(), "  5. 목표 방향 회전");
    RCLCPP_INFO(node_->get_logger(), "  6. 목표 위치 이동");
    RCLCPP_INFO(node_->get_logger(), "  7. 목표지점 호버링 (5초)");
    RCLCPP_INFO(node_->get_logger(), "  8. RTL (자동 귀환)");
    RCLCPP_INFO(node_->get_logger(), "==============================================\n");

    // 상태 초기화
    current_state_.store(MissionState::PREPARING);

    // 타이머 시작 (10Hz)
    timer_ = node_->create_wall_timer(
        100ms,
        std::bind(&OffboardManager::timerCallback, this));

    // 미션 완료 또는 중단까지 대기
    while (mission_running_.load()) {
        auto state = current_state_.load();
        if (state == MissionState::LANDED || state == MissionState::ERROR) {
            break;
        }

        // RTL 상태에서 disarm 감지 (타이머가 취소된 경우를 위한 백업 체크)
        if (state == MissionState::RTL && arming_state_.load() == 1) {
            RCLCPP_INFO(node_->get_logger(), "[RTL] Disarm detected in wait loop, mission complete!");
            current_state_.store(MissionState::LANDED);
            mission_running_.store(false);
            break;
        }

        std::this_thread::sleep_for(100ms);
    }

    // 타이머 정지
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }

    formation_mode_ = false;
    mission_running_.store(false);

    auto final_state = current_state_.load();
    if (final_state == MissionState::LANDED) {
        RCLCPP_INFO(node_->get_logger(), "==============================================");
        RCLCPP_INFO(node_->get_logger(), "  Mission Complete!");
        RCLCPP_INFO(node_->get_logger(), "==============================================");
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Mission failed (state: %s)", getStateName(final_state).c_str());
        return false;
    }
}

bool OffboardManager::executeMission4(const MissionConfig& config)
{
    RCLCPP_INFO(node_->get_logger(), "[MISSION4] 군집비행 미션 시작 (suppress_hover=30s)");
    formation_mode_ = true;
    return executeMission3(config);
}

void OffboardManager::timerCallback()
{
    // 미션이 실행 중이 아니면 아무것도 하지 않음
    if (!mission_running_.load()) {
        return;
    }

    // 중단 요청 확인
    if (abort_requested_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[ABORT] Mission aborted, triggering RTL...");
        abort_requested_.store(false);  // 한 번만 실행

        // 타이머 먼저 취소
        if (timer_) {
            timer_->cancel();
            RCLCPP_INFO(node_->get_logger(), "[RTL] Timer cancelled - no more heartbeats");
        }

        // 명시적 AUTO_RTL 모드 전환 (main=4, sub=5)
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f, 4.0f, 5.0f);
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        current_state_.store(MissionState::RTL);
        return;
    }

    auto state = current_state_.load();
    uint8_t nav = nav_state_.load();

    // PX4가 RTL 모드(5)이거나 우리 상태가 RTL/LANDED/ERROR면 heartbeat 발행 안 함
    // nav_state: 5=AUTO_RTL, 14=OFFBOARD
    // 단, PREPARING/OFFBOARD/ARMING 단계에서는 이전 RTL 잔여 상태 무시
    // (새 미션 시작 직후 PX4가 아직 이전 AUTO_RTL인 경우 방지)
    bool past_arming = (state != MissionState::PREPARING &&
                        state != MissionState::OFFBOARD &&
                        state != MissionState::ARMING);
    if ((nav == 5 && past_arming) || state == MissionState::RTL || state == MissionState::LANDED || state == MissionState::ERROR) {
        // RTL 완료 체크 (착륙 감지)
        if ((state == MissionState::RTL || (nav == 5 && past_arming)) && arming_state_.load() == 1) {
            RCLCPP_INFO(node_->get_logger(), "[RTL] Vehicle disarmed, mission complete!");
            current_state_.store(MissionState::LANDED);
            mission_running_.store(false);  // 이후 콜백에서 아무것도 하지 않음
        }
        return;
    }

    uint64_t counter = setpoint_counter_.fetch_add(1);

    // 디버그: 상태와 카운터 출력 (1초마다)
    if (counter % 10 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[DEBUG] counter=%ld, state=%s, nav=%d",
                    counter, getStateName(state).c_str(), nav);
    }

    // ★★★ 중요: RTL 전환 체크를 heartbeat 발행 전에 먼저 수행 ★★★
    // 이렇게 해야 RTL 전환 시 마지막 heartbeat가 발행되지 않음

    // 1. 목표 도달 체크 → HOVER_AT_TARGET 전환
    if (state == MissionState::NAVIGATE && counter > MOVE_START + 10) {
        float dx = target_ned_x_ - current_local_x_.load();
        float dy = target_ned_y_ - current_local_y_.load();
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < WAYPOINT_THRESHOLD) {
            RCLCPP_INFO(node_->get_logger(), "\n[Step 6] 목표 도착! 목표지점 호버링 시작 (%.1f초)...",
                        TARGET_HOVER_TICKS / 10.0f);
            hover_at_target_start_.store(counter);
            current_state_.store(MissionState::HOVER_AT_TARGET);
            // 타이머 유지 (heartbeat 계속 필요)
        }
    }

    // 2. 목표지점 호버링 완료 → RTL 전환
    if (state == MissionState::HOVER_AT_TARGET) {
        uint64_t hover_start = hover_at_target_start_.load();
        uint64_t hover_limit = formation_mode_ ? SUPPRESS_HOVER_TICKS : TARGET_HOVER_TICKS;
        if (counter - hover_start >= hover_limit) {
            RCLCPP_INFO(node_->get_logger(), "\n[Step 7] 목표지점 호버링 완료! (%.0f초) RTL 시작...",
                        hover_limit / 10.0);

            // 타이머 취소 (heartbeat 발행 없이 종료)
            if (timer_) {
                timer_->cancel();
                RCLCPP_INFO(node_->get_logger(), "[RTL] Timer cancelled - no more heartbeats");
            }

            // 명시적 AUTO_RTL 모드 전환
            publishVehicleCommand(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1.0f, 4.0f, 5.0f);
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
            current_state_.store(MissionState::RTL);
            return;  // heartbeat 발행 없이 바로 종료
        }
    }

    // 2. 타임아웃 체크 제거 - 미션 완료 또는 수동 개입으로만 RTL 진입

    // ★★★ 핵심: heartbeat 발행 (RTL 체크 통과 후에만) ★★★
    publishOffboardControlMode();

    // ========== 상태 머신 ==========

    if (counter == PREPARE_COUNT) {
        // 2초 후: OFFBOARD 모드 전환 요청
        RCLCPP_INFO(node_->get_logger(), "\n[Step 1] OFFBOARD 모드로 전환 요청...");
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        current_state_.store(MissionState::OFFBOARD);

    } else if (counter == ARM_COUNT) {
        // 4.5초 후: ARM
        RCLCPP_INFO(node_->get_logger(), "\n[Step 2] ARM (시동) 요청...");
        initial_yaw_ = current_yaw_.load();  // 이륙 시점 헤딩 기억
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        current_state_.store(MissionState::ARMING);

    } else if (counter == ARM_COUNT + 10) {
        // ARM 확인 후 TAKEOFF 상태로
        if (arming_state_.load() == 2) {
            RCLCPP_INFO(node_->get_logger(), "[ARM] Vehicle armed successfully!");
            current_state_.store(MissionState::TAKEOFF);
        }

    } else if (counter == TAKEOFF_STABLE) {
        if (continuous_update_mode_.load()) {
            // ★★★ 팔로워: HOVER/ROTATE 건너뛰고 바로 NAVIGATE ★★★
            // 이륙 직후 오프셋 추적 시작 — HOVER/ROTATE 중 표류/간섭 원천 차단
            RCLCPP_INFO(node_->get_logger(), "\n[Step 3] 이륙 완료 → 편대 추적 즉시 시작 (NAVIGATE direct)");
            RCLCPP_INFO(node_->get_logger(), "[NAV-DIRECT] target_ned=(%.1f, %.1f), GPS target=(%.7f, %.7f)",
                        target_ned_x_, target_ned_y_,
                        mission_config_.target_waypoint.latitude,
                        mission_config_.target_waypoint.longitude);
            current_state_.store(MissionState::NAVIGATE);
            setpoint_counter_.store(MOVE_START + 1);  // ROTATE/MOVE 카운터 건너뛰기
        } else {
            // 리더/단독: 정상 HOVER → ROTATE → NAVIGATE 시퀀스
            RCLCPP_INFO(node_->get_logger(), "\n[Step 3] 이륙 완료, 호버링 시작 (%.1f초)...",
                        mission_config_.hover_duration_sec);
            current_state_.store(MissionState::HOVER);

            // 목표 좌표 계산: 현재 GPS → 목표 GPS 오프셋을 시작 위치에 더함
            double cur_lat = current_lat_.load();
            double cur_lon = current_lon_.load();
            double tgt_lat = mission_config_.target_waypoint.latitude;
            double tgt_lon = mission_config_.target_waypoint.longitude;

            constexpr double DEG_TO_M_LAT = 111320.0;
            double deg_to_m_lon = 111320.0 * std::cos(cur_lat * M_PI / 180.0);
            float offset_north = static_cast<float>((tgt_lat - cur_lat) * DEG_TO_M_LAT);
            float offset_east = static_cast<float>((tgt_lon - cur_lon) * deg_to_m_lon);

            target_ned_x_ = start_local_x_ + offset_north;
            target_ned_y_ = start_local_y_ + offset_east;
            target_ned_z_ = -mission_config_.takeoff_altitude;
            target_yaw_ = calculateTargetYaw(offset_north, offset_east);

            RCLCPP_INFO(node_->get_logger(), "[NAV] Current GPS: (%.7f, %.7f)", cur_lat, cur_lon);
            RCLCPP_INFO(node_->get_logger(), "[NAV] Target GPS: (%.7f, %.7f)", tgt_lat, tgt_lon);
            RCLCPP_INFO(node_->get_logger(), "[NAV] GPS offset: North=%.1fm, East=%.1fm", offset_north, offset_east);
            RCLCPP_INFO(node_->get_logger(), "[NAV] Target NED: (%.1f, %.1f, %.1f), Yaw: %.1f deg",
                        target_ned_x_, target_ned_y_, target_ned_z_, target_yaw_ * 180.0f / M_PI);
        }

    } else if (counter == ROTATE_START) {
        // continuous_update_mode는 이미 NAVIGATE 상태이므로 여기 도달 안 함
        initial_yaw_ = current_yaw_.load();

        bool needs_gate = formation_mode_ && !continuous_update_mode_.load();
        if (needs_gate && !formation_ready_to_rotate_.load()) {
            RCLCPP_INFO(node_->get_logger(), "\n[Step 4] 편대 모드: 팔로워 이륙 대기중 (HOVER 유지)...");
        } else {
            RCLCPP_INFO(node_->get_logger(), "\n[Step 4] 목표 방향으로 회전 시작 (initial_yaw=%.1f°, target_yaw=%.1f°)...",
                        initial_yaw_ * 180.0f / M_PI, target_yaw_ * 180.0f / M_PI);
            current_state_.store(MissionState::ROTATE);
        }

    } else if (counter == ROTATE_END) {
        RCLCPP_INFO(node_->get_logger(), "\n[Step 5] 회전 완료, 이동 시작...");

    } else if (counter == MOVE_START) {
        bool needs_gate = formation_mode_ && !continuous_update_mode_.load();
        if (needs_gate && !formation_ready_to_navigate_.load()) {
            RCLCPP_INFO(node_->get_logger(), "\n[Step 6] 편대 모드: 편대 배치 대기중 (ROTATE 유지)...");
        } else {
            current_state_.store(MissionState::NAVIGATE);
        }
    }

    // === 편대 모드: 지연 상태 전환 (게이트 통과 후) ===
    if (formation_mode_ && !continuous_update_mode_.load()) {
        // HOVER 유지 중: 팔로워 이륙 확인 시 → ROTATE 전환
        if (state == MissionState::HOVER && counter > ROTATE_START) {
            if (formation_ready_to_rotate_.load()) {
                initial_yaw_ = current_yaw_.load();  // 현재 yaw로 갱신
                RCLCPP_INFO(node_->get_logger(),
                    "[FORMATION] 팔로워 이륙 확인! ROTATE 시작 (대기: %.1fs, yaw=%.1f°→%.1f°)",
                    (counter - ROTATE_START) / 10.0f,
                    initial_yaw_ * 180.0f / M_PI, target_yaw_ * 180.0f / M_PI);
                setpoint_counter_.store(ROTATE_START);
                current_state_.store(MissionState::ROTATE);
            } else if (counter % 50 == 0) {
                RCLCPP_INFO(node_->get_logger(), "[FORMATION] HOVER 대기: 팔로워 이륙 대기중...");
            }
        }
        // ROTATE 유지 중: 편대 배치 완료 시 → NAVIGATE 전환
        if (state == MissionState::ROTATE && counter > MOVE_START) {
            if (formation_ready_to_navigate_.load()) {
                RCLCPP_INFO(node_->get_logger(), "[FORMATION] 편대 배치 완료! NAVIGATE 시작");
                setpoint_counter_.store(MOVE_START);
                current_state_.store(MissionState::NAVIGATE);
            } else if (counter % 50 == 0) {
                RCLCPP_INFO(node_->get_logger(), "[FORMATION] ROTATE 대기: 팔로워 편대 배치 대기중...");
            }
        }
    }

    // 진행 상황 로깅 (2초마다) - 목표 도달/타임아웃 체크는 위에서 heartbeat 전에 수행됨
    if (state == MissionState::NAVIGATE && counter > MOVE_START + 10 && counter % 20 == 0) {
        float dx = target_ned_x_ - current_local_x_.load();
        float dy = target_ned_y_ - current_local_y_.load();
        float dist = std::sqrt(dx * dx + dy * dy);
        RCLCPP_INFO(node_->get_logger(), "[NAV] Distance to target: %.1f m", dist);
    }

    // TrajectorySetpoint 발행 (ARM 이후)
    if (counter > ARM_COUNT && state != MissionState::RTL) {
        publishTrajectorySetpoint();
    }

    // 진행 상황 로깅 (준비 중)
    if (counter < PREPARE_COUNT && counter % 5 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[준비 중] Heartbeat 발행 중... %ld/%ld",
                    counter, PREPARE_COUNT);
    }
}

void OffboardManager::publishOffboardControlMode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);

    // 디버그: heartbeat 발행 시점 로깅 (RTL 이슈 디버깅용)
    static uint64_t hb_count = 0;
    if (++hb_count % 50 == 0) {  // 5초마다
        RCLCPP_DEBUG(node_->get_logger(), "[HB] Heartbeat #%ld published (state=%s, nav=%d)",
                     hb_count, getStateName(current_state_.load()).c_str(), nav_state_.load());
    }
}

void OffboardManager::publishTrajectorySetpoint()
{
    auto state = current_state_.load();
    uint64_t counter = setpoint_counter_.load();

    float sp_x, sp_y, sp_z;
    float yaw_setpoint = NAN;
    float yawspeed = 0.0f;

    if (state == MissionState::TAKEOFF || state == MissionState::HOVER ||
        (state == MissionState::ROTATE && counter < ROTATE_START)) {
        // 1단계: 이륙 및 호버링 (시작 위치에서 takeoff_altitude)
        sp_x = start_local_x_;
        sp_y = start_local_y_;
        sp_z = -mission_config_.takeoff_altitude;  // NED: Z down이 양수
        yaw_setpoint = initial_yaw_;

    } else if (state == MissionState::ROTATE ||
               (counter >= ROTATE_START && counter < ROTATE_END)) {
        // 2단계: 회전 (yawspeed 사용 - 감속 프로파일)
        sp_x = start_local_x_;
        sp_y = start_local_y_;
        sp_z = -mission_config_.takeoff_altitude;

        // 현재 yaw와 목표 yaw 차이 계산
        float current_yaw = current_yaw_.load();
        float yaw_diff = target_yaw_ - current_yaw;

        // -π ~ π 범위로 정규화
        while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

        // 감속 프로파일: 각도 차이 비례 (오버슈팅 방지)
        constexpr float YAW_DECEL_ANGLE = 0.5f;  // 감속 시작 각도 (rad, ~28.6도)
        float abs_yaw_diff = std::fabs(yaw_diff);

        if (abs_yaw_diff < 0.02f) {  // ~1도 이내: 정지
            yawspeed = 0.0f;
        } else if (abs_yaw_diff < YAW_DECEL_ANGLE) {
            // 감속 구간: 각도 비례 (최소 0.1 rad/s)
            float speed = MAX_YAW_RATE * (abs_yaw_diff / YAW_DECEL_ANGLE);
            speed = std::max(0.1f, speed);
            yawspeed = (yaw_diff > 0) ? speed : -speed;
        } else {
            // 최대 속도 구간
            yawspeed = (yaw_diff > 0) ? MAX_YAW_RATE : -MAX_YAW_RATE;
        }

        yaw_setpoint = NAN;  // yawspeed 사용 시 yaw는 NAN

    } else if (counter >= ROTATE_END && counter < MOVE_START) {
        // 전환 구간: 호버링 유지
        sp_x = start_local_x_;
        sp_y = start_local_y_;
        sp_z = -mission_config_.takeoff_altitude;
        yaw_setpoint = target_yaw_;

    } else if (state == MissionState::HOVER_AT_TARGET) {
        // 4단계: 목표지점 호버링 (position setpoint으로 위치 유지)
        float effective_alt = (mission_config_.target_altitude > 0.0f)
                              ? mission_config_.target_altitude
                              : mission_config_.takeoff_altitude;
        sp_x = target_ned_x_;
        sp_y = target_ned_y_;
        sp_z = -effective_alt;
        yaw_setpoint = target_yaw_;

    } else if (state == MissionState::NAVIGATE || counter >= MOVE_START) {
        // 3단계: 목표 위치로 이동 (position + velocity 동시 전송 - 피드포워드 제어)
        // PX4는 position과 velocity를 동시에 받으면 velocity를 피드포워드로 사용하여
        // "이 위치에 도달할 때 이 속도가 되어야 한다"를 인지하고 강력한 제동 수행
        float cur_x = current_local_x_.load();
        float cur_y = current_local_y_.load();
        float dx = target_ned_x_ - cur_x;
        float dy = target_ned_y_ - cur_y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // 감속 프로파일: 거리 비례 속도 (목표에서 0)
        // 현재 실제 피드포워드 속도 계산
        float current_ff_speed = std::sqrt(prev_vx_ * prev_vx_ + prev_vy_ * prev_vy_);
        float speed = mission_config_.flight_speed;
        constexpr float DECEL_RADIUS = 80.0f;  // 감속 시작 거리 (m) - 45kg기체 12m/s 기준
        if (dist < DECEL_RADIUS) {
            // 거리 비례 감속, 단 현재 속도보다 높으면 현재 속도로 제한
            float decel_speed = speed * (dist / DECEL_RADIUS);
            speed = std::max(0.3f, std::min(decel_speed, current_ff_speed));
        }

        // velocity 피드포워드 계산 (목표 방향 * 속도, 목표 근처에서 0으로 수렴)
        float ff_vx = 0.0f, ff_vy = 0.0f;
        if (dist > 0.3f) {
            float target_vx = (dx / dist) * speed;
            float target_vy = (dy / dist) * speed;

            // Low-pass filter (부드러운 선회 유지, alpha 약간 높임)
            constexpr float VELOCITY_ALPHA = 0.15f;
            ff_vx = prev_vx_ * (1.0f - VELOCITY_ALPHA) + target_vx * VELOCITY_ALPHA;
            ff_vy = prev_vy_ * (1.0f - VELOCITY_ALPHA) + target_vy * VELOCITY_ALPHA;
        } else {
            // 목표 극근처: velocity = 0 (정지 명령)
            ff_vx = prev_vx_ * 0.5f;  // 빠르게 감쇠
            ff_vy = prev_vy_ * 0.5f;
        }
        prev_vx_ = ff_vx;
        prev_vy_ = ff_vy;

        // yaw 제어 (감속 프로파일)
        float current_yaw = current_yaw_.load();
        float yaw_diff = target_yaw_ - current_yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

        constexpr float YAW_DECEL_ANGLE = 0.5f;  // 감속 시작 각도 (rad, ~28.6도)
        float abs_yaw_diff = std::fabs(yaw_diff);
        if (abs_yaw_diff < 0.02f) {
            yawspeed = 0.0f;
        } else if (abs_yaw_diff < YAW_DECEL_ANGLE) {
            float speed = MAX_YAW_RATE * (abs_yaw_diff / YAW_DECEL_ANGLE);
            speed = std::max(0.1f, speed);
            yawspeed = (yaw_diff > 0) ? speed : -speed;
        } else {
            yawspeed = (yaw_diff > 0) ? MAX_YAW_RATE : -MAX_YAW_RATE;
        }

        // Position + Velocity 전송
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        msg.position = {target_ned_x_, target_ned_y_, target_ned_z_};  // updateMissionTarget에서 설정된 고도 사용
        if (continuous_update_mode_.load()) {
            msg.velocity = {NAN, NAN, NAN};  // 연속 추적 모드: position only (FF 간섭 방지)
        } else {
            msg.velocity = {ff_vx, ff_vy, NAN};  // 단독 비행: 피드포워드 속도
        }
        msg.yaw = NAN;
        msg.yawspeed = yawspeed;
        trajectory_setpoint_pub_->publish(msg);

        if (counter % 10 == 0) {
            RCLCPP_INFO(node_->get_logger(),
                        "[NAVIGATE] tgtNED=(%.1f,%.1f) curNED=(%.1f,%.1f) curGPS=(%.7f,%.7f) tgtGPS=(%.7f,%.7f) dist=%.1f yaw=%.1f→%.1f",
                        target_ned_x_, target_ned_y_, cur_x, cur_y,
                        current_lat_.load(), current_lon_.load(),
                        mission_config_.target_waypoint.latitude,
                        mission_config_.target_waypoint.longitude,
                        dist,
                        current_yaw * 180.0f / M_PI, target_yaw_ * 180.0f / M_PI);
        }
        return;  // 이미 메시지 발행했으므로 아래 코드 스킵

    } else {
        // 기본: 현재 위치 유지
        sp_x = current_local_x_.load();
        sp_y = current_local_y_.load();
        sp_z = current_local_z_.load();
        yaw_setpoint = current_yaw_.load();
    }

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = {sp_x, sp_y, sp_z};
    msg.velocity = {NAN, NAN, NAN};  // PX4가 속도 제어
    msg.yaw = yaw_setpoint;
    msg.yawspeed = yawspeed;

    trajectory_setpoint_pub_->publish(msg);

    // 진행 상황 로깅 (1초마다)
    if (counter % 10 == 0 && state != MissionState::PREPARING) {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] Pos(%.1f, %.1f, %.1f) GPS(%.6f, %.6f) Alt: %.1f m | Yaw: %.1f°",
                    getStateName(state).c_str(),
                    sp_x, sp_y, sp_z,
                    current_lat_.load(), current_lon_.load(),
                    -current_local_z_.load(),
                    (std::isnan(yaw_setpoint) ? current_yaw_.load() : yaw_setpoint) * 180.0f / M_PI);
    }
}

void OffboardManager::publishVehicleCommand(uint16_t command, float param1, float param2, float param3)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.target_system = target_system_;
    msg.target_component = 1;
    msg.source_system = target_system_;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);

    // 디버그: 명령 전송 로깅
    const char* cmd_name = "UNKNOWN";
    if (command == 176) cmd_name = "DO_SET_MODE";
    else if (command == 400) cmd_name = "ARM_DISARM";
    else if (command == 20) cmd_name = "NAV_RETURN_TO_LAUNCH";

    RCLCPP_INFO(node_->get_logger(), "[CMD] %s (cmd=%d, p1=%.1f, p2=%.1f, p3=%.1f)",
                cmd_name, command, param1, param2, param3);
}

void OffboardManager::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    uint8_t old_nav = nav_state_.load();
    uint8_t old_arm = arming_state_.load();

    nav_state_.store(msg->nav_state);
    arming_state_.store(msg->arming_state);

    // 상태 변화 로깅
    if (old_nav != msg->nav_state) {
        const char* nav_name = "UNKNOWN";
        switch (msg->nav_state) {
            case 0: nav_name = "MANUAL"; break;
            case 2: nav_name = "ALTCTL"; break;
            case 3: nav_name = "POSCTL"; break;
            case 4: nav_name = "AUTO_LOITER"; break;
            case 5: nav_name = "AUTO_RTL"; break;
            case 14: nav_name = "OFFBOARD"; break;
            case 17: nav_name = "AUTO_TAKEOFF"; break;
            case 18: nav_name = "AUTO_LAND"; break;
        }
        RCLCPP_INFO(node_->get_logger(), "[STATUS] nav_state: %d -> %d (%s) | my_state=%s, timer=%s",
                    old_nav, msg->nav_state, nav_name,
                    getStateName(current_state_.load()).c_str(),
                    (timer_ && !timer_->is_canceled()) ? "ACTIVE" : "CANCELLED");

        // RTL에서 OFFBOARD로 전환 시 경고
        if (old_nav == 5 && msg->nav_state == 14) {
            RCLCPP_WARN(node_->get_logger(),
                        "[WARNING] PX4 switched from AUTO_RTL to OFFBOARD! (timer=%s, arming=%d)",
                        (timer_ && !timer_->is_canceled()) ? "ACTIVE" : "CANCELLED",
                        msg->arming_state);
        }
    }
    if (old_arm != msg->arming_state) {
        RCLCPP_INFO(node_->get_logger(), "[STATUS] arming: %d -> %d %s",
                    old_arm, msg->arming_state,
                    (msg->arming_state == 2 ? "(ARMED)" : "(DISARMED)"));
    }
}

void OffboardManager::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_local_x_.store(msg->x);
    current_local_y_.store(msg->y);
    current_local_z_.store(msg->z);
    current_yaw_.store(msg->heading);
    position_received_.store(true);
}

void OffboardManager::vehicleGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    current_lat_.store(msg->lat);
    current_lon_.store(msg->lon);
    current_alt_amsl_.store(msg->alt);

    // Home 위치 저장 (최초 1회)
    if (!home_set_) {
        home_lat_ = msg->lat;
        home_lon_ = msg->lon;
        home_alt_amsl_ = msg->alt;
        home_set_ = true;
        RCLCPP_INFO(node_->get_logger(), "[HOME] Position set: (%.7f, %.7f, %.1f m)",
                    home_lat_, home_lon_, home_alt_amsl_);
    }
}

float OffboardManager::calculateTargetYaw(float target_north, float target_east)
{
    // NED 좌표계에서 방위각 계산: atan2(East, North)
    return std::atan2(target_east, target_north);
}

void OffboardManager::gpsToLocalNED(double target_lat, double target_lon, float target_alt,
                                     float& local_x, float& local_y, float& local_z)
{
    if (!home_set_) {
        RCLCPP_WARN(node_->get_logger(), "[GPS->NED] Home not set, using current position");
        local_x = 0.0f;
        local_y = 0.0f;
        local_z = -target_alt;
        return;
    }

    // GPS → 로컬 NED 변환
    constexpr double DEG_TO_M_LAT = 111320.0;
    const double deg_to_m_lon = 111320.0 * std::cos(home_lat_ * M_PI / 180.0);

    // NED: X = North, Y = East, Z = Down
    local_x = static_cast<float>((target_lat - home_lat_) * DEG_TO_M_LAT);
    local_y = static_cast<float>((target_lon - home_lon_) * deg_to_m_lon);
    local_z = -target_alt;  // 상대 고도 (Down이 양수)
}

void OffboardManager::abortMission()
{
    RCLCPP_WARN(node_->get_logger(), "[ABORT] Mission abort requested!");
    abort_requested_.store(true);
}

void OffboardManager::emergencyRTL()
{
    RCLCPP_ERROR(node_->get_logger(), "[EMERGENCY] RTL triggered!");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
    current_state_.store(MissionState::RTL);
}

bool OffboardManager::updateMissionTarget(const GPSCoordinate& new_target, float yaw_override)
{
    if (!mission_running_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[UPDATE] No mission running, cannot update target");
        return false;
    }

    auto state = current_state_.load();

    // RTL/LANDED/ERROR 상태에서는 업데이트 불가
    if (state == MissionState::RTL || state == MissionState::LANDED ||
        state == MissionState::ERROR) {
        RCLCPP_WARN(node_->get_logger(), "[UPDATE] Cannot update target in %s state", getStateName(state).c_str());
        return false;
    }

    // HOVER_AT_TARGET: 새 목표 수신 시 NAVIGATE 복귀 (리더/팔로워 모두)
    if (state == MissionState::HOVER_AT_TARGET) {
        RCLCPP_INFO(node_->get_logger(), "[UPDATE] HOVER_AT_TARGET → NAVIGATE (new target received%s)",
                    continuous_update_mode_.load() ? ", continuous" : "");
        current_state_.store(MissionState::NAVIGATE);
        setpoint_counter_.store(MOVE_START + 1);
        state = MissionState::NAVIGATE;
    }

    // 미션 설정 업데이트
    mission_config_.target_waypoint = new_target;

    // ★★★ start_local 보존: HOVER/ROTATE 위치 유지를 위해 start_local을 리셋하지 않음 ★★★
    // start_local_x/y는 executeMission3()에서 1회 설정 (이륙 위치)
    // HOVER/ROTATE에서 start_local을 hover 기준점으로 사용하므로,
    // 여기서 리셋하면 position hold가 무력화됨 (위치오차=0 → 보정력=0 → 바람에 표류)
    float ref_local_x = current_local_x_.load();
    float ref_local_y = current_local_y_.load();

    // 현재 GPS 위치 기준으로 새 목표 NED 좌표 계산
    double cur_lat = current_lat_.load();
    double cur_lon = current_lon_.load();

    constexpr double DEG_TO_M_LAT = 111320.0;
    double deg_to_m_lon = 111320.0 * std::cos(cur_lat * M_PI / 180.0);

    // 현재 위치에서 새 목표까지의 오프셋
    float offset_north = static_cast<float>((new_target.latitude - cur_lat) * DEG_TO_M_LAT);
    float offset_east = static_cast<float>((new_target.longitude - cur_lon) * deg_to_m_lon);

    // 현재 로컬 위치 + 오프셋 = 새 목표 NED (start_local 대신 현재 로컬 좌표 사용)
    float old_x = target_ned_x_;
    float old_y = target_ned_y_;

    target_ned_x_ = ref_local_x + offset_north;
    target_ned_y_ = ref_local_y + offset_east;

    // 고도 설정: 팔로워(연속 추적)는 AMSL→NED 변환, 리더/단독은 AGL 직접 사용
    if (continuous_update_mode_.load() && home_set_ && new_target.altitude > 0.0f) {
        // 팔로워: 리더의 AMSL 고도를 수신 → home 기준 NED 변환
        target_ned_z_ = -(new_target.altitude - home_alt_amsl_);
    } else if (new_target.altitude > 0.0f) {
        // 리더/단독: target_alt는 이륙 기준 상대 고도(AGL) → 직접 NED Z로 변환
        target_ned_z_ = -new_target.altitude;
    }
    // altitude == 0이면 기존 target_ned_z_ 유지 (고도 변경 없음)

    // 새 목표 방향 계산 (yaw_override가 있으면 리더 헤딩 사용)
    if (!std::isnan(yaw_override)) {
        target_yaw_ = yaw_override;
    } else {
        target_yaw_ = calculateTargetYaw(offset_north, offset_east);
    }

    // initial_yaw_도 보존: HOVER에서 yaw_setpoint로 사용됨, 리셋하면 yaw hold도 무력화

    // 로깅 (continuous_update_mode에서는 5초마다만)
    bool should_log = true;
    if (continuous_update_mode_.load()) {
        static uint32_t continuous_log_count = 0;
        should_log = (continuous_log_count++ % 50 == 0);
    }

    if (should_log) {
        RCLCPP_INFO(node_->get_logger(), "==============================================");
        RCLCPP_INFO(node_->get_logger(), "  [UPDATE] New Target%s",
                    continuous_update_mode_.load() ? " (continuous)" : " - Smooth Turn");
        RCLCPP_INFO(node_->get_logger(), "  State: %s | GPS: (%.7f, %.7f) | NED: (%.1f, %.1f, %.1f)",
                    getStateName(state).c_str(), new_target.latitude, new_target.longitude,
                    target_ned_x_, target_ned_y_, target_ned_z_);
        RCLCPP_INFO(node_->get_logger(), "  Yaw: %.1f deg", target_yaw_ * 180.0f / M_PI);
        RCLCPP_INFO(node_->get_logger(), "==============================================");
    }

    // ★★★ 상태별 목적지 업데이트 처리 ★★★
    if (state == MissionState::PREPARING || state == MissionState::OFFBOARD ||
        state == MissionState::ARMING || state == MissionState::TAKEOFF) {
        // ARM/이륙 전: 목표 좌표만 저장, 상태 전환 안 함 (시퀀스 유지)
        if (should_log) RCLCPP_INFO(node_->get_logger(), "  → Target saved, state unchanged (%s)", getStateName(state).c_str());
    } else if (state == MissionState::NAVIGATE) {
        // NAVIGATE 중: 정지 없이 새 target으로 부드럽게 선회하면서 이동
        if (should_log) RCLCPP_INFO(node_->get_logger(), "  → NAVIGATE state maintained (smooth turn)");
    } else if (state == MissionState::ROTATE || state == MissionState::HOVER) {
        if (continuous_update_mode_.load()) {
            // 팔로워 연속 추적 모드: 좌표만 업데이트, 상태 전환 안 함 (카운터 진행 유지)
            if (should_log) RCLCPP_INFO(node_->get_logger(), "  → Target updated, state unchanged (continuous)");
        } else {
            // ROTATE/HOVER 중: 새 목적지로 헤딩 정렬 재시작
            setpoint_counter_.store(ROTATE_START);
            current_state_.store(MissionState::ROTATE);
            RCLCPP_INFO(node_->get_logger(), "  → ROTATE restart for new target (heading realign)");
        }
    } else {
        // 기타: NAVIGATE로 전환
        setpoint_counter_.store(MOVE_START + 1);
        current_state_.store(MissionState::NAVIGATE);
        RCLCPP_INFO(node_->get_logger(), "  → State changed to NAVIGATE (immediate move)");
    }

    return true;
}

void OffboardManager::resetToIdle()
{
    current_state_.store(MissionState::IDLE);
    mission_running_.store(false);
    abort_requested_.store(false);
    setpoint_counter_.store(0);

    // Home 위치 리셋 (다음 미션에서 현재 위치 기준으로 재설정)
    home_set_ = false;

    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }

    RCLCPP_INFO(node_->get_logger(), "[RESET] State reset to IDLE (home_set_ cleared)");
}

void OffboardManager::setFormationReadyToRotate(bool ready) {
    formation_ready_to_rotate_.store(ready);
    RCLCPP_INFO(node_->get_logger(), "[FORMATION] ready_to_rotate = %s", ready ? "true" : "false");
}

void OffboardManager::setFormationReadyToNavigate(bool ready) {
    formation_ready_to_navigate_.store(ready);
    RCLCPP_INFO(node_->get_logger(), "[FORMATION] ready_to_navigate = %s", ready ? "true" : "false");
}

void OffboardManager::setContinuousUpdateMode(bool enabled) {
    continuous_update_mode_.store(enabled);
    RCLCPP_INFO(node_->get_logger(), "[FORMATION] continuous_update_mode = %s", enabled ? "true" : "false");
}

std::string OffboardManager::getStateName(MissionState state)
{
    switch (state) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::PREPARING: return "PREPARING";
        case MissionState::OFFBOARD: return "OFFBOARD";
        case MissionState::ARMING: return "ARMING";
        case MissionState::TAKEOFF: return "TAKEOFF";
        case MissionState::HOVER: return "HOVER";
        case MissionState::ROTATE: return "ROTATE";
        case MissionState::NAVIGATE: return "NAVIGATE";
        case MissionState::HOVER_AT_TARGET: return "HOVER_AT_TARGET";
        case MissionState::RTL: return "RTL";
        case MissionState::LANDED: return "LANDED";
        case MissionState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}
