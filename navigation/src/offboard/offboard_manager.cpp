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
#include "collision/collision_avoidance.h"
#include "handlers/prepare_handler.h"
#include "handlers/offboard_handler.h"
#include "handlers/arm_handler.h"
#include "handlers/takeoff_handler.h"
#include "handlers/hover_handler.h"
#include "handlers/rotate_handler.h"
#include "handlers/navigate_handler.h"
#include "handlers/hover_at_target_handler.h"
#include "handlers/rtl_handler.h"
#include <cstdlib>

using namespace std::chrono_literals;

OffboardManager::OffboardManager(rclcpp::Node::SharedPtr node, const std::string& px4_ns)
    : node_(node)
{

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

    // ========== 핸들러 초기화 ==========
    prepare_handler_ = std::make_unique<PrepareHandler>();
    offboard_handler_ = std::make_unique<OffboardHandler>();
    arm_handler_ = std::make_unique<ArmHandler>();
    takeoff_handler_ = std::make_unique<TakeoffHandler>();
    hover_handler_ = std::make_unique<HoverHandler>();
    rotate_handler_ = std::make_unique<RotateHandler>();
    navigate_handler_ = std::make_unique<NavigateHandler>();
    hover_at_target_handler_ = std::make_unique<HoverAtTargetHandler>();
    rtl_handler_ = std::make_unique<RtlHandler>();

    // ctx_ 명령 발행 콜백 설정
    ctx_.publishCommand = [this](uint16_t cmd, float p1, float p2, float p3) {
        publishVehicleCommand(cmd, p1, p2, p3);
    };
    ctx_.logger = node_->get_logger();
    ctx_.target_system = target_system_;

    RCLCPP_INFO(node_->get_logger(), "OffboardManager initialized (ns=%s, target_sys=%d, handler_arch=ON)",
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
    // ★ 미션 시작 구간 mutex (중복 스레드 동시 시작 방지)
    // ApplicationManager에서 스레드로 호출 시, 중복 커스텀 메시지로 두 스레드가
    // 동시에 진입하면 타이머가 2개 생성되는 race condition 방지
    {
        std::lock_guard<std::mutex> lock(mission_start_mutex_);

        // 이미 미션 실행 중이면 좌표만 업데이트 (경로 변경)
        if (mission_running_.load()) {
            auto state = current_state_.load();
            if (arming_state_.load() != 2) {  // 2 = ARMED
                // 이륙 시퀀스 진행 중(PREPARING/OFFBOARD/ARMING/TAKEOFF)이면 좌표만 저장
                if (state == MissionState::PREPARING || state == MissionState::OFFBOARD ||
                    state == MissionState::ARMING || state == MissionState::TAKEOFF) {
                    RCLCPP_INFO(node_->get_logger(),
                        "[MISSION] Startup in progress (state=%s), updating target only (no reset)",
                        getStateName(state).c_str());
                    mission_config_.target_waypoint = config.target_waypoint;
                    mission_config_.takeoff_altitude = config.takeoff_altitude;
                    mission_config_.flight_speed = config.flight_speed;
                    if (config.target_altitude >= 0.0f) {
                        mission_config_.target_altitude = config.target_altitude;
                    }
                    return true;
                }
                // 이륙 시퀀스가 아닌데 DISARMED → 이전 미션 비정상 종료, 상태 리셋
                RCLCPP_WARN(node_->get_logger(),
                    "[MISSION] Stale mission state detected! (mission_running=true, armed=OFF, state=%s) → Resetting",
                    getStateName(state).c_str());
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

        // ========== 핸들러 컨텍스트 초기화 ==========
        ctx_.reset();
        ctx_.loadFromConfig(config);
        ctx_.formation_mode = formation_mode_;
        ctx_.target_system = target_system_;
        syncContextFromMembers();

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

        // 상태 초기화 → 핸들러 기반
        current_state_.store(MissionState::PREPARING);
        transitionTo(prepare_handler_.get());

        // 타이머 시작 (10Hz)
        timer_ = node_->create_wall_timer(
            100ms,
            std::bind(&OffboardManager::timerCallback, this));
    } // ★ mutex 해제 — 이후 대기 루프는 mutex 밖에서 실행

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
    // ★ DDS 연결 유지: OffboardControlMode를 항상 발행
    auto state = current_state_.load();
    if (state != MissionState::RTL && state != MissionState::LANDED) {
        publishOffboardControlMode();
    }

    // 미션이 실행 중이 아니면 heartbeat만 발행하고 종료
    if (!mission_running_.load()) {
        return;
    }

    // 중단 요청 확인
    if (abort_requested_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[ABORT] Mission aborted, triggering RTL...");
        abort_requested_.store(false);
        if (current_handler_) {
            current_handler_->onExit(ctx_);
        }
        current_state_.store(MissionState::RTL);
        transitionTo(rtl_handler_.get());
        return;
    }

    state = current_state_.load();

    // ========== 충돌 방지 체크 (10Hz, ARM 이후) ==========
    if (collision_avoidance_ && arming_state_.load() == 2) {
        auto action = collision_avoidance_->checkAndUpdate();
        bool active = (action != CollisionAction::NONE);

        if (active && !was_collision_active_) {
            if (action == CollisionAction::HOLD) {
                hold_x_ = current_local_x_.load();
                hold_y_ = current_local_y_.load();
                hold_z_ = current_local_z_.load();
                hold_yaw_ = current_yaw_.load();
                RCLCPP_WARN(node_->get_logger(),
                    "[COLLISION] HOLD at (%.1f, %.1f, %.1f) - threat drone %d",
                    hold_x_, hold_y_, -hold_z_,
                    collision_avoidance_->getThreatId());
            } else {
                hold_x_ = current_local_x_.load();
                hold_y_ = current_local_y_.load();
                hold_z_ = current_local_z_.load();
                hold_yaw_ = current_yaw_.load();
                evade_offset_n_ = collision_avoidance_->getEvadeOffsetN();
                evade_offset_e_ = collision_avoidance_->getEvadeOffsetE();
                RCLCPP_WARN(node_->get_logger(),
                    "[COLLISION] EVADE RIGHT offset=(%.1f, %.1f) - threat drone %d",
                    evade_offset_n_, evade_offset_e_,
                    collision_avoidance_->getThreatId());
            }
        } else if (!active && was_collision_active_) {
            RCLCPP_INFO(node_->get_logger(),
                "[COLLISION] CLEAR - resuming %s",
                getStateName(state).c_str());
            evade_offset_n_ = 0.0f;
            evade_offset_e_ = 0.0f;

            // HOVER_AT_TARGET에서 새 타겟 감지 → NAVIGATE 전환
            if (state == MissionState::HOVER_AT_TARGET) {
                float dx = target_ned_x_ - current_local_x_.load();
                float dy = target_ned_y_ - current_local_y_.load();
                if (std::sqrt(dx * dx + dy * dy) > WAYPOINT_THRESHOLD * 2) {
                    RCLCPP_INFO(node_->get_logger(),
                        "[COLLISION] Post-clear: deferred target (%.1fm), NAVIGATE",
                        std::sqrt(dx * dx + dy * dy));
                    current_state_.store(MissionState::NAVIGATE);
                    syncContextFromMembers();
                    transitionTo(navigate_handler_.get());
                }
            }
        } else if (active && action == CollisionAction::EVADE_RIGHT) {
            evade_offset_n_ = collision_avoidance_->getEvadeOffsetN();
            evade_offset_e_ = collision_avoidance_->getEvadeOffsetE();
        }

        collision_action_.store(static_cast<int>(action));
        was_collision_active_ = active;

        // ctx_에 충돌 상태 동기화
        ctx_.collision_action.store(static_cast<int>(action));
        ctx_.hold_x = hold_x_;
        ctx_.hold_y = hold_y_;
        ctx_.hold_z = hold_z_;
        ctx_.hold_yaw = hold_yaw_;
        ctx_.evade_offset_n = evade_offset_n_;
        ctx_.evade_offset_e = evade_offset_e_;
    }

    // ========== 핸들러 모드 ==========
    if (current_handler_) {
        syncContextFromMembers();

        auto result = current_handler_->tick(ctx_);

        switch (result) {
            case TransitionResult::STAY:
                break;
            case TransitionResult::COMPLETE:
                advanceToNextHandler();
                break;
            case TransitionResult::ABORT_RTL:
                current_handler_->onExit(ctx_);
                current_state_.store(MissionState::RTL);
                transitionTo(rtl_handler_.get());
                return;
            case TransitionResult::ERROR:
                current_handler_->onExit(ctx_);
                current_handler_ = nullptr;
                current_state_.store(MissionState::ERROR);
                mission_running_.store(false);
                RCLCPP_ERROR(node_->get_logger(), "[ERROR] Handler error, mission aborted");
                return;
        }

        // setpoint 발행
        if (current_handler_) {
            px4_msgs::msg::TrajectorySetpoint sp{};
            sp.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
            if (current_handler_->fillSetpoint(ctx_, sp)) {
                trajectory_setpoint_pub_->publish(sp);
            }
        }
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
    // 모든 상태가 핸들러로 대체됨 — 이 함수는 더 이상 호출되지 않음
    // (선언은 헤더에 남아있으므로 빈 구현 유지)
    RCLCPP_WARN_ONCE(node_->get_logger(),
        "[LEGACY] publishTrajectorySetpoint() called - should not happen in handler mode");
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
    actual_vx_.store(msg->vx);
    actual_vy_.store(msg->vy);
    actual_vz_.store(msg->vz);
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
    collision_action_.store(0);
    was_collision_active_ = false;
    evade_offset_n_ = 0.0f;
    evade_offset_e_ = 0.0f;
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

    // 충돌 방지 상태 확인
    bool collision_active = (collision_action_.load() != 0);

    // HOVER_AT_TARGET: 새 목표 수신 시 NAVIGATE 복귀 (리더/팔로워 모두)
    if (state == MissionState::HOVER_AT_TARGET) {
        if (collision_active) {
            // 충돌 중: 좌표만 저장, 상태 전환 보류 (해제 시 timerCallback에서 처리)
            RCLCPP_WARN(node_->get_logger(),
                "[UPDATE] HOVER_AT_TARGET new target, but collision active - deferring state change");
        } else {
            RCLCPP_INFO(node_->get_logger(), "[UPDATE] HOVER_AT_TARGET → NAVIGATE (new target received%s)",
                        continuous_update_mode_.load() ? ", continuous" : "");
            current_state_.store(MissionState::NAVIGATE);
            syncContextFromMembers();
            transitionTo(navigate_handler_.get());
            state = MissionState::NAVIGATE;
        }
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
        // 리더/단독: target_alt는 시작 위치 기준 상대 고도(AGL)
        target_ned_z_ = start_local_z_ - new_target.altitude;
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

    // ★★★ 상태별 목적지 업데이트 처리 (핸들러 모드) ★★★
    // ctx_에도 목표 반영
    ctx_.target_ned_x = target_ned_x_;
    ctx_.target_ned_y = target_ned_y_;
    ctx_.target_ned_z = target_ned_z_;
    ctx_.target_yaw = target_yaw_;
    ctx_.target_lat = new_target.latitude;
    ctx_.target_lon = new_target.longitude;

    if (state == MissionState::PREPARING || state == MissionState::OFFBOARD ||
        state == MissionState::ARMING || state == MissionState::TAKEOFF) {
        if (should_log) RCLCPP_INFO(node_->get_logger(), "  -> Target saved, state unchanged (%s)", getStateName(state).c_str());
    } else if (collision_active) {
        if (should_log) RCLCPP_WARN(node_->get_logger(), "  -> Target saved, state change DEFERRED (collision active, state=%s)", getStateName(state).c_str());
    } else if (state == MissionState::NAVIGATE) {
        if (should_log) RCLCPP_INFO(node_->get_logger(), "  -> NAVIGATE state maintained (smooth turn)");
    } else if (state == MissionState::ROTATE || state == MissionState::HOVER) {
        if (continuous_update_mode_.load()) {
            if (should_log) RCLCPP_INFO(node_->get_logger(), "  -> Target updated, state unchanged (continuous)");
        } else {
            // ROTATE/HOVER 중: 새 목적지로 ROTATE 재시작 (핸들러)
            current_state_.store(MissionState::ROTATE);
            transitionTo(rotate_handler_.get());
            RCLCPP_INFO(node_->get_logger(), "  -> ROTATE restart for new target (heading realign)");
        }
    } else {
        // 기타 (HOVER_AT_TARGET 등): NAVIGATE 핸들러로 전환
        current_state_.store(MissionState::NAVIGATE);
        syncContextFromMembers();
        transitionTo(navigate_handler_.get());
        RCLCPP_INFO(node_->get_logger(), "  -> State changed to NAVIGATE (immediate move)");
    }

    return true;
}

void OffboardManager::resetToIdle()
{
    // 핸들러 정리
    if (current_handler_) {
        current_handler_->onExit(ctx_);
        current_handler_ = nullptr;
    }

    current_state_.store(MissionState::IDLE);
    mission_running_.store(false);
    abort_requested_.store(false);
    setpoint_counter_.store(0);
    collision_action_.store(0);
    was_collision_active_ = false;
    evade_offset_n_ = 0.0f;
    evade_offset_e_ = 0.0f;

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

// ========== 핸들러 아키텍처 메서드 ==========

void OffboardManager::transitionTo(StateHandler* handler)
{
    if (current_handler_) {
        current_handler_->onExit(ctx_);
    }
    current_handler_ = handler;
    if (current_handler_) {
        ctx_.state_enter_time = std::chrono::steady_clock::now();
        current_handler_->onEnter(ctx_);
        RCLCPP_INFO(node_->get_logger(), "[STATE] → %s", current_handler_->name());
    }
}

void OffboardManager::advanceToNextHandler()
{
    if (!current_handler_) return;

    current_handler_->onExit(ctx_);

    if (current_handler_ == prepare_handler_.get()) {
        // PREPARE → OFFBOARD
        current_state_.store(MissionState::OFFBOARD);
        transitionTo(offboard_handler_.get());

    } else if (current_handler_ == offboard_handler_.get()) {
        // OFFBOARD → ARM
        current_state_.store(MissionState::ARMING);
        transitionTo(arm_handler_.get());

    } else if (current_handler_ == arm_handler_.get()) {
        // ARM 완료 → TAKEOFF (핸들러)
        current_state_.store(MissionState::TAKEOFF);
        transitionTo(takeoff_handler_.get());

    } else if (current_handler_ == takeoff_handler_.get()) {
        // TAKEOFF 완료 → 목표 NED 계산 → HOVER 또는 NAVIGATE(팔로워)

        // 목표 NED 좌표 계산
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
        target_ned_z_ = start_local_z_ - mission_config_.takeoff_altitude;
        target_yaw_ = calculateTargetYaw(offset_north, offset_east);

        // ctx_에도 반영
        ctx_.target_ned_x = target_ned_x_;
        ctx_.target_ned_y = target_ned_y_;
        ctx_.target_ned_z = target_ned_z_;
        ctx_.target_yaw = target_yaw_;

        RCLCPP_INFO(node_->get_logger(), "[NAV] GPS: (%.7f, %.7f) → (%.7f, %.7f)",
                    cur_lat, cur_lon, tgt_lat, tgt_lon);
        RCLCPP_INFO(node_->get_logger(), "[NAV] Target NED: (%.1f, %.1f, %.1f), Yaw: %.1f°",
                    target_ned_x_, target_ned_y_, target_ned_z_,
                    target_yaw_ * 180.0f / M_PI);

        if (continuous_update_mode_.load()) {
            // 팔로워: HOVER/ROTATE 건너뛰고 바로 NAVIGATE 핸들러
            current_state_.store(MissionState::NAVIGATE);
            transitionTo(navigate_handler_.get());
            RCLCPP_INFO(node_->get_logger(),
                "[TAKEOFF→NAV] 편대 추적 즉시 시작 (NAVIGATE direct)");
        } else {
            // 리더/단독: HOVER 핸들러로 전환
            current_state_.store(MissionState::HOVER);
            transitionTo(hover_handler_.get());
        }

    } else if (current_handler_ == hover_handler_.get()) {
        // HOVER 완료 → ROTATE 핸들러
        current_state_.store(MissionState::ROTATE);
        transitionTo(rotate_handler_.get());

    } else if (current_handler_ == rotate_handler_.get()) {
        // ROTATE 완료 → NAVIGATE 핸들러
        current_state_.store(MissionState::NAVIGATE);
        transitionTo(navigate_handler_.get());

    } else if (current_handler_ == navigate_handler_.get()) {
        // NAVIGATE 완료 → HOVER_AT_TARGET 핸들러
        current_state_.store(MissionState::HOVER_AT_TARGET);
        transitionTo(hover_at_target_handler_.get());

    } else if (current_handler_ == hover_at_target_handler_.get()) {
        // HOVER_AT_TARGET 완료 → RTL 핸들러
        current_state_.store(MissionState::RTL);
        transitionTo(rtl_handler_.get());

    } else if (current_handler_ == rtl_handler_.get()) {
        // RTL 완료 (disarm 감지) → LANDED
        current_handler_ = nullptr;
        current_state_.store(MissionState::LANDED);
        mission_running_.store(false);
        RCLCPP_INFO(node_->get_logger(), "[RTL→LANDED] 미션 완료!");
    }
}

void OffboardManager::syncContextFromMembers()
{
    // 기존 atomic 멤버 → ctx_ 동기화 (핸들러가 읽을 수 있도록)
    ctx_.current_local_x.store(current_local_x_.load());
    ctx_.current_local_y.store(current_local_y_.load());
    ctx_.current_local_z.store(current_local_z_.load());
    ctx_.current_yaw.store(current_yaw_.load());
    ctx_.current_lat.store(current_lat_.load());
    ctx_.current_lon.store(current_lon_.load());
    ctx_.current_alt_amsl.store(current_alt_amsl_.load());
    ctx_.actual_vx.store(actual_vx_.load());
    ctx_.actual_vy.store(actual_vy_.load());
    ctx_.actual_vz.store(actual_vz_.load());
    ctx_.position_received.store(position_received_.load());
    ctx_.nav_state.store(nav_state_.load());
    ctx_.arming_state.store(arming_state_.load());
    ctx_.formation_ready_to_rotate.store(formation_ready_to_rotate_.load());
    ctx_.formation_ready_to_navigate.store(formation_ready_to_navigate_.load());
    ctx_.continuous_update_mode.store(continuous_update_mode_.load());
    ctx_.collision_action.store(collision_action_.load());
}

void OffboardManager::syncMembersFromContext()
{
    // ctx_ → 기존 멤버 동기화 (핸들러가 설정한 값을 레거시 코드에서 사용)
    start_local_x_ = ctx_.start_local_x;
    start_local_y_ = ctx_.start_local_y;
    start_local_z_ = ctx_.start_local_z;
    initial_yaw_ = ctx_.initial_yaw;
    target_ned_x_ = ctx_.target_ned_x;
    target_ned_y_ = ctx_.target_ned_y;
    target_ned_z_ = ctx_.target_ned_z;
    target_yaw_ = ctx_.target_yaw;
    prev_vx_ = ctx_.prev_vx;
    prev_vy_ = ctx_.prev_vy;
    home_lat_ = ctx_.home_lat;
    home_lon_ = ctx_.home_lon;
    home_alt_amsl_ = ctx_.home_alt_amsl;
    home_set_ = ctx_.home_set;
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
