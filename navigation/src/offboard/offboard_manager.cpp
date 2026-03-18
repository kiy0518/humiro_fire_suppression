/**
 * @file offboard_manager.cpp
 * @brief PX4 Offboard 제어 - DDS Domain 분리 버전
 *
 * v2.0: px4_msgs ROS2 토픽 제거, FCBridgeClient (로컬 UDP IPC)로 대체.
 * FC 통신은 fc_bridge 프로세스(Domain 0)가 담당하고,
 * 이 모듈은 Domain 1 (WiFi, 편대 토픽)에서만 동작.
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
#include "handlers/distance_adjust_handler.h"
#include "handlers/hover_at_target_handler.h"
#include "handlers/tracking_hover_handler.h"
#include "handlers/rtl_handler.h"
#include <cstdlib>

using namespace std::chrono_literals;

OffboardManager::OffboardManager(rclcpp::Node::SharedPtr node,
                                 std::shared_ptr<FCBridgeClient> fc_bridge)
    : node_(node), fc_bridge_(fc_bridge)
{
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
    distance_adjust_handler_ = std::make_unique<DistanceAdjustHandler>();
    hover_at_target_handler_ = std::make_unique<HoverAtTargetHandler>();
    tracking_hover_handler_ = std::make_unique<TrackingHoverHandler>();
    rtl_handler_ = std::make_unique<RtlHandler>();

    // ========== OSD 상태 퍼블리셔 ==========
    offboard_status_pub_ = node_->create_publisher<std_msgs::msg::String>("/offboard/status", 10);

    // ctx_ 명령 발행 콜백 설정 (FCBridgeClient를 통해 전송)
    ctx_.publishCommand = [this](uint16_t cmd, float p1, float p2, float p3) {
        publishVehicleCommand(cmd, p1, p2, p3);
    };
    ctx_.logger = node_->get_logger();
    ctx_.target_system = target_system_;

    RCLCPP_INFO(node_->get_logger(), "OffboardManager initialized (fc_bridge=IPC, target_sys=%d, handler_arch=ON)",
                (int)target_system_);
}

OffboardManager::~OffboardManager()
{
    if (timer_) {
        timer_->cancel();
    }
}

bool OffboardManager::executeMissionSolo(const MissionConfig& config)
{
    formation_mode_ = false;
    RCLCPP_INFO(node_->get_logger(), "[SOLO] 단독비행 미션 시작");
    return executeMissionInternal(config);
}

bool OffboardManager::executeMissionFormation(const MissionConfig& config)
{
    formation_mode_ = true;
    RCLCPP_INFO(node_->get_logger(), "[FORMATION] 편대비행 미션 시작");
    return executeMissionInternal(config);
}

bool OffboardManager::executeMissionSwarm(const MissionConfig& config)
{
    formation_mode_ = false;  // 편대 게이트 비활성
    swarm_mode_ = true;
    continuous_update_mode_.store(false);  // swarm: 독립 비행이므로 NAVIGATE 도착 판정 활성화
    RCLCPP_INFO(node_->get_logger(), "[SWARM] 독립 편대 미션 시작");
    return executeMissionInternal(config);
}

void OffboardManager::setSwarmSuppressTarget(double lat, double lon, float yaw)
{
    ctx_.swarm_suppress_lat = lat;
    ctx_.swarm_suppress_lon = lon;
    ctx_.swarm_suppress_yaw = yaw;
    ctx_.swarm_alignment_received.store(true);
    RCLCPP_INFO(node_->get_logger(), "[SWARM] 진압 위치 설정: (%.7f, %.7f), yaw=%.1f°",
                lat, lon, yaw * 180.0f / M_PI);
}

bool OffboardManager::executeMissionInternal(const MissionConfig& config)
{
    const char* mode_str = swarm_mode_ ? "SWARM" : (formation_mode_ ? "FORMATION" : "SOLO");

    // ★ 미션 시작 구간 mutex (중복 스레드 동시 시작 방지)
    {
        std::lock_guard<std::mutex> lock(mission_start_mutex_);

        // 이미 미션 실행 중이면 좌표만 업데이트 (경로 변경)
        if (mission_running_.load()) {
            auto state = current_state_.load();
            if (arming_state_.load() != 2) {  // 2 = ARMED
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
                RCLCPP_WARN(node_->get_logger(),
                    "[MISSION] Stale mission state detected! (mission_running=true, armed=OFF, state=%s) → Resetting",
                    getStateName(state).c_str());
                mission_running_.store(false);
                current_state_.store(MissionState::IDLE);
                abort_requested_.store(false);
                if (timer_) timer_->cancel();
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

        // Home 위치 리셋
        home_set_ = false;

        // 시작 위치 저장
        start_local_x_ = current_local_x_.load();
        start_local_y_ = current_local_y_.load();
        start_local_z_ = current_local_z_.load();

        // ========== 핸들러 컨텍스트 초기화 ==========
        ctx_.reset();
        ctx_.loadFromConfig(config);
        ctx_.formation_mode = formation_mode_;
        ctx_.swarm_mode = swarm_mode_;
        ctx_.target_system = target_system_;
        syncContextFromMembers();

        RCLCPP_INFO(node_->get_logger(), "==============================================");
        RCLCPP_INFO(node_->get_logger(), "  Starting Mission [%s]", mode_str);
        RCLCPP_INFO(node_->get_logger(), "  Start position (NED): (%.1f, %.1f, %.1f)",
                    start_local_x_, start_local_y_, start_local_z_);
        RCLCPP_INFO(node_->get_logger(), "==============================================");
        RCLCPP_INFO(node_->get_logger(), "  Takeoff altitude: %.1f m", config.takeoff_altitude);
        RCLCPP_INFO(node_->get_logger(), "  Target: Lat=%.7f, Lon=%.7f",
                    config.target_waypoint.latitude, config.target_waypoint.longitude);
        RCLCPP_INFO(node_->get_logger(), "  Hover duration: %.1f sec", config.hover_duration_sec);
        RCLCPP_INFO(node_->get_logger(), "==============================================\n");

        // 상태 초기화 → 핸들러 기반
        current_state_.store(MissionState::PREPARING);
        transitionTo(prepare_handler_.get());

        // 타이머 시작 (10Hz)
        timer_ = node_->create_wall_timer(
            100ms,
            std::bind(&OffboardManager::timerCallback, this));
    } // ★ mutex 해제

    // 미션 완료 또는 중단까지 대기
    while (mission_running_.load()) {
        auto state = current_state_.load();
        if (state == MissionState::LANDED || state == MissionState::ERROR) {
            break;
        }
        if (state == MissionState::RTL && arming_state_.load() == 1) {
            RCLCPP_INFO(node_->get_logger(), "[RTL] Disarm detected in wait loop, mission complete!");
            current_state_.store(MissionState::LANDED);
            publishOffboardStatus("DISARMED");
            mission_running_.store(false);
            break;
        }
        std::this_thread::sleep_for(100ms);
    }

    // 타이머 정지 + 상태 리셋
    {
        std::lock_guard<std::mutex> lock(mission_start_mutex_);
        if (!mission_running_.load()) {
            if (timer_) {
                RCLCPP_INFO(node_->get_logger(), "[CLEANUP] 타이머 cancel (nav=%d, arm=%d)",
                            nav_state_.load(), arming_state_.load());
                timer_->cancel();
                timer_.reset();
            }
            formation_mode_ = false;
            swarm_mode_ = false;
        }
    }
    mission_running_.store(false);

    auto final_state = current_state_.load();
    if (final_state == MissionState::LANDED) {
        RCLCPP_INFO(node_->get_logger(), "==============================================");
        RCLCPP_INFO(node_->get_logger(), "  Mission Complete! [%s]", mode_str);
        RCLCPP_INFO(node_->get_logger(), "==============================================");
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Mission failed [%s] (state: %s)", mode_str, getStateName(final_state).c_str());
        return false;
    }
}

void OffboardManager::timerCallback()
{
    // ★ FCBridgeClient에서 FC 상태 폴링 (10Hz)
    updateFromFCState();

    auto state = current_state_.load();

    // ★ RC 모드 전환 감지 (heartbeat/setpoint 발행보다 반드시 먼저 실행!)
    // 순서가 중요한 이유:
    //   감지 → heartbeat 순서여야 nav_state 변경 즉시 heartbeat 중단 가능.
    //   heartbeat → 감지 순서면 nav_state 변경 tick에도 heartbeat가 1회 발행되어
    //   PX4가 OFFBOARD로 복귀 → 다음 tick에서 recovery → 디바운스 리셋 → 무한 루프.
    if (mission_running_.load() && arming_state_.load() == 2 &&
        state != MissionState::LANDED &&
        state != MissionState::IDLE) {
        if (nav_state_.load() != 14) {
            if (!offboard_lost_tracking_) {
                offboard_lost_start_ = std::chrono::steady_clock::now();
                offboard_lost_tracking_ = true;
                RCLCPP_WARN(node_->get_logger(),
                    "[OFFBOARD] nav_state=%d (not OFFBOARD), heartbeat+setpoint 즉시 중단 (state=%s)",
                    nav_state_.load(), getStateName(state).c_str());
            } else {
                auto elapsed = std::chrono::steady_clock::now() - offboard_lost_start_;
                if (elapsed > std::chrono::seconds(2)) {
                    RCLCPP_WARN(node_->get_logger(),
                        "[RC OVERRIDE] nav_state=%d for %.1fs → mission abort (state=%s)",
                        nav_state_.load(),
                        std::chrono::duration<float>(elapsed).count(),
                        getStateName(state).c_str());
                    offboard_lost_tracking_ = false;
                    if (current_handler_) {
                        current_handler_->onExit(ctx_);
                        current_handler_ = nullptr;
                    }
                    current_state_.store(MissionState::IDLE);
                    mission_running_.store(false);
                    abort_requested_.store(false);
                    if (timer_) timer_->cancel();
                    return;
                }
            }
        } else {
            if (offboard_lost_tracking_) {
                auto elapsed = std::chrono::steady_clock::now() - offboard_lost_start_;
                RCLCPP_INFO(node_->get_logger(),
                    "[OFFBOARD] Recovered after %.0fms (count=%d)",
                    std::chrono::duration<float>(elapsed).count() * 1000.0f,
                    offboard_recovery_count_ + 1);
                offboard_lost_tracking_ = false;
                offboard_recovery_count_++;
            }
        }
    }

    // ★ OFFBOARD heartbeat 발행 (RC override 감지 후!)
    // offboard_lost_tracking_ == true → heartbeat 즉시 중단
    // → PX4가 OFFBOARD 모드에서 이탈, 조종기 모드 유지
    if (mission_running_.load() && state != MissionState::LANDED && !offboard_lost_tracking_) {
        publishOffboardControlMode();
    }

    if (!mission_running_.load()) return;

    // 중단 요청
    if (abort_requested_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[ABORT] Mission aborted, triggering RTL...");
        abort_requested_.store(false);
        if (current_handler_) current_handler_->onExit(ctx_);
        current_state_.store(MissionState::RTL);
        transitionTo(rtl_handler_.get());
        return;
    }

    state = current_state_.load();

    // ========== 충돌 방지 (10Hz, ARM 이후) ==========
    if (collision_avoidance_ && arming_state_.load() == 2) {
        auto action = collision_avoidance_->checkAndUpdate();
        bool active = (action != CollisionAction::NONE);

        if (active && !was_collision_active_) {
            hold_x_ = current_local_x_.load();
            hold_y_ = current_local_y_.load();
            hold_z_ = current_local_z_.load();
            hold_yaw_ = current_yaw_.load();
            if (action == CollisionAction::HOLD) {
                RCLCPP_WARN(node_->get_logger(), "[COLLISION] HOLD at (%.1f, %.1f, %.1f)",
                            hold_x_, hold_y_, -hold_z_);
            } else {
                evade_offset_n_ = collision_avoidance_->getEvadeOffsetN();
                evade_offset_e_ = collision_avoidance_->getEvadeOffsetE();
                RCLCPP_WARN(node_->get_logger(), "[COLLISION] EVADE RIGHT offset=(%.1f, %.1f)",
                            evade_offset_n_, evade_offset_e_);
            }
        } else if (!active && was_collision_active_) {
            RCLCPP_INFO(node_->get_logger(), "[COLLISION] CLEAR - resuming %s", getStateName(state).c_str());
            evade_offset_n_ = 0.0f;
            evade_offset_e_ = 0.0f;
            if (state == MissionState::HOVER_AT_TARGET) {
                float dx = target_ned_x_ - current_local_x_.load();
                float dy = target_ned_y_ - current_local_y_.load();
                if (std::sqrt(dx * dx + dy * dy) > WAYPOINT_THRESHOLD * 2) {
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
                publishOffboardStatus("IDLE");
                mission_running_.store(false);
                RCLCPP_ERROR(node_->get_logger(), "[ERROR] Handler error, mission aborted");
                return;
        }

        // RTL 서브페이즈 OSD 업데이트 (tick에서 ctx_.rtl_sub_phase 갱신)
        if (current_state_.load() == MissionState::RTL && !ctx_.rtl_sub_phase.empty()) {
            std::string rtl_status = "RETURNING:" + ctx_.rtl_sub_phase;
            publishOffboardStatus(rtl_status);
        }

        // setpoint 발행 (핸들러 → FCCommand 변환 → FCBridgeClient)
        // RC override 감지 중(offboard_lost_tracking_)에는 setpoint도 중단
        // → PX4가 OFFBOARD 모드로 잠시 복귀해도 setpoint 없어 제어 불가
        if (current_handler_ && !offboard_lost_tracking_) {
            px4_msgs::msg::TrajectorySetpoint sp{};
            sp.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
            if (current_handler_->fillSetpoint(ctx_, sp)) {
                FCCommand setpoint_cmd = FCCommand::makeSetpoint(
                    sp.position[0], sp.position[1], sp.position[2],
                    sp.velocity[0], sp.velocity[1], sp.velocity[2],
                    sp.yaw, sp.yawspeed);
                fc_bridge_->sendSetpoint(setpoint_cmd);
            }
        }
    }
}

// ========== FC 상태 업데이트 (FCBridgeClient에서 폴링) ==========

void OffboardManager::updateFromFCState()
{
    if (!fc_bridge_ || !fc_bridge_->isConnected()) return;

    FCState state = fc_bridge_->getLatestState();
    if (!state.isValid()) return;

    uint8_t old_nav = nav_state_.load();
    uint8_t old_arm = arming_state_.load();

    nav_state_.store(state.nav_state);
    arming_state_.store(state.arming_state);

    if (old_nav != state.nav_state) {
        const char* nav_name = "UNKNOWN";
        switch (state.nav_state) {
            case 0: nav_name = "MANUAL"; break;
            case 2: nav_name = "ALTCTL"; break;
            case 3: nav_name = "POSCTL"; break;
            case 4: nav_name = "AUTO_LOITER"; break;
            case 5: nav_name = "AUTO_RTL"; break;
            case 14: nav_name = "OFFBOARD"; break;
            case 17: nav_name = "AUTO_TAKEOFF"; break;
            case 18: nav_name = "AUTO_LAND"; break;
        }
        RCLCPP_INFO(node_->get_logger(), "[STATUS] nav_state: %d -> %d (%s) | my_state=%s",
                    old_nav, state.nav_state, nav_name,
                    getStateName(current_state_.load()).c_str());

        if (old_nav == 14 && state.nav_state != 14 && mission_running_.load()) {
            RCLCPP_WARN(node_->get_logger(),
                        "[RC OVERRIDE] OFFBOARD → %s (nav=%d)", nav_name, state.nav_state);
        }
    }
    if (old_arm != state.arming_state) {
        RCLCPP_INFO(node_->get_logger(), "[STATUS] arming: %d -> %d %s",
                    old_arm, state.arming_state,
                    (state.arming_state == 2 ? "(ARMED)" : "(DISARMED)"));
    }

    // 위치/속도
    current_local_x_.store(state.local_x);
    current_local_y_.store(state.local_y);
    current_local_z_.store(state.local_z);
    current_yaw_.store(state.yaw);
    actual_vx_.store(state.vx);
    actual_vy_.store(state.vy);
    actual_vz_.store(state.vz);
    dist_bottom_.store(state.dist_bottom);
    dist_bottom_valid_.store(state.dist_bottom_valid != 0);
    land_detected_.store(state.land_detected != 0);
    maybe_landed_.store(state.maybe_landed != 0);
    position_received_.store(true);

    // GPS
    current_lat_.store(state.latitude);
    current_lon_.store(state.longitude);
    current_alt_amsl_.store(state.altitude_amsl);

    // 배터리/GPS 상태
    battery_remaining_.store(state.battery_remaining);
    gps_fix_type_.store(state.gps_fix_type);
    gps_satellites_.store(state.gps_satellites);

    // Home 위치 (최초 1회)
    if (!home_set_ && state.latitude != 0.0 && state.longitude != 0.0) {
        home_lat_ = state.latitude;
        home_lon_ = state.longitude;
        home_alt_amsl_ = state.altitude_amsl;
        home_set_ = true;
        RCLCPP_INFO(node_->get_logger(), "[HOME] Position set: (%.7f, %.7f, %.1f m)",
                    home_lat_, home_lon_, home_alt_amsl_);
    }
}

// ========== FC Bridge를 통한 메시지 발행 ==========

void OffboardManager::publishOffboardControlMode()
{
    if (!fc_bridge_) return;
    fc_bridge_->sendHeartbeat(true, false, false);

    static uint64_t hb_count = 0;
    hb_count++;
    if (hb_count % 50 == 0 || hb_count <= 3) {
        RCLCPP_INFO(node_->get_logger(), "[HB] Heartbeat #%ld (state=%s, nav=%d, armed=%d, running=%d)",
                     hb_count, getStateName(current_state_.load()).c_str(),
                     nav_state_.load(), arming_state_.load(), mission_running_.load());
    }
}

void OffboardManager::publishSetpoint(const FCCommand& cmd)
{
    if (!fc_bridge_) return;
    fc_bridge_->sendSetpoint(cmd);
}

void OffboardManager::publishVehicleCommand(uint16_t command, float param1, float param2, float param3)
{
    if (!fc_bridge_) return;

    fc_bridge_->sendVehicleCommand(command, param1, param2, param3, target_system_);

    const char* cmd_name = "UNKNOWN";
    if (command == 176) cmd_name = "DO_SET_MODE";
    else if (command == 400) cmd_name = "ARM_DISARM";
    else if (command == 20) cmd_name = "NAV_RETURN_TO_LAUNCH";

    RCLCPP_INFO(node_->get_logger(), "[CMD] %s (cmd=%d, p1=%.1f, p2=%.1f, p3=%.1f) via IPC",
                cmd_name, command, param1, param2, param3);
}

// ========== 헬퍼 함수 ==========

float OffboardManager::calculateTargetYaw(float target_north, float target_east)
{
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

    constexpr double DEG_TO_M_LAT = 111320.0;
    const double deg_to_m_lon = 111320.0 * std::cos(home_lat_ * M_PI / 180.0);
    local_x = static_cast<float>((target_lat - home_lat_) * DEG_TO_M_LAT);
    local_y = static_cast<float>((target_lon - home_lon_) * deg_to_m_lon);
    local_z = -target_alt;
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
    // 이미 RTL 중이거나 착륙 완료 시 중복 호출 무시
    auto state = current_state_.load();
    if (state == MissionState::RTL || state == MissionState::LANDED) {
        RCLCPP_WARN(node_->get_logger(), "[EMERGENCY] RTL 중복 호출 무시 (state=%s)",
                    getStateName(state).c_str());
        return;
    }

    RCLCPP_ERROR(node_->get_logger(), "[EMERGENCY] RTL triggered! (from state=%s)",
                 getStateName(state).c_str());
    publishVehicleCommand(20);  // VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

    // 핸들러 정리: setpoint 발행 중단 (PX4 RTL 모드와 충돌 방지)
    if (current_handler_) {
        current_handler_->onExit(ctx_);
        current_handler_ = nullptr;
    }
    current_state_.store(MissionState::RTL);
    publishOffboardStatus("RETURNING");
    mission_running_.store(false);
}

bool OffboardManager::updateMissionTarget(const GPSCoordinate& new_target, float yaw_override)
{
    if (!mission_running_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[UPDATE] No mission running, cannot update target");
        return false;
    }

    auto state = current_state_.load();
    if (state == MissionState::RTL || state == MissionState::LANDED || state == MissionState::ERROR) {
        RCLCPP_WARN(node_->get_logger(), "[UPDATE] Cannot update target in %s state", getStateName(state).c_str());
        return false;
    }

    bool collision_active = (collision_action_.load() != 0);

    if (state == MissionState::HOVER_AT_TARGET) {
        if (collision_active) {
            RCLCPP_WARN(node_->get_logger(), "[UPDATE] HOVER_AT_TARGET new target, collision active - deferring");
        } else {
            RCLCPP_INFO(node_->get_logger(), "[UPDATE] HOVER_AT_TARGET → NAVIGATE (new target%s)",
                        continuous_update_mode_.load() ? ", continuous" : "");
            current_state_.store(MissionState::NAVIGATE);
            syncContextFromMembers();
            transitionTo(navigate_handler_.get());
            state = MissionState::NAVIGATE;
        }
    }

    mission_config_.target_waypoint = new_target;

    float ref_local_x = current_local_x_.load();
    float ref_local_y = current_local_y_.load();
    double cur_lat = current_lat_.load();
    double cur_lon = current_lon_.load();

    constexpr double DEG_TO_M_LAT = 111320.0;
    double deg_to_m_lon = 111320.0 * std::cos(cur_lat * M_PI / 180.0);

    float offset_north = static_cast<float>((new_target.latitude - cur_lat) * DEG_TO_M_LAT);
    float offset_east = static_cast<float>((new_target.longitude - cur_lon) * deg_to_m_lon);

    target_ned_x_ = ref_local_x + offset_north;
    target_ned_y_ = ref_local_y + offset_east;

    if (continuous_update_mode_.load() && home_set_ && new_target.altitude > 0.0f) {
        target_ned_z_ = -(new_target.altitude - home_alt_amsl_);
    } else if (new_target.altitude > 0.0f) {
        target_ned_z_ = start_local_z_ - new_target.altitude;
    }

    if (!std::isnan(yaw_override)) {
        target_yaw_ = yaw_override;
    } else {
        target_yaw_ = calculateTargetYaw(offset_north, offset_east);
    }

    bool should_log = true;
    if (continuous_update_mode_.load()) {
        static uint32_t continuous_log_count = 0;
        should_log = (continuous_log_count++ % 50 == 0);
    }

    if (should_log) {
        RCLCPP_INFO(node_->get_logger(), "[UPDATE] New Target%s: GPS(%.7f,%.7f) NED(%.1f,%.1f,%.1f) Yaw=%.1f°",
                    continuous_update_mode_.load() ? " (continuous)" : "",
                    new_target.latitude, new_target.longitude,
                    target_ned_x_, target_ned_y_, target_ned_z_, target_yaw_ * 180.0f / M_PI);
    }

    ctx_.target_ned_x = target_ned_x_;
    ctx_.target_ned_y = target_ned_y_;
    ctx_.target_ned_z = target_ned_z_;
    ctx_.target_yaw = target_yaw_;
    ctx_.target_lat = new_target.latitude;
    ctx_.target_lon = new_target.longitude;

    if (state == MissionState::PREPARING || state == MissionState::OFFBOARD ||
        state == MissionState::ARMING || state == MissionState::TAKEOFF) {
        // 이륙 시퀀스: 좌표만 저장
    } else if (collision_active) {
        // 충돌 중: 상태 변경 보류
    } else if (state == MissionState::NAVIGATE) {
        // NAVIGATE: 부드러운 선회
    } else if (state == MissionState::ROTATE || state == MissionState::HOVER) {
        if (!continuous_update_mode_.load()) {
            current_state_.store(MissionState::ROTATE);
            transitionTo(rotate_handler_.get());
        }
    } else {
        current_state_.store(MissionState::NAVIGATE);
        syncContextFromMembers();
        transitionTo(navigate_handler_.get());
    }

    return true;
}

void OffboardManager::resetToIdle()
{
    if (current_handler_) {
        current_handler_->onExit(ctx_);
        current_handler_ = nullptr;
    }

    current_state_.store(MissionState::IDLE);
    publishOffboardStatus("IDLE");
    last_published_status_.clear();
    mission_running_.store(false);
    abort_requested_.store(false);
    setpoint_counter_.store(0);
    collision_action_.store(0);
    was_collision_active_ = false;
    evade_offset_n_ = 0.0f;
    evade_offset_e_ = 0.0f;
    formation_mode_ = false;
    swarm_mode_ = false;
    formation_ready_to_rotate_.store(false);
    formation_ready_to_navigate_.store(false);
    offboard_lost_tracking_ = false;
    offboard_recovery_count_ = 0;
    home_set_ = false;

    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }

    RCLCPP_INFO(node_->get_logger(), "[RESET] State reset to IDLE");
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

void OffboardManager::setForceNavigateComplete(bool force) {
    ctx_.force_navigate_complete.store(force);
    RCLCPP_INFO(node_->get_logger(), "[FORMATION] force_navigate_complete = %s → 개별 진행 허용", force ? "true" : "false");
}

void OffboardManager::setFireAmmoState(std::atomic<int>* index_ptr, int total_count) {
    ctx_.fire_gpio_index_ptr = index_ptr;
    ctx_.fire_gpio_count = total_count;
    RCLCPP_INFO(node_->get_logger(), "[FIRE] 소화탄 상태 연결 (총 %d발)", total_count);
}

void OffboardManager::setThermalData(ThermalData* data_ptr) {
    ctx_.thermal_data_ptr = data_ptr;
    RCLCPP_INFO(node_->get_logger(), "[THERMAL] ThermalData 포인터 연결");
}

void OffboardManager::setThermalTrackingMode(bool auto_mode) {
    ctx_.thermal_tracking_auto.store(auto_mode);
    RCLCPP_INFO(node_->get_logger(), "[THERMAL] 추적 모드: %s", auto_mode ? "자동" : "수동");
}

void OffboardManager::setThermalTrackingActive(bool active) {
    ctx_.thermal_tracking_active.store(active);
    RCLCPP_INFO(node_->get_logger(), "[THERMAL] 추적 %s", active ? "활성화" : "비활성화");
}

void OffboardManager::setLidarInterface(LidarInterface* lidar_ptr) {
    ctx_.lidar_ptr = lidar_ptr;
    RCLCPP_INFO(node_->get_logger(), "[LIDAR] LidarInterface 포인터 연결");
}

// ========== 핸들러 아키텍처 ==========

void OffboardManager::transitionTo(StateHandler* handler)
{
    if (current_handler_) current_handler_->onExit(ctx_);
    current_handler_ = handler;
    if (current_handler_) {
        ctx_.state_enter_time = std::chrono::steady_clock::now();
        current_handler_->onEnter(ctx_);
        RCLCPP_INFO(node_->get_logger(), "[STATE] → %s", current_handler_->name());

        // OSD 상태 발행
        auto state = current_state_.load();
        publishOffboardStatus(missionStateToStatusString(state));
    }
}

void OffboardManager::advanceToNextHandler()
{
    if (!current_handler_) return;
    current_handler_->onExit(ctx_);

    if (current_handler_ == prepare_handler_.get()) {
        current_state_.store(MissionState::OFFBOARD);
        transitionTo(offboard_handler_.get());
    } else if (current_handler_ == offboard_handler_.get()) {
        current_state_.store(MissionState::ARMING);
        transitionTo(arm_handler_.get());
    } else if (current_handler_ == arm_handler_.get()) {
        current_state_.store(MissionState::TAKEOFF);
        transitionTo(takeoff_handler_.get());
    } else if (current_handler_ == takeoff_handler_.get()) {
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

        ctx_.target_ned_x = target_ned_x_;
        ctx_.target_ned_y = target_ned_y_;
        ctx_.target_ned_z = target_ned_z_;
        ctx_.target_yaw = target_yaw_;

        RCLCPP_INFO(node_->get_logger(), "[NAV] Target NED: (%.1f, %.1f, %.1f), Yaw: %.1f°",
                    target_ned_x_, target_ned_y_, target_ned_z_, target_yaw_ * 180.0f / M_PI);

        if (continuous_update_mode_.load()) {
            current_state_.store(MissionState::NAVIGATE);
            transitionTo(navigate_handler_.get());
        } else {
            current_state_.store(MissionState::HOVER);
            transitionTo(hover_handler_.get());
        }
    } else if (current_handler_ == hover_handler_.get()) {
        current_state_.store(MissionState::ROTATE);
        transitionTo(rotate_handler_.get());
    } else if (current_handler_ == rotate_handler_.get()) {
        current_state_.store(MissionState::NAVIGATE);
        transitionTo(navigate_handler_.get());
    } else if (current_handler_ == navigate_handler_.get()) {
        // 거리 조정: LiDAR 포인터가 있고 팔로워가 아닌 경우 DISTANCE_ADJUST
        if (ctx_.lidar_ptr && !ctx_.continuous_update_mode.load()) {
            current_state_.store(MissionState::DISTANCE_ADJUST);
            transitionTo(distance_adjust_handler_.get());
        } else {
            current_state_.store(MissionState::HOVER_AT_TARGET);
            if (ctx_.thermal_data_ptr) {
                transitionTo(tracking_hover_handler_.get());
            } else {
                transitionTo(hover_at_target_handler_.get());
            }
        }
    } else if (current_handler_ == distance_adjust_handler_.get()) {
        // 거리 조정 완료 → HOVER_AT_TARGET / TRACKING_HOVER
        current_state_.store(MissionState::HOVER_AT_TARGET);
        if (ctx_.thermal_data_ptr) {
            transitionTo(tracking_hover_handler_.get());
        } else {
            transitionTo(hover_at_target_handler_.get());
        }
    } else if (current_handler_ == tracking_hover_handler_.get()) {
        current_state_.store(MissionState::RTL);
        transitionTo(rtl_handler_.get());
    } else if (current_handler_ == hover_at_target_handler_.get()) {
        // Swarm 팔로워: 리더 정렬 완료 수신 → 진압 위치로 재이동
        if (ctx_.swarm_reposition_needed) {
            ctx_.swarm_reposition_needed = false;

            // 진압 위치로 타겟 좌표 업데이트
            double suppress_lat = ctx_.swarm_suppress_lat;
            double suppress_lon = ctx_.swarm_suppress_lon;
            float suppress_yaw = ctx_.swarm_suppress_yaw;

            float local_x, local_y, local_z;
            gpsToLocalNED(suppress_lat, suppress_lon, mission_config_.takeoff_altitude,
                          local_x, local_y, local_z);

            target_ned_x_ = local_x;
            target_ned_y_ = local_y;
            // target_ned_z_ 유지 (현재 고도)
            target_yaw_ = suppress_yaw;

            ctx_.target_ned_x = target_ned_x_;
            ctx_.target_ned_y = target_ned_y_;
            ctx_.target_yaw = target_yaw_;
            ctx_.target_lat = suppress_lat;
            ctx_.target_lon = suppress_lon;

            RCLCPP_INFO(node_->get_logger(),
                "[SWARM] 진압 위치로 재이동: GPS(%.7f,%.7f) NED(%.1f,%.1f) Yaw=%.1f°",
                suppress_lat, suppress_lon, local_x, local_y, suppress_yaw * 180.0f / M_PI);

            current_state_.store(MissionState::NAVIGATE);
            transitionTo(navigate_handler_.get());
            return;
        }
        current_state_.store(MissionState::RTL);
        transitionTo(rtl_handler_.get());
    } else if (current_handler_ == rtl_handler_.get()) {
        current_handler_ = nullptr;
        mission_running_.store(false);  // heartbeat 중단 먼저
        current_state_.store(MissionState::LANDED);
        RCLCPP_INFO(node_->get_logger(), "[RTL→LANDED] 미션 완료! (heartbeat 중단됨, nav=%d, arm=%d)",
                    nav_state_.load(), arming_state_.load());
    }
}

void OffboardManager::syncContextFromMembers()
{
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
    ctx_.dist_bottom.store(dist_bottom_.load());
    ctx_.dist_bottom_valid.store(dist_bottom_valid_.load());
    ctx_.position_received.store(position_received_.load());
    ctx_.nav_state.store(nav_state_.load());
    ctx_.arming_state.store(arming_state_.load());
    ctx_.land_detected.store(land_detected_.load());
    ctx_.maybe_landed.store(maybe_landed_.load());
    ctx_.formation_ready_to_rotate.store(formation_ready_to_rotate_.load());
    ctx_.formation_ready_to_navigate.store(formation_ready_to_navigate_.load());
    ctx_.continuous_update_mode.store(continuous_update_mode_.load());
    ctx_.collision_action.store(collision_action_.load());
}

void OffboardManager::syncMembersFromContext()
{
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
        case MissionState::DISTANCE_ADJUST: return "DISTANCE_ADJUST";
        case MissionState::HOVER_AT_TARGET: return "HOVER_AT_TARGET";
        case MissionState::RTL: return "RTL";
        case MissionState::LANDED: return "LANDED";
        case MissionState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

std::string OffboardManager::missionStateToStatusString(MissionState state)
{
    // MissionState → /offboard/status 토픽 문자열 (OSD DroneStatus와 매핑)
    switch (state) {
        case MissionState::IDLE:            return "IDLE";
        case MissionState::PREPARING:       return "ARMING";    // 준비 = 시동 과정
        case MissionState::OFFBOARD:        return "ARMING";
        case MissionState::ARMING:          return "ARMING";
        case MissionState::TAKEOFF:         return "TAKEOFF";
        case MissionState::HOVER:           return "HOVERING";
        case MissionState::ROTATE:          return "ROTATING";
        case MissionState::NAVIGATE:        return "NAVIGATING";
        case MissionState::DISTANCE_ADJUST: return "DISTANCE_ADJUST";
        case MissionState::HOVER_AT_TARGET: return "TRACKING";  // 열원 추적/진압 대기
        case MissionState::RTL:             return "RETURNING";
        case MissionState::LANDED:          return "DISARMED";
        case MissionState::ERROR:           return "IDLE";
        default:                            return "IDLE";
    }
}

void OffboardManager::publishOffboardStatus(const std::string& status)
{
    // 같은 상태 중복 발행 방지
    if (status == last_published_status_) return;
    last_published_status_ = status;

    auto msg = std_msgs::msg::String();
    msg.data = status;
    offboard_status_pub_->publish(msg);
}
