/**
 * @file formation_controller.cpp
 * @brief 편대 비행 제어기 구현 - ROS2 DDS 기반 리더/팔로워
 */

#include "formation_controller.h"
#include "../offboard_manager.h"
#include <thread>

using namespace std::chrono_literals;

// ============================================================================
// 생성자 / 소멸자
// ============================================================================

FormationController::FormationController(
    rclcpp::Node::SharedPtr node,
    OffboardManager* offboard_mgr,
    uint8_t drone_id,
    FormationRole role)
    : node_(node)
    , offboard_mgr_(offboard_mgr)
    , drone_id_(drone_id)
    , role_(role)
{
    last_heartbeat_time_ = std::chrono::steady_clock::now();
    last_leader_pose_time_ = std::chrono::steady_clock::now();

    std::cout << "[FormationController] 초기화: drone_id=" << (int)drone_id_
              << ", role=" << (role_ == FormationRole::LEADER ? "LEADER" : "FOLLOWER")
              << std::endl;
}

FormationController::~FormationController() {
    stop();
}

// ============================================================================
// 시작 / 중지
// ============================================================================

void FormationController::start() {
    if (running_.load()) return;

    if (role_ == FormationRole::LEADER) {
        initLeader();
    } else {
        initFollower();
    }

    running_.store(true);
    std::cout << "[FormationController] 시작됨" << std::endl;
}

void FormationController::stop() {
    if (!running_.load()) return;
    running_.store(false);

    // 타이머 취소
    if (leader_pose_timer_) leader_pose_timer_->cancel();
    if (leader_status_timer_) leader_status_timer_->cancel();
    if (heartbeat_timer_) heartbeat_timer_->cancel();
    if (follower_status_timer_) follower_status_timer_->cancel();
    if (leader_timeout_timer_) leader_timeout_timer_->cancel();

    std::cout << "[FormationController] 중지됨" << std::endl;
}

// ============================================================================
// 리더 초기화
// ============================================================================

void FormationController::initLeader() {
    // BestEffort QoS for real-time pose data
    rclcpp::QoS qos_best_effort(5);
    qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_best_effort.durability(rclcpp::DurabilityPolicy::Volatile);

    // Reliable QoS for commands and status
    rclcpp::QoS qos_reliable(10);
    qos_reliable.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_reliable.durability(rclcpp::DurabilityPolicy::Volatile);

    // Publishers (네임스페이스 자동 적용됨)
    leader_pose_pub_ = node_->create_publisher<humiro_msgs::msg::LeaderPose>(
        "formation/leader_pose", qos_best_effort);
    leader_status_pub_ = node_->create_publisher<humiro_msgs::msg::LeaderStatus>(
        "formation/leader_status", qos_reliable);
    heartbeat_pub_ = node_->create_publisher<humiro_msgs::msg::FormationHeartbeat>(
        "/formation/heartbeat", qos_best_effort);
    command_pub_ = node_->create_publisher<humiro_msgs::msg::FormationCommand>(
        "/formation/command", qos_reliable);

    // Subscriber: 모든 팔로워의 상태 수신 (글로벌 토픽)
    follower_status_sub_ = node_->create_subscription<humiro_msgs::msg::FollowerStatus>(
        "/formation/follower_status", qos_best_effort,
        std::bind(&FormationController::onFollowerStatus, this, std::placeholders::_1));

    // 타이머
    leader_pose_timer_ = node_->create_wall_timer(
        200ms, std::bind(&FormationController::leaderPoseTimerCallback, this));
    leader_status_timer_ = node_->create_wall_timer(
        1000ms, std::bind(&FormationController::leaderStatusTimerCallback, this));
    heartbeat_timer_ = node_->create_wall_timer(
        1000ms, std::bind(&FormationController::heartbeatTimerCallback, this));

    std::cout << "[FormationController] 리더 초기화 완료" << std::endl;
    std::cout << "  - 발행: formation/leader_pose (5Hz)" << std::endl;
    std::cout << "  - 발행: formation/leader_status (1Hz)" << std::endl;
    std::cout << "  - 발행: /formation/heartbeat (1Hz)" << std::endl;
    std::cout << "  - 발행: /formation/command (이벤트)" << std::endl;
    std::cout << "  - 구독: /formation/follower_status" << std::endl;
}

// ============================================================================
// 팔로워 초기화
// ============================================================================

void FormationController::initFollower() {
    // BestEffort QoS
    rclcpp::QoS qos_best_effort(5);
    qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_best_effort.durability(rclcpp::DurabilityPolicy::Volatile);

    // Reliable QoS
    rclcpp::QoS qos_reliable(10);
    qos_reliable.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_reliable.durability(rclcpp::DurabilityPolicy::Volatile);

    // 리더 네임스페이스 기반 토픽 구독
    std::string leader_ns = leader_namespace_.empty() ? "drone1" : leader_namespace_;
    std::string leader_pose_topic = "/" + leader_ns + "/formation/leader_pose";
    std::string leader_status_topic = "/" + leader_ns + "/formation/leader_status";

    leader_pose_sub_ = node_->create_subscription<humiro_msgs::msg::LeaderPose>(
        leader_pose_topic, qos_best_effort,
        std::bind(&FormationController::onLeaderPose, this, std::placeholders::_1));
    leader_status_sub_ = node_->create_subscription<humiro_msgs::msg::LeaderStatus>(
        leader_status_topic, qos_reliable,
        std::bind(&FormationController::onLeaderStatus, this, std::placeholders::_1));
    heartbeat_sub_ = node_->create_subscription<humiro_msgs::msg::FormationHeartbeat>(
        "/formation/heartbeat", qos_best_effort,
        std::bind(&FormationController::onHeartbeat, this, std::placeholders::_1));
    command_sub_ = node_->create_subscription<humiro_msgs::msg::FormationCommand>(
        "/formation/command", qos_reliable,
        std::bind(&FormationController::onFormationCommand, this, std::placeholders::_1));

    // 팔로워 상태 발행 (글로벌 토픽)
    follower_status_pub_ = node_->create_publisher<humiro_msgs::msg::FollowerStatus>(
        "/formation/follower_status", qos_best_effort);

    // 타이머
    follower_status_timer_ = node_->create_wall_timer(
        500ms, std::bind(&FormationController::followerStatusTimerCallback, this));
    leader_timeout_timer_ = node_->create_wall_timer(
        1000ms, std::bind(&FormationController::leaderTimeoutTimerCallback, this));

    std::cout << "[FormationController] 팔로워 초기화 완료" << std::endl;
    std::cout << "  - 구독: " << leader_pose_topic << " (5Hz)" << std::endl;
    std::cout << "  - 구독: " << leader_status_topic << " (1Hz)" << std::endl;
    std::cout << "  - 구독: /formation/heartbeat (1Hz)" << std::endl;
    std::cout << "  - 구독: /formation/command (이벤트)" << std::endl;
    std::cout << "  - 발행: /formation/follower_status (2Hz)" << std::endl;
    std::cout << "  - 오프셋: right=" << offset_right_cm_ << "cm, behind="
              << offset_behind_cm_ << "cm, above=" << offset_above_cm_ << "cm" << std::endl;
}

// ============================================================================
// 리더 설정
// ============================================================================

void FormationController::setFormationDroneCount(uint8_t count) {
    formation_drone_count_ = count;
}

void FormationController::setFormationPhase(const std::string& phase) {
    formation_phase_ = phase;
}

// ============================================================================
// 팔로워 설정
// ============================================================================

void FormationController::setOffset(int16_t right_cm, int16_t behind_cm, int16_t above_cm) {
    offset_right_cm_ = right_cm;
    offset_behind_cm_ = behind_cm;
    offset_above_cm_ = above_cm;
    std::cout << "[FormationController] 오프셋 설정: right=" << right_cm
              << "cm, behind=" << behind_cm << "cm, above=" << above_cm << "cm" << std::endl;
}

void FormationController::setLeaderNamespace(const std::string& ns) {
    leader_namespace_ = ns;
}

// ============================================================================
// 리더: LeaderPose 발행 (5Hz)
// ============================================================================

void FormationController::leaderPoseTimerCallback() {
    if (!running_.load() || !offboard_mgr_) return;

    auto msg = humiro_msgs::msg::LeaderPose();
    msg.header.stamp = node_->now();

    // OffboardManager의 atomic 변수에서 현재 위치 읽기
    msg.latitude = offboard_mgr_->getCurrentLat();
    msg.longitude = offboard_mgr_->getCurrentLon();
    msg.altitude = offboard_mgr_->getCurrentAltAmsl();
    msg.yaw_deg = offboard_mgr_->getCurrentYaw() * 180.0f / M_PI;
    msg.speed = offboard_mgr_->getCurrentSpeed();

    // NED 좌표
    msg.x = offboard_mgr_->getCurrentLocalX();
    msg.y = offboard_mgr_->getCurrentLocalY();
    msg.z = offboard_mgr_->getCurrentLocalZ();

    // NED 속도
    msg.vx = offboard_mgr_->getCurrentVx();
    msg.vy = offboard_mgr_->getCurrentVy();
    msg.vz = 0.0f;

    leader_pose_pub_->publish(msg);
}

// ============================================================================
// 리더: LeaderStatus 발행 (1Hz)
// ============================================================================

void FormationController::leaderStatusTimerCallback() {
    if (!running_.load() || !offboard_mgr_) return;

    auto msg = humiro_msgs::msg::LeaderStatus();
    msg.header.stamp = node_->now();
    msg.phase = OffboardManager::getStateName(offboard_mgr_->getCurrentState());
    msg.mission_active = offboard_mgr_->isMissionRunning();
    msg.altitude = offboard_mgr_->getCurrentAltAmsl();
    msg.shot_number = 0;
    msg.progress = 0;

    leader_status_pub_->publish(msg);

    // HOVER_AT_TARGET 도달 시 → SUPPRESS 자동 전환
    MissionState current = offboard_mgr_->getCurrentState();
    if (current == MissionState::HOVER_AT_TARGET && formation_phase_ != "SUPPRESS") {
        triggerSuppressPhase();
    }

    // RTL 상태 시 → 팔로워들에게 RTL 명령
    if (current == MissionState::RTL && formation_phase_ != "RTL") {
        formation_phase_ = "RTL";
        sendCommand(0, humiro_msgs::msg::FormationCommand::CMD_RTL);
        std::cout << "[FormationController] 리더 RTL → 팔로워 RTL 명령 전송" << std::endl;
    }

    // SUPPRESS 상태에서 아직 SUPPRESSING 아닌 팔로워에게 재전송 (1Hz)
    if (formation_phase_ == "SUPPRESS") {
        std::lock_guard<std::mutex> lock(followers_mutex_);
        for (auto& [id, info] : followers_) {
            if (info.mission_state != "SUPPRESSING") {
                sendCommand(id, humiro_msgs::msg::FormationCommand::CMD_SUPPRESS,
                            mission_target_lat_, mission_target_lon_);
            }
        }
    }
}

// ============================================================================
// 리더: FormationHeartbeat 발행 (1Hz)
// ============================================================================

void FormationController::heartbeatTimerCallback() {
    if (!running_.load()) return;

    auto msg = humiro_msgs::msg::FormationHeartbeat();
    msg.header.stamp = node_->now();
    msg.leader_id = drone_id_;
    msg.drone_count = formation_drone_count_;
    msg.formation_phase = formation_phase_;

    heartbeat_pub_->publish(msg);
}

// ============================================================================
// 리더: 편대 명령 전송
// ============================================================================

void FormationController::sendCommand(uint8_t target_drone_id, uint8_t command,
                                      double target_lat, double target_lon) {
    if (!running_.load() || role_ != FormationRole::LEADER) return;

    auto msg = humiro_msgs::msg::FormationCommand();
    msg.header.stamp = node_->now();
    msg.target_drone_id = target_drone_id;
    msg.command = command;
    msg.target_latitude = target_lat;
    msg.target_longitude = target_lon;
    msg.offset_right_cm = 0;
    msg.offset_behind_cm = 0;
    msg.offset_above_cm = 0;

    command_pub_->publish(msg);

    std::cout << "[FormationController] 명령 전송: target=" << (int)target_drone_id
              << ", cmd=" << (int)command << std::endl;
}

// ============================================================================
// 리더: 팔로워 상태 수신
// ============================================================================

void FormationController::onFollowerStatus(const humiro_msgs::msg::FollowerStatus::SharedPtr msg) {
    if (!running_.load()) return;

    std::lock_guard<std::mutex> lock(followers_mutex_);
    FollowerInfo info;
    info.drone_id = msg->drone_id;
    info.latitude = msg->latitude;
    info.longitude = msg->longitude;
    info.altitude = msg->altitude;
    info.offset_error = msg->offset_error;
    info.mission_state = msg->mission_state;
    info.battery_percent = msg->battery_percent;
    info.ammo_count = msg->ammo_count;
    info.last_update = std::chrono::steady_clock::now();

    followers_[msg->drone_id] = info;
}

// ============================================================================
// 팔로워: LeaderPose 수신 → 오프셋 위치로 이동
// ============================================================================

void FormationController::onLeaderPose(const humiro_msgs::msg::LeaderPose::SharedPtr msg) {
    if (!running_.load() || !offboard_mgr_) return;

    last_leader_pose_time_ = std::chrono::steady_clock::now();

    // FOLLOWING 상태일 때만 리더 추적 (SUPPRESSING/HOLD/RTL에서는 무시)
    if (follower_phase_ != FollowerPhase::FOLLOWING) return;

    // 미션이 실행 중일 때만 오프셋 추적
    if (!offboard_mgr_->isMissionRunning()) return;

    // 오프셋 목표 GPS 계산
    GPSCoordinate target = calculateOffsetTarget(*msg);

    // OffboardManager에 새 목표 전달
    offboard_mgr_->updateMissionTarget(target);
}

// ============================================================================
// 팔로워: LeaderStatus 수신
// ============================================================================

void FormationController::onLeaderStatus(const humiro_msgs::msg::LeaderStatus::SharedPtr msg) {
    if (!running_.load()) return;
    leader_phase_ = msg->phase;
}

// ============================================================================
// 팔로워: Heartbeat 수신 → 리더 생존 확인
// ============================================================================

void FormationController::onHeartbeat(const humiro_msgs::msg::FormationHeartbeat::SharedPtr msg) {
    if (!running_.load()) return;

    last_heartbeat_time_ = std::chrono::steady_clock::now();
    leader_alive_.store(true);

    // leader_namespace가 설정되지 않은 경우, heartbeat에서 자동 설정
    if (leader_namespace_.empty()) {
        leader_namespace_ = "drone" + std::to_string(msg->leader_id);
    }

    // 팔로워 phase 복구 (CMD 패킷 유실 대응)
    if (msg->formation_phase == "SUPPRESS" && follower_phase_ == FollowerPhase::FOLLOWING) {
        // SUPPRESS 명령을 놓침 → HOLD로 전환 (리더가 1Hz 재전송할 때 SUPPRESS로 전환됨)
        follower_phase_ = FollowerPhase::HOLD;
        std::cout << "[FormationController] Heartbeat로 SUPPRESS phase 감지 → HOLD 전환" << std::endl;
    }
    if (msg->formation_phase == "RTL" && follower_phase_ != FollowerPhase::RTL) {
        // RTL 명령을 놓침 → 즉시 RTL
        follower_phase_ = FollowerPhase::RTL;
        if (offboard_mgr_) {
            offboard_mgr_->emergencyRTL();
        }
        std::cout << "[FormationController] Heartbeat로 RTL phase 감지 → 즉시 RTL" << std::endl;
    }
}

// ============================================================================
// 팔로워: 편대 명령 수신
// ============================================================================

void FormationController::onFormationCommand(const humiro_msgs::msg::FormationCommand::SharedPtr msg) {
    if (!running_.load()) return;

    // 자신에게 보낸 명령이거나 전체 명령인지 확인
    if (msg->target_drone_id != 0 && msg->target_drone_id != drone_id_) return;

    std::cout << "[FormationController] 명령 수신: cmd=" << (int)msg->command
              << ", target_id=" << (int)msg->target_drone_id << std::endl;

    // 오프셋 업데이트 (명령에 포함된 경우)
    if (msg->offset_right_cm != 0 || msg->offset_behind_cm != 0 || msg->offset_above_cm != 0) {
        setOffset(msg->offset_right_cm, msg->offset_behind_cm, msg->offset_above_cm);
    }

    // 명령 처리
    switch (msg->command) {
        case humiro_msgs::msg::FormationCommand::CMD_HOLD:
            follower_phase_ = FollowerPhase::HOLD;
            std::cout << "  → HOLD: 현재 위치 유지" << std::endl;
            break;

        case humiro_msgs::msg::FormationCommand::CMD_FOLLOW:
            follower_phase_ = FollowerPhase::FOLLOWING;
            std::cout << "  → FOLLOW: 리더 추적 시작" << std::endl;
            // 외부 콜백으로 ApplicationManager에 미션 시작 요청
            if (command_callback_) {
                command_callback_(msg->command, msg->target_latitude, msg->target_longitude);
            }
            return;  // 콜백 중복 호출 방지

        case humiro_msgs::msg::FormationCommand::CMD_GOTO:
            if (offboard_mgr_ && msg->target_latitude != 0.0) {
                GPSCoordinate target;
                target.latitude = msg->target_latitude;
                target.longitude = msg->target_longitude;
                offboard_mgr_->updateMissionTarget(target);
                std::cout << "  → GOTO: " << msg->target_latitude << ", "
                          << msg->target_longitude << std::endl;
            }
            break;

        case humiro_msgs::msg::FormationCommand::CMD_RTL:
            follower_phase_ = FollowerPhase::RTL;
            if (offboard_mgr_) {
                offboard_mgr_->emergencyRTL();
                std::cout << "  → RTL: 귀환 명령" << std::endl;
            }
            break;

        case humiro_msgs::msg::FormationCommand::CMD_SUPPRESS: {
            follower_phase_ = FollowerPhase::SUPPRESSING;
            std::cout << "  → SUPPRESS: 진압 편대 전환" << std::endl;

            // 타겟 GPS + 자신의 SUPPRESS_DISTANCE/ANGLE로 진압 위치 계산
            if (offboard_mgr_ && msg->target_latitude != 0.0) {
                double angle_rad = suppress_angle_deg_ * M_PI / 180.0;
                double dist = (double)suppress_distance_m_;
                double target_lat_rad = msg->target_latitude * M_PI / 180.0;
                double cos_lat = std::cos(target_lat_rad);

                GPSCoordinate suppress_pos;
                suppress_pos.latitude = msg->target_latitude
                    + (dist * std::cos(angle_rad)) / DEG_TO_M_LAT;
                suppress_pos.longitude = msg->target_longitude
                    + (dist * std::sin(angle_rad)) / (DEG_TO_M_LAT * cos_lat);
                suppress_pos.altitude = offboard_mgr_->getCurrentAltAmsl();

                offboard_mgr_->updateMissionTarget(suppress_pos);
                std::cout << "  → 진압 위치: " << suppress_pos.latitude << ", "
                          << suppress_pos.longitude << " (dist=" << dist
                          << "m, angle=" << suppress_angle_deg_ << "°)" << std::endl;
            }
            break;
        }

        default:
            std::cout << "  → 알 수 없는 명령: " << (int)msg->command << std::endl;
            break;
    }

    // 외부 콜백 (CMD_FOLLOW는 위에서 이미 호출)
    if (command_callback_) {
        command_callback_(msg->command, msg->target_latitude, msg->target_longitude);
    }
}

// ============================================================================
// 팔로워: FollowerStatus 발행 (2Hz)
// ============================================================================

void FormationController::followerStatusTimerCallback() {
    if (!running_.load() || !offboard_mgr_) return;

    auto msg = humiro_msgs::msg::FollowerStatus();
    msg.header.stamp = node_->now();
    msg.drone_id = drone_id_;
    msg.latitude = offboard_mgr_->getCurrentLat();
    msg.longitude = offboard_mgr_->getCurrentLon();
    msg.altitude = offboard_mgr_->getCurrentAltAmsl();
    msg.offset_error = 0.0f;  // TODO: 실제 오프셋 오차 계산
    msg.mission_state = followerPhaseToString(follower_phase_);
    msg.battery_percent = 0;  // TODO: StatusROS2Subscriber에서 가져오기
    msg.ammo_count = 6;       // TODO: 실제 소화탄 수

    follower_status_pub_->publish(msg);
}

// ============================================================================
// 팔로워: 리더 하트비트 타임아웃 체크 (1Hz)
// ============================================================================

void FormationController::leaderTimeoutTimerCallback() {
    if (!running_.load()) return;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_heartbeat_time_).count();

    if (elapsed > LEADER_TIMEOUT_SEC && leader_alive_.load()) {
        leader_alive_.store(false);
        follower_phase_ = FollowerPhase::HOLD;
        std::cout << "[FormationController] ⚠ 리더 하트비트 타임아웃! ("
                  << elapsed << "초) → HOLD" << std::endl;
    }
}

// ============================================================================
// 리더: 미션 타겟 설정
// ============================================================================

void FormationController::setMissionTarget(double lat, double lon) {
    mission_target_lat_ = lat;
    mission_target_lon_ = lon;
    std::cout << "[FormationController] 미션 타겟 설정: " << lat << ", " << lon << std::endl;
}

// ============================================================================
// 팔로워: 진압 파라미터 설정
// ============================================================================

void FormationController::setSuppressParams(int16_t distance_m, int16_t angle_deg) {
    suppress_distance_m_ = distance_m;
    suppress_angle_deg_ = angle_deg;
    std::cout << "[FormationController] 진압 파라미터: distance=" << distance_m
              << "m, angle=" << angle_deg << "°" << std::endl;
}

// ============================================================================
// 리더: SUPPRESS phase 전환 (CMD_SUPPRESS 3회 전송)
// ============================================================================

void FormationController::triggerSuppressPhase() {
    formation_phase_ = "SUPPRESS";
    std::cout << "[FormationController] SUPPRESS phase 전환! target=("
              << mission_target_lat_ << ", " << mission_target_lon_ << ")" << std::endl;

    // WiFi 유실 대비 3회 전송
    for (int i = 0; i < 3; i++) {
        sendCommand(0, humiro_msgs::msg::FormationCommand::CMD_SUPPRESS,
                    mission_target_lat_, mission_target_lon_);
        std::this_thread::sleep_for(100ms);
    }
}

// ============================================================================
// FollowerPhase → 문자열 변환
// ============================================================================

std::string FormationController::followerPhaseToString(FollowerPhase phase) {
    switch (phase) {
        case FollowerPhase::IDLE:        return "IDLE";
        case FollowerPhase::FOLLOWING:   return "FOLLOWING";
        case FollowerPhase::SUPPRESSING: return "SUPPRESSING";
        case FollowerPhase::HOLD:        return "HOLD";
        case FollowerPhase::RTL:         return "RTL";
        default:                         return "UNKNOWN";
    }
}

// ============================================================================
// 오프셋 목표 GPS 계산 (팔로워 핵심 알고리즘)
// ============================================================================

GPSCoordinate FormationController::calculateOffsetTarget(
    const humiro_msgs::msg::LeaderPose& leader_pose) {

    // 1. 리더 바디프레임 오프셋 (cm → m)
    float body_x = -(float)offset_behind_cm_ / 100.0f;   // 뒤 = 음의 전방
    float body_y = (float)offset_right_cm_ / 100.0f;     // 우측

    // 2. NED 프레임으로 회전 (리더 yaw 기준)
    float yaw_rad = leader_pose.yaw_deg * M_PI / 180.0f;
    float ned_x = std::cos(yaw_rad) * body_x - std::sin(yaw_rad) * body_y;  // North
    float ned_y = std::sin(yaw_rad) * body_x + std::cos(yaw_rad) * body_y;  // East

    // 3. GPS 좌표로 변환
    double cos_lat = std::cos(leader_pose.latitude * M_PI / 180.0);

    GPSCoordinate target;
    target.latitude = leader_pose.latitude + (double)ned_x / DEG_TO_M_LAT;
    target.longitude = leader_pose.longitude + (double)ned_y / (DEG_TO_M_LAT * cos_lat);
    target.altitude = leader_pose.altitude + (float)offset_above_cm_ / 100.0f;

    return target;
}
