/**
 * @file formation_controller.cpp
 * @brief 편대 비행 제어기 구현 - ROS2 DDS 기반 리더/팔로워
 */

#include "formation_controller.h"
#include "../offboard_manager.h"
#include <algorithm>
#include <fstream>
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

    // ROS_NAMESPACE 기반 토픽 경로 (노드가 root namespace에 있으므로 수동 prefix)
    const char* ns_env = getenv("ROS_NAMESPACE");
    std::string ns_prefix = ns_env ? ("/" + std::string(ns_env)) : "";

    leader_pose_pub_ = node_->create_publisher<humiro_msgs::msg::LeaderPose>(
        ns_prefix + "/formation/leader_pose", qos_best_effort);
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
        100ms, std::bind(&FormationController::leaderPoseTimerCallback, this));
    leader_status_timer_ = node_->create_wall_timer(
        1000ms, std::bind(&FormationController::leaderStatusTimerCallback, this));  // 편대 동기화 1Hz
    heartbeat_timer_ = node_->create_wall_timer(
        1000ms, std::bind(&FormationController::heartbeatTimerCallback, this));

    // offboard_config.json에서 진압 고도 로드 (리더도 SUPPRESS시 사용)
    {
        std::ifstream cfg("/home/khadas/humiro_fire_suppression/config/offboard_config.json");
        if (cfg.is_open()) {
            std::string line;
            while (std::getline(cfg, line)) {
                auto pos = line.find("\"target_altitude\"");
                if (pos != std::string::npos) {
                    auto colon = line.find(':', pos);
                    if (colon != std::string::npos) {
                        std::string val = line.substr(colon + 1);
                        val.erase(std::remove(val.begin(), val.end(), ','), val.end());
                        try {
                            float alt = std::stof(val);
                            if (alt > 0.0f) suppress_altitude_ = alt;
                        } catch (...) {}
                    }
                }
            }
        }
    }

    std::cout << "[FormationController] 리더 초기화 완료" << std::endl;
    std::cout << "  - 발행: " << ns_prefix << "/formation/leader_pose (10Hz, mission_state 포함)" << std::endl;
    std::cout << "  - 발행: /formation/heartbeat (1Hz)" << std::endl;
    std::cout << "  - 발행: /formation/command (이벤트)" << std::endl;
    std::cout << "  - 구독: /formation/follower_status" << std::endl;
    std::cout << "  - 진압 고도: " << suppress_altitude_ << "m, 이격거리: "
              << suppress_distance_m_ << "m" << std::endl;
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

    leader_pose_sub_ = node_->create_subscription<humiro_msgs::msg::LeaderPose>(
        leader_pose_topic, qos_best_effort,
        std::bind(&FormationController::onLeaderPose, this, std::placeholders::_1));
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

    // offboard_config.json에서 진압 고도 로드
    {
        std::ifstream cfg("/home/khadas/humiro_fire_suppression/config/offboard_config.json");
        if (cfg.is_open()) {
            std::string line;
            while (std::getline(cfg, line)) {
                auto pos = line.find("\"target_altitude\"");
                if (pos != std::string::npos) {
                    auto colon = line.find(':', pos);
                    if (colon != std::string::npos) {
                        std::string val = line.substr(colon + 1);
                        val.erase(std::remove(val.begin(), val.end(), ','), val.end());
                        try {
                            float alt = std::stof(val);
                            if (alt > 0.0f) suppress_altitude_ = alt;
                        } catch (...) {}
                    }
                }
            }
        }
    }

    std::cout << "[FormationController] 팔로워 초기화 완료" << std::endl;
    std::cout << "  - 구독: " << leader_pose_topic << " (10Hz, mission_state 포함)" << std::endl;
    std::cout << "  - 구독: /formation/heartbeat (1Hz)" << std::endl;
    std::cout << "  - 구독: /formation/command (이벤트)" << std::endl;
    std::cout << "  - 발행: /formation/follower_status (2Hz)" << std::endl;
    std::cout << "  - 오프셋: right=" << offset_right_cm_ << "cm, behind="
              << offset_behind_cm_ << "cm, above=" << offset_above_cm_ << "cm" << std::endl;
    std::cout << "  - 진압 고도: " << suppress_altitude_ << "m (offboard_config.json)" << std::endl;
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

    // 미션 상태 (10Hz로 팔로워에게 실시간 전달)
    msg.mission_state = OffboardManager::getStateName(offboard_mgr_->getCurrentState());

    leader_pose_pub_->publish(msg);
}

// ============================================================================
// 리더: 편대 동기화 체크 (1Hz) — LeaderStatus는 LeaderPose에 통합 (10Hz)
// ============================================================================

void FormationController::leaderStatusTimerCallback() {
    if (!running_.load() || !offboard_mgr_) return;

    // 미션 시작 즉시 → CMD_FOLLOW 전송 (리더/팔로워 동시 시동+이륙)
    MissionState current = offboard_mgr_->getCurrentState();
    if (offboard_mgr_->isMissionRunning() && !cmd_follow_sent_) {
        cmd_follow_sent_ = true;

        // 네트워크 팔로워 로그
        {
            std::lock_guard<std::mutex> lock(followers_mutex_);
            std::cout << "[Formation] 네트워크 팔로워: " << followers_.size() << "대" << std::endl;
            for (auto& [id, info] : followers_) {
                std::cout << "  - drone" << (int)id << " state=" << info.mission_state << std::endl;
            }
        }

        // CMD_FOLLOW 3회 전송 (WiFi 유실 대비, 고도/속도 포함)
        for (int i = 0; i < 3; i++) {
            auto cmd_msg = humiro_msgs::msg::FormationCommand();
            cmd_msg.header.stamp = node_->now();
            cmd_msg.target_drone_id = 0;
            cmd_msg.command = humiro_msgs::msg::FormationCommand::CMD_FOLLOW;
            cmd_msg.target_latitude = mission_target_lat_;   // 미션 목적지 (팔로워 미러링 판정용)
            cmd_msg.target_longitude = mission_target_lon_;
            cmd_msg.takeoff_altitude = mission_takeoff_altitude_;
            cmd_msg.flight_speed = mission_flight_speed_;
            command_pub_->publish(cmd_msg);
            if (i < 2) std::this_thread::sleep_for(100ms);
        }
        std::cout << "[Formation] CMD_FOLLOW 전송 완료 (alt=" << mission_takeoff_altitude_
                  << "m, speed=" << mission_flight_speed_ << "m/s)" << std::endl;
    }

    // === 편대 동기화 체크 (1Hz) ===
    if (cmd_follow_sent_ && offboard_mgr_) {
        // 체크 1: 팔로워 이륙 확인 → ROTATE 허가
        if (!formation_ready_to_rotate_notified_) {
            std::lock_guard<std::mutex> lock(followers_mutex_);
            bool all_airborne = !followers_.empty();
            for (auto& [id, info] : followers_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - info.last_update).count();
                // "FOLLOWING" = 이륙 완료(HOVER 이상), "TAKEOFF" = 이륙 중
                if (elapsed > 5 || info.mission_state != "FOLLOWING") {
                    all_airborne = false;
                    std::cout << "[Formation] 대기: drone" << (int)id
                              << " state=" << info.mission_state
                              << " (elapsed=" << elapsed << "s)" << std::endl;
                    break;
                }
            }
            if (all_airborne) {
                offboard_mgr_->setFormationReadyToRotate(true);
                formation_ready_to_rotate_notified_ = true;
                std::cout << "[Formation] 모든 팔로워 이륙 확인 → ROTATE 허가" << std::endl;
            }
        }

        // 체크 2: 팔로워 편대 배치 완료 → NAVIGATE 허가
        if (formation_ready_to_rotate_notified_ && !formation_ready_to_navigate_notified_) {
            std::lock_guard<std::mutex> lock(followers_mutex_);
            bool all_in_position = !followers_.empty();
            for (auto& [id, info] : followers_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - info.last_update).count();
                if (elapsed > 5 || info.mission_state != "FOLLOWING" ||
                    info.offset_error > FORMATION_COMPLETE_THRESHOLD_M) {
                    all_in_position = false;
                    std::cout << "[Formation] 대기: drone" << (int)id
                              << " offset_error=" << info.offset_error
                              << "m (threshold=" << FORMATION_COMPLETE_THRESHOLD_M << "m)" << std::endl;
                    break;
                }
            }
            if (all_in_position) {
                offboard_mgr_->setFormationReadyToNavigate(true);
                formation_ready_to_navigate_notified_ = true;
                std::cout << "[Formation] 모든 팔로워 편대 배치 완료 → NAVIGATE 허가" << std::endl;
            }
        }
    }

    // HOVER_AT_TARGET 도달 시 → SUPPRESS 자동 전환
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

    // 리더 미션 상태 실시간 업데이트 (10Hz, onLeaderStatus 대체)
    if (!msg->mission_state.empty()) {
        leader_phase_ = msg->mission_state;
    }

    // FOLLOWING 상태일 때만 리더 추적 (SUPPRESSING/HOLD/RTL에서는 무시)
    if (follower_phase_ != FollowerPhase::FOLLOWING) return;

    // 미션이 실행 중일 때만 오프셋 추적
    if (!offboard_mgr_->isMissionRunning()) return;

    // 충돌 방지: 리더 안전 반경 체크
    double follower_lat = offboard_mgr_->getCurrentLat();
    double follower_lon = offboard_mgr_->getCurrentLon();
    double cos_lat_check = std::cos(follower_lat * M_PI / 180.0);
    double d_lat = (msg->latitude - follower_lat) * DEG_TO_M_LAT;
    double d_lon = (msg->longitude - follower_lon) * DEG_TO_M_LAT * cos_lat_check;
    float dist_to_leader = std::sqrt(d_lat * d_lat + d_lon * d_lon);

    if (dist_to_leader < SAFETY_RADIUS_M) {
        static std::chrono::steady_clock::time_point last_safety_log{};
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_safety_log).count() >= 2) {
            std::cout << "[Formation] 안전거리 미확보 (" << dist_to_leader
                      << "m < " << SAFETY_RADIUS_M << "m) - 대기" << std::endl;
            last_safety_log = now;
        }
        return;  // 리더에 너무 가까움 - 목표 업데이트 안함 (현재 위치 유지)
    }

    // 오프셋 목표 GPS 계산
    GPSCoordinate target = calculateOffsetTarget(*msg);

    // OffboardManager에 새 목표 전달 (리더 yaw로 헤딩 정렬)
    float leader_yaw_rad = msg->yaw_deg * M_PI / 180.0f;
    offboard_mgr_->updateMissionTarget(target, leader_yaw_rad);
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
            lateral_offset_mirrored_ = false;
            std::cout << "  → HOLD: 현재 위치 유지" << std::endl;
            break;

        case humiro_msgs::msg::FormationCommand::CMD_FOLLOW:
            follower_phase_ = FollowerPhase::FOLLOWING;
            lateral_offset_mirrored_ = false;
            last_offset_valid_ = false;          // 이전 미션 offset_error 리셋 (999→새 계산)
            precomputed_suppress_valid_ = false;  // 이전 미션 진압 위치 리셋
            // 미션 목적지 저장 (위치 기반 미러링 판정용)
            if (msg->target_latitude != 0.0) {
                mission_dest_lat_ = msg->target_latitude;
                mission_dest_lon_ = msg->target_longitude;
                mission_dest_set_ = true;
                // 초기 approach_bearing 설정 (팔로워 현재 위치 → 목적지)
                if (offboard_mgr_) {
                    double cos_lat = std::cos(offboard_mgr_->getCurrentLat() * M_PI / 180.0);
                    double tn = (mission_dest_lat_ - offboard_mgr_->getCurrentLat()) * DEG_TO_M_LAT;
                    double te = (mission_dest_lon_ - offboard_mgr_->getCurrentLon()) * DEG_TO_M_LAT * cos_lat;
                    approach_bearing_deg_ = std::atan2(te, tn) * 180.0f / M_PI;
                }
                std::cout << "  → 목적지 저장: " << mission_dest_lat_ << ", " << mission_dest_lon_
                          << ", approach_bearing=" << approach_bearing_deg_ << "°" << std::endl;
            }
            received_takeoff_altitude_ = msg->takeoff_altitude;
            received_flight_speed_ = msg->flight_speed;
            std::cout << "  → FOLLOW: alt=" << received_takeoff_altitude_
                      << "m, speed=" << received_flight_speed_ << "m/s" << std::endl;
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
            lateral_offset_mirrored_ = false;
            if (offboard_mgr_) {
                offboard_mgr_->emergencyRTL();
                std::cout << "  → RTL: 귀환 명령" << std::endl;
            }
            break;

        case humiro_msgs::msg::FormationCommand::CMD_SUPPRESS: {
            follower_phase_ = FollowerPhase::SUPPRESSING;
            std::cout << "  → SUPPRESS: 진압 편대 전환 (dist=" << suppress_distance_m_
                      << "m, angle=" << suppress_angle_deg_ << "°, alt="
                      << suppress_altitude_ << "m, approach=" << approach_bearing_deg_
                      << "°)" << std::endl;

            if (offboard_mgr_ && msg->target_latitude != 0.0) {
                GPSCoordinate suppress_pos;

                // 발화점 중심 기하학 (사전계산 또는 fallback 동일 로직)
                // 발화점 = 미션목표 + suppress_distance × 접근방향
                // 팔로워 = 발화점 + suppress_distance × direction(접근+180°-각도)
                double ab_rad = approach_bearing_deg_ * M_PI / 180.0;
                double dist = (double)suppress_distance_m_;
                double cos_lat_tgt = std::cos(msg->target_latitude * M_PI / 180.0);

                // 발화점 GPS
                double fire_n = dist * std::cos(ab_rad);
                double fire_e = dist * std::sin(ab_rad);
                double fire_lat = msg->target_latitude + fire_n / DEG_TO_M_LAT;
                double fire_lon = msg->target_longitude + fire_e / (DEG_TO_M_LAT * cos_lat_tgt);

                if (precomputed_suppress_valid_) {
                    suppress_pos.latitude = precomputed_suppress_lat_;
                    suppress_pos.longitude = precomputed_suppress_lon_;
                    std::cout << "  → 진압 위치(사전계산, mirror="
                              << (lateral_offset_mirrored_ ? "ON" : "OFF") << ")" << std::endl;
                } else {
                    // fallback: 발화점 중심 계산 (미러링 적용)
                    double effective_angle = lateral_offset_mirrored_
                        ? -(double)suppress_angle_deg_ : (double)suppress_angle_deg_;
                    double foll_bearing_rad = (approach_bearing_deg_ + 180.0 - effective_angle) * M_PI / 180.0;
                    double foll_n = fire_n + dist * std::cos(foll_bearing_rad);
                    double foll_e = fire_e + dist * std::sin(foll_bearing_rad);
                    suppress_pos.latitude = msg->target_latitude + foll_n / DEG_TO_M_LAT;
                    suppress_pos.longitude = msg->target_longitude + foll_e / (DEG_TO_M_LAT * cos_lat_tgt);
                    std::cout << "  → 진압 위치(fallback, mirror="
                              << (lateral_offset_mirrored_ ? "ON" : "OFF") << ")" << std::endl;
                }
                suppress_pos.altitude = offboard_mgr_->getCurrentAltAmsl();

                // 진압 고도 설정 (offboard_config.json target_altitude)
                if (suppress_altitude_ > 0.0f) {
                    offboard_mgr_->setTargetAltitude(suppress_altitude_);
                }

                // 헤딩: 발화점 방향으로 설정
                double dn = (fire_lat - suppress_pos.latitude) * DEG_TO_M_LAT;
                double de = (fire_lon - suppress_pos.longitude) * DEG_TO_M_LAT
                            * std::cos(suppress_pos.latitude * M_PI / 180.0);
                float yaw_to_fire = std::atan2(de, dn);

                offboard_mgr_->updateMissionTarget(suppress_pos, yaw_to_fire);
                std::cout << "  → 팔로워 진압 위치: " << suppress_pos.latitude << ", "
                          << suppress_pos.longitude << std::endl;
                std::cout << "  → 발화점: " << fire_lat << ", " << fire_lon
                          << ", yaw=" << (yaw_to_fire * 180.0f / M_PI) << "°" << std::endl;
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

    // 오프셋 오차 계산 (목표 편대 위치와의 거리)
    if (last_offset_valid_) {
        double cur_lat = offboard_mgr_->getCurrentLat();
        double cur_lon = offboard_mgr_->getCurrentLon();
        double cos_lat = std::cos(cur_lat * M_PI / 180.0);
        double err_lat = (last_target_lat_ - cur_lat) * DEG_TO_M_LAT;
        double err_lon = (last_target_lon_ - cur_lon) * DEG_TO_M_LAT * cos_lat;
        msg.offset_error = static_cast<float>(std::sqrt(err_lat * err_lat + err_lon * err_lon));
    } else {
        msg.offset_error = 999.0f;  // 아직 목표 없음
    }

    // 미션 상태: 리더가 이해할 수 있는 상세 상태
    if (follower_phase_ == FollowerPhase::FOLLOWING && offboard_mgr_->isMissionRunning()) {
        MissionState ms = offboard_mgr_->getCurrentState();
        if (ms == MissionState::HOVER || ms == MissionState::ROTATE ||
            ms == MissionState::NAVIGATE || ms == MissionState::HOVER_AT_TARGET) {
            msg.mission_state = "FOLLOWING";  // 이륙 완료, 편대 추적 중
        } else {
            msg.mission_state = "TAKEOFF";    // 아직 이륙 중
        }
    } else {
        msg.mission_state = followerPhaseToString(follower_phase_);
    }

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

void FormationController::setMissionParams(float takeoff_alt, float flight_speed) {
    mission_takeoff_altitude_ = takeoff_alt;
    mission_flight_speed_ = flight_speed;
    cmd_follow_sent_ = false;
    formation_ready_to_rotate_notified_ = false;
    formation_ready_to_navigate_notified_ = false;
    std::cout << "[FormationController] 미션 파라미터: alt=" << takeoff_alt
              << "m, speed=" << flight_speed << "m/s" << std::endl;
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

    // 리더 헤딩: 발화점(타겟 이격거리 전방) 방향으로 갱신
    // ★ setTargetYaw만 사용 (updateMissionTarget은 HOVER_AT_TARGET→NAVIGATE 전환 유발)
    if (offboard_mgr_) {
        float cur_yaw = offboard_mgr_->getCurrentYaw();  // 현재 헤딩(≈접근방향) rad
        double cos_lat = std::cos(mission_target_lat_ * M_PI / 180.0);
        double fire_lat = mission_target_lat_
            + (double)suppress_distance_m_ * std::cos((double)cur_yaw) / DEG_TO_M_LAT;
        double fire_lon = mission_target_lon_
            + (double)suppress_distance_m_ * std::sin((double)cur_yaw) / (DEG_TO_M_LAT * cos_lat);

        double dn = (fire_lat - offboard_mgr_->getCurrentLat()) * DEG_TO_M_LAT;
        double de = (fire_lon - offboard_mgr_->getCurrentLon()) * DEG_TO_M_LAT * cos_lat;
        float yaw_to_fire = std::atan2(de, dn);

        offboard_mgr_->setTargetYaw(yaw_to_fire);

        // 리더 진압 고도 설정
        if (suppress_altitude_ > 0.0f) {
            offboard_mgr_->setTargetAltitude(suppress_altitude_);
        }

        std::cout << "  → 리더 발화점: " << fire_lat << ", " << fire_lon
                  << ", yaw=" << (yaw_to_fire * 180.0f / M_PI) << "°" << std::endl;
    }

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

    float yaw_rad = leader_pose.yaw_deg * M_PI / 180.0f;
    double cos_lat = std::cos(leader_pose.latitude * M_PI / 180.0);

    // 1. 바디프레임 → NED 회전 (원본 오프셋)
    float body_x = -(float)offset_behind_cm_ / 100.0f;   // 뒤 = 음의 전방
    float body_y = (float)offset_right_cm_ / 100.0f;     // 우측
    float ned_x = std::cos(yaw_rad) * body_x - std::sin(yaw_rad) * body_y;
    float ned_y = std::sin(yaw_rad) * body_x + std::cos(yaw_rad) * body_y;

    // 2. 위치 기반 충돌 방지: NED 오프셋을 경로선 기준으로 반사
    //    리더→목적지 경로의 수직 성분으로 팔로워/타겟 좌우를 판정하고
    //    반대편이면 NED 오프셋을 경로선 대칭으로 반사 (body_y 반전이 아닌 NED 반사)
    if (mission_dest_set_ && offboard_mgr_) {
        double travel_n = (mission_dest_lat_ - leader_pose.latitude) * DEG_TO_M_LAT;
        double travel_e = (mission_dest_lon_ - leader_pose.longitude) * DEG_TO_M_LAT * cos_lat;
        double travel_dist = std::sqrt(travel_n * travel_n + travel_e * travel_e);

        if (travel_dist > 5.0) {
            double t_hat_n = travel_n / travel_dist;
            double t_hat_e = travel_e / travel_dist;

            // 접근 방향 갱신 (CMD_SUPPRESS에서 사용)
            approach_bearing_deg_ = std::atan2(travel_e, travel_n) * 180.0f / M_PI;

            // 오프셋의 수직 성분 (경로 방향에 수직인 부분)
            double offset_parallel = (double)ned_x * t_hat_n + (double)ned_y * t_hat_e;
            double perp_n = (double)ned_x - offset_parallel * t_hat_n;
            double perp_e = (double)ned_y - offset_parallel * t_hat_e;

            // 팔로워의 수직 성분
            double foll_n = (offboard_mgr_->getCurrentLat() - leader_pose.latitude) * DEG_TO_M_LAT;
            double foll_e = (offboard_mgr_->getCurrentLon() - leader_pose.longitude) * DEG_TO_M_LAT * cos_lat;
            double foll_parallel = foll_n * t_hat_n + foll_e * t_hat_e;
            double foll_perp_n = foll_n - foll_parallel * t_hat_n;
            double foll_perp_e = foll_e - foll_parallel * t_hat_e;

            // 수직 성분 내적: 같은 쪽이면 양수, 반대쪽이면 음수
            double perp_dot = perp_n * foll_perp_n + perp_e * foll_perp_e;

            bool should_mirror = (perp_dot < 0);
            if (should_mirror) {
                // NED 반사: 경로 방향 평행 성분 유지, 수직 성분 반전
                ned_x = (float)(offset_parallel * t_hat_n - perp_n);
                ned_y = (float)(offset_parallel * t_hat_e - perp_e);
            }

            if (should_mirror != lateral_offset_mirrored_) {
                lateral_offset_mirrored_ = should_mirror;
                std::cout << "[Formation] L/R 반사 " << (should_mirror ? "ON" : "OFF")
                          << " (perp_dot=" << perp_dot << ")" << std::endl;
            }

            // 4. 진압 위치 사전 계산 (발화점 중심 배치)
            //    GUI 진압편대도: 타겟(발화점)이 중심, 드론들이 이격거리에 배치
            //    발화점 = 미션목적지 + suppress_distance × 접근방향
            //    팔로워 = 발화점 + suppress_distance × direction(접근+180°-각도)
            //    → 리더-발화점-팔로워 각도 = suppress_angle
            //    미러링 시 suppress_angle 부호 반전 (경로 횡단 방지)
            {
                double ab_rad = approach_bearing_deg_ * M_PI / 180.0;
                double dist = (double)suppress_distance_m_;
                double dest_cos = std::cos(mission_dest_lat_ * M_PI / 180.0);

                // 발화점 NED (미션 목적지 기준)
                double fire_n = dist * std::cos(ab_rad);
                double fire_e = dist * std::sin(ab_rad);

                // 미러링 적용: 팔로워가 반대편에 있으면 suppress_angle 반전
                double effective_angle = lateral_offset_mirrored_
                    ? -(double)suppress_angle_deg_ : (double)suppress_angle_deg_;

                // 팔로워 방향: 접근방향 + 180° - suppress_angle (발화점 기준)
                double foll_bearing_rad = (approach_bearing_deg_ + 180.0 - effective_angle) * M_PI / 180.0;
                double foll_n = fire_n + dist * std::cos(foll_bearing_rad);
                double foll_e = fire_e + dist * std::sin(foll_bearing_rad);

                precomputed_suppress_lat_ = mission_dest_lat_ + foll_n / DEG_TO_M_LAT;
                precomputed_suppress_lon_ = mission_dest_lon_ + foll_e / (DEG_TO_M_LAT * dest_cos);
                precomputed_suppress_valid_ = true;
            }
        }
        // travel_dist <= 5.0: 목적지 근처 → 마지막 값 유지
    }

    // 3. GPS 좌표로 변환
    GPSCoordinate target;
    target.latitude = leader_pose.latitude + (double)ned_x / DEG_TO_M_LAT;
    target.longitude = leader_pose.longitude + (double)ned_y / (DEG_TO_M_LAT * cos_lat);
    target.altitude = leader_pose.altitude + (float)offset_above_cm_ / 100.0f;

    last_target_lat_ = target.latitude;
    last_target_lon_ = target.longitude;
    last_offset_valid_ = true;

    // 디버그: 5초마다
    {
        static std::chrono::steady_clock::time_point last_debug_log{};
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_debug_log).count() >= 5) {
            std::cout << "[Offset] right=" << offset_right_cm_ << "cm, behind=" << offset_behind_cm_
                      << "cm, yaw=" << leader_pose.yaw_deg << "°"
                      << ", ned=(" << ned_x << ", " << ned_y << ")"
                      << (lateral_offset_mirrored_ ? " [반사]" : "")
                      << ", approach=" << approach_bearing_deg_ << "°" << std::endl;
            last_debug_log = now;
        }
    }

    return target;
}
