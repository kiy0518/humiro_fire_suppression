#include "outdoor_mission_manager.h"
#include "../gps_utils.h"
#include <cmath>
#include <chrono>

namespace outdoor_mission {

// gps_utils 상수 사용
using gps_utils::EARTH_RADIUS;
using gps_utils::DEG_TO_RAD;
using gps_utils::RAD_TO_DEG;

// ============================================================================
// 생성자/소멸자
// ============================================================================

OutdoorMissionManager::OutdoorMissionManager(rclcpp::Node::SharedPtr node, uint8_t vehicle_id)
    : node_(node), vehicle_id_(vehicle_id), is_leader_(vehicle_id == 1)
{
    RCLCPP_INFO(node_->get_logger(),
        "[OutdoorMission] 초기화 - Vehicle ID: %d (%s)",
        vehicle_id_, is_leader_ ? "리더" : "팔로워");

    initPublishers();
    initSubscribers();

    if (is_leader_) {
        initLeaderTopics();
    } else {
        initFollowerTopics();
    }
}

OutdoorMissionManager::~OutdoorMissionManager() {
    stopMission();
    stopPosePublishing();
}

// ============================================================================
// 초기화
// ============================================================================

void OutdoorMissionManager::initPublishers() {
    // PX4 명령 발행자
    std::string prefix = "/px4_" + std::to_string(vehicle_id_);

    trajectory_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        prefix + "/fmu/in/trajectory_setpoint", 10);

    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        prefix + "/fmu/in/offboard_control_mode", 10);

    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        prefix + "/fmu/in/vehicle_command", 10);
}

void OutdoorMissionManager::initSubscribers() {
    std::string prefix = "/px4_" + std::to_string(vehicle_id_);

    // PX4 uXRCE-DDS QoS 설정 (BestEffort + TransientLocal)
    // depth를 100으로 설정하여 페이로드 크기 문제 해결
    rclcpp::QoS px4_qos(100);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // 로컬 위치 구독
    local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        prefix + "/fmu/out/vehicle_local_position", px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            current_ned_.x = msg->x;
            current_ned_.y = msg->y;
            current_ned_.z = msg->z;
            current_yaw_ = msg->heading * RAD_TO_DEG;
        });

    // 글로벌 위치 구독
    global_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        prefix + "/fmu/out/vehicle_global_position", px4_qos,
        [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
            current_gps_.latitude = msg->lat;
            current_gps_.longitude = msg->lon;
            current_gps_.altitude = msg->alt;
        });
}

void OutdoorMissionManager::initLeaderTopics() {
    // 리더 상태 발행자 (팔로워들이 구독)
    leader_status_pub_ = node_->create_publisher<humiro_msgs::msg::LeaderStatus>(
        "/leader/status", 10);

    leader_pose_pub_ = node_->create_publisher<humiro_msgs::msg::LeaderPose>(
        "/leader/pose", 10);

    leader_aim_pose_pub_ = node_->create_publisher<humiro_msgs::msg::LeaderAimPose>(
        "/leader/aim_pose", 10);

    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 리더 토픽 발행자 초기화 완료");
}

void OutdoorMissionManager::initFollowerTopics() {
    // 리더 토픽 구독자
    leader_status_sub_ = node_->create_subscription<humiro_msgs::msg::LeaderStatus>(
        "/leader/status", 10,
        std::bind(&OutdoorMissionManager::onLeaderStatusCallback, this, std::placeholders::_1));

    leader_pose_sub_ = node_->create_subscription<humiro_msgs::msg::LeaderPose>(
        "/leader/pose", 10,
        std::bind(&OutdoorMissionManager::onLeaderPoseCallback, this, std::placeholders::_1));

    leader_aim_pose_sub_ = node_->create_subscription<humiro_msgs::msg::LeaderAimPose>(
        "/leader/aim_pose", 10,
        std::bind(&OutdoorMissionManager::onLeaderAimPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 팔로워 토픽 구독자 초기화 완료");
}

// ============================================================================
// 메인 미션 실행
// ============================================================================

bool OutdoorMissionManager::executeMission(const OutdoorMissionConfig& config) {
    if (mission_active_) {
        RCLCPP_WARN(node_->get_logger(), "[OutdoorMission] 이미 미션이 실행 중입니다");
        return false;
    }

    config_ = config;
    mission_active_ = true;
    stop_requested_ = false;
    current_phase_ = MissionPhase::IDLE;

    // 현재 위치를 홈으로 저장
    home_position_ = current_gps_;

    RCLCPP_INFO(node_->get_logger(),
        "\n======================================\n"
        "  야외 스웜 미션 시작\n"
        "======================================\n"
        "  기체: %d (%s)\n"
        "  웨이포인트: (%.6f, %.6f, %.1fm)\n"
        "  홈: (%.6f, %.6f)\n"
        "======================================",
        vehicle_id_, is_leader_ ? "리더" : "팔로워",
        config_.waypoint.latitude, config_.waypoint.longitude, config_.waypoint.altitude,
        home_position_.latitude, home_position_.longitude);

    bool success = false;

    if (is_leader_) {
        success = runLeaderSequence();
    } else {
        success = runFollowerSequence();
    }

    mission_active_ = false;
    current_phase_ = MissionPhase::IDLE;

    RCLCPP_INFO(node_->get_logger(),
        "[OutdoorMission] 미션 %s", success ? "성공" : "실패");

    return success;
}

void OutdoorMissionManager::stopMission() {
    stop_requested_ = true;
    RCLCPP_WARN(node_->get_logger(), "[OutdoorMission] 미션 중지 요청");
}

// ============================================================================
// 리더 시퀀스
// ============================================================================

bool OutdoorMissionManager::runLeaderSequence() {
    RCLCPP_INFO(node_->get_logger(), "[리더] 시퀀스 시작");

    // Phase 1: 시동
    if (!leaderArm()) return false;

    // Phase 2: 이륙
    if (!leaderTakeoff()) return false;

    // Phase 3: 웨이포인트 이동
    if (!leaderFlyToWaypoint()) return false;

    // Phase 4: 거리 조정 (타임아웃 적용)
    if (!leaderAdjustDistance()) {
        RCLCPP_WARN(node_->get_logger(), "[리더] 거리 조정 실패/타임아웃 - 복귀 진행");
    }

    // Phase 5-6: 조준/격발 (야외 테스트에서는 스킵)
    // TODO: 실제 운용 시 활성화
    // if (!leaderAim()) return false;
    // if (!leaderFire()) return false;
    RCLCPP_INFO(node_->get_logger(), "[리더] 조준/격발 단계 스킵 (테스트 모드)");

    // 호버링 3초 대기
    RCLCPP_INFO(node_->get_logger(), "[리더] 3초 호버링 후 복귀...");
    for (int i = 0; i < 30 && !stop_requested_; i++) {
        holdPosition();
        sleepMs(100);
    }

    // Phase 7: 복귀
    if (!leaderReturn()) return false;

    // Phase 8: 착륙
    if (!leaderLand()) return false;

    // Phase 9: 시동 해제
    if (!leaderDisarm()) return false;

    return true;
}

bool OutdoorMissionManager::leaderArm() {
    current_phase_ = MissionPhase::ARMING;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 1: 시동");

    if (!arm()) {
        RCLCPP_ERROR(node_->get_logger(), "[리더] 시동 실패");
        current_phase_ = MissionPhase::ERROR;
        return false;
    }

    return true;
}

bool OutdoorMissionManager::leaderTakeoff() {
    current_phase_ = MissionPhase::TAKEOFF;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 2: 이륙 (%.1fm)", config_.takeoff_altitude);

    if (!takeoff(config_.takeoff_altitude)) {
        RCLCPP_ERROR(node_->get_logger(), "[리더] 이륙 실패");
        current_phase_ = MissionPhase::ERROR;
        return false;
    }

    return true;
}

bool OutdoorMissionManager::leaderFlyToWaypoint() {
    current_phase_ = MissionPhase::WAYPOINT;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 3: 웨이포인트 이동");

    // 실시간 위치 발행 시작
    startPosePublishing();

    // 웨이포인트로 이동
    if (!flyToPosition(config_.waypoint)) {
        RCLCPP_ERROR(node_->get_logger(), "[리더] 웨이포인트 이동 실패");
        stopPosePublishing();
        current_phase_ = MissionPhase::ERROR;
        return false;
    }

    // 도착
    current_phase_ = MissionPhase::ARRIVED;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] 웨이포인트 도착");

    return true;
}

bool OutdoorMissionManager::leaderAdjustDistance() {
    current_phase_ = MissionPhase::DISTANCE_ADJ;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 4: 거리 조정 (목표: %.1fm)", config_.target_distance);

    float stable_start_time = 0.0f;
    bool was_stable = false;

    // 타임아웃 설정 (30초)
    const float DISTANCE_ADJ_TIMEOUT = 30.0f;
    float start_time = node_->now().seconds();
    int lidar_fail_count = 0;
    const int MAX_LIDAR_FAILS = 50;  // 5초간 LiDAR 실패 시 스킵

    while (!stop_requested_) {
        // 타임아웃 체크
        float elapsed = node_->now().seconds() - start_time;
        if (elapsed >= DISTANCE_ADJ_TIMEOUT) {
            RCLCPP_WARN(node_->get_logger(), "[리더] 거리 조정 타임아웃 (%.1f초)", elapsed);
            break;
        }

        float distance = getLidarDistance();

        if (distance < 0) {
            lidar_fail_count++;
            if (lidar_fail_count >= MAX_LIDAR_FAILS) {
                RCLCPP_WARN(node_->get_logger(), "[리더] LiDAR 연속 실패 (%d회) - 거리 조정 스킵", lidar_fail_count);
                break;
            }
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[리더] LiDAR 거리 측정 실패 (%d/%d)", lidar_fail_count, MAX_LIDAR_FAILS);
            holdPosition();  // LiDAR 실패 시 제자리 유지
            sleepMs(100);
            continue;
        }

        lidar_fail_count = 0;  // 성공 시 카운터 리셋

        float error = distance - config_.target_distance;

        if (std::abs(error) <= config_.distance_tolerance) {
            // 목표 거리 도달
            if (!was_stable) {
                stable_start_time = node_->now().seconds();
                was_stable = true;
            }

            float stable_duration = node_->now().seconds() - stable_start_time;
            if (stable_duration >= config_.distance_stable_time) {
                RCLCPP_INFO(node_->get_logger(),
                    "[리더] 거리 안정화 완료 (%.2fm, %.1f초)",
                    distance, stable_duration);
                break;
            }

            holdPosition();
        } else {
            was_stable = false;

            if (error > 0) {
                // 너무 멀면 전진
                float vx = std::min(config_.forward_speed, error * 0.5f);
                publishTrajectorySetpoint(
                    current_ned_.x + vx * std::cos(current_yaw_ * DEG_TO_RAD),
                    current_ned_.y + vx * std::sin(current_yaw_ * DEG_TO_RAD),
                    current_ned_.z,
                    current_yaw_ * DEG_TO_RAD);
            } else {
                // 너무 가까우면 후진 (천천히!)
                float vx = std::min(config_.backward_speed, std::abs(error) * 0.3f);
                publishTrajectorySetpoint(
                    current_ned_.x - vx * std::cos(current_yaw_ * DEG_TO_RAD),
                    current_ned_.y - vx * std::sin(current_yaw_ * DEG_TO_RAD),
                    current_ned_.z,
                    current_yaw_ * DEG_TO_RAD);
            }
        }

        sleepMs(100);
    }

    // 목표물 위치 계산 및 발행
    current_phase_ = MissionPhase::FIRE_FIGHTING;
    float final_distance = getLidarDistance();
    if (final_distance > 0) {
        target_position_ = calculateOffsetPosition(
            current_gps_, final_distance, current_yaw_);
        RCLCPP_INFO(node_->get_logger(),
            "[리더] 목표물 위치 계산: (%.6f, %.6f), 거리: %.2fm",
            target_position_.latitude, target_position_.longitude, final_distance);
    } else {
        RCLCPP_WARN(node_->get_logger(), "[리더] LiDAR 없음 - 목표물 위치 미계산");
    }
    publishLeaderStatus();
    publishLeaderAimPose();

    return !stop_requested_;
}

bool OutdoorMissionManager::leaderAim() {
    current_phase_ = MissionPhase::AIMING;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 5: 조준");

    float stable_start_time = 0.0f;
    bool was_stable = false;

    while (!stop_requested_) {
        auto [center_x, center_y] = getThermalCenter();

        if (center_x < 0 || center_y < 0) {
            RCLCPP_WARN(node_->get_logger(), "[리더] 열화상 중심점 감지 실패");
            sleepMs(100);
            continue;
        }

        // 이미지 중심 (0.5, 0.5)과의 오차 계산
        float error_x = center_x - 0.5f;
        float error_y = center_y - 0.5f;

        if (std::abs(error_x) <= config_.aim_tolerance &&
            std::abs(error_y) <= config_.aim_tolerance) {
            // 정조준
            if (!was_stable) {
                stable_start_time = node_->now().seconds();
                was_stable = true;
            }

            float stable_duration = node_->now().seconds() - stable_start_time;
            if (stable_duration >= config_.aim_stable_time) {
                RCLCPP_INFO(node_->get_logger(),
                    "[리더] 조준 완료 (%.1f초)", stable_duration);
                break;
            }

            holdPosition();
        } else {
            was_stable = false;

            // Yaw 조정 (좌우)
            float yaw_adj = -error_x * 10.0f;  // 조정 게인

            // 고도 조정 (상하)
            float alt_adj = -error_y * 0.5f;  // 조정 게인

            float new_yaw = current_yaw_ + yaw_adj;
            float new_z = current_ned_.z + alt_adj;

            publishTrajectorySetpoint(
                current_ned_.x, current_ned_.y, new_z, new_yaw * DEG_TO_RAD);
        }

        sleepMs(100);
    }

    // 조준 완료 발행
    publishLeaderAimPose();

    return !stop_requested_;
}

bool OutdoorMissionManager::leaderFire() {
    current_phase_ = MissionPhase::FIRING;

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 6: 격발 (%d발)", config_.num_shots);

    for (uint8_t shot = 1; shot <= config_.num_shots && !stop_requested_; shot++) {
        publishLeaderStatus(shot);

        RCLCPP_INFO(node_->get_logger(), "[리더] 격발 %d/%d", shot, config_.num_shots);

        // 조준 확인
        holdPosition();
        sleepMs(1000);

        // 격발 신호
        sendFireSignal(shot);

        // 대기
        sleepMs(static_cast<int>(config_.shot_interval * 1000));
    }

    return !stop_requested_;
}

bool OutdoorMissionManager::leaderReturn() {
    current_phase_ = MissionPhase::RETURNING;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 7: 복귀");

    // 위치 발행 중지
    stopPosePublishing();

    // RTL 고도로 상승
    GpsPosition rtl_pos = home_position_;
    rtl_pos.altitude = config_.rtl_altitude;

    // 홈으로 이동
    if (!flyToPosition(rtl_pos)) {
        RCLCPP_ERROR(node_->get_logger(), "[리더] 복귀 실패");
        current_phase_ = MissionPhase::ERROR;
        return false;
    }

    return true;
}

bool OutdoorMissionManager::leaderLand() {
    current_phase_ = MissionPhase::LANDING;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 8: 착륙");

    if (!land()) {
        RCLCPP_ERROR(node_->get_logger(), "[리더] 착륙 실패");
        current_phase_ = MissionPhase::ERROR;
        return false;
    }

    return true;
}

bool OutdoorMissionManager::leaderDisarm() {
    current_phase_ = MissionPhase::DISARMED;
    publishLeaderStatus();

    RCLCPP_INFO(node_->get_logger(), "[리더] Phase 9: 시동 해제");

    if (!disarm()) {
        RCLCPP_ERROR(node_->get_logger(), "[리더] 시동 해제 실패");
        return false;
    }

    return true;
}

// ============================================================================
// 팔로워 시퀀스
// ============================================================================

bool OutdoorMissionManager::runFollowerSequence() {
    RCLCPP_INFO(node_->get_logger(), "[팔로워 %d] 리더 토픽 대기 중...", vehicle_id_);

    // 팔로워는 리더 상태에 따라 반응적으로 동작
    // 메인 루프에서 리더 상태 변화를 모니터링

    while (!stop_requested_ && mission_active_) {
        std::lock_guard<std::mutex> lock(leader_data_mutex_);

        // 리더 상태에 따른 동작
        std::string leader_phase = leader_status_.phase;

        if (leader_phase == "ARMING" && current_phase_ == MissionPhase::IDLE) {
            // 리더 시동 → 팔로워 시동
            RCLCPP_INFO(node_->get_logger(), "[팔로워 %d] 리더 시동 확인 → 시동", vehicle_id_);
            current_phase_ = MissionPhase::ARMING;
            arm();
        }
        else if (leader_phase == "TAKEOFF" && current_phase_ == MissionPhase::ARMING) {
            // 리더 이륙 → 팔로워 이륙
            RCLCPP_INFO(node_->get_logger(), "[팔로워 %d] 리더 이륙 확인 → 이륙", vehicle_id_);
            current_phase_ = MissionPhase::TAKEOFF;
            takeoff(config_.takeoff_altitude);
        }
        else if (leader_phase == "WAYPOINT" && current_phase_ == MissionPhase::TAKEOFF) {
            // 리더 이동 → 편대 비행
            current_phase_ = MissionPhase::WAYPOINT;
        }
        else if (current_phase_ == MissionPhase::WAYPOINT) {
            // 편대 유지하며 이동
            GpsPosition formation_pos = calculateFormationPosition();
            flyToPosition(formation_pos);
        }
        else if ((leader_phase == "ARRIVED" || leader_phase == "DISTANCE_ADJ" ||
                  leader_phase == "FIRE_FIGHTING") &&
                 current_phase_ == MissionPhase::WAYPOINT) {
            // 발사 지점으로 이동
            current_phase_ = MissionPhase::ARRIVED;
            GpsPosition fire_pos = calculateFirePosition();
            flyToPosition(fire_pos);

            // 목표물 방향으로 헤딩
            float target_yaw = calculateBearing(fire_pos, target_position_);
            setYaw(target_yaw);
        }
        else if (leader_phase == "FIRE_FIGHTING" && current_phase_ == MissionPhase::ARRIVED) {
            // 거리 조정 및 조준
            current_phase_ = MissionPhase::DISTANCE_ADJ;
            // LiDAR 거리 조정 (옵션)
            // 열화상 조준
            current_phase_ = MissionPhase::AIMING;
            current_phase_ = MissionPhase::FIRING;

            // 격발
            for (uint8_t shot = 1; shot <= config_.num_shots && !stop_requested_; shot++) {
                sleepMs(1000);
                sendFireSignal(shot);
                sleepMs(static_cast<int>(config_.shot_interval * 1000));
            }
        }
        else if (leader_phase == "RETURNING" && current_phase_ == MissionPhase::FIRING) {
            // 복귀
            current_phase_ = MissionPhase::RETURNING;
            GpsPosition rtl_pos = home_position_;
            rtl_pos.altitude = config_.rtl_altitude;
            flyToPosition(rtl_pos);
        }
        else if (leader_phase == "LANDING" && current_phase_ == MissionPhase::RETURNING) {
            // 착륙
            current_phase_ = MissionPhase::LANDING;
            land();
        }
        else if (leader_phase == "DISARMED" && current_phase_ == MissionPhase::LANDING) {
            // 시동 해제
            current_phase_ = MissionPhase::DISARMED;
            disarm();
            break;
        }

        sleepMs(100);
    }

    return !stop_requested_;
}

void OutdoorMissionManager::onLeaderStatusCallback(
    const humiro_msgs::msg::LeaderStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(leader_data_mutex_);
    leader_status_ = *msg;

    RCLCPP_DEBUG(node_->get_logger(),
        "[팔로워 %d] 리더 상태: %s", vehicle_id_, msg->phase.c_str());
}

void OutdoorMissionManager::onLeaderPoseCallback(
    const humiro_msgs::msg::LeaderPose::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(leader_data_mutex_);
    leader_pose_ = *msg;
}

void OutdoorMissionManager::onLeaderAimPoseCallback(
    const humiro_msgs::msg::LeaderAimPose::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(leader_data_mutex_);
    leader_aim_pose_ = *msg;

    // 목표물 위치 저장
    target_position_.latitude = msg->target_latitude;
    target_position_.longitude = msg->target_longitude;
}

GpsPosition OutdoorMissionManager::calculateFormationPosition() {
    // 이동 중 편대 위치 계산 (리더 기준 후방 45도)
    std::lock_guard<std::mutex> lock(leader_data_mutex_);

    float angle_offset = (vehicle_id_ == 2)
        ? config_.formation_angle_2
        : config_.formation_angle_3;

    float bearing = leader_pose_.yaw_deg + angle_offset;

    GpsPosition leader_pos;
    leader_pos.latitude = leader_pose_.latitude;
    leader_pos.longitude = leader_pose_.longitude;
    leader_pos.altitude = leader_pose_.altitude;

    return calculateOffsetPosition(leader_pos, config_.formation_distance, bearing);
}

GpsPosition OutdoorMissionManager::calculateFirePosition() {
    // 발사 지점 계산 (목표물 기준 정삼각형)
    std::lock_guard<std::mutex> lock(leader_data_mutex_);

    float angle_offset = (vehicle_id_ == 2)
        ? config_.fire_angle_2
        : config_.fire_angle_3;

    // 리더 헤딩 기준으로 각도 계산
    float bearing = leader_aim_pose_.yaw_deg + angle_offset + 180.0f;

    return calculateOffsetPosition(target_position_, config_.fire_distance, bearing);
}

// ============================================================================
// 리더 토픽 발행
// ============================================================================

void OutdoorMissionManager::publishLeaderStatus(uint8_t shot) {
    if (!is_leader_ || !leader_status_pub_) return;

    auto msg = humiro_msgs::msg::LeaderStatus();
    msg.header.stamp = node_->now();
    msg.phase = phaseToString(current_phase_);
    msg.shot_number = shot;
    msg.mission_active = mission_active_;
    msg.altitude = -current_ned_.z;  // NED → 고도
    msg.progress = 0;  // TODO: 계산

    leader_status_pub_->publish(msg);
}

void OutdoorMissionManager::publishLeaderPose() {
    if (!is_leader_ || !leader_pose_pub_) return;

    auto msg = humiro_msgs::msg::LeaderPose();
    msg.header.stamp = node_->now();
    msg.latitude = current_gps_.latitude;
    msg.longitude = current_gps_.longitude;
    msg.altitude = -current_ned_.z;
    msg.yaw_deg = current_yaw_;
    msg.speed = 0.0f;  // TODO: 계산
    msg.x = current_ned_.x;
    msg.y = current_ned_.y;
    msg.z = current_ned_.z;

    leader_pose_pub_->publish(msg);
}

void OutdoorMissionManager::publishLeaderAimPose() {
    if (!is_leader_ || !leader_aim_pose_pub_) return;

    auto msg = humiro_msgs::msg::LeaderAimPose();
    msg.header.stamp = node_->now();
    msg.latitude = current_gps_.latitude;
    msg.longitude = current_gps_.longitude;
    msg.altitude = -current_ned_.z;
    msg.yaw_deg = current_yaw_;
    msg.distance_to_target = getLidarDistance();
    msg.target_latitude = target_position_.latitude;
    msg.target_longitude = target_position_.longitude;
    msg.aim_locked = (current_phase_ == MissionPhase::FIRING);
    msg.x = current_ned_.x;
    msg.y = current_ned_.y;
    msg.z = current_ned_.z;

    leader_aim_pose_pub_->publish(msg);
}

void OutdoorMissionManager::startPosePublishing() {
    if (!is_leader_) return;

    pose_publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz
        [this]() { publishLeaderPose(); });

    RCLCPP_INFO(node_->get_logger(), "[리더] 위치 발행 시작 (10Hz)");
}

void OutdoorMissionManager::stopPosePublishing() {
    if (pose_publish_timer_) {
        pose_publish_timer_->cancel();
        pose_publish_timer_.reset();
        RCLCPP_INFO(node_->get_logger(), "[리더] 위치 발행 중지");
    }
}

// ============================================================================
// 공통 비행 함수
// ============================================================================

bool OutdoorMissionManager::arm() {
    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 시동 명령");

    // OFFBOARD 모드 활성화를 위해 setpoint 먼저 전송 (최소 1초)
    for (int i = 0; i < 10; i++) {
        publishOffboardControlMode();
        publishTrajectorySetpoint(current_ned_.x, current_ned_.y, current_ned_.z, current_yaw_ * DEG_TO_RAD);
        sleepMs(100);
    }

    // OFFBOARD 모드 전환 (setpoint 계속 발행하면서)
    sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    for (int i = 0; i < 5; i++) {  // 500ms 대기하면서 setpoint 발행
        publishOffboardControlMode();
        publishTrajectorySetpoint(current_ned_.x, current_ned_.y, current_ned_.z, current_yaw_ * DEG_TO_RAD);
        sleepMs(100);
    }

    // 시동
    sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);

    // 시동 대기 (setpoint 계속 발행)
    float elapsed = 0.0f;
    const float timeout = 10.0f;
    while (!isArmed() && elapsed < timeout && !stop_requested_) {
        publishOffboardControlMode();
        publishTrajectorySetpoint(current_ned_.x, current_ned_.y, current_ned_.z, current_yaw_ * DEG_TO_RAD);
        sleepMs(100);
        elapsed += 0.1f;
    }

    if (isArmed()) {
        RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 시동 완료 (%.1f초)", elapsed);
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "[OutdoorMission] 시동 실패 (타임아웃 %.1f초)", elapsed);
        return false;
    }
}

bool OutdoorMissionManager::disarm() {
    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 시동 해제 명령");

    sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);

    return waitUntil([this]() { return !isArmed(); }, 10.0f);
}

bool OutdoorMissionManager::takeoff(float altitude) {
    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 이륙 명령 (%.1fm)", altitude);

    float target_z = -altitude;  // NED: 아래가 양수

    // 타임아웃 설정 (30초)
    const float TAKEOFF_TIMEOUT = 30.0f;
    float start_time = node_->now().seconds();

    while (!stop_requested_) {
        // 타임아웃 체크
        float elapsed = node_->now().seconds() - start_time;
        if (elapsed >= TAKEOFF_TIMEOUT) {
            RCLCPP_ERROR(node_->get_logger(), "[OutdoorMission] 이륙 타임아웃 (%.1f초)", elapsed);
            return false;
        }

        publishOffboardControlMode();
        publishTrajectorySetpoint(current_ned_.x, current_ned_.y, target_z, current_yaw_ * DEG_TO_RAD);

        if (isAltitudeReached(altitude, 0.5f)) {
            RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 이륙 완료 (%.1f초)", elapsed);
            return true;
        }

        sleepMs(100);
    }

    return false;
}

bool OutdoorMissionManager::land() {
    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 착륙 명령");

    sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

    return waitUntil([this]() { return !isArmed(); }, 60.0f);
}

bool OutdoorMissionManager::flyToPosition(const GpsPosition& target) {
    // GPS → NED 변환
    NedPosition target_ned = gpsToNed(target, home_position_);
    target_ned.z = -target.altitude;  // 고도 설정

    return flyToNedPosition(target_ned);
}

bool OutdoorMissionManager::flyToNedPosition(const NedPosition& target) {
    RCLCPP_INFO(node_->get_logger(),
        "[OutdoorMission] 이동 → (%.1f, %.1f, %.1f)",
        target.x, target.y, -target.z);

    // 타임아웃 설정 (120초 - 최대 이동 시간)
    const float FLY_TIMEOUT = 120.0f;
    float start_time = node_->now().seconds();

    while (!stop_requested_) {
        // 타임아웃 체크
        float elapsed = node_->now().seconds() - start_time;
        if (elapsed >= FLY_TIMEOUT) {
            RCLCPP_ERROR(node_->get_logger(), "[OutdoorMission] 이동 타임아웃 (%.1f초)", elapsed);
            return false;
        }

        publishOffboardControlMode();
        publishTrajectorySetpoint(target.x, target.y, target.z, current_yaw_ * DEG_TO_RAD);

        float dx = target.x - current_ned_.x;
        float dy = target.y - current_ned_.y;
        float dz = target.z - current_ned_.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        // 10초마다 진행 상황 로그
        if (static_cast<int>(elapsed) % 10 == 0 && static_cast<int>(elapsed * 10) % 100 == 0) {
            RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 이동 중... 남은 거리: %.1fm", dist);
        }

        if (dist < 1.0f) {
            RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 위치 도착 (%.1f초)", elapsed);
            return true;
        }

        sleepMs(100);
    }

    return false;
}

bool OutdoorMissionManager::holdPosition() {
    publishOffboardControlMode();
    publishTrajectorySetpoint(current_ned_.x, current_ned_.y, current_ned_.z, current_yaw_ * DEG_TO_RAD);
    return true;
}

bool OutdoorMissionManager::setYaw(float yaw_deg) {
    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 헤딩 조정 → %.1f°", yaw_deg);

    float target_yaw = yaw_deg * DEG_TO_RAD;

    while (!stop_requested_) {
        publishOffboardControlMode();
        publishTrajectorySetpoint(current_ned_.x, current_ned_.y, current_ned_.z, target_yaw);

        float yaw_error = std::abs(yaw_deg - current_yaw_);
        if (yaw_error > 180.0f) yaw_error = 360.0f - yaw_error;

        if (yaw_error < 5.0f) {
            return true;
        }

        sleepMs(100);
    }

    return false;
}

// ============================================================================
// PX4 명령
// ============================================================================

void OutdoorMissionManager::sendVehicleCommand(uint16_t command, float param1,
                                                float param2, float param3) {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.timestamp = node_->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
}

void OutdoorMissionManager::publishOffboardControlMode() {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.timestamp = node_->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);
}

void OutdoorMissionManager::publishTrajectorySetpoint(float x, float y, float z, float yaw) {
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp = node_->now().nanoseconds() / 1000;
    msg.position = {x, y, z};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.yaw = yaw;
    msg.yawspeed = NAN;

    trajectory_setpoint_pub_->publish(msg);
}

// ============================================================================
// 유틸리티
// ============================================================================

float OutdoorMissionManager::getLidarDistance() {
    if (get_lidar_distance_) {
        return get_lidar_distance_();
    }
    return -1.0f;
}

std::pair<float, float> OutdoorMissionManager::getThermalCenter() {
    if (get_thermal_center_) {
        return get_thermal_center_();
    }
    return {-1.0f, -1.0f};
}

void OutdoorMissionManager::sendFireSignal(uint8_t shot_number) {
    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 격발 신호: %d", shot_number);
    if (send_fire_signal_) {
        send_fire_signal_(shot_number);
    }
}

GpsPosition OutdoorMissionManager::calculateOffsetPosition(
    const GpsPosition& origin, float distance_m, float bearing_deg) {

    double lat1 = origin.latitude * DEG_TO_RAD;
    double lon1 = origin.longitude * DEG_TO_RAD;
    double bearing = bearing_deg * DEG_TO_RAD;
    double d = distance_m / EARTH_RADIUS;

    double lat2 = std::asin(
        std::sin(lat1) * std::cos(d) +
        std::cos(lat1) * std::sin(d) * std::cos(bearing));

    double lon2 = lon1 + std::atan2(
        std::sin(bearing) * std::sin(d) * std::cos(lat1),
        std::cos(d) - std::sin(lat1) * std::sin(lat2));

    GpsPosition result;
    result.latitude = lat2 * RAD_TO_DEG;
    result.longitude = lon2 * RAD_TO_DEG;
    result.altitude = origin.altitude;
    result.yaw_deg = origin.yaw_deg;

    return result;
}

float OutdoorMissionManager::calculateBearing(const GpsPosition& from, const GpsPosition& to) {
    // gps_utils의 표준 bearing 함수 사용 (radians 반환)
    double bearing_rad = gps_utils::calculateBearing(
        from.latitude, from.longitude,
        to.latitude, to.longitude);

    double bearing = bearing_rad * RAD_TO_DEG;

    // 0~360 범위로 정규화
    while (bearing < 0) bearing += 360.0;
    while (bearing >= 360) bearing -= 360.0;

    return static_cast<float>(bearing);
}

float OutdoorMissionManager::calculateDistance(const GpsPosition& from, const GpsPosition& to) {
    // gps_utils의 표준 haversine 함수 사용
    return static_cast<float>(gps_utils::haversineDistance(
        from.latitude, from.longitude,
        to.latitude, to.longitude));
}

NedPosition OutdoorMissionManager::gpsToNed(const GpsPosition& gps, const GpsPosition& origin) {
    // 간단한 근사 변환 (작은 거리용)
    double dlat = (gps.latitude - origin.latitude) * DEG_TO_RAD;
    double dlon = (gps.longitude - origin.longitude) * DEG_TO_RAD;
    double lat_avg = (gps.latitude + origin.latitude) / 2.0 * DEG_TO_RAD;

    NedPosition ned;
    ned.x = static_cast<float>(dlat * EARTH_RADIUS);                      // North
    ned.y = static_cast<float>(dlon * EARTH_RADIUS * std::cos(lat_avg)); // East
    ned.z = -(gps.altitude - origin.altitude);                            // Down

    return ned;
}

bool OutdoorMissionManager::isArmed() {
    // TODO: VehicleStatus에서 확인
    return true;  // 임시
}

bool OutdoorMissionManager::isAltitudeReached(float target_alt, float tolerance) {
    float current_alt = -current_ned_.z;
    return std::abs(current_alt - target_alt) <= tolerance;
}

bool OutdoorMissionManager::isPositionReached(const GpsPosition& target, float tolerance) {
    float dist = calculateDistance(current_gps_, target);
    return dist <= tolerance;
}

bool OutdoorMissionManager::waitUntil(std::function<bool()> condition, float timeout_sec) {
    float elapsed = 0.0f;
    while (!condition() && elapsed < timeout_sec && !stop_requested_) {
        sleepMs(100);
        elapsed += 0.1f;
    }
    return condition();
}

void OutdoorMissionManager::sleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace outdoor_mission
