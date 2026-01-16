#ifndef OUTDOOR_MISSION_MANAGER_H
#define OUTDOOR_MISSION_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <cmath>

// PX4 메시지
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

// Humiro 커스텀 메시지
#include <humiro_msgs/msg/leader_status.hpp>
#include <humiro_msgs/msg/leader_pose.hpp>
#include <humiro_msgs/msg/leader_aim_pose.hpp>

namespace outdoor_mission {

// 미션 단계 정의
enum class MissionPhase {
    IDLE,
    ARMING,
    TAKEOFF,
    WAYPOINT,
    ARRIVED,
    DISTANCE_ADJ,
    FIRE_FIGHTING,
    AIMING,
    FIRING,
    RETURNING,
    LANDING,
    DISARMED,
    ERROR
};

// 미션 단계 이름 변환
inline std::string phaseToString(MissionPhase phase) {
    switch (phase) {
        case MissionPhase::IDLE: return "IDLE";
        case MissionPhase::ARMING: return "ARMING";
        case MissionPhase::TAKEOFF: return "TAKEOFF";
        case MissionPhase::WAYPOINT: return "WAYPOINT";
        case MissionPhase::ARRIVED: return "ARRIVED";
        case MissionPhase::DISTANCE_ADJ: return "DISTANCE_ADJ";
        case MissionPhase::FIRE_FIGHTING: return "FIRE_FIGHTING";
        case MissionPhase::AIMING: return "AIMING";
        case MissionPhase::FIRING: return "FIRING";
        case MissionPhase::RETURNING: return "RETURNING";
        case MissionPhase::LANDING: return "LANDING";
        case MissionPhase::DISARMED: return "DISARMED";
        case MissionPhase::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// GPS 위치 구조체
struct GpsPosition {
    double latitude = 0.0;   // degrees
    double longitude = 0.0;  // degrees
    float altitude = 0.0f;   // meters
    float yaw_deg = 0.0f;    // degrees (0~360)
};

// NED 위치 구조체
struct NedPosition {
    float x = 0.0f;  // North (m)
    float y = 0.0f;  // East (m)
    float z = 0.0f;  // Down (m, 음수가 위)
};

// 미션 설정
struct OutdoorMissionConfig {
    // 웨이포인트 (목표 위치)
    GpsPosition waypoint;

    // 이륙/RTL 고도
    float takeoff_altitude = 10.0f;
    float rtl_altitude = 15.0f;

    // 거리 제어
    float target_distance = 10.0f;      // 목표물까지 거리 (m)
    float distance_tolerance = 0.5f;    // 허용 오차 (m)
    float forward_speed = 0.4f;         // 전진 속도 (m/s)
    float backward_speed = 0.25f;       // 후진 속도 (m/s)
    float distance_stable_time = 3.0f;  // 안정화 시간 (초)

    // 조준 제어
    float aim_tolerance = 0.05f;        // 5%
    float aim_stable_time = 1.0f;       // 초

    // 격발 제어
    uint8_t num_shots = 6;
    float shot_interval = 5.0f;         // 초

    // 편대 설정 (팔로워용)
    float formation_distance = 10.0f;   // 리더와의 거리 (m)
    float formation_angle_2 = 135.0f;   // 2번: 좌측 후방
    float formation_angle_3 = -135.0f;  // 3번: 우측 후방

    // 발사 지점 설정 (정삼각형)
    float fire_distance = 10.0f;        // 목표물과의 거리 (m)
    float fire_angle_2 = 60.0f;         // 2번: +60도
    float fire_angle_3 = -60.0f;        // 3번: -60도
};

/**
 * @brief 야외 GPS 기반 스웜 미션 관리자
 *
 * 리더(1번): QGC 메시지 수신 → 웨이포인트 이동 → 조준 → 격발 → 복귀
 * 팔로워(2,3번): 리더 토픽 구독 → 편대 비행 → 정삼각형 포메이션 → 조준 → 격발 → 복귀
 */
class OutdoorMissionManager {
public:
    OutdoorMissionManager(rclcpp::Node::SharedPtr node, uint8_t vehicle_id);
    ~OutdoorMissionManager();

    /**
     * @brief 야외 미션 실행 (메인 진입점)
     * @param config 미션 설정
     * @return 성공 여부
     */
    bool executeMission(const OutdoorMissionConfig& config);

    /**
     * @brief 미션 중지
     */
    void stopMission();

    /**
     * @brief 현재 미션 단계 반환
     */
    MissionPhase getCurrentPhase() const { return current_phase_; }

    /**
     * @brief LiDAR 거리 콜백 설정
     */
    void setLidarDistanceCallback(std::function<float()> callback) {
        get_lidar_distance_ = callback;
    }

    /**
     * @brief 열화상 중심점 콜백 설정
     */
    void setThermalCenterCallback(std::function<std::pair<float, float>()> callback) {
        get_thermal_center_ = callback;
    }

    /**
     * @brief 격발 신호 콜백 설정
     */
    void setFireSignalCallback(std::function<void(uint8_t)> callback) {
        send_fire_signal_ = callback;
    }

private:
    // ========== 노드 및 기본 설정 ==========
    rclcpp::Node::SharedPtr node_;
    uint8_t vehicle_id_;
    bool is_leader_;
    std::atomic<bool> mission_active_{false};
    std::atomic<bool> stop_requested_{false};
    MissionPhase current_phase_ = MissionPhase::IDLE;
    OutdoorMissionConfig config_;

    // ========== 상태 정보 ==========
    GpsPosition home_position_;
    GpsPosition current_gps_;
    NedPosition current_ned_;
    float current_yaw_ = 0.0f;
    GpsPosition target_position_;  // 목표물 위치 (리더가 계산)

    // ========== 리더 토픽 발행자 ==========
    rclcpp::Publisher<humiro_msgs::msg::LeaderStatus>::SharedPtr leader_status_pub_;
    rclcpp::Publisher<humiro_msgs::msg::LeaderPose>::SharedPtr leader_pose_pub_;
    rclcpp::Publisher<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_pose_pub_;

    // ========== 팔로워 토픽 구독자 ==========
    rclcpp::Subscription<humiro_msgs::msg::LeaderStatus>::SharedPtr leader_status_sub_;
    rclcpp::Subscription<humiro_msgs::msg::LeaderPose>::SharedPtr leader_pose_sub_;
    rclcpp::Subscription<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_pose_sub_;

    // ========== PX4 토픽 ==========
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    // ========== 리더 상태 (팔로워용) ==========
    humiro_msgs::msg::LeaderStatus leader_status_;
    humiro_msgs::msg::LeaderPose leader_pose_;
    humiro_msgs::msg::LeaderAimPose leader_aim_pose_;
    std::mutex leader_data_mutex_;

    // ========== 외부 콜백 ==========
    std::function<float()> get_lidar_distance_;
    std::function<std::pair<float, float>()> get_thermal_center_;
    std::function<void(uint8_t)> send_fire_signal_;

    // ========== 타이머 ==========
    rclcpp::TimerBase::SharedPtr pose_publish_timer_;

    // ========== 초기화 ==========
    void initPublishers();
    void initSubscribers();
    void initLeaderTopics();
    void initFollowerTopics();

    // ========== 리더 시퀀스 ==========
    bool runLeaderSequence();
    bool leaderArm();
    bool leaderTakeoff();
    bool leaderFlyToWaypoint();
    bool leaderAdjustDistance();
    bool leaderAim();
    bool leaderFire();
    bool leaderReturn();
    bool leaderLand();
    bool leaderDisarm();

    // ========== 팔로워 시퀀스 ==========
    bool runFollowerSequence();
    void onLeaderStatusCallback(const humiro_msgs::msg::LeaderStatus::SharedPtr msg);
    void onLeaderPoseCallback(const humiro_msgs::msg::LeaderPose::SharedPtr msg);
    void onLeaderAimPoseCallback(const humiro_msgs::msg::LeaderAimPose::SharedPtr msg);
    GpsPosition calculateFormationPosition();   // 이동 중 편대 위치
    GpsPosition calculateFirePosition();        // 발사 지점 (정삼각형)

    // ========== 리더 토픽 발행 ==========
    void publishLeaderStatus(uint8_t shot = 0);
    void publishLeaderPose();
    void publishLeaderAimPose();
    void startPosePublishing();
    void stopPosePublishing();

    // ========== 공통 비행 함수 ==========
    bool arm();
    bool disarm();
    bool takeoff(float altitude);
    bool land();
    bool flyToPosition(const GpsPosition& target);
    bool flyToNedPosition(const NedPosition& target);
    bool holdPosition();
    bool setYaw(float yaw_deg);

    // ========== PX4 명령 ==========
    void sendVehicleCommand(uint16_t command, float param1 = 0.0f,
                           float param2 = 0.0f, float param3 = 0.0f);
    void publishOffboardControlMode();
    void publishTrajectorySetpoint(float x, float y, float z, float yaw);

    // ========== 유틸리티 ==========
    float getLidarDistance();
    std::pair<float, float> getThermalCenter();
    void sendFireSignal(uint8_t shot_number);

    // GPS 계산
    static GpsPosition calculateOffsetPosition(const GpsPosition& origin,
                                               float distance_m, float bearing_deg);
    static float calculateBearing(const GpsPosition& from, const GpsPosition& to);
    static float calculateDistance(const GpsPosition& from, const GpsPosition& to);
    static NedPosition gpsToNed(const GpsPosition& gps, const GpsPosition& origin);

    // 상태 확인
    bool isArmed();
    bool isAltitudeReached(float target_alt, float tolerance = 0.5f);
    bool isPositionReached(const GpsPosition& target, float tolerance = 1.0f);
    bool isDistanceStable(float target_dist, float tolerance, float stable_time);
    bool isAimingStable(float tolerance, float stable_time);

    // 대기 함수
    bool waitUntil(std::function<bool()> condition, float timeout_sec = 30.0f);
    void sleepMs(int ms);
};

} // namespace outdoor_mission

#endif // OUTDOOR_MISSION_MANAGER_H
