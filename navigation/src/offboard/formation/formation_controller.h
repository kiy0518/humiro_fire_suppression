/**
 * @file formation_controller.h
 * @brief 편대 비행 제어기 - ROS2 DDS 기반 리더/팔로워
 *
 * 리더: 자신의 위치/상태를 ROS2 토픽으로 발행
 * 팔로워: 리더 토픽을 구독하여 오프셋 위치로 추적
 *
 * 통신: ROS2 DDS over WiFi (FastDDS)
 * 메시지: humiro_msgs (LeaderPose(+mission_state), FollowerStatus 등)
 */

#ifndef FORMATION_CONTROLLER_H
#define FORMATION_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <humiro_msgs/msg/leader_pose.hpp>
#include <humiro_msgs/msg/leader_aim_pose.hpp>
#include <humiro_msgs/msg/follower_status.hpp>
#include <humiro_msgs/msg/formation_command.hpp>
#include <humiro_msgs/msg/formation_heartbeat.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>

// 전방 선언
class OffboardManager;
struct GPSCoordinate;

enum class FormationRole {
    LEADER,
    FOLLOWER
};

enum class FollowerPhase {
    IDLE,           // 미션 대기
    FOLLOWING,      // 리더 추적 (이동 편대)
    SUPPRESSING,    // 진압 위치 비행 (타겟 중심)
    HOLD,           // 현재 위치 유지
    RTL             // 귀환
};

class FormationController {
public:
    FormationController(rclcpp::Node::SharedPtr node,
                        OffboardManager* offboard_mgr,
                        uint8_t drone_id,
                        FormationRole role);
    ~FormationController();

    // 편대 제어 시작/중지
    void start();
    void stop();
    bool isRunning() const { return running_.load(); }

    // === 리더 전용 ===
    void setFormationDroneCount(uint8_t count);
    void setFormationPhase(const std::string& phase);
    void sendCommand(uint8_t target_drone_id, uint8_t command,
                     double target_lat = 0.0, double target_lon = 0.0);
    void setMissionTarget(double lat, double lon);
    void setMissionParams(float takeoff_alt, float flight_speed);

    // === 팔로워 전용 ===
    void setOffset(int16_t right_cm, int16_t behind_cm, int16_t above_cm);
    void setLeaderNamespace(const std::string& ns);
    void setSuppressParams(int16_t distance_m, int16_t angle_deg);

    // Solo 모드 제어 (solo 모드에서는 LeaderPose/Heartbeat/CMD_FOLLOW 발행 안 함)
    void setSoloMode(bool solo) { solo_mode_.store(solo); }
    bool isSoloMode() const { return solo_mode_.load(); }

    // 상태 조회
    FormationRole getRole() const { return role_; }
    uint8_t getDroneId() const { return drone_id_; }
    FollowerPhase getFollowerPhase() const { return follower_phase_; }
    float getReceivedAltitude() const { return received_takeoff_altitude_; }
    float getReceivedSpeed() const { return received_flight_speed_; }

    // 명령 수신 콜백 (ApplicationManager에서 설정)
    using CommandCallback = std::function<void(uint8_t command, double lat, double lon)>;
    void setCommandCallback(CommandCallback cb) { command_callback_ = cb; }

private:
    // === 리더 기능 ===
    void initLeader();
    void leaderPoseTimerCallback();       // 10Hz (위치+mission_state)
    void leaderStatusTimerCallback();     // 1Hz (편대 동기화 체크)
    void heartbeatTimerCallback();        // 1Hz
    void onFollowerStatus(const humiro_msgs::msg::FollowerStatus::SharedPtr msg);

    // === 팔로워 기능 ===
    void initFollower();
    void onLeaderPose(const humiro_msgs::msg::LeaderPose::SharedPtr msg);  // 위치+상태 수신
    void onHeartbeat(const humiro_msgs::msg::FormationHeartbeat::SharedPtr msg);
    void onFormationCommand(const humiro_msgs::msg::FormationCommand::SharedPtr msg);
    void followerStatusTimerCallback();   // 2Hz
    void leaderTimeoutTimerCallback();    // 1Hz: 리더 하트비트 타임아웃 체크

    // === 오프셋 계산 ===
    GPSCoordinate calculateOffsetTarget(const humiro_msgs::msg::LeaderPose& leader_pose);

    // === 리더: SUPPRESS 전환 ===
    void triggerSuppressPhase();
    static std::string followerPhaseToString(FollowerPhase phase);

    // === 리더: LeaderAimPose 발행 (DISTANCE_ADJUST 완료 후) ===
    void publishLeaderAimPose();

    // === 팔로워: LeaderAimPose 수신/진압 위치 계산 ===
    void onLeaderAimPose(const humiro_msgs::msg::LeaderAimPose::SharedPtr msg);
    GPSCoordinate calculateSuppressPosition();

    // === ROS2 노드 ===
    rclcpp::Node::SharedPtr node_;
    OffboardManager* offboard_mgr_;
    uint8_t drone_id_;
    FormationRole role_;
    std::atomic<bool> running_{false};
    std::atomic<bool> solo_mode_{true};   // 기본값: solo (formation 모드에서만 false로 전환)

    // === 리더 Publishers ===
    rclcpp::Publisher<humiro_msgs::msg::LeaderPose>::SharedPtr leader_pose_pub_;
    rclcpp::Publisher<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_pose_pub_;
    rclcpp::Publisher<humiro_msgs::msg::FormationHeartbeat>::SharedPtr heartbeat_pub_;
    rclcpp::Publisher<humiro_msgs::msg::FormationCommand>::SharedPtr command_pub_;

    // === 리더 Subscribers ===
    rclcpp::Subscription<humiro_msgs::msg::FollowerStatus>::SharedPtr follower_status_sub_;

    // === 팔로워 Publishers ===
    rclcpp::Publisher<humiro_msgs::msg::FollowerStatus>::SharedPtr follower_status_pub_;

    // === 팔로워 Subscribers ===
    rclcpp::Subscription<humiro_msgs::msg::LeaderPose>::SharedPtr leader_pose_sub_;
    rclcpp::Subscription<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_pose_sub_;
    rclcpp::Subscription<humiro_msgs::msg::FormationHeartbeat>::SharedPtr heartbeat_sub_;
    rclcpp::Subscription<humiro_msgs::msg::FormationCommand>::SharedPtr command_sub_;

    // === 팔로워: LeaderAimPose 저장 ===
    humiro_msgs::msg::LeaderAimPose leader_aim_pose_;
    bool leader_aim_received_{false};

    // === 타이머 ===
    rclcpp::TimerBase::SharedPtr leader_pose_timer_;      // 10Hz (위치+상태)
    rclcpp::TimerBase::SharedPtr leader_status_timer_;    // 1Hz (편대 동기화)
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;        // 1Hz
    rclcpp::TimerBase::SharedPtr follower_status_timer_;  // 2Hz
    rclcpp::TimerBase::SharedPtr leader_timeout_timer_;   // 1Hz 타임아웃 체크

    // === 편대 오프셋 (팔로워) ===
    int16_t offset_right_cm_{0};
    int16_t offset_behind_cm_{0};
    int16_t offset_above_cm_{0};
    std::string leader_namespace_;

    // === 팔로워 상태 머신 ===
    FollowerPhase follower_phase_{FollowerPhase::IDLE};
    int16_t suppress_distance_m_{10};
    int16_t suppress_angle_deg_{0};

    // === SUPPRESS 우회 경로 (횡단 방지) ===
    double suppress_final_lat_{0.0};            // 최종 진압 위치
    double suppress_final_lon_{0.0};
    float suppress_final_alt_{0.0f};
    bool suppress_detour_active_{false};        // 중간 경유점 사용 중

    // === 편대 상태 (리더) ===
    uint8_t formation_drone_count_{1};
    std::string formation_phase_{"IDLE"};
    double mission_target_lat_{0.0};
    double mission_target_lon_{0.0};

    // === 미션 파라미터 (리더→팔로워 전달) ===
    float mission_takeoff_altitude_{10.0f};
    float mission_flight_speed_{5.0f};
    bool cmd_follow_sent_{false};

    // === 편대 동기화 (리더: OffboardManager 게이트 제어) ===
    bool formation_ready_to_rotate_notified_{false};
    bool formation_ready_to_navigate_notified_{false};

    // === 팔로워가 CMD_FOLLOW에서 수신한 파라미터 ===
    float received_takeoff_altitude_{10.0f};
    float received_flight_speed_{5.0f};

    // === 리더 하트비트 타임아웃 (팔로워) ===
    std::chrono::steady_clock::time_point last_heartbeat_time_;
    std::chrono::steady_clock::time_point last_leader_pose_time_;
    static constexpr int LEADER_TIMEOUT_SEC = 3;

    // === 팔로워 상태 (리더가 추적) ===
    struct FollowerInfo {
        uint8_t drone_id;
        double latitude;
        double longitude;
        float altitude;
        float offset_error;
        std::string mission_state;
        uint8_t battery_percent;
        uint8_t ammo_count;
        std::chrono::steady_clock::time_point last_update;
    };
    std::mutex followers_mutex_;
    std::map<uint8_t, FollowerInfo> followers_;

    // === 경로 기반 오프셋 ===
    double last_leader_lat_{0.0};         // 최근 수신 리더 위치
    double last_leader_lon_{0.0};
    double mission_dest_lat_{0.0};        // 미션 목적지
    double mission_dest_lon_{0.0};
    bool mission_dest_set_{false};

    // === 고정 경로선 (CMD_FOLLOW 시점 확정, 목적지 변경 시 재계산) ===
    double leader_start_lat_{0.0};        // 리더 이륙(고정) 위치
    double leader_start_lon_{0.0};
    float fixed_heading_rad_{0.0f};       // 고정 경로 heading (리더이륙→목적지)
    bool fixed_heading_set_{false};       // 고정 heading 설정 여부
    bool lateral_mirrored_{false};        // offset_right 부호 반전 (팔로워 시작 쪽 유지)

    // === 팔로워 오프셋 추적 (offset_error 계산용) ===
    double last_target_lat_{0.0};
    double last_target_lon_{0.0};
    bool last_offset_valid_{false};

    // === 마지막 수신 리더 상태 (팔로워) ===
    std::string leader_phase_{"IDLE"};
    std::atomic<bool> leader_alive_{false};

    // === SUPPRESS 대기 로그 제어 ===
    bool suppress_wait_logged_{false};

    // === 콜백 ===
    CommandCallback command_callback_;

    // === GPS 변환 상수 ===
    static constexpr double DEG_TO_M_LAT = 111320.0;

    // === 편대 동기화 상수 ===
    static constexpr float SAFETY_RADIUS_M = 5.0f;                 // 리더 안전 반경 (m)
    static constexpr float FORMATION_COMPLETE_THRESHOLD_M = 5.0f;  // 편대 배치 완료 기준 (m)
};

#endif // FORMATION_CONTROLLER_H
