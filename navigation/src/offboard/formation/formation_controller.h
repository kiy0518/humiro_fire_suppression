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

    // === ROS2 노드 ===
    rclcpp::Node::SharedPtr node_;
    OffboardManager* offboard_mgr_;
    uint8_t drone_id_;
    FormationRole role_;
    std::atomic<bool> running_{false};

    // === 리더 Publishers ===
    rclcpp::Publisher<humiro_msgs::msg::LeaderPose>::SharedPtr leader_pose_pub_;
    rclcpp::Publisher<humiro_msgs::msg::FormationHeartbeat>::SharedPtr heartbeat_pub_;
    rclcpp::Publisher<humiro_msgs::msg::FormationCommand>::SharedPtr command_pub_;

    // === 리더 Subscribers ===
    rclcpp::Subscription<humiro_msgs::msg::FollowerStatus>::SharedPtr follower_status_sub_;

    // === 팔로워 Publishers ===
    rclcpp::Publisher<humiro_msgs::msg::FollowerStatus>::SharedPtr follower_status_pub_;

    // === 팔로워 Subscribers ===
    rclcpp::Subscription<humiro_msgs::msg::LeaderPose>::SharedPtr leader_pose_sub_;
    rclcpp::Subscription<humiro_msgs::msg::FormationHeartbeat>::SharedPtr heartbeat_sub_;
    rclcpp::Subscription<humiro_msgs::msg::FormationCommand>::SharedPtr command_sub_;

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
    float suppress_altitude_{-1.0f};   // 진압 고도 (offboard_config.json에서 로드)

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

    // === L/R 오프셋 미러링 (위치 기반 충돌 방지) ===
    // 팔로워와 오프셋 타겟이 리더→목적지 경로 반대편에 있으면 미러링
    bool lateral_offset_mirrored_{false}; // 현재 미러링 상태
    float approach_bearing_deg_{0.0f};    // 리더→목적지 방위각

    // === 진압 위치 사전 계산 (calculateOffsetTarget에서 10Hz 갱신) ===
    double precomputed_suppress_lat_{0.0};
    double precomputed_suppress_lon_{0.0};
    bool precomputed_suppress_valid_{false};

    // === 미션 목적지 (CMD_FOLLOW에서 수신, 미러링 판정용) ===
    double mission_dest_lat_{0.0};
    double mission_dest_lon_{0.0};
    bool mission_dest_set_{false};

    // === 팔로워 오프셋 추적 (offset_error 계산용) ===
    double last_target_lat_{0.0};
    double last_target_lon_{0.0};
    bool last_offset_valid_{false};

    // === 마지막 수신 리더 상태 (팔로워) ===
    std::string leader_phase_{"IDLE"};
    std::atomic<bool> leader_alive_{false};

    // === 콜백 ===
    CommandCallback command_callback_;

    // === GPS 변환 상수 ===
    static constexpr double DEG_TO_M_LAT = 111320.0;

    // === 편대 동기화 상수 ===
    static constexpr float SAFETY_RADIUS_M = 5.0f;                 // 리더 안전 반경 (m)
    static constexpr float FORMATION_COMPLETE_THRESHOLD_M = 5.0f;  // 편대 배치 완료 기준 (m)
};

#endif // FORMATION_CONTROLLER_H
