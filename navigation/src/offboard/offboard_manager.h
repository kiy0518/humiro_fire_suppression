/**
 * @file offboard_manager.h
 * @brief PX4 Offboard 제어 - 단순화된 버전
 *
 * PX4 공식 문서 기반:
 * - OffboardControlMode heartbeat를 2Hz 이상 지속적으로 발행 (10Hz 사용)
 * - TrajectorySetpoint으로 위치 제어
 * - Arming 전부터 메시지 발행 필요
 */

#ifndef OFFBOARD_MANAGER_H
#define OFFBOARD_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>

// GPS 좌표 구조체
struct GPSCoordinate {
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0f;
};

// 미션 설정 구조체
struct MissionConfig {
    float takeoff_altitude = 5.0f;           // 이륙 고도 (미터)
    float target_altitude = -1.0f;           // 목표지점 고도 (미터), -1이면 takeoff_altitude 사용
    float flight_speed = 5.0f;               // 비행 속도 (m/s)
    GPSCoordinate target_waypoint;           // 목표 위치
    float hover_duration_sec = 3.0f;         // 호버링 시간 (초)
};

// 미션 상태
enum class MissionState {
    IDLE,           // 대기
    PREPARING,      // heartbeat 발행 중 (2초)
    OFFBOARD,       // OFFBOARD 모드 전환
    ARMING,         // ARM 요청
    TAKEOFF,        // 이륙 중
    HOVER,          // 호버링 (안정화)
    ROTATE,         // 목표 방향 회전
    NAVIGATE,       // 목표 위치 이동
    HOVER_AT_TARGET,// 목표지점 호버링 (5초)
    RTL,            // 귀환
    LANDED,         // 착륙 완료
    ERROR           // 에러
};

class OffboardManager {
public:
    explicit OffboardManager(rclcpp::Node::SharedPtr node);
    ~OffboardManager();

    /**
     * @brief 미션 실행 (블로킹)
     * 순서: PREPARING → OFFBOARD → ARMING → TAKEOFF → HOVER → ROTATE → NAVIGATE → HOVER_AT_TARGET → RTL
     * @param config 미션 설정
     * @return 미션 성공 여부
     */
    bool executeMission3(const MissionConfig& config);

    /**
     * @brief 군집비행 미션 (executeMission3 + 진압 대기 30초)
     * HOVER_AT_TARGET에서 30초간 진압 편대 유지 후 RTL
     */
    bool executeMission4(const MissionConfig& config);

    /**
     * @brief 미션 진행 중 목표 좌표 업데이트 (경로 변경)
     * @param new_target 새 목표 GPS 좌표
     * @param yaw_override 헤딩 오버라이드 (rad). NAN이면 목표방향 자동계산
     * @return 업데이트 성공 여부 (미션 진행 중일 때만 true)
     */
    bool updateMissionTarget(const GPSCoordinate& new_target, float yaw_override = NAN);

    /**
     * @brief 미션 진행 중인지 확인
     */
    bool isMissionRunning() const { return mission_running_.load(); }

    /**
     * @brief 미션 중단 요청
     */
    void abortMission();

    /**
     * @brief 긴급 RTL
     */
    void emergencyRTL();

    /**
     * @brief 현재 상태 반환
     */
    MissionState getCurrentState() const { return current_state_.load(); }

    /**
     * @brief 상태 이름 반환
     */
    static std::string getStateName(MissionState state);

    /**
     * @brief IDLE 상태로 리셋
     */
    void resetToIdle();

    // ========== 편대 비행 게이트 (FormationController 연동) ==========
    void setFormationReadyToRotate(bool ready);
    void setFormationReadyToNavigate(bool ready);
    void setContinuousUpdateMode(bool enabled);

    // ========== 위치/속도 Getter (FormationController용) ==========
    double getCurrentLat() const { return current_lat_.load(); }
    double getCurrentLon() const { return current_lon_.load(); }
    float getCurrentAltAmsl() const { return current_alt_amsl_.load(); }
    float getCurrentYaw() const { return current_yaw_.load(); }
    float getCurrentLocalX() const { return current_local_x_.load(); }
    float getCurrentLocalY() const { return current_local_y_.load(); }
    float getCurrentLocalZ() const { return current_local_z_.load(); }
    float getCurrentVx() const { return actual_vx_.load(); }
    float getCurrentVy() const { return actual_vy_.load(); }
    float getCurrentVz() const { return actual_vz_.load(); }
    float getCurrentSpeed() const {
        float vx = actual_vx_.load(), vy = actual_vy_.load();
        return std::sqrt(vx * vx + vy * vy);
    }

    // ========== 타겟 위치 Getter ==========
    float getTargetNedX() const { return target_ned_x_; }
    float getTargetNedY() const { return target_ned_y_; }

    // ========== 충돌 방지 ==========
    void setCollisionAvoidance(class CollisionAvoidance* ca) { collision_avoidance_ = ca; }
    bool isCollisionActive() const { return collision_action_.load() != 0; }
    int getCollisionAction() const { return collision_action_.load(); }

private:
    // ========== 타이머 콜백 (핵심!) ==========
    void timerCallback();

    // ========== 메시지 발행 ==========
    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f);

    // ========== 콜백 ==========
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicleGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

    // ========== 헬퍼 함수 ==========
    float calculateTargetYaw(float target_north, float target_east);
    void gpsToLocalNED(double target_lat, double target_lon, float target_alt,
                       float& local_x, float& local_y, float& local_z);

    // ========== ROS2 노드 ==========
    rclcpp::Node::SharedPtr node_;

    // ========== Publishers ==========
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // ========== Subscribers ==========
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;

    // ========== Timer ==========
    rclcpp::TimerBase::SharedPtr timer_;

    // ========== 상태 ==========
    std::atomic<MissionState> current_state_{MissionState::IDLE};
    std::atomic<uint64_t> setpoint_counter_{0};
    std::atomic<bool> abort_requested_{false};
    std::atomic<bool> mission_running_{false};
    std::mutex mission_start_mutex_;  // 중복 미션 시작 방지 (executeMission3 진입 보호)

    // ========== FC 상태 ==========
    std::atomic<uint8_t> nav_state_{0};
    std::atomic<uint8_t> arming_state_{0};

    // ========== 위치 데이터 (atomic) ==========
    std::atomic<float> current_local_x_{0.0f};
    std::atomic<float> current_local_y_{0.0f};
    std::atomic<float> current_local_z_{0.0f};
    std::atomic<float> current_yaw_{0.0f};
    std::atomic<double> current_lat_{0.0};
    std::atomic<double> current_lon_{0.0};
    std::atomic<float> current_alt_amsl_{0.0f};
    std::atomic<bool> position_received_{false};
    std::atomic<float> actual_vx_{0.0f};    // PX4 실제 NED 속도
    std::atomic<float> actual_vy_{0.0f};
    std::atomic<float> actual_vz_{0.0f};

    // ========== Home 위치 ==========
    double home_lat_{0.0};
    double home_lon_{0.0};
    float home_alt_amsl_{0.0f};
    bool home_set_{false};

    // ========== 미션 설정 ==========
    MissionConfig mission_config_;
    float initial_yaw_{0.0f};           // 이륙 시점 헤딩

    // ========== 시작 위치 (미션 시작 시점 로컬 NED) ==========
    float start_local_x_{0.0f};
    float start_local_y_{0.0f};
    float start_local_z_{0.0f};

    // ========== 목표 NED 좌표 ==========
    float target_ned_x_{0.0f};
    float target_ned_y_{0.0f};
    float target_ned_z_{0.0f};
    float target_yaw_{0.0f};

    // ========== Velocity 보간용 (부드러운 선회) ==========
    float prev_vx_{0.0f};
    float prev_vy_{0.0f};

    // ========== 목표지점 호버링 ==========
    std::atomic<uint64_t> hover_at_target_start_{0};

    // ========== 타이밍 상수 (10Hz 기준) ==========
    static constexpr uint64_t PREPARE_COUNT = 20;    // 2초: heartbeat 준비
    static constexpr uint64_t ARM_COUNT = 45;        // 4.5초: ARM
    static constexpr uint64_t TAKEOFF_STABLE = 80;   // 8초: 이륙 안정화
    static constexpr uint64_t ROTATE_START = 110;    // 11초: 회전 시작
    static constexpr uint64_t ROTATE_END = 170;      // 17초: 회전 완료
    static constexpr uint64_t MOVE_START = 180;      // 18초: 이동 시작
    // RTL_TIMEOUT 제거 - 미션 시간 제한 없음 (미션 완료 또는 수동 개입으로만 RTL)
    static constexpr uint64_t TARGET_HOVER_TICKS = 50; // 5초: 목표지점 호버링 (10Hz)
    static constexpr uint64_t SUPPRESS_HOVER_TICKS = 300; // 30초: 진압 편대 대기 (10Hz)

    // ========== 군집비행 모드 ==========
    bool formation_mode_{false};
    std::atomic<bool> formation_ready_to_rotate_{false};
    std::atomic<bool> formation_ready_to_navigate_{false};
    std::atomic<bool> continuous_update_mode_{false};

    // ========== 드론 식별 ==========
    uint8_t target_system_{1};  // PX4 MAV_SYS_ID (VehicleCommand target)

    // ========== 제어 파라미터 ==========
    static constexpr float MAX_YAW_RATE = 0.5f;      // 최대 회전 속도 (rad/s, ~28.6 deg/s)
    static constexpr float WAYPOINT_THRESHOLD = 2.0f; // 도착 판정 거리 (m)

    // ========== 충돌 방지 ==========
    class CollisionAvoidance* collision_avoidance_{nullptr};
    std::atomic<int> collision_action_{0};  // 0=NONE, 1=HOLD, 2=EVADE_RIGHT
    bool was_collision_active_{false};
    float hold_x_{0.0f}, hold_y_{0.0f}, hold_z_{0.0f}, hold_yaw_{0.0f};
    float evade_offset_n_{0.0f}, evade_offset_e_{0.0f};  // 우회 오프셋 (NED)

};

#endif // OFFBOARD_MANAGER_H
