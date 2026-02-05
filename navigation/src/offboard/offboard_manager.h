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

// GPS 좌표 구조체
struct GPSCoordinate {
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0f;
};

// 미션 설정 구조체
struct MissionConfig {
    float takeoff_altitude = 5.0f;           // 이륙 고도 (미터)
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
     * 순서: PREPARING → OFFBOARD → ARMING → TAKEOFF → HOVER → ROTATE → NAVIGATE → RTL
     * @param config 미션 설정
     * @return 미션 성공 여부
     */
    bool executeMission3(const MissionConfig& config);

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

private:
    // ========== 타이머 콜백 (핵심!) ==========
    void timerCallback();

    // ========== 메시지 발행 ==========
    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

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

    // ========== Home 위치 ==========
    double home_lat_{0.0};
    double home_lon_{0.0};
    float home_alt_amsl_{0.0f};
    bool home_set_{false};

    // ========== 미션 설정 ==========
    MissionConfig mission_config_;
    float initial_yaw_{0.0f};           // 이륙 시점 헤딩
    float prev_yaw_diff_{0.0f};         // PD 제어용

    // ========== 목표 NED 좌표 ==========
    float target_ned_x_{0.0f};
    float target_ned_y_{0.0f};
    float target_ned_z_{0.0f};
    float target_yaw_{0.0f};

    // ========== 타이밍 상수 (10Hz 기준) ==========
    static constexpr uint64_t PREPARE_COUNT = 20;    // 2초: heartbeat 준비
    static constexpr uint64_t ARM_COUNT = 45;        // 4.5초: ARM
    static constexpr uint64_t TAKEOFF_STABLE = 80;   // 8초: 이륙 안정화
    static constexpr uint64_t ROTATE_START = 110;    // 11초: 회전 시작
    static constexpr uint64_t ROTATE_END = 170;      // 17초: 회전 완료
    static constexpr uint64_t MOVE_START = 180;      // 18초: 이동 시작
    static constexpr uint64_t RTL_TIMEOUT = 600;     // 60초: 최대 미션 시간

    // ========== 제어 파라미터 ==========
    static constexpr float MAX_YAW_RATE = 0.5f;      // 최대 회전 속도 (rad/s)
    static constexpr float WAYPOINT_THRESHOLD = 2.0f; // 도착 판정 거리 (m)
    static constexpr float K_P = 1.5f;               // PD 제어 비례 게인
    static constexpr float K_D = 0.9f;               // PD 제어 미분 게인

};

#endif // OFFBOARD_MANAGER_H
