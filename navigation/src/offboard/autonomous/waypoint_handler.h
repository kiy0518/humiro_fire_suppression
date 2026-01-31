#ifndef WAYPOINT_HANDLER_H
#define WAYPOINT_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <cmath>
#include "gps_utils.h"

struct GPSCoordinate {
    double latitude;   // 위도 (degrees)
    double longitude;  // 경도 (degrees)
    float altitude;    // 고도 (meters, AMSL - Above Mean Sea Level)
};

class WaypointHandler {
public:
    explicit WaypointHandler(rclcpp::Node::SharedPtr node);

    /**
     * @brief GPS 좌표로 이동
     * @param target 목표 GPS 좌표
     * @param timeout_ms 타임아웃 (밀리초, 기본값 60초)
     * @return 이동 성공 여부
     */
    bool goToWaypoint(const GPSCoordinate& target, int timeout_ms = 60000, float flight_speed = 0.0f);

    /**
     * @brief 현재 GPS 위치 반환
     * @return 현재 GPS 좌표
     */
    GPSCoordinate getCurrentPosition() const;

    /**
     * @brief 목표 위치까지의 거리 계산 (Haversine formula)
     * @param target 목표 GPS 좌표
     * @return 거리 (미터)
     */
    double getDistanceToTarget(const GPSCoordinate& target) const;

    /**
     * @brief 목표 도달 여부 확인
     * @return 목표 위치 도달 여부
     */
    bool isWaypointReached() const;

    /**
     * @brief 현재 위치에서 hold-position setpoint를 지정 시간 동안 발행
     * @param duration_ms 유지 시간 (밀리초)
     */
    void holdPosition(int duration_ms);

    /**
     * @brief 외부 중단 플래그 설정 (OffboardManager에서 호출)
     */
    void setAbortFlag(std::atomic<bool>* abort_flag) { abort_flag_ = abort_flag; }

private:
    /**
     * @brief VehicleGlobalPosition 콜백
     */
    void vehicleGlobalPositionCallback(
        const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

    /**
     * @brief VehicleLocalPosition 콜백
     */
    void vehicleLocalPositionCallback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    /**
     * @brief TrajectorySetpoint 발행 (Local NED 좌표)
     */
    void publishTrajectorySetpoint(float x, float y, float z, float yaw,
                                       float vx = std::nanf(""), float vy = std::nanf(""), float vz = std::nanf(""));

    /**
     * @brief OffboardControlMode 발행
     */
    void publishOffboardControlMode();

    /**
     * @brief GPS를 Local NED 좌표로 변환
     * @param target 목표 GPS 좌표
     * @param local_x 출력: Local X (북쪽, 미터)
     * @param local_y 출력: Local Y (동쪽, 미터)
     * @param local_z 출력: Local Z (아래, 미터)
     */
    void gpsToLocalNED(const GPSCoordinate& target,
                       float& local_x, float& local_y, float& local_z);

    rclcpp::Node::SharedPtr node_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    // Timer for OFFBOARD mode heartbeat
    rclcpp::TimerBase::SharedPtr offboard_timer_;

    // State variables (Global)
    std::atomic<double> current_latitude_{0.0};
    std::atomic<double> current_longitude_{0.0};
    std::atomic<float> current_altitude_amsl_{0.0f};  // AMSL

    // State variables (Local NED)
    std::atomic<float> current_local_x_{0.0f};
    std::atomic<float> current_local_y_{0.0f};
    std::atomic<float> current_local_z_{0.0f};
    std::atomic<float> current_yaw_{0.0f};

    std::atomic<bool> global_position_received_{false};
    std::atomic<bool> local_position_received_{false};

    // Home position (for NED conversion)
    double home_latitude_{0.0};
    double home_longitude_{0.0};
    float home_altitude_amsl_{0.0f};

    // Target waypoint
    GPSCoordinate target_waypoint_{0.0, 0.0, 0.0f};

    // Abort flag (외부에서 설정)
    std::atomic<bool>* abort_flag_{nullptr};

    // Constants
    static constexpr float WAYPOINT_THRESHOLD = 2.0f;   // 2m 오차 허용
};

#endif // WAYPOINT_HANDLER_H
