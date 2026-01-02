#ifndef DISTANCE_ADJUSTER_H
#define DISTANCE_ADJUSTER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <atomic>
#include <chrono>
#include <memory>

class DistanceAdjuster {
public:
    explicit DistanceAdjuster(rclcpp::Node::SharedPtr node);

    /**
     * @brief 목표 거리로 조정 (전후 이동)
     * @param target_distance 목표 거리 (미터, 기본값 10.0m)
     * @param tolerance 허용 오차 (미터, 기본값 1.0m)
     * @param timeout_ms 타임아웃 (밀리초, 기본값 30초)
     * @return 거리 조정 성공 여부
     */
    bool adjustDistance(float target_distance = 10.0f,
                       float tolerance = 1.0f,
                       int timeout_ms = 30000);

    /**
     * @brief 현재 전방 거리 반환
     * @return 전방 거리 (미터)
     */
    float getFrontDistance() const;

    /**
     * @brief 거리 조정 완료 여부 확인
     * @return 목표 거리 범위 내 도달 여부
     */
    bool isDistanceAdjusted() const;

    /**
     * @brief 현재 위치에서 호버링
     */
    void hover();

private:
    /**
     * @brief LiDAR 전방 거리 콜백
     */
    void frontDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief VehicleLocalPosition 콜백
     */
    void vehicleLocalPositionCallback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    /**
     * @brief TrajectorySetpoint 발행 (Local NED 좌표)
     */
    void publishTrajectorySetpoint(float x, float y, float z, float yaw);

    /**
     * @brief OffboardControlMode 발행
     */
    void publishOffboardControlMode();

    rclcpp::Node::SharedPtr node_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_distance_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    // Timer for OFFBOARD mode heartbeat
    rclcpp::TimerBase::SharedPtr offboard_timer_;

    // State variables
    std::atomic<float> front_distance_{0.0f};      // LiDAR 전방 거리
    std::atomic<float> current_local_x_{0.0f};     // 현재 X (북쪽)
    std::atomic<float> current_local_y_{0.0f};     // 현재 Y (동쪽)
    std::atomic<float> current_local_z_{0.0f};     // 현재 Z (아래)
    std::atomic<float> current_yaw_{0.0f};         // 현재 Yaw

    std::atomic<bool> distance_received_{false};
    std::atomic<bool> position_received_{false};

    // Target parameters
    float target_distance_{10.0f};
    float distance_tolerance_{1.0f};

    // Initial position (for relative movement)
    float start_x_{0.0f};

    // Constants
    static constexpr float MAX_DISTANCE_ERROR = 5.0f;  // 최대 거리 오차 (안전)
    static constexpr float APPROACH_SPEED = 1.0f;      // 접근 속도 (m/s)
};

#endif // DISTANCE_ADJUSTER_H
