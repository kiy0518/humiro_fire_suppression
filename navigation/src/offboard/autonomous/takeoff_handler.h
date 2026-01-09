#ifndef TAKEOFF_HANDLER_H
#define TAKEOFF_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <atomic>
#include <chrono>
#include <memory>

class TakeoffHandler {
public:
    explicit TakeoffHandler(rclcpp::Node::SharedPtr node);

    /**
     * @brief 지정된 고도로 이륙
     * @param altitude_m 목표 고도 (미터, 기본값 5.0m)
     * @param timeout_ms 타임아웃 (밀리초, 기본값 30초)
     * @return 이륙 성공 여부
     */
    bool takeoff(float altitude_m = 5.0f, int timeout_ms = 30000);

    /**
     * @brief 현재 고도 반환
     * @return 현재 고도 (미터, NED 좌표계)
     */
    float getCurrentAltitude() const;

    /**
     * @brief 이륙 완료 여부 확인
     * @return 목표 고도 도달 여부
     */
    bool isTakeoffComplete() const;

    /**
     * @brief 현재 위치에서 호버링
     */
    void hover();

private:
    /**
     * @brief VehicleLocalPosition 콜백
     */
    void vehicleLocalPositionCallback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    /**
     * @brief TrajectorySetpoint 발행 (OFFBOARD 제어)
     */
    void publishTrajectorySetpoint(float x, float y, float z, float yaw);

    /**
     * @brief OffboardControlMode 발행
     */
    void publishOffboardControlMode();

    /**
     * @brief Vehicle Command 발행
     */
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f,
                               float param2 = 0.0f, float param3 = 0.0f,
                               float param4 = 0.0f, float param5 = 0.0f,
                               float param6 = 0.0f, float param7 = 0.0f);

    rclcpp::Node::SharedPtr node_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    // Timer for OFFBOARD mode heartbeat
    rclcpp::TimerBase::SharedPtr offboard_timer_;

    // State variables
    std::atomic<float> current_altitude_{0.0f};  // NED Z (아래 방향이 양수)
    std::atomic<float> current_x_{0.0f};
    std::atomic<float> current_y_{0.0f};
    std::atomic<float> current_yaw_{0.0f};
    std::atomic<bool> position_received_{false};

    // Takeoff parameters
    float target_altitude_{0.0f};
    float takeoff_start_altitude_{0.0f};

    // FC 시스템 ID (DRONE_ID 환경 변수에서 읽음)
    uint8_t target_system_;

    // Constants
    static constexpr float ALTITUDE_THRESHOLD = 0.3f;  // 30cm 오차 허용
    static constexpr uint16_t VEHICLE_CMD_NAV_TAKEOFF = 22;
};

#endif // TAKEOFF_HANDLER_H
