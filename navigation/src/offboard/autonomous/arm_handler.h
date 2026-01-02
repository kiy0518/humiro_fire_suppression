#ifndef ARM_HANDLER_H
#define ARM_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <chrono>
#include <atomic>

/**
 * @brief ARM 핸들러 - 드론 시동 제어
 * 
 * 기능:
 * - OFFBOARD 모드 진입
 * - ARM 명령 전송
 * - 시동 상태 확인
 * - 타임아웃 처리
 */
class ArmHandler {
public:
    /**
     * @brief 생성자
     * @param node ROS2 노드
     */
    explicit ArmHandler(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief 소멸자
     */
    ~ArmHandler();
    
    /**
     * @brief 시동 걸기 (ARM)
     * @param timeout_ms 타임아웃 (밀리초, 기본 5000ms)
     * @return true: 성공, false: 실패
     */
    bool arm(int timeout_ms = 5000);
    
    /**
     * @brief 시동 끄기 (DISARM)
     * @param timeout_ms 타임아웃 (밀리초, 기본 3000ms)
     * @return true: 성공, false: 실패
     */
    bool disarm(int timeout_ms = 3000);
    
    /**
     * @brief 현재 시동 상태 확인
     * @return true: 시동 ON, false: 시동 OFF
     */
    bool isArmed() const;
    
    /**
     * @brief OFFBOARD 모드 활성화
     * @return true: 성공, false: 실패
     */
    bool enableOffboardMode();

private:
    /**
     * @brief Vehicle Status 콜백
     */
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    
    /**
     * @brief Vehicle Command 발행
     * @param command 명령 코드
     * @param param1 파라미터 1
     * @param param2 파라미터 2
     */
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    
    /**
     * @brief Offboard Control Mode 발행 (heartbeat)
     */
    void publishOffboardControlMode();
    
    // ROS2 노드
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    
    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    // 상태 변수
    std::atomic<bool> is_armed_;
    std::atomic<uint8_t> nav_state_;
    std::atomic<uint8_t> arming_state_;
    
    // Offboard heartbeat timer
    rclcpp::TimerBase::SharedPtr offboard_timer_;
};

#endif // ARM_HANDLER_H
