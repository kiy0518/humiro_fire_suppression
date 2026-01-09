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
     * @brief 현재 비행 모드 확인 (nav_state)
     * @return nav_state 값 (14 = OFFBOARD)
     */
    uint8_t getNavState() const { return nav_state_.load(std::memory_order_acquire); }
    
    /**
     * @brief OFFBOARD 모드인지 확인
     * @return true: OFFBOARD 모드, false: 다른 모드
     */
    bool isOffboardMode() const { return nav_state_.load(std::memory_order_acquire) == 14; }
    
    /**
     * @brief OFFBOARD 모드 활성화
     * @return true: 성공, false: 실패
     */
    bool enableOffboardMode();
    
    /**
     * @brief OFFBOARD 모드 비활성화 (heartbeat 중단)
     * @brief 중요: heartbeat 중단 시 PX4가 자동으로 다른 모드(MANUAL 등)로 전환됨
     * @brief 위급 상황 시 다른 모드로 전환 가능하도록 heartbeat 중단 필요
     * @brief 호출 즉시 heartbeat가 중단되므로 언제든지 호출 가능 (임무 중간에도 가능)
     * @return true: 성공, false: 실패
     */
    bool disableOffboardMode();
    
    /**
     * @brief OFFBOARD heartbeat가 활성화되어 있는지 확인
     * @return true: heartbeat 발행 중, false: heartbeat 중단됨
     */
    bool isOffboardHeartbeatActive() const { return offboard_timer_ != nullptr; }

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
    
    // FC 시스템 ID (DRONE_ID 환경 변수에서 읽음)
    uint8_t target_system_;
    
    // Offboard heartbeat timer
    rclcpp::TimerBase::SharedPtr offboard_timer_;
};

#endif // ARM_HANDLER_H
