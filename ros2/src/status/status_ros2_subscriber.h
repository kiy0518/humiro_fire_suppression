#ifndef STATUS_ROS2_SUBSCRIBER_H
#define STATUS_ROS2_SUBSCRIBER_H

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "../../../osd/src/status/status_overlay.h"
#include <memory>
#include <string>
#include <chrono>

// PX4 uXRCE-DDS 메시지 (px4_msgs) - 필수 사용
// px4_msgs 헤더 파일 include (경로 중복 문제로 인해 px4_msgs/px4_msgs/msg/ 경로 사용)
#include <px4_msgs/px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/px4_msgs/msg/sensor_gps.hpp>

/**
 * StatusOverlay를 위한 ROS2 토픽 구독자
 * PX4 상태, 배터리, GPS, 자동 제어 상태 등을 구독하여 StatusOverlay 업데이트
 */
class StatusROS2Subscriber {
public:
    StatusROS2Subscriber(rclcpp::Node::SharedPtr node, StatusOverlay* status_overlay);
    ~StatusROS2Subscriber();
    
    /**
     * ROS2 스핀 (주기적으로 호출)
     */
    void spin();
    
private:
    // PX4 상태 콜백 (uXRCE-DDS)
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    
    // 배터리 상태 콜백 (uXRCE-DDS)
    void batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    
    // GPS 상태 콜백 (uXRCE-DDS)
    void gpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg);
    
    // 자동 제어 상태 콜백 (VIM4 커스텀 토픽)
    void autoControlStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    
    // 소화탄 갯수 콜백 (커스텀 토픽)
    void ammunitionCallback(const std_msgs::msg::Int32::SharedPtr msg);
    
    // 편대 정보 콜백 (커스텀 토픽)
    void formationCallback(const std_msgs::msg::Int32::SharedPtr msg);
    
    // PX4 nav_state를 모드 문자열로 변환
    std::string navStateToModeString(uint8_t nav_state);
    
    rclcpp::Node::SharedPtr node_;
    StatusOverlay* status_overlay_;
    
    // ROS2 구독자 (uXRCE-DDS 토픽)
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr auto_control_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ammunition_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr formation_sub_;
    
    // 마지막 업데이트 시간 (디버깅용)
    std::chrono::steady_clock::time_point last_state_update_;
    std::chrono::steady_clock::time_point last_battery_update_;
    std::chrono::steady_clock::time_point last_gps_update_;
};

#endif // STATUS_ROS2_SUBSCRIBER_H

#endif // ENABLE_ROS2

