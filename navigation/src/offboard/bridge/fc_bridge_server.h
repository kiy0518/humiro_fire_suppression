/**
 * @file fc_bridge_server.h
 * @brief FC Bridge 서버 (fc_bridge 프로세스 측)
 *
 * ROS2 Domain 0에서 /fmu/* 토픽을 구독/발행하고,
 * 로컬 UDP IPC로 메인앱과 통신.
 */

#ifndef FC_BRIDGE_SERVER_H
#define FC_BRIDGE_SERVER_H

#include "fc_bridge_protocol.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <atomic>
#include <mutex>
#include <thread>

class FCBridgeServer {
public:
    /**
     * @param node ROS2 노드 (Domain 0)
     * @param px4_ns PX4 토픽 네임스페이스 (예: "/drone1")
     * @param state_port FC 상태 전송 포트 (기본: 17001)
     * @param command_port FC 명령 수신 포트 (기본: 17002)
     */
    FCBridgeServer(rclcpp::Node::SharedPtr node,
                   const std::string& px4_ns,
                   uint16_t state_port = FC_BRIDGE_STATE_PORT,
                   uint16_t command_port = FC_BRIDGE_COMMAND_PORT);
    ~FCBridgeServer();

    /** UDP 소켓 및 ROS2 구독 시작 */
    bool start();

    /** 종료 */
    void stop();

private:
    // ROS2 콜백
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicleGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    void gpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg);

    // 50Hz 타이머: FCState → UDP 전송
    void sendStateTimer();

    // UDP 명령 수신 스레드
    void commandReceiveLoop();

    // 수신한 FCCommand → ROS2 토픽 발행
    void processCommand(const FCCommand& cmd);

    rclcpp::Node::SharedPtr node_;
    std::string px4_ns_;
    uint16_t state_port_;
    uint16_t command_port_;

    // ROS2 Subscribers (Domain 0)
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;

    // ROS2 Publishers (Domain 0)
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    // 50Hz 타이머
    rclcpp::TimerBase::SharedPtr state_timer_;

    // FC 상태 (콜백에서 업데이트, 타이머에서 전송)
    std::mutex state_mutex_;
    FCState current_state_{};
    std::atomic<bool> status_received_{false};
    std::atomic<bool> position_received_{false};

    // UDP 소켓
    int state_send_fd_ = -1;      // FCState 송신
    int command_recv_fd_ = -1;    // FCCommand 수신

    // 명령 수신 스레드
    std::thread command_thread_;
    std::atomic<bool> running_{false};
};

#endif // FC_BRIDGE_SERVER_H
