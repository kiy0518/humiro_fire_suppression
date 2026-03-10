/**
 * @file fc_bridge_server.cpp
 * @brief FC Bridge 서버 구현 (fc_bridge 프로세스 측)
 */

#include "fc_bridge_server.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;

FCBridgeServer::FCBridgeServer(rclcpp::Node::SharedPtr node,
                               const std::string& px4_ns,
                               uint16_t state_port,
                               uint16_t command_port)
    : node_(node), px4_ns_(px4_ns), state_port_(state_port), command_port_(command_port)
{
}

FCBridgeServer::~FCBridgeServer()
{
    stop();
}

bool FCBridgeServer::start()
{
    if (running_.load()) return true;

    // PX4 호환 QoS (sensor_data: BEST_EFFORT + VOLATILE)
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    // ========== ROS2 Subscribers (Domain 0, /droneN/fmu/out/*) ==========
    vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        px4_ns_ + "/fmu/out/vehicle_status_v1", qos,
        std::bind(&FCBridgeServer::vehicleStatusCallback, this, std::placeholders::_1));

    local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        px4_ns_ + "/fmu/out/vehicle_local_position", qos,
        std::bind(&FCBridgeServer::vehicleLocalPositionCallback, this, std::placeholders::_1));

    global_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        px4_ns_ + "/fmu/out/vehicle_global_position", qos,
        std::bind(&FCBridgeServer::vehicleGlobalPositionCallback, this, std::placeholders::_1));

    battery_sub_ = node_->create_subscription<px4_msgs::msg::BatteryStatus>(
        px4_ns_ + "/fmu/out/battery_status", qos,
        std::bind(&FCBridgeServer::batteryCallback, this, std::placeholders::_1));

    gps_sub_ = node_->create_subscription<px4_msgs::msg::SensorGps>(
        px4_ns_ + "/fmu/out/vehicle_gps_position", qos,
        std::bind(&FCBridgeServer::gpsCallback, this, std::placeholders::_1));

    // ========== ROS2 Publishers (Domain 0, /droneN/fmu/in/*) ==========
    offboard_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_ns_ + "/fmu/in/offboard_control_mode", qos);

    setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_ns_ + "/fmu/in/trajectory_setpoint", qos);

    vehicle_cmd_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        px4_ns_ + "/fmu/in/vehicle_command", qos);

    // ========== UDP 소켓 ==========

    // FCState 송신 소켓
    state_send_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (state_send_fd_ < 0) {
        RCLCPP_ERROR(node_->get_logger(), "[FCBridgeServer] Failed to create state socket: %s",
                     strerror(errno));
        return false;
    }

    // FCCommand 수신 소켓
    command_recv_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (command_recv_fd_ < 0) {
        RCLCPP_ERROR(node_->get_logger(), "[FCBridgeServer] Failed to create command socket: %s",
                     strerror(errno));
        close(state_send_fd_);
        state_send_fd_ = -1;
        return false;
    }

    int reuse = 1;
    setsockopt(command_recv_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // 수신 타임아웃 (100ms)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(command_recv_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 바인드
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(command_port_);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(command_recv_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "[FCBridgeServer] Failed to bind command port %d: %s",
                     command_port_, strerror(errno));
        close(state_send_fd_);
        close(command_recv_fd_);
        state_send_fd_ = -1;
        command_recv_fd_ = -1;
        return false;
    }

    // ========== 50Hz 타이머 (FCState 전송) ==========
    state_timer_ = node_->create_wall_timer(
        20ms, std::bind(&FCBridgeServer::sendStateTimer, this));

    // ========== 명령 수신 스레드 ==========
    running_.store(true);
    command_thread_ = std::thread(&FCBridgeServer::commandReceiveLoop, this);

    RCLCPP_INFO(node_->get_logger(),
        "[FCBridgeServer] Started (ns=%s, state_port=%d, command_port=%d)",
        px4_ns_.c_str(), state_port_, command_port_);

    return true;
}

void FCBridgeServer::stop()
{
    running_.store(false);

    if (state_timer_) {
        state_timer_->cancel();
    }

    if (command_thread_.joinable()) {
        command_thread_.join();
    }

    if (state_send_fd_ >= 0) { close(state_send_fd_); state_send_fd_ = -1; }
    if (command_recv_fd_ >= 0) { close(command_recv_fd_); command_recv_fd_ = -1; }
}

// ========== ROS2 콜백 ==========

void FCBridgeServer::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.nav_state = msg->nav_state;
    current_state_.arming_state = msg->arming_state;
    current_state_.fc_connected = 1;
    status_received_.store(true);
}

void FCBridgeServer::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.local_x = msg->x;
    current_state_.local_y = msg->y;
    current_state_.local_z = msg->z;
    current_state_.vx = msg->vx;
    current_state_.vy = msg->vy;
    current_state_.vz = msg->vz;
    current_state_.yaw = msg->heading;
    current_state_.dist_bottom = msg->dist_bottom_valid ? msg->dist_bottom : -1.0f;
    current_state_.dist_bottom_valid = msg->dist_bottom_valid ? 1 : 0;
    position_received_.store(true);
}

void FCBridgeServer::vehicleGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.latitude = msg->lat;
    current_state_.longitude = msg->lon;
    current_state_.altitude_amsl = msg->alt;
}

void FCBridgeServer::batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.battery_voltage = msg->voltage_v;
    current_state_.battery_remaining = msg->remaining;
}

void FCBridgeServer::gpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.gps_fix_type = msg->fix_type;
    current_state_.gps_satellites = msg->satellites_used;
    current_state_.gps_hdop = msg->hdop;
}

// ========== 50Hz 타이머 ==========

void FCBridgeServer::sendStateTimer()
{
    if (state_send_fd_ < 0) return;

    FCState state_copy;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_copy = current_state_;
    }

    // 타임스탬프 설정
    state_copy.timestamp_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    // FC 연결 상태 (status 수신 여부)
    state_copy.fc_connected = status_received_.load() ? 1 : 0;

    // UDP 전송
    struct sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(state_port_);
    dest.sin_addr.s_addr = inet_addr("127.0.0.1");

    sendto(state_send_fd_, &state_copy, sizeof(state_copy), 0,
           (struct sockaddr*)&dest, sizeof(dest));
}

// ========== 명령 수신 ==========

void FCBridgeServer::commandReceiveLoop()
{
    uint8_t buffer[512];

    while (running_.load()) {
        ssize_t n = recv(command_recv_fd_, buffer, sizeof(buffer), 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (running_.load()) {
                RCLCPP_WARN(node_->get_logger(), "[FCBridgeServer] Command recv error: %s",
                            strerror(errno));
            }
            continue;
        }

        if (n < static_cast<ssize_t>(sizeof(FCCommand))) continue;

        const FCCommand* cmd = reinterpret_cast<const FCCommand*>(buffer);
        if (!cmd->isValid()) continue;

        processCommand(*cmd);
    }
}

void FCBridgeServer::processCommand(const FCCommand& cmd)
{
    auto type = static_cast<FCCommandType>(cmd.command_type);
    uint64_t ts = node_->get_clock()->now().nanoseconds() / 1000;

    switch (type) {
        case FCCommandType::HEARTBEAT: {
            px4_msgs::msg::OffboardControlMode msg{};
            msg.timestamp = ts;
            msg.position = cmd.position;
            msg.velocity = cmd.velocity;
            msg.acceleration = cmd.acceleration;
            msg.attitude = false;
            msg.body_rate = false;
            offboard_mode_pub_->publish(msg);
            break;
        }

        case FCCommandType::SETPOINT: {
            px4_msgs::msg::TrajectorySetpoint msg{};
            msg.timestamp = ts;
            msg.position = {cmd.x, cmd.y, cmd.z};
            msg.velocity = {cmd.vx, cmd.vy, cmd.vz};
            msg.yaw = cmd.yaw;
            msg.yawspeed = cmd.yaw_speed;
            setpoint_pub_->publish(msg);
            break;
        }

        case FCCommandType::VEHICLE_CMD: {
            px4_msgs::msg::VehicleCommand msg{};
            msg.timestamp = ts;
            msg.command = cmd.command;
            msg.param1 = cmd.param1;
            msg.param2 = cmd.param2;
            msg.param3 = cmd.param3;
            msg.target_system = cmd.target_system;
            msg.target_component = cmd.target_component;
            msg.source_system = cmd.source_system;
            msg.source_component = 1;
            msg.from_external = true;
            vehicle_cmd_pub_->publish(msg);
            break;
        }
    }
}
