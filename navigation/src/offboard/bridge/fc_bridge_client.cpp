/**
 * @file fc_bridge_client.cpp
 * @brief FC Bridge 클라이언트 구현 (메인앱 측)
 */

#include "fc_bridge_client.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

FCBridgeClient::FCBridgeClient(uint16_t state_port, uint16_t command_port)
    : state_port_(state_port), command_port_(command_port)
{
}

FCBridgeClient::~FCBridgeClient()
{
    stop();
}

bool FCBridgeClient::start()
{
    if (running_.load()) return true;

    // 수신 소켓 생성 (fc_bridge → 메인앱)
    recv_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd_ < 0) {
        std::cerr << "[FCBridgeClient] Failed to create recv socket: " << strerror(errno) << std::endl;
        return false;
    }

    // SO_REUSEADDR
    int reuse = 1;
    setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // 수신 타임아웃 (100ms — 수신 루프 종료 검사용)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(recv_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 바인드
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(state_port_);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(recv_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[FCBridgeClient] Failed to bind recv socket to port "
                  << state_port_ << ": " << strerror(errno) << std::endl;
        close(recv_fd_);
        recv_fd_ = -1;
        return false;
    }

    // 송신 소켓 생성 (메인앱 → fc_bridge)
    send_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd_ < 0) {
        std::cerr << "[FCBridgeClient] Failed to create send socket: " << strerror(errno) << std::endl;
        close(recv_fd_);
        recv_fd_ = -1;
        return false;
    }

    running_.store(true);
    recv_thread_ = std::thread(&FCBridgeClient::receiveLoop, this);

    std::cout << "[FCBridgeClient] Started (state_port=" << state_port_
              << ", command_port=" << command_port_ << ")" << std::endl;
    return true;
}

void FCBridgeClient::stop()
{
    running_.store(false);

    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }

    if (recv_fd_ >= 0) {
        close(recv_fd_);
        recv_fd_ = -1;
    }
    if (send_fd_ >= 0) {
        close(send_fd_);
        send_fd_ = -1;
    }
}

FCState FCBridgeClient::getLatestState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

bool FCBridgeClient::isConnected() const
{
    if (!state_received_.load()) return false;

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_receive_time_).count();
    return elapsed < FC_BRIDGE_TIMEOUT_MS;
}

std::chrono::steady_clock::time_point FCBridgeClient::getLastReceiveTime() const
{
    return last_receive_time_;
}

bool FCBridgeClient::sendHeartbeat(bool position, bool velocity, bool acceleration)
{
    auto cmd = FCCommand::makeHeartbeat(position, velocity, acceleration);
    return sendCommand(cmd);
}

bool FCBridgeClient::sendSetpoint(const FCCommand& cmd)
{
    return sendCommand(cmd);
}

bool FCBridgeClient::sendVehicleCommand(uint16_t command, float param1, float param2,
                                         float param3, uint8_t target_system)
{
    auto cmd = FCCommand::makeVehicleCommand(command, param1, param2, param3, target_system);
    return sendCommand(cmd);
}

void FCBridgeClient::receiveLoop()
{
    uint8_t buffer[512];

    while (running_.load()) {
        ssize_t n = recv(recv_fd_, buffer, sizeof(buffer), 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 타임아웃 — 연결 끊김 감지
                bool now_connected = isConnected();
                if (was_connected_.load() && !now_connected) {
                    std::cerr << "[FCBridgeClient] Connection lost (no data for "
                              << FC_BRIDGE_TIMEOUT_MS << "ms)" << std::endl;
                    if (connection_callback_) {
                        connection_callback_(false);
                    }
                    was_connected_.store(false);
                }
                continue;
            }
            // 실제 에러
            if (running_.load()) {
                std::cerr << "[FCBridgeClient] recv error: " << strerror(errno) << std::endl;
            }
            continue;
        }

        if (n < static_cast<ssize_t>(sizeof(FCState))) continue;

        const FCState* state = reinterpret_cast<const FCState*>(buffer);
        if (!state->isValid()) continue;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = *state;
        }

        last_receive_time_ = std::chrono::steady_clock::now();
        state_received_.store(true);

        // 연결 복구 감지
        if (!was_connected_.load()) {
            std::cout << "[FCBridgeClient] Connected to fc_bridge" << std::endl;
            if (connection_callback_) {
                connection_callback_(true);
            }
            was_connected_.store(true);
        }
    }
}

bool FCBridgeClient::sendCommand(const FCCommand& cmd)
{
    if (send_fd_ < 0) return false;

    struct sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(command_port_);
    dest.sin_addr.s_addr = inet_addr("127.0.0.1");

    ssize_t sent = sendto(send_fd_, &cmd, sizeof(cmd), 0,
                          (struct sockaddr*)&dest, sizeof(dest));
    return sent == sizeof(cmd);
}
