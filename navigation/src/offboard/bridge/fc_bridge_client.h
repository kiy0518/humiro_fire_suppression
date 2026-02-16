/**
 * @file fc_bridge_client.h
 * @brief FC Bridge 클라이언트 (메인앱 측)
 *
 * fc_bridge 프로세스로부터 FC 상태를 수신하고,
 * FC 명령(heartbeat, setpoint, vehicle_command)을 전송.
 * 로컬 UDP 127.0.0.1 통신으로 DDS와 완전 격리.
 */

#ifndef FC_BRIDGE_CLIENT_H
#define FC_BRIDGE_CLIENT_H

#include "fc_bridge_protocol.h"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <functional>

class FCBridgeClient {
public:
    /**
     * @param state_port fc_bridge → 메인앱 수신 포트 (기본: 17001)
     * @param command_port 메인앱 → fc_bridge 송신 포트 (기본: 17002)
     */
    explicit FCBridgeClient(uint16_t state_port = FC_BRIDGE_STATE_PORT,
                            uint16_t command_port = FC_BRIDGE_COMMAND_PORT);
    ~FCBridgeClient();

    /** 수신 스레드 시작 */
    bool start();

    /** 수신 스레드 종료 */
    void stop();

    // ========== FC 상태 읽기 ==========

    /** 최신 FC 상태 반환 (복사, thread-safe) */
    FCState getLatestState() const;

    /** FC 연결 여부 (최근 TIMEOUT_MS 이내 수신) */
    bool isConnected() const;

    /** 최근 상태 수신 시각 */
    std::chrono::steady_clock::time_point getLastReceiveTime() const;

    // ========== FC 명령 전송 ==========

    /** OffboardControlMode heartbeat 전송 */
    bool sendHeartbeat(bool position = true, bool velocity = false, bool acceleration = false);

    /** TrajectorySetpoint 전송 (raw FCCommand) */
    bool sendSetpoint(const FCCommand& cmd);

    /** VehicleCommand 전송 */
    bool sendVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f,
                            float param3 = 0.0f, uint8_t target_system = 1);

    // ========== 콜백 (옵션) ==========

    /** 연결 상태 변경 콜백 (connected→disconnected 또는 반대) */
    using ConnectionCallback = std::function<void(bool connected)>;
    void setConnectionCallback(ConnectionCallback cb) { connection_callback_ = cb; }

private:
    void receiveLoop();
    bool sendCommand(const FCCommand& cmd);

    uint16_t state_port_;
    uint16_t command_port_;
    int recv_fd_ = -1;
    int send_fd_ = -1;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex state_mutex_;
    FCState latest_state_{};
    std::atomic<bool> state_received_{false};
    std::chrono::steady_clock::time_point last_receive_time_{};

    ConnectionCallback connection_callback_;
    std::atomic<bool> was_connected_{false};
};

#endif // FC_BRIDGE_CLIENT_H
