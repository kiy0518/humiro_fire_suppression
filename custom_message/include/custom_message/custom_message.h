/**
 * @file custom_message.h
 * @brief 커스텀 MAVLink 메시지 송수신 라이브러리
 *
 * QGC와 VIM4 간 화재 진압 미션 전용 커스텀 MAVLink 메시지를 송수신하는 라이브러리
 *
 * 메시지 ID (60000번대 사용 - 50001/50002는 CUBEPILOT/HERELINK와 충돌):
 * - 60000: FIRE_MISSION_START (미션 스타트)
 * - 60001: FIRE_AUTO_AIM (자동조준)
 * - 60002: FIRE_LAUNCH (발사)
 * - 60003: FIRE_RETURN (복귀)
 *
 * @author Humiro Fire Suppression Team
 * @date 2026-01-26
 * @version 4.0 - 60000번대로 MSG ID 변경
 */

#ifndef CUSTOM_MESSAGE_H
#define CUSTOM_MESSAGE_H

#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include "custom_message_type.h"

namespace custom_message {

// 전방 선언
class CustomMessageImpl;

/**
 * @brief 미션 시작 콜백 함수 타입 (60000)
 */
using FireMissionStartCallback = std::function<void(const FireMissionStart& start)>;

/**
 * @brief 자동조준 콜백 함수 타입 (60001)
 */
using FireAutoAimCallback = std::function<void(const FireAutoAim& aim)>;

/**
 * @brief 발사 콜백 함수 타입 (60002)
 */
using FireLaunchCallback = std::function<void(const FireLaunch& launch)>;

/**
 * @brief 복귀 콜백 함수 타입 (60003)
 */
using FireReturnCallback = std::function<void(const FireReturn& ret)>;

/**
 * @brief COMMAND_LONG 콜백 함수 타입
 */
using CommandLongCallback = std::function<void(uint8_t target_system, uint8_t target_component, uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)>;

/**
 * @brief SET_MODE 콜백 함수 타입
 */
using SetModeCallback = std::function<void(uint8_t target_system, uint8_t base_mode, uint32_t custom_mode)>;

/**
 * @brief 고도 정보 콜백 (GLOBAL_POSITION_INT에서 추출, QGC alt(rel)과 동일)
 * @param relative_alt_m 홈 기준 상대 고도 (m)
 * @param alt_amsl_m AMSL 절대 고도 (m)
 */
using AltitudeCallback = std::function<void(float relative_alt_m, float alt_amsl_m)>;

/**
 * @brief 하방 거리센서 콜백 (DISTANCE_SENSOR에서 추출)
 * @param distance_m 거리 (m)
 */
using DistanceSensorCallback = std::function<void(float distance_m)>;

/**
 * @brief 커스텀 MAVLink 메시지 송수신기
 */
class CustomMessage {
public:
    /**
     * @brief 생성자
     */
    CustomMessage(
        uint16_t receive_port = 14550,
        uint16_t send_port = 14550,
        const std::string& bind_address = "0.0.0.0",
        const std::string& target_address = "127.0.0.1",
        uint8_t system_id = 1,
        uint8_t component_id = 1
    );

    ~CustomMessage();

    CustomMessage(const CustomMessage&) = delete;
    CustomMessage& operator=(const CustomMessage&) = delete;
    CustomMessage(CustomMessage&&) = delete;
    CustomMessage& operator=(CustomMessage&&) = delete;

    bool start();
    void stop();
    bool isRunning() const;

    // ========== 수신 콜백 등록 ==========
    void setFireMissionStartCallback(FireMissionStartCallback callback);
    void setFireAutoAimCallback(FireAutoAimCallback callback);
    void setFireLaunchCallback(FireLaunchCallback callback);
    void setFireReturnCallback(FireReturnCallback callback);
    void setCommandLongCallback(CommandLongCallback callback);
    void setSetModeCallback(SetModeCallback callback);
    void setAltitudeCallback(AltitudeCallback callback);
    void setDistanceSensorCallback(DistanceSensorCallback callback);

    // ========== 송신 함수 ==========
    bool sendFireMissionStart(const FireMissionStart& start);
    bool sendFireAutoAim(const FireAutoAim& aim);
    bool sendFireLaunch(const FireLaunch& launch);
    bool sendFireReturn(const FireReturn& ret);

    void setTargetAddress(const std::string& address, uint16_t port);

    // ========== 통계 정보 ==========
    struct Statistics {
        uint64_t messages_received = 0;
        uint64_t mission_start_received = 0;
        uint64_t auto_aim_received = 0;
        uint64_t launch_received = 0;
        uint64_t return_received = 0;
        uint64_t mission_start_sent = 0;
        uint64_t auto_aim_sent = 0;
        uint64_t launch_sent = 0;
        uint64_t return_sent = 0;
        uint64_t unknown_message_count = 0;
        uint64_t parse_error_count = 0;
        uint64_t send_error_count = 0;
        uint64_t queue_drop_count = 0;
    };

    Statistics getStatistics() const;
    void resetStatistics();

private:
    std::unique_ptr<CustomMessageImpl> impl_;
};

} // namespace custom_message

#endif // CUSTOM_MESSAGE_H
