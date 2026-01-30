/**
 * @file custom_message_type.h
 * @brief 커스텀 MAVLink 메시지 타입 정의 (Wire Format 순서)
 *
 * QGC와 VIM4 간 화재 진압 미션 전용 커스텀 MAVLink 메시지 타입 정의
 *
 * ★ 중요: MAVLink Wire Format 순서 (타입 크기 내림차순)
 * - 4바이트 필드 (int32_t, float) 먼저
 * - 2바이트 필드 (int16_t) 다음
 * - 1바이트 필드 (uint8_t) 마지막
 *
 * @author Humiro Fire Suppression Team
 * @date 2026-01-27
 * @version 4.0 - Wire Format 순서로 수정
 */

#ifndef CUSTOM_MESSAGE_TYPE_H
#define CUSTOM_MESSAGE_TYPE_H

#include <cstdint>

namespace custom_message {

/**
 * @brief 화재 진압 미션 단계 열거형 (FIRE_MISSION_PHASE)
 * XML enum 정의와 완전 동일
 */
enum class FireMissionPhase : uint8_t {
    FIRE_PHASE_IDLE = 0,            // Waiting for mission start
    FIRE_PHASE_NAVIGATING = 1,      // Flying to target location
    FIRE_PHASE_SCANNING = 2,        // Scanning with thermal camera
    FIRE_PHASE_READY_TO_FIRE = 3,   // Ready to launch projectile
    FIRE_PHASE_SUPPRESSING = 4,     // Active fire suppression
    FIRE_PHASE_VERIFYING = 5,       // Verifying suppression result
    FIRE_PHASE_COMPLETE = 6         // Mission complete
};

/**
 * @brief 발사 제어 명령 열거형
 * FIRE_LAUNCH_CONTROL 메시지의 command 필드 값
 */
enum class LaunchCommand : uint8_t {
    LAUNCH_CMD_CONFIRM = 0,         // Confirm launch
    LAUNCH_CMD_ABORT = 1,           // Abort launch
    LAUNCH_CMD_REQUEST_STATUS = 2   // Request current status
};

/**
 * @brief PX4 비행 모드 열거형
 * FIRE_SET_MODE 메시지의 px4_mode 필드 값
 */
enum class PX4Mode : uint8_t {
    PX4_MODE_MANUAL = 1,       // Manual mode
    PX4_MODE_ALTCTL = 2,       // Altitude control
    PX4_MODE_POSCTL = 3,       // Position control
    PX4_MODE_AUTO = 4,         // Auto (mission, loiter, RTL 등)
    PX4_MODE_ACRO = 5,         // Acro mode
    PX4_MODE_OFFBOARD = 6,     // Offboard mode
    PX4_MODE_STABILIZED = 7,   // Stabilized mode
    PX4_MODE_RATTITUDE = 8     // Rattitude mode
};

/**
 * @brief 화재 진압 미션 시작 메시지 (FIRE_MISSION_START)
 *
 * Message ID: 60000
 * Direction: QGC → VIM4
 *
 * ★ Wire Format 순서 (타입 크기 내림차순)
 */
struct __attribute__((packed)) FireMissionStart {
    // 4-byte fields first (Wire Format 순서)
    int32_t target_lat;           // Target latitude * 1e7
    int32_t target_lon;           // Target longitude * 1e7
    float target_alt;             // Target altitude (m)
    float takeoff_speed;          // Takeoff speed (m/s)
    float flight_speed;           // Flight speed (m/s)
    // 1-byte fields last
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
    uint8_t auto_fire;            // 0=manual, 1=auto
    uint8_t max_projectiles;      // Max projectiles to use
};

/**
 * @brief 자동 조준 메시지 (AUTO_AIM)
 *
 * Message ID: 60001
 * Direction: QGC → VIM4
 *
 * 자동 조준 명령 (1바이트 필드만 있어 순서 무관)
 */
struct __attribute__((packed)) FireAutoAim {
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
};

/**
 * @brief 발사 메시지 (FIRE_COMMAND)
 *
 * Message ID: 60002
 * Direction: QGC → VIM4
 *
 * 발사 명령 (1바이트 필드만 있어 순서 무관)
 */
struct __attribute__((packed)) FireLaunch {
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
};

/**
 * @brief 복귀 메시지 (RETURN_TO_LAUNCH)
 *
 * Message ID: 60003
 * Direction: QGC → VIM4
 *
 * 복귀(RTL) 명령 (1바이트 필드만 있어 순서 무관)
 */
struct __attribute__((packed)) FireReturn {
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
};

/**
 * @brief 미션 상태 메시지 (FIRE_MISSION_STATUS)
 *
 * Message ID: 60010
 * Direction: VIM4 → QGC
 *
 * ★ Wire Format 순서 (타입 크기 내림차순)
 */
struct __attribute__((packed)) FireMissionStatus {
    // 4-byte fields first
    int32_t target_lat;           // Target latitude * 1e7
    int32_t target_lon;           // Target longitude * 1e7
    float target_alt;             // Target altitude (m)
    float distance_to_target;     // Distance to target (m)
    // 2-byte fields
    int16_t thermal_max_temp;     // Max temperature * 10 (0.1 C)
    // 1-byte fields last
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
    uint8_t phase;                // Current mission phase
    uint8_t progress;             // Progress percentage (0-100)
    uint8_t projectiles_used;     // Projectiles used
    uint8_t remaining_projectiles;// Projectiles remaining
};

/**
 * @brief 진압 결과 메시지 (FIRE_SUPPRESSION_RESULT)
 *
 * Message ID: 60011
 * Direction: VIM4 → QGC
 *
 * ★ Wire Format 순서 (타입 크기 내림차순)
 */
struct __attribute__((packed)) FireSuppressionResult {
    // 4-byte fields first
    int32_t impact_lat;           // Impact latitude * 1e7
    int32_t impact_lon;           // Impact longitude * 1e7
    float thermal_reduction;      // Thermal signature reduction (%)
    // 2-byte fields
    int16_t temp_before;          // Temperature before * 10 (0.1 C)
    int16_t temp_after;           // Temperature after * 10 (0.1 C)
    // 1-byte fields last
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
    uint8_t shot_number;          // Shot number
    uint8_t success;              // 0=failed, 1=success
};

/**
 * @brief 메시지 타입 열거형
 *
 * NOTE: 60000번대 사용 (50001/50002는 CUBEPILOT_RAW_RC, HERELINK_VIDEO_STREAM_INFORMATION과 충돌)
 */
enum class MessageType {
    // QGC → VIM4 (명령)
    FIRE_MISSION_START = 60000,      // 미션 스타트
    FIRE_AUTO_AIM = 60001,           // 자동 조준
    FIRE_LAUNCH = 60002,             // 발사
    FIRE_RETURN = 60003,             // 복귀
    // VIM4 → QGC (상태)
    FIRE_MISSION_STATUS = 60010,     // 미션 상태
    FIRE_SUPPRESSION_RESULT = 60011, // 진압 결과
    UNKNOWN = 0
};

} // namespace custom_message

#endif // CUSTOM_MESSAGE_TYPE_H
