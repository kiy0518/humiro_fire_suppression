/**
 * @file custom_message_type.h
 * @brief 커스텀 MAVLink 메시지 타입 정의 (XML 정의 기반)
 *
 * QGC와 VIM4 간 화재 진압 미션 전용 커스텀 MAVLink 메시지 타입 정의
 * XML mavlink 정의 파일과 완전 동기화
 *
 * @author Humiro Fire Suppression Team
 * @date 2026-01-04
 * @version 3.0 - XML 정의 완전 동기화
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
 * @brief 화재 진압 미션 시작 메시지 (FIRE_MISSION_START)
 *
 * Message ID: 12900
 * Direction: QGC → VIM4
 *
 * GO 버튼을 누를 때 전송되는 미션 시작 명령
 */
struct FireMissionStart {
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
    int32_t target_lat;           // Target latitude * 1e7
    int32_t target_lon;           // Target longitude * 1e7
    float target_alt;             // Target altitude MSL (m)
    uint8_t auto_fire;            // 0=manual, 1=auto
    uint8_t max_projectiles;      // Max projectiles to use
};

/**
 * @brief 화재 진압 미션 상태 메시지 (FIRE_MISSION_STATUS)
 *
 * Message ID: 12901
 * Direction: VIM4 → QGC
 *
 * 주기적으로 전송되는 미션 상태 정보
 */
struct FireMissionStatus {
    uint8_t phase;                // Current mission phase (FIRE_MISSION_PHASE)
    uint8_t progress;             // Progress 0-100%
    uint8_t remaining_projectiles;// Projectiles left
    float distance_to_target;     // Distance to target (m)
    int16_t thermal_max_temp;     // Max temp (°C * 10)
    char status_text[50];         // Status message
};

/**
 * @brief 발사 제어 메시지 (FIRE_LAUNCH_CONTROL)
 *
 * Message ID: 12902
 * Direction: QGC ↔ VIM4 (양방향)
 *
 * 발사 확인, 중단, 상태 요청 등의 제어 명령
 */
struct FireLaunchControl {
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
    uint8_t command;              // 0=confirm, 1=abort, 2=request_status
};

/**
 * @brief 화재 진압 결과 메시지 (FIRE_SUPPRESSION_RESULT)
 *
 * Message ID: 12903
 * Direction: VIM4 → QGC
 *
 * 발사 후 결과 정보
 */
struct FireSuppressionResult {
    uint8_t shot_number;          // Shot number
    int16_t temp_before;          // Temp before (°C * 10)
    int16_t temp_after;           // Temp after (°C * 10)
    uint8_t success;              // 0=failed, 1=success
};

/**
 * @brief 메시지 타입 열거형
 */
enum class MessageType {
    FIRE_MISSION_START = 12900,      // QGC → VIM4
    FIRE_MISSION_STATUS = 12901,     // VIM4 → QGC
    FIRE_LAUNCH_CONTROL = 12902,     // QGC ↔ VIM4
    FIRE_SUPPRESSION_RESULT = 12903, // VIM4 → QGC
    UNKNOWN = 0
};

} // namespace custom_message

#endif // CUSTOM_MESSAGE_TYPE_H
