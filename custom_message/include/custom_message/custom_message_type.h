/**
 * @file custom_message_type.h
 * @brief 커스텀 MAVLink 메시지 타입 정의
 * 
 * QGC와 VIM4 간 화재 진압 미션 전용 커스텀 MAVLink 메시지 타입 정의
 * 
 * @author Humiro Fire Suppression Team
 * @date 2026-01-03
 */

#ifndef CUSTOM_MESSAGE_TYPE_H
#define CUSTOM_MESSAGE_TYPE_H

#include <cstdint>

namespace custom_message {

/**
 * @brief 화재 진압 미션 단계 열거형
 */
enum class FireMissionPhase : uint8_t {
    FIRE_PHASE_IDLE = 0,           // 미션 시작 대기
    FIRE_PHASE_NAVIGATING = 1,     // 목표로 이동 중
    FIRE_PHASE_SCANNING = 2,       // 열화상 스캔 중
    FIRE_PHASE_READY_TO_FIRE = 3,  // 발사 준비 완료
    FIRE_PHASE_SUPPRESSING = 4,    // 화재 진압 중
    FIRE_PHASE_VERIFYING = 5,      // 효과 검증 중
    FIRE_PHASE_COMPLETE = 6         // 미션 완료
};

/**
 * @brief 발사 제어 명령 열거형
 */
enum class FireLaunchCommand : uint8_t {
    FIRE_LAUNCH_CONFIRM = 0,        // 발사 확인
    FIRE_LAUNCH_ABORT = 1,         // 발사 중단
    FIRE_LAUNCH_REQUEST_STATUS = 2 // 상태 요청
};

/**
 * @brief 화재 진압 미션 시작 메시지 (FIRE_MISSION_START)
 * 
 * Message ID: 12900
 * QGC → VIM4
 * 
 * GO 버튼을 눌렀을 때 전송되는 미션 시작 명령
 */
struct FireMissionStart {
    uint8_t target_system;      // 시스템 ID
    uint8_t target_component;    // 컴포넌트 ID
    int32_t target_lat;         // 목표 위도 * 1e7
    int32_t target_lon;         // 목표 경도 * 1e7
    float target_alt;           // 목표 고도 MSL (m)
    uint8_t auto_fire;          // 0=수동, 1=자동
    uint8_t max_projectiles;    // 최대 발사 횟수
    uint8_t reserved[2];        // 예약 필드
};

/**
 * @brief 화재 진압 미션 상태 메시지 (FIRE_MISSION_STATUS)
 * 
 * Message ID: 12901
 * VIM4 → QGC
 * 
 * 미션 진행 상태를 주기적으로 전송
 */
struct FireMissionStatus {
    uint8_t phase;               // 현재 미션 단계 (0-6)
    uint8_t progress;            // 진행률 0-100%
    uint8_t remaining_projectiles; // 남은 발사 횟수
    float distance_to_target;    // 목표까지 거리 (m)
    int16_t thermal_max_temp;    // 최대 온도 (°C * 10)
    char status_text[50];        // 상태 메시지 (UTF-8)
};

/**
 * @brief 발사 제어 메시지 (FIRE_LAUNCH_CONTROL)
 * 
 * Message ID: 12902
 * QGC ↔ VIM4
 * 
 * 발사 확인, 중단, 상태 요청
 */
struct FireLaunchControl {
    uint8_t target_system;       // 시스템 ID
    uint8_t target_component;   // 컴포넌트 ID
    uint8_t command;            // 0=확인, 1=중단, 2=상태 요청
    uint8_t reserved[5];        // 예약 필드
};

/**
 * @brief 화재 진압 결과 메시지 (FIRE_SUPPRESSION_RESULT)
 * 
 * Message ID: 12903
 * VIM4 → QGC
 * 
 * 발사 후 결과 전송
 * (현재 단계에서는 온도 감소 확인 제외)
 */
struct FireSuppressionResult {
    uint8_t shot_number;         // 발사 번호
    uint8_t success;             // 0=실패, 1=성공 (발사 메커니즘 동작 여부)
    uint8_t reserved[6];        // 예약 필드
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
