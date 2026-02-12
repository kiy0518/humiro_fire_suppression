/**
 * @file state_handler.h
 * @brief 상태 핸들러 베이스 클래스
 *
 * 각 미션 상태(PREPARE, ARM, TAKEOFF 등)를 독립적으로 구현하는 인터페이스.
 * - tick(): 10Hz 호출, 전환 조건 확인
 * - fillSetpoint(): TrajectorySetpoint 생성
 * - onEnter()/onExit(): 상태 진입/퇴출 시 1회 실행
 */

#ifndef STATE_HANDLER_H
#define STATE_HANDLER_H

#include <px4_msgs/msg/trajectory_setpoint.hpp>

// Forward declaration
struct MissionContext;

/**
 * tick() 반환값: 현재 상태에서의 전환 결정
 */
enum class TransitionResult {
    STAY,       // 현재 상태 유지
    COMPLETE,   // 핸들러 완료 → 다음 상태로 진행
    ABORT_RTL,  // 긴급 상황 → RTL로 전환
    ERROR       // 복구 불가 에러
};

/**
 * 모든 상태 핸들러의 베이스 클래스
 */
class StateHandler {
public:
    virtual ~StateHandler() = default;

    /** 상태 진입 시 1회 호출 */
    virtual void onEnter(MissionContext& ctx) = 0;

    /** 상태 퇴출 시 1회 호출 */
    virtual void onExit(MissionContext& ctx) = 0;

    /** 10Hz 호출. 전환 조건 확인 및 TransitionResult 반환 */
    virtual TransitionResult tick(MissionContext& ctx) = 0;

    /**
     * TrajectorySetpoint 생성.
     * @return true면 setpoint 발행, false면 발행 안 함 (RTL 등)
     */
    virtual bool fillSetpoint(MissionContext& ctx,
                              px4_msgs::msg::TrajectorySetpoint& sp) = 0;

    /** 핸들러 이름 (로깅용) */
    virtual const char* name() const = 0;
};

#endif // STATE_HANDLER_H
