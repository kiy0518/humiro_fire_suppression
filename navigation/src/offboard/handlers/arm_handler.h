/**
 * @file arm_handler.h
 * @brief ARM (시동) 핸들러
 *
 * OFFBOARD 모드 안정화 후 ARM 명령 전송.
 * PX4는 OFFBOARD 진입 직후 ARM을 받으면 모드 전환이 불안정할 수 있으므로
 * 최소 1.5초의 안정화 시간을 확보한 후 ARM을 전송한다.
 * (v0.16.2에서는 카운터 기반으로 OFFBOARD→ARM 간 2.5초 간격이 있었음)
 *
 * 전환 조건: arming_state == 2 → COMPLETE
 * 타임아웃: 15초 → ERROR
 */

#ifndef ARM_HANDLER_H
#define ARM_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class ArmHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        last_cmd_time_ = {};
        arm_sent_ = false;

        // ARM 전 시작 위치 1차 캡처
        if (ctx.position_received.load()) {
            ctx.start_local_x = ctx.current_local_x.load();
            ctx.start_local_y = ctx.current_local_y.load();
            ctx.start_local_z = ctx.current_local_z.load();
        }
        ctx.initial_yaw = ctx.current_yaw.load();

        RCLCPP_INFO(ctx.logger, "[ARM] OFFBOARD 안정화 대기 (%.1fs)...", OFFBOARD_STABILIZE_SEC);
    }

    void onExit(MissionContext& ctx) override {
        (void)ctx;
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        // OFFBOARD 안정화 대기: 최소 1.5초 동안 OffboardControlMode 발행 유지
        // PX4가 OFFBOARD 모드를 완전히 안정화할 시간을 확보
        if (!arm_sent_ && elapsed < OFFBOARD_STABILIZE_SEC) {
            // nav_state 확인 (OFFBOARD 모드가 유지되는지)
            if (ctx.nav_state.load() != 14) {
                RCLCPP_WARN(ctx.logger, "[ARM] OFFBOARD 모드 이탈 감지 (nav=%d), 대기 계속...",
                            ctx.nav_state.load());
            }
            return TransitionResult::STAY;
        }

        // ARM 명령 최초 전송
        if (!arm_sent_) {
            RCLCPP_INFO(ctx.logger, "[ARM] 시동 요청... (안정화 %.1fs 완료, nav_state=%d)",
                        elapsed, ctx.nav_state.load());
            ctx.publishCommand(400 /*VEHICLE_CMD_COMPONENT_ARM_DISARM*/, 1.0f, 0.0f, 0.0f);
            last_cmd_time_ = std::chrono::steady_clock::now();
            arm_sent_ = true;
        }

        // 조건: PX4가 ARM 확인
        if (ctx.arming_state.load() == 2) {
            // ARM 확인 시 시작 위치 최종 캡처 (가장 정확한 시점)
            ctx.start_local_x = ctx.current_local_x.load();
            ctx.start_local_y = ctx.current_local_y.load();
            ctx.start_local_z = ctx.current_local_z.load();
            ctx.initial_yaw = ctx.current_yaw.load();

            RCLCPP_INFO(ctx.logger, "[ARM] 시동 확인! Start NED: (%.1f, %.1f, %.1f) Yaw: %.1f deg (%.1fs)",
                        ctx.start_local_x, ctx.start_local_y, ctx.start_local_z,
                        ctx.initial_yaw * 180.0f / M_PI, elapsed);
            return TransitionResult::COMPLETE;
        }

        // 2초마다 ARM 명령 재전송
        auto now = std::chrono::steady_clock::now();
        double since_last = std::chrono::duration<double>(now - last_cmd_time_).count();
        if (arm_sent_ && since_last >= 2.0) {
            RCLCPP_INFO(ctx.logger, "[ARM] 재시도: ARM 명령 (arming_state=%d, nav=%d)",
                        ctx.arming_state.load(), ctx.nav_state.load());
            ctx.publishCommand(400, 1.0f, 0.0f, 0.0f);
            last_cmd_time_ = now;
        }

        // 타임아웃: 15초
        if (elapsed > 15.0) {
            RCLCPP_ERROR(ctx.logger, "[ARM] 타임아웃! arming_state=%d (expected 2)",
                         ctx.arming_state.load());
            return TransitionResult::ERROR;
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        // ARM 대기 중: 현재 위치 유지
        sp.position = {
            ctx.current_local_x.load(),
            ctx.current_local_y.load(),
            ctx.current_local_z.load()
        };
        sp.velocity = {NAN, NAN, NAN};
        sp.yaw = ctx.current_yaw.load();
        sp.yawspeed = 0.0f;
        return true;
    }

    const char* name() const override { return "ARMING"; }

private:
    static constexpr double OFFBOARD_STABILIZE_SEC = 1.5;  // OFFBOARD 안정화 대기 시간
    std::chrono::steady_clock::time_point last_cmd_time_;
    bool arm_sent_{false};
};

#endif // ARM_HANDLER_H
