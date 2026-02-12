/**
 * @file arm_handler.h
 * @brief ARM (시동) 핸들러
 *
 * PX4에 ARM 명령을 전송하고 arming_state == 2 확인.
 * ARM 확인 시 start_local_x/y/z를 캡처 (이륙 기준점).
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
        last_cmd_time_ = ctx.state_enter_time;

        // ARM 전 시작 위치 1차 캡처
        if (ctx.position_received.load()) {
            ctx.start_local_x = ctx.current_local_x.load();
            ctx.start_local_y = ctx.current_local_y.load();
            ctx.start_local_z = ctx.current_local_z.load();
        }
        ctx.initial_yaw = ctx.current_yaw.load();

        // ARM 명령 전송
        RCLCPP_INFO(ctx.logger, "[ARM] 시동 요청...");
        ctx.publishCommand(400 /*VEHICLE_CMD_COMPONENT_ARM_DISARM*/, 1.0f, 0.0f, 0.0f);
    }

    void onExit(MissionContext& ctx) override {
        (void)ctx;
    }

    TransitionResult tick(MissionContext& ctx) override {
        // 조건: PX4가 ARM 확인
        if (ctx.arming_state.load() == 2) {
            // ARM 확인 시 시작 위치 최종 캡처 (가장 정확한 시점)
            ctx.start_local_x = ctx.current_local_x.load();
            ctx.start_local_y = ctx.current_local_y.load();
            ctx.start_local_z = ctx.current_local_z.load();
            ctx.initial_yaw = ctx.current_yaw.load();

            RCLCPP_INFO(ctx.logger, "[ARM] 시동 확인! Start NED: (%.1f, %.1f, %.1f) Yaw: %.1f deg (%.1fs)",
                        ctx.start_local_x, ctx.start_local_y, ctx.start_local_z,
                        ctx.initial_yaw * 180.0f / M_PI, ctx.elapsedSec());
            return TransitionResult::COMPLETE;
        }

        // 2초마다 ARM 명령 재전송
        auto now = std::chrono::steady_clock::now();
        double since_last = std::chrono::duration<double>(now - last_cmd_time_).count();
        if (since_last >= 2.0) {
            RCLCPP_INFO(ctx.logger, "[ARM] 재시도: ARM 명령 (arming_state=%d)",
                        ctx.arming_state.load());
            ctx.publishCommand(400, 1.0f, 0.0f, 0.0f);
            last_cmd_time_ = now;
        }

        // 타임아웃: 15초
        if (ctx.elapsedSec() > 15.0) {
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
    std::chrono::steady_clock::time_point last_cmd_time_;
};

#endif // ARM_HANDLER_H
