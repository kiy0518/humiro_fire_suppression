/**
 * @file offboard_handler.h
 * @brief OFFBOARD 모드 확인 핸들러
 *
 * PX4가 OFFBOARD 모드(nav_state == 14)로 전환되었는지 확인.
 * 실패 시 1초마다 명령 재전송.
 *
 * 전환 조건: nav_state == 14 → COMPLETE
 * 타임아웃: 10초 → ERROR
 */

#ifndef OFFBOARD_HANDLER_H
#define OFFBOARD_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class OffboardHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        last_cmd_time_ = ctx.state_enter_time;
        RCLCPP_INFO(ctx.logger, "[OFFBOARD] OFFBOARD 모드 확인 대기 (nav_state=14)...");
    }

    void onExit(MissionContext& ctx) override {
        (void)ctx;
    }

    TransitionResult tick(MissionContext& ctx) override {
        // 조건: PX4가 OFFBOARD 모드 확인
        if (ctx.nav_state.load() == 14) {
            RCLCPP_INFO(ctx.logger, "[OFFBOARD] PX4 OFFBOARD 모드 확인! (%.1fs)", ctx.elapsedSec());
            return TransitionResult::COMPLETE;
        }

        // 1초마다 명령 재전송
        auto now = std::chrono::steady_clock::now();
        double since_last = std::chrono::duration<double>(now - last_cmd_time_).count();
        if (since_last >= 1.0) {
            RCLCPP_INFO(ctx.logger, "[OFFBOARD] 재시도: OFFBOARD 모드 전환 명령 (nav=%d)",
                        ctx.nav_state.load());
            ctx.publishCommand(176 /*VEHICLE_CMD_DO_SET_MODE*/, 1.0f, 6.0f, 0.0f);
            last_cmd_time_ = now;
        }

        // 타임아웃: 10초
        if (ctx.elapsedSec() > 10.0) {
            RCLCPP_ERROR(ctx.logger, "[OFFBOARD] 타임아웃! nav_state=%d (expected 14)",
                         ctx.nav_state.load());
            return TransitionResult::ERROR;
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        // OFFBOARD 대기 중: 현재 위치 유지
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

    const char* name() const override { return "OFFBOARD"; }

private:
    std::chrono::steady_clock::time_point last_cmd_time_;
};

#endif // OFFBOARD_HANDLER_H
