/**
 * @file rtl_handler.h
 * @brief RTL (Return To Launch) 핸들러
 *
 * PX4 AUTO_RTL 모드로 전환하고 disarm 감지.
 * PX4가 자체적으로 귀환 경로 계산 및 착륙 수행.
 *
 * 전환 조건: arming_state == 1 (DISARMED) → COMPLETE
 * 타임아웃: 없음 (PX4 RTL이 자체 관리)
 */

#ifndef RTL_HANDLER_H
#define RTL_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class RtlHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();

        // PX4 AUTO_RTL 모드 전환
        RCLCPP_INFO(ctx.logger, "[RTL] 귀환 시작...");
        ctx.publishCommand(
            176 /*VEHICLE_CMD_DO_SET_MODE*/, 1.0f, 4.0f, 5.0f);
        ctx.publishCommand(
            20 /*VEHICLE_CMD_NAV_RETURN_TO_LAUNCH*/, 0.0f, 0.0f, 0.0f);
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[RTL] 착륙 완료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        // Disarm 감지 = 착륙 완료
        if (ctx.arming_state.load() == 1) {
            RCLCPP_INFO(ctx.logger, "[RTL] Disarm 감지! 미션 완료 (%.1fs)", ctx.elapsedSec());
            return TransitionResult::COMPLETE;
        }

        // 진행 로깅 (10초마다)
        int tick_10s = static_cast<int>(ctx.elapsedSec() / 10.0);
        if (tick_10s != last_log_tick_ && tick_10s > 0) {
            last_log_tick_ = tick_10s;
            RCLCPP_INFO(ctx.logger, "[RTL] 귀환 중 (%.0fs) nav=%d alt=%.1fm",
                        ctx.elapsedSec(),
                        ctx.nav_state.load(),
                        -(ctx.current_local_z.load()));
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        // RTL 모드에서는 PX4가 자체 제어, setpoint 발행 불필요
        (void)ctx;
        (void)sp;
        return false;
    }

    const char* name() const override { return "RTL"; }

private:
    int last_log_tick_{-1};
};

#endif // RTL_HANDLER_H
