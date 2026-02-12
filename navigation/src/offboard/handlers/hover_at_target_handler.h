/**
 * @file hover_at_target_handler.h
 * @brief HOVER_AT_TARGET (목표지점 호버링) 핸들러
 *
 * 목표 도착 후 호버링 + 점진적 고도 하강.
 * target_altitude > 0이면 해당 고도로 하강, 아니면 takeoff_altitude 유지.
 *
 * 전환 조건:
 *   단독: 5초(TARGET_HOVER_SEC) 경과 → COMPLETE
 *   편대: 30초(SUPPRESS_HOVER_SEC) 경과 → COMPLETE
 */

#ifndef HOVER_AT_TARGET_HANDLER_H
#define HOVER_AT_TARGET_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class HoverAtTargetHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();

        // 최종 목표 고도 결정
        float effective_alt = (ctx.target_altitude > 0.0f)
                              ? ctx.target_altitude
                              : ctx.takeoff_altitude;
        final_z_ = ctx.start_local_z - effective_alt;

        hover_duration_ = ctx.formation_mode ? SUPPRESS_HOVER_SEC : TARGET_HOVER_SEC;

        RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 목표지점 호버링 시작 (%.0f초), 목표고도: %.1fm",
                    hover_duration_, effective_alt);
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 호버링 종료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        if (elapsed >= hover_duration_) {
            RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 호버링 완료 (%.0f초)! RTL 전환",
                        hover_duration_);
            return TransitionResult::COMPLETE;
        }

        // 진행 로깅 (5초마다)
        int tick_5s = static_cast<int>(elapsed / 5.0);
        if (tick_5s != last_log_tick_) {
            last_log_tick_ = tick_5s;
            float current_z = ctx.current_local_z.load();
            RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] %.0f/%.0fs, 고도: %.1fm (목표: %.1fm)",
                        elapsed, hover_duration_,
                        -(current_z - ctx.start_local_z),
                        -(final_z_ - ctx.start_local_z));
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        float current_z = ctx.current_local_z.load();

        // 점진적 하강: 0.5m/s
        constexpr float DESCENT_PER_TICK = 0.05f;  // 0.5m/s * 0.1s

        float sp_z;
        if (current_z - final_z_ > 0.3f) {
            // NED: z가 작을수록 높음, +z = 아래로
            sp_z = current_z + DESCENT_PER_TICK;
            if (sp_z > final_z_) sp_z = final_z_;
        } else {
            sp_z = final_z_;
        }

        sp.position = {ctx.target_ned_x, ctx.target_ned_y, sp_z};
        sp.velocity = {NAN, NAN, NAN};
        sp.yaw = ctx.target_yaw;
        sp.yawspeed = 0.0f;
        return true;
    }

    const char* name() const override { return "HOVER_AT_TARGET"; }

private:
    static constexpr float TARGET_HOVER_SEC = 5.0f;
    static constexpr float SUPPRESS_HOVER_SEC = 30.0f;

    float final_z_{0.0f};
    float hover_duration_{5.0f};
    int last_log_tick_{-1};
};

#endif // HOVER_AT_TARGET_HANDLER_H
