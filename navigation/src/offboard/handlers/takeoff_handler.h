/**
 * @file takeoff_handler.h
 * @brief TAKEOFF (이륙) 핸들러
 *
 * ARM 완료 후 목표 고도까지 상승.
 * 시작 위치(start_local) 기준 상대 고도로 이륙.
 *
 * 전환 조건: 고도 오차 < 0.5m + 1초 안정화 + 최소 3초 경과 → COMPLETE
 * 타임아웃: 30초 → ABORT_RTL
 */

#ifndef TAKEOFF_HANDLER_H
#define TAKEOFF_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class TakeoffHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        stable_start_ = std::chrono::steady_clock::time_point{};
        stable_logged_ = false;

        // 이륙 목표 고도 (NED: z가 작을수록 높음)
        takeoff_z_ = ctx.start_local_z - ctx.takeoff_altitude;

        RCLCPP_INFO(ctx.logger, "[TAKEOFF] 이륙 시작: 목표 고도 %.1fm (start_z=%.1f, target_z=%.1f)",
                    ctx.takeoff_altitude, ctx.start_local_z, takeoff_z_);
    }

    void onExit(MissionContext& ctx) override {
        (void)ctx;
    }

    TransitionResult tick(MissionContext& ctx) override {
        float current_z = ctx.current_local_z.load();
        float error = std::fabs(current_z - takeoff_z_);
        auto now = std::chrono::steady_clock::now();

        // 고도 오차 < 0.5m → 안정화 타이머 시작
        if (error < 0.5f) {
            if (stable_start_ == std::chrono::steady_clock::time_point{}) {
                stable_start_ = now;
                if (!stable_logged_) {
                    RCLCPP_INFO(ctx.logger, "[TAKEOFF] 목표 고도 근접 (오차: %.2fm), 안정화 대기...", error);
                    stable_logged_ = true;
                }
            }

            double stable_sec = std::chrono::duration<double>(now - stable_start_).count();

            // 최소 3초 경과 + 1초 안정화 → COMPLETE
            if (ctx.elapsedSec() >= 3.0 && stable_sec >= 1.0) {
                RCLCPP_INFO(ctx.logger, "[TAKEOFF] 이륙 완료! 고도 오차: %.2fm, 경과: %.1fs, 안정: %.1fs",
                            error, ctx.elapsedSec(), stable_sec);
                return TransitionResult::COMPLETE;
            }
        } else {
            // 안정화 중 오차 다시 커지면 리셋
            if (stable_start_ != std::chrono::steady_clock::time_point{}) {
                RCLCPP_INFO(ctx.logger, "[TAKEOFF] 안정화 리셋 (오차: %.2fm)", error);
                stable_start_ = std::chrono::steady_clock::time_point{};
            }
        }

        // 타임아웃: 30초
        if (ctx.elapsedSec() > 30.0) {
            RCLCPP_ERROR(ctx.logger, "[TAKEOFF] 타임아웃 (30초)! 고도 오차: %.2fm → ABORT_RTL", error);
            return TransitionResult::ABORT_RTL;
        }

        // 진행 로깅 (2초마다)
        int tick_2s = static_cast<int>(ctx.elapsedSec() / 2.0);
        if (tick_2s != last_log_tick_) {
            last_log_tick_ = tick_2s;
            RCLCPP_INFO(ctx.logger, "[TAKEOFF] 고도: %.1fm / %.1fm (오차: %.2fm, %.1fs)",
                        -(current_z - ctx.start_local_z), ctx.takeoff_altitude,
                        error, ctx.elapsedSec());
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        sp.position = {
            ctx.start_local_x,
            ctx.start_local_y,
            takeoff_z_
        };
        sp.velocity = {NAN, NAN, NAN};
        sp.yaw = ctx.initial_yaw;
        sp.yawspeed = 0.0f;
        return true;
    }

    const char* name() const override { return "TAKEOFF"; }

private:
    float takeoff_z_{0.0f};
    std::chrono::steady_clock::time_point stable_start_{};
    bool stable_logged_{false};
    int last_log_tick_{-1};
};

#endif // TAKEOFF_HANDLER_H
