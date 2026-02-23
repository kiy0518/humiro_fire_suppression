/**
 * @file hover_at_target_handler.h
 * @brief HOVER_AT_TARGET (목표지점 호버링) 핸들러
 *
 * 목표 도착 후 호버링 + 점진적 고도 하강.
 * target_altitude > 0이면 해당 고도로 하강, 아니면 takeoff_altitude 유지.
 *
 * 전환 조건 (OR):
 *   1. 소화탄 모두 소진 → 2초 대기 후 COMPLETE
 *   2. 타임아웃 (TARGET_HOVER_SEC) 경과 → COMPLETE
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

        // 소화탄 소진 상태 초기화
        ammo_depleted_ = false;
        ammo_depleted_time_ = {};

        int remaining = getRemaining(ctx);
        RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 목표지점 호버링 시작 (최대 %.0f초), 목표고도: %.1fm, 잔탄: %d/%d",
                    hover_duration_, effective_alt, remaining, ctx.fire_gpio_count);
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 호버링 종료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        // 조건 1: 소화탄 모두 소진 → 2초 대기 후 RTL
        if (ctx.fire_gpio_index_ptr && ctx.fire_gpio_count > 0) {
            int idx = ctx.fire_gpio_index_ptr->load();
            if (idx >= ctx.fire_gpio_count) {
                if (!ammo_depleted_) {
                    ammo_depleted_ = true;
                    ammo_depleted_time_ = std::chrono::steady_clock::now();
                    RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 소화탄 모두 소진! 2초 후 RTL 전환");
                }
                auto since = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - ammo_depleted_time_).count();
                if (since >= AMMO_DEPLETED_DELAY_SEC) {
                    RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 소화탄 소진 → RTL 전환 (%.1f초 경과)", elapsed);
                    return TransitionResult::COMPLETE;
                }
            }
        }

        // 조건 2: 타임아웃
        if (elapsed >= hover_duration_) {
            RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 타임아웃 (%.0f초) → RTL 전환", hover_duration_);
            return TransitionResult::COMPLETE;
        }

        // 진행 로깅 (10초마다)
        int tick_10s = static_cast<int>(elapsed / 10.0);
        if (tick_10s != last_log_tick_) {
            last_log_tick_ = tick_10s;
            float current_z = ctx.current_local_z.load();
            int remaining = getRemaining(ctx);
            RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] %.0f/%.0fs, 잔탄: %d/%d, 고도: %.1fm",
                        elapsed, hover_duration_,
                        remaining, ctx.fire_gpio_count,
                        -(current_z - ctx.start_local_z));
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        float current_z = ctx.current_local_z.load();

        // 점진적 하강: 0.3m/s (중량 기체 안전값)
        constexpr float DESCENT_PER_TICK = 0.03f;  // 0.3m/s * 0.1s

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
    // 테스트용: 30초 (운용: 300초로 변경)
    static constexpr float TARGET_HOVER_SEC = 30.0f;
    static constexpr float SUPPRESS_HOVER_SEC = 30.0f;
    static constexpr float AMMO_DEPLETED_DELAY_SEC = 2.0f;  // 소화탄 소진 후 RTL까지 대기

    float final_z_{0.0f};
    float hover_duration_{30.0f};
    int last_log_tick_{-1};

    // 소화탄 소진 상태
    bool ammo_depleted_{false};
    std::chrono::steady_clock::time_point ammo_depleted_time_;

    int getRemaining(const MissionContext& ctx) const {
        if (!ctx.fire_gpio_index_ptr || ctx.fire_gpio_count <= 0) return 0;
        int idx = ctx.fire_gpio_index_ptr->load();
        return std::max(0, ctx.fire_gpio_count - idx);
    }
};

#endif // HOVER_AT_TARGET_HANDLER_H
