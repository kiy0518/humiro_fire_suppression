/**
 * @file navigate_handler.h
 * @brief NAVIGATE (목표 위치 이동) 핸들러
 *
 * 목표 위치까지 position + velocity 피드포워드로 이동.
 * 감속 프로파일, yawspeed 추적, EVADE 오프셋 적용.
 *
 * 전환 조건: 수평 거리 < WAYPOINT_THRESHOLD (2.0m) → COMPLETE
 *
 * 편대 비행 고려:
 * - continuous_update_mode: position only (FF 간섭 방지)
 * - 단독/리더: position + velocity 피드포워드
 */

#ifndef NAVIGATE_HANDLER_H
#define NAVIGATE_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class NavigateHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        ctx.prev_vx = 0.0f;
        ctx.prev_vy = 0.0f;

        navigate_z_ = ctx.start_local_z - ctx.takeoff_altitude;

        RCLCPP_INFO(ctx.logger, "[NAVIGATE] 이동 시작: 목표 NED (%.1f, %.1f, %.1f)",
                    ctx.target_ned_x, ctx.target_ned_y, navigate_z_);
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[NAVIGATE] 이동 종료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        float cur_x = ctx.current_local_x.load();
        float cur_y = ctx.current_local_y.load();

        // 유효 타겟 (EVADE 오프셋 적용)
        float eff_x = ctx.target_ned_x + ctx.evade_offset_n;
        float eff_y = ctx.target_ned_y + ctx.evade_offset_e;
        float dx = eff_x - cur_x;
        float dy = eff_y - cur_y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // 도착 판정 (EVADE 중에는 판정 안 함)
        if (dist < ctx.WAYPOINT_THRESHOLD && ctx.collision_action.load() == 0) {
            RCLCPP_INFO(ctx.logger, "[NAVIGATE] 목표 도착! 거리: %.2fm (%.1fs)",
                        dist, ctx.elapsedSec());
            return TransitionResult::COMPLETE;
        }

        // 진행 로깅 (2초마다)
        int tick_2s = static_cast<int>(ctx.elapsedSec() / 2.0);
        if (tick_2s != last_log_tick_) {
            last_log_tick_ = tick_2s;
            RCLCPP_INFO(ctx.logger,
                "[NAVIGATE] dist=%.1fm NED=(%.1f,%.1f) GPS=(%.7f,%.7f) yaw=%.1f°→%.1f°",
                dist, cur_x, cur_y,
                ctx.current_lat.load(), ctx.current_lon.load(),
                ctx.current_yaw.load() * 180.0f / M_PI,
                ctx.target_yaw * 180.0f / M_PI);
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        // 충돌 방지: HOLD
        int ca = ctx.collision_action.load();
        if (ca == 1 /* HOLD */) {
            sp.position = {ctx.hold_x, ctx.hold_y, ctx.hold_z};
            sp.velocity = {NAN, NAN, NAN};
            sp.yaw = ctx.hold_yaw;
            sp.yawspeed = 0.0f;
            ctx.prev_vx = 0.0f;
            ctx.prev_vy = 0.0f;
            return true;
        }

        float cur_x = ctx.current_local_x.load();
        float cur_y = ctx.current_local_y.load();

        // 유효 타겟 (EVADE 오프셋 적용)
        float eff_x = ctx.target_ned_x + ctx.evade_offset_n;
        float eff_y = ctx.target_ned_y + ctx.evade_offset_e;
        float dx = eff_x - cur_x;
        float dy = eff_y - cur_y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // === Velocity 피드포워드 계산 ===
        float ff_vx = 0.0f, ff_vy = 0.0f;

        if (!ctx.continuous_update_mode.load()) {
            // 단독/리더: position + velocity 피드포워드
            float current_ff_speed = std::sqrt(ctx.prev_vx * ctx.prev_vx + ctx.prev_vy * ctx.prev_vy);
            float speed = ctx.flight_speed;

            // 감속 프로파일
            constexpr float DECEL_RADIUS = 80.0f;
            if (dist < DECEL_RADIUS) {
                float decel_speed = speed * (dist / DECEL_RADIUS);
                speed = std::max(0.3f, std::min(decel_speed, current_ff_speed));
            }

            if (dist > 0.3f) {
                float target_vx = (dx / dist) * speed;
                float target_vy = (dy / dist) * speed;

                constexpr float VELOCITY_ALPHA = 0.15f;
                ff_vx = ctx.prev_vx * (1.0f - VELOCITY_ALPHA) + target_vx * VELOCITY_ALPHA;
                ff_vy = ctx.prev_vy * (1.0f - VELOCITY_ALPHA) + target_vy * VELOCITY_ALPHA;
            } else {
                ff_vx = ctx.prev_vx * 0.5f;
                ff_vy = ctx.prev_vy * 0.5f;
            }
            ctx.prev_vx = ff_vx;
            ctx.prev_vy = ff_vy;
        }

        // === Yaw 제어 (감속 프로파일) ===
        float current_yaw = ctx.current_yaw.load();
        float yaw_diff = ctx.target_yaw - current_yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

        float yawspeed = 0.0f;
        float abs_diff = std::fabs(yaw_diff);
        constexpr float YAW_DECEL_ANGLE = 0.5f;

        if (abs_diff < 0.02f) {
            yawspeed = 0.0f;
        } else if (abs_diff < YAW_DECEL_ANGLE) {
            float s = ctx.MAX_YAW_RATE * (abs_diff / YAW_DECEL_ANGLE);
            s = std::max(0.1f, s);
            yawspeed = (yaw_diff > 0) ? s : -s;
        } else {
            yawspeed = (yaw_diff > 0) ? ctx.MAX_YAW_RATE : -ctx.MAX_YAW_RATE;
        }

        // === Setpoint 발행 ===
        sp.position = {eff_x, eff_y, navigate_z_};
        if (ctx.continuous_update_mode.load()) {
            sp.velocity = {NAN, NAN, NAN};  // 연속 추적: position only
        } else {
            sp.velocity = {ff_vx, ff_vy, NAN};  // 단독: 피드포워드
        }
        sp.yaw = NAN;
        sp.yawspeed = yawspeed;

        return true;
    }

    const char* name() const override { return "NAVIGATE"; }

private:
    float navigate_z_{0.0f};
    int last_log_tick_{-1};
};

#endif // NAVIGATE_HANDLER_H
