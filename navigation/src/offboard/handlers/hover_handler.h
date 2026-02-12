/**
 * @file hover_handler.h
 * @brief HOVER (호버링) 핸들러
 *
 * TAKEOFF 완료 후 이륙 위치에서 안정화 호버링.
 * 편대 모드에서는 formation_ready_to_rotate 게이트 대기.
 *
 * 전환 조건:
 *   단독: hover_duration_sec 경과 → COMPLETE
 *   편대 리더: hover_duration_sec 경과 + formation_ready_to_rotate → COMPLETE
 *   편대 팔로워(continuous): 이 핸들러 진입 안 함 (TAKEOFF→NAVIGATE 직행)
 */

#ifndef HOVER_HANDLER_H
#define HOVER_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class HoverHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        gate_logged_ = false;

        hover_x_ = ctx.start_local_x;
        hover_y_ = ctx.start_local_y;
        hover_z_ = ctx.start_local_z - ctx.takeoff_altitude;

        RCLCPP_INFO(ctx.logger, "[HOVER] 호버링 시작 (%.1f초), 위치: (%.1f, %.1f, %.1f)",
                    ctx.hover_duration_sec, hover_x_, hover_y_, hover_z_);
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[HOVER] 호버링 종료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        // 최소 호버 시간 경과 확인
        if (elapsed < ctx.hover_duration_sec) {
            return TransitionResult::STAY;
        }

        // 편대 모드: 게이트 대기
        if (ctx.formation_mode && !ctx.continuous_update_mode.load()) {
            if (!ctx.formation_ready_to_rotate.load()) {
                if (!gate_logged_) {
                    RCLCPP_INFO(ctx.logger, "[HOVER] 편대 모드: 팔로워 이륙 대기중...");
                    gate_logged_ = true;
                }
                // 10초마다 리마인더
                if (static_cast<int>(elapsed) % 10 == 0 && static_cast<int>(elapsed * 10) % 100 == 0) {
                    RCLCPP_INFO(ctx.logger, "[HOVER] 편대 대기중 (%.0fs)...", elapsed);
                }
                return TransitionResult::STAY;
            }
            RCLCPP_INFO(ctx.logger, "[HOVER] 팔로워 이륙 확인! ROTATE로 전환 (대기: %.1fs)", elapsed);
        }

        // initial_yaw 갱신 (현재 yaw → ROTATE에서 참조)
        ctx.initial_yaw = ctx.current_yaw.load();

        RCLCPP_INFO(ctx.logger, "[HOVER] 호버링 완료 (%.1fs), yaw=%.1f° → target=%.1f°",
                    elapsed,
                    ctx.initial_yaw * 180.0f / M_PI,
                    ctx.target_yaw * 180.0f / M_PI);
        return TransitionResult::COMPLETE;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        (void)ctx;
        sp.position = {hover_x_, hover_y_, hover_z_};
        sp.velocity = {NAN, NAN, NAN};
        sp.yaw = ctx.initial_yaw;
        sp.yawspeed = 0.0f;
        return true;
    }

    const char* name() const override { return "HOVER"; }

private:
    float hover_x_{0.0f}, hover_y_{0.0f}, hover_z_{0.0f};
    bool gate_logged_{false};
};

#endif // HOVER_HANDLER_H
