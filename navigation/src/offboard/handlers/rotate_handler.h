/**
 * @file rotate_handler.h
 * @brief ROTATE (목표 방향 회전) 핸들러
 *
 * 목표 방향(target_yaw)으로 yawspeed 감속 프로파일 회전.
 * 위치는 이륙 위치에서 고도 유지.
 *
 * 전환 조건: |yaw_error| < 0.05 rad (~2.9°) → COMPLETE
 *   편대 리더: + formation_ready_to_navigate 게이트
 * 타임아웃: 30초 → ABORT_RTL
 */

#ifndef ROTATE_HANDLER_H
#define ROTATE_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class RotateHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        yaw_converged_ = false;
        gate_logged_ = false;

        hold_x_ = ctx.start_local_x;
        hold_y_ = ctx.start_local_y;
        hold_z_ = ctx.start_local_z - ctx.takeoff_altitude;

        RCLCPP_INFO(ctx.logger, "[ROTATE] 회전 시작: %.1f° → %.1f°",
                    ctx.initial_yaw * 180.0f / M_PI,
                    ctx.target_yaw * 180.0f / M_PI);
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[ROTATE] 회전 종료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        float current_yaw = ctx.current_yaw.load();
        float yaw_diff = ctx.target_yaw - current_yaw;

        // -π ~ π 정규화
        while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

        float abs_diff = std::fabs(yaw_diff);

        // yaw 수렴 확인: |error| < 0.05 rad (~2.9°)
        if (abs_diff < 0.05f) {
            if (!yaw_converged_) {
                yaw_converged_ = true;
                RCLCPP_INFO(ctx.logger, "[ROTATE] yaw 수렴 (오차: %.2f°, %.1fs)",
                            abs_diff * 180.0f / M_PI, ctx.elapsedSec());
            }

            // 편대 게이트 확인
            if (ctx.formation_mode && !ctx.continuous_update_mode.load()) {
                if (!ctx.formation_ready_to_navigate.load()) {
                    if (!gate_logged_) {
                        RCLCPP_INFO(ctx.logger, "[ROTATE] 편대 배치 대기중...");
                        gate_logged_ = true;
                    }
                    return TransitionResult::STAY;
                }
                RCLCPP_INFO(ctx.logger, "[ROTATE] 편대 배치 완료! NAVIGATE로 전환");
            }

            return TransitionResult::COMPLETE;
        }

        // yaw가 다시 벌어지면 리셋
        if (yaw_converged_ && abs_diff >= 0.1f) {
            yaw_converged_ = false;
        }

        // 타임아웃: 30초
        if (ctx.elapsedSec() > 30.0) {
            RCLCPP_ERROR(ctx.logger, "[ROTATE] 타임아웃 (30초)! yaw 오차: %.1f° → ABORT_RTL",
                         abs_diff * 180.0f / M_PI);
            return TransitionResult::ABORT_RTL;
        }

        // 진행 로깅 (3초마다)
        int tick_3s = static_cast<int>(ctx.elapsedSec() / 3.0);
        if (tick_3s != last_log_tick_) {
            last_log_tick_ = tick_3s;
            RCLCPP_INFO(ctx.logger, "[ROTATE] yaw: %.1f° → %.1f° (오차: %.1f°, %.1fs)",
                        current_yaw * 180.0f / M_PI,
                        ctx.target_yaw * 180.0f / M_PI,
                        abs_diff * 180.0f / M_PI,
                        ctx.elapsedSec());
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        sp.position = {hold_x_, hold_y_, hold_z_};
        sp.velocity = {NAN, NAN, NAN};

        // yawspeed 감속 프로파일
        float current_yaw = ctx.current_yaw.load();
        float yaw_diff = ctx.target_yaw - current_yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

        float abs_diff = std::fabs(yaw_diff);
        constexpr float YAW_DECEL_ANGLE = 0.5f;  // 감속 시작 각도 (rad, ~28.6°)

        if (abs_diff < 0.02f) {
            // ~1° 이내: 정지
            sp.yaw = ctx.target_yaw;
            sp.yawspeed = 0.0f;
        } else if (abs_diff < YAW_DECEL_ANGLE) {
            // 감속 구간: 각도 비례 (최소 0.1 rad/s)
            float speed = ctx.MAX_YAW_RATE * (abs_diff / YAW_DECEL_ANGLE);
            speed = std::max(0.1f, speed);
            sp.yaw = NAN;
            sp.yawspeed = (yaw_diff > 0) ? speed : -speed;
        } else {
            // 최대 속도 구간
            sp.yaw = NAN;
            sp.yawspeed = (yaw_diff > 0) ? ctx.MAX_YAW_RATE : -ctx.MAX_YAW_RATE;
        }

        return true;
    }

    const char* name() const override { return "ROTATE"; }

private:
    float hold_x_{0.0f}, hold_y_{0.0f}, hold_z_{0.0f};
    bool yaw_converged_{false};
    bool gate_logged_{false};
    int last_log_tick_{-1};
};

#endif // ROTATE_HANDLER_H
