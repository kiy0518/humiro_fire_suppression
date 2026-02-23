/**
 * @file takeoff_handler.h
 * @brief TAKEOFF (이륙) 핸들러
 *
 * ARM 완료 후 목표 고도까지 상승.
 * 시작 위치(start_local) 기준 상대 고도로 이륙.
 * MotionProfile1D로 부드러운 상승 (사다리꼴 속도 프로파일).
 *
 * 전환 조건: 고도 오차 < 0.5m + 1초 안정화 + 최소 3초 경과 → COMPLETE
 * 타임아웃: 30초 → ABORT_RTL
 */

#ifndef TAKEOFF_HANDLER_H
#define TAKEOFF_HANDLER_H

#include "state_handler.h"
#include "motion_profile.h"
#include "../mission_context.h"

class TakeoffHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        stable_start_ = std::chrono::steady_clock::time_point{};
        stable_logged_ = false;

        // 이륙 목표 고도 (NED: z가 작을수록 높음)
        takeoff_z_ = ctx.start_local_z - ctx.takeoff_altitude;

        // Z축 모션 프로파일: 현재 지면 높이에서 시작, 부드럽게 상승
        z_profile_.reset(ctx.start_local_z, TAKEOFF_MAX_VZ, TAKEOFF_MAX_AZ, 0.1f);

        RCLCPP_INFO(ctx.logger, "[TAKEOFF] 이륙 시작: 목표 고도 %.1fm (start_z=%.1f, target_z=%.1f, max_vz=%.1f)",
                    ctx.takeoff_altitude, ctx.start_local_z, takeoff_z_, TAKEOFF_MAX_VZ);
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
            RCLCPP_INFO(ctx.logger, "[TAKEOFF] 고도: %.1fm / %.1fm (오차: %.2fm, %.1fs, sp_z=%.2f, sp_vz=%.2f)",
                        -(current_z - ctx.start_local_z), ctx.takeoff_altitude,
                        error, ctx.elapsedSec(),
                        z_profile_.getPosition(), z_profile_.getVelocity());
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        // Z축 프로파일 업데이트: 목표 고도를 향해 점진적 이동
        z_profile_.update(takeoff_z_);

        sp.position = {
            ctx.start_local_x,
            ctx.start_local_y,
            z_profile_.getPosition()    // 점진적 상승 (was: takeoff_z_ step)
        };
        sp.velocity = {
            NAN,
            NAN,
            z_profile_.getVelocity()    // PX4에 속도 힌트 제공 (was: NAN)
        };
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

    // Z축 모션 프로파일
    MotionProfile1D z_profile_;

    // 이륙 제한값 (헥사콥터 안전 기본값)
    static constexpr float TAKEOFF_MAX_VZ = 1.5f;    // m/s (PX4 MPC_TKO_SPEED와 일치)
    static constexpr float TAKEOFF_MAX_AZ = 1.0f;    // m/s² (1.5초 만에 최대 속도)
};

#endif // TAKEOFF_HANDLER_H
