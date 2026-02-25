/**
 * @file takeoff_handler.h
 * @brief TAKEOFF (이륙) 핸들러
 *
 * ARM 완료 후 목표 고도까지 상승.
 * 시작 위치(start_local) 기준 상대 고도로 이륙.
 * MotionProfile1D로 부드러운 상승 (사다리꼴 속도 프로파일).
 *
 * 전환 조건: 고도 오차 < 0.5m + 1초 안정화 + 최소 3초 경과 → COMPLETE
 * 타임아웃: 동적 (고도/속도 기반, 30~180초) → ABORT_RTL
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
        // max_accel = max_speed / tau (사다리꼴 프로파일: tau초 만에 최대 속도 도달)
        float max_vz = std::max(0.05f, ctx.takeoff_max_speed);
        float tau   = std::max(0.1f, ctx.takeoff_accel_tau);
        float max_az = max_vz / tau;
        z_profile_.reset(ctx.start_local_z, max_vz, max_az, 0.1f);

        // 동적 타임아웃: (고도/속도)*1.5 + 가속구간 + 안정화 여유
        timeout_sec_ = (ctx.takeoff_altitude / max_vz) * 1.5f + tau + 10.0f;
        timeout_sec_ = std::max(30.0f, std::min(timeout_sec_, 180.0f));  // 30~180초 클램프

        RCLCPP_INFO(ctx.logger, "[TAKEOFF] 이륙 시작: 목표 고도 %.1fm (start_z=%.1f, target_z=%.1f, max_vz=%.2f, tau=%.1fs, max_az=%.3f, timeout=%.0fs)",
                    ctx.takeoff_altitude, ctx.start_local_z, takeoff_z_, max_vz, tau, max_az, timeout_sec_);
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

        // 동적 타임아웃
        if (ctx.elapsedSec() > timeout_sec_) {
            RCLCPP_ERROR(ctx.logger, "[TAKEOFF] 타임아웃 (%.0f초)! 고도 오차: %.2fm → ABORT_RTL", timeout_sec_, error);
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
        float current_z = ctx.current_local_z.load();

        // 프로파일 위치가 실제 드론보다 너무 앞서가지 않도록 제한 (max_lead_z)
        // NED 좌표계: z가 작을수록 높음. 이륙 시 프로파일은 음수 방향으로 진행.
        // 프로파일이 실제 위치보다 max_lead_z 이상 앞서면, 실제 위치 + lead로 리셋
        constexpr float MAX_LEAD_Z = 2.0f;  // 프로파일이 실제보다 최대 2m 앞서 허용
        float profile_z = z_profile_.getPosition();
        float lead = current_z - profile_z;  // 양수 = 프로파일이 위에 (앞서감)

        if (lead > MAX_LEAD_Z) {
            // 프로파일이 너무 앞서감 → 실제 위치 + MAX_LEAD_Z로 클램프
            float clamped_z = current_z - MAX_LEAD_Z;
            float max_vz = std::max(0.05f, ctx.takeoff_max_speed);
            float tau = std::max(0.1f, ctx.takeoff_accel_tau);
            z_profile_.reset(clamped_z, max_vz, max_vz / tau, 0.1f);
        }

        // Z축 프로파일 업데이트: 목표 고도를 향해 점진적 이동
        z_profile_.update(takeoff_z_);

        // X,Y: 이륙 위치 고정 (position control)
        // Z: position + velocity 피드포워드 → PX4가 위치 오차 보정 + 속도 힌트로 부드럽게 상승
        sp.position = {
            ctx.start_local_x,
            ctx.start_local_y,
            z_profile_.getPosition()    // 프로파일 위치 → PX4 위치 제어 (오차 보정)
        };
        sp.velocity = {
            NAN,
            NAN,
            z_profile_.getVelocity()    // 프로파일 속도 → 피드포워드 (부드러운 상승)
        };
        sp.yaw = ctx.initial_yaw;
        sp.yawspeed = 0.0f;
        return true;
    }

    const char* name() const override { return "TAKEOFF"; }

private:
    float takeoff_z_{0.0f};
    float timeout_sec_{30.0f};
    std::chrono::steady_clock::time_point stable_start_{};
    bool stable_logged_{false};
    int last_log_tick_{-1};

    // Z축 모션 프로파일
    MotionProfile1D z_profile_;
};

#endif // TAKEOFF_HANDLER_H
