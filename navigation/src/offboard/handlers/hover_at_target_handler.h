/**
 * @file hover_at_target_handler.h
 * @brief HOVER_AT_TARGET (목표지점 호버링) 핸들러
 *
 * 목표 도착 후 호버링 + 점진적 고도 하강.
 * target_altitude > 0이면 해당 고도로 하강, 아니면 takeoff_altitude 유지.
 * MotionProfile1D로 부드러운 고도 전환 (사다리꼴 속도 프로파일).
 *
 * 전환 조건 (OR):
 *   1. 소화탄 모두 소진 → 2초 대기 후 COMPLETE
 *   2. 타임아웃 (TARGET_HOVER_SEC) 경과 → COMPLETE
 */

#ifndef HOVER_AT_TARGET_HANDLER_H
#define HOVER_AT_TARGET_HANDLER_H

#include "state_handler.h"
#include "motion_profile.h"
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

        // Z축 모션 프로파일: 현재 고도에서 목표 고도로 부드럽게 전환
        float current_z = ctx.current_local_z.load();
        float descent_speed = std::max(0.1f, ctx.nav_max_speed_z);  // 이동 상승/하강 속도 적용
        float descent_accel = descent_speed;  // 가속도 = 속도 (1초에 최대 속도 도달)
        z_profile_.reset(current_z, descent_speed, descent_accel, 0.1f);

        // Swarm 팔로워: 리더 정렬 대기 모드 (최대 300초)
        swarm_wait_mode_ = (ctx.swarm_mode && !ctx.formation_mode);

        hover_duration_ = swarm_wait_mode_ ? SWARM_ALIGNMENT_WAIT_SEC
                        : ctx.formation_mode ? SUPPRESS_HOVER_SEC
                        : TARGET_HOVER_SEC;

        // 소화탄 소진 상태 초기화
        ammo_depleted_ = false;
        ammo_depleted_time_ = {};

        int remaining = getRemaining(ctx);
        float alt_diff = -(final_z_ - current_z);  // 고도 차이 (양수=하강)
        RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 목표지점 호버링 시작 (최대 %.0f초), 목표고도: %.1fm, 고도차: %.1fm, 하강속도: %.1fm/s, 잔탄: %d/%d",
                    hover_duration_, effective_alt, alt_diff, descent_speed, remaining, ctx.fire_gpio_count);
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 호버링 종료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        // Swarm 팔로워: 리더 정렬 완료 대기
        if (swarm_wait_mode_) {
            if (ctx.swarm_alignment_received.load()) {
                ctx.swarm_alignment_received.store(false);  // 시그널 소비 (재진입 방지)
                RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] 리더 정렬 완료 수신 → 진압 위치로 이동 (%.1fs 대기)", elapsed);
                ctx.swarm_reposition_needed = true;
                return TransitionResult::COMPLETE;
            }
            // 300초 타임아웃 (안전)
            if (elapsed >= SWARM_ALIGNMENT_WAIT_SEC) {
                RCLCPP_WARN(ctx.logger, "[HOVER_AT_TARGET] Swarm 정렬 대기 타임아웃 (%.0fs) → RTL", elapsed);
                return TransitionResult::ABORT_RTL;
            }
            // 로깅 (30초마다)
            int tick_30s = static_cast<int>(elapsed / 30.0);
            if (tick_30s != last_swarm_log_tick_) {
                last_swarm_log_tick_ = tick_30s;
                RCLCPP_INFO(ctx.logger, "[HOVER_AT_TARGET] Swarm 정렬 대기 중... %.0f/%.0fs",
                            elapsed, SWARM_ALIGNMENT_WAIT_SEC);
            }
            return TransitionResult::STAY;
        }

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
        // Z축 모션 프로파일로 부드러운 고도 전환 (사다리꼴 가감속)
        z_profile_.update(final_z_);

        sp.position = {ctx.target_ned_x, ctx.target_ned_y, z_profile_.getPosition()};
        sp.velocity = {NAN, NAN, z_profile_.getVelocity()};  // Z velocity 피드포워드
        sp.yaw = ctx.target_yaw;
        sp.yawspeed = 0.0f;
        return true;
    }

    const char* name() const override { return "HOVER_AT_TARGET"; }

private:
    // 테스트용: 30초 (운용: 300초로 변경)
    static constexpr float TARGET_HOVER_SEC = 30.0f;
    static constexpr float SUPPRESS_HOVER_SEC = 30.0f;
    static constexpr float SWARM_ALIGNMENT_WAIT_SEC = 300.0f;  // Swarm 정렬 대기 최대 시간
    static constexpr float AMMO_DEPLETED_DELAY_SEC = 2.0f;  // 소화탄 소진 후 RTL까지 대기

    float final_z_{0.0f};
    float hover_duration_{30.0f};
    int last_log_tick_{-1};

    // Swarm 팔로워 정렬 대기
    bool swarm_wait_mode_{false};
    int last_swarm_log_tick_{-1};

    // Z축 모션 프로파일 (부드러운 고도 전환)
    MotionProfile1D z_profile_;

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
