/**
 * @file navigate_handler.h
 * @brief NAVIGATE (목표 위치 이동) 핸들러
 *
 * MotionProfile3D로 부드러운 가감속 이동.
 * 모든 거리에서 position + velocity setpoint을 동시에 제공.
 * yawspeed 추적, EVADE 오프셋, 충돌 HOLD 지원.
 *
 * 전환 조건: 수평 거리 < WAYPOINT_THRESHOLD (2.0m) → COMPLETE
 *
 * 편대 비행:
 * - continuous_update_mode에서 목표가 10Hz로 갱신되어도
 *   MotionProfile이 자연스럽게 추적 (별도 분기 불필요)
 */

#ifndef NAVIGATE_HANDLER_H
#define NAVIGATE_HANDLER_H

#include "state_handler.h"
#include "motion_profile.h"
#include "../mission_context.h"

class NavigateHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();

        navigate_z_ = ctx.start_local_z - ctx.takeoff_altitude;

        // 현재 드론 위치에서 모션 프로파일 초기화 (목적지가 아닌 현재 위치!)
        float cur_x = ctx.current_local_x.load();
        float cur_y = ctx.current_local_y.load();
        float cur_z = ctx.current_local_z.load();

        profile_.reset(
            cur_x, cur_y, cur_z,
            ctx.flight_speed, NAV_MAX_AXY,   // XY: 미션 속도, 안전 가속도
            NAV_MAX_VZ, NAV_MAX_AZ,           // Z: 보수적 수직 이동
            0.1f                               // dt = 100ms (10Hz)
        );
        was_holding_ = false;
        prev_yawspeed_ = 0.0f;

        RCLCPP_INFO(ctx.logger, "[NAVIGATE] 이동 시작: 현재 (%.1f,%.1f,%.1f) → 목표 (%.1f,%.1f,%.1f) speed=%.1f accel=%.1f",
                    cur_x, cur_y, cur_z,
                    ctx.target_ned_x, ctx.target_ned_y, navigate_z_,
                    ctx.flight_speed, NAV_MAX_AXY);
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

        // 팔로워 소화탄 소진 → 개별 RTL을 위해 NAVIGATE 완료
        if (ctx.continuous_update_mode.load() &&
            ctx.fire_gpio_index_ptr && ctx.fire_gpio_count > 0) {
            int idx = ctx.fire_gpio_index_ptr->load();
            if (idx >= ctx.fire_gpio_count) {
                RCLCPP_INFO(ctx.logger,
                    "[NAVIGATE] 팔로워 소화탄 소진 (%d/%d) → 개별 RTL 진행",
                    idx, ctx.fire_gpio_count);
                return TransitionResult::COMPLETE;
            }
        }

        // 도착 판정
        bool can_complete = !ctx.continuous_update_mode.load() ||
                            ctx.force_navigate_complete.load();
        if (can_complete &&
            dist < ctx.WAYPOINT_THRESHOLD && ctx.collision_action.load() == 0) {
            RCLCPP_INFO(ctx.logger, "[NAVIGATE] 목표 도착! 거리: %.2fm (%.1fs)%s",
                        dist, ctx.elapsedSec(),
                        ctx.force_navigate_complete.load() ? " [개별진행]" : "");
            return TransitionResult::COMPLETE;
        }

        // 진행 로깅 (2초마다)
        int tick_2s = static_cast<int>(ctx.elapsedSec() / 2.0);
        if (tick_2s != last_log_tick_) {
            last_log_tick_ = tick_2s;
            auto vel = profile_.getVelocity();
            float sp_speed = std::sqrt(vel[0]*vel[0] + vel[1]*vel[1]);
            RCLCPP_INFO(ctx.logger,
                "[NAVIGATE] dist=%.1fm NED=(%.1f,%.1f) sp_vel=%.1fm/s yaw=%.1f°→%.1f°",
                dist, cur_x, cur_y, sp_speed,
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
            was_holding_ = true;
            return true;
        }

        // HOLD 해제 후 프로파일 재초기화 (현재 위치에서 다시 시작)
        if (was_holding_) {
            float cur_x = ctx.current_local_x.load();
            float cur_y = ctx.current_local_y.load();
            float cur_z = ctx.current_local_z.load();
            profile_.reset(cur_x, cur_y, cur_z,
                           ctx.flight_speed, NAV_MAX_AXY,
                           NAV_MAX_VZ, NAV_MAX_AZ, 0.1f);
            was_holding_ = false;
            prev_yawspeed_ = 0.0f;
            RCLCPP_INFO(ctx.logger, "[NAVIGATE] HOLD 해제 → 프로파일 재초기화 (%.1f,%.1f,%.1f)",
                        cur_x, cur_y, cur_z);
        }

        // 유효 타겟 (EVADE 오프셋 적용)
        float eff_x = ctx.target_ned_x + ctx.evade_offset_n;
        float eff_y = ctx.target_ned_y + ctx.evade_offset_e;

        // 모션 프로파일 업데이트: 목표를 향해 가속도 제한 내에서 이동
        profile_.update(eff_x, eff_y, navigate_z_);

        auto pos = profile_.getPosition();
        auto vel = profile_.getVelocity();

        // === Yaw 제어 (사다리꼴 프로파일: 가속→순항→감속→정지) ===
        float current_yaw = ctx.current_yaw.load();
        float yaw_diff = ctx.target_yaw - current_yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

        float abs_diff = std::fabs(yaw_diff);

        float target_yawspeed = 0.0f;
        if (abs_diff < 0.02f) {
            // ~1° 이내: 정지
            target_yawspeed = 0.0f;
            prev_yawspeed_ = 0.0f;
        } else if (abs_diff < YAW_DECEL_ANGLE) {
            float s = ctx.MAX_YAW_RATE * (abs_diff / YAW_DECEL_ANGLE);
            s = std::max(0.05f, s);
            target_yawspeed = (yaw_diff > 0) ? s : -s;
        } else {
            target_yawspeed = (yaw_diff > 0) ? ctx.MAX_YAW_RATE : -ctx.MAX_YAW_RATE;
        }

        // 가속도 제한 적용
        float max_delta = MAX_YAW_ACCEL * 0.1f;
        float delta = target_yawspeed - prev_yawspeed_;
        if (delta > max_delta) delta = max_delta;
        if (delta < -max_delta) delta = -max_delta;
        float yawspeed = prev_yawspeed_ + delta;
        prev_yawspeed_ = yawspeed;

        // === Setpoint: 항상 position + velocity 동시 제공 ===
        sp.position = {pos[0], pos[1], pos[2]};
        sp.velocity = {vel[0], vel[1], vel[2]};
        sp.yaw = NAN;
        sp.yawspeed = yawspeed;

        return true;
    }

    const char* name() const override { return "NAVIGATE"; }

private:
    float navigate_z_{0.0f};
    int last_log_tick_{-1};

    // 3D 모션 프로파일
    MotionProfile3D profile_;
    bool was_holding_{false};

    // yaw 가속도 제한
    float prev_yawspeed_{0.0f};

    // 수평 이동 제한값 (헥사콥터 안전 기본값)
    // NAV_MAX_VXY는 ctx.flight_speed 사용 (미션별 설정 가능)
    static constexpr float NAV_MAX_AXY = 1.5f;    // m/s² (최대 틸트 ~8.7°)
    static constexpr float NAV_MAX_VZ  = 1.0f;    // m/s (순항 중 수직 이동)
    static constexpr float NAV_MAX_AZ  = 1.0f;    // m/s² (수직 가속도)

    // yaw 제어 상수
    static constexpr float MAX_YAW_ACCEL = 0.2f;    // rad/s² (rotate_handler와 동일)
    static constexpr float YAW_DECEL_ANGLE = 0.5f;  // rad (~28.6°) 감속 시작 각도
};

#endif // NAVIGATE_HANDLER_H
