/**
 * @file rtl_handler.h
 * @brief RTL (Return To Launch) 핸들러 — OFFBOARD 커스텀 착륙
 *
 * OFFBOARD 위치/속도 제어로 RTL을 직접 수행.
 * PX4 AUTO_RTL을 사용하지 않고, 4단계 페이즈로 귀환+착륙.
 *
 * 페이즈:
 *   NAVIGATE_HOME → 홈 위치로 수평 이동 (비행고도 유지)
 *   DESCEND       → soft_land_alt AGL까지 descent_speed 하강
 *   SOFT_LAND     → soft_land_alt→0m, descent_speed→landing_speed_min 선형 감속
 *   GROUND_DISARM → 착지 감지 후 disarm 명령
 *
 * 전환 조건: arming_state == 1 (DISARMED) → COMPLETE
 */

#ifndef RTL_HANDLER_H
#define RTL_HANDLER_H

#include "state_handler.h"
#include "motion_profile.h"
#include "../mission_context.h"

class RtlHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();

        // 홈 위치 = ARM 시점 시작 위치
        home_x_ = ctx.start_local_x;
        home_y_ = ctx.start_local_y;
        home_z_ = ctx.start_local_z;  // 지상 NED z

        // 현재 비행 고도 유지하면서 홈으로 이동
        flight_z_ = ctx.current_local_z.load();

        // ctx에서 착륙 파라미터 로드
        descent_speed_ = ctx.rtl_descent_speed;
        landing_speed_min_ = ctx.rtl_landing_speed_min;
        soft_land_alt_ = ctx.rtl_soft_land_alt;

        // DESCEND 타임아웃 계산 (고도/속도 기반)
        float agl_now = -(ctx.current_local_z.load() - ctx.start_local_z);
        float descend_alt = std::max(0.0f, agl_now - soft_land_alt_);
        descend_timeout_ = std::max(60.0f, std::min(descend_alt / descent_speed_ * 2.0f + 30.0f, 300.0f));
        // SOFT_LAND 타임아웃: 동적 (매 tick AGL 기반으로 갱신), 비상 180초

        // 착지 감지 초기화
        ground_detected_ = false;
        disarm_sent_ = false;
        disarm_attempts_ = 0;
        last_disarm_time_ = std::chrono::steady_clock::time_point{};

        // 현재 드론 위치에서 모션 프로파일 초기화
        float cur_x = ctx.current_local_x.load();
        float cur_y = ctx.current_local_y.load();
        float cur_z = ctx.current_local_z.load();
        float rtl_max_axy = ctx.nav_max_accel_xy;
        float rtl_max_vz  = ctx.nav_max_speed_z;
        nav_profile_.reset(cur_x, cur_y, cur_z,
                           ctx.flight_speed, rtl_max_axy,
                           rtl_max_vz, RTL_MAX_AZ, 0.1f);

        // 초기 페이즈 결정: 이미 홈 근처면 바로 하강
        float dx = home_x_ - cur_x;
        float dy = home_y_ - cur_y;
        float home_dist = std::sqrt(dx * dx + dy * dy);

        if (home_dist < HOME_ARRIVAL_DIST) {
            phase_ = RtlPhase::DESCEND;
            phase_enter_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(ctx.logger,
                "[RTL] 귀환 시작 (홈 근처 %.1fm → 즉시 하강) AGL=%.1fm, descent=%.2fm/s, descend_timeout=%.0fs",
                home_dist, getAGL(ctx), descent_speed_, descend_timeout_);
        } else {
            phase_ = RtlPhase::NAVIGATE_HOME;
            phase_enter_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(ctx.logger,
                "[RTL] 귀환 시작: 홈까지 %.1fm, AGL=%.1fm, speed=%.1f, descent=%.2fm/s, descend_timeout=%.0fs",
                home_dist, getAGL(ctx), ctx.flight_speed, descent_speed_, descend_timeout_);
        }
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[RTL] 착륙 완료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        // Disarm 감지 = 착륙 완료 (전 페이즈 공통)
        if (ctx.arming_state.load() == 1) {
            RCLCPP_INFO(ctx.logger, "[RTL] Disarm 감지! 미션 완료 (%.1fs)", ctx.elapsedSec());
            return TransitionResult::COMPLETE;
        }

        float agl = getAGL(ctx);
        double phase_elapsed = phaseSec();

        switch (phase_) {
        case RtlPhase::NAVIGATE_HOME: {
            float dx = home_x_ - ctx.current_local_x.load();
            float dy = home_y_ - ctx.current_local_y.load();
            float dist = std::sqrt(dx * dx + dy * dy);

            // 도착 판정
            if (dist < HOME_ARRIVAL_DIST) {
                RCLCPP_INFO(ctx.logger,
                    "[RTL] 홈 도착 (%.1fm) → 하강 시작, AGL=%.1fm",
                    dist, agl);
                setPhase(RtlPhase::DESCEND);
                break;
            }

            // 타임아웃: 120초
            if (phase_elapsed > 120.0) {
                RCLCPP_WARN(ctx.logger,
                    "[RTL] NAVIGATE_HOME 타임아웃 (120초, 거리 %.1fm) → 현재 위치에서 하강",
                    dist);
                // 현재 위치를 홈으로 대체
                home_x_ = ctx.current_local_x.load();
                home_y_ = ctx.current_local_y.load();
                setPhase(RtlPhase::DESCEND);
                break;
            }

            // 로깅 (5초마다)
            logPeriodic(ctx, 5.0, "[RTL] NAVIGATE_HOME dist=%.1fm AGL=%.1fm (%.0fs)",
                        dist, agl, ctx.elapsedSec());
            break;
        }

        case RtlPhase::DESCEND: {
            // 1.5m AGL 이하 도달 → SOFT_LAND
            if (agl <= soft_land_alt_) {
                RCLCPP_INFO(ctx.logger,
                    "[RTL] AGL %.2fm ≤ %.1fm → 감속 착륙 시작",
                    agl, soft_land_alt_);
                setPhase(RtlPhase::SOFT_LAND);
                break;
            }

            // 동적 타임아웃
            if (phase_elapsed > descend_timeout_) {
                RCLCPP_WARN(ctx.logger,
                    "[RTL] DESCEND 타임아웃 (%.0f초, AGL=%.1fm) → SOFT_LAND 강제 전환", descend_timeout_, agl);
                setPhase(RtlPhase::SOFT_LAND);
                break;
            }

            logPeriodic(ctx, 3.0, "[RTL] DESCEND AGL=%.1fm vz=%.2fm/s (%.0fs)",
                        agl, ctx.actual_vz.load(), ctx.elapsedSec());
            break;
        }

        case RtlPhase::SOFT_LAND: {
            // 착지 판정: AGL < 0.15m
            if (agl < GROUND_THRESHOLD) {
                if (!ground_detected_) {
                    ground_detected_ = true;
                    ground_detect_time_ = std::chrono::steady_clock::now();
                    RCLCPP_INFO(ctx.logger,
                        "[RTL] 착지 감지 (AGL=%.2fm) → disarm 대기", agl);
                }
            }

            // 착지 감지 후 1초 안정화 → DISARM
            if (ground_detected_) {
                auto now = std::chrono::steady_clock::now();
                double ground_sec = std::chrono::duration<double>(now - ground_detect_time_).count();
                if (ground_sec >= 1.0) {
                    RCLCPP_INFO(ctx.logger,
                        "[RTL] 착지 안정화 완료 (%.1fs) → DISARM 시도", ground_sec);
                    setPhase(RtlPhase::GROUND_DISARM);
                    break;
                }
            }

            // 동적 타임아웃: 현재 AGL 기반으로 매 tick 갱신
            // 기압계 오차로 AGL이 부정확할 수 있으므로, 타임아웃 시 GROUND_DISARM 강제 전환하지 않음
            // → landing_speed_min으로 계속 하강하여 착지 감지 대기
            float dynamic_timeout = calcDynamicSoftLandTimeout(agl);
            if (phase_elapsed > dynamic_timeout) {
                // 타임아웃 초과해도 하강 계속 (착지 감지될 때까지)
                logPeriodic(ctx, 10.0,
                    "[RTL] SOFT_LAND 동적타임아웃 초과 (%.0fs > %.0fs) AGL=%.2fm → landing_speed_min(%.2fm/s)으로 하강 계속",
                    phase_elapsed, dynamic_timeout, agl, landing_speed_min_);
            }

            // 비상 타임아웃: 180초 → 일반 disarm 시도 (force 아님, PX4 자체 안전검사)
            if (phase_elapsed > SOFT_LAND_EMERGENCY_TIMEOUT) {
                RCLCPP_WARN(ctx.logger,
                    "[RTL] SOFT_LAND 비상 타임아웃 (%.0fs, AGL=%.2fm) → 일반 disarm 시도",
                    phase_elapsed, agl);
                setPhase(RtlPhase::GROUND_DISARM);
                break;
            }

            logPeriodic(ctx, 2.0, "[RTL] SOFT_LAND AGL=%.2fm vz=%.2fm/s speed=%.2fm/s (%.0f/%.0fs)",
                        agl, ctx.actual_vz.load(), calcLandingSpeed(agl),
                        phase_elapsed, dynamic_timeout);
            break;
        }

        case RtlPhase::GROUND_DISARM: {
            auto now = std::chrono::steady_clock::now();

            // DISARM 명령 전송 (3초 간격, 최대 5회)
            // 일반 disarm (param2=0): PX4 자체 공중 안전검사가 거부 가능
            // force disarm (param2=21196): 착지 감지 경로에서만 사용
            if (!disarm_sent_ || (disarm_attempts_ < 5 &&
                std::chrono::duration<double>(now - last_disarm_time_).count() >= 3.0)) {
                disarm_attempts_++;
                disarm_sent_ = true;
                last_disarm_time_ = now;

                float force_param = ground_detected_ ? 21196.0f : 0.0f;
                ctx.publishCommand(400, 0.0f, force_param, 0.0f);
                RCLCPP_INFO(ctx.logger,
                    "[RTL] DISARM 명령 전송 (%d/%d, AGL=%.2fm, %s)",
                    disarm_attempts_, 5, agl,
                    ground_detected_ ? "FORCE" : "NORMAL");
            }

            // 타임아웃: 20초
            if (phase_elapsed > 20.0) {
                RCLCPP_WARN(ctx.logger,
                    "[RTL] GROUND_DISARM 타임아웃 (20초, AGL=%.2fm) → COMPLETE 강제 반환", agl);
                return TransitionResult::COMPLETE;
            }

            break;
        }
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {

        switch (phase_) {
        case RtlPhase::NAVIGATE_HOME: {
            // === 수평 이동 (MotionProfile3D 적용) ===
            nav_profile_.update(home_x_, home_y_, flight_z_);

            auto pos = nav_profile_.getPosition();
            auto vel = nav_profile_.getVelocity();

            sp.position = {pos[0], pos[1], pos[2]};
            sp.velocity = {vel[0], vel[1], vel[2]};

            // Yaw: 홈 방향
            float cur_x = ctx.current_local_x.load();
            float cur_y = ctx.current_local_y.load();
            float dx = home_x_ - cur_x;
            float dy = home_y_ - cur_y;
            float dist = std::sqrt(dx * dx + dy * dy);

            if (dist > 1.0f) {
                float target_yaw = std::atan2(dy, dx);
                float current_yaw = ctx.current_yaw.load();
                float yaw_diff = target_yaw - current_yaw;
                while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
                while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

                float abs_diff = std::fabs(yaw_diff);
                float target_yawspeed = 0.0f;
                if (abs_diff < 0.02f) {
                    target_yawspeed = 0.0f;
                    prev_yawspeed_ = 0.0f;
                } else if (abs_diff < RTL_YAW_DECEL_ANGLE) {
                    float s = ctx.MAX_YAW_RATE * (abs_diff / RTL_YAW_DECEL_ANGLE);
                    s = std::max(0.05f, s);
                    target_yawspeed = (yaw_diff > 0) ? s : -s;
                } else {
                    target_yawspeed = (yaw_diff > 0) ? ctx.MAX_YAW_RATE : -ctx.MAX_YAW_RATE;
                }

                // 가속도 제한 적용
                float max_delta = RTL_MAX_YAW_ACCEL * 0.1f;
                float delta = target_yawspeed - prev_yawspeed_;
                if (delta > max_delta) delta = max_delta;
                if (delta < -max_delta) delta = -max_delta;
                float yawspeed = prev_yawspeed_ + delta;
                prev_yawspeed_ = yawspeed;

                sp.yaw = NAN;
                sp.yawspeed = yawspeed;
            } else {
                sp.yaw = ctx.current_yaw.load();
                sp.yawspeed = 0.0f;
                prev_yawspeed_ = 0.0f;
            }
            return true;
        }

        case RtlPhase::DESCEND: {
            // 수직 하강, 수평 위치 홈 고정
            // Z: velocity-only 제어 → PX4 100Hz 속도 제어 (계단식 방지)
            sp.position = {home_x_, home_y_, NAN};    // Z position 비활성화
            sp.velocity = {NAN, NAN, descent_speed_};  // velocity-only Z (NED +z = 하강)
            sp.yaw = ctx.current_yaw.load();
            sp.yawspeed = 0.0f;
            return true;
        }

        case RtlPhase::SOFT_LAND: {
            // 감속 착륙: AGL 기반 속도 보간
            // Z: velocity-only 제어 → 부드러운 착륙 (계단식 방지)
            float agl = getAGL(ctx);
            float speed = calcLandingSpeed(agl);

            sp.position = {home_x_, home_y_, NAN};    // Z position 비활성화
            sp.velocity = {NAN, NAN, speed};           // velocity-only Z (NED +z = 하강)
            sp.yaw = ctx.current_yaw.load();
            sp.yawspeed = 0.0f;
            return true;
        }

        case RtlPhase::GROUND_DISARM: {
            // 지면에 눌리도록 최소 하강 속도 유지 (호버링 방지)
            // position hold(home_z)로 하면 10cm 위에서 호버링 → disarm 시 추락
            sp.position = {home_x_, home_y_, NAN};
            sp.velocity = {NAN, NAN, landing_speed_min_};  // NED +z = 하강 계속
            sp.yaw = ctx.current_yaw.load();
            sp.yawspeed = 0.0f;
            return true;
        }
        }

        return false;
    }

    const char* name() const override { return "RTL"; }

private:
    // === 내부 페이즈 ===
    enum class RtlPhase {
        NAVIGATE_HOME,   // 홈으로 수평 이동
        DESCEND,         // 0.7m/s 하강 (→ 1.5m AGL)
        SOFT_LAND,       // 감속 착륙 (0.7→0.1 m/s)
        GROUND_DISARM    // 착지 감지 + disarm
    };

    // === 설정 가능 파라미터 (onEnter에서 ctx로부터 로드) ===
    float descent_speed_{0.4f};       // 기본 하강 속도 (m/s)
    float landing_speed_min_{0.05f};   // 최종 착륙 속도 (m/s)
    float soft_land_alt_{2.0f};        // 감속 시작 고도 (m AGL)
    float descend_timeout_{60.0f};     // DESCEND 페이즈 타임아웃 (동적)

    // === 상수 ===
    static constexpr float GROUND_THRESHOLD = 0.15f;     // 착지 판정 고도 (m AGL)
    static constexpr float FINAL_DECEL_ALT = 0.3f;      // 최종 감속 시작 고도 (m AGL)
    static constexpr float HOME_ARRIVAL_DIST = 2.0f;     // 홈 도착 거리 (m)
    static constexpr float RTL_MAX_AZ  = 1.0f;           // m/s²
    static constexpr float SOFT_LAND_EMERGENCY_TIMEOUT = 180.0f;  // SOFT_LAND 비상 타임아웃 (초)

    // yaw 제어 상수
    static constexpr float RTL_MAX_YAW_ACCEL = 0.2f;    // rad/s²
    static constexpr float RTL_YAW_DECEL_ANGLE = 0.5f;  // rad (~28.6°)

    // === 상태 ===
    RtlPhase phase_{RtlPhase::NAVIGATE_HOME};
    std::chrono::steady_clock::time_point phase_enter_time_;

    float home_x_{0.0f}, home_y_{0.0f}, home_z_{0.0f};
    float flight_z_{0.0f};

    // 3D 모션 프로파일 (NAVIGATE_HOME 페이즈)
    MotionProfile3D nav_profile_;
    float prev_yawspeed_{0.0f};

    bool ground_detected_{false};
    std::chrono::steady_clock::time_point ground_detect_time_;
    bool disarm_sent_{false};
    int disarm_attempts_{0};
    std::chrono::steady_clock::time_point last_disarm_time_;

    int last_log_tick_{-1};

    // === 유틸리티 ===

    /** AGL 계산 (NED: z 음수 = 위) */
    float getAGL(const MissionContext& ctx) const {
        return -(ctx.current_local_z.load() - home_z_);
    }

    /** 감속 착륙 속도 계산
     * 2단계 감속: soft_land_alt → FINAL_DECEL_ALT 구간은 descent_speed 유지,
     * FINAL_DECEL_ALT 이하에서만 landing_speed_min까지 감속.
     * PX4 land_detector가 낮은 속도를 "착지"로 오인하는 것을 방지.
     */
    float calcLandingSpeed(float agl) const {
        if (agl > soft_land_alt_) return descent_speed_;
        if (agl <= 0.0f) return landing_speed_min_;
        if (agl > FINAL_DECEL_ALT) return descent_speed_;
        // FINAL_DECEL_ALT 이하: descent_speed → landing_speed_min 선형 감속
        return landing_speed_min_ + (descent_speed_ - landing_speed_min_) * (agl / FINAL_DECEL_ALT);
    }

    /** 동적 SOFT_LAND 타임아웃: 현재 AGL 기반으로 매 tick 갱신 */
    float calcDynamicSoftLandTimeout(float agl) const {
        float safe_agl = std::max(0.0f, agl);
        float avg_speed = (calcLandingSpeed(safe_agl) + landing_speed_min_) * 0.5f;
        if (avg_speed < 0.01f) avg_speed = 0.01f;
        // 예상 착륙 시간 × 2배 마진 + 15초 여유
        return std::max(30.0f, safe_agl / avg_speed * 2.0f + 15.0f);
    }

    /** 페이즈 전환 */
    void setPhase(RtlPhase new_phase) {
        phase_ = new_phase;
        phase_enter_time_ = std::chrono::steady_clock::now();
        last_log_tick_ = -1;
    }

    /** 페이즈 경과 시간 */
    double phaseSec() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - phase_enter_time_).count();
    }

    /** 주기적 로깅 */
    template<typename... Args>
    void logPeriodic(MissionContext& ctx, double interval_sec, const char* fmt, Args... args) {
        int tick = static_cast<int>(phaseSec() / interval_sec);
        if (tick != last_log_tick_) {
            last_log_tick_ = tick;
            RCLCPP_INFO(ctx.logger, fmt, args...);
        }
    }
};

#endif // RTL_HANDLER_H
