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

        // RTL 고도: 5m AGL 고정 (NED: 지상 z - 5m)
        flight_z_ = ctx.start_local_z - RTL_ALTITUDE;

        // ctx에서 착륙 파라미터 로드
        descent_speed_ = ctx.rtl_descent_speed;
        soft_land_alt_ = ctx.rtl_soft_land_alt;
        // 거리계 설치 높이 (기체마다 다름, GUI 설정)
        rf_mount_height_ = ctx.rangefinder_mount_height;
        // 최종 착륙 속도: 거리계 있으면 설정값 그대로, 없으면 VZ_STABLE_THRESHOLD*2 이상 강제
        // (거리계 유무는 비행 중 변할 수 있으므로 안전측으로 하한 적용)
        landing_speed_min_ = std::max(ctx.rtl_landing_speed_min, VZ_STABLE_THRESHOLD * 2.0f);
        // 속도 역전 방지: landing_speed_min이 descent_speed보다 크면 클램프
        landing_speed_min_ = std::min(landing_speed_min_, descent_speed_);

        // DESCEND 타임아웃 계산 (고도/속도 기반, 150m/0.2m/s 최악 시나리오 대응)
        float agl_now = -(ctx.current_local_z.load() - ctx.start_local_z);
        float descend_alt = std::max(0.0f, agl_now - soft_land_alt_);
        float safe_speed = std::max(descent_speed_, 0.1f);
        descend_timeout_ = std::max(DESCEND_MIN_TIMEOUT, descend_alt / safe_speed * 2.0f + 60.0f);
        // SOFT_LAND 타임아웃: 동적 (매 tick AGL 기반으로 갱신), 비상 180초

        // 착지 감지 초기화
        ground_detected_ = false;
        land_detected_by_px4_ = false;
        land_detected_by_sensor_ = false;
        disarm_sent_ = false;
        disarm_attempts_ = 0;
        last_disarm_time_ = std::chrono::steady_clock::time_point{};
        prev_dist_bottom_ = -1.0f;
        dist_stable_count_ = 0;
        vz_stable_count_ = 0;
        baro_agl_stable_count_ = 0;

        // 현재 드론 위치에서 모션 프로파일 초기화
        float cur_x = ctx.current_local_x.load();
        float cur_y = ctx.current_local_y.load();
        float cur_z = ctx.current_local_z.load();
        float rtl_max_axy = ctx.nav_max_accel_xy;
        float rtl_max_vz  = ctx.nav_max_speed_z;
        nav_profile_.reset(cur_x, cur_y, cur_z,
                           ctx.flight_speed, rtl_max_axy,
                           rtl_max_vz, RTL_MAX_AZ, 0.1f);

        // RTL yaw: 현재 헤딩 유지 (회전 없이 복귀)
        float dx = home_x_ - cur_x;
        float dy = home_y_ - cur_y;
        rtl_yaw_ = ctx.current_yaw.load();

        // 초기 페이즈 결정: 이미 홈 근처면 바로 하강
        float home_dist = std::sqrt(dx * dx + dy * dy);

        // NAVIGATE_HOME 타임아웃 (거리/속도 기반, 최소 120초)
        float flight_speed = std::max(ctx.flight_speed, 1.0f);
        nav_home_timeout_ = std::max(NAV_HOME_MIN_TIMEOUT, home_dist / flight_speed * 2.0f + 60.0f);

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
                "[RTL] 귀환 시작: 홈까지 %.1fm, AGL=%.1fm, speed=%.1f, nav_home_timeout=%.0fs, descend_timeout=%.0fs",
                home_dist, getAGL(ctx), ctx.flight_speed, nav_home_timeout_, descend_timeout_);
        }
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[RTL] 착륙 완료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        // OSD 서브페이즈 + 착지 디버그 업데이트
        ctx.rtl_sub_phase = phaseToString(phase_);
        ctx.rtl_land_debug = buildLandDebugString(ctx);

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

            // 동적 타임아웃 (거리/속도 기반)
            if (phase_elapsed > nav_home_timeout_) {
                RCLCPP_WARN(ctx.logger,
                    "[RTL] NAVIGATE_HOME 타임아웃 (%.0f초, 거리 %.1fm) → 현재 위치에서 하강",
                    nav_home_timeout_, dist);
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
            // 삼중화 착지 감지: 1순위 PX4 → 2순위 거리계+수직속도 → 3순위 기압계+수직속도
            float actual_vz = std::fabs(ctx.actual_vz.load());
            bool has_rf = ctx.dist_bottom_valid.load();
            bool px4_landed = ctx.land_detected.load();
            float dist_bottom = ctx.dist_bottom.load();

            // --- 수직 속도 안정화 카운터 (전 센서 공통) ---
            // 10Hz tick 기준, vz < 0.05m/s 연속 카운트
            if (actual_vz < VZ_STABLE_THRESHOLD) {
                vz_stable_count_++;
            } else {
                vz_stable_count_ = 0;
            }
            bool vz_stable = (vz_stable_count_ >= VZ_STABLE_TICKS);  // 3초간 vz ≈ 0

            // --- 거리계 안정화 카운터 (거리 변화량 기반) ---
            if (has_rf && prev_dist_bottom_ > 0.0f) {
                float dist_delta = std::fabs(dist_bottom - prev_dist_bottom_);
                if (dist_delta < DIST_STABLE_DELTA) {  // 3cm 이내 변화 없음
                    dist_stable_count_++;
                } else {
                    dist_stable_count_ = 0;
                }
            } else {
                dist_stable_count_ = 0;
            }
            if (has_rf) prev_dist_bottom_ = dist_bottom;

            bool dist_stable = (dist_stable_count_ >= DIST_STABLE_TICKS);  // 3초간 거리 안정

            // --- 기압계 AGL 안정화 카운터 (기압 드리프트 오판 방지) ---
            if (!has_rf && agl < GROUND_THRESHOLD && agl >= -0.5f) {
                baro_agl_stable_count_++;
            } else {
                baro_agl_stable_count_ = 0;
            }
            bool baro_agl_stable = (baro_agl_stable_count_ >= BARO_AGL_STABLE_TICKS);  // 5초간 AGL 낮음

            // 2순위 조건: 거리계 유효 + 거리 안정 + 수직속도 안정 (동시 충족)
            bool rf_on_ground = has_rf && dist_stable && vz_stable
                             && (agl < RNG_AGL_LAND_THRESHOLD);
            // 3순위 조건: 거리계 무효 + 기압계 AGL 연속 5초 낮음 + 수직속도 안정
            bool baro_on_ground = !has_rf && vz_stable && baro_agl_stable;

            // === 1순위: PX4 land_detected (가속도계+기압계+자이로 융합, 가장 신뢰성 높음) ===
            if (px4_landed) {
                if (!ground_detected_) {
                    ground_detected_ = true;
                    land_detected_by_px4_ = true;
                    land_detected_by_sensor_ = false;
                    ground_detect_time_ = std::chrono::steady_clock::now();
                    RCLCPP_INFO(ctx.logger, "[RTL] 1순위 PX4 land_detected 확인! %.1fs 안정화 대기",
                                GROUND_STABLE_SEC_PX4);
                } else if (!land_detected_by_px4_) {
                    // 거리계/기압계로 이미 감지 중 → PX4 확인으로 업그레이드 (NORMAL DISARM 사용)
                    land_detected_by_px4_ = true;
                    land_detected_by_sensor_ = false;
                    RCLCPP_INFO(ctx.logger,
                        "[RTL] PX4 land_detected 확인 → 센서→PX4 업그레이드 (NORMAL DISARM)");
                }
            }
            // === 2순위: 거리계 안정 + 수직속도 안정 (3초간 동시 충족) ===
            else if (rf_on_ground && !land_detected_by_px4_) {
                if (!ground_detected_) {
                    ground_detected_ = true;
                    land_detected_by_px4_ = false;
                    land_detected_by_sensor_ = true;
                    ground_detect_time_ = std::chrono::steady_clock::now();
                    RCLCPP_INFO(ctx.logger,
                        "[RTL] 2순위 거리계+vz 착지 감지 (dist=%.2fm, vz=%.3fm/s, 안정 %.1fs) → %.1fs 안정화 대기",
                        dist_bottom, actual_vz,
                        dist_stable_count_ / 10.0f, GROUND_STABLE_SEC_SENSOR);
                }
            }
            // === 3순위: 기압계 AGL 낮음 + 수직속도 안정 (거리계 무효 시 폴백) ===
            else if (baro_on_ground && !land_detected_by_px4_ && !land_detected_by_sensor_) {
                if (!ground_detected_) {
                    ground_detected_ = true;
                    land_detected_by_px4_ = false;
                    land_detected_by_sensor_ = false;
                    ground_detect_time_ = std::chrono::steady_clock::now();
                    RCLCPP_INFO(ctx.logger,
                        "[RTL] 3순위 기압계+vz 착지 감지 (AGL=%.2fm, vz=%.3fm/s) → %.1fs 안정화 대기",
                        agl, actual_vz, GROUND_STABLE_SEC_BARO);
                }
            }
            // PX4 미감지 + 센서 조건 불충족 → 비PX4 감지만 리셋
            else if (!px4_landed && ground_detected_ && !land_detected_by_px4_) {
                RCLCPP_INFO(ctx.logger,
                    "[RTL] 착지 감지 리셋 (px4=%d, dist=%.2f, baro_agl=%.2f, vz=%.3f, ds=%d, vs=%d, bs=%d)",
                    (int)px4_landed, dist_bottom, agl, actual_vz,
                    dist_stable_count_, vz_stable_count_, baro_agl_stable_count_);
                ground_detected_ = false;
                land_detected_by_sensor_ = false;
            }

            if (ground_detected_) {
                auto now = std::chrono::steady_clock::now();
                double ground_sec = std::chrono::duration<double>(now - ground_detect_time_).count();
                float stable_sec = land_detected_by_px4_   ? GROUND_STABLE_SEC_PX4
                                 : land_detected_by_sensor_ ? GROUND_STABLE_SEC_SENSOR
                                 :                            GROUND_STABLE_SEC_BARO;
                if (ground_sec >= stable_sec) {
                    const char* method = land_detected_by_px4_   ? "PX4(1순위)"
                                       : land_detected_by_sensor_ ? "거리계+vz(2순위)"
                                       :                            "기압계+vz(3순위)";
                    RCLCPP_INFO(ctx.logger,
                        "[RTL] 착지 안정화 완료 (%.1fs, %s) → GROUND_DISARM", ground_sec, method);
                    // SOFT_LAND 최종 디버그 스냅샷 저장 (OSD에 GROUND_DISARM과 함께 표시)
                    soft_land_snapshot_ = buildSoftLandDebugString(ctx);
                    setPhase(RtlPhase::GROUND_DISARM);
                    break;
                }
            }

            // 동적 타임아웃 + 비상 타임아웃 로직
            float dynamic_timeout = calcDynamicSoftLandTimeout(agl);
            if (phase_elapsed > dynamic_timeout) {
                logPeriodic(ctx, 10.0,
                    "[RTL] SOFT_LAND 동적타임아웃 초과 (%.0fs > %.0fs) AGL=%.2fm → 하강 계속",
                    phase_elapsed, dynamic_timeout, agl);
            }
            if (phase_elapsed > SOFT_LAND_EMERGENCY_TIMEOUT) {
                RCLCPP_WARN(ctx.logger,
                    "[RTL] SOFT_LAND 비상 타임아웃 (%.0fs, AGL=%.2fm) → FORCE disarm",
                    phase_elapsed, agl);
                setPhase(RtlPhase::GROUND_DISARM);
                break;
            }

            logPeriodic(ctx, 2.0,
                "[RTL] SOFT_LAND AGL=%.2fm dist=%.2fm vz=%.3f speed=%.2f px4=%d ds=%d vs=%d bs=%d (%.0f/%.0fs)",
                agl, dist_bottom, ctx.actual_vz.load(), calcLandingSpeed(agl),
                (int)px4_landed, dist_stable_count_, vz_stable_count_, baro_agl_stable_count_,
                phase_elapsed, dynamic_timeout);
            break;
        }

        case RtlPhase::GROUND_DISARM: {
            auto now = std::chrono::steady_clock::now();

            if (!disarm_sent_ || (disarm_attempts_ < 5 &&
                std::chrono::duration<double>(now - last_disarm_time_).count() >= 3.0)) {
                disarm_attempts_++;
                disarm_sent_ = true;
                last_disarm_time_ = now;

                // PX4 감지(1순위): 처음 2회는 NORMAL, 3회차부터 FORCE 에스컬레이션
                // 거리계/기압계/비상: 처음부터 FORCE
                float force_param;
                const char* disarm_type;
                if (land_detected_by_px4_ && disarm_attempts_ <= 2) {
                    force_param = 0.0f;       // NORMAL DISARM
                    disarm_type = "NORMAL";
                } else if (ground_detected_ || land_detected_by_sensor_) {
                    force_param = 21196.0f;   // FORCE DISARM
                    disarm_type = (land_detected_by_px4_ && disarm_attempts_ > 2) ? "FORCE(에스컬)" : "FORCE";
                } else {
                    force_param = 21196.0f;   // 비상 타임아웃 경로: FORCE (180초 이상 감지 실패)
                    disarm_type = "FORCE(비상)";
                }

                ctx.publishCommand(400, 0.0f, force_param, 0.0f);
                RCLCPP_INFO(ctx.logger,
                    "[RTL] DISARM 명령 전송 (%d/%d, AGL=%.2fm, %s)",
                    disarm_attempts_, 5, agl, disarm_type);
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

            // Yaw: 현재 헤딩 유지 (회전 없이 복귀)
            sp.yaw = rtl_yaw_;
            sp.yawspeed = 0.0f;
            return true;
        }

        case RtlPhase::DESCEND: {
            // 수직 하강, 수평 위치 홈 고정
            // Z: velocity-only 제어 → PX4 100Hz 속도 제어 (계단식 방지)
            sp.position = {home_x_, home_y_, NAN};    // Z position 비활성화
            sp.velocity = {NAN, NAN, descent_speed_};  // velocity-only Z (NED +z = 하강)
            sp.yaw = rtl_yaw_;
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
            sp.yaw = rtl_yaw_;
            sp.yawspeed = 0.0f;
            return true;
        }

        case RtlPhase::GROUND_DISARM: {
            // 지면에 눌리도록 최소 하강 속도 유지 (호버링 방지)
            // position hold(home_z)로 하면 10cm 위에서 호버링 → disarm 시 추락
            sp.position = {home_x_, home_y_, NAN};
            sp.velocity = {NAN, NAN, landing_speed_min_};  // NED +z = 하강 계속
            sp.yaw = rtl_yaw_;
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
    float nav_home_timeout_{120.0f};   // NAVIGATE_HOME 타임아웃 (동적)
    float rf_mount_height_{0.18f};     // 거리계 설치 높이 (m, GUI 설정)

    // === 상수 ===
    static constexpr float RTL_ALTITUDE = 5.0f;          // RTL 귀환 고도 (m AGL, 이륙 기준)
    static constexpr float HOME_ARRIVAL_DIST = 2.0f;     // 홈 도착 거리 (m)
    static constexpr float RTL_MAX_AZ  = 1.0f;           // m/s²
    static constexpr float NAV_HOME_MIN_TIMEOUT = 120.0f;         // NAVIGATE_HOME 최소 타임아웃 (초)
    static constexpr float DESCEND_MIN_TIMEOUT  = 60.0f;          // DESCEND 최소 타임아웃 (초)
    static constexpr float SOFT_LAND_EMERGENCY_TIMEOUT = 180.0f;  // SOFT_LAND 비상 타임아웃 (초)

    // --- 착지 감지 상수 ---
    // 1순위 PX4
    static constexpr float GROUND_STABLE_SEC_PX4 = 0.5f;   // PX4 land_detected 안정화 시간 (초)
    // 2순위 거리계+수직속도
    static constexpr float RNG_AGL_LAND_THRESHOLD = 0.15f;      // 거리계 AGL 착지 상한 (설치높이 보정 후)
    static constexpr float DIST_STABLE_DELTA = 0.03f;      // 거리계 안정 판정: 변화량 < 3cm
    static constexpr int   DIST_STABLE_TICKS = 30;         // 거리계 안정 판정: 연속 30틱 (10Hz × 3초)
    static constexpr float GROUND_STABLE_SEC_SENSOR = 0.5f; // 2순위 감지 후 추가 안정화 (초)
    // 3순위 기압계+수직속도 (거리계 무효 시 폴백)
    static constexpr float GROUND_THRESHOLD = 0.3f;        // 기압계 AGL 임계값 (m)
    static constexpr int   BARO_AGL_STABLE_TICKS = 50;     // 기압계 AGL < threshold 연속 50틱 (10Hz × 5초)
    static constexpr float GROUND_STABLE_SEC_BARO = 1.0f;  // 3순위 감지 후 안정화 (초)
    // 수직속도 공통
    static constexpr float VZ_STABLE_THRESHOLD = 0.05f;    // vz 안정 판정 (m/s)
    static constexpr int   VZ_STABLE_TICKS = 30;           // vz 안정 판정: 연속 30틱 (10Hz × 3초)

    // === 상태 ===
    RtlPhase phase_{RtlPhase::NAVIGATE_HOME};
    std::chrono::steady_clock::time_point phase_enter_time_;

    float home_x_{0.0f}, home_y_{0.0f}, home_z_{0.0f};
    float flight_z_{0.0f};
    float rtl_yaw_{0.0f};  // 후진 비행용 yaw (홈 반대 방향)

    // 3D 모션 프로파일 (NAVIGATE_HOME 페이즈)
    MotionProfile3D nav_profile_;

    bool ground_detected_{false};
    bool land_detected_by_px4_{false};     // 1순위 PX4로 감지됨 → NORMAL DISARM
    bool land_detected_by_sensor_{false};  // 2순위 거리계+vz로 감지됨 → FORCE DISARM
    std::chrono::steady_clock::time_point ground_detect_time_;
    bool disarm_sent_{false};
    int disarm_attempts_{0};
    std::chrono::steady_clock::time_point last_disarm_time_;

    // 거리계 안정화 추적
    float prev_dist_bottom_{-1.0f};   // 이전 tick 거리계 값
    int dist_stable_count_{0};        // 거리 변화 < 3cm 연속 카운트
    int vz_stable_count_{0};          // vz < 0.05m/s 연속 카운트
    int baro_agl_stable_count_{0};    // 기압계 AGL < threshold 연속 카운트
    std::string soft_land_snapshot_;  // SOFT_LAND 최종 디버그 스냅샷 (GROUND_DISARM에서 표시)

    int last_log_tick_{-1};

    // === 유틸리티 ===

    /** AGL 계산: 거리계 유효 시 설치 높이 보정 후 사용, 아니면 기압계 폴백 */
    float getAGL(const MissionContext& ctx) const {
        if (ctx.dist_bottom_valid.load()) {
            float d = ctx.dist_bottom.load();
            if (d >= 0.0f) return std::max(0.0f, d - rf_mount_height_);
        }
        return -(ctx.current_local_z.load() - home_z_);
    }

    /** 감속 착륙 속도 계산
     * soft_land_alt → 0m 전체 구간에서 descent_speed → landing_speed_min 선형 감속.
     * GUI에서 soft_land_alt 변경 시 감속 구간 길이가 실제로 반영됨.
     */
    float calcLandingSpeed(float agl) const {
        if (soft_land_alt_ <= 0.0f) return landing_speed_min_;
        if (agl >= soft_land_alt_) return descent_speed_;
        if (agl <= 0.0f) return landing_speed_min_;
        // soft_land_alt → 0m: descent_speed → landing_speed_min 선형 감속
        float ratio = agl / soft_land_alt_;
        return landing_speed_min_ + (descent_speed_ - landing_speed_min_) * ratio;
    }

    /** 동적 SOFT_LAND 타임아웃: 현재 AGL 기반으로 매 tick 갱신 */
    float calcDynamicSoftLandTimeout(float agl) const {
        float safe_agl = std::max(0.0f, agl);
        float avg_speed = (calcLandingSpeed(safe_agl) + landing_speed_min_) * 0.5f;
        if (avg_speed < 0.01f) avg_speed = 0.01f;
        // 예상 착륙 시간 × 2배 마진 + 15초 여유
        return std::max(30.0f, safe_agl / avg_speed * 2.0f + 15.0f);
    }

    /** 착지 감지 디버그 문자열 (OSD 표시용) */
    std::string buildLandDebugString(const MissionContext& ctx) const {
        switch (phase_) {
        case RtlPhase::NAVIGATE_HOME: {
            float dx = home_x_ - ctx.current_local_x.load();
            float dy = home_y_ - ctx.current_local_y.load();
            float dist = std::sqrt(dx * dx + dy * dy);
            char buf[64];
            std::snprintf(buf, sizeof(buf), "HOME:%.0fm", dist);
            return buf;
        }
        case RtlPhase::DESCEND: {
            float agl = getAGL(ctx);
            char buf[64];
            std::snprintf(buf, sizeof(buf), "AGL:%.1fm spd:%.2f", agl, descent_speed_);
            return buf;
        }
        case RtlPhase::SOFT_LAND:
            return buildSoftLandDebugString(ctx);
        case RtlPhase::GROUND_DISARM: {
            const char* method = land_detected_by_px4_   ? "PX4"
                               : land_detected_by_sensor_ ? "RNG+VZ"
                               : ground_detected_          ? "BARO"
                               :                            "EMRG";
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%s #%d/%d %s",
                method, disarm_attempts_, 5,
                (land_detected_by_px4_ && disarm_attempts_ <= 2) ? "NORM" : "FORCE");
            // SOFT_LAND 스냅샷 + GROUND_DISARM 결합 (OSD에서 확인 가능)
            if (!soft_land_snapshot_.empty()) {
                return soft_land_snapshot_ + " > " + buf;
            }
            return buf;
        }
        }
        return "";
    }

    /** 페이즈 문자열 (OSD 표시용) */
    static const char* phaseToString(RtlPhase p) {
        switch (p) {
            case RtlPhase::NAVIGATE_HOME: return "NAV_HOME";
            case RtlPhase::DESCEND:       return "DESCEND";
            case RtlPhase::SOFT_LAND:     return "SOFT_LAND";
            case RtlPhase::GROUND_DISARM: return "DISARMING";
            default:                      return "";
        }
    }

    /** SOFT_LAND 디버그 문자열 (스냅샷 저장용으로도 사용) */
    std::string buildSoftLandDebugString(const MissionContext& ctx) const {
        float dist_bottom = ctx.dist_bottom.load();
        bool px4 = ctx.land_detected.load();
        bool maybe = ctx.maybe_landed.load();
        bool has_rf = ctx.dist_bottom_valid.load();
        float agl = getAGL(ctx);
        char buf[128];
        if (has_rf) {
            std::snprintf(buf, sizeof(buf), "PX4:%c ML:%c RNG:%d/%d VZ:%d/%d d=%.2f(%.2f)",
                px4 ? 'Y' : 'N', maybe ? 'Y' : 'N',
                dist_stable_count_, DIST_STABLE_TICKS,
                vz_stable_count_, VZ_STABLE_TICKS, agl, dist_bottom);
        } else {
            std::snprintf(buf, sizeof(buf), "PX4:%c ML:%c BA:%d/%d VZ:%d/%d a=%.2f",
                px4 ? 'Y' : 'N', maybe ? 'Y' : 'N',
                baro_agl_stable_count_, BARO_AGL_STABLE_TICKS,
                vz_stable_count_, VZ_STABLE_TICKS, agl);
        }
        return buf;
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
