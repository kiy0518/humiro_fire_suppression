/**
 * @file tracking_hover_handler.h
 * @brief HOVER_AT_TARGET + 열원 추적 PD 제어 핸들러
 *
 * 열화상 카메라의 최고온도 지점(핫스팟)을 화면 중앙에 유지하도록
 * 드론의 yaw(좌우 회전) + 고도(상하)를 실시간 PD 제어.
 *
 * 카메라: 전방 수평 장착
 *   - rel_x (수평 오프셋) → yawspeed 보정
 *   - rel_y (수직 오프셋) → 고도 velocity 보정
 *
 * 활성화 모드:
 *   - 자동 (thermal_tracking_auto=true): onEnter 시 즉시 추적
 *   - 수동 (thermal_tracking_auto=false): thermal_tracking_active 플래그 대기
 *
 * 전환 조건:
 *   - 소화탄 모두 소진 (6발) → 2초 대기 후 COMPLETE → RTL
 *   - 열원 미감지 (80°C 이상 2분간 없음) → RTL (화재 없음 판정)
 */

#ifndef TRACKING_HOVER_HANDLER_H
#define TRACKING_HOVER_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"
#include "thermal_data.h"

#include <algorithm>

class TrackingHoverHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();

        // 최종 목표 고도 결정 (hover_at_target_handler와 동일)
        float effective_alt = (ctx.target_altitude > 0.0f)
                              ? ctx.target_altitude
                              : ctx.takeoff_altitude;
        final_z_ = ctx.start_local_z - effective_alt;

        // XY 위치 고정점
        hold_x_ = ctx.target_ned_x;
        hold_y_ = ctx.target_ned_y;

        // 고도 안전 한계 (NED: 작을수록 높음)
        min_z_ = ctx.start_local_z - MAX_ALTITUDE;  // 최대 고도 (가장 작은 z)
        max_z_ = ctx.start_local_z - MIN_ALTITUDE;  // 최소 고도 (가장 큰 z)

        // PD 제어 상태 초기화
        prev_error_yaw_ = 0.0f;
        prev_error_alt_ = 0.0f;
        prev_thermal_timestamp_ = 0.0;
        tracking_started_ = false;
        consecutive_invalid_ = 0;

        // 소화탄 소진 상태 초기화
        ammo_depleted_ = false;
        ammo_depleted_time_ = {};
        last_log_tick_ = -1;

        // 열원 미감지 타이머 초기화 (진입 시점부터 카운트)
        last_fire_detected_time_ = std::chrono::steady_clock::now();
        no_fire_warned_ = false;
        prev_ammo_index_ = ctx.fire_gpio_index_ptr ? ctx.fire_gpio_index_ptr->load() : 0;

        // 자동 모드이면 즉시 추적 시작
        if (ctx.thermal_tracking_auto.load()) {
            tracking_started_ = true;
            ctx.thermal_tracking_active.store(true);
        }

        int remaining = getRemaining(ctx);
        RCLCPP_INFO(ctx.logger,
            "[TRACKING_HOVER] 시작 (소화탄 소진 시 RTL), 목표고도: %.1fm, 잔탄: %d/%d, 추적: %s",
            effective_alt, remaining, ctx.fire_gpio_count,
            tracking_started_ ? "자동" : "수동대기");
    }

    void onExit(MissionContext& ctx) override {
        RCLCPP_INFO(ctx.logger, "[TRACKING_HOVER] 종료 (%.1fs)", ctx.elapsedSec());
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        // 수동 모드: GUI에서 active 플래그 확인
        if (!tracking_started_ && ctx.thermal_tracking_active.load()) {
            tracking_started_ = true;
            RCLCPP_INFO(ctx.logger, "[TRACKING_HOVER] 수동 추적 활성화 (%.1fs)", elapsed);
        }

        // 추적 중 active가 false로 변경되면 추적 중지
        if (tracking_started_ && !ctx.thermal_tracking_auto.load() && !ctx.thermal_tracking_active.load()) {
            tracking_started_ = false;
            prev_error_yaw_ = 0.0f;
            prev_error_alt_ = 0.0f;
            RCLCPP_INFO(ctx.logger, "[TRACKING_HOVER] 추적 비활성화 → 위치 고정");
        }

        // 조건 1: 소화탄 모두 소진 → 2초 대기 후 RTL
        if (ctx.fire_gpio_index_ptr && ctx.fire_gpio_count > 0) {
            int idx = ctx.fire_gpio_index_ptr->load();
            if (idx >= ctx.fire_gpio_count) {
                if (!ammo_depleted_) {
                    ammo_depleted_ = true;
                    ammo_depleted_time_ = std::chrono::steady_clock::now();
                    RCLCPP_INFO(ctx.logger, "[TRACKING_HOVER] 소화탄 모두 소진! 2초 후 RTL 전환");
                }
                auto since = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - ammo_depleted_time_).count();
                if (since >= AMMO_DEPLETED_DELAY_SEC) {
                    RCLCPP_INFO(ctx.logger, "[TRACKING_HOVER] 소화탄 소진 → RTL (%.1f초)", elapsed);
                    return TransitionResult::COMPLETE;
                }
            }
        }

        // 조건 2: 열원 미감지 (80°C 이상 2분간 없음 → RTL)
        //   리셋 조건: 80°C+ 열원 감지 OR 격발 (ammo index 변화)
        if (tracking_started_ && ctx.thermal_data_ptr) {
            ThermalData data = ctx.thermal_data_ptr->copy();
            // 격발 감지 → 타이머 리셋
            if (ctx.fire_gpio_index_ptr) {
                int cur_idx = ctx.fire_gpio_index_ptr->load();
                if (cur_idx != prev_ammo_index_) {
                    RCLCPP_INFO(ctx.logger, "[TRACKING_HOVER] 격발 감지 (%d→%d) → 미감지 타이머 리셋",
                        prev_ammo_index_, cur_idx);
                    prev_ammo_index_ = cur_idx;
                    last_fire_detected_time_ = std::chrono::steady_clock::now();
                    no_fire_warned_ = false;
                }
            }
            // 열원 감지 → 타이머 리셋
            if (data.valid && data.max_temp_celsius >= FIRE_TEMP_THRESHOLD) {
                last_fire_detected_time_ = std::chrono::steady_clock::now();
                no_fire_warned_ = false;
            }
            auto no_fire_sec = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_fire_detected_time_).count();

            if (no_fire_sec >= NO_FIRE_TIMEOUT_SEC) {
                RCLCPP_WARN(ctx.logger,
                    "[TRACKING_HOVER] %.0f°C 이상 열원 %.0f초간 미감지 → RTL",
                    FIRE_TEMP_THRESHOLD, NO_FIRE_TIMEOUT_SEC);
                return TransitionResult::COMPLETE;
            }
            if (!no_fire_warned_ && no_fire_sec >= NO_FIRE_TIMEOUT_SEC * 0.5) {
                no_fire_warned_ = true;
                RCLCPP_WARN(ctx.logger,
                    "[TRACKING_HOVER] 열원 미감지 %.0f초 경과 (%.0f초 후 RTL)",
                    no_fire_sec, NO_FIRE_TIMEOUT_SEC - no_fire_sec);
            }
        }

        // 진행 로깅 (5초마다)
        int tick_5s = static_cast<int>(elapsed / 5.0);
        if (tick_5s != last_log_tick_) {
            last_log_tick_ = tick_5s;
            float current_z = ctx.current_local_z.load();
            int remaining = getRemaining(ctx);

            if (tracking_started_ && ctx.thermal_data_ptr) {
                ThermalData data = ctx.thermal_data_ptr->copy();
                RCLCPP_INFO(ctx.logger,
                    "[TRACKING] %.0fs rel=(%d,%d) temp=%.1f°C 잔탄=%d/%d 고도=%.1fm %s",
                    elapsed,
                    data.rel_x, data.rel_y,
                    data.max_temp_celsius,
                    remaining, ctx.fire_gpio_count,
                    -(current_z - ctx.start_local_z),
                    data.valid ? "VALID" : "INVALID");
            } else {
                RCLCPP_INFO(ctx.logger,
                    "[TRACKING_HOVER] %.0fs 잔탄=%d/%d 고도=%.1fm (추적%s)",
                    elapsed,
                    remaining, ctx.fire_gpio_count,
                    -(current_z - ctx.start_local_z),
                    tracking_started_ ? "ON" : "OFF");
            }
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        float current_z = ctx.current_local_z.load();
        float yaw_rate = 0.0f;
        float vz = 0.0f;
        bool use_vz = false;

        // === 열원 추적 PD 제어 ===
        if (tracking_started_ && ctx.thermal_data_ptr) {
            ThermalData data = ctx.thermal_data_ptr->copy();

            // 유효성 검사: valid + 온도 임계값 + 데이터 경과 시간
            bool thermal_valid = data.valid
                && data.max_temp_celsius >= MIN_TEMP_THRESHOLD;

            // 타임스탬프 검사 (데이터가 너무 오래되면 무효)
            if (thermal_valid && data.timestamp > 0.0) {
                double now_sec = static_cast<double>(
                    std::chrono::steady_clock::now().time_since_epoch().count()) / 1e9;
                // ThermalData의 timestamp가 같은 클럭 기준이 아닐 수 있으므로
                // 이전 타임스탬프와 비교하여 업데이트 여부 확인
                if (data.timestamp == prev_thermal_timestamp_ && consecutive_invalid_ > 10) {
                    thermal_valid = false;  // 10회(1초) 이상 같은 데이터 → 타임아웃
                }
            }

            if (thermal_valid) {
                consecutive_invalid_ = 0;
                int rel_x = data.rel_x;  // 핫스팟 수평 오프셋 (양수=오른쪽)
                int rel_y = data.rel_y;  // 핫스팟 수직 오프셋 (양수=아래)

                // --- Yaw PD 제어 (수평 보정) ---
                if (std::abs(rel_x) > DEADZONE_H) {
                    float error_yaw = static_cast<float>(rel_x) * RAD_PER_PIXEL_H;
                    float d_error_yaw = 0.0f;
                    if (prev_thermal_timestamp_ > 0.0 &&
                        data.timestamp != prev_thermal_timestamp_) {
                        d_error_yaw = (error_yaw - prev_error_yaw_) / DT;
                    }

                    // PD 출력: 양수 rel_x(오른쪽) → 양수 yaw_rate(시계방향 회전)
                    // PX4 yawspeed: 양수 = 시계방향 (오른쪽으로 회전)
                    // 카메라가 전방이므로 핫스팟이 오른쪽에 있으면 기체를 오른쪽으로 회전
                    yaw_rate = KP_YAW * error_yaw + KD_YAW * d_error_yaw;
                    yaw_rate = std::clamp(yaw_rate, -MAX_TRACKING_YAW_RATE, MAX_TRACKING_YAW_RATE);

                    prev_error_yaw_ = error_yaw;
                } else {
                    prev_error_yaw_ = 0.0f;  // 데드존 내: D항 리셋
                }

                // --- 고도 PD 제어 (수직 보정) ---
                if (std::abs(rel_y) > DEADZONE_V) {
                    float error_alt_m = static_cast<float>(rel_y) * M_PER_PIXEL_V;
                    float d_error_alt = 0.0f;
                    if (prev_thermal_timestamp_ > 0.0 &&
                        data.timestamp != prev_thermal_timestamp_) {
                        d_error_alt = (error_alt_m - prev_error_alt_) / DT;
                    }

                    // PD 출력: 양수 rel_y(아래) → 양수 vz(하강)
                    // NED: 양수 vz = 아래로 이동
                    // 핫스팟이 화면 아래에 있으면 → 기체를 아래로 (vz 양수)
                    vz = KP_ALT * error_alt_m + KD_ALT * d_error_alt;
                    vz = std::clamp(vz, -MAX_TRACKING_VZ, MAX_TRACKING_VZ);

                    // 고도 안전 한계 검사
                    float proposed_z = current_z + vz * DT;
                    if (proposed_z < min_z_) vz = 0.0f;  // 최대 고도 초과 방지
                    if (proposed_z > max_z_) vz = 0.0f;  // 최소 고도 미달 방지

                    use_vz = true;
                    prev_error_alt_ = error_alt_m;
                } else {
                    prev_error_alt_ = 0.0f;
                }

                prev_thermal_timestamp_ = data.timestamp;
            } else {
                // 열화상 데이터 미유효: 제어 출력 0, 위치 고정
                consecutive_invalid_++;
                if (consecutive_invalid_ == 11) {
                    RCLCPP_WARN(ctx.logger, "[TRACKING] 열화상 데이터 타임아웃 → 위치 고정");
                }
                yaw_rate = 0.0f;
                vz = 0.0f;
                prev_error_yaw_ = 0.0f;
                prev_error_alt_ = 0.0f;
            }
        }

        // === Setpoint 구성 ===
        if (use_vz) {
            // 추적 중: XY 위치 고정, 고도는 속도 제어
            sp.position = {hold_x_, hold_y_, NAN};
            sp.velocity = {NAN, NAN, vz};
        } else {
            // 추적 비활성 또는 데드존 내: 전체 위치 제어
            // 점진적 하강 로직 (hover_at_target_handler와 동일)
            float sp_z;
            if (current_z - final_z_ > 0.3f) {
                sp_z = current_z + DESCENT_PER_TICK;
                if (sp_z > final_z_) sp_z = final_z_;
            } else {
                sp_z = final_z_;
            }
            sp.position = {hold_x_, hold_y_, sp_z};
            sp.velocity = {NAN, NAN, NAN};
        }

        sp.yaw = NAN;          // yawspeed로 제어
        sp.yawspeed = yaw_rate;

        return true;
    }

    const char* name() const override { return "TRACKING_HOVER"; }

private:
    // ========== 카메라 지오메트리 (FLIR Lepton 3.5) ==========
    // 수평: 57° / 480px = 0.11875°/px = 0.002073 rad/px
    static constexpr float RAD_PER_PIXEL_H = 0.002073f;
    // 수직: 10m 거리 기준, tan(19.5°) * 10 / 165 = 0.02145 m/px
    static constexpr float M_PER_PIXEL_V = 0.02145f;

    // ========== PD 제어 게인 (10m 정밀 조준용) ==========
    static constexpr float KP_YAW = 0.35f;   // 비례 (45kg 기체 관성 고려, 부드러운 응답)
    static constexpr float KD_YAW = 0.25f;   // 미분 (오버슈트 방지 강화)
    static constexpr float KP_ALT = 0.3f;    // 고도 비례
    static constexpr float KD_ALT = 0.15f;   // 고도 미분

    // ========== 데드존 (픽셀) ==========
    static constexpr int DEADZONE_H = 10;    // ~1.2° 수평 (10m: ±21cm 정밀도)
    static constexpr int DEADZONE_V = 8;     // ~1.0° 수직

    // ========== 제어 한계 (10m 정밀 조준) ==========
    static constexpr float MAX_TRACKING_YAW_RATE = 0.1f;   // rad/s (~5.7°/s, 10m: ~1m/s)
    static constexpr float MAX_TRACKING_VZ = 0.2f;          // m/s
    static constexpr float MIN_ALTITUDE = 3.0f;            // 최소 고도 (m)
    static constexpr float MAX_ALTITUDE = 30.0f;           // 최대 고도 (m)

    // ========== 유효성 검사 ==========
    static constexpr float MIN_TEMP_THRESHOLD = 50.0f;     // °C (화재 판정 온도)
    static constexpr float DT = 0.1f;                       // 10Hz 제어 주기

    // ========== 타이밍 ==========
    static constexpr float AMMO_DEPLETED_DELAY_SEC = 2.0f;
    static constexpr float DESCENT_PER_TICK = 0.05f;        // 0.5m/s 하강

    // ========== 열원 미감지 RTL ==========
    static constexpr float FIRE_TEMP_THRESHOLD = 80.0f;     // °C (화재 판정 온도)
    static constexpr float NO_FIRE_TIMEOUT_SEC = 120.0f;    // 2분간 미감지 시 RTL

    // ========== PD 제어 상태 ==========
    float prev_error_yaw_{0.0f};
    float prev_error_alt_{0.0f};
    double prev_thermal_timestamp_{0.0};
    int consecutive_invalid_{0};

    // ========== 위치/고도 ==========
    float hold_x_{0.0f}, hold_y_{0.0f};
    float final_z_{0.0f};
    float min_z_{0.0f}, max_z_{0.0f};

    // ========== 호버링 상태 ==========
    int last_log_tick_{-1};
    bool tracking_started_{false};

    // ========== 소화탄 ==========
    bool ammo_depleted_{false};
    std::chrono::steady_clock::time_point ammo_depleted_time_;

    // ========== 열원 미감지 ==========
    std::chrono::steady_clock::time_point last_fire_detected_time_;
    bool no_fire_warned_{false};
    int prev_ammo_index_{0};

    int getRemaining(const MissionContext& ctx) const {
        if (!ctx.fire_gpio_index_ptr || ctx.fire_gpio_count <= 0) return 0;
        int idx = ctx.fire_gpio_index_ptr->load();
        return std::max(0, ctx.fire_gpio_count - idx);
    }
};

#endif // TRACKING_HOVER_HANDLER_H
