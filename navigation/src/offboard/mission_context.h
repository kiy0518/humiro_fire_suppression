/**
 * @file mission_context.h
 * @brief 핸들러 간 공유 상태 구조체
 *
 * 모든 StateHandler가 참조하는 vehicle 상태, 미션 파라미터, 편대/충돌 상태.
 * OffboardManager가 소유하고, 핸들러에 참조로 전달.
 */

#ifndef MISSION_CONTEXT_H
#define MISSION_CONTEXT_H

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <rclcpp/rclcpp.hpp>

// Forward declaration (포인터만 사용, 헤더 의존성 없음)
struct ThermalData;
class LidarInterface;

// GPS 좌표 구조체
struct GPSCoordinate {
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0f;
};

// 미션 설정 구조체
struct MissionConfig {
    float takeoff_altitude = 5.0f;           // 이륙 고도 (미터)
    float target_altitude = -1.0f;           // 목표지점 고도 (미터), -1이면 takeoff_altitude 사용
    float flight_speed = 3.0f;               // 비행 속도 (m/s)
    GPSCoordinate target_waypoint;           // 목표 위치
    float hover_duration_sec = 3.0f;         // 호버링 시간 (초)
    float takeoff_max_speed = 0.2f;          // 이륙 최대 속도 (m/s)
    float takeoff_accel_tau = 1.0f;          // 이륙 가속 시정수 (초, 0→max_speed까지 걸리는 시간)
    // 이동 (Navigate)
    float nav_max_accel_xy = 1.5f;           // 수평 가속도 (m/s²)
    float nav_max_speed_z = 1.0f;            // 수직 속도 (m/s)
    // 거리조정 (Distance Adjust)
    float adjust_approach_speed = 0.3f;      // 접근 속도 (m/s)
    float adjust_target_wall_dist = 10.0f;   // 목표 벽 거리 (m)
    float adjust_retreat_dist = 4.0f;        // 후퇴 거리 (m)
    // 복귀/착륙 (RTL)
    float rtl_descent_speed = 0.4f;          // 하강 속도 (m/s)
    float rtl_soft_land_alt = 2.0f;          // 소프트랜딩 시작 고도 (m AGL)
    float rtl_landing_speed_min = 0.05f;     // 최종 착지 속도 (m/s)
    // 조준 데드존 (Aiming)
    int deadzone_h_px = 33;                  // 수평 데드존 (px, 10m 기준 ~69cm)
    int deadzone_v_px = 33;                  // 수직 데드존 (px, 10m 기준 ~71cm)
};

struct MissionContext {
    // === Vehicle 상태 (PX4 구독에서 업데이트) ===
    std::atomic<float> current_local_x{0.0f};
    std::atomic<float> current_local_y{0.0f};
    std::atomic<float> current_local_z{0.0f};
    std::atomic<float> current_yaw{0.0f};
    std::atomic<double> current_lat{0.0};
    std::atomic<double> current_lon{0.0};
    std::atomic<float> current_alt_amsl{0.0f};
    std::atomic<float> actual_vx{0.0f};
    std::atomic<float> actual_vy{0.0f};
    std::atomic<float> actual_vz{0.0f};
    std::atomic<bool> position_received{false};

    // === 하방 거리계 ===
    std::atomic<float> dist_bottom{-1.0f};       // 지면까지 거리 (m), 무효 시 -1
    std::atomic<bool> dist_bottom_valid{false};

    // === FC 상태 ===
    std::atomic<uint8_t> nav_state{0};       // 14=OFFBOARD, 5=AUTO_RTL
    std::atomic<uint8_t> arming_state{0};    // 2=ARMED, 1=DISARMED
    std::atomic<bool> land_detected{false};  // PX4 vehicle_land_detected.landed
    std::atomic<bool> maybe_landed{false};   // PX4 vehicle_land_detected.maybe_landed

    // === RTL 서브페이즈 (OSD 표시용) ===
    std::string rtl_sub_phase;               // "DESCEND", "SOFT_LAND", "GROUND_DISARM", "" (빈 문자열=표시 안함)

    // === 미션 파라미터 ===
    float takeoff_altitude{5.0f};
    float target_altitude{-1.0f};
    float flight_speed{3.0f};
    float hover_duration_sec{3.0f};
    float takeoff_max_speed{0.2f};           // 이륙 최대 속도 (m/s)
    float takeoff_accel_tau{1.0f};           // 이륙 가속 시정수 (초)
    float nav_max_accel_xy{1.5f};            // 수평 가속도 (m/s²)
    float nav_max_speed_z{1.0f};             // 수직 속도 (m/s)
    float adjust_approach_speed{0.3f};       // 거리조정 접근 속도 (m/s)
    float adjust_target_wall_dist{10.0f};    // 목표 벽 거리 (m)
    float adjust_retreat_dist{4.0f};         // 후퇴 거리 (m)
    float rtl_descent_speed{0.4f};           // 하강 속도 (m/s)
    float rtl_soft_land_alt{2.0f};           // 소프트랜딩 시작 고도 (m AGL)
    float rtl_landing_speed_min{0.05f};      // 최종 착지 속도 (m/s)
    int deadzone_h_px{33};                   // 조준 데드존 수평 (px)
    int deadzone_v_px{33};                   // 조준 데드존 수직 (px)
    double target_lat{0.0};
    double target_lon{0.0};

    // === 기준 위치 (ARM 확인 시 캡처) ===
    float start_local_x{0.0f};
    float start_local_y{0.0f};
    float start_local_z{0.0f};
    float initial_yaw{0.0f};

    // === 목표 NED 좌표 ===
    float target_ned_x{0.0f};
    float target_ned_y{0.0f};
    float target_ned_z{0.0f};
    float target_yaw{0.0f};

    // === Home 위치 ===
    double home_lat{0.0};
    double home_lon{0.0};
    float home_alt_amsl{0.0f};
    bool home_set{false};

    // === Velocity 피드포워드 (NAVIGATE) — DEPRECATED by MotionProfile ===
    // navigate_handler가 MotionProfile3D로 전환됨. 하위 호환성을 위해 유지.
    float prev_vx{0.0f};
    float prev_vy{0.0f};

    // === 편대 ===
    bool formation_mode{false};
    std::atomic<bool> formation_ready_to_rotate{false};
    std::atomic<bool> formation_ready_to_navigate{false};
    std::atomic<bool> continuous_update_mode{false};
    std::atomic<bool> force_navigate_complete{false};   // CMD_SUPPRESS → 팔로워 NAVIGATE 완료 허용

    // === Swarm 모드 (독립 편대 비행) ===
    bool swarm_mode{false};                              // Swarm 미션 모드 활성
    std::atomic<bool> swarm_alignment_received{false};   // 리더 정렬 완료 수신 (팔로워)
    bool swarm_reposition_needed{false};                 // 진압 위치로 재이동 필요 (팔로워)
    double swarm_suppress_lat{0.0};                      // 진압 목표 GPS 위도
    double swarm_suppress_lon{0.0};                      // 진압 목표 GPS 경도
    float swarm_suppress_yaw{NAN};                       // 진압 시 헤딩 (화점 방향, rad)

    // === 소화탄 상태 (ApplicationManager가 설정) ===
    std::atomic<int>* fire_gpio_index_ptr{nullptr};  // 현재 발사 인덱스 포인터
    int fire_gpio_count{0};                           // 총 소화탄 수 (6)

    // === 열원 추적 (thermal tracking) ===
    ThermalData* thermal_data_ptr{nullptr};               // ApplicationManager의 thermal_data_ 포인터
    std::atomic<bool> thermal_tracking_auto{true};         // 자동 모드 (true=자동, false=수동)
    std::atomic<bool> thermal_tracking_active{false};      // 추적 활성 (수동 모드: GUI 버튼으로 제어)

    // === 자동 격발 (60001 FIRE_AUTO_AIM) ===
    std::atomic<bool> auto_fire_enabled{false};            // 60001 수신 → true
    std::atomic<bool> auto_fire_request{false};            // 핸들러 → ApplicationManager 격발 요청

    // === LiDAR 인터페이스 (ApplicationManager가 설정) ===
    LidarInterface* lidar_ptr{nullptr};

    // === 거리 조정 결과 (리더 전용) ===
    struct DistanceAdjustResult {
        bool completed{false};           // 거리 조정 완료 여부
        double final_lat{0.0};           // 최종 GPS 위도
        double final_lon{0.0};           // 최종 GPS 경도
        float final_alt_amsl{0.0f};     // 최종 고도 (AMSL)
        float final_yaw{0.0f};          // 최종 헤딩 (벽 수직, rad)
        float wall_distance{0.0f};      // 벽까지 거리 (m)
    } distance_adjust_result;

    // === 충돌 방지 ===
    std::atomic<int> collision_action{0};  // 0=NONE, 1=HOLD, 2=EVADE_RIGHT
    float hold_x{0.0f}, hold_y{0.0f}, hold_z{0.0f}, hold_yaw{0.0f};
    float evade_offset_n{0.0f}, evade_offset_e{0.0f};

    // === 타이밍 ===
    std::chrono::steady_clock::time_point state_enter_time;

    // === 드론 ID ===
    uint8_t target_system{1};

    // === 제어 상수 ===
    static constexpr float MAX_YAW_RATE = 0.3f;       // rad/s (~17.2 deg/s) — 중량 기체 안전값
    static constexpr float WAYPOINT_THRESHOLD = 2.0f;  // m

    // === 명령 발행 콜백 (OffboardManager가 설정) ===
    std::function<void(uint16_t cmd, float p1, float p2, float p3)> publishCommand;

    // === 로거 ===
    rclcpp::Logger logger{rclcpp::get_logger("offboard")};

    // === 유틸리티 ===

    /** 현재 상태 진입 후 경과 시간 (초) */
    double elapsedSec() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - state_enter_time).count();
    }

    /** 현재 위치에서 목표까지 수평 거리 (m) */
    float distanceToTarget() const {
        float dx = target_ned_x - current_local_x.load();
        float dy = target_ned_y - current_local_y.load();
        return std::sqrt(dx * dx + dy * dy);
    }

    /** MissionConfig에서 컨텍스트로 파라미터 복사 */
    void loadFromConfig(const MissionConfig& config) {
        takeoff_altitude = config.takeoff_altitude;
        target_altitude = config.target_altitude;
        flight_speed = config.flight_speed;
        hover_duration_sec = config.hover_duration_sec;
        target_lat = config.target_waypoint.latitude;
        target_lon = config.target_waypoint.longitude;
        takeoff_max_speed = config.takeoff_max_speed;
        takeoff_accel_tau = config.takeoff_accel_tau;
        nav_max_accel_xy = config.nav_max_accel_xy;
        nav_max_speed_z = config.nav_max_speed_z;
        adjust_approach_speed = config.adjust_approach_speed;
        adjust_target_wall_dist = config.adjust_target_wall_dist;
        adjust_retreat_dist = config.adjust_retreat_dist;
        rtl_descent_speed = config.rtl_descent_speed;
        rtl_soft_land_alt = config.rtl_soft_land_alt;
        rtl_landing_speed_min = config.rtl_landing_speed_min;
        deadzone_h_px = config.deadzone_h_px;
        deadzone_v_px = config.deadzone_v_px;
    }

    /** 컨텍스트 초기화 (새 미션 시작 시) */
    void reset() {
        prev_vx = 0.0f;
        prev_vy = 0.0f;
        formation_ready_to_rotate.store(false);
        formation_ready_to_navigate.store(false);
        force_navigate_complete.store(false);
        thermal_tracking_active.store(false);
        collision_action.store(0);
        hold_x = hold_y = hold_z = hold_yaw = 0.0f;
        evade_offset_n = evade_offset_e = 0.0f;
        home_set = false;
        target_ned_x = target_ned_y = target_ned_z = target_yaw = 0.0f;
        distance_adjust_result = DistanceAdjustResult{};
        // 자동 격발 리셋
        auto_fire_enabled.store(false);
        auto_fire_request.store(false);
        // Swarm 모드 리셋
        swarm_alignment_received.store(false);
        swarm_reposition_needed = false;
        swarm_suppress_lat = 0.0;
        swarm_suppress_lon = 0.0;
        swarm_suppress_yaw = NAN;
    }
};

#endif // MISSION_CONTEXT_H
