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

// Forward declaration (포인터만 사용, thermal 헤더 의존성 없음)
struct ThermalData;

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
    float flight_speed = 5.0f;               // 비행 속도 (m/s)
    GPSCoordinate target_waypoint;           // 목표 위치
    float hover_duration_sec = 3.0f;         // 호버링 시간 (초)
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

    // === FC 상태 ===
    std::atomic<uint8_t> nav_state{0};       // 14=OFFBOARD, 5=AUTO_RTL
    std::atomic<uint8_t> arming_state{0};    // 2=ARMED, 1=DISARMED

    // === 미션 파라미터 ===
    float takeoff_altitude{5.0f};
    float target_altitude{-1.0f};
    float flight_speed{5.0f};
    float hover_duration_sec{3.0f};
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

    // === Velocity 피드포워드 (NAVIGATE) ===
    float prev_vx{0.0f};
    float prev_vy{0.0f};

    // === 편대 ===
    bool formation_mode{false};
    std::atomic<bool> formation_ready_to_rotate{false};
    std::atomic<bool> formation_ready_to_navigate{false};
    std::atomic<bool> continuous_update_mode{false};

    // === 소화탄 상태 (ApplicationManager가 설정) ===
    std::atomic<int>* fire_gpio_index_ptr{nullptr};  // 현재 발사 인덱스 포인터
    int fire_gpio_count{0};                           // 총 소화탄 수 (6)

    // === 열원 추적 (thermal tracking) ===
    ThermalData* thermal_data_ptr{nullptr};               // ApplicationManager의 thermal_data_ 포인터
    std::atomic<bool> thermal_tracking_auto{true};         // 자동 모드 (true=자동, false=수동)
    std::atomic<bool> thermal_tracking_active{false};      // 추적 활성 (수동 모드: GUI 버튼으로 제어)

    // === 충돌 방지 ===
    std::atomic<int> collision_action{0};  // 0=NONE, 1=HOLD, 2=EVADE_RIGHT
    float hold_x{0.0f}, hold_y{0.0f}, hold_z{0.0f}, hold_yaw{0.0f};
    float evade_offset_n{0.0f}, evade_offset_e{0.0f};

    // === 타이밍 ===
    std::chrono::steady_clock::time_point state_enter_time;

    // === 드론 ID ===
    uint8_t target_system{1};

    // === 제어 상수 ===
    static constexpr float MAX_YAW_RATE = 0.5f;       // rad/s (~28.6 deg/s)
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
    }

    /** 컨텍스트 초기화 (새 미션 시작 시) */
    void reset() {
        prev_vx = 0.0f;
        prev_vy = 0.0f;
        formation_ready_to_rotate.store(false);
        formation_ready_to_navigate.store(false);
        thermal_tracking_active.store(false);
        collision_action.store(0);
        hold_x = hold_y = hold_z = hold_yaw = 0.0f;
        evade_offset_n = evade_offset_e = 0.0f;
        home_set = false;
        target_ned_x = target_ned_y = target_ned_z = target_yaw = 0.0f;
    }
};

#endif // MISSION_CONTEXT_H
