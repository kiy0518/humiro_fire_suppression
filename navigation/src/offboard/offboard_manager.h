/**
 * @file offboard_manager.h
 * @brief PX4 Offboard 제어 - 핸들러 기반 아키텍처
 *
 * PX4 공식 문서 기반:
 * - OffboardControlMode heartbeat를 2Hz 이상 지속적으로 발행 (10Hz 사용)
 * - TrajectorySetpoint으로 위치 제어
 * - Arming 전부터 메시지 발행 필요
 *
 * 각 상태(PREPARE, ARM, TAKEOFF 등)를 독립 핸들러로 분리.
 * 조건 기반 전환 (카운터 기반 → nav_state, arming_state, 고도, 거리 확인).
 *
 * v2.0: DDS Domain 분리 - FCBridgeClient를 통해 FC와 통신 (px4_msgs 제거)
 */

#ifndef OFFBOARD_MANAGER_H
#define OFFBOARD_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <memory>

#include "bridge/fc_bridge_protocol.h"
#include "bridge/fc_bridge_client.h"
#include "mission_context.h"
#include "handlers/state_handler.h"

// 미션 상태
enum class MissionState {
    IDLE,           // 대기
    PREPARING,      // heartbeat 발행 중 (2초)
    OFFBOARD,       // OFFBOARD 모드 전환
    ARMING,         // ARM 요청
    TAKEOFF,        // 이륙 중
    HOVER,          // 호버링 (안정화)
    ROTATE,         // 목표 방향 회전
    NAVIGATE,       // 목표 위치 이동
    DISTANCE_ADJUST,// LiDAR 벽 감지 + 10m 거리 확보
    HOVER_AT_TARGET,// 목표지점 호버링 (5초)
    RTL,            // 귀환
    LANDED,         // 착륙 완료
    ERROR           // 에러
};

class OffboardManager {
public:
    /**
     * @param node ROS2 노드 (Domain 1, 편대 토픽용)
     * @param fc_bridge FC Bridge 클라이언트 (로컬 UDP IPC)
     */
    explicit OffboardManager(rclcpp::Node::SharedPtr node,
                             std::shared_ptr<FCBridgeClient> fc_bridge);
    ~OffboardManager();

    /**
     * @brief 단독비행 미션 (블로킹)
     * 순서: PREPARING → OFFBOARD → ARMING → TAKEOFF → HOVER → ROTATE → NAVIGATE → HOVER_AT_TARGET → RTL
     * @param config 미션 설정
     * @return 미션 성공 여부
     */
    bool executeMissionSolo(const MissionConfig& config);

    /**
     * @brief 편대비행 미션 (블로킹)
     * 편대 게이트 동기화 + 팔로워 연동 + DISTANCE_ADJUST + TRACKING_HOVER
     * @param config 미션 설정
     * @return 미션 성공 여부
     */
    bool executeMissionFormation(const MissionConfig& config);

    /**
     * @brief Swarm 독립 편대 미션 (블로킹)
     * 편대 게이트 없음. 각 기체 독립 비행.
     * 팔로워: 리더 정렬 알림 수신 후 진압 위치로 재이동.
     * @param config 미션 설정
     * @return 미션 성공 여부
     */
    bool executeMissionSwarm(const MissionConfig& config);

    /**
     * @brief Swarm 진압 위치 설정 (FormationController → OffboardManager)
     * CMD_ALIGN_COMPLETE 수신 시 호출. 팔로워가 진압 위치로 재이동.
     */
    void setSwarmSuppressTarget(double lat, double lon, float yaw);

    /**
     * @brief 미션 진행 중 목표 좌표 업데이트 (경로 변경)
     * @param new_target 새 목표 GPS 좌표
     * @param yaw_override 헤딩 오버라이드 (rad). NAN이면 목표방향 자동계산
     * @return 업데이트 성공 여부 (미션 진행 중일 때만 true)
     */
    bool updateMissionTarget(const GPSCoordinate& new_target, float yaw_override = NAN);

    /**
     * @brief 미션 진행 중인지 확인
     */
    bool isMissionRunning() const { return mission_running_.load(); }

    /**
     * @brief 미션 중단 요청
     */
    void abortMission();

    /**
     * @brief 긴급 RTL
     */
    void emergencyRTL();

    /**
     * @brief 현재 상태 반환
     */
    MissionState getCurrentState() const { return current_state_.load(); }

    /**
     * @brief 미션 컨텍스트 참조 반환
     */
    MissionContext& getContext() { return ctx_; }

    /**
     * @brief 상태 이름 반환
     */
    static std::string getStateName(MissionState state);

    /**
     * @brief IDLE 상태로 리셋
     */
    void resetToIdle();

    // ========== 소화탄 상태 연결 (ApplicationManager 연동) ==========
    void setFireAmmoState(std::atomic<int>* index_ptr, int total_count);

    // ========== 열원 추적 (ApplicationManager 연동) ==========
    void setThermalData(struct ThermalData* data_ptr);
    void setThermalTrackingMode(bool auto_mode);     // 자동/수동 모드
    void setThermalTrackingActive(bool active);       // 수동 모드 트리거

    // ========== LiDAR 인터페이스 (ApplicationManager 연동) ==========
    void setLidarInterface(class LidarInterface* lidar_ptr);

    // ========== 편대 비행 게이트 (FormationController 연동) ==========
    void setFormationReadyToRotate(bool ready);
    void setFormationReadyToNavigate(bool ready);
    void setContinuousUpdateMode(bool enabled);
    void setForceNavigateComplete(bool force);

    // ========== 위치/속도 Getter (FormationController용) ==========
    double getCurrentLat() const { return current_lat_.load(); }
    double getCurrentLon() const { return current_lon_.load(); }
    float getCurrentAltAmsl() const { return current_alt_amsl_.load(); }
    float getCurrentYaw() const { return current_yaw_.load(); }
    float getCurrentLocalX() const { return current_local_x_.load(); }
    float getCurrentLocalY() const { return current_local_y_.load(); }
    float getCurrentLocalZ() const { return current_local_z_.load(); }
    float getCurrentVx() const { return actual_vx_.load(); }
    float getCurrentVy() const { return actual_vy_.load(); }
    float getCurrentVz() const { return actual_vz_.load(); }
    float getCurrentSpeed() const {
        float vx = actual_vx_.load(), vy = actual_vy_.load();
        return std::sqrt(vx * vx + vy * vy);
    }

    // ========== 타겟 위치 Getter ==========
    float getTargetNedX() const { return target_ned_x_; }
    float getTargetNedY() const { return target_ned_y_; }

    // ========== DISTANCE_ADJUST 결과 접근 (FormationController용) ==========
    bool getDistanceAdjustCompleted() const { return ctx_.distance_adjust_result.completed; }
    const MissionContext::DistanceAdjustResult& getDistanceAdjustResult() const {
        return ctx_.distance_adjust_result;
    }

    // ========== 소화탄 잔여 확인 ==========
    bool hasAmmoRemaining() const {
        return ctx_.fire_gpio_index_ptr && ctx_.fire_gpio_count > 0 &&
               ctx_.fire_gpio_index_ptr->load() < ctx_.fire_gpio_count;
    }

    // ========== 충돌 방지 ==========
    void setCollisionAvoidance(class CollisionAvoidance* ca) { collision_avoidance_ = ca; }
    bool isCollisionActive() const { return collision_action_.load() != 0; }
    int getCollisionAction() const { return collision_action_.load(); }

    // ========== MAVLink DISTANCE_SENSOR (PX4 EKF 우회, 거리계 원시값) ==========
    void setMavlinkDistBottom(float distance_m) {
        mavlink_dist_bottom_.store(distance_m);
        mavlink_dist_bottom_valid_.store(distance_m > 0.0f);
    }

    // ========== FC Bridge 접근 (StatusROS2Subscriber 등에서 사용) ==========
    FCBridgeClient* getFCBridge() const { return fc_bridge_.get(); }

    // ========== FC 상태 Getter (FCBridgeClient에서 가져온 데이터) ==========
    float getBatteryRemaining() const { return battery_remaining_.load(); }
    uint8_t getGPSFixType() const { return gps_fix_type_.load(); }
    uint8_t getGPSSatellites() const { return gps_satellites_.load(); }

private:
    // ========== 미션 실행 공통 로직 ==========
    bool executeMissionInternal(const MissionConfig& config);

    // ========== 타이머 콜백 (핵심!) ==========
    void timerCallback();

    // ========== FC Bridge를 통한 메시지 발행 ==========
    void publishOffboardControlMode();
    void publishSetpoint(const FCCommand& cmd);
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f);

    // ========== OSD 상태 발행 (/offboard/status) ==========
    void publishOffboardStatus(const std::string& status);
    std::string missionStateToStatusString(MissionState state);

    // ========== FC 상태 업데이트 (FCBridgeClient에서 폴링) ==========
    void updateFromFCState();

    // ========== 헬퍼 함수 ==========
    float calculateTargetYaw(float target_north, float target_east);
    void gpsToLocalNED(double target_lat, double target_lon, float target_alt,
                       float& local_x, float& local_y, float& local_z);

    // ========== ROS2 노드 (Domain 1, 편대 토픽용) ==========
    rclcpp::Node::SharedPtr node_;

    // ========== OSD 상태 퍼블리셔 ==========
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offboard_status_pub_;
    std::string last_published_status_;

    // ========== FC Bridge 클라이언트 (로컬 UDP IPC) ==========
    std::shared_ptr<FCBridgeClient> fc_bridge_;

    // ========== Timer ==========
    rclcpp::TimerBase::SharedPtr timer_;

    // ========== 상태 ==========
    std::atomic<MissionState> current_state_{MissionState::IDLE};
    std::atomic<uint64_t> setpoint_counter_{0};
    std::atomic<bool> abort_requested_{false};
    std::atomic<bool> mission_running_{false};
    std::mutex mission_start_mutex_;  // 중복 미션 시작 방지 (executeMissionSolo 진입 보호)

    // ========== FC 상태 ==========
    std::atomic<uint8_t> nav_state_{0};
    std::atomic<uint8_t> arming_state_{0};
    std::atomic<bool> land_detected_{false};
    std::atomic<bool> maybe_landed_{false};

    // ========== 위치 데이터 (atomic) ==========
    std::atomic<float> current_local_x_{0.0f};
    std::atomic<float> current_local_y_{0.0f};
    std::atomic<float> current_local_z_{0.0f};
    std::atomic<float> current_yaw_{0.0f};
    std::atomic<double> current_lat_{0.0};
    std::atomic<double> current_lon_{0.0};
    std::atomic<float> current_alt_amsl_{0.0f};
    std::atomic<bool> position_received_{false};
    std::atomic<float> actual_vx_{0.0f};    // PX4 실제 NED 속도
    std::atomic<float> actual_vy_{0.0f};
    std::atomic<float> actual_vz_{0.0f};
    std::atomic<float> current_pitch_{0.0f};  // radians
    std::atomic<float> dist_bottom_{-1.0f};    // 하방 거리계 (m) (PX4 EKF 경유)
    std::atomic<bool> dist_bottom_valid_{false};
    std::atomic<float> mavlink_dist_bottom_{-1.0f};    // MAVLink DISTANCE_SENSOR 직접값
    std::atomic<bool> mavlink_dist_bottom_valid_{false};

    // ========== 배터리/GPS (FCBridgeClient에서 수신) ==========
    std::atomic<float> battery_remaining_{0.0f};
    std::atomic<uint8_t> gps_fix_type_{0};
    std::atomic<uint8_t> gps_satellites_{0};

    // ========== Home 위치 ==========
    double home_lat_{0.0};
    double home_lon_{0.0};
    float home_alt_amsl_{0.0f};
    bool home_set_{false};

    // ========== 미션 설정 ==========
    MissionConfig mission_config_;
    float initial_yaw_{0.0f};           // 이륙 시점 헤딩

    // ========== 시작 위치 (미션 시작 시점 로컬 NED) ==========
    float start_local_x_{0.0f};
    float start_local_y_{0.0f};
    float start_local_z_{0.0f};

    // ========== 목표 NED 좌표 ==========
    float target_ned_x_{0.0f};
    float target_ned_y_{0.0f};
    float target_ned_z_{0.0f};
    float target_yaw_{0.0f};

    // ========== Velocity 보간용 (부드러운 선회) ==========
    float prev_vx_{0.0f};
    float prev_vy_{0.0f};

    // ========== 목표지점 호버링 ==========
    std::atomic<uint64_t> hover_at_target_start_{0};

    // ========== 타이밍 상수 (10Hz 기준) ==========
    static constexpr uint64_t PREPARE_COUNT = 20;    // 2초: heartbeat 준비
    static constexpr uint64_t ARM_COUNT = 45;        // 4.5초: ARM
    static constexpr uint64_t TAKEOFF_STABLE = 80;   // 8초: 이륙 안정화
    static constexpr uint64_t ROTATE_START = 110;    // 11초: 회전 시작
    static constexpr uint64_t ROTATE_END = 170;      // 17초: 회전 완료
    static constexpr uint64_t MOVE_START = 180;      // 18초: 이동 시작
    static constexpr uint64_t TARGET_HOVER_TICKS = 50; // 5초: 목표지점 호버링 (10Hz)
    static constexpr uint64_t SUPPRESS_HOVER_TICKS = 300; // 30초: 진압 편대 대기 (10Hz)

    // ========== 군집비행 모드 ==========
    bool formation_mode_{false};
    bool swarm_mode_{false};       // Swarm 독립 편대 모드
    std::atomic<bool> formation_ready_to_rotate_{false};
    std::atomic<bool> formation_ready_to_navigate_{false};
    std::atomic<bool> continuous_update_mode_{false};

    // ========== 드론 식별 ==========
    uint8_t target_system_{1};  // PX4 MAV_SYS_ID (VehicleCommand target)

    // ========== 제어 파라미터 ==========
    static constexpr float MAX_YAW_RATE = 0.5f;      // 최대 회전 속도 (rad/s, ~28.6 deg/s)
    static constexpr float WAYPOINT_THRESHOLD = 2.0f; // 도착 판정 거리 (m)

    // ========== OFFBOARD 모드 이탈 디바운스 (RC OVERRIDE 감지) ==========
    std::chrono::steady_clock::time_point offboard_lost_start_{};
    bool offboard_lost_tracking_{false};
    int offboard_recovery_count_{0};

    // ========== 충돌 방지 ==========
    class CollisionAvoidance* collision_avoidance_{nullptr};
    std::atomic<int> collision_action_{0};  // 0=NONE, 1=HOLD, 2=EVADE_RIGHT
    bool was_collision_active_{false};
    float hold_x_{0.0f}, hold_y_{0.0f}, hold_z_{0.0f}, hold_yaw_{0.0f};
    float evade_offset_n_{0.0f}, evade_offset_e_{0.0f};  // 우회 오프셋 (NED)

    // ========== 핸들러 아키텍처 (점진적 전환) ==========
    MissionContext ctx_;
    std::unique_ptr<StateHandler> prepare_handler_;
    std::unique_ptr<StateHandler> offboard_handler_;
    std::unique_ptr<StateHandler> arm_handler_;
    std::unique_ptr<StateHandler> takeoff_handler_;
    std::unique_ptr<StateHandler> hover_handler_;
    std::unique_ptr<StateHandler> rotate_handler_;
    std::unique_ptr<StateHandler> navigate_handler_;
    std::unique_ptr<StateHandler> distance_adjust_handler_;
    std::unique_ptr<StateHandler> hover_at_target_handler_;
    std::unique_ptr<StateHandler> tracking_hover_handler_;
    std::unique_ptr<StateHandler> rtl_handler_;
    StateHandler* current_handler_{nullptr};  // 현재 활성 핸들러 (소유권 없음)

    void transitionTo(StateHandler* handler);
    void advanceToNextHandler();
    void syncContextFromMembers();   // 기존 멤버 → ctx_ 동기화
    void syncMembersFromContext();   // ctx_ → 기존 멤버 동기화
};

#endif // OFFBOARD_MANAGER_H
