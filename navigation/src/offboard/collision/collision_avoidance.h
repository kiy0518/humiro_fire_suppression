#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include <rclcpp/rclcpp.hpp>
#include <humiro_msgs/msg/drone_position.hpp>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <cmath>

class OffboardManager;

enum class CollisionAction : int {
    NONE = 0,
    HOLD = 1,          // 높은 우선순위 (낮은 ID) → 제자리 정지
    EVADE_RIGHT = 2    // 낮은 우선순위 (높은 ID) → 경로 오른쪽 회피
};

struct DroneState {
    uint8_t drone_id{0};
    double latitude{0.0};
    double longitude{0.0};
    float altitude{0.0f};     // MSL
    float vx{0.0f}, vy{0.0f}, vz{0.0f};   // NED 속도
    double home_latitude{0.0};   // 상대 드론의 홈 위치
    double home_longitude{0.0};
    std::string mission_state;
    std::chrono::steady_clock::time_point last_update;
    bool valid{false};
};

class CollisionAvoidance {
public:
    CollisionAvoidance(rclcpp::Node::SharedPtr node,
                       OffboardManager* offboard_mgr,
                       uint8_t drone_id);
    ~CollisionAvoidance();

    void start();
    void stop();

    // 거리 설정 (GUI offboard_config.json에서 로드)
    void setDistances(float danger, float warning, float safe) {
        DANGER_DISTANCE_ = danger;
        WARNING_DISTANCE_ = warning;
        SAFE_DISTANCE_ = safe;
        RCLCPP_INFO(node_->get_logger(),
            "[CollisionAvoid] Distances updated: DANGER=%.1fm, WARNING=%.1fm, SAFE=%.1fm",
            danger, warning, safe);
    }

    // timerCallback()에서 10Hz로 호출. CollisionAction 반환
    CollisionAction checkAndUpdate();

    bool isCollisionHold() const { return collision_hold_.load(); }
    bool isWarningActive() const { return warning_active_.load(); }
    uint8_t getThreatId() const { return current_threat_id_.load(); }

    // EVADE_RIGHT 회피 오프셋 (NED, 경로 직각 오른쪽 5m)
    float getEvadeOffsetN() const { return evade_offset_n_.load(); }
    float getEvadeOffsetE() const { return evade_offset_e_.load(); }

private:
    // 5Hz 위치 브로드캐스트
    void positionBroadcastCallback();

    // 타 드론 위치 수신
    void onDronePosition(const humiro_msgs::msg::DronePosition::SharedPtr msg);

    // GPS 기반 수평 거리 (m)
    float calculateHorizontalGPSDistance(double lat1, double lon1,
                                          double lat2, double lon2) const;

    // 고도 차이가 충분한지 (충돌 위험 없음)
    bool hasAltitudeClearance(float alt1, float alt2) const;

    // 내가 상대를 향해 접근 중인지 (속도 벡터 투영)
    bool isIApproachingOther(const DroneState& me, const DroneState& other) const;

    // 우선순위 판정: 홈 거리 기반 (가까운 쪽 진행) + ID 타이브레이크
    CollisionAction determineAction(uint8_t my_id, uint8_t other_id,
                                     bool i_am_approaching, bool other_is_approaching,
                                     float my_home_dist, float other_home_dist);

    // EVADE 오프셋 계산 (현재 heading 기준 오른쪽 5m)
    void computeEvadeOffset();

    // ROS2
    rclcpp::Node::SharedPtr node_;
    OffboardManager* offboard_mgr_;
    uint8_t drone_id_;
    std::atomic<bool> running_{false};

    // Publisher / Subscriber
    rclcpp::Publisher<humiro_msgs::msg::DronePosition>::SharedPtr position_pub_;
    rclcpp::Subscription<humiro_msgs::msg::DronePosition>::SharedPtr position_sub_;
    rclcpp::TimerBase::SharedPtr broadcast_timer_;  // 5Hz

    // 타 드론 상태
    std::mutex drones_mutex_;
    std::map<uint8_t, DroneState> other_drones_;

    // 충돌 상태
    std::atomic<bool> collision_hold_{false};
    std::atomic<bool> warning_active_{false};
    std::atomic<uint8_t> current_threat_id_{0};
    std::atomic<float> evade_offset_n_{0.0f};
    std::atomic<float> evade_offset_e_{0.0f};

    // 히스테리시스: 한번 정해진 우선순위를 거리 차이 역전 시까지 유지
    // locked_priority_holder_: 충돌 쌍에서 진행권을 가진 드론 ID (0=미결정)
    uint8_t locked_priority_holder_{0};

    // 거리 설정 (GUI에서 변경 가능)
    float WARNING_DISTANCE_ = 12.5f;     // 경고 → 감속 (m)
    float DANGER_DISTANCE_ = 10.0f;      // 위험 → HOLD (m)
    float SAFE_DISTANCE_ = 15.0f;        // 안전 → 재개 (m)
    static constexpr float ALTITUDE_CLEARANCE = 5.0f;   // 고도 차이 이상이면 안전 (m)
    static constexpr float APPROACH_SPEED_THRESHOLD = 0.1f;  // 접근 판정 임계 (m/s)
    static constexpr float EVADE_OFFSET_M = 5.0f;       // 회피 오프셋 (m)
    static constexpr int STALE_TIMEOUT_MS = 2000;        // 데이터 유효 시간 (ms)
    static constexpr float HOME_DIST_HYSTERESIS = 5.0f; // 거리 차이 이내면 기존 우선순위 유지 (m)
    static constexpr double DEG_TO_M_LAT = 111320.0;
};

#endif // COLLISION_AVOIDANCE_H
