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

    // timerCallback()에서 10Hz로 호출. CollisionAction 반환
    CollisionAction checkAndUpdate();

    bool isCollisionHold() const { return collision_hold_.load(); }
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

    // 우선순위 판정: 낮은 ID = 높은 우선순위
    CollisionAction determineAction(uint8_t my_id, uint8_t other_id,
                                     bool i_am_approaching, bool other_is_approaching) const;

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
    std::atomic<uint8_t> current_threat_id_{0};
    std::atomic<float> evade_offset_n_{0.0f};
    std::atomic<float> evade_offset_e_{0.0f};

    // 상수
    static constexpr float WARNING_DISTANCE = 12.0f;     // 경고 (m)
    static constexpr float DANGER_DISTANCE = 8.0f;      // 위험 → 행동 (m)
    static constexpr float SAFE_DISTANCE = 15.0f;       // 안전 → 재개 (m)
    static constexpr float ALTITUDE_CLEARANCE = 5.0f;   // 고도 차이 이상이면 안전 (m)
    static constexpr float APPROACH_SPEED_THRESHOLD = 0.1f;  // 접근 판정 임계 (m/s)
    static constexpr float EVADE_OFFSET_M = 5.0f;       // 회피 오프셋 (m)
    static constexpr int STALE_TIMEOUT_MS = 2000;        // 데이터 유효 시간 (ms)
    static constexpr double DEG_TO_M_LAT = 111320.0;
};

#endif // COLLISION_AVOIDANCE_H
