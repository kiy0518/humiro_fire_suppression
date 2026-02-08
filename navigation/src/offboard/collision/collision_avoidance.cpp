#include "collision_avoidance.h"
#include "../offboard_manager.h"

using namespace std::chrono_literals;

CollisionAvoidance::CollisionAvoidance(
    rclcpp::Node::SharedPtr node,
    OffboardManager* offboard_mgr,
    uint8_t drone_id)
    : node_(node), offboard_mgr_(offboard_mgr), drone_id_(drone_id)
{
    // QoS: BestEffort (LeaderPose와 동일)
    auto qos = rclcpp::QoS(10).best_effort();

    position_pub_ = node_->create_publisher<humiro_msgs::msg::DronePosition>(
        "/formation/drone_position", qos);

    position_sub_ = node_->create_subscription<humiro_msgs::msg::DronePosition>(
        "/formation/drone_position", qos,
        std::bind(&CollisionAvoidance::onDronePosition, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
        "[CollisionAvoid] Initialized for drone %d (DANGER=%.0fm, SAFE=%.0fm, ALT_CLR=%.0fm)",
        drone_id_, DANGER_DISTANCE, SAFE_DISTANCE, ALTITUDE_CLEARANCE);
}

CollisionAvoidance::~CollisionAvoidance() {
    stop();
}

void CollisionAvoidance::start() {
    if (running_.load()) return;
    running_.store(true);

    // 5Hz (200ms) 위치 브로드캐스트
    broadcast_timer_ = node_->create_wall_timer(
        200ms,
        std::bind(&CollisionAvoidance::positionBroadcastCallback, this));

    RCLCPP_INFO(node_->get_logger(), "[CollisionAvoid] Started (5Hz broadcast)");
}

void CollisionAvoidance::stop() {
    running_.store(false);
    if (broadcast_timer_) {
        broadcast_timer_->cancel();
        broadcast_timer_.reset();
    }
    collision_hold_.store(false);
    RCLCPP_INFO(node_->get_logger(), "[CollisionAvoid] Stopped");
}

// ===== 5Hz 위치 브로드캐스트 =====
void CollisionAvoidance::positionBroadcastCallback() {
    if (!offboard_mgr_) return;

    humiro_msgs::msg::DronePosition msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.drone_id = drone_id_;
    msg.latitude = offboard_mgr_->getCurrentLat();
    msg.longitude = offboard_mgr_->getCurrentLon();
    msg.altitude = offboard_mgr_->getCurrentAltAmsl();
    msg.vx = offboard_mgr_->getCurrentVx();
    msg.vy = offboard_mgr_->getCurrentVy();
    msg.vz = offboard_mgr_->getCurrentVz();
    msg.mission_state = OffboardManager::getStateName(offboard_mgr_->getCurrentState());

    position_pub_->publish(msg);
}

// ===== 타 드론 위치 수신 =====
void CollisionAvoidance::onDronePosition(
    const humiro_msgs::msg::DronePosition::SharedPtr msg)
{
    // 자기 자신 필터링
    if (msg->drone_id == drone_id_) return;

    std::lock_guard<std::mutex> lock(drones_mutex_);
    auto& state = other_drones_[msg->drone_id];
    state.drone_id = msg->drone_id;
    state.latitude = msg->latitude;
    state.longitude = msg->longitude;
    state.altitude = msg->altitude;
    state.vx = msg->vx;
    state.vy = msg->vy;
    state.vz = msg->vz;
    state.mission_state = msg->mission_state;
    state.last_update = std::chrono::steady_clock::now();
    state.valid = true;
}

// ===== 10Hz 충돌 체크 (timerCallback에서 호출) =====
bool CollisionAvoidance::checkAndUpdate() {
    if (!running_.load() || !offboard_mgr_) return false;

    // 내 상태 수집
    DroneState me;
    me.drone_id = drone_id_;
    me.latitude = offboard_mgr_->getCurrentLat();
    me.longitude = offboard_mgr_->getCurrentLon();
    me.altitude = offboard_mgr_->getCurrentAltAmsl();
    me.vx = offboard_mgr_->getCurrentVx();
    me.vy = offboard_mgr_->getCurrentVy();
    me.vz = offboard_mgr_->getCurrentVz();

    // GPS 미수신 시 체크 불가
    if (me.latitude == 0.0 && me.longitude == 0.0) return false;

    auto now = std::chrono::steady_clock::now();
    bool any_threat = false;
    uint8_t threat_id = 0;

    std::lock_guard<std::mutex> lock(drones_mutex_);
    for (auto& [id, other] : other_drones_) {
        if (!other.valid) continue;

        // 스테일 데이터 체크
        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - other.last_update).count();

        if (age_ms > STALE_TIMEOUT_MS) {
            // 이미 hold 중인 위협이면 hold 유지
            if (collision_hold_.load() && current_threat_id_.load() == id) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[CollisionAvoid] Stale data from drone %d (%ldms) - maintaining hold",
                    id, age_ms);
                any_threat = true;
                threat_id = id;
            }
            continue;
        }

        // 고도 차이 체크 (사전 필터링)
        if (hasAltitudeClearance(me.altitude, other.altitude)) continue;

        // 수평 거리 계산
        float dist = calculateHorizontalGPSDistance(
            me.latitude, me.longitude, other.latitude, other.longitude);

        // 접근 방향 체크
        bool i_approach = isIApproachingOther(me, other);
        bool other_approach = isIApproachingOther(other, me);

        if (dist < DANGER_DISTANCE) {
            // 위험 거리 → 정지 판단
            if (shouldIStop(drone_id_, id, i_approach, other_approach)) {
                any_threat = true;
                threat_id = id;

                if (!collision_hold_.load()) {
                    const char* reason = "unknown";
                    if (i_approach && other_approach) reason = "Head-on (both approaching)";
                    else if (i_approach) reason = "I am approaching";
                    else if (other_approach) reason = "Other approaching (lower priority)";

                    RCLCPP_WARN(node_->get_logger(),
                        "[CollisionAvoid] DANGER! drone %d at %.1fm - %s - STOPPING",
                        id, dist, reason);
                }
            }
        } else if (dist < WARNING_DISTANCE && (i_approach || other_approach)) {
            // 경고 거리
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[CollisionAvoid] Warning: drone %d at %.1fm (me_approach=%d, other_approach=%d)",
                id, dist, i_approach, other_approach);
        }

        // 안전 거리 확보 확인 (현재 hold 중인 위협 드론과의 거리)
        if (collision_hold_.load() && current_threat_id_.load() == id && dist < SAFE_DISTANCE) {
            any_threat = true;
            threat_id = id;
        }
    }

    collision_hold_.store(any_threat);
    current_threat_id_.store(threat_id);
    return any_threat;
}

// ===== GPS 수평 거리 계산 =====
float CollisionAvoidance::calculateHorizontalGPSDistance(
    double lat1, double lon1, double lat2, double lon2) const
{
    double cos_lat = std::cos((lat1 + lat2) * 0.5 * M_PI / 180.0);
    double dn = (lat2 - lat1) * DEG_TO_M_LAT;
    double de = (lon2 - lon1) * DEG_TO_M_LAT * cos_lat;
    return static_cast<float>(std::sqrt(dn * dn + de * de));
}

// ===== 고도 차이 충분 여부 =====
bool CollisionAvoidance::hasAltitudeClearance(float alt1, float alt2) const {
    return std::abs(alt1 - alt2) >= ALTITUDE_CLEARANCE;
}

// ===== 내가 상대를 향해 접근 중인지 =====
bool CollisionAvoidance::isIApproachingOther(
    const DroneState& me, const DroneState& other) const
{
    // GPS → NED 방향 벡터 (me → other)
    double cos_lat = std::cos(me.latitude * M_PI / 180.0);
    float dn = static_cast<float>((other.latitude - me.latitude) * DEG_TO_M_LAT);
    float de = static_cast<float>((other.longitude - me.longitude) * DEG_TO_M_LAT * cos_lat);
    float dist = std::sqrt(dn * dn + de * de);

    if (dist < 0.5f) return false;  // 너무 가까우면 방향 벡터 불안정

    // 단위 방향 벡터
    float dir_n = dn / dist;
    float dir_e = de / dist;

    // 내 속도를 상대 방향으로 투영 (내적)
    float speed_toward = me.vx * dir_n + me.vy * dir_e;

    return speed_toward > APPROACH_SPEED_THRESHOLD;
}

// ===== 내가 정지해야 하는지 (검증 완료된 4규칙) =====
bool CollisionAvoidance::shouldIStop(
    uint8_t my_id, uint8_t other_id,
    bool i_am_approaching, bool other_is_approaching) const
{
    // 규칙 1: 양쪽 모두 접근 중 (정면 충돌/교차) → 우선순위 낮은 쪽 정지
    if (i_am_approaching && other_is_approaching) {
        return my_id > other_id;
    }

    // 규칙 2: 내가 혼자 접근 중 (추월/정지 드론 접근) → 무조건 정지
    if (i_am_approaching && !other_is_approaching) {
        return true;
    }

    // 규칙 3: 상대방만 접근 중 → 우선순위 낮으면 정지
    if (!i_am_approaching && other_is_approaching) {
        return my_id > other_id;
    }

    // 규칙 4: 둘 다 접근 안 함 → 계속
    return false;
}
