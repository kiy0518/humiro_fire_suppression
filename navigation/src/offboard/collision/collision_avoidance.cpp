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
        "[CollisionAvoid] Initialized for drone %d (DANGER=%.0fm, SAFE=%.0fm, EVADE=%.0fm)",
        drone_id_, DANGER_DISTANCE, SAFE_DISTANCE, EVADE_OFFSET_M);
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
    evade_offset_n_.store(0.0f);
    evade_offset_e_.store(0.0f);
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

    // 홈 좌표 (충돌 방지 우선순위 계산용)
    msg.home_latitude = offboard_mgr_->getHomeLat();
    msg.home_longitude = offboard_mgr_->getHomeLon();

    // 안전 데이터 (편대 공유)
    msg.battery_remaining = offboard_mgr_->getBatteryRemaining();
    msg.gps_fix_type = offboard_mgr_->getGPSFixType();
    msg.gps_satellites = offboard_mgr_->getGPSSatellites();

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
    state.home_latitude = msg->home_latitude;
    state.home_longitude = msg->home_longitude;
    state.mission_state = msg->mission_state;
    state.last_update = std::chrono::steady_clock::now();
    state.valid = true;
}

// ===== 10Hz 충돌 체크 (timerCallback에서 호출) =====
CollisionAction CollisionAvoidance::checkAndUpdate() {
    if (!running_.load() || !offboard_mgr_) return CollisionAction::NONE;

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
    if (me.latitude == 0.0 && me.longitude == 0.0) return CollisionAction::NONE;

    // 내 홈 거리 계산
    double home_lat = offboard_mgr_->getHomeLat();
    double home_lon = offboard_mgr_->getHomeLon();
    float my_home_dist = (home_lat == 0.0 && home_lon == 0.0) ? 9999.0f
        : calculateHorizontalGPSDistance(me.latitude, me.longitude, home_lat, home_lon);

    auto now = std::chrono::steady_clock::now();
    CollisionAction worst_action = CollisionAction::NONE;
    uint8_t threat_id = 0;

    std::lock_guard<std::mutex> lock(drones_mutex_);
    for (auto& [id, other] : other_drones_) {
        if (!other.valid) continue;

        // 스테일 데이터 체크
        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - other.last_update).count();

        if (age_ms > STALE_TIMEOUT_MS) {
            // 이미 hold/evade 중인 위협이면 유지
            if (collision_hold_.load() && current_threat_id_.load() == id) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[CollisionAvoid] Stale data from drone %d (%ldms) - maintaining action",
                    id, age_ms);
                worst_action = CollisionAction::HOLD;
                threat_id = id;
            }
            continue;
        }

        // 상대 드론이 착륙 완료(시동 꺼짐)면 충돌 위험 없음 → 건너뛰기
        if (other.mission_state == "LANDED" || other.mission_state == "IDLE") {
            continue;
        }

        // 고도 차이 체크 (사전 필터링)
        // 단, 상대가 RTL(착륙 진행) 중이면 고도 분리만으로 통과 금지 (하향풍 위험)
        bool other_is_landing = (other.mission_state == "RTL");
        if (!other_is_landing && hasAltitudeClearance(me.altitude, other.altitude)) continue;

        // 수평 거리 계산
        float dist = calculateHorizontalGPSDistance(
            me.latitude, me.longitude, other.latitude, other.longitude);

        // 접근 방향 체크
        bool i_approach = isIApproachingOther(me, other);
        bool other_approach = isIApproachingOther(other, me);

        // 상대 홈 거리 계산 (상대 드론 자신의 홈 기준)
        float other_home_dist = (other.home_latitude == 0.0 && other.home_longitude == 0.0) ? 9999.0f
            : calculateHorizontalGPSDistance(other.latitude, other.longitude,
                                              other.home_latitude, other.home_longitude);

        if (dist < DANGER_DISTANCE) {
            // 위험 거리 → 행동 결정 (홈 거리 기반 우선순위)
            CollisionAction action = determineAction(
                drone_id_, id, i_approach, other_approach, my_home_dist, other_home_dist);

            if (action != CollisionAction::NONE) {
                if (!collision_hold_.load()) {
                    const char* action_str = (action == CollisionAction::HOLD) ? "HOLD" : "EVADE_RIGHT";
                    RCLCPP_WARN(node_->get_logger(),
                        "[CollisionAvoid] DANGER! drone %d at %.1fm → %s (my_home=%.0fm, other_home=%.0fm)",
                        id, dist, action_str, my_home_dist, other_home_dist);
                }

                // 더 심각한 액션 우선 (HOLD > EVADE)
                if (action == CollisionAction::HOLD ||
                    worst_action == CollisionAction::NONE) {
                    worst_action = action;
                    threat_id = id;
                }
            }
        } else if (dist < WARNING_DISTANCE && (i_approach || other_approach)) {
            // 경고 거리
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[CollisionAvoid] Warning: drone %d at %.1fm (me_approach=%d, other_approach=%d)",
                id, dist, i_approach, other_approach);
        }

        // 안전 거리 확보 확인 (현재 hold/evade 중인 위협 드론과의 거리)
        // 히스테리시스: DANGER~SAFE 구간에서 기존 HOLD 유지 (접근 여부 무관)
        // → SAFE_DISTANCE(25m) 초과해야만 해제
        if (collision_hold_.load() && current_threat_id_.load() == id && dist < SAFE_DISTANCE) {
            if (worst_action == CollisionAction::NONE) {
                worst_action = CollisionAction::HOLD;
                threat_id = id;
            }
        }
    }

    bool any_active = (worst_action != CollisionAction::NONE);
    collision_hold_.store(any_active);
    current_threat_id_.store(threat_id);

    // 충돌 해제 시 히스테리시스 잠금도 해제
    if (!any_active) {
        locked_priority_holder_ = 0;
    }

    // EVADE_RIGHT 시 회피 오프셋 계산
    if (worst_action == CollisionAction::EVADE_RIGHT) {
        computeEvadeOffset();
    } else {
        evade_offset_n_.store(0.0f);
        evade_offset_e_.store(0.0f);
    }

    return worst_action;
}

// ===== 홈 거리 기반 행동 결정 =====
// 핵심 원칙:
//   1. 홈에 가까운 기체가 진행권 (먼저 착륙)
//   2. 거리 차이 5m 이내(GPS 노이즈)면 기존 결정 유지, 초회는 ID로 타이브레이크
//   3. 데드락 방지: 높은 우선순위는 항상 NONE, 낮은 우선순위는 항상 EVADE_RIGHT
//      → 접근 여부와 무관하게 우선순위만으로 결정 (HOLD 진동 방지)
CollisionAction CollisionAvoidance::determineAction(
    uint8_t my_id, uint8_t other_id,
    bool /*i_am_approaching*/, bool /*other_is_approaching*/,
    float my_home_dist, float other_home_dist)
{
    // 우선순위 결정: 홈에 가까운 쪽이 진행
    float diff = my_home_dist - other_home_dist;  // 음수 = 내가 더 가까움

    bool i_have_priority;
    if (std::abs(diff) > HOME_DIST_HYSTERESIS) {
        // 거리 차이가 충분히 큼 → 거리 기반 결정
        i_have_priority = (diff < 0.0f);  // 내가 홈에 더 가까우면 진행권
        // 히스테리시스 잠금 갱신
        locked_priority_holder_ = i_have_priority ? my_id : other_id;
    } else {
        // 거리 차이 5m 이내 → GPS 노이즈 구간
        if (locked_priority_holder_ != 0) {
            // 기존 잠금 유지
            i_have_priority = (locked_priority_holder_ == my_id);
        } else {
            // 초회: ID로 타이브레이크 (낮은 ID 우선)
            i_have_priority = (my_id < other_id);
            locked_priority_holder_ = i_have_priority ? my_id : other_id;
        }
    }

    // 규칙 1: 내가 진행권 보유 → 계속 진행
    if (i_have_priority) {
        return CollisionAction::NONE;
    }

    // 규칙 2: 상대가 진행권 보유 → 내가 양보
    // 접근 중이면 EVADE_RIGHT (RTL 필터에서 HOLD로 변환)
    // 정지 중이면(상대만 접근) EVADE_RIGHT (RTL 필터에서 HOLD로 변환)
    return CollisionAction::EVADE_RIGHT;
}

// ===== EVADE 오프셋 계산 (현재 비행 방향 직각 오른쪽) =====
void CollisionAvoidance::computeEvadeOffset() {
    if (!offboard_mgr_) return;

    // 현재 속도 벡터에서 heading 추출
    float vx = offboard_mgr_->getCurrentVx();
    float vy = offboard_mgr_->getCurrentVy();
    float speed = std::sqrt(vx * vx + vy * vy);

    if (speed < 0.5f) {
        // 속도가 너무 작으면 yaw 기준
        float yaw = offboard_mgr_->getCurrentYaw();
        // yaw: NED (North=0, clockwise). 오른쪽 = yaw + 90도
        float right_yaw = yaw + static_cast<float>(M_PI / 2.0);
        evade_offset_n_.store(EVADE_OFFSET_M * std::cos(right_yaw));
        evade_offset_e_.store(EVADE_OFFSET_M * std::sin(right_yaw));
    } else {
        // 속도 방향 기준 오른쪽 직각
        float dir_n = vx / speed;
        float dir_e = vy / speed;
        // 오른쪽 = (dir_e, -dir_n) → NED에서 직각 오른쪽
        evade_offset_n_.store(EVADE_OFFSET_M * dir_e);
        evade_offset_e_.store(EVADE_OFFSET_M * (-dir_n));
    }
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
