# 충돌 방지 시스템 설계 (v0.8.3)

**작성일**: 2026-01-15
**대상 버전**: v0.8.3
**우선순위**: HIGH (안전 기능)

---

## 1. 요구사항

### 1.1 핵심 요구사항
- ✅ 프로그램 시작 시 충돌 방지 자동 활성화
- ✅ 회피 기동 대신 **정지(호버링) 방식**
- ✅ 우선순위: DRONE_ID 기반 (1 > 2 > 3)
- ✅ 양방향 접근 시나리오 처리
- ✅ 백그라운드에서 지속적 모니터링

### 1.2 안전 기준
- **경고 거리**: 8.0m (모니터링 시작)
- **위험 거리**: 5.0m (정지 명령)
- **안전 거리**: 10.0m (재개 가능)
- **고도 차이 임계값**: 5.0m (고도 차이가 이상이면 충돌 위험 없음)
- **체크 주기**: 100ms (10Hz)

**중요**: 고도 차이가 5.0m 이상이면 수평 거리와 관계없이 충돌 위험이 없다고 판단

---

## 2. 시스템 아키텍처

```
[Application Manager] (시작 시)
         ↓
[Collision Avoidance Manager] ← 항상 실행
         ↓
   ┌─────┴─────┐
   ↓           ↓
[Distance    [Priority
 Monitor]     Handler]
   ↓           ↓
[ROS2 Topics: /px4_1/vehicle_local_position]
              /px4_2/vehicle_local_position
              /px4_3/vehicle_local_position
```

### 2.1 주요 컴포넌트

#### CollisionAvoidanceManager
```cpp
class CollisionAvoidanceManager {
public:
    CollisionAvoidanceManager(uint8_t my_vehicle_id);

    // 충돌 방지 시스템 시작/종료
    void start();
    void stop();

    // 충돌 위험 체크
    CollisionStatus checkCollision();

    // 긴급 정지 명령
    void emergencyStop();

    // 정지 해제 (안전 시)
    void resume();

private:
    uint8_t my_vehicle_id_;
    std::map<uint8_t, VehicleState> other_vehicles_;

    // ROS2 subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        vehicle1_sub_, vehicle2_sub_, vehicle3_sub_;

    // 백그라운드 스레드
    std::thread monitor_thread_;
    std::atomic<bool> is_running_;

    // 충돌 체크 로직
    bool isCollisionRisk(const VehicleState& other);
    bool shouldIStop(uint8_t other_vehicle_id);
};
```

#### VehicleState
```cpp
struct VehicleState {
    uint8_t vehicle_id;
    double x, y, z;           // NED 좌표
    double vx, vy, vz;        // 속도
    uint64_t timestamp;
    bool is_active;
};
```

#### CollisionStatus
```cpp
struct CollisionStatus {
    bool has_risk;                    // 충돌 위험 있음
    uint8_t threat_vehicle_id;        // 위협 드론 ID
    float distance;                   // 현재 거리
    bool should_stop;                 // 내가 정지해야 함
    std::string reason;               // 이유
};
```

---

## 3. 충돌 감지 알고리즘

### 3.1 거리 계산 및 고도 차이 체크

**3D 거리 계산**:
```cpp
float calculateDistance(const VehicleState& v1, const VehicleState& v2) {
    float dx = v1.x - v2.x;
    float dy = v1.y - v2.y;
    float dz = v1.z - v2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
```

**고도 차이 체크** (충돌 위험 사전 필터링):
```cpp
bool hasAltitudeClearance(const VehicleState& v1, const VehicleState& v2) {
    // NED 좌표계: z는 아래 방향이 양수
    // 고도 차이 = |z1 - z2|
    float altitude_diff = std::abs(v1.z - v2.z);

    // 고도 차이가 5.0m 이상이면 충돌 위험 없음
    return altitude_diff >= 5.0f;
}
```

**사용 예시**:
```cpp
if (hasAltitudeClearance(my_state, other_state)) {
    // 고도 차이가 충분함 → 충돌 방지 시스템 작동 안 함
    return CollisionStatus{false, 0, distance, false, "Altitude clearance OK"};
}
```

**시나리오**:
| 드론1 고도 | 드론2 고도 | 고도 차이 | 충돌 체크 |
|-----------|-----------|----------|----------|
| z=-1.0m (1m) | z=-1.0m (1m) | 0.0m | ✅ 필요 |
| z=-1.0m (1m) | z=-3.0m (3m) | 2.0m | ✅ 필요 |
| z=-1.0m (1m) | z=-4.0m (4m) | 3.0m | ✅ 필요 |
| z=-1.0m (1m) | z=-6.0m (6m) | 5.0m | ❌ 불필요 (안전) |
| z=-2.0m (2m) | z=-8.0m (8m) | 6.0m | ❌ 불필요 (안전) |

### 3.2 접근 방향 감지 (절대 방향 기반)

**핵심**: 내 속도 벡터를 상대 방향으로 투영하여 판단

```cpp
bool isIApproachingOther(const VehicleState& me, const VehicleState& other) {
    // 1. 방향 벡터 계산 (나 → 상대)
    float dx = other.x - me.x;
    float dy = other.y - me.y;
    float dz = other.z - me.z;
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 너무 가까우면 판단 불가
    if (distance < 0.01f) return false;

    // 2. 단위 방향 벡터
    float dir_x = dx / distance;
    float dir_y = dy / distance;
    float dir_z = dz / distance;

    // 3. 내 속도를 상대 방향으로 투영 (내적)
    float my_speed_toward_other = me.vx * dir_x +
                                   me.vy * dir_y +
                                   me.vz * dir_z;

    // 4. 양수 = 상대를 향해 이동 중
    // 임계값 0.1m/s (노이즈 제거)
    return my_speed_toward_other > 0.1f;
}
```

**장점**:
- ✅ 추월 시나리오 정확히 감지 (빠른 드론만 "접근 중"으로 판정)
- ✅ 정지 드론은 절대 "접근 중"이 아님
- ✅ 교차 시나리오 양방향 감지
- ✅ 상대 속도가 아닌 절대 방향 기반

### 3.3 우선순위 및 접근 방향 로직

**핵심 원칙**:
1. 양쪽 모두 접근 중 → 우선순위 낮은 쪽만 정지
2. 내가 혼자 접근 중 → 무조건 정지
3. 상대방만 접근 중 → 우선순위 낮으면 정지
4. 둘 다 접근 안 함 → 계속 진행

```cpp
bool shouldIStop(uint8_t my_id, uint8_t other_id,
                 bool i_am_approaching, bool other_is_approaching) {

    // 규칙 1: 양쪽 모두 접근 중 (정면 충돌 또는 교차)
    if (i_am_approaching && other_is_approaching) {
        // 우선순위 낮은 쪽만 정지
        return my_id > other_id;
    }

    // 규칙 2: 내가 혼자 접근 중 (추월 또는 정지 드론 접근)
    if (i_am_approaching && !other_is_approaching) {
        // 무조건 정지
        return true;
    }

    // 규칙 3: 상대방만 접근 중 (나는 정지 또는 멀어지는 중)
    if (!i_am_approaching && other_is_approaching) {
        // 우선순위 낮으면 정지
        return my_id > other_id;
    }

    // 규칙 4: 둘 다 접근 안 함 (멀어지는 중 또는 평행 이동)
    return false;
}
```

**시나리오별 동작**:

| 상황 | 드론1 접근? | 드론2 접근? | 결과 | 규칙 |
|------|------------|------------|------|-----|
| 드론1 → 드론2(정지) | ✅ | ❌ | **드론1 정지** | 규칙2 |
| 드론1(정지) ← 드론2 | ❌ | ✅ | **드론2 정지** | 규칙3 |
| 드론1 → ← 드론2 | ✅ | ✅ | **드론2 정지** | 규칙1 |
| 드론1 ← → 드론2 | ❌ | ❌ | 둘 다 계속 | 규칙4 |
| 드론1(빠름) → 드론2(느림) → | ✅ | ❌ | **드론1 정지** | 규칙2 |

---

## 4. 동작 시나리오

### 4.1 시나리오 A: 드론1과 드론2 충돌 위험

```
초기 상태:
  드론1 (x=0, y=0, vx=1.0)  ──→
  드론2 (x=10, y=0, vx=-1.0) ←──
  거리: 10.0m

2초 후:
  드론1 (x=2, y=0)
  드론2 (x=8, y=0)
  거리: 6.0m ⚠️ 경고!

2.5초 후:
  드론1 (x=2.5, y=0)
  드론2 (x=7.5, y=0)
  거리: 5.0m 🚨 위험!

CollisionAvoidanceManager 동작:
  1. 드론1: my_id(1) < other_id(2) → 계속 진행 ✅
  2. 드론2: my_id(2) > other_id(1) → 즉시 정지! 🛑
     - OffboardManager::emergencyStop() 호출
     - 현재 위치 (x=7.5, y=0)에서 호버링
     - 상태: COLLISION_AVOIDANCE

5초 후:
  드론1 (x=5, y=0, 계속 진행)
  드론2 (x=7.5, y=0, 정지 중)
  거리: 2.5m (드론2가 정지해서 안전!)

8초 후:
  드론1 (x=8, y=0, 통과 중)
  드론2 (x=7.5, y=0, 정지 유지)
  거리: 0.5m (드론1 통과)

12초 후:
  드론1 (x=12, y=0, 통과 완료)
  드론2 (x=7.5, y=0)
  거리: 4.5m

15초 후:
  거리: 7.5m → 아직 대기

18초 후:
  거리: 10.5m ✅ 안전거리 확보
  드론2: 미션 재개 (resume())
```

### 4.2 시나리오 B: 드론2가 드론1로 접근 (드론1 정지 중)

```
초기 상태:
  드론1 (x=10, y=0, vx=0, vy=0, vz=0)  ⏸️ 완전 정지
  드론2 (x=0, y=0, vx=1.0)  ──→
  거리: 10.0m

2초 후:
  드론1 (x=10, y=0, 정지 유지)
  드론2 (x=2, y=0)
  거리: 8.0m ⚠️ 경고!

5초 후:
  드론1 (x=10, y=0, 여전히 정지) ⏸️
  드론2 (x=5, y=0, 접근 중)
  거리: 5.0m 🚨 위험!

CollisionAvoidanceManager 동작:
  1. 드론2의 CollisionManager가 감지:
     - 드론1과의 거리: 5.0m
     - 드론1 속도: vx=0 (정지 중)
     - 내 속도: vx=1.0 (접근 중)
     - my_id(2) > other_id(1) → 내가 정지해야 함!

  2. 드론2: 즉시 정지! 🛑
     - emergencyStop() 호출
     - 현재 위치 (x=5, y=0)에서 호버링

  3. 드론1: 아무 동작 안 함 ⏸️
     - 충돌 방지 시스템이 드론1에게는 명령하지 않음
     - 우선순위 높음 → 방해받지 않음

10초 후:
  드론1 (x=10, y=0, 계속 정지)
  드론2 (x=5, y=0, 정지 대기 중)
  거리: 5.0m 유지

결과:
  - 드론1: ✅ 영향 없음 (절대 이동하지 않음!)
  - 드론2: 🛑 5m 거리에서 정지 대기
  - 드론2는 드론1이 이동하거나 안전거리(10m) 확보될 때까지 대기
```

**중요**: 접근 방향이 중요합니다! 상대가 정지해 있어도 내가 접근 중이면 **내가 정지**합니다.

### 4.3 시나리오 C: 드론1이 정지한 드론2로 접근 (핵심!)

```
초기 상태:
  드론1 (x=0, y=0, vx=1.0)  ──→ 이동 중
  드론2 (x=8, y=0, vx=0)    ⏸️ 완전 정지
  거리: 8.0m

3초 후:
  드론1 (x=3, y=0, vx=1.0, 계속 이동)
  드론2 (x=8, y=0, vx=0, 정지 유지)
  거리: 5.0m 🚨 위험!

CollisionAvoidanceManager 동작:

  드론1의 판단:
    1. 다른 드론 감지: 드론2
    2. 거리: 5.0m (위험 거리)
    3. 접근 확인:
       - 내 속도: vx=1.0 (이동 중)
       - 상대 속도: vx=0 (정지)
       - isApproaching(me, other) = true (내가 접근 중!)
       - isApproaching(other, me) = false (상대는 정지)
    4. shouldIStop(1, 2, true, false):
       - i_am_approaching = true → 무조건 정지!
       - 우선순위 무관!
    5. 결론: 드론1 정지! 🛑

  드론1 동작:
    - emergencyStop() 호출
    - 현재 위치 (x=3, y=0)에서 호버링
    - 상태: COLLISION_AVOIDANCE
    - RCLCPP_WARN: "Approaching stopped vehicle - STOPPING"

  드론2 동작:
    - 아무 동작 없음 (계속 정지)
    - 충돌 방지 시스템이 드론2에게는 명령하지 않음

결과:
  - 드론1 (x=3, y=0, 정지 중)
  - 드론2 (x=8, y=0, 정지 유지)
  - 거리: 5.0m 안전하게 유지
  - 양쪽 모두 정지 상태로 대기

재개 조건:
  - 수동으로 드론1 또는 드론2를 이동시켜 10m 이상 확보
  - 또는 한쪽이 RTL/Mission 재개로 멀어짐
```

**핵심**: 우선순위가 높은 드론1이라도, **자신이 접근하는 상황**이면 정지합니다!

### 4.4 시나리오 D: 3대 동시 미션

```
초기 포메이션:
  드론1 (중앙)     x=0,  y=0
  드론2 (좌측 60°) x=-5, y=-8.66
  드론3 (우측 60°) x=-5, y=+8.66

전진 중:
  드론1: vx=0.5 ──→
  드론2: vx=0.5 ──→
  드론3: vx=0.5 ──→

만약 드론3이 드론1 쪽으로 이탈:
  드론3 (x=5, y=5, 드론1 방향으로 이동)
  드론1과의 거리: 7.1m → 경고!

  계속 접근:
  드론3 (x=3, y=3)
  드론1과의 거리: 4.2m 🚨 위험!

CollisionAvoidanceManager 동작:
  1. 드론3: my_id(3) > other_id(1) → 정지! 🛑
  2. 드론1: 계속 진행 ✅
  3. 드론2: 영향 없음 ✅

드론3 재개:
  - 드론1과의 거리 10m 이상 확보 후
  - 원래 포메이션 위치로 복귀
  - 미션 재개
```

---

## 5. 구현 세부사항

### 5.1 파일 구조

```
navigation/src/offboard/autonomous/
├── collision_avoidance_manager.h    (새 파일)
├── collision_avoidance_manager.cpp  (새 파일)
├── offboard_manager.h               (수정)
└── offboard_manager.cpp             (수정)

application/src/
├── application_manager.cpp          (수정: 시작 시 충돌방지 활성화)
```

### 5.2 ROS2 Topic 구독

```cpp
void CollisionAvoidanceManager::initSubscribers() {
    // 내 ID가 1이면 2, 3 구독
    // 내 ID가 2이면 1, 3 구독
    // 내 ID가 3이면 1, 2 구독

    if (my_vehicle_id_ != 1) {
        vehicle1_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/px4_1/fmu/out/vehicle_local_position", 10,
            std::bind(&CollisionAvoidanceManager::vehicle1Callback, this, std::placeholders::_1)
        );
    }

    if (my_vehicle_id_ != 2) {
        vehicle2_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/px4_2/fmu/out/vehicle_local_position", 10,
            std::bind(&CollisionAvoidanceManager::vehicle2Callback, this, std::placeholders::_1)
        );
    }

    if (my_vehicle_id_ != 3) {
        vehicle3_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/px4_3/fmu/out/vehicle_local_position", 10,
            std::bind(&CollisionAvoidanceManager::vehicle3Callback, this, std::placeholders::_1)
        );
    }
}
```

### 5.3 모니터링 루프

```cpp
void CollisionAvoidanceManager::monitorLoop() {
    while (is_running_) {
        // 1. 모든 다른 드론과의 거리 체크
        for (auto& [other_id, other_state] : other_vehicles_) {
            if (!other_state.is_active) continue;

            // 1-1. 고도 차이 체크 (사전 필터링)
            if (hasAltitudeClearance(my_state_, other_state)) {
                // 고도 차이 5m 이상 → 충돌 위험 없음, 다음 드론 체크
                continue;
            }

            // 1-2. 3D 거리 계산
            float distance = calculateDistance(my_state_, other_state);

            // 2. 접근 방향 체크 (양방향)
            bool i_am_approaching = isIApproachingOther(my_state_, other_state);
            bool other_is_approaching = isIApproachingOther(other_state, my_state_);

            // 3. 충돌 위험 판단
            if (distance < DANGER_DISTANCE) {
                // 4. 누가 정지해야 하는지 판단
                if (shouldIStop(my_vehicle_id_, other_id,
                               i_am_approaching, other_is_approaching)) {

                    // 접근 이유 로깅
                    std::string reason;
                    if (i_am_approaching && other_is_approaching) {
                        reason = "Head-on collision (both approaching)";
                    } else if (i_am_approaching) {
                        reason = "I am approaching stopped/moving vehicle";
                    } else if (other_is_approaching) {
                        reason = "Other vehicle approaching (lower priority)";
                    }

                    RCLCPP_WARN(node_->get_logger(),
                        "[CollisionAvoid] DANGER! Vehicle %d at %.2fm - %s - STOPPING",
                        other_id, distance, reason.c_str());

                    // 5. 긴급 정지
                    emergencyStop();

                    // 6. 안전거리까지 대기
                    waitForSafeDistance(other_id);

                    // 7. 재개
                    resume();
                }
            }
            else if (distance < WARNING_DISTANCE) {
                if (i_am_approaching || other_is_approaching) {
                    RCLCPP_INFO(node_->get_logger(),
                        "[CollisionAvoid] Warning: Vehicle %d at %.2fm (approaching: me=%d, other=%d)",
                        other_id, distance, i_am_approaching, other_is_approaching);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }
}
```

### 5.4 긴급 정지 구현

```cpp
void CollisionAvoidanceManager::emergencyStop() {
    if (is_stopped_) return;  // 이미 정지 중

    // OffboardManager에 정지 명령
    offboard_manager_->holdPosition();

    is_stopped_ = true;
    stop_timestamp_ = std::chrono::steady_clock::now();

    RCLCPP_ERROR(node_->get_logger(),
        "[CollisionAvoid] ⛔ EMERGENCY STOP - Holding position");
}
```

### 5.5 안전거리 대기

```cpp
void CollisionAvoidanceManager::waitForSafeDistance(uint8_t threat_id) {
    RCLCPP_INFO(node_->get_logger(),
        "[CollisionAvoid] Waiting for safe distance from Vehicle %d...", threat_id);

    while (is_running_ && is_stopped_) {
        auto& threat_state = other_vehicles_[threat_id];
        float distance = calculateDistance(my_state_, threat_state);

        if (distance >= SAFE_DISTANCE) {
            RCLCPP_INFO(node_->get_logger(),
                "[CollisionAvoid] ✅ Safe distance reached: %.2fm", distance);
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
```

### 5.6 미션 재개

```cpp
void CollisionAvoidanceManager::resume() {
    if (!is_stopped_) return;

    RCLCPP_INFO(node_->get_logger(),
        "[CollisionAvoid] ▶️ Resuming mission");

    // OffboardManager에 재개 명령
    offboard_manager_->resumeMission();

    is_stopped_ = false;
}
```

---

## 6. OffboardManager 통합

### 6.1 새로운 메서드 추가

```cpp
class OffboardManager {
public:
    // 충돌 방지 관련
    void holdPosition();           // 현재 위치 고정
    void resumeMission();          // 미션 재개

private:
    std::shared_ptr<CollisionAvoidanceManager> collision_manager_;
};
```

### 6.2 holdPosition() 구현

```cpp
void OffboardManager::holdPosition() {
    // 현재 위치 저장
    saved_position_ = current_position_;
    saved_state_ = current_state_;

    // 위치 고정 모드로 전환
    current_state_ = MissionState::COLLISION_HOLD;

    // Setpoint을 현재 위치로 고정
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    setpoint.position[0] = current_position_.x;
    setpoint.position[1] = current_position_.y;
    setpoint.position[2] = current_position_.z;
    setpoint.velocity[0] = 0.0f;
    setpoint.velocity[1] = 0.0f;
    setpoint.velocity[2] = 0.0f;
    setpoint.yaw = current_yaw_;

    // 지속적으로 전송 (호버링 유지)
    while (current_state_ == MissionState::COLLISION_HOLD) {
        setpoint.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(setpoint);
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
    }
}
```

### 6.3 resumeMission() 구현

```cpp
void OffboardManager::resumeMission() {
    RCLCPP_INFO(node_->get_logger(),
        "[OffboardManager] Resuming from collision hold");

    // 이전 상태로 복귀
    current_state_ = saved_state_;

    // 미션 계속 진행
    // (현재 단계부터 재개)
}
```

---

## 7. ApplicationManager 통합

### 7.1 시작 시 충돌 방지 활성화

```cpp
void ApplicationManager::initializeOffboardManager() {
#ifdef ENABLE_ROS2
    // OffboardManager 초기화
    offboard_manager_ = std::make_unique<OffboardManager>(node_);

    // 충돌 방지 매니저 생성 및 시작
    uint8_t vehicle_id = readDroneId();
    collision_manager_ = std::make_shared<CollisionAvoidanceManager>(
        node_, vehicle_id, offboard_manager_.get()
    );

    // 충돌 방지 시스템 시작
    collision_manager_->start();

    RCLCPP_INFO(node_->get_logger(),
        "[ApplicationManager] Collision avoidance system started for Vehicle %d",
        vehicle_id);
#endif
}
```

---

## 8. 테스트 시나리오

### 8.1 단위 테스트

```cpp
TEST(CollisionAvoidanceTest, ApproachingLogic) {
    // 내가 접근 중 → 무조건 정지
    EXPECT_TRUE(shouldIStop(1, 2, true, false));   // 드론1이 접근 중
    EXPECT_TRUE(shouldIStop(1, 2, true, true));    // 양쪽 접근 중 (우선순위)

    // 상대만 접근 중 → 우선순위에 따라
    EXPECT_TRUE(shouldIStop(2, 1, false, true));   // 드론2, 상대 접근, 우선순위 낮음
    EXPECT_FALSE(shouldIStop(1, 2, false, true));  // 드론1, 상대 접근, 우선순위 높음

    // 둘 다 접근 안 함 → 정지 안 함
    EXPECT_FALSE(shouldIStop(1, 2, false, false));
    EXPECT_FALSE(shouldIStop(2, 1, false, false));
}

TEST(CollisionAvoidanceTest, PriorityWithApproaching) {
    // 시나리오: 드론1이 정지한 드론2로 접근
    bool i_am_approaching = true;
    bool other_is_approaching = false;
    EXPECT_TRUE(shouldIStop(1, 2, i_am_approaching, other_is_approaching));

    // 시나리오: 드론2가 정지한 드론1로 접근
    EXPECT_TRUE(shouldIStop(2, 1, i_am_approaching, other_is_approaching));

    // 핵심: 접근하는 쪽이 우선순위 관계없이 정지!
}

TEST(CollisionAvoidanceTest, DistanceCalculation) {
    VehicleState v1{1, 0, 0, 0};
    VehicleState v2{2, 3, 4, 0};
    EXPECT_FLOAT_EQ(calculateDistance(v1, v2), 5.0f);
}

TEST(CollisionAvoidanceTest, ApproachingDetection) {
    VehicleState me{1, 0, 0, 0, 1.0, 0, 0};     // 우측으로 이동
    VehicleState other{2, 5, 0, 0, -1.0, 0, 0}; // 좌측으로 이동
    EXPECT_TRUE(isApproaching(me, other));

    // 정지한 드론에게 접근
    VehicleState me_moving{1, 0, 0, 0, 1.0, 0, 0};
    VehicleState other_stopped{2, 5, 0, 0, 0, 0, 0};
    EXPECT_TRUE(isApproaching(me_moving, other_stopped));
    EXPECT_FALSE(isApproaching(other_stopped, me_moving));
}
```

### 8.2 통합 테스트

#### Test 1: 정면 충돌
```
1. 드론1, 드론2 5m 거리에서 서로를 향해 이동
2. 3m 거리에서 드론2 자동 정지 확인
3. 드론1 통과 확인
4. 안전거리 확보 후 드론2 재개 확인
```

#### Test 2: 우선순위 높은 드론이 정지 드론으로 접근
```
1. 드론2 정지 상태, 드론1이 드론2 방향으로 이동
2. 5m 거리에서 드론1 자동 정지 확인 (접근하는 쪽이 정지!)
3. 양쪽 모두 정지 상태 확인
4. 수동으로 거리 확보 필요
```

#### Test 3: 후방 접근
```
1. 드론1 정지, 드론2가 뒤에서 접근
2. 5m 거리에서 드론2 자동 정지 확인
3. 드론1 이동 시작
4. 안전거리 확보 후 드론2 재개 확인
```

#### Test 4: 3대 동시 미션
```
1. 정상 포메이션 비행 중
2. 드론3 의도적으로 드론1 방향 이동
3. 드론3 자동 정지, 드론1/2 계속 진행
4. 드론3 포메이션 복귀 후 재개
```

---

## 9. 설정 파라미터

```cpp
// collision_avoidance_config.h
namespace CollisionAvoidance {
    constexpr float WARNING_DISTANCE = 8.0f;   // 경고 거리 (m)
    constexpr float DANGER_DISTANCE = 5.0f;    // 위험 거리 (m)
    constexpr float SAFE_DISTANCE = 10.0f;     // 안전 거리 (m)
    constexpr int CHECK_RATE_HZ = 10;          // 체크 주기 (Hz)
    constexpr float PREDICTION_TIME = 0.5f;    // 예측 시간 (초)
    constexpr float EMERGENCY_DISTANCE = 2.0f; // 긴급 정지 거리 (m)
}
```

---

## 10. 구현 순서

### Phase 1: 기본 프레임워크 (v0.8.3-alpha)
- [ ] CollisionAvoidanceManager 클래스 생성
- [ ] 거리 계산 및 접근 감지 로직
- [ ] 우선순위 로직 구현
- [ ] 단위 테스트

### Phase 2: OffboardManager 통합 (v0.8.3-beta)
- [ ] holdPosition() 구현
- [ ] resumeMission() 구현
- [ ] 상태 관리 (COLLISION_HOLD)
- [ ] 통합 테스트

### Phase 3: ROS2 통합 (v0.8.3-rc)
- [ ] ROS2 topic 구독
- [ ] 실시간 위치 업데이트
- [ ] 백그라운드 모니터링 스레드
- [ ] 실기체 테스트

### Phase 4: 최적화 (v0.8.3)
- [ ] 성능 최적화
- [ ] 로깅 개선
- [ ] 비정상 상황 처리
- [ ] 최종 검증

---

## 11. 수동 조종(Manual Mode) 시 충돌 방지

### 11.1 개요

조종기로 수동 조종 중에도 충돌 방지 시스템은 **항상 작동**합니다.

```
조종기 입력 (Manual 모드)
        ↓
PX4 Flight Controller
        ↓
Vehicle Local Position (ROS2)
        ↓
CollisionAvoidanceManager (모니터링)
        ↓
충돌 위험 감지 → 긴급 정지!
```

### 11.2 수동 조종 시나리오

#### 시나리오: 조종기로 드론2를 드론1 쪽으로 조종

```
초기 상태:
  드론1 (x=5, y=0, Manual 모드, 조종자 A가 호버링 중)
  드론2 (x=0, y=0, Manual 모드, 조종자 B가 조종 중)

조종자 B가 드론2를 우측으로 조종:
  2초: 드론2 (x=2, y=0)  거리: 3.0m
  3초: 드론2 (x=3, y=0)  거리: 2.0m ⚠️ 경고!
  5초: 드론2 (x=5, y=0)  거리: 0m → 접근 중

드론2가 5m 거리 근접:
  거리: 5.0m 🚨 위험!

CollisionAvoidanceManager 동작 (드론2):
  1. 드론1과의 거리 5.0m 감지
  2. my_id(2) > other_id(1) → 정지 필요!
  3. 긴급 정지 명령 전송

드론2 동작:
  - PX4에 HOLD 모드 전환 요청
  - 또는 Position 모드로 전환 + 현재 위치 고정
  - 조종기 입력 무시 (일시적)
  - 경고음/진동 (가능하면)

조종자 B의 경험:
  - 조종 스틱을 밀어도 드론이 더 이상 전진하지 않음
  - "충돌 방지 활성화" 메시지 (QGC)
  - 드론1이 멀어지면 자동으로 조종 가능 상태 복귀
```

### 11.3 구현 방법

#### 옵션 1: PX4 모드 전환 (권장)

```cpp
void CollisionAvoidanceManager::emergencyStopManualMode() {
    // Manual 모드 → Position 모드로 전환
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    cmd.param2 = 3;  // PX4_CUSTOM_MAIN_MODE_POSCTL (Position Control)
    cmd.target_system = vehicle_id_;

    vehicle_command_publisher_->publish(cmd);

    // Position hold setpoint 전송
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position[0] = current_position_.x;
    setpoint.position[1] = current_position_.y;
    setpoint.position[2] = current_position_.z;
    setpoint.velocity[0] = 0.0f;
    setpoint.velocity[1] = 0.0f;
    setpoint.velocity[2] = 0.0f;

    trajectory_setpoint_publisher_->publish(setpoint);

    RCLCPP_WARN(node_->get_logger(),
        "[CollisionAvoid] Manual mode interrupted - HOLDING POSITION");
}
```

#### 옵션 2: 조종자 경고만 (최소 개입)

```cpp
void CollisionAvoidanceManager::warnManualOperator() {
    // QGC에 경고 메시지 전송
    sendStatusText("COLLISION RISK! Keep distance from Vehicle "
                   + std::to_string(threat_id_));

    // 조종자가 직접 조치 (권장하지 않음)
}
```

### 11.4 모드별 동작

| 비행 모드 | 충돌 방지 동작 | 비고 |
|-----------|---------------|------|
| **Manual** | Position 모드 전환 + 위치 고정 | 조종 일시 중단 |
| **Altitude** | Position 모드 전환 + 위치 고정 | 조종 일시 중단 |
| **Position** | 현재 위치 고정 | 자연스러운 전환 |
| **OFFBOARD** | holdPosition() 호출 | 기존 방식 |
| **Mission** | HOLD 명령 전송 | 미션 일시 중단 |

### 11.5 안전 복귀 조건

```cpp
void CollisionAvoidanceManager::checkManualModeResume() {
    if (!is_manual_mode_interrupted_) return;

    // 안전 거리 확보 확인
    float distance = calculateDistance(my_state_, threat_state_);

    if (distance >= SAFE_DISTANCE) {
        // 원래 모드로 복귀
        restoreOriginalMode();

        RCLCPP_INFO(node_->get_logger(),
            "[CollisionAvoid] Safe distance - Manual control restored");

        is_manual_mode_interrupted_ = false;
    }
}
```

### 11.6 조종자 안내

#### QGC 메시지
```
[경고] 드론1과 충돌 위험 - 조종 일시 중단
[정보] 안전 거리 확보 중... (현재: 6.5m / 필요: 10.0m)
[정보] 조종 가능 - 안전 거리 확보됨
```

#### 조종 스틱 동작
```
조종자가 전진 스틱 입력 → 드론 반응 없음 (고정)
               ↓
        안전 거리 확보
               ↓
조종자가 전진 스틱 입력 → 드론 정상 이동 ✅
```

### 11.7 우선순위 적용

**중요**: 수동 조종 시에도 **우선순위 규칙 동일**

```
드론2 조종자가 드론1 방향으로 조종
  → 드론2만 정지 (조종 중단)
  → 드론1은 영향 없음 (조종자 A 계속 조종 가능)

드론1 조종자가 드론2 방향으로 조종
  → 드론1 계속 진행 (조종 가능)
  → 드론2만 정지 (자동 또는 수동 모두)
```

### 11.8 긴급 상황 대비

```cpp
// 긴급 상황: 두 조종자가 서로를 향해 조종
if (distance < EMERGENCY_DISTANCE) {  // 2.0m
    // 양쪽 모두 강제 정지!
    emergencyStopBoth();

    RCLCPP_ERROR(node_->get_logger(),
        "[CollisionAvoid] ⛔ EMERGENCY STOP - Both vehicles halted");
}
```

---

## 12. 알려진 제약사항

### 12.1 통신 지연
- ROS2 topic 전송 지연 고려 필요
- 100ms 체크 주기로 최대 10cm 오차 가능 (1m/s 속도 기준)

### 12.2 GPS/Optical Flow 정확도
- 실내: Optical Flow ±0.3m 오차
- 실외: GPS ±1.0m 오차
- 안전 거리를 충분히 크게 설정

### 12.3 3대 이상 동시 충돌
- 현재 설계는 2대 간 충돌만 고려
- 3대 이상 동시 충돌 시 순차 처리

### 12.4 수동 조종 개입
- 조종자가 강제로 조종할 경우 시스템 무력화 가능
- 교육 및 경고 메시지 필수
- 최종 안전은 조종자 책임

---

## 12. 다음 단계

- v0.8.3: 충돌 방지 시스템 구현
- v0.8.4: 포메이션 복귀 로직 추가
- v0.9.0: 야외 GPS 기반 테스트

---

**작성자**: Claude Sonnet 4.5
**검토 필요**: 우선순위 낮은 기체가 높은 기체로 접근 시나리오
