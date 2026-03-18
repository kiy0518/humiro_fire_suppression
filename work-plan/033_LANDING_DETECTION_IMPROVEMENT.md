# 033 OFFBOARD 착지 감지 및 DISARM 시퀀스 개선

**작성일**: 2026-03-18
**상태**: 구현 완료 + 15에이전트 검증 통과 (실기체 테스트 대기)

---

## 문제 배경

반복 비행 테스트 시 세 가지 착지 관련 문제가 발생함:

1. **착지 미감지 / 시동 미꺼짐**: 기압계 AGL 기준점(home_z_)이 매 ARM마다 달라져 지면 착지 후에도 `on_ground` 조건 미충족
2. **공중 DISARM**: 거리계 노이즈 또는 AGL 오판으로 살짝 떠있을 때 FORCE DISARM 실행 → 추락
3. **한 스키드 들림**: FORCE DISARM(param2=21196) = 즉시 모터 컷 → PX4 자세 제어 중단 → 한쪽 스키드 들림

---

## 근본 원인 분석

### 기존 착지 판단 (rtl_handler.h)

```
AGL = dist_bottom (거리계) OR -(local_z - home_z_)
on_ground = (AGL < 0.15m) AND (|vz| < 0.1m/s)
→ 1초(거리계) 또는 3초(기압계) 유지 후 FORCE DISARM
```

**문제점:**
- `home_z_` 기반 기압계 AGL: 매 ARM 시마다 기준점이 달라짐 (기압계 드리프트)
- **거리계 장착 높이(0.17~0.20m) > GROUND_THRESHOLD(0.15m)** → 착지 시 거리계 기반 감지가 절대 불가
- **vz 절대 임계값(0.1) ≈ landing_speed_min(0.05~0.1)** → 하강 중 오감지 가능
- FORCE DISARM(21196) = 즉시 모터 컷 → 한쪽 스키드 들림
- PX4 `vehicle_land_detected` 미활용

---

## 해결 방안: 삼중화 착지 감지 + DISARM 에스컬레이션

### 핵심 설계 원칙

1. **정확한 센서 우선**: PX4 융합 → 거리계 직접측정 → 기압계 추정 순서
2. **절대 임계값 대신 변화율 감지**: "거리가 더 이상 안 줄어든다" + "수직속도가 0이다" = 착지
3. **NORMAL DISARM 우선**: PX4 확인 시 부드러운 모터 램프다운 → 스키드 들림 방지

### 착지 감지 우선순위 (3-tier)

| 우선순위 | 조건 | 원리 | 추가 안정화 | DISARM |
|----------|------|------|-------------|--------|
| 1순위 PX4 | `land_detected == true` | 가속도계 충격 + 기압+자이로 융합 | 0.5초 | NORMAL |
| 2순위 거리계+vz | `dist 변화 < 3cm` 3초 + `vz < 0.05` 3초 + `dist < 0.35m` | 하강 명령인데 안 내려감 = 착지 | 0.5초 | FORCE |
| 3순위 기압계+vz | `AGL < 0.3m` + `vz < 0.05` 3초 | 거리계 없을 때 폴백 | 1.0초 | FORCE |
| 비상 | 180초 타임아웃 | 모든 센서 실패 시 | 즉시 | FORCE |

**2순위 상세 설명:**
- `dist_stable`: 매 tick(10Hz)마다 이전 거리계 값과 비교, 변화량 < 3cm이면 카운트 증가. 30틱(3초) 연속이면 안정
- `vz_stable`: 매 tick마다 |vz| < 0.05m/s이면 카운트 증가. 30틱(3초) 연속이면 안정
- `dist < 0.35m`: 거리계 장착 높이(0.17~0.20m) + 마진 = 지면 근처 확인
- 세 조건 **동시 충족** 시 착지 판정 → 단일 조건 오판 방지

**거리계 임계값 근거**: 장착 높이 0.17~0.20m + 안전 마진. 착지 시 실제 dist_bottom ≈ 0.17~0.20m.
지면 효과(ground effect)로 ±3~5cm 진동 가능 → `DIST_STABLE_DELTA = 0.03m`으로 흡수.

**vz 임계값 vs 하강 속도:**
- `VZ_STABLE_THRESHOLD = 0.05 m/s` — 센서 노이즈 수준
- `landing_speed_min` 하한 = `VZ_STABLE_THRESHOLD * 2 = 0.1 m/s` — 하강 중 vz_stable 오발 방지
- 거리계 있을 때: dist_stable이 주 판정이므로 landing_speed_min 제약 없음
- 거리계 없을 때: 0.1 m/s 하한 강제 (45kg 기체에 충격 최소화)

### DISARM 에스컬레이션

| 감지 방법 | 1-2회차 | 3회차 이후 |
|-----------|---------|-----------|
| PX4 (1순위) | NORMAL (param2=0) | FORCE (에스컬레이션) |
| 센서 (2순위) | FORCE (21196) | FORCE |
| 기압계 (3순위) | FORCE (21196) | FORCE |
| 비상 타임아웃 | FORCE (21196) | FORCE |

### 센서→PX4 업그레이드

거리계/기압계로 먼저 착지 감지 중 PX4 `land_detected=true` 수신 시,
감지 방법을 PX4(1순위)로 업그레이드 → NORMAL DISARM 사용 (스키드 들림 방지).

---

## 데이터 흐름

```
vehicle_land_detected (DDS) → fc_bridge_server.cpp (람다 콜백)
    → FCState.land_detected (packed struct UDP 17001)
    → fc_bridge_client.getLatestState()
    → offboard_manager.cpp :: updateFromFCState() → land_detected_ 멤버 저장
    → syncContextFromMembers() → ctx_.land_detected
    → rtl_handler.h :: tick() 에서 ctx.land_detected.load() 사용
```

---

## 변경 대상 파일 (7개)

### 1. `navigation/src/offboard/bridge/fc_bridge_protocol.h`

FCState packed 구조체에 필드 추가 (`fc_connected` 바로 위):

```cpp
uint8_t land_detected;    // 1 = PX4가 착지 확인
uint8_t maybe_landed;     // 1 = 착지 가능성 있음
```

주의: `fc_connected` 오프셋 2바이트 이동 → fc_bridge + application 동시 배포 필수.

### 2. `navigation/src/offboard/bridge/fc_bridge_server.h`

```cpp
#include <px4_msgs/msg/vehicle_land_detected.hpp>
rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
```

### 3. `navigation/src/offboard/bridge/fc_bridge_server.cpp`

VehicleLandDetected 구독 람다 추가.

### 4. `navigation/src/offboard/mission_context.h`

```cpp
std::atomic<bool> land_detected{false};
std::atomic<bool> maybe_landed{false};
```

### 5. `navigation/src/offboard/offboard_manager.h`

```cpp
std::atomic<bool> land_detected_{false};
std::atomic<bool> maybe_landed_{false};
```

### 6. `navigation/src/offboard/offboard_manager.cpp`

`updateFromFCState()` + `syncContextFromMembers()` 에서 전파.

### 7. `navigation/src/offboard/handlers/rtl_handler.h` — 핵심 변경

#### 추가된 상수

```cpp
// 1순위 PX4
static constexpr float GROUND_STABLE_SEC_PX4 = 0.5f;

// 2순위 거리계+수직속도
static constexpr float DIST_BOTTOM_LAND_THRESHOLD = 0.35f;  // 거리 상한 (장착높이+마진)
static constexpr float DIST_STABLE_DELTA = 0.03f;           // 안정 판정: 변화량 < 3cm
static constexpr int   DIST_STABLE_TICKS = 30;              // 안정 판정: 30틱 (3초 @10Hz)
static constexpr float GROUND_STABLE_SEC_SENSOR = 0.5f;     // 감지 후 추가 안정화

// 3순위 기압계+수직속도
static constexpr float GROUND_THRESHOLD = 0.3f;             // AGL 임계값
static constexpr float GROUND_STABLE_SEC_BARO = 1.0f;       // 감지 후 안정화

// 수직속도 공통
static constexpr float VZ_STABLE_THRESHOLD = 0.05f;         // vz 안정 판정
static constexpr int   VZ_STABLE_TICKS = 30;                // 30틱 (3초 @10Hz)
```

#### 추가된 멤버 변수

```cpp
bool land_detected_by_px4_{false};     // 1순위 PX4 감지 → NORMAL DISARM
bool land_detected_by_sensor_{false};  // 2순위 거리계+vz 감지 → FORCE DISARM
float prev_dist_bottom_{-1.0f};        // 이전 tick 거리계 값
int dist_stable_count_{0};             // 거리 안정 연속 카운트
int vz_stable_count_{0};              // vz 안정 연속 카운트
```

#### landing_speed_min 클램프 체인

```cpp
// onEnter()에서:
landing_speed_min_ = std::max(ctx.rtl_landing_speed_min, VZ_STABLE_THRESHOLD * 2.0f);
// → 최소 0.1 m/s (거리계 없을 때 vz 기반 3순위 오판 방지)

landing_speed_min_ = std::min(landing_speed_min_, descent_speed_);
// → 속도역전 방지: landing_speed_min이 descent_speed보다 크면 고도 낮아질수록 속도 증가하는 역전 현상 발생
```

#### calcLandingSpeed 0나눗셈 방어

```cpp
// calcLandingSpeed() 진입부:
if (soft_land_alt_ <= 0.0f) return landing_speed_min_;
// → soft_land_alt=0일 때 ratio = agl / 0 → INF/NaN 방지
```

#### SOFT_LAND — 삼중화 착지 감지

매 tick마다:
1. `vz_stable_count_` 업데이트 (|vz| < 0.05 연속 카운트)
2. `dist_stable_count_` 업데이트 (|dist_delta| < 0.03 연속 카운트)
3. 우선순위별 감지:
   - PX4 `land_detected` → 즉시 감지 + 0.5초 안정화
   - 거리계: `dist_stable(3초) + vz_stable(3초) + dist < 0.35m` → 감지 + 0.5초 안정화
   - 기압계: `agl < 0.3m + vz_stable(3초)` → 감지 + 1.0초 안정화
4. 센서→PX4 업그레이드 지원
5. 비PX4 감지는 조건 불충족 시 리셋 (PX4 감지는 절대 리셋 안 됨)

#### GROUND_DISARM — DISARM 에스컬레이션

PX4 감지: 1-2회 NORMAL → 3회차 FORCE
센서/기압계/비상: 처음부터 FORCE

---

## 변경하지 않는 것

- NAVIGATE_HOME, DESCEND 페이즈 로직
- GROUND_DISARM 재시도 로직 (3초 간격 x 최대 5회)
- GROUND_DISARM 타임아웃 20초
- SOFT_LAND 동적 타임아웃 (`calcDynamicSoftLandTimeout`)
- SOFT_LAND 비상 타임아웃 180초

---

## 안전 분석

### 공중 오감지(false positive) 방지

- **PX4**: 가속도계 충격 필요 → 비행 중 발생 거의 없음. 만약 발생해도 NORMAL DISARM → PX4가 거부
- **거리계**: dist_stable 3초 + vz_stable 3초 동시 충족 필요. 하강 중 vz > 0.05이므로 vz_stable 불충족
- **기압계**: AGL < 0.3m + vz_stable 3초. 하강 중 vz > 0.05이므로 불충족. 기압 드리프트로 AGL < 0.3 되어도 vz가 0이 아니면 안전

### 시동 안 꺼지는 경우 방지

- 삼중화: PX4 → 거리계+vz → 기압계+vz 중 하나라도 동작
- NORMAL DISARM 거부 시 FORCE 에스컬레이션 (3회차)
- 비상 타임아웃 180초 → FORCE DISARM
- GROUND_DISARM 타임아웃 20초 → COMPLETE 강제

### 한쪽 스키드 들림 해결

- PX4 감지 시 NORMAL DISARM → 자세제어 부드러운 종료 + 모터 램프다운

### 장애물 위 착지

- 거리계+vz가 동시에 안정 = 물리적으로 무언가에 닿음 → 장애물이든 지면이든 DISARM이 맞음

### 엣지 케이스

| 시나리오 | 동작 | 결과 |
|----------|------|------|
| 정상 착지 (PX4 감지) | 0.5초 → NORMAL | 정상 종료 |
| PX4 미감지 + 거리계 안정 3초 + vz 안정 3초 | 0.5초 → FORCE | 정상 종료 |
| PX4 미감지 + 거리계 무효 + 기압 낮음 + vz 안정 3초 | 1.0초 → FORCE | 정상 종료 |
| NORMAL DISARM 거부 | 3회차 FORCE 에스컬 | 안전 종료 |
| 지면효과 dist 진동 ±5cm | 3cm 초과 → dist_stable 리셋 | 오감지 방지 |
| landing_speed 0.04 설정 시 | 0.1로 클램프 | 하강 중 vz_stable 방지 |
| landing_speed_min(0.3) > descent_speed(0.1) | descent_speed로 클램프 | 속도역전 방지 |
| soft_land_alt = 0 설정 시 | landing_speed_min 즉시 반환 | 0나눗셈 방지 |
| 180초 모든 센서 실패 | FORCE(비상) | 비상 종료 |

---

## 배포 주의사항

FCState packed struct 크기 변경 (2바이트 추가) → **fc_bridge와 application 동시 배포 필수**.

---

## 검증 시나리오

1. **정상 착지 (PX4 감지)**: SOFT_LAND → land_detected=true → 0.5초 → NORMAL DISARM
2. **거리계+vz 착지**: dist 안정 3초 + vz 안정 3초 + dist < 0.35m → 0.5초 → FORCE DISARM
3. **센서→PX4 업그레이드**: 거리계 먼저 감지 → PX4 확인 → NORMAL DISARM 전환
4. **기압계 폴백**: 거리계 무효 + AGL < 0.3m + vz 안정 3초 → 1.0초 → FORCE DISARM
5. **NORMAL DISARM 거부 에스컬레이션**: 1-2회 NORMAL 실패 → 3회차 FORCE
6. **반복 테스트**: home_alt 변동에도 PX4/거리계 기반으로 일관 동작
7. **공중 오감지 방지**: 하강 중 vz > 0.05 → vz_stable 불충족 확인
8. **비상 타임아웃**: 180초 → FORCE DISARM
9. **SITL 검증**: Gazebo에서 vehicle_land_detected + dist_bottom 흐름 확인

---

## 관련 파일

- `navigation/src/offboard/bridge/fc_bridge_protocol.h`
- `navigation/src/offboard/bridge/fc_bridge_server.h`
- `navigation/src/offboard/bridge/fc_bridge_server.cpp`
- `navigation/src/offboard/mission_context.h`
- `navigation/src/offboard/offboard_manager.h`
- `navigation/src/offboard/offboard_manager.cpp`
- `navigation/src/offboard/handlers/rtl_handler.h`
