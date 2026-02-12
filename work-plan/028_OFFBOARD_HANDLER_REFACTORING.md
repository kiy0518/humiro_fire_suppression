# OffboardManager 커맨드 핸들러 리팩토링

## 배경

현재 `offboard_manager.cpp`(1088줄)은 카운터 기반 monolithic 상태 머신.
고정 시간 전환(2s, 4.5s, 8s 등)으로 실제 FC 상태와 무관하게 진행됨.

## 변경 내용

### Before (카운터 기반)
```
counter == 20  → OFFBOARD 명령 전송 (2초 후 무조건)
counter == 45  → ARM 명령 전송 (4.5초 후 무조건)
counter == 80  → TAKEOFF 완료 간주 (8초 후 무조건)
```

### After (조건 기반)
```
nav_state == 14     → OFFBOARD 확인됨
arming_state == 2   → ARM 확인됨
고도 오차 < 0.5m    → TAKEOFF 완료
yaw 오차 < 0.05rad  → ROTATE 완료
거리 < 2.0m         → NAVIGATE 도착
```

## 아키텍처

```
StateHandler (인터페이스)
  ├── onEnter()       : 상태 진입 시 1회 실행
  ├── onExit()        : 상태 퇴출 시 1회 실행
  ├── tick()          : 10Hz 호출, TransitionResult 반환
  └── fillSetpoint()  : TrajectorySetpoint 생성

MissionContext (공유 상태)
  └── 모든 핸들러가 참조하는 vehicle 상태, 미션 파라미터, 편대/충돌 상태

OffboardManager.timerCallback() (50줄)
  1. publishOffboardControlMode()  — 10Hz 하트비트
  2. abort 체크
  3. current_handler_->tick(ctx_)
  4. advanceToNextHandler() 또는 transitionTo(rtl)
  5. fillSetpoint() + collision override + publish
```

## 핸들러 목록

| # | 핸들러 | 전환 조건 | 타임아웃 |
|---|--------|-----------|----------|
| 1 | PrepareHandler | 2s 경과 | - |
| 2 | OffboardHandler | nav_state == 14 | 10s→ERROR |
| 3 | ArmHandler | arming_state == 2 | 15s→ERROR |
| 4 | TakeoffHandler | 고도 오차 < 0.5m, 1초 안정 | 30s→RTL |
| 5 | HoverHandler | 2초 + 편대 게이트 | 편대: 무한 |
| 6 | RotateHandler | yaw 오차 < 0.05rad + 편대 게이트 | 20s→강제 |
| 7 | NavigateHandler | 거리 < 2.0m | - |
| 8 | HoverAtTargetHandler | 5초(solo) / 30초(formation) | - |
| 9 | RtlHandler | arming_state == 1 | - |

## 전체 미션 시퀀스 (단일 비행 기준)

```
STEP 1: PREPARE      — heartbeat 2초
STEP 2: OFFBOARD     — FC 모드 전환 확인
STEP 3: ARM          — 시동 확인
STEP 4: TAKEOFF      — 목표 고도 도달
STEP 5: HOVER        — 안정화 + 편대 게이트
STEP 6: ROTATE       — 목표 방향 회전
STEP 7: NAVIGATE     — 목표 위치 이동 (편대 구성 비행 고려)
STEP 8: 타겟 거리 조정  — 진압 최적 거리로 접근 (TODO)
STEP 9: 조준 및 격발    — 리더: 자동 조준+격발
                        팔로워: 리더 좌표 기준 격발위치 이동, 자동조준+격발 (TODO)
STEP 10: 복귀 (RTL)   — PX4 AUTO_RTL
STEP 11: 부드러운 랜딩  — 도착 후 안전 착륙 (TODO)
```

> **설계 원칙**: 일단 단일 비행으로 코드를 작성하되, 추후 편대비행을 염두에 두고 구조를 설계.
> STEP 7(NAVIGATE)에서도 편대 구성 비행 부분을 고려해야 함.

## 핸들러 목록

### 구현 완료

| # | 핸들러 | 전환 조건 | 타임아웃 |
|---|--------|-----------|----------|
| 1 | PrepareHandler | 2s 경과 | - |
| 2 | OffboardHandler | nav_state == 14 | 10s→ERROR |
| 3 | ArmHandler | arming_state == 2 | 15s→ERROR |
| 4 | TakeoffHandler | 고도 오차 < 0.5m, 1초 안정 | 30s→RTL |
| 5 | HoverHandler | hover_duration_sec + 편대 게이트 | 편대: 무한 |
| 6 | RotateHandler | yaw 오차 < 0.05rad + 편대 게이트 | 30s→RTL |
| 7 | NavigateHandler | 거리 < 2.0m | - |
| 8 | HoverAtTargetHandler | 5초(solo) / 30초(formation) | - |
| 9 | RtlHandler | arming_state == 1 (disarm) | - |

### 추가 예정 (STEP 8~11)

| # | 핸들러 (예정) | 설명 |
|---|--------------|------|
| 10 | DistanceAdjustHandler | 타겟과의 거리 조정 (진압 최적 거리 접근) |
| 11 | AimFireHandler | 리더: 자동 조준 및 격발 / 팔로워: 리더 기체 좌표 기준 격발위치 이동, 자동조준, 격발 |
| 12 | SmoothLandHandler | 도착 후 부드러운 랜딩 (RTL 후 또는 별도) |

## 변경하지 않는 것

- GCS 인터페이스 (FIRE_MISSION_START 60000)
- executeMission3() / executeMission4() 공개 API
- FormationController, CollisionAvoidance 코드
- ApplicationManager 코드

## 구현 순서

### Phase 1: 기본 핸들러 리팩토링 (완료)

1. ✅ 인프라 (mission_context.h, state_handler.h)
2. ✅ PREPARE + OFFBOARD + ARM → 빌드 확인
3. ✅ TAKEOFF → 빌드 확인
4. ✅ HOVER + ROTATE → 빌드 확인
5. ✅ NAVIGATE → 빌드 확인
6. ✅ HOVER_AT_TARGET + RTL → 빌드 확인
7. SITL 통합 테스트

### Phase 2: 진압 기능 확장 (예정)

8. STEP 8: DistanceAdjustHandler — 타겟 접근 거리 조정
9. STEP 9: AimFireHandler — 자동 조준+격발 (리더/팔로워 분기)
10. STEP 10: 복귀 로직 개선 (RTL 경로 최적화)
11. STEP 11: SmoothLandHandler — 부드러운 착륙

### Phase 3: 편대 비행 통합 (예정)

12. NAVIGATE 핸들러에 편대 구성 비행 로직 추가
13. AimFireHandler 팔로워 모드 (리더 좌표 기준 격발위치 계산)
14. 편대 전체 미션 시퀀스 테스트
