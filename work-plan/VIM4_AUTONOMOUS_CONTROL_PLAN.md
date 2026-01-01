# 자동 제어 시스템 개발 계획 (Autonomous Control System Development Plan)

작성일: 2026-01-01  
**상태**: 계획 단계

---

## 개요

VIM4에서 실행되는 자동 제어 시스템의 메시지 흐름 및 기능을 정의합니다.

**중요 용어 구분**:
- **PX4 Mission 모드** (`AUTO_MISSION`): QGC에서 미션을 업로드하고 PX4가 자동으로 실행하는 모드 (ArduPilot의 AUTO 모드와 유사)
- **VIM4 자동 제어 시스템** (`OFFBOARD` 모드 사용): VIM4에서 ROS2를 통해 드론을 제어하는 커스텀 자동 시스템
- **상태 모니터링 OSD**: 모든 비행 모드에서 동작하는 기체 상태 표시 시스템

**시스템 구성**:
- **VIM4**: 자동 제어 로직 실행 (OFFBOARD 모드)
- **FC (Flight Controller)**: PX4 기반 비행 제어
- **QGC (QGroundControl)**: 지상 관제소 (모니터링 및 명령 수신)

---

## 메시지 흐름 분류

### 1. VIM4 → FC (Flight Controller) 메시지

**목적**: 드론 비행 제어 명령 전송

#### 1.1 시동 (Arm)
- **메시지 타입**: `COMMAND_LONG` (MAVLink)
- **명령 코드**: `MAV_CMD_COMPONENT_ARM_DISARM`
- **파라미터**:
  - `param1`: 1 (arm), 0 (disarm)
  - `param2`: 0 (force disarm)
- **전송 주기**: 일회성 (명령 시)
- **구현 위치**: `navigation/src/auto_mode/arm_handler.cpp`

#### 1.2 이륙 (Takeoff)
- **메시지 타입**: `COMMAND_LONG` (MAVLink)
- **명령 코드**: `MAV_CMD_NAV_TAKEOFF`
- **파라미터**:
  - `param7`: 고도 (미터)
- **전송 주기**: 일회성 (명령 시)
- **구현 위치**: `navigation/src/auto_mode/takeoff_handler.cpp`

#### 1.3 이동 (Waypoint Navigation)
- **메시지 타입**: `SET_POSITION_TARGET_LOCAL_NED` (MAVLink)
- **좌표계**: Local NED (North-East-Down)
- **파라미터**:
  - `x`, `y`, `z`: 목표 위치 (미터)
  - `vx`, `vy`, `vz`: 속도 벡터 (m/s, 선택적)
  - `type_mask`: 제어 모드 플래그
- **전송 주기**: 10Hz (100ms 간격)
- **구현 위치**: `navigation/src/auto_mode/waypoint_handler.cpp`

#### 1.4 대기 (Hover/Hold Position)
- **메시지 타입**: `SET_POSITION_TARGET_LOCAL_NED` (MAVLink)
- **파라미터**:
  - 현재 위치 유지 (속도 = 0)
  - `type_mask`: 위치 고정 모드
- **전송 주기**: 10Hz (100ms 간격)
- **구현 위치**: `navigation/src/auto_mode/hover_handler.cpp`

#### 1.5 거리 조정 (Distance Adjustment)
- **메시지 타입**: `SET_POSITION_TARGET_LOCAL_NED` (MAVLink)
- **목적**: 타겟과의 거리를 10m로 유지
- **파라미터**:
  - LiDAR 거리 데이터 기반 위치 계산
  - 전방 방향으로 미세 조정
- **전송 주기**: 20Hz (50ms 간격, 빠른 반응 필요)
- **구현 위치**: `navigation/src/auto_mode/distance_adjustment_handler.cpp`

#### 1.6 오토타겟팅 (Auto Targeting)
- **메시지 타입**: `SET_POSITION_TARGET_LOCAL_NED` (MAVLink)
- **목적**: 핫스팟을 화면 중심에 유지
- **파라미터**:
  - 핫스팟 위치 오차 기반 미세 조정
  - 상하좌우 위치 보정
- **전송 주기**: 20Hz (50ms 간격)
- **구현 위치**: `navigation/src/auto_mode/auto_targeting_handler.cpp`

#### 1.7 복귀 (Return to Launch)
- **메시지 타입**: `COMMAND_LONG` (MAVLink)
- **명령 코드**: `MAV_CMD_NAV_RETURN_TO_LAUNCH`
- **전송 주기**: 일회성 (명령 시)
- **구현 위치**: `navigation/src/auto_mode/rtl_handler.cpp`

---

### 2. VIM4 → QGC (QGroundControl) 메시지

**목적**: 상태 정보 및 알림 전송

#### 2.1 복귀 목적지 도착 알림
- **메시지 타입**: `STATUSTEXT` (MAVLink)
- **텍스트**: "Return destination reached" 또는 사용자 정의 메시지
- **레벨**: `MAV_SEVERITY_INFO` 또는 `MAV_SEVERITY_NOTICE`
- **전송 주기**: 일회성 (도착 시)
- **구현 위치**: `navigation/src/auto_mode/rtl_handler.cpp`

#### 2.2 자동모드 상태 정보
- **메시지 타입**: `HEARTBEAT` (MAVLink) + 커스텀 상태
- **상태 정보**:
  - 현재 모드 (시동, 이륙, 이동, 대기, 타겟팅, 복귀 등)
  - 타겟 거리 (LiDAR)
  - 핫스팟 추적 상태
- **전송 주기**: 1Hz (1초 간격)
- **구현 위치**: `navigation/src/auto_mode/status_reporter.cpp`

#### 2.3 타겟팅 상태
- **메시지 타입**: 커스텀 메시지 또는 `STATUSTEXT`
- **정보**:
  - "LOCKED" 상태 (정조준 완료)
  - 타겟팅 오차
- **전송 주기**: 2Hz (0.5초 간격)
- **구현 위치**: `navigation/src/auto_mode/auto_targeting_handler.cpp`

#### 2.4 거리 정보
- **메시지 타입**: 커스텀 메시지 또는 `DISTANCE_SENSOR` (MAVLink)
- **정보**: LiDAR 거리 데이터
- **전송 주기**: 10Hz (100ms 간격)
- **구현 위치**: `navigation/src/auto_mode/distance_reporter.cpp`

---

### 3. QGC → VIM4 메시지 (수신)

**목적**: 지상 관제소로부터 명령 수신

#### 3.1 자동모드 시작/중지
- **메시지 타입**: 커스텀 ROS2 토픽 또는 MAVLink `COMMAND_LONG`
- **명령 코드**: 커스텀 (예: `AUTO_MODE_START`, `AUTO_MODE_STOP`)
- **파라미터**:
  - 목표 좌표 (위도, 경도, 고도)
  - 모드 선택 (전체 자동, 수동 개입 가능 등)
- **구현 위치**: `navigation/src/auto_mode/command_receiver.cpp`

#### 3.2 격발 명령
- **메시지 타입**: ROS2 토픽 `/fire_command` (기존 구현)
- **데이터 타입**: `std_msgs::msg::Bool`
- **동작**: `true` 시 격발 실행
- **구현 위치**: `targeting/src/hotspot_tracker.cpp` (기존) + `throwing_mechanism/`

#### 3.3 비상 정지
- **메시지 타입**: `COMMAND_LONG` (MAVLink)
- **명령 코드**: `MAV_CMD_COMPONENT_ARM_DISARM` (disarm) 또는 커스텀
- **전송 주기**: 즉시 처리
- **구현 위치**: `navigation/src/auto_mode/emergency_handler.cpp`

#### 3.4 목표 좌표 변경
- **메시지 타입**: `SET_POSITION_TARGET_GLOBAL_INT` (MAVLink)
- **파라미터**: 새로운 목표 좌표
- **구현 위치**: `navigation/src/auto_mode/waypoint_handler.cpp`

---

### 4. VIM4 단독 기능 (외부 통신 불필요)

**목적**: VIM4 내부에서만 처리되는 기능

#### 4.1 LiDAR 거리 모니터링
- **입력**: LD19 LiDAR 센서 (UART)
- **처리**: 거리 데이터 수집 및 필터링
- **출력**: 내부 상태 변수 업데이트
- **구현 위치**: `lidar/src/lidar_interface.cpp` (기존)

#### 4.2 열화상 핫스팟 감지
- **입력**: PureThermal 카메라
- **처리**: 핫스팟 위치 계산
- **출력**: 내부 상태 변수 업데이트
- **구현 위치**: `thermal/src/thermal_processor.cpp` (기존)

#### 4.3 타겟팅 오차 계산
- **입력**: 핫스팟 위치, 화면 중심 좌표
- **처리**: 오차 벡터 계산 (픽셀 단위)
- **출력**: 위치 보정 명령 생성
- **구현 위치**: `targeting/src/hotspot_tracker.cpp` (기존) + `navigation/src/auto_mode/auto_targeting_handler.cpp`

#### 4.4 거리 조정 로직
- **입력**: LiDAR 거리 데이터, 목표 거리 (10m)
- **처리**: 거리 차이 계산 및 위치 보정 명령 생성
- **출력**: FC로 위치 명령 전송
- **구현 위치**: `navigation/src/auto_mode/distance_adjustment_handler.cpp`

#### 4.5 자동모드 상태 머신
- **상태**: 
  - `IDLE`: 대기
  - `ARMING`: 시동 중
  - `TAKEOFF`: 이륙 중
  - `NAVIGATING`: 이동 중
  - `HOVERING`: 대기 중
  - `TARGETING`: 타겟팅 중
  - `FIRING`: 격발 중
  - `RETURNING`: 복귀 중
  - `LANDING`: 착륙 중
- **전이 조건**: 각 상태별 완료 조건
- **구현 위치**: `navigation/src/auto_mode/state_machine.cpp`

#### 4.6 격발 메커니즘 제어
- **입력**: 격발 명령 (QGC 또는 내부)
- **처리**: 서보 제어, 발사 트리거
- **출력**: GPIO 신호
- **구현 위치**: `throwing_mechanism/src/fire_controller.cpp`

---

## 기능별 상세 구현 계획

### Phase 1: 기본 비행 제어 (1주)

#### 1.1 시동 (Arm)
- [ ] `arm_handler.cpp` 구현
- [ ] MAVLink `COMMAND_LONG` 전송
- [ ] 시동 상태 확인 (ACK 대기)
- [ ] 에러 처리 (시동 실패 시 재시도)

#### 1.2 이륙 (Takeoff)
- [ ] `takeoff_handler.cpp` 구현
- [ ] 목표 고도 설정
- [ ] 이륙 완료 확인 (고도 도달)
- [ ] 에러 처리

#### 1.3 이동 (Waypoint Navigation)
- [ ] `waypoint_handler.cpp` 구현
- [ ] 좌표 변환 (GPS → Local NED)
- [ ] 경로 계획 (직선 이동)
- [ ] 도착 판단 (목표 반경 내)
- [ ] 에러 처리 (경로 이탈)

#### 1.4 대기 (Hover)
- [ ] `hover_handler.cpp` 구현
- [ ] 현재 위치 유지
- [ ] 위치 안정화 (드리프트 보정)

---

### Phase 2: 복귀 및 알림 (1주)

#### 2.1 복귀 (Return to Launch)
- [ ] `rtl_handler.cpp` 구현
- [ ] 복귀 경로 계산
- [ ] 복귀 완료 판단 (목적지 도착)
- [ ] QGC 알림 전송 (`STATUSTEXT`)

#### 2.2 상태 보고
- [ ] `status_reporter.cpp` 구현
- [ ] 주기적 상태 전송 (1Hz)
- [ ] 커스텀 상태 메시지 정의

---

### Phase 3: 타겟팅 및 거리 조정 (2주)

#### 3.1 거리 조정
- [ ] `distance_adjustment_handler.cpp` 구현
- [ ] LiDAR 데이터 연동
- [ ] 목표 거리 (10m) 유지 로직
- [ ] 전방 방향 미세 조정
- [ ] 안정화 로직 (오실레이션 방지)

#### 3.2 오토타겟팅
- [ ] `auto_targeting_handler.cpp` 구현
- [ ] 핫스팟 위치 연동 (`targeting/`)
- [ ] 화면 중심 오차 계산
- [ ] 위치 보정 명령 생성
- [ ] "LOCKED" 상태 판단
- [ ] QGC 상태 전송

---

### Phase 4: 격발 및 통합 (1주)

#### 4.1 격발 메커니즘
- [ ] `fire_controller.cpp` 구현
- [ ] QGC 격발 명령 수신
- [ ] 정조준 완료 확인 (LOCKED)
- [ ] 서보 제어
- [ ] 발사 트리거

#### 4.2 상태 머신 통합
- [ ] `state_machine.cpp` 구현
- [ ] 모든 상태 통합
- [ ] 상태 전이 로직
- [ ] 에러 처리 및 복구

#### 4.3 전체 시스템 통합
- [ ] 모든 모듈 통합 테스트
- [ ] 메시지 흐름 검증
- [ ] 실시간 성능 최적화

---

## 메시지 통신 구조

### MAVLink 메시지 흐름

```
┌─────────┐         ┌─────────┐         ┌─────────┐
│   QGC   │◄───────►│  VIM4   │◄───────►│   FC   │
└─────────┘         └─────────┘         └─────────┘
     │                   │                   │
     │  상태/알림        │  비행 명령        │
     │  (수신)           │  (전송)           │
     │                   │                   │
     │  명령             │  상태             │
     │  (전송)           │  (수신)           │
```

### ROS2 토픽 구조

```
/auto_mode/command          (std_msgs::msg::Bool)      # 자동모드 시작/중지
/auto_mode/status           (custom_msgs::msg::Status) # 자동모드 상태
/fire_command               (std_msgs::msg::Bool)      # 격발 명령 (기존)
/lidar/distance             (sensor_msgs::msg::Range)  # LiDAR 거리 (기존)
/thermal/hotspot            (custom_msgs::msg::Hotspot) # 핫스팟 위치 (기존)
```

---

## 파일 구조

```
navigation/
├── src/
│   └── auto_mode/
│       ├── arm_handler.cpp              # 시동
│       ├── takeoff_handler.cpp          # 이륙
│       ├── waypoint_handler.cpp        # 이동
│       ├── hover_handler.cpp           # 대기
│       ├── rtl_handler.cpp             # 복귀
│       ├── distance_adjustment_handler.cpp  # 거리 조정
│       ├── auto_targeting_handler.cpp  # 오토타겟팅
│       ├── status_reporter.cpp         # 상태 보고
│       ├── command_receiver.cpp        # 명령 수신
│       ├── emergency_handler.cpp       # 비상 정지
│       ├── state_machine.cpp           # 상태 머신
│       └── auto_mode_manager.cpp       # 통합 관리자
├── include/
│   └── auto_mode/
│       ├── arm_handler.h
│       ├── takeoff_handler.h
│       ├── waypoint_handler.h
│       ├── hover_handler.h
│       ├── rtl_handler.h
│       ├── distance_adjustment_handler.h
│       ├── auto_targeting_handler.h
│       ├── status_reporter.h
│       ├── command_receiver.h
│       ├── emergency_handler.h
│       ├── state_machine.h
│       └── auto_mode_manager.h
└── CMakeLists.txt
```

---

## 의존성

### 기존 모듈 활용
- `lidar/`: 거리 데이터 제공
- `thermal/`: 핫스팟 위치 제공
- `targeting/`: 타겟팅 오차 계산

### 외부 라이브러리
- **MAVLink**: PX4 통신
- **ROS2**: 메시지 통신
- **PX4 ROS2 Interface**: 드론 제어

---

## 테스트 계획

### 단위 테스트
- 각 핸들러별 독립 테스트
- 메시지 전송/수신 검증
- 상태 전이 로직 검증

### 통합 테스트
- 전체 자동모드 시나리오 테스트
- QGC 연동 테스트
- FC 연동 테스트 (시뮬레이터)

### 실전 테스트
- 실제 드론 환경에서 테스트
- 각 단계별 검증
- 에러 복구 테스트

---

## 우선순위

### 높음 (즉시 시작)
1. 시동 (Arm)
2. 이륙 (Takeoff)
3. 이동 (Waypoint Navigation)
4. 대기 (Hover)

### 중간 (1주 내)
5. 복귀 (Return to Launch)
6. 복귀 목적지 도착 알림

### 낮음 (2주 내)
7. 거리 조정
8. 오토타겟팅
9. 격발

---

## 참고사항

- 모든 MAVLink 메시지는 표준 프로토콜 준수
- ROS2 토픽은 기존 구조와 호환성 유지
- 에러 처리 및 복구 로직 필수
- 실시간 성능 고려 (20Hz 이상 제어 루프)

---

## 관련 문서

**기체 상태 모니터링 OSD**: 별도 문서 참조
- 파일: `work-plan/STATUS_MONITORING_PLAN.md`
- 설명: 모든 비행 모드에서 동작하는 기체 상태 표시 시스템
- 통합: VIM4 자동 제어 시스템과 독립적으로 동작하지만, OFFBOARD 모드일 때는 `/auto_mode/status` 토픽을 구독하여 상세 상태를 표시

---

---

## 참고사항

### PX4 비행 모드와 상태 매핑

| PX4 모드 | 상태 표시 | 설명 |
|---------|----------|------|
| `MANUAL` | 수동 비행 | 수동 조종 모드 |
| `STABILIZED` | 수동 비행 | 안정화 모드 |
| `POSCTL` | 수동 비행 | 위치 제어 모드 |
| `AUTO_MISSION` | 이동중/목적지도착/임무완료 | QGC Mission 모드 |
| `AUTO_RTL` | 복귀중/착륙 | 자동 복귀 모드 |
| `AUTO_LAND` | 착륙 | 자동 착륙 모드 |
| `AUTO_LOITER` | 대기 | 자동 대기 모드 |
| `AUTO_TAKEOFF` | 이륙 | 자동 이륙 모드 |
| `OFFBOARD` | 자동모드 상태 확인 | 오프보드 모드 (자동모드 포함) |

### PX4 Mission 모드 동작 (AUTO_MISSION)

**참고**: ArduPilot의 AUTO 모드와 유사하지만, PX4에서는 `AUTO_MISSION` 모드입니다.

QGC에서 Mission을 업로드하고 실행하면:
1. PX4가 `AUTO_MISSION` 모드로 전환
2. `/mavros/state` 토픽의 `mode` 필드가 `"AUTO.MISSION"`으로 변경
3. StatusOverlay가 이를 감지하여 "이동중" 상태로 표시
4. Waypoint 도착 시 "목적지도착" 상태로 변경 (MAVLink `MISSION_ITEM_REACHED` 메시지 활용)
5. 모든 Waypoint 완료 시 "임무완료" 상태로 변경

### VIM4 자동 제어 시스템 동작 (OFFBOARD)

VIM4에서 ROS2를 통해 드론을 제어할 때:
1. PX4가 `OFFBOARD` 모드로 전환
2. `/mavros/state` 토픽의 `mode` 필드가 `"OFFBOARD"`로 변경
3. StatusOverlay가 PX4 모드를 감지하고, 추가로 `/auto_mode/status` 토픽을 구독
4. VIM4 자동 제어 시스템의 상태(시동, 이륙, 이동중, 격발대기 등)를 표시
5. PX4 Mission 모드와는 별개로 동작하는 커스텀 자동 제어 시스템

---

**작성자**: Claude Code Assistant  
**버전**: v1.3 (용어 정리: PX4 Mission 모드와 VIM4 자동 제어 시스템 구분)  
**작성일**: 2025-01-01  
**다음 리뷰**: Phase 1 완료 시

