# 자동 제어 시스템 개발 계획 (Autonomous Control System Development Plan)

작성일: 2026-01-01  
**상태**: 계획 단계

---

## 개요

VIM4에서 실행되는 자동 제어 시스템의 메시지 흐름 및 기능을 정의합니다.  
**기반**: 화재 진압 드론 임무 플로우차트

**중요 용어 구분**:
- **PX4 Mission 모드** (`AUTO_MISSION`): QGC에서 미션을 업로드하고 PX4가 자동으로 실행하는 모드 (ArduPilot의 AUTO 모드와 유사)
- **VIM4 자동 제어 시스템** (`OFFBOARD` 모드 사용): VIM4에서 ROS2를 통해 드론을 제어하는 커스텀 자동 시스템
- **상태 모니터링 OSD**: 모든 비행 모드에서 동작하는 기체 상태 표시 시스템

**시스템 구성**:
- **VIM4**: 자동 제어 로직 실행 (OFFBOARD 모드)
- **FC (Flight Controller)**: PX4 기반 비행 제어
- **QGC (QGroundControl)**: 지상 관제소 (모니터링 및 명령 수신)

**통신 방향 구분** (플로우차트 색상 기준):
- **🟡 노란색**: QGC → DRONE(FC) 또는 VIM4 → FC (비행 명령)
- **🟢 초록색**: DRONE(VIM4) → QGC (상태 알림)
- **🔴 빨간색**: QGC → DRONE(VIM4) (격발 명령)
- **🔵 파란색**: DRONE 단독 (VIM4 내부 처리)

**상세 토픽 구조**: `ROS2_TOPIC_ARCHITECTURE.md` 참조

---

## 메시지 흐름 분류

### 1. VIM4 → FC (Flight Controller) 메시지 🟡

**목적**: 드론 비행 제어 명령 전송 (플로우차트 노란색, QGC → FC와 동일한 방식)

#### 1.1 시동 (Arm)
- **ROS2 토픽**: `/fmu/in/vehicle_command`
- **메시지 타입**: `px4_msgs::VehicleCommand`
- **명령 코드**: `MAV_CMD_COMPONENT_ARM_DISARM`
- **파라미터**:
  - `param1`: 1 (arm), 0 (disarm)
  - `param2`: 0 (force disarm)
- **전송 주기**: 일회성 (명령 시)
- **구현 위치**: `navigation/src/offboard/arm_handler.cpp`
- **플로우차트**: "시동" (노란색)

#### 1.2 이륙 (Takeoff)
- **ROS2 토픽**: `/fmu/in/vehicle_command`
- **메시지 타입**: `px4_msgs::VehicleCommand`
- **명령 코드**: `MAV_CMD_NAV_TAKEOFF`
- **파라미터**:
  - `param7`: 고도 (미터)
- **전송 주기**: 일회성 (명령 시)
- **구현 위치**: `navigation/src/offboard/takeoff_handler.cpp`
- **플로우차트**: "이륙" (노란색)

#### 1.3 이동 (Waypoint Navigation)
- **ROS2 토픽**: `/fmu/in/trajectory_setpoint`
- **메시지 타입**: `px4_msgs::TrajectorySetpoint`
- **좌표계**: Local NED (North-East-Down)
- **파라미터**:
  - `position`: 목표 위치 (x, y, z) 미터
  - `velocity`: 속도 벡터 (vx, vy, vz) m/s (선택적)
  - `yaw`: 방향 (라디안)
- **전송 주기**: 10Hz (100ms 간격)
- **구현 위치**: `navigation/src/offboard/waypoint_handler.cpp`
- **플로우차트**: "이동 (편대비행, 충돌방지)" (노란색)

#### 1.4 대기 (Hover/Hold Position)
- **ROS2 토픽**: `/fmu/in/trajectory_setpoint`
- **메시지 타입**: `px4_msgs::TrajectorySetpoint`
- **파라미터**:
  - 현재 위치 유지 (속도 = 0)
- **전송 주기**: 10Hz (100ms 간격)
- **구현 위치**: `navigation/src/offboard/hover_handler.cpp`

#### 1.5 거리 조정 (Distance Adjustment) 🔵
- **ROS2 토픽**: `/fmu/in/trajectory_setpoint`
- **메시지 타입**: `px4_msgs::TrajectorySetpoint`
- **목적**: 타겟과의 거리를 10m로 유지
- **입력**: `/lidar/distance` (LiDAR 거리 데이터)
- **파라미터**:
  - LiDAR 거리 데이터 기반 위치 계산
  - 전방 방향으로 미세 조정
- **전송 주기**: 20Hz (50ms 간격, 빠른 반응 필요)
- **구현 위치**: `navigation/src/offboard/distance_adjustment_handler.cpp`
- **플로우차트**: "타겟과 거리조정+조준" (파란색, DRONE 단독)

#### 1.6 오토타겟팅 (Auto Targeting) 🔵
- **ROS2 토픽**: `/fmu/in/trajectory_setpoint`
- **메시지 타입**: `px4_msgs::TrajectorySetpoint`
- **목적**: 핫스팟을 화면 중심에 유지
- **입력**: `/thermal/hotspot` (핫스팟 위치)
- **파라미터**:
  - 핫스팟 위치 오차 기반 미세 조정
  - 상하좌우 위치 보정
- **전송 주기**: 20Hz (50ms 간격)
- **구현 위치**: `navigation/src/offboard/auto_targeting_handler.cpp`
- **플로우차트**: "타겟과 거리조정+조준", "격발 (조준 트래킹)" (파란색, DRONE 단독)

#### 1.7 복귀 (Return to Launch) 🔵
- **ROS2 토픽**: `/fmu/in/vehicle_command`
- **메시지 타입**: `px4_msgs::VehicleCommand`
- **명령 코드**: `MAV_CMD_NAV_RETURN_TO_LAUNCH`
- **전송 주기**: 일회성 (명령 시)
- **구현 위치**: `navigation/src/offboard/rtl_handler.cpp`
- **플로우차트**: "복귀" (파란색, DRONE 단독)

---

### 2. VIM4 → QGC (QGroundControl) 메시지 🟢

**목적**: 상태 정보 및 알림 전송 (플로우차트 초록색)

#### 2.1 목적지 도착 완료 알림
- **ROS2 토픽**: `/offboard/destination_reached`
- **메시지 타입**: `std_msgs::Bool`
- **값**: `true` (목적지 도착 시)
- **전송 주기**: 일회성 (도착 시)
- **구현 위치**: `navigation/src/offboard/waypoint_handler.cpp`
- **플로우차트**: "목적지 도착 완료 알림" (초록색)

#### 2.2 OFFBOARD 모드 상태 정보
- **ROS2 토픽**: `/offboard/status`
- **메시지 타입**: `std_msgs::String`
- **값**: "ARMING", "TAKEOFF", "NAVIGATING", "DESTINATION_REACHED", "FIRE_READY", "FIRING_AUTO_TARGETING", "AUTO_FIRING", "RETURNING", "LANDING" 등
- **전송 주기**: 상태 변경 시 즉시 + 주기적 (1Hz)
- **구현 위치**: `navigation/src/offboard/status_reporter.cpp`
- **구독 위치**: `ros2/src/status/status_ros2_subscriber.cpp` (StatusOverlay 업데이트), QGC

#### 2.3 격발 준비 완료 알림
- **ROS2 토픽**: `/offboard/fire_ready`
- **메시지 타입**: `std_msgs::Bool`
- **값**: `true` (격발 준비 완료 시)
- **전송 주기**: 일회성 (준비 완료 시)
- **구현 위치**: `navigation/src/offboard/auto_targeting_handler.cpp`
- **플로우차트**: "준비 알림" (초록색)

---

### 3. QGC → VIM4 메시지 (수신) 🔴

**목적**: 지상 관제소로부터 명령 수신 (플로우차트 빨간색)

#### 3.1 격발 명령
- **ROS2 토픽**: `/gcs/fire_command`
- **메시지 타입**: `std_msgs::Bool`
- **값**: `true` (격발 명령)
- **전송 주기**: 일회성 (격발 시)
- **구독 위치**: `navigation/src/offboard/command_receiver.cpp`
- **처리 위치**: `throwing_mechanism/src/fire_controller.cpp`
- **플로우차트**: "격발" (빨간색)

#### 3.2 비상 정지
- **ROS2 토픽**: `/gcs/emergency_stop`
- **메시지 타입**: `std_msgs::Bool`
- **값**: `true` (비상 정지 명령)
- **전송 주기**: 즉시 처리
- **구독 위치**: `navigation/src/offboard/emergency_handler.cpp`

#### 3.3 목표 좌표 변경
- **ROS2 토픽**: `/gcs/waypoint_update`
- **메시지 타입**: `geometry_msgs::PoseStamped`
- **필드**: `pose.position` (x, y, z), `pose.orientation`
- **구독 위치**: `navigation/src/offboard/waypoint_handler.cpp`

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
- **구현 위치**: `targeting/src/hotspot_tracker.cpp` (기존) + `navigation/src/offboard/auto_targeting_handler.cpp`

#### 4.4 거리 조정 로직
- **입력**: LiDAR 거리 데이터, 목표 거리 (10m)
- **처리**: 거리 차이 계산 및 위치 보정 명령 생성
- **출력**: FC로 위치 명령 전송
- **구현 위치**: `navigation/src/offboard/distance_adjustment_handler.cpp`

#### 4.5 OFFBOARD 모드 상태 머신
- **상태**: 
  - `IDLE`: 대기
  - `ARMING`: 시동 중
  - `TAKEOFF`: 이륙 중
  - `NAVIGATING`: 이동 중
  - `DESTINATION_REACHED`: 목적지 도착
  - `FIRE_READY`: 격발 대기 (수동 격발 대기)
  - `FIRING_AUTO_TARGETING`: 격발 중 (자동조준, 수동 격발 모드)
  - `AUTO_FIRING`: 자동조준격발 (자동 격발 모드)
  - `MISSION_COMPLETE`: 임무 완료
  - `RETURNING`: 복귀 중
  - `LANDING`: 착륙 중
  - `DISARMED`: 시동 끔
- **전이 조건**: 각 상태별 완료 조건
- **구현 위치**: `navigation/src/offboard/state_machine.cpp`

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

#### VIM4 → QGC (상태 알림) 🟢
```
/offboard/status              (std_msgs::msg::String)    # OFFBOARD 모드 상태
/offboard/destination_reached (std_msgs::msg::Bool)     # 목적지 도착 완료 알림
/offboard/fire_ready          (std_msgs::msg::Bool)     # 격발 준비 완료 알림
```

#### QGC → VIM4 (명령) 🔴
```
/gcs/fire_command            (std_msgs::msg::Bool)     # 격발 명령
/gcs/emergency_stop          (std_msgs::msg::Bool)     # 비상 정지 명령
/gcs/waypoint_update         (geometry_msgs::PoseStamped) # 목표 좌표 변경
```

#### VIM4 → PX4 (명령) 🟡
```
/fmu/in/vehicle_command      (px4_msgs::VehicleCommand) # 시동, 이륙, 복귀 명령
/fmu/in/trajectory_setpoint   (px4_msgs::TrajectorySetpoint) # 위치/속도 명령
/fmu/in/offboard_control_mode (px4_msgs::OffboardControlMode) # 오프보드 제어 모드
```

#### PX4 → VIM4 (상태 수신)
```
/fmu/out/vehicle_status_v1    (px4_msgs::VehicleStatus) # 비행 모드, 시동 상태
/fmu/out/battery_status       (px4_msgs::BatteryStatus) # 배터리 상태
/fmu/out/vehicle_gps_position (px4_msgs::SensorGps)    # GPS 정보
```

#### VIM4 내부 (DRONE 단독) 🔵
```
/lidar/distance              (sensor_msgs::msg::Range)  # LiDAR 거리
/thermal/hotspot             (custom_msgs::msg::Hotspot) # 핫스팟 위치
/ammunition/current          (std_msgs::msg::Int32)     # 소화탄 갯수
/formation/current           (std_msgs::msg::Int32)     # 편대 번호
```

**상세 내용**: `ROS2_TOPIC_ARCHITECTURE.md` 참조

---

## 파일 구조

```
navigation/
├── src/
│   └── offboard/
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
│       └── offboard_manager.cpp       # 통합 관리자
├── include/
│   └── offboard/
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
│       └── offboard_manager.h
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
- 전체 OFFBOARD 모드 시나리오 테스트
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
- 통합: VIM4 자동 제어 시스템과 독립적으로 동작하지만, OFFBOARD 모드일 때는 `/offboard/status` 토픽을 구독하여 상세 상태를 표시

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
| `OFFBOARD` | OFFBOARD 모드 상태 확인 | 오프보드 모드 (VIM4 자동 제어 시스템) |

### PX4 Mission 모드 동작 (AUTO_MISSION)

**참고**: ArduPilot의 AUTO 모드와 유사하지만, PX4에서는 `AUTO_MISSION` 모드입니다.

QGC에서 Mission을 업로드하고 실행하면:
1. PX4가 `AUTO_MISSION` 모드로 전환
2. `/fmu/out/vehicle_status` 의 `nav_state` 필드가 AUTO_MISSION(14)으로 변경
3. StatusOverlay가 이를 감지하여 "이동중" 상태로 표시
4. Waypoint 도착 시 "목적지도착" 상태로 변경 (MAVLink `MISSION_ITEM_REACHED` 메시지 활용)
5. 모든 Waypoint 완료 시 "임무완료" 상태로 변경

### VIM4 자동 제어 시스템 동작 (OFFBOARD)

VIM4에서 ROS2를 통해 드론을 제어할 때:
1. PX4가 `OFFBOARD` 모드로 전환
2. `/fmu/out/vehicle_status` 의 `nav_state` 필드가 OFFBOARD(14)로 변경
3. StatusOverlay가 PX4 모드를 감지하고, 추가로 `/offboard/status` 토픽을 구독
4. VIM4 자동 제어 시스템의 상태(시동, 이륙, 이동중, 격발대기 등)를 표시
5. PX4 Mission 모드와는 별개로 동작하는 커스텀 자동 제어 시스템

---

**작성자**: Claude Code Assistant  
**버전**: v1.3 (용어 정리: PX4 Mission 모드와 VIM4 자동 제어 시스템 구분)  
**작성일**: 2025-01-01  
**다음 리뷰**: Phase 1 완료 시

