# ROS2 토픽 아키텍처 (ROS2 Topic Architecture)

작성일: 2026-01-01  
**기반**: 화재 진압 드론 임무 플로우차트  
**상태**: 설계 완료

---

## 개요

화재 진압 드론 시스템의 ROS2 토픽 통신 구조를 정의합니다. 플로우차트의 색상 구분에 따라 통신 방향과 역할을 명확히 구분합니다.

---

## 통신 방향 구분 (플로우차트 색상 기준)

### 🟡 노란색: QGC → DRONE(FC)
QGC에서 Flight Controller로 직접 명령 전송 (uXRCE-DDS)

### 🟢 초록색: DRONE(VIM4) → QGC
VIM4에서 QGC로 상태 알림 및 보고

### 🔴 빨간색: QGC → DRONE(VIM4)
QGC에서 VIM4로 명령 전송

### 🔵 파란색: DRONE 단독
VIM4 내부 처리 (외부 통신 없음)

---

## 토픽 발행 구조

### 1. PX4 uXRCE-DDS 토픽 (PX4 Flight Controller)

#### 1.1 PX4 → VIM4 (읽기 전용)

| 토픽 | 메시지 타입 | 필드 | 용도 | 업데이트 주기 |
|------|------------|------|------|--------------|
| `/fmu/out/vehicle_status_v1` | `px4_msgs::VehicleStatus` | `nav_state`, `arming_state`, `failsafe` | 비행 모드, 시동 상태 | 10Hz |
| `/fmu/out/battery_status` | `px4_msgs::BatteryStatus` | `remaining`, `voltage_v`, `current_a` | 배터리 잔량, 전압, 전류 | 1Hz |
| `/fmu/out/vehicle_gps_position` | `px4_msgs::SensorGps` | `latitude_deg`, `longitude_deg`, `satellites_used`, `fix_type` | GPS 위치, 위성 수 | 1Hz |

**구독 위치**: `ros2/src/status/status_ros2_subscriber.cpp`

#### 1.2 VIM4 → PX4 (명령 전송)

| 토픽 | 메시지 타입 | 필드 | 용도 | 발행 위치 |
|------|------------|------|------|----------|
| `/fmu/in/vehicle_command` | `px4_msgs::VehicleCommand` | `command`, `param1-7` | 시동, 이륙, 착륙 명령 | `navigation/src/offboard/arm_handler.cpp`, `takeoff_handler.cpp` |
| `/fmu/in/offboard_control_mode` | `px4_msgs::OffboardControlMode` | `position`, `velocity`, `acceleration` | 오프보드 제어 모드 설정 | `navigation/src/offboard/waypoint_handler.cpp` |
| `/fmu/in/trajectory_setpoint` | `px4_msgs::TrajectorySetpoint` | `position`, `velocity`, `yaw` | 궤적 설정점 (이동, 거리조정, 조준) | `navigation/src/offboard/waypoint_handler.cpp`, `distance_adjustment_handler.cpp`, `auto_targeting_handler.cpp` |

**발행 위치**: `navigation/src/offboard/` (예정)

---

### 2. VIM4 커스텀 토픽

#### 2.1 VIM4 → QGC (상태 알림) 🟢

| 토픽 | 메시지 타입 | 값 예시 | 용도 | 발행 위치 |
|------|------------|---------|------|----------|
| `/offboard/status` | `std_msgs::String` | "ARMING", "TAKEOFF", "NAVIGATING", "DESTINATION_REACHED", "FIRE_READY", "FIRING_AUTO_TARGETING", "RETURNING", "LANDING" | OFFBOARD 모드 상태 | `navigation/src/offboard/status_reporter.cpp` |
| `/offboard/destination_reached` | `std_msgs::Bool` | `true` (도착 시) | 목적지 도착 완료 알림 | `navigation/src/offboard/waypoint_handler.cpp` |
| `/offboard/fire_ready` | `std_msgs::Bool` | `true` (준비 완료 시) | 격발 준비 완료 알림 | `navigation/src/offboard/auto_targeting_handler.cpp` |

**구독 위치**: QGC (향후 구현)

#### 2.2 VIM4 → StatusOverlay (상태 표시)

| 토픽 | 메시지 타입 | 용도 | 구독 위치 |
|------|------------|------|----------|
| `/offboard/status` | `std_msgs::String` | OFFBOARD 모드 상태 (녹색 표시) | `ros2/src/status/status_ros2_subscriber.cpp` |
| `/ammunition/current` | `std_msgs::Int32` | 현재 소화탄 갯수 (녹색 표시) | `ros2/src/status/status_ros2_subscriber.cpp` |
| `/formation/current` | `std_msgs::Int32` | 현재 편대 번호 (녹색 표시) | `ros2/src/status/status_ros2_subscriber.cpp` |

**구독 위치**: `ros2/src/status/status_ros2_subscriber.cpp` ✅ 완료

#### 2.3 VIM4 내부 토픽 (DRONE 단독) 🔵

| 토픽 | 메시지 타입 | 용도 | 발행/구독 위치 |
|------|------------|------|---------------|
| `/lidar/distance` | `sensor_msgs::Range` | LiDAR 거리 데이터 | `lidar/src/lidar_ros2_publisher.cpp` → `navigation/src/offboard/distance_adjustment_handler.cpp` |
| `/thermal/hotspot` | `custom_msgs::Hotspot` | 핫스팟 위치 | `thermal/src/thermal_ros2_publisher.cpp` → `navigation/src/offboard/auto_targeting_handler.cpp` |
| `/offboard/internal/state` | `std_msgs::String` | 내부 상태 머신 상태 | `navigation/src/offboard/state_machine.cpp` (내부) |

**참고**: 이 토픽들은 VIM4 내부에서만 사용되며, QGC나 외부로 전송되지 않음

---

### 3. QGC → VIM4 토픽 (명령 수신) 🔴

| 토픽 | 메시지 타입 | 값 | 용도 | 구독 위치 |
|------|------------|-----|------|----------|
| `/gcs/fire_command` | `std_msgs::Bool` | `true` (격발 명령) | 격발 명령 수신 | `navigation/src/offboard/command_receiver.cpp` |
| `/gcs/emergency_stop` | `std_msgs::Bool` | `true` (비상 정지) | 비상 정지 명령 | `navigation/src/offboard/emergency_handler.cpp` |
| `/gcs/waypoint_update` | `geometry_msgs::PoseStamped` | 목표 좌표 | 목표 좌표 변경 | `navigation/src/offboard/waypoint_handler.cpp` |

**구독 위치**: `navigation/src/offboard/command_receiver.cpp` (예정)

---

## 플로우차트 기반 토픽 매핑

### Phase 1: 초기 작업 (상단 행)

#### 1. 화재 포착 (목적지 입력) 🟡
- **QGC → FC**: `/fmu/in/trajectory_setpoint` (목표 좌표 설정)
- **발행 위치**: QGC 또는 VIM4 (`navigation/src/offboard/waypoint_handler.cpp`)

#### 2. 시동 🟡
- **QGC → FC**: `/fmu/in/vehicle_command` (`MAV_CMD_COMPONENT_ARM_DISARM`)
- **발행 위치**: QGC 또는 VIM4 (`navigation/src/offboard/arm_handler.cpp`)
- **상태 확인**: `/fmu/out/vehicle_status_v1` (`arming_state`)

#### 3. 이륙 🟡
- **QGC → FC**: `/fmu/in/vehicle_command` (`MAV_CMD_NAV_TAKEOFF`)
- **발행 위치**: QGC 또는 VIM4 (`navigation/src/offboard/takeoff_handler.cpp`)
- **상태 확인**: `/fmu/out/vehicle_status_v1` (`nav_state = 17`)

#### 4. 이동 (편대비행, 충돌방지) 🟡
- **QGC → FC**: `/fmu/in/trajectory_setpoint` (waypoint 이동)
- **발행 위치**: VIM4 (`navigation/src/offboard/waypoint_handler.cpp`)
- **상태 확인**: `/fmu/out/vehicle_status_v1` (`nav_state = 14` OFFBOARD)
- **VIM4 상태**: `/offboard/status` = "NAVIGATING"

#### 5. 목적지 도착 완료 알림 🟢
- **VIM4 → QGC**: `/offboard/destination_reached` (`true`)
- **VIM4 → StatusOverlay**: `/offboard/status` = "DESTINATION_REACHED"
- **발행 위치**: `navigation/src/offboard/waypoint_handler.cpp`

---

### Phase 2: 타겟 조준 및 격발 (하단 행)

#### 1. 타겟과 거리조정+조준 🔵
- **VIM4 내부**: `/lidar/distance` → 거리 조정
- **VIM4 내부**: `/thermal/hotspot` → 조준
- **VIM4 → FC**: `/fmu/in/trajectory_setpoint` (미세 위치 조정)
- **발행 위치**: 
  - `navigation/src/offboard/distance_adjustment_handler.cpp` (거리 조정)
  - `navigation/src/offboard/auto_targeting_handler.cpp` (조준)
- **상태**: `/offboard/status` = "NAVIGATING" (내부 처리 중)

#### 2. 준비 알림 🟢
- **VIM4 → QGC**: `/offboard/fire_ready` (`true`)
- **VIM4 → StatusOverlay**: `/offboard/status` = "FIRE_READY"
- **발행 위치**: `navigation/src/offboard/auto_targeting_handler.cpp`

#### 3. 격발 신호대기 🟡
- **대기 중**: `/gcs/fire_command` 토픽 수신 대기
- **구독 위치**: `navigation/src/offboard/command_receiver.cpp`

#### 4. 격발 🔴
- **QGC → VIM4**: `/gcs/fire_command` (`true`)
- **구독 위치**: `navigation/src/offboard/command_receiver.cpp`
- **처리**: `throwing_mechanism/src/fire_controller.cpp`

#### 5. 격발 (조준 트래킹) 🔵
- **VIM4 내부**: `/thermal/hotspot` → 핫스팟 추적
- **VIM4 → FC**: `/fmu/in/trajectory_setpoint` (조준 유지)
- **VIM4 → StatusOverlay**: `/offboard/status` = "FIRING_AUTO_TARGETING"
- **발행 위치**: `navigation/src/offboard/auto_targeting_handler.cpp`

#### 6. 복귀 🔵
- **VIM4 → FC**: `/fmu/in/vehicle_command` (`MAV_CMD_NAV_RETURN_TO_LAUNCH`)
- **VIM4 → StatusOverlay**: `/offboard/status` = "RETURNING"
- **발행 위치**: `navigation/src/offboard/rtl_handler.cpp`

---

## 토픽 발행/구독 매트릭스

### VIM4에서 발행하는 토픽

| 토픽 | 발행 위치 | 구독 위치 | 용도 |
|------|----------|----------|------|
| `/offboard/status` | `navigation/src/offboard/status_reporter.cpp` | `ros2/src/status/status_ros2_subscriber.cpp`, QGC | OFFBOARD 모드 상태 |
| `/offboard/destination_reached` | `navigation/src/offboard/waypoint_handler.cpp` | QGC | 목적지 도착 알림 |
| `/offboard/fire_ready` | `navigation/src/offboard/auto_targeting_handler.cpp` | QGC | 격발 준비 알림 |
| `/ammunition/current` | `throwing_mechanism/src/fire_controller.cpp` | `ros2/src/status/status_ros2_subscriber.cpp` | 소화탄 갯수 |
| `/formation/current` | `navigation/src/formation/formation_controller.cpp` | `ros2/src/status/status_ros2_subscriber.cpp` | 편대 번호 |
| `/fmu/in/vehicle_command` | `navigation/src/offboard/arm_handler.cpp`, `takeoff_handler.cpp`, `rtl_handler.cpp` | PX4 FC | 시동, 이륙, 복귀 명령 |
| `/fmu/in/trajectory_setpoint` | `navigation/src/offboard/waypoint_handler.cpp`, `distance_adjustment_handler.cpp`, `auto_targeting_handler.cpp` | PX4 FC | 위치/속도 명령 |

### VIM4에서 구독하는 토픽

| 토픽 | 발행 위치 | 구독 위치 | 용도 |
|------|----------|----------|------|
| `/fmu/out/vehicle_status_v1` | PX4 FC (uXRCE-DDS) | `ros2/src/status/status_ros2_subscriber.cpp` | 비행 모드, 시동 상태 |
| `/fmu/out/battery_status` | PX4 FC (uXRCE-DDS) | `ros2/src/status/status_ros2_subscriber.cpp` | 배터리 상태 |
| `/fmu/out/vehicle_gps_position` | PX4 FC (uXRCE-DDS) | `ros2/src/status/status_ros2_subscriber.cpp` | GPS 정보 |
| `/gcs/fire_command` | QGC | `navigation/src/offboard/command_receiver.cpp` | 격발 명령 |
| `/gcs/emergency_stop` | QGC | `navigation/src/offboard/emergency_handler.cpp` | 비상 정지 |
| `/lidar/distance` | `lidar/src/lidar_ros2_publisher.cpp` | `navigation/src/offboard/distance_adjustment_handler.cpp` | LiDAR 거리 |
| `/thermal/hotspot` | `thermal/src/thermal_ros2_publisher.cpp` | `navigation/src/offboard/auto_targeting_handler.cpp` | 핫스팟 위치 |

---

## 상태 전환 및 토픽 발행 시점

### OFFBOARD 모드 상태 머신

```
IDLE
  ↓ (시동 명령)
ARMING → /offboard/status = "ARMING"
  ↓ (이륙 명령)
TAKEOFF → /offboard/status = "TAKEOFF"
  ↓ (이륙 완료)
NAVIGATING → /offboard/status = "NAVIGATING"
  ↓ (목적지 도착)
DESTINATION_REACHED → /offboard/status = "DESTINATION_REACHED"
                      /offboard/destination_reached = true
  ↓ (거리 조정 + 조준 완료)
FIRE_READY → /offboard/status = "FIRE_READY"
             /offboard/fire_ready = true
  ↓ (수동 격발: /gcs/fire_command 수신)
FIRING_AUTO_TARGETING → /offboard/status = "FIRING_AUTO_TARGETING"
  ↓ (자동 격발: 자동으로 조준하고 격발)
AUTO_FIRING → /offboard/status = "AUTO_FIRING"
  ↓ (격발 완료)
RETURNING → /offboard/status = "RETURNING"
  ↓ (복귀 완료)
LANDING → /offboard/status = "LANDING"
  ↓ (착륙 완료)
DISARMED → /offboard/status = "DISARMED"
```

---

## 구현 우선순위

### Phase 1: 기본 통신 (완료 ✅)
- [x] PX4 상태 수신 (`/fmu/out/vehicle_status_v1`)
- [x] 배터리/GPS 수신 (`/fmu/out/battery_status`, `/fmu/out/vehicle_gps_position`)
- [x] OFFBOARD 상태 구독 (`/offboard/status`)
- [x] 소화탄/편대 정보 구독 (`/ammunition/current`, `/formation/current`)

### Phase 2: VIM4 상태 발행 (다음 작업)
- [ ] `/offboard/status` 발행 (`navigation/src/offboard/status_reporter.cpp`)
- [ ] `/offboard/destination_reached` 발행 (`navigation/src/offboard/waypoint_handler.cpp`)
- [ ] `/offboard/fire_ready` 발행 (`navigation/src/offboard/auto_targeting_handler.cpp`)

### Phase 3: PX4 명령 발행 (다음 작업)
- [ ] `/fmu/in/vehicle_command` 발행 (시동, 이륙, 복귀)
- [ ] `/fmu/in/trajectory_setpoint` 발행 (이동, 거리 조정, 조준)

### Phase 4: QGC 명령 수신 (향후)
- [ ] `/gcs/fire_command` 구독
- [ ] `/gcs/emergency_stop` 구독
- [ ] `/gcs/waypoint_update` 구독

---

## 참고사항

### QoS 설정
- **PX4 uXRCE-DDS 토픽**: `BestEffort`, `Volatile`, Depth=10
- **VIM4 커스텀 토픽**: `Reliable`, `Volatile`, Depth=10
- **QGC 토픽**: `Reliable`, `Volatile`, Depth=10

### 메시지 타입
- **표준 메시지**: `std_msgs`, `geometry_msgs`, `sensor_msgs`
- **PX4 메시지**: `px4_msgs` (uXRCE-DDS)
- **커스텀 메시지**: 필요 시 `custom_msgs` 패키지 생성

---

**작성자**: Claude Code Assistant  
**버전**: v1.0  
**작성일**: 2026-01-01  
**기반**: 화재 진압 드론 임무 플로우차트

