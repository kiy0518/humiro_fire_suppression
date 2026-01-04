# 화재 진압 임무 시나리오 - 메시지 흐름 상세

**작성일**: 2026-01-03
**버전**: v1.0
**기반**: VIM4 자동 제어 시스템 (OFFBOARD 모드)

---

## 개요

화재 발생부터 진압 완료까지의 전체 시나리오를 **GCS (QGC)**, **VIM4 (Companion Computer)**, **FC (PX4 Flight Controller)** 간의 메시지 흐름을 중심으로 상세하게 설명합니다.

**시스템 구성**:
- **GCS (QGC)**: 지상 관제소 (화재 지점 입력, 격발 명령, 상태 모니터링)
- **VIM4**: 온보드 컴퓨터 (자동 제어, LiDAR/Thermal 처리, ROS2)
- **FC (PX4)**: 비행 제어기 (OFFBOARD 모드, 자세/위치 제어)

**통신 프로토콜**:
- **GCS ↔ VIM4**: ROS2 토픽 (또는 MAVLink 커스텀 메시지)
- **VIM4 ↔ FC**: uXRCE-DDS (PX4 ↔ ROS2)
- **GCS ↔ FC**: MAVLink (상태 모니터링, RC 제어)

---

## 시나리오 단계별 메시지 흐름

### **Phase 0: 초기 상태 (대기)**

#### 시스템 상태
- **FC**: 시동 꺼짐 (DISARMED), 수동 모드 (MANUAL)
- **VIM4**: ROS2 노드 실행 중, OFFBOARD 상태 머신 = `IDLE`
- **GCS**: QGC 연결됨, 드론 상태 모니터링 중

#### 메시지 흐름

**FC → VIM4** (지속적, 10Hz):
```yaml
Topic: /fmu/out/vehicle_status_v1
Type: px4_msgs::VehicleStatus
Data:
  nav_state: 2              # MANUAL
  arming_state: 1           # DISARMED
  failsafe: false
  timestamp: 1234567890
```

**FC → VIM4** (지속적, 1Hz):
```yaml
Topic: /fmu/out/battery_status
Type: px4_msgs::BatteryStatus
Data:
  remaining: 1.0            # 100% (0.0-1.0)
  voltage_v: 16.8
  current_a: 0.5
  timestamp: 1234567890
```

**VIM4 → GCS** (지속적, 1Hz):
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "IDLE"
```

**VIM4 → GCS (MAVLink)** (지속적, 1Hz):
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  drone_id: 1               // 드론 ID
  lat: 375665000            // 37.5665° * 1e7
  lon: 1269780000           // 126.9780° * 1e7
  alt: 100000               // 100m MSL (mm)
  battery_percent: 100      // 100%
  ammo_count: 6             // 소화탄 6개
  mission_state: 0          // IDLE
  target_id: 0              // 할당된 목표 없음
  timestamp: 1234567890     // μs
```

---

### **Phase 1: 화재 감지 및 임무 할당**

#### 시나리오
1. GCS 운영자가 QGC 지도에서 화재 지점 클릭
2. 화재 좌표 입력: `37.5672°N, 126.9788°E`
3. 우선순위 설정: HIGH (0.9)
4. Drone #1에 목표 할당

#### 메시지 흐름

**GCS → VIM4 (ROS2)** (일회성):
```yaml
Topic: /gcs/waypoint_update
Type: geometry_msgs::PoseStamped
Data:
  header:
    stamp: now
    frame_id: "map"
  pose:
    position:
      x: 126.9788              # Longitude (경도)
      y: 37.5672               # Latitude (위도)
      z: 50.0                  # 목표 고도 (m)
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0                   # Yaw 0° (북쪽)
```

**또는 GCS → VIM4 (MAVLink)** (일회성):
```c
Message: TARGET_ASSIGNMENT (ID: 12921)
Fields:
  drone_id: 1               // 드론 1에 할당
  target_id: 1              // 화재 지점 ID
  lat: 375672000            // 37.5672° * 1e7
  lon: 1269788000           // 126.9788° * 1e7
  alt: 50000                // 50m AGL (mm)
  priority: 0.9             // HIGH
  timestamp: 1234567890
```

**VIM4 내부 처리**:
- Waypoint 저장: `target_lat = 37.5672, target_lon = 126.9788, target_alt = 50.0`
- 상태 전환: `IDLE` → `READY_TO_ARM`

**VIM4 → GCS** (상태 업데이트):
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "READY_TO_ARM"
```

**VIM4 → GCS (MAVLink)** (상태 업데이트):
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 0          // IDLE (아직 시동 안 걸림)
  target_id: 1              // 목표 할당됨
  ...
```

---

### **Phase 2: 시동 (Arming)**

#### 시나리오
1. GCS에서 시동 명령 전송 (또는 VIM4 자동 시동)
2. FC가 Pre-arm 체크 수행
3. 시동 성공

#### 메시지 흐름

**VIM4 → FC** (일회성):
```yaml
Topic: /fmu/in/vehicle_command
Type: px4_msgs::VehicleCommand
Data:
  command: 400              # MAV_CMD_COMPONENT_ARM_DISARM
  param1: 1.0               # 1 = ARM, 0 = DISARM
  param2: 0.0               # Force (0 = normal)
  target_system: 1
  target_component: 1
  source_system: 1
  source_component: 191     # MAV_COMP_ID_ONBOARD_COMPUTER
  timestamp: 1234567890
```

**FC → VIM4** (응답):
```yaml
Topic: /fmu/out/vehicle_command_ack
Type: px4_msgs::VehicleCommandAck
Data:
  command: 400              # MAV_CMD_COMPONENT_ARM_DISARM
  result: 0                 # MAV_RESULT_ACCEPTED
  timestamp: 1234567891
```

**FC → VIM4** (상태 업데이트):
```yaml
Topic: /fmu/out/vehicle_status_v1
Type: px4_msgs::VehicleStatus
Data:
  nav_state: 2              # MANUAL (아직 OFFBOARD 아님)
  arming_state: 2           # ARMED
  timestamp: 1234567891
```

**VIM4 내부 처리**:
- 시동 확인 완료
- 상태 전환: `READY_TO_ARM` → `ARMING` → `ARMED`

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "ARMED"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 1          // ARMING
  ...
```

---

### **Phase 3: 이륙 (Takeoff)**

#### 시나리오
1. VIM4가 이륙 명령 전송
2. FC가 자동 이륙 (목표 고도 10m)
3. 이륙 완료 후 OFFBOARD 모드로 전환

#### 메시지 흐름

**VIM4 → FC** (이륙 명령):
```yaml
Topic: /fmu/in/vehicle_command
Type: px4_msgs::VehicleCommand
Data:
  command: 22               # MAV_CMD_NAV_TAKEOFF
  param7: 10.0              # 목표 고도 (m AGL)
  target_system: 1
  target_component: 1
  timestamp: 1234567900
```

**FC → VIM4** (응답):
```yaml
Topic: /fmu/out/vehicle_command_ack
Type: px4_msgs::VehicleCommandAck
Data:
  command: 22               # MAV_CMD_NAV_TAKEOFF
  result: 0                 # MAV_RESULT_ACCEPTED
  timestamp: 1234567901
```

**FC → VIM4** (이륙 중 상태):
```yaml
Topic: /fmu/out/vehicle_status_v1
Type: px4_msgs::VehicleStatus
Data:
  nav_state: 17             # AUTO_TAKEOFF
  arming_state: 2           # ARMED
  timestamp: 1234567905
```

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "TAKEOFF"
```

**이륙 완료 후 (고도 10m 도달)**:

**VIM4 → FC** (OFFBOARD 모드 활성화, 주기적 2Hz):
```yaml
Topic: /fmu/in/offboard_control_mode
Type: px4_msgs::OffboardControlMode
Data:
  position: true            # 위치 제어 활성화
  velocity: false
  acceleration: false
  attitude: false
  body_rate: false
  timestamp: 1234567920
```

**VIM4 → FC** (현재 위치 유지):
```yaml
Topic: /fmu/in/trajectory_setpoint
Type: px4_msgs::TrajectorySetpoint
Data:
  position: [0.0, 0.0, -10.0]  # NED: 현재 위치, 고도 10m
  velocity: [NaN, NaN, NaN]    # 속도 제어 안 함
  yaw: 0.0                     # 북쪽
  timestamp: 1234567920
```

**FC → VIM4** (OFFBOARD 모드 전환됨):
```yaml
Topic: /fmu/out/vehicle_status_v1
Type: px4_msgs::VehicleStatus
Data:
  nav_state: 14             # OFFBOARD
  arming_state: 2           # ARMED
  timestamp: 1234567921
```

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "OFFBOARD_ACTIVE"
```

---

### **Phase 4: 목표 지점 이동 (Navigation)**

#### 시나리오
1. VIM4가 목표 좌표로 이동 명령 전송
2. FC가 OFFBOARD 모드로 waypoint 추종
3. 주기적으로 위치 명령 업데이트 (2Hz)

#### 메시지 흐름

**VIM4 → FC** (위치 명령, 지속적 2Hz):
```yaml
Topic: /fmu/in/trajectory_setpoint
Type: px4_msgs::TrajectorySetpoint
Data:
  # NED 좌표계로 변환된 목표 위치
  position: [50.0, 100.0, -50.0]  # North 50m, East 100m, Down -50m (고도 50m)
  velocity: [5.0, 0.0, 0.0]       # 북쪽으로 5m/s 이동
  yaw: 0.785                       # 45° (목표 방향)
  timestamp: 1234568000
```

**FC → VIM4** (현재 위치, 지속적 10Hz):
```yaml
Topic: /fmu/out/vehicle_local_position
Type: px4_msgs::VehicleLocalPosition
Data:
  x: 25.0                   # North (m)
  y: 50.0                   # East (m)
  z: -50.0                  # Down (m), 고도 50m
  vx: 5.0                   # 속도 (m/s)
  vy: 0.0
  vz: 0.0
  heading: 0.785            # Yaw (rad)
  timestamp: 1234568001
```

**VIM4 → GCS** (상태 업데이트):
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "NAVIGATING"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  lat: 375668000            // 현재 위치 업데이트
  lon: 1269785000
  alt: 50000                // 50m
  mission_state: 3          // NAVIGATING
  target_id: 1
  ...
```

---

### **Phase 5: 목표 도착 및 상태 알림**

#### 시나리오
1. VIM4가 목표 지점 도착 감지 (거리 < 5m)
2. 현재 위치 유지 (Hovering)
3. GCS에 도착 알림 전송

#### 메시지 흐름

**VIM4 내부 처리**:
```python
# 거리 계산
distance = haversine(current_lat, current_lon, target_lat, target_lon)
if distance < 5.0:  # 5m 이내
    state = "DESTINATION_REACHED"
```

**VIM4 → FC** (위치 유지):
```yaml
Topic: /fmu/in/trajectory_setpoint
Type: px4_msgs::TrajectorySetpoint
Data:
  position: [50.0, 100.0, -50.0]  # 현재 위치 유지
  velocity: [0.0, 0.0, 0.0]       # 정지
  yaw: 0.785                      # 현재 방향 유지
  timestamp: 1234568100
```

**VIM4 → GCS** (도착 알림):
```yaml
Topic: /offboard/destination_reached
Type: std_msgs::Bool
Data: true
```

**VIM4 → GCS** (상태 업데이트):
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "DESTINATION_REACHED"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 4          // DESTINATION_REACHED
  ...
```

---

### **Phase 6: 거리 조정 (LiDAR 기반)**

#### 시나리오
1. VIM4가 LiDAR로 전방 거리 측정
2. 목표 거리 10m 유지하도록 위치 조정
3. 거리 조정 완료 (오차 < 0.5m)

#### 메시지 흐름

**LiDAR → VIM4** (지속적, 10Hz):
```yaml
Topic: /lidar/distance
Type: sensor_msgs::Range
Data:
  radiation_type: 1         # INFRARED
  field_of_view: 0.0349     # 2° (rad)
  min_range: 0.2            # 최소 측정 거리 (m)
  max_range: 40.0           # 최대 측정 거리 (m)
  range: 8.5                # 현재 거리 8.5m
  header:
    stamp: now
    frame_id: "lidar_link"
```

**VIM4 내부 처리**:
```python
# 거리 조정 계산
target_distance = 10.0          # 목표 거리 10m
current_distance = 8.5          # LiDAR 측정값
error = target_distance - current_distance  # 1.5m (더 가까워야 함)

# 위치 보정 (NED 좌표)
correction_distance = error * cos(heading)  # 헤딩 방향으로 보정
new_position = current_position + [correction_distance, 0, 0]
```

**VIM4 → FC** (위치 보정):
```yaml
Topic: /fmu/in/trajectory_setpoint
Type: px4_msgs::TrajectorySetpoint
Data:
  position: [51.5, 100.0, -50.0]  # North +1.5m 이동
  velocity: [0.5, 0.0, 0.0]       # 천천히 이동
  yaw: 0.785
  timestamp: 1234568200
```

**거리 조정 완료 (range = 10.0m ± 0.5m)**:

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "DISTANCE_ADJUSTED"
```

---

### **Phase 7: 자동 조준 (Thermal 카메라 기반)**

#### 시나리오
1. Thermal 카메라로 핫스팟 감지
2. 핫스팟 중심으로 기체 위치/요 조정
3. 조준 완료 (오차 < 10 pixels)

#### 메시지 흐름

**Thermal 카메라 → VIM4** (지속적, 5Hz):
```yaml
Topic: /thermal/hotspot
Type: custom_msgs::Hotspot
Data:
  x: 320                    # 이미지 중심 기준 X (pixels)
  y: 240                    # 이미지 중심 기준 Y (pixels)
  width: 50                 # 핫스팟 너비
  height: 60                # 핫스팟 높이
  temperature: 450.0        # 온도 (°C)
  confidence: 0.95          # 신뢰도
  image_width: 640          # 이미지 해상도
  image_height: 480
  header:
    stamp: now
```

**VIM4 내부 처리**:
```python
# 핫스팟 중심 계산
center_x = 640 / 2 = 320
center_y = 480 / 2 = 240
error_x = hotspot_x - center_x  # 0 (중심에 있음)
error_y = hotspot_y - center_y  # 0

# 픽셀 오차를 각도로 변환 (FOV = 60°)
pixel_to_angle = 60.0 / 640  # 0.09375 °/pixel
angle_error_x = error_x * pixel_to_angle
angle_error_y = error_y * pixel_to_angle

# 위치/요 보정
if abs(error_x) > 10:  # 10 pixels 이상 오차
    yaw_correction = angle_error_x * (pi / 180)
    new_yaw = current_yaw + yaw_correction
```

**VIM4 → FC** (조준 보정):
```yaml
Topic: /fmu/in/trajectory_setpoint
Type: px4_msgs::TrajectorySetpoint
Data:
  position: [51.5, 100.0, -50.0]  # 위치 유지
  velocity: [0.0, 0.0, 0.0]
  yaw: 0.800                       # Yaw 미세 조정
  timestamp: 1234568300
```

**조준 완료 (error < 10 pixels)**:

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "AUTO_TARGETING_COMPLETE"
```

---

### **Phase 8: 격발 준비 완료 알림**

#### 시나리오
1. 거리 조정 완료 + 자동 조준 완료
2. 소화탄 잔량 확인 (ammo > 0)
3. GCS에 격발 준비 완료 알림

#### 메시지 흐름

**VIM4 내부 처리**:
```python
# 격발 조건 확인
distance_ok = abs(lidar_distance - 10.0) < 0.5  # 거리 OK
targeting_ok = abs(hotspot_error) < 10          # 조준 OK
ammo_ok = ammunition_count > 0                  # 소화탄 있음

if distance_ok and targeting_ok and ammo_ok:
    fire_ready = True
```

**VIM4 → GCS** (격발 준비 알림):
```yaml
Topic: /offboard/fire_ready
Type: std_msgs::Bool
Data: true
```

**VIM4 → GCS** (상태 업데이트):
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "FIRE_READY"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 5          // FIRE_READY
  ammo_count: 6             // 소화탄 6개
  ...
```

---

### **Phase 9: 격발 명령 (GCS → VIM4)**

#### 시나리오
1. GCS 운영자가 격발 버튼 클릭
2. VIM4가 격발 명령 수신
3. 격발 시작 (자동 조준 트래킹 유지)

#### 메시지 흐름

**GCS → VIM4** (격발 명령):
```yaml
Topic: /gcs/fire_command
Type: std_msgs::Bool
Data: true
```

**또는 GCS → VIM4 (MAVLink)**:
```c
Message: FIRE_COMMAND (ID: 12922)
Fields:
  drone_id: 1               // 드론 1에 명령
  fire_enable: 1            // 1 = FIRE, 0 = STOP
  timestamp: 1234568400
```

**VIM4 내부 처리**:
```python
# 격발 명령 수신
fire_command_received = True
state = "FIRING_AUTO_TARGETING"

# 격발 메커니즘 활성화
throwing_mechanism.fire()
```

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "FIRING_AUTO_TARGETING"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 6          // FIRING
  ammo_count: 5             // 소화탄 1개 발사 (6 → 5)
  ...
```

---

### **Phase 10: 격발 실행 (조준 트래킹 유지)**

#### 시나리오
1. 격발 중 핫스팟 계속 추적
2. 실시간 조준 보정 (Yaw/위치)
3. 격발 완료 (3초 소요)

#### 메시지 흐름

**격발 중 (0-3초)**:

**Thermal 카메라 → VIM4** (지속적):
```yaml
Topic: /thermal/hotspot
Type: custom_msgs::Hotspot
Data:
  x: 325                    # 핫스팟 약간 이동
  y: 240
  ...
```

**VIM4 → FC** (조준 트래킹):
```yaml
Topic: /fmu/in/trajectory_setpoint
Type: px4_msgs::TrajectorySetpoint
Data:
  position: [51.5, 100.0, -50.0]
  velocity: [0.0, 0.0, 0.0]
  yaw: 0.805                # 핫스팟 추적하여 Yaw 미세 조정
  timestamp: 1234568450
```

**VIM4 내부** (격발 메커니즘):
```python
# 소화탄 투척
time.sleep(3.0)  # 격발 시간 3초
ammunition_count -= 1  # 소화탄 1개 소모
```

**격발 완료**:

**VIM4 → GCS**:
```yaml
Topic: /ammunition/current
Type: std_msgs::Int32
Data: 5                     # 소화탄 5개 남음
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 6          // FIRING (완료)
  ammo_count: 5
  ...
```

---

### **Phase 11: 임무 완료 및 복귀**

#### 시나리오
1. 격발 완료 확인
2. RTL (Return to Launch) 명령 전송
3. FC가 자동 복귀 시작

#### 메시지 흐름

**VIM4 → FC** (복귀 명령):
```yaml
Topic: /fmu/in/vehicle_command
Type: px4_msgs::VehicleCommand
Data:
  command: 20               # MAV_CMD_NAV_RETURN_TO_LAUNCH
  target_system: 1
  target_component: 1
  timestamp: 1234568500
```

**FC → VIM4** (응답):
```yaml
Topic: /fmu/out/vehicle_command_ack
Type: px4_msgs::VehicleCommandAck
Data:
  command: 20
  result: 0                 # MAV_RESULT_ACCEPTED
  timestamp: 1234568501
```

**FC → VIM4** (복귀 모드 전환):
```yaml
Topic: /fmu/out/vehicle_status_v1
Type: px4_msgs::VehicleStatus
Data:
  nav_state: 5              # AUTO_RTL
  arming_state: 2           # ARMED
  timestamp: 1234568502
```

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "RETURNING"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 7          // RETURNING
  target_id: 0              // 목표 완료
  ...
```

**복귀 중 (FC가 자동 제어)**:

**FC → VIM4** (위치 업데이트):
```yaml
Topic: /fmu/out/vehicle_local_position
Type: px4_msgs::VehicleLocalPosition
Data:
  x: 25.0                   # 홈으로 복귀 중
  y: 50.0
  z: -50.0
  vx: -5.0                  # 홈 방향으로 이동
  ...
```

---

### **Phase 12: 착륙 및 시동 해제**

#### 시나리오
1. 홈 위치 도착
2. FC가 자동 착륙 시작
3. 착륙 완료 후 자동 시동 해제

#### 메시지 흐름

**FC → VIM4** (착륙 시작):
```yaml
Topic: /fmu/out/vehicle_status_v1
Type: px4_msgs::VehicleStatus
Data:
  nav_state: 6              # AUTO_LAND
  arming_state: 2           # ARMED
  timestamp: 1234568600
```

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "LANDING"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 8          // LANDING
  ...
```

**착륙 완료 (지면 감지)**:

**FC → VIM4**:
```yaml
Topic: /fmu/out/vehicle_land_detected
Type: px4_msgs::VehicleLandDetected
Data:
  landed: true
  ground_contact: true
  timestamp: 1234568700
```

**FC → VIM4** (자동 시동 해제):
```yaml
Topic: /fmu/out/vehicle_status_v1
Type: px4_msgs::VehicleStatus
Data:
  nav_state: 2              # MANUAL (착륙 후)
  arming_state: 1           # DISARMED
  timestamp: 1234568701
```

**VIM4 → GCS**:
```yaml
Topic: /offboard/status
Type: std_msgs::String
Data: "MISSION_COMPLETE"
```

**VIM4 → GCS (MAVLink)**:
```c
Message: FORMATION_MEMBER_STATUS (ID: 12920)
Fields:
  mission_state: 9          // COMPLETED
  ammo_count: 5             // 소화탄 5개 남음
  ...
```

**VIM4 → GCS (임무 진행 상황)**:
```c
Message: MISSION_PROGRESS (ID: 12923)
Fields:
  target_id: 1              // 화재 지점 ID
  drone_id: 1               // 드론 1
  progress_status: 2        // COMPLETED
  timestamp: 1234568702
```

---

## 메시지 흐름 요약

### GCS → VIM4 (명령)
| 단계 | 토픽/메시지 | 데이터 | 용도 |
|------|------------|--------|------|
| 1. 임무 할당 | `/gcs/waypoint_update` 또는 `TARGET_ASSIGNMENT` | 화재 좌표, 우선순위 | 목표 할당 |
| 9. 격발 명령 | `/gcs/fire_command` 또는 `FIRE_COMMAND` | `fire_enable = 1` | 격발 시작 |

### VIM4 → GCS (상태 알림)
| 단계 | 토픽/메시지 | 데이터 | 용도 |
|------|------------|--------|------|
| 전체 | `/offboard/status` | "IDLE", "ARMING", "NAVIGATING", ... | 상태 업데이트 |
| 5. 도착 알림 | `/offboard/destination_reached` | `true` | 목표 도착 |
| 8. 격발 준비 | `/offboard/fire_ready` | `true` | 격발 가능 |
| 전체 | `FORMATION_MEMBER_STATUS` | 위치, 배터리, 소화탄, 상태 | 드론 상태 |

### VIM4 → FC (제어 명령)
| 단계 | 토픽 | 데이터 | 용도 |
|------|------|--------|------|
| 2. 시동 | `/fmu/in/vehicle_command` | `MAV_CMD_COMPONENT_ARM_DISARM` | 시동 |
| 3. 이륙 | `/fmu/in/vehicle_command` | `MAV_CMD_NAV_TAKEOFF` | 이륙 |
| 3-10. OFFBOARD | `/fmu/in/offboard_control_mode` | `position = true` | OFFBOARD 활성화 |
| 4-10. 이동/조준 | `/fmu/in/trajectory_setpoint` | 위치, 속도, Yaw | 위치 제어 |
| 11. 복귀 | `/fmu/in/vehicle_command` | `MAV_CMD_NAV_RETURN_TO_LAUNCH` | RTL |

### FC → VIM4 (상태 피드백)
| 토픽 | 주기 | 데이터 | 용도 |
|------|------|--------|------|
| `/fmu/out/vehicle_status_v1` | 10Hz | nav_state, arming_state | 비행 모드 |
| `/fmu/out/battery_status` | 1Hz | remaining, voltage, current | 배터리 |
| `/fmu/out/vehicle_local_position` | 10Hz | x, y, z, vx, vy, vz | 현재 위치 |
| `/fmu/out/vehicle_command_ack` | 일회성 | command, result | 명령 응답 |

### VIM4 내부 (센서 → 제어)
| 토픽 | 주기 | 데이터 | 용도 |
|------|------|--------|------|
| `/lidar/distance` | 10Hz | range (거리) | 거리 조정 |
| `/thermal/hotspot` | 5Hz | x, y, temperature | 자동 조준 |

---

## 타이밍 다이어그램

```
시간(s)  GCS                 VIM4                FC
--------------------------------------------------------------
0        [화재 감지]
         화재 좌표 입력
         (37.5672, 126.9788)

1        TARGET_ASSIGNMENT → [목표 저장]
                             상태: READY_TO_ARM

2                            ARM 명령 →         [Pre-arm 체크]
                                                [시동 ON]
                             ← ACK
                             상태: ARMED

3                            TAKEOFF 명령 →     [이륙 시작]
                                                nav_state: AUTO_TAKEOFF
                             ← ACK
                             상태: TAKEOFF

5                            [고도 10m 도달]
                             OFFBOARD 활성화 →  nav_state: OFFBOARD
                             상태: OFFBOARD_ACTIVE

6                            [목표 좌표 계산]
                             위치 명령(2Hz) →   [waypoint 추종]
                             상태: NAVIGATING

20       ← MEMBER_STATUS     ← 위치 피드백
         (NAVIGATING)

50                           [목표 도착 감지]
                             상태: DEST_REACHED
         ← destination_reached

51                           [LiDAR 측정]
                             8.5m → 목표 10m
                             위치 보정 →        [위치 조정]

55                           [거리 OK: 10.0m]
                             [Thermal 감지]
                             핫스팟 추적 →      [Yaw 조정]

60                           [조준 완료]
                             상태: FIRE_READY
         ← fire_ready

61       [운영자 확인]
         [격발 버튼 클릭]
         FIRE_COMMAND →      [격발 시작]
                             상태: FIRING

64                           [격발 완료 3초]
                             소화탄: 6 → 5
         ← MEMBER_STATUS
         (FIRING, ammo=5)

65                           RTL 명령 →         nav_state: AUTO_RTL
                             상태: RETURNING

80                                              [홈 도착]
                                                nav_state: AUTO_LAND
                             상태: LANDING

85                                              [착륙 완료]
                                                DISARMED
                             상태: MISSION_COMPLETE
         ← MISSION_PROGRESS
         (COMPLETED)
```

---

## 메시지 정의 참고

### ROS2 토픽
- **발행 위치**: `navigation/src/offboard/*.cpp`
- **구독 위치**: `ros2/src/status/status_ros2_subscriber.cpp`
- **상세**: `work-plan/ROS2_TOPIC_ARCHITECTURE.md`

### MAVLink 커스텀 메시지
- **Message ID 범위**: 12920-12923
- **상세**: `work-plan/QGC_DEVELOPMENT_GUIDE.md`

### PX4 uXRCE-DDS 메시지
- **메시지 타입**: `px4_msgs`
- **상세**: PX4 공식 문서

---

**작성자**: Claude Code Assistant
**버전**: v1.0
**작성일**: 2026-01-03
**기반**: ROS2_TOPIC_ARCHITECTURE.md, QGC_DEVELOPMENT_GUIDE.md
