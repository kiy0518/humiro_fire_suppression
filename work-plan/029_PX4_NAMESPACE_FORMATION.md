# PX4 네임스페이스 - 단독/편대 비행 자동 분기

## 배경

편대 비행 시 여러 VIM4가 동일 WiFi에 연결되면 FastDDS discovery로 `/fmu/*` 토픽이 교차 발견됨.
단독 비행에서는 문제없지만, 편대 비행에서는 네임스페이스로 드론별 토픽을 분리해야 함.

## 현재 상태

- `px4_ns = ""` 하드코딩 (offboard_manager.cpp:32, status_ros2_subscriber.cpp:54)
- 단독/편대 모드는 `offboard_config.json`의 `mission_mode`로 결정
- DRONE_ID는 환경변수로 읽음

## 변경 계획

### 토픽 분리 방식

```
단독 (solo):    /fmu/in/offboard_control_mode          (네임스페이스 없음)
편대 (formation): /drone3/fmu/in/offboard_control_mode  (DRONE_ID 기반)
```

### VIM4 코드 변경 (C++)

**1. application_manager.cpp** — px4_ns 계산 후 전달
- readMissionModeFromConfig() 결과 + DRONE_ID로 px4_ns 결정
- solo: `px4_ns = ""`
- formation: `px4_ns = "/drone" + DRONE_ID`
- OffboardManager, StatusROS2Subscriber 생성자에 px4_ns 전달

**2. offboard_manager.h/.cpp** — 생성자 시그니처 변경
- `OffboardManager(rclcpp::Node::SharedPtr node)` →
  `OffboardManager(rclcpp::Node::SharedPtr node, const std::string& px4_ns = "")`
- 로컬 변수 `px4_ns` → 생성자 파라미터 사용

**3. status_ros2_subscriber.h/.cpp** — 생성자 시그니처 변경
- `StatusROS2Subscriber(node, overlay)` →
  `StatusROS2Subscriber(node, overlay, const std::string& px4_ns = "")`
- 로컬 변수 `px4_ns` → 생성자 파라미터 사용

### PX4 (SITL) 측 변경

**편대 SITL 테스트 시 PX4 콘솔에서:**
```bash
# 드론 1
uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888 -n drone1

# 드론 3
uxrce_dds_client start -t udp -h 192.168.100.31 -p 8888 -n drone3
```

**단독 SITL 테스트 시:**
```bash
uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888
# -n 없음 (네임스페이스 없음)
```

### PX4 (실제 FC) 측 — 추후 작업
- PX4 v1.16.0 UXRCE_DDS 파라미터에 네임스페이스 설정
- 또는 rcS 스크립트에서 MAV_SYS_ID 기반 `-n droneN` 자동 적용

## 변경하지 않는 것

- 편대 토픽 (`/droneN/formation/*`) — 이미 네임스페이스 적용됨
- VIM4 커스텀 토픽 (`/offboard/status`, `/ammunition/current`) — PX4 무관
- GUI, CustomMessage, FormationController 코드
- micro-ros-agent 실행 방식 (변경 불필요, PX4 클라이언트가 네임스페이스 결정)
