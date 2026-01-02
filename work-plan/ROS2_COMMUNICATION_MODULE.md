# ROS2 통신 모듈 개발 계획 (ROS2 Communication Module Development Plan)

작성일: 2026-01-01  
**상태**: 구현 중

---

## 개요

PX4와 VIM4 간의 ROS2 통신을 담당하는 모듈입니다. uXRCE-DDS를 통해 PX4 데이터를 수신하고, 향후 충돌 회피, 명령 발행 등의 기능을 확장할 수 있는 구조로 설계되었습니다.

**기반**: 화재 진압 드론 임무 플로우차트  
**통신 방식**: uXRCE-DDS (Micro-ROS Agent를 통한 PX4 ↔ ROS2 브리지)

**통신 방향 구분** (플로우차트 색상 기준):
- **🟡 노란색**: QGC → FC 또는 VIM4 → FC (비행 명령)
- **🟢 초록색**: VIM4 → QGC (상태 알림)
- **🔴 빨간색**: QGC → VIM4 (격발 명령)
- **🔵 파란색**: VIM4 내부 처리 (DRONE 단독)

**상세 토픽 구조**: `ROS2_TOPIC_ARCHITECTURE.md` 참조

---

## 폴더 구조

```
ros2/
├── src/
│   ├── status/                    # ✅ 완료: 상태 모니터링
│   │   ├── status_ros2_subscriber.h
│   │   └── status_ros2_subscriber.cpp
│   ├── collision/                 # 🔜 향후: 충돌 회피
│   │   ├── collision_avoidance_subscriber.h
│   │   ├── collision_avoidance_subscriber.cpp
│   │   ├── obstacle_detection_subscriber.h
│   │   └── obstacle_detection_subscriber.cpp
│   ├── command/                   # 🔜 향후: 명령 발행
│   │   ├── px4_command_publisher.h
│   │   ├── px4_command_publisher.cpp
│   │   ├── offboard_publisher.h
│   │   └── offboard_publisher.cpp
│   ├── formation/                 # 🔜 향후: 편대 제어
│   │   └── formation_controller.*
│   └── CMakeLists.txt
└── README.md
```

---

## 구현 완료: Status ROS2 Subscriber

### 기능
- PX4 상태 정보 구독 및 StatusOverlay 업데이트
- 배터리, GPS 정보 구독
- VIM4 자동 제어 상태 구독
- 소화탄, 편대 정보 구독

### 구독 토픽

#### PX4 uXRCE-DDS 토픽 (읽기 전용)
| 토픽 | 메시지 타입 | 필드 | 용도 | 업데이트 주기 |
|------|------------|------|------|--------------|
| `/fmu/out/vehicle_status_v1` | `px4_msgs::VehicleStatus` | `nav_state`, `arming_state`, `failsafe` | 비행 모드, 시동 상태 | 10Hz |
| `/fmu/out/battery_status` | `px4_msgs::BatteryStatus` | `remaining`, `voltage_v`, `current_a` | 배터리 잔량, 전압, 전류 | 1Hz |
| `/fmu/out/vehicle_gps_position` | `px4_msgs::SensorGps` | `latitude_deg`, `longitude_deg`, `satellites_used`, `fix_type` | GPS 위치, 위성 수 | 1Hz |

#### VIM4 커스텀 토픽 (구독)
| 토픽 | 메시지 타입 | 용도 | 발행 위치 |
|------|------------|------|----------|
| `/offboard/status` | `std_msgs::String` | VIM4 OFFBOARD 모드 상태 | `navigation/src/offboard/status_reporter.cpp` |
| `/ammunition/current` | `std_msgs::Int32` | 현재 소화탄 갯수 | `throwing_mechanism/src/fire_controller.cpp` |
| `/formation/current` | `std_msgs::Int32` | 현재 편대 번호 | `navigation/src/formation/formation_controller.cpp` |

#### VIM4 → QGC 토픽 (발행 예정)
| 토픽 | 메시지 타입 | 용도 | 발행 위치 |
|------|------------|------|----------|
| `/offboard/destination_reached` | `std_msgs::Bool` | 목적지 도착 완료 알림 | `navigation/src/offboard/waypoint_handler.cpp` |
| `/offboard/fire_ready` | `std_msgs::Bool` | 격발 준비 완료 알림 | `navigation/src/offboard/auto_targeting_handler.cpp` |

#### QGC → VIM4 토픽 (구독 예정)
| 토픽 | 메시지 타입 | 용도 | 구독 위치 |
|------|------------|------|----------|
| `/gcs/fire_command` | `std_msgs::Bool` | 격발 명령 | `navigation/src/offboard/command_receiver.cpp` |
| `/gcs/emergency_stop` | `std_msgs::Bool` | 비상 정지 명령 | `navigation/src/offboard/emergency_handler.cpp` |

#### VIM4 → PX4 토픽 (발행 예정)
| 토픽 | 메시지 타입 | 용도 | 발행 위치 |
|------|------------|------|----------|
| `/fmu/in/vehicle_command` | `px4_msgs::VehicleCommand` | 시동, 이륙, 복귀 명령 | `navigation/src/offboard/arm_handler.cpp`, `takeoff_handler.cpp`, `rtl_handler.cpp` |
| `/fmu/in/trajectory_setpoint` | `px4_msgs::TrajectorySetpoint` | 위치/속도 명령 (이동, 거리조정, 조준) | `navigation/src/offboard/waypoint_handler.cpp`, `distance_adjustment_handler.cpp`, `auto_targeting_handler.cpp` |
| `/fmu/in/offboard_control_mode` | `px4_msgs::OffboardControlMode` | 오프보드 제어 모드 설정 | `navigation/src/offboard/waypoint_handler.cpp` |

### PX4 nav_state 매핑

| nav_state | 모드 문자열 | 설명 |
|-----------|------------|------|
| 0 | MANUAL | 수동 모드 |
| 1 | ALTCTL | 고도 제어 모드 |
| 2 | POSCTL | 위치 제어 모드 |
| 3 | AUTO_MISSION | Mission 모드 |
| 4 | AUTO_LOITER | 자동 대기 모드 |
| 5 | AUTO_RTL | 자동 복귀 모드 |
| 6 | AUTO_TAKEOFF | 자동 이륙 모드 |
| 7 | AUTO_LAND | 자동 착륙 모드 |
| 9 | OFFBOARD | 오프보드 모드 |

### QoS (Quality of Service) 설정

**중요**: PX4 uXRCE-DDS 토픽을 구독할 때는 발행자와 동일한 QoS 설정을 사용해야 합니다.

#### PX4 uXRCE-DDS QoS 설정

PX4 16.0.0 이상에서는 모든 `/fmu/out/*` 토픽이 다음 QoS 설정을 사용합니다:

| QoS 속성 | 값 | 설명 |
|---------|-----|------|
| **Reliability** | `BEST_EFFORT` | 최선 노력 전송 (일부 메시지 손실 허용) |
| **Durability** | `TRANSIENT_LOCAL` | 마지막 발행된 메시지를 유지 (구독자 연결 시 전달) |
| **History** | `KEEP_LAST` | 마지막 N개 메시지 유지 |
| **Depth** | `10` | 큐 크기 |

#### 구독자 QoS 설정

VIM4 구독자는 반드시 PX4 발행자와 동일한 QoS를 사용해야 합니다:

```cpp
// status_ros2_subscriber.cpp
rclcpp::QoS px4_qos(10);
px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);  // 중요!
```

#### QoS 불일치 문제

**증상**:
- `ros2 topic list`에서는 토픽이 보이지만
- `ros2 topic echo`로 메시지를 받지 못함
- 구독자가 메시지를 수신하지 않음

**원인**:
- Durability 불일치: Publisher는 `TRANSIENT_LOCAL`, Subscriber는 `VOLATILE`
- ROS2 DDS는 QoS가 일치하지 않으면 구독자와 발행자를 매칭하지 않음

**해결**:
- 구독자의 Durability를 `TransientLocal`로 변경
- Reliability도 `BestEffort`로 일치시킴

#### VIM4 커스텀 토픽 QoS

VIM4에서 발행하는 커스텀 토픽은 기본 QoS를 사용합니다:

```cpp
// 기본 QoS (Reliable, Volatile)
rclcpp::QoS default_qos(10);
```

**참고**: 
- `/offboard/status`, `/ammunition/current`, `/formation/current` 등은 기본 QoS 사용
- PX4 토픽과는 독립적으로 동작

#### 디버깅 도구

올바른 QoS로 토픽 테스트:

```bash
# PX4 토픽 echo (올바른 QoS 사용)
ros2 topic echo /fmu/out/vehicle_status_v1 \
  --qos-profile sensor_data \
  --qos-durability transient_local

# 또는 스크립트 사용
./scripts/debug/ros2_topic_echo_px4.sh /fmu/out/vehicle_status_v1
```

**상세 내용**: `docs/QOS_COMPATIBILITY_FIX.md` 참조

---

## 향후 확장 계획

### 1. 충돌 회피 모듈 (Collision Avoidance)

**목적**: 다른 드론과의 충돌 방지, 장애물 회피

**구독 토픽**:
- `/drone_*/position` - 다른 드론의 위치 정보
- `/obstacle/detection` - 장애물 감지 정보
- `/collision/warning` - 충돌 경고

**발행 토픽**:
- `/collision/avoidance_command` - 회피 명령

**파일 구조**:
```
ros2/src/collision/
├── collision_avoidance_subscriber.h
├── collision_avoidance_subscriber.cpp
├── obstacle_detection_subscriber.h
└── obstacle_detection_subscriber.cpp
```

### 2. 명령 발행 모듈 (Command Publisher)

**목적**: PX4로 명령 전송, 자동 제어 명령 발행

**발행 토픽** (PX4 uXRCE-DDS):
- `/fmu/in/vehicle_command` - PX4 명령 (시동, 착륙 등)
- `/fmu/in/offboard_control_mode` - 오프보드 제어 모드
- `/fmu/in/trajectory_setpoint` - 궤적 설정점

**커스텀 발행 토픽**:
- `/offboard/command` - OFFBOARD 모드 명령

**파일 구조**:
```
ros2/src/command/
├── px4_command_publisher.h
├── px4_command_publisher.cpp
├── offboard_publisher.h
└── offboard_publisher.cpp
```

### 3. 편대 제어 모듈 (Formation Control)

**목적**: 다중 드론 편대 유지 및 재구성

**구독 토픽**:
- `/formation/command` - 편대 명령
- `/formation/target` - 목표 편대 구성

**발행 토픽**:
- `/formation/status` - 편대 상태
- `/formation/position` - 편대 내 위치

---

## 통신 아키텍처

```
PX4 Flight Controller
    ↓ (uXRCE-DDS)
Micro-ROS Agent
    ↓ (ROS2 DDS)
ROS2 토픽 (/fmu/out/*, /fmu/in/*)
    ↓
ros2/src/ 모듈들
    ↓
애플리케이션 (StatusOverlay, Command 등)
```

**중요**: 
- uXRCE-DDS는 Micro-ROS Agent가 처리
- 이 모듈은 ROS2 토픽 레벨에서만 동작
- PX4와 직접 통신하지 않음

---

## 빌드 및 사용

### 빌드
```bash
cd /home/khadas/humiro_fire_suppression/application
cmake -DENABLE_ROS2=ON ..
make
```

### 사용
```cpp
#ifdef ENABLE_ROS2
#include "../ros2/src/status/status_ros2_subscriber.h"

// 초기화
status_ros2_subscriber = new StatusROS2Subscriber(ros2_node, status_overlay);

// 메인 루프에서 스핀
status_ros2_subscriber->spin();
#endif
```

---

## 의존성

- **ROS2**: rclcpp, std_msgs
- **PX4 메시지**: px4_msgs (선택적, 없으면 기본 구조체 사용)
- **OpenCV**: StatusOverlay 사용

---

## 참고사항

### uXRCE-DDS vs MAVROS
- **uXRCE-DDS**: PX4 v1.14+ 기본 통신 방식, 직접 ROS2 토픽 제공
- **MAVROS**: ROS1/ROS2 브리지, 현재 미사용
- **현재 구현**: uXRCE-DDS 사용 (`/fmu/out/*`, `/fmu/in/*` 토픽)

### 토픽 네이밍 규칙
- **PX4 → ROS2**: `/fmu/out/*` (읽기, uXRCE-DDS)
- **ROS2 → PX4**: `/fmu/in/*` (쓰기, uXRCE-DDS)
- **VIM4 → QGC**: `/offboard/*` (상태 알림)
- **QGC → VIM4**: `/gcs/*` (명령)
- **VIM4 내부**: `/lidar/*`, `/thermal/*`, `/ammunition/*`, `/formation/*` 등

### 플로우차트 기반 통신 방향
- **🟡 노란색 (QGC → FC)**: `/fmu/in/vehicle_command`, `/fmu/in/trajectory_setpoint`
- **🟢 초록색 (VIM4 → QGC)**: `/offboard/status`, `/offboard/destination_reached`, `/offboard/fire_ready`
- **🔴 빨간색 (QGC → VIM4)**: `/gcs/fire_command`, `/gcs/emergency_stop`
- **🔵 파란색 (DRONE 단독)**: VIM4 내부 처리 (`/lidar/*`, `/thermal/*`)

**상세 내용**: `ROS2_TOPIC_ARCHITECTURE.md` 참조

---

**작성자**: Claude Code Assistant  
**버전**: v1.0  
**작성일**: 2026-01-01  
**최종 업데이트**: 2026-01-01

