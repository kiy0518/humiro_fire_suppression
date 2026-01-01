# ROS2 통신 모듈 개발 계획 (ROS2 Communication Module Development Plan)

작성일: 2026-01-01  
**상태**: 구현 중

---

## 개요

PX4와 VIM4 간의 ROS2 통신을 담당하는 모듈입니다. uXRCE-DDS를 통해 PX4 데이터를 수신하고, 향후 충돌 회피, 명령 발행 등의 기능을 확장할 수 있는 구조로 설계되었습니다.

**통신 방식**: uXRCE-DDS (Micro-ROS Agent를 통한 PX4 ↔ ROS2 브리지)

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
│   │   ├── auto_control_publisher.h
│   │   └── auto_control_publisher.cpp
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
| 토픽 | 메시지 타입 | 필드 | 용도 |
|------|------------|------|------|
| `/fmu/out/vehicle_status` | `px4_msgs::VehicleStatus` | `nav_state`, `arming_state` | 비행 모드, 시동 상태 |
| `/fmu/out/battery_status` | `px4_msgs::BatteryStatus` | `remaining`, `voltage_v`, `current_a` | 배터리 잔량, 전압, 전류 |
| `/fmu/out/vehicle_gps_position` | `px4_msgs::VehicleGpsPosition` | `satellites_used`, `lat`, `lon` | GPS 위성 수, 위치 |

#### VIM4 커스텀 토픽
| 토픽 | 메시지 타입 | 용도 |
|------|------------|------|
| `/auto_mode/status` | `std_msgs::String` | VIM4 자동 제어 상태 (OFFBOARD 모드) |
| `/ammunition/current` | `std_msgs::Int32` | 현재 소화탄 갯수 |
| `/formation/current` | `std_msgs::Int32` | 현재 편대 번호 |

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
- `/auto_mode/command` - 자동 제어 명령

**파일 구조**:
```
ros2/src/command/
├── px4_command_publisher.h
├── px4_command_publisher.cpp
├── auto_control_publisher.h
└── auto_control_publisher.cpp
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
- **PX4 → ROS2**: `/fmu/out/*` (읽기)
- **ROS2 → PX4**: `/fmu/in/*` (쓰기)
- **커스텀**: `/auto_mode/*`, `/ammunition/*`, `/formation/*` 등

---

**작성자**: Claude Code Assistant  
**버전**: v1.0  
**작성일**: 2026-01-01  
**최종 업데이트**: 2026-01-01

