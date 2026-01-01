# ROS2 통신 모듈

PX4와 VIM4 간의 ROS2 통신을 담당하는 모듈입니다.

## 구조

```
ros2/src/
├── status/              # 상태 모니터링 관련
│   ├── status_ros2_subscriber.h
│   └── status_ros2_subscriber.cpp
├── collision/           # 충돌 회피 관련 (향후 추가)
│   └── ...
├── command/             # 명령 발행 관련 (향후 추가)
│   └── ...
└── CMakeLists.txt
```

## 현재 구현

### Status ROS2 Subscriber
- **파일**: `status/status_ros2_subscriber.h`, `.cpp`
- **기능**: PX4 상태, 배터리, GPS 등을 구독하여 StatusOverlay 업데이트
- **구독 토픽**:
  - `/fmu/out/vehicle_status` - PX4 비행 상태
  - `/fmu/out/battery_status` - 배터리 상태
  - `/fmu/out/vehicle_gps_position` - GPS 정보
  - `/offboard/status` - OFFBOARD 모드 상태
  - `/ammunition/current` - 소화탄 갯수
  - `/formation/current` - 편대 정보

## 향후 확장

### Collision Avoidance (충돌 회피)
- 다른 드론의 위치 정보 구독
- 장애물 감지 정보 구독
- 회피 명령 발행

### Command Publisher (명령 발행)
- PX4로 명령 전송 (`/fmu/in/*` 토픽)
- 자동 제어 명령 발행

### Formation Control (편대 제어)
- 편대 유지 명령
- 편대 재구성 명령

## 빌드

```bash
cd /home/khadas/humiro_fire_suppression/application
cmake -DENABLE_ROS2=ON ..
make
```

## 사용법

`application/main.cpp`에서:

```cpp
#ifdef ENABLE_ROS2
#include "../ros2/src/status/status_ros2_subscriber.h"

// 초기화
status_ros2_subscriber = new StatusROS2Subscriber(ros2_node, status_overlay);

// 메인 루프에서 스핀
status_ros2_subscriber->spin();
#endif
```

