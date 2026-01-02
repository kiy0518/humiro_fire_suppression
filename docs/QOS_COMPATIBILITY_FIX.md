# QoS 호환성 수정

## 문제

PX4에서 발행하는 토픽의 QoS 설정과 VIM4 구독자의 QoS 설정이 불일치하여 메시지를 수신하지 못하는 문제가 있었습니다.

## 원인

- **Publisher (PX4)**: `Reliability=BEST_EFFORT`, `Durability=TRANSIENT_LOCAL`
- **Subscriber (VIM4)**: `Reliability=BEST_EFFORT`, `Durability=VOLATILE`

Durability 설정이 다르면 ROS2 DDS는 구독자와 발행자를 매칭하지 않습니다.

## 해결

`status_ros2_subscriber.cpp`에서 QoS 설정을 수정:

```cpp
// 변경 전
px4_qos.durability(rclcpp::DurabilityPolicy::Volatile);

// 변경 후
px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
```

## 적용 방법

1. 코드 변경 완료
2. 재빌드:
   ```bash
   cd /home/khadas/humiro_fire_suppression/application/build
   cmake ..
   make -j4
   ```
3. 프로그램 재시작:
   ```bash
   ./scripts/runtime/service-control.sh restart
   ```

## 확인

재빌드 후 다음 명령으로 메시지 수신 확인:

```bash
# 올바른 QoS로 테스트
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-profile sensor_data --qos-durability transient_local --once

# 또는 스크립트 사용
./scripts/debug/ros2_topic_echo_px4.sh /fmu/out/vehicle_status_v1 --once
```

## 참고

- PX4 16.0.0 이상에서는 모든 `/fmu/out/*` 토픽이 `TRANSIENT_LOCAL` durability를 사용합니다.
- `TRANSIENT_LOCAL`은 구독자가 나중에 연결되어도 마지막 발행된 메시지를 받을 수 있게 해줍니다.

