# PX4 16.0.0 uXRCE-DDS 설정 가이드

PX4 펌웨어 버전 16.0.0 이상에서는 `ROS_DOMAIN_ID` 대신 `uXRCE-DDS_DOM_ID` 파라미터를 사용합니다.

## 파라미터 설정

### QGroundControl에서 설정

1. **QGroundControl 연결**
   - 드론을 QGC에 연결

2. **파라미터 검색**
   - 상단 메뉴: **Parameters** → 검색창에 `uXRCE` 입력

3. **uXRCE-DDS_DOM_ID 설정**
   - `uXRCE-DDS_DOM_ID` 파라미터를 찾아서 **0**으로 설정
   - 이 값은 VIM4의 `ROS_DOMAIN_ID=0`과 일치해야 함

4. **uXRCE-DDS Agent 설정 확인** (PX4 16.0.0+)
   - `uxrce_dds_ag_ip`: micro-ROS Agent IP 주소 (정수 형식)
     - 예: `167772171` = `10.0.0.11` (VIM4 eth0 IP)
     - 변환: `(a << 24) | (b << 16) | (c << 8) | d` (a.b.c.d)
   - `UXRCE_DDS_PRT`: micro-ROS Agent UDP 포트 (예: `8888`)

### 파라미터 요약

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `uXRCE-DDS_DOM_ID` | 0 | DDS 도메인 ID (ROS_DOMAIN_ID와 동일) |
| `uxrce_dds_ag_ip` | 167772171 | micro-ROS Agent IP (정수 형식, 10.0.0.11) |
| `UXRCE_DDS_PRT` | 8888 | micro-ROS Agent UDP 포트 |

## 연결 확인

### 1. 네트워크 연결 확인

```bash
# FC IP로 ping 테스트
ping 10.0.0.12
```

### 2. micro-ROS Agent 실행 확인

```bash
# Agent 실행 상태 확인
ps aux | grep micro_ros_agent

# 포트 확인
netstat -un | grep 8888
```

### 3. ROS2 토픽 확인

```bash
# 토픽 리스트 확인
ros2 topic list | grep "^/fmu/"

# 토픽 정보 확인 (Publisher 존재 여부)
ros2 topic info /fmu/out/vehicle_status_v1

# 메시지 수신 테스트 (올바른 QoS 사용)
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-profile sensor_data
```

### 4. 연결 진단 스크립트

```bash
# 전체 연결 상태 확인
./scripts/debug/check_px4_connection.sh
```

## 문제 해결

### 메시지가 수신되지 않는 경우

1. **PX4 시동 확인**
   - QGC에서 PX4가 시동이 걸려 있는지 확인
   - 시동이 안 걸리면 데이터를 발행하지 않음

2. **uXRCE-DDS_DOM_ID 확인**
   - QGC에서 `uXRCE-DDS_DOM_ID = 0` 확인
   - VIM4의 `ROS_DOMAIN_ID=0`과 일치해야 함

3. **네트워크 연결 확인**
   ```bash
   ping 10.0.0.12  # FC IP
   ```

4. **micro-ROS Agent 재시작**
   ```bash
   pkill -f micro_ros_agent
   ./scripts/runtime/start_micro_ros_agent_wrapper.sh &
   ```

5. **QoS 설정 확인**
   - PX4는 `BEST_EFFORT` QoS를 사용
   - `ros2 topic echo` 시 `--qos-profile sensor_data` 옵션 사용
   - 또는 `./scripts/debug/ros2_topic_echo_px4.sh` 스크립트 사용

## 참고

- PX4 16.0.0 이전 버전: `ROS_DOMAIN_ID` 파라미터 사용
- PX4 16.0.0 이상: `uXRCE-DDS_DOM_ID` 파라미터 사용
- 두 파라미터는 동일한 기능을 하지만 이름만 다름

