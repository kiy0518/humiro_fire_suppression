# 드론 통신 설정 업데이트 가이드

## 중요: 프로젝트 구조 변경 후 필수 작업

프로젝트 구조를 재정비했으므로, 드론 통신 관련 설정을 새 경로로 업데이트해야 합니다.

## 업데이트가 필요한 항목

### 1. systemd 서비스 파일
- `micro-ros-agent.service` - Micro-ROS Agent 경로
- `mavlink-router.service` - MAVLink Router 설정
- `dnsmasq-px4.service` - DHCP 서버 설정

### 2. 네트워크 설정
- eth0 Netplan 설정
- WiFi Netplan 설정
- dnsmasq DHCP 설정

### 3. 환경 변수
- `.bashrc`의 ROS 환경 설정
- Micro-ROS Agent wrapper 스크립트

## 업데이트 방법

### 방법 1: 설정 스크립트 재실행 (권장)

```bash
cd ~/humiro_fire_suppression
source setup_env.sh
sudo ./scripts/install/003-apply_config.sh
```

이 스크립트가 자동으로:
1. systemd 서비스 파일 업데이트 (새 경로 반영)
2. 네트워크 설정 적용
3. Micro-ROS Agent wrapper 스크립트 업데이트
4. `.bashrc` 환경 변수 업데이트
5. 서비스 재시작

### 방법 2: 수동 업데이트

#### systemd 서비스 파일 업데이트

```bash
cd ~/humiro_fire_suppression

# micro-ros-agent.service 업데이트
sudo cp deployment/systemd/micro-ros-agent.service /etc/systemd/system/
sudo sed -i "s|%h|$HOME|g; s|khadas|$USER|g" /etc/systemd/system/micro-ros-agent.service

# mavlink-router.service 업데이트
sudo cp deployment/systemd/mavlink-router.service /etc/systemd/system/

# dnsmasq-px4.service 업데이트
sudo cp deployment/systemd/dnsmasq-px4.service /etc/systemd/system/

# systemd 재로드 및 서비스 재시작
sudo systemctl daemon-reload
sudo systemctl restart micro-ros-agent.service
sudo systemctl restart mavlink-router.service
sudo systemctl restart dnsmasq-px4.service
```

## 확인 방법

### 1. 서비스 상태 확인

```bash
sudo systemctl status micro-ros-agent.service
sudo systemctl status mavlink-router.service
sudo systemctl status dnsmasq-px4.service
```

### 2. 연결 확인

```bash
cd ~/humiro_fire_suppression
./scripts/check/101-check_px4_connection.sh
```

### 3. 서비스 로그 확인

```bash
sudo journalctl -u micro-ros-agent.service -n 50
sudo journalctl -u mavlink-router.service -n 50
```

## 변경 사항

### 이전 경로
- `/home/khadas/projects/Cluster_Drone/micro_ros_ws/`
- `/home/khadas/projects/Cluster_Drone/px4_ros2_ws/`
- `/home/khadas/projects/Cluster_Drone/start_micro_ros_agent_wrapper.sh`

### 새 경로 (환경 변수 사용)
- `$MICRO_ROS_WS` → `~/humiro_fire_suppression/workspaces/micro_ros_ws/`
- `$PX4_ROS2_WS` → `~/humiro_fire_suppression/workspaces/px4_ros2_ws/`
- `$SCRIPTS_RUNTIME/start_micro_ros_agent_wrapper.sh`

## 주의사항

1. **재부팅 권장**: 네트워크 설정 변경 후 재부팅하는 것이 안전합니다
2. **서비스 순서**: dnsmasq → mavlink-router → micro-ros-agent 순서로 시작됩니다
3. **환경 변수**: 모든 스크립트는 `setup_env.sh`를 통해 환경 변수를 로드합니다

## 문제 해결

### 서비스가 시작되지 않는 경우

1. 경로 확인:
   ```bash
   ls -la ~/humiro_fire_suppression/scripts/runtime/start_micro_ros_agent_wrapper.sh
   ```

2. 권한 확인:
   ```bash
   chmod +x ~/humiro_fire_suppression/scripts/runtime/start_micro_ros_agent_wrapper.sh
   ```

3. 로그 확인:
   ```bash
   sudo journalctl -u micro-ros-agent.service -n 100
   ```

### 네트워크 연결 문제

1. Netplan 설정 확인:
   ```bash
   cat /etc/netplan/01-eth0-px4.yaml
   ```

2. Netplan 적용:
   ```bash
   sudo netplan apply
   ```

3. 네트워크 인터페이스 확인:
   ```bash
   ip addr show eth0
   ```
