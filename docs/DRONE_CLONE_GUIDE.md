# 드론 기체 복제 가이드

## 개요

1번 기체에서 정상 동작 확인 후, 백업 이미지를 생성하여 2번 기체에 적용하는 방법입니다.

## 적용 절차

### 1. 백업 이미지 생성 (1번 기체에서)

```bash
# 전체 시스템 백업 이미지 생성 (예: Clonezilla, dd 등 사용)
# 또는 SD 카드 전체 복제
```

### 2. 백업 이미지 복원 (2번 기체에)

```bash
# 백업 이미지를 2번 기체에 복원
```

### 3. device_config.env 수정 (2번 기체에서)

```bash
cd ~/humiro_fire_suppression
nano config/device_config.env
```

**수정할 항목:**

```bash
# 기체 식별
DRONE_ID=2                        # 1 → 2로 변경
ROS_NAMESPACE=drone2              # drone1 → drone2로 변경

# 네트워크: eth0
ETH0_IP=10.0.0.21                 # 10.0.0.11 → 10.0.0.21로 변경
FC_IP=10.0.0.22                   # 10.0.0.12 → 10.0.0.22로 변경

# 네트워크: WiFi
WIFI_IP=192.168.100.21            # 192.168.100.11 → 192.168.100.21로 변경
```

**WiFi SSID/Password는 동일하게 유지** (같은 네트워크 사용 시)

### 4. 설정 적용 (2번 기체에서)

```bash
cd ~/humiro_fire_suppression
sudo ./scripts/install/003-apply_config.sh
```

이 스크립트가 자동으로:
- ✅ Netplan 네트워크 설정 (eth0, WiFi)
- ✅ dnsmasq DHCP 서버 설정
- ✅ mavlink-router 설정
- ✅ micro-ROS Agent 설정
- ✅ systemd 서비스 파일 업데이트
- ✅ .bashrc 환경 변수 업데이트
- ✅ 서비스 활성화 및 재시작

### 5. 재부팅 (필수)

```bash
sudo reboot
```

### 6. 정상 동작 확인

```bash
cd ~/humiro_fire_suppression
source setup_env.sh
./scripts/check/101-check_px4_connection.sh
```

## 기체별 설정 요약

| 항목 | 1번 기체 | 2번 기체 | 3번 기체 |
|------|----------|----------|----------|
| DRONE_ID | 1 | 2 | 3 |
| ROS_NAMESPACE | drone1 | drone2 | drone3 |
| ETH0_IP | 10.0.0.11 | 10.0.0.21 | 10.0.0.31 |
| FC_IP | 10.0.0.12 | 10.0.0.22 | 10.0.0.32 |
| WIFI_IP | 192.168.100.11 | 192.168.100.21 | 192.168.100.31 |

## 주의사항

### ✅ 자동 처리되는 항목

다음 항목들은 `003-apply_config.sh`가 자동으로 처리하므로 **수정 불필요**:

- FC IP (DHCP 범위 자동 계산)
- QGC 브로드캐스트 주소 (WiFi 서브넷 기반 자동 계산)
- ROS_DOMAIN_ID (모든 기체 동일: 0)
- XRCE_DDS_PORT (모든 기체 동일: 8888)
- FC_MAVLINK_PORT (모든 기체 동일: 14540)
- QGC_UDP_PORT (모든 기체 동일: 14550)

### ⚠️ 추가 확인 사항

1. **PX4 파라미터 설정 (QGroundControl에서)**
   - `MAV_SYS_ID` = DRONE_ID 값으로 설정
   - `UXRCE_DDS_AG_IP` = ETH0_IP의 10진수 변환값
   - `UXRCE_DDS_PRT` = 8888
   - `UXRCE_DDS_DOM_ID` = 0
   - `UXRCE_DDS_CFG` = 1000 (Ethernet)

2. **카메라 하드웨어**
   - 열화상 카메라와 RGB 카메라가 정상 연결되어 있는지 확인
   - `/dev/video*` 디바이스 확인

3. **워크스페이스 빌드**
   - 백업 이미지에 빌드된 워크스페이스가 포함되어 있다면 추가 빌드 불필요
   - 빌드가 누락된 경우:
     ```bash
     ./scripts/install/004-rebuild_workspaces.sh
     ```

## 문제 해결

### 네트워크 연결 안 됨
```bash
# Netplan 설정 확인
sudo netplan get
sudo netplan apply

# NetworkManager 상태 확인
nmcli connection show
nmcli device status
```

### 서비스 시작 안 됨
```bash
# 서비스 상태 확인
sudo systemctl status micro-ros-agent.service
sudo systemctl status mavlink-router.service
sudo systemctl status dnsmasq-px4.service

# 로그 확인
sudo journalctl -u micro-ros-agent.service -f
sudo journalctl -u mavlink-router.service -f
```

### PX4 연결 안 됨
```bash
# eth0 연결 확인
ip addr show eth0
ping 10.0.0.12  # FC IP (1번 기체 기준)

# dnsmasq 로그 확인
sudo tail -f /var/log/dnsmasq-px4.log
```

## 요약

**2번 기체 적용은 다음 3단계로 완료:**

1. ✅ `device_config.env` 수정 (DRONE_ID, IP 주소)
2. ✅ `sudo ./scripts/install/003-apply_config.sh` 실행
3. ✅ `sudo reboot` 후 정상 동작 확인

**끝!** 🎉
