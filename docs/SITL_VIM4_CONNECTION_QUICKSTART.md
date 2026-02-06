# SITL ↔ VIM4 연결 빠른 시작 가이드

## 개요

PX4 SITL 시뮬레이터와 VIM4 미션 컴퓨터 간의 연결 설정 가이드입니다.

**연결 구조:**
```
┌─────────────────────────────────────────────────────────────────────┐
│                    SITL PC (192.168.100.100)                        │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    PX4 SITL (-i 0)                          │   │
│  │                                                              │   │
│  │  ┌─────────────────┐    ┌─────────────────────────────────┐ │   │
│  │  │   MAVLink       │    │     uXRCE-DDS Client            │ │   │
│  │  │   UDP 14540     │    │     UDP → 192.168.100.11:8888   │ │   │
│  │  │   → VIM4:18001  │    │                                 │ │   │
│  │  └────────┬────────┘    └────────────────┬────────────────┘ │   │
│  └───────────┼──────────────────────────────┼──────────────────┘   │
│              │                              │                       │
└──────────────┼──────────────────────────────┼───────────────────────┘
               │ MAVLink (FC 제어)             │ ROS2 토픽 (상태 정보)
               ▼                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    VIM4 (192.168.100.11)                            │
│                                                                     │
│  ┌─────────────────┐    ┌─────────────────────────────────────┐    │
│  │  mavlink-router │    │     MicroXRCE-DDS Agent             │    │
│  │  Port 18001     │    │     Port 8888                       │    │
│  │       ↓         │    │            ↓                        │    │
│  │  Application    │    │     /fmu/out/vehicle_status_v1      │    │
│  │  (커스텀메시지) │    │     /fmu/out/vehicle_local_position │    │
│  └─────────────────┘    └─────────────────────────────────────┘    │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                 humiro_fire_suppression                      │   │
│  │                                                              │   │
│  │   CustomMessage ─────────────────────┐                       │   │
│  │   (커스텀메시지 수신/처리)            │                       │   │
│  │                                       ▼                       │   │
│  │   OffboardManager ───────── ROS2 Subscriber ◄── vehicle_status│   │
│  │   (OFFBOARD 제어)          (FC 상태 구독)                    │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 필수 조건

| 항목 | SITL PC | VIM4 |
|------|---------|------|
| IP | 192.168.100.100 | 192.168.100.11 |
| PX4 | SITL 실행 | - |
| mavlink-router | - | 실행 중 |
| 애플리케이션 | - | humiro_fire_suppression |

---

## Step 1: 네트워크 확인

### VIM4에서 실행
```bash
# SITL PC로 ping
ping -c 3 192.168.100.100
```

### SITL PC에서 실행
```bash
# VIM4로 ping
ping -c 3 192.168.100.11
```

---

## Step 2: SITL PC에서 PX4 실행

```bash
cd ~/PX4-Autopilot

# 시뮬레이션 시작 (Gazebo 포함)
make px4_sitl gazebo

# 또는 Headless 모드 (GUI 없이)
HEADLESS=1 make px4_sitl gz_x500
```

**시뮬레이션 시작 확인:**
- `pxh>` 프롬프트가 나타나면 성공

---

## Step 3: MAVLink 연결 설정 (SITL PC → VIM4)

**PX4 콘솔 (pxh>)에서 실행:**

```bash
# RC 없이 OFFBOARD 허용
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0

# 기존 mavlink 중지
mavlink stop-all

# VIM4로 MAVLink 전송
mavlink start -u 14540 -o 18001 -t 192.168.100.11 -r 4000000
```

**옵션 설명:**
| 옵션 | 값 | 설명 |
|------|-----|------|
| `-u` | 14540 | SITL 로컬 포트 |
| `-o` | 18001 | VIM4 수신 포트 |
| `-t` | 192.168.100.11 | VIM4 IP 주소 |
| `-r` | 4000000 | 전송 속도 (4Mbps) |

---

## Step 4: uXRCE-DDS 연결 설정 (ROS2 토픽)

> **중요**: 이 단계가 없으면 VIM4에서 FC 상태를 수신할 수 없습니다!

**PX4 콘솔 (pxh>)에서 실행:**

```bash
# uXRCE-DDS 클라이언트 시작 (VIM4로 ROS2 토픽 전송)
uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888
```

**옵션 설명:**
| 옵션 | 값 | 설명 |
|------|-----|------|
| `-t` | udp | 전송 프로토콜 |
| `-h` | 192.168.100.11 | VIM4 IP 주소 |
| `-p` | 8888 | MicroXRCE-DDS Agent 포트 |

---

## Step 5: VIM4 mavlink-router 확인

```bash
# 상태 확인
systemctl status mavlink-router

# 필요 시 재시작
sudo systemctl restart mavlink-router
```

**설정 파일 확인 (`/etc/mavlink-router/main.conf`):**
```ini
[UdpEndpoint SITL]
Mode = Server
Address = 0.0.0.0
Port = 18001
```

---

## Step 6: VIM4 애플리케이션 실행

```bash
cd /home/khadas/humiro_fire_suppression/application/build
./humiro_fire_suppression
```

**정상 연결 시 출력:**
```
✓ RGB 카메라 준비 완료
✓ 열화상 카메라 준비 완료
✓ [토픽 수신 확인] /fmu/out/vehicle_status_v1 첫 메시지 수신!
```

---

## Step 7: 연결 확인

### MAVLink 패킷 확인 (VIM4)
```bash
sudo tcpdump -i wlan0 -c 5 udp port 18001
```

### ROS2 토픽 확인 (VIM4)
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep fmu
ros2 topic hz /fmu/out/vehicle_status_v1
```

---

## 전체 명령어 요약 (복사용)

### SITL PC (PX4 콘솔에서)
```bash
# 1. 파라미터 설정
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0

# 2. MAVLink 설정
mavlink stop-all
mavlink start -u 14540 -o 18001 -t 192.168.100.11 -r 4000000

# 3. uXRCE-DDS 설정 (ROS2)
uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888
```

### VIM4
```bash
# mavlink-router 재시작
sudo systemctl restart mavlink-router

# 애플리케이션 실행
cd /home/khadas/humiro_fire_suppression/application/build
./humiro_fire_suppression
```

---

## 문제 해결

### "FC 연결 없음" 에러

**원인**: uXRCE-DDS 클라이언트가 실행되지 않음

**해결**: SITL PC의 PX4 콘솔에서 실행
```bash
uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888
```

### MAVLink 패킷은 오지만 ROS2 토픽 없음

**원인**: uXRCE-DDS 연결 실패

**확인:**
```bash
# VIM4에서 ROS2 토픽 확인
ros2 topic list | grep fmu
# 빈 결과 → uXRCE-DDS 미연결
```

### WiFi 끊김 후 재연결 안됨

**해결:**
1. VIM4: `sudo systemctl restart mavlink-router`
2. SITL PC: PX4 콘솔에서 mavlink와 uxrce_dds_client 재시작

---

## 다중 드론 설정

| 드론 | VIM4 IP | MAVLink 포트 | uXRCE-DDS 포트 |
|------|---------|--------------|----------------|
| #1 | 192.168.100.11 | 18001 | 8888 |
| #2 | 192.168.100.12 | 18002 | 8888 |
| #3 | 192.168.100.13 | 18003 | 8888 |

**드론 2 예시 (PX4 -i 1):**
```bash
mavlink start -u 14541 -o 18002 -t 192.168.100.12 -r 4000000
uxrce_dds_client start -t udp -h 192.168.100.12 -p 8888
```

---

**작성일**: 2026-02-05
**버전**: 1.0
