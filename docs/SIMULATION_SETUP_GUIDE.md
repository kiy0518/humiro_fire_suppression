# 군집 드론 시뮬레이션 환경 구축 가이드 (VirtualBox)

## 개요

VIM4 미션 컴퓨터에서 3대의 SITL 드론을 제어하기 위한 시뮬레이션 환경 구축 가이드입니다.

**VirtualBox 브리지 모드**를 사용하면 NAT 문제 없이 깔끔하게 구성할 수 있습니다.

---

## 시스템 구성도

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                  LAN (192.168.100.x/24)                             │
│                                                                                     │
│  ┌──────────────────────────────────────────────────────────────────────────────┐   │
│  │                        VirtualBox VM (192.168.100.50)                        │   │
│  │                              Ubuntu 22.04                                    │   │
│  │                                                                              │   │
│  │   ┌─────────────┐      ┌─────────────┐      ┌─────────────┐                  │   │
│  │   │  SITL #1    │      │  SITL #2    │      │  SITL #3    │                  │   │
│  │   │  -i 0       │      │  -i 1       │      │  -i 2       │                  │   │
│  │   │             │      │             │      │             │                  │   │
│  │   │ → .30:18001 │      │ → .31:18002 │      │ → .32:18003 │                  │   │
│  │   └──────┬──────┘      └──────┬──────┘      └──────┬──────┘                  │   │
│  │          │                    │                    │                         │   │
│  └──────────┼────────────────────┼────────────────────┼─────────────────────────┘   │
│             │ UDP                │ UDP                │ UDP                         │
│             ▼                    ▼                    ▼                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐                      │
│  │ VIM4 드론1       │  │ VIM4 드론2       │  │ VIM4 드론3       │                      │
│  │ 192.168.100.30  │  │ 192.168.100.31  │  │ 192.168.100.32  │                      │
│  │                 │  │                 │  │                 │                      │
│  │ ┌─────────────┐ │  │ ┌─────────────┐ │  │ ┌─────────────┐ │                      │
│  │ │mavlink-     │ │  │ │mavlink-     │ │  │ │mavlink-     │ │                      │
│  │ │router       │ │  │ │router       │ │  │ │router       │ │                      │
│  │ │             │ │  │ │             │ │  │ │             │ │                      │
│  │ │ FC:14540    │ │  │ │ FC:14540    │ │  │ │ FC:14540    │ │                      │
│  │ │ SITL:18001  │ │  │ │ SITL:18002  │ │  │ │ SITL:18003  │ │                      │
│  │ └──────┬──────┘ │  │ └──────┬──────┘ │  │ └──────┬──────┘ │                      │
│  │        │        │  │        │        │  │        │        │                      │
│  │ ┌──────┴──────┐ │  │ ┌──────┴──────┐ │  │ ┌──────┴──────┐ │                      │
│  │ │  WEB GUI    │ │  │ │  WEB GUI    │ │  │ │  WEB GUI    │ │                      │
│  │ │  :5000      │ │  │ │  :5000      │ │  │ │  :5000      │ │                      │
│  │ │ [FC/SITL]   │ │  │ │ [FC/SITL]   │ │  │ │ [FC/SITL]   │ │                      │
│  │ └─────────────┘ │  │ └─────────────┘ │  │ └─────────────┘ │                      │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘                      │
│           │ eth0               │ eth0               │ eth0                          │
│           │ 10.0.0.11          │ 10.0.0.11          │ 10.0.0.11                     │
└───────────┼────────────────────┼────────────────────┼────────────────────────────────┘
            ▼                    ▼                    ▼
┌───────────────────┐ ┌───────────────────┐ ┌───────────────────┐
│   실제 FC          │ │   실제 FC          │ │   실제 FC          │
│   10.0.0.12:14540 │ │   10.0.0.12:14540 │ │   10.0.0.12:14540 │
│   (실제 비행 시)    │ │   (실제 비행 시)    │ │   (실제 비행 시)    │
└───────────────────┘ └───────────────────┘ └───────────────────┘
```

---

## 동작 원리

### FC/SITL 자동 전환

mavlink-router는 **FC와 SITL 엔드포인트를 모두** 가지고 있습니다.

```ini
# FC 연결 (실제 비행)
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = 14540

# SITL 연결 (시뮬레이션)
[UdpEndpoint SITL]
Mode = Server
Address = 0.0.0.0
Port = 18001  # 드론별: 18001, 18002, 18003
```

| 상황 | FC (14540) | SITL (18001) | 결과 |
|------|------------|--------------|------|
| 실제 비행 | ✅ 연결됨 | ❌ 없음 | FC로 메시지 라우팅 |
| 시뮬레이션 | ❌ 없음 | ✅ 연결됨 | SITL로 메시지 라우팅 |

**웹 GUI에서 별도 설정 없이 자동 전환**됩니다.

---

## Part 1: VirtualBox 설치 및 VM 생성

### 1.1 VirtualBox 다운로드

https://www.virtualbox.org/wiki/Downloads 에서 다운로드 후 설치

### 1.2 Ubuntu ISO 다운로드

https://ubuntu.com/download/desktop 에서 Ubuntu 22.04 LTS 다운로드

### 1.3 VM 생성

1. VirtualBox 실행 → **새로 만들기** 클릭
2. 설정:
   - **이름**: PX4-SITL
   - **종류**: Linux
   - **버전**: Ubuntu (64-bit)
   - **메모리**: 8192 MB (최소 4GB, 권장 8GB)
   - **프로세서**: 4개 이상
   - **하드 디스크**: 50GB (동적 할당)

3. **만들기** 클릭

### 1.4 브리지 네트워크 설정 (중요!)

1. 생성된 VM 선택 → **설정** 클릭
2. **네트워크** → **어댑터 1**
3. 설정:
   - **연결 대상**: `브리지 어댑터`
   - **이름**: 실제 사용 중인 네트워크 어댑터 선택
     - 유선: `Intel(R) Ethernet...` 또는 `Realtek PCIe GbE...`
     - 무선: `Intel(R) Wi-Fi...` 또는 `Realtek...Wireless...`
   - **무작위 모드**: 모두 허용
4. **확인** 클릭

### 1.5 Ubuntu 설치

1. VM 선택 → **시작** 클릭
2. Ubuntu ISO 선택
3. 일반적인 Ubuntu 설치 진행
4. 설치 완료 후 재부팅

---

## Part 2: VM 네트워크 설정

### 2.1 IP 주소 확인

```bash
ip addr show
```

**192.168.100.x** 대역의 IP가 할당되어야 합니다.

### 2.2 고정 IP 설정 (권장)

```bash
sudo nano /etc/netplan/01-netcfg.yaml
```

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    enp0s3:
      dhcp4: no
      addresses:
        - 192.168.100.50/24
      routes:
        - to: default
          via: 192.168.100.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
```

```bash
sudo netplan apply
```

### 2.3 VIM4와 통신 테스트

```bash
ping 192.168.100.30  # VIM4 드론1
ping 192.168.100.31  # VIM4 드론2
ping 192.168.100.32  # VIM4 드론3
```

---

## Part 3: PX4 + Gazebo 설치

### 3.1 시스템 업데이트 및 의존성 설치

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
    git wget cmake build-essential \
    python3-pip python3-venv \
    openjdk-11-jdk ninja-build \
    exiftool astyle \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad
```

### 3.2 PX4 소스 다운로드

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# PX4 빌드 도구 설치 (시간 소요)
bash ./Tools/setup/ubuntu.sh
```

### 3.3 첫 빌드 (필수)

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

빌드가 완료되면 `Ctrl+C`로 종료합니다.

---

## Part 4: 시뮬레이션 실행

### 4.1 기존 프로세스 정리

```bash
pkill -9 px4
pkill -9 gz
pkill -9 ruby
```

### 4.2 터미널 3개 열고 PX4 실행

**시뮬레이션 좌표**: 위도 35.905863, 경도 128.802615

**배치 구조**:
```
        좌측(-10m)      센터(0m)       우측(+10m)
            ▼             ▼              ▼
         드론 2         드론 1          드론 3
        [-10,0,0]       [0,0,0]        [10,0,0]
```

**터미널 1 (드론 1 - 센터):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 \
HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```

**터미널 2 (드론 2 - 좌측 10m):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 \
HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-10,0,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 1
```

**터미널 3 (드론 3 - 우측 10m):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 \
HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="10,0,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 2
```

### 4.3 각 PX4 콘솔에서 mavlink 설정

> **중요**: 각 SITL이 해당 VIM4로 직접 메시지를 전송합니다.

**드론 1 콘솔 (pxh>) → VIM4 드론1 (192.168.100.30):**
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
mavlink stop-all
mavlink start -u 14540 -o 18001 -t 192.168.100.30 -r 4000000
```

**드론 2 콘솔 → VIM4 드론2 (192.168.100.31):**
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
mavlink stop-all
mavlink start -u 14541 -o 18002 -t 192.168.100.31 -r 4000000
```

**드론 3 콘솔 → VIM4 드론3 (192.168.100.32):**
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
mavlink stop-all
mavlink start -u 14542 -o 18003 -t 192.168.100.32 -r 4000000
```

---

## Part 5: VIM4 mavlink-router 설정

### 5.1 설정 파일 구조

각 VIM4의 `/etc/mavlink-router/main.conf`:

```ini
# ==============================================================================
# MAVLink Router 설정 - FC/SITL 자동 전환 모드
# ==============================================================================

[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

# FC 연결 (실제 비행용) - eth0 네트워크
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = 14540

# SITL 연결 (시뮬레이션용) - wlan 네트워크
# 드론1=18001, 드론2=18002, 드론3=18003
[UdpEndpoint SITL]
Mode = Server
Address = 0.0.0.0
Port = 18001  # ← 드론별로 변경

# GCS (QGroundControl)
[UdpEndpoint GCS]
Mode = Normal
Address = 192.168.100.255
Port = 14550

# External (테스트/디버깅)
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = 16001

# ROS2 노드
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = 14551

# Application Manager
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = 15001
```

### 5.2 드론별 SITL 포트

| VIM4 | IP 주소 | SITL 포트 |
|------|---------|-----------|
| 드론 1 | 192.168.100.30 | 18001 |
| 드론 2 | 192.168.100.31 | 18002 |
| 드론 3 | 192.168.100.32 | 18003 |

### 5.3 mavlink-router 재시작

```bash
sudo systemctl restart mavlink-router
```

---

## Part 6: QGroundControl 연결 (선택사항)

QGC에서도 시뮬레이션 드론을 모니터링하려면:

### 6.1 각 PX4 콘솔에서 QGC용 mavlink 추가

```
# 드론 1 (VIM4 + QGC 동시 연결)
mavlink start -u 14550 -o 14550 -t 192.168.100.4 -r 4000000

# 드론 2
mavlink start -u 14551 -o 14551 -t 192.168.100.4 -r 4000000

# 드론 3
mavlink start -u 14552 -o 14552 -t 192.168.100.4 -r 4000000
```

### 6.2 QGC Comm Links 설정

| Name | Type | Listening Port |
|------|------|----------------|
| Drone1 | UDP | 14550 |
| Drone2 | UDP | 14551 |
| Drone3 | UDP | 14552 |

---

## Part 7: 테스트

### 7.1 VIM4 웹 GUI에서 확인

1. 브라우저에서 `http://192.168.100.30:5000` 접속
2. **라우터 페이지**에서 SITL 모드 확인
3. FC 연결 상태가 "연결됨"으로 표시되면 성공

### 7.2 PX4 콘솔에서 이륙 테스트

```
commander arm -f
commander takeoff
```

### 7.3 웹 GUI에서 명령 테스트

1. 대시보드에서 이륙/착륙 버튼 클릭
2. SITL 드론이 반응하는지 확인

---

## Part 8: 문제 해결

### 8.1 VM이 192.168.100.x IP를 받지 못함

**원인**: 브리지 어댑터 설정 오류

**해결**:
1. VM 종료
2. 설정 → 네트워크 → 어댑터 1
3. 연결 대상: "브리지 어댑터" 확인
4. 이름: 실제 사용 중인 네트워크 어댑터 선택
5. VM 재시작

### 8.2 VIM4에서 SITL 연결 안됨

**확인사항**:
```bash
# VM에서 VIM4로 ping
ping 192.168.100.30

# VIM4에서 mavlink-router 로그 확인
journalctl -u mavlink-router -f
```

### 8.3 웹 GUI에서 FC 연결 안됨 표시

**확인사항**:
1. SITL이 올바른 VIM4 IP로 메시지 전송 중인지 확인
2. VIM4 방화벽 확인: `sudo ufw status`
3. SITL 포트(18001~18003) 열려있는지 확인

### 8.4 여러 드론이 같은 위치에 생성됨

**해결**: `PX4_GZ_MODEL_POSE` 환경변수로 위치 분리
```bash
PX4_GZ_MODEL_POSE="0,0,0,0,0,0"    # 드론 1
PX4_GZ_MODEL_POSE="-10,0,0,0,0,0"  # 드론 2
PX4_GZ_MODEL_POSE="10,0,0,0,0,0"   # 드론 3
```

---

## 환경 변수 참조

| 변수 | 설명 | 예시 |
|------|------|------|
| `HEADLESS` | GUI 없이 실행 | `1` |
| `PX4_SYS_AUTOSTART` | 기체 타입 | `4001` (x500) |
| `PX4_GZ_MODEL_POSE` | 초기 위치 (x,y,z,roll,pitch,yaw) | `"10,0,0,0,0,0"` |
| `PX4_HOME_LAT` | 홈 위치 위도 | `35.905863` |
| `PX4_HOME_LON` | 홈 위치 경도 | `128.802615` |
| `PX4_HOME_ALT` | 홈 위치 고도 (m) | `0` |
| `-i` | 인스턴스 번호 | `0`, `1`, `2` |

---

## mavlink 명령어 참조

```
mavlink start -u <local_port> -o <remote_port> -t <target_ip> -r <rate>
```

| 옵션 | 설명 |
|------|------|
| `-u` | 로컬 UDP 포트 |
| `-o` | 원격 UDP 포트 |
| `-t` | 타겟 IP 주소 |
| `-r` | 전송 속도 (bps) |

---

## 빠른 시작 체크리스트

- [ ] VirtualBox 설치 및 Ubuntu VM 생성
- [ ] VM 브리지 네트워크 설정 (192.168.100.50)
- [ ] PX4 빌드 완료
- [ ] 각 VIM4 mavlink-router에 SITL 포트 설정
- [ ] SITL 3대 실행 및 mavlink 타겟 설정
- [ ] 웹 GUI에서 연결 확인

---

**작성일**: 2026-01-22
**버전**: 3.1
**테스트 환경**: VirtualBox 7.x + Ubuntu 22.04 + PX4 v1.15 + VIM4
