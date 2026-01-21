# 군집 드론 시뮬레이션 환경 구축 가이드

## 빠른 시작 (설치 완료 후)

### 3대 드론 헤드리스 실행 (10m 간격 배치)

**시뮬레이션 좌표**: 위도 35.905863, 경도 128.802615 (대구 지역)

**배치 구조**:
```
        좌측(-10m)      센터(0m)       우측(+10m)
            ▼             ▼              ▼
         드론 2         드론 1          드론 3
        [-10,0,0]       [0,0,0]        [10,0,0]

        ◄────── 10m ──────►◄────── 10m ──────►
```

**WSL2 터미널 3개 열기**

**터미널 1 (드론 1 - 센터):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 0
```

**터미널 2 (드론 2 - 좌측 10m):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-10,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 1
```

**터미널 3 (드론 3 - 우측 10m):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="10,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 2
```

### PX4 콘솔 설정 (각 터미널에서)

**드론 1 콘솔:**
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
mavlink stop-all
mavlink start -u 18001 -o 18001 -t 172.20.64.1 -r 4000000
```

**드론 2 콘솔:**
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
mavlink stop-all
mavlink start -u 18002 -o 18002 -t 172.20.64.1 -r 4000000
```

**드론 3 콘솔:**
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
mavlink stop-all
mavlink start -u 18003 -o 18003 -t 172.20.64.1 -r 4000000
```

> **참고**: `172.20.64.1`은 WSL2에서 Windows Host로 접근하는 IP입니다.
> 환경에 따라 다를 수 있으니 `ip route | grep default` 명령으로 확인하세요.

### QGroundControl 설정

**Comm Links 추가** (Application Settings → Comm Links):

| Name | Type | Listening Port |
|------|------|----------------|
| Drone1 | UDP | 18001 |
| Drone2 | UDP | 18002 |
| Drone3 | UDP | 18003 |

각 링크에서 **Connect** 클릭

### 이륙 테스트

각 PX4 콘솔에서:
```
commander arm -f
commander takeoff
```

---

## Part 1: WSL2 설치

### 1.1 WSL2 설치 (PowerShell 관리자 권한)

```powershell
# WSL 설치
wsl --install

# Ubuntu 22.04 설치
wsl --install -d Ubuntu-22.04

# 재부팅 후 버전 확인
wsl --list --verbose
```

### 1.2 Windows 방화벽 설정

```powershell
# MAVLink UDP 포트 허용
netsh advfirewall firewall add rule name="PX4 SITL Multi" dir=in action=allow protocol=UDP localport=18001-18003
```

---

## Part 2: PX4 + Gazebo 설치 (WSL2)

### 2.1 시스템 업데이트 및 의존성 설치

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

### 2.2 PX4 소스 다운로드

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# PX4 빌드 도구 설치 (시간 소요)
bash ./Tools/setup/ubuntu.sh
```

### 2.3 첫 빌드 (필수)

```bash
cd ~/PX4-Autopilot

# Gazebo Harmonic용 x500 모델 빌드
make px4_sitl gz_x500
```

빌드가 완료되면 `Ctrl+C`로 종료합니다.

---

## Part 3: 네트워크 구조

```
┌─────────────────────────────────────────────────────────────┐
│                    WSL2 Ubuntu 22.04                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐          │
│  │ PX4 SITL #1 │  │ PX4 SITL #2 │  │ PX4 SITL #3 │          │
│  │ Port 18001  │  │ Port 18002  │  │ Port 18003  │          │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘          │
│         │                │                │                  │
│         └────────────────┼────────────────┘                  │
│                          │                                   │
│              WSL2 IP: 172.20.65.x                            │
└──────────────────────────┼───────────────────────────────────┘
                           │ UDP (172.20.64.1)
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                     Windows Host                              │
│              IP: 172.20.64.1 (WSL Gateway)                   │
│              IP: 192.168.100.4 (LAN)                         │
│  ┌────────────────────────────────────┐                      │
│  │         QGroundControl             │                      │
│  │   UDP Listen: 18001, 18002, 18003  │                      │
│  └────────────────────────────────────┘                      │
└──────────────────────────────────────────────────────────────┘
```

### 핵심 포인트
- WSL2에서 Windows Host IP는 `172.20.64.1` (기본 게이트웨이)
- PX4 mavlink는 이 IP로 데이터를 전송
- QGC는 해당 포트에서 UDP 수신

---

## Part 4: 상세 실행 가이드

### 4.1 WSL2 IP 확인

```bash
# WSL2 자신의 IP
hostname -I
# 예: 172.20.65.239

# Windows Host (Gateway) IP
ip route | grep default | awk '{print $3}'
# 예: 172.20.64.1
```

### 4.2 단일 드론 테스트

```bash
cd ~/PX4-Autopilot
HEADLESS=1 PX4_SYS_AUTOSTART=4001 ./build/px4_sitl_default/bin/px4
```

PX4 콘솔에서:
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
mavlink stop-all
mavlink start -u 18001 -o 18001 -t 172.20.64.1 -r 4000000
```

QGC에서 UDP 18001로 연결 테스트

### 4.3 프로세스 정리

```bash
# 모든 PX4 프로세스 종료
pkill -9 px4
pkill -9 gz
pkill -9 ruby
```

---

## Part 5: VIM4 연결

### 5.1 구조

```
┌─────────────────┐     ┌─────────────────────────────────┐     ┌──────────────┐
│  VIM4 드론들     │     │         Windows PC              │     │    WSL2      │
│                 │     │        192.168.100.4            │     │              │
│ .30 (드론1) ────┼─UDP─┼──► :18001 ──socat──► WSL:18001 ─┼─────┼─► SITL #1    │
│ .31 (드론2) ────┼─UDP─┼──► :18002 ──socat──► WSL:18002 ─┼─────┼─► SITL #2    │
│ .32 (드론3) ────┼─UDP─┼──► :18003 ──socat──► WSL:18003 ─┼─────┼─► SITL #3    │
│                 │     │                                 │     │              │
└─────────────────┘     └─────────────────────────────────┘     └──────────────┘
```

### 5.2 Windows UDP 포트 포워딩 (socat 사용)

Windows에서 `netsh`는 TCP만 지원하므로, UDP 포워딩에는 **socat** 필요합니다.

#### 방법 1: WSL2에서 socat 실행 (권장)

WSL2 터미널에서 (PX4 실행 전에):

```bash
# socat 설치
sudo apt install -y socat

# WSL2 IP 확인
WSL_IP=$(hostname -I | awk '{print $1}')
echo "WSL2 IP: $WSL_IP"

# UDP 포워딩 시작 (백그라운드)
socat UDP4-LISTEN:18001,fork,reuseaddr UDP4:127.0.0.1:18001 &
socat UDP4-LISTEN:18002,fork,reuseaddr UDP4:127.0.0.1:18002 &
socat UDP4-LISTEN:18003,fork,reuseaddr UDP4:127.0.0.1:18003 &

echo "UDP 포워딩 시작됨"
```

#### 방법 2: Windows에서 직접 실행

1. **socat for Windows 설치**: https://github.com/tech128/socat-1.7.3.0-windows

2. **PowerShell에서 실행**:
```powershell
# WSL2 IP 확인
$wslIP = (wsl hostname -I).Trim().Split(" ")[0]
Write-Host "WSL2 IP: $wslIP"

# 각 포트에 대해 socat 실행
Start-Process socat -ArgumentList "UDP4-LISTEN:18001,fork,reuseaddr UDP4:${wslIP}:18001"
Start-Process socat -ArgumentList "UDP4-LISTEN:18002,fork,reuseaddr UDP4:${wslIP}:18002"
Start-Process socat -ArgumentList "UDP4-LISTEN:18003,fork,reuseaddr UDP4:${wslIP}:18003"
```

### 5.3 VIM4 mavlink-router 설정

각 VIM4에서 `/etc/mavlink-router/main.conf`:

```ini
[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

# SITL 연결 (Windows PC)
[UdpEndpoint SITL]
Mode = Normal
Address = 192.168.100.4
Port = 18001  # 드론2는 18002, 드론3은 18003

# 로컬 GUI
[UdpEndpoint LocalGUI]
Mode = Eavesdropping
Address = 127.0.0.1
Port = 14551
```

### 5.4 Windows 방화벽 설정

```powershell
# UDP 인바운드 허용
netsh advfirewall firewall add rule name="SITL UDP Inbound" dir=in action=allow protocol=UDP localport=18001-18003
```

---

## Part 6: 문제 해결

### 6.1 QGC 연결 안됨

**증상**: "Could Not Read Data - No Data Available!"

**해결**:
1. Windows Host IP 확인: `ip route | grep default | awk '{print $3}'`
2. mavlink 타겟 IP가 올바른지 확인
3. Windows 방화벽에서 UDP 포트 허용 확인

### 6.2 Gazebo GUI 깨짐 (WSLg)

**해결**: 헤드리스 모드 사용
```bash
HEADLESS=1 PX4_SYS_AUTOSTART=4001 ./build/px4_sitl_default/bin/px4 -i 0
```

### 6.3 mavlink 포트 충돌

**증상**: 기존 mavlink 인스턴스가 포트 점유

**해결**:
```
mavlink stop-all
mavlink start -u 18001 -o 18001 -t 172.20.64.1 -r 4000000
```

### 6.4 WSL2 재시작 후 IP 변경

WSL2는 재시작마다 IP가 변경됩니다.

**확인**:
```bash
hostname -I
ip route | grep default | awk '{print $3}'
```

mavlink 타겟 IP를 새 게이트웨이 IP로 업데이트하세요.

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

**작성일**: 2026-01-21
**버전**: 2.3
**테스트 환경**: Windows 11 + WSL2 Ubuntu 22.04 + PX4 v1.15 + QGC 4.x
