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
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 0
```

**터미널 2 (드론 2 - 좌측 10m):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-10,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 1
```

**터미널 3 (드론 3 - 우측 10m):**
```bash
cd ~/PX4-Autopilot
PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="10,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 2
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

## Part 5: VIM4 연결 (향후)

### 5.1 구조

```
VIM4 (192.168.100.30) ──UDP──► Windows PC (192.168.100.4:18001) ──► WSL2 SITL #1
VIM4 (192.168.100.31) ──UDP──► Windows PC (192.168.100.4:18002) ──► WSL2 SITL #2
VIM4 (192.168.100.32) ──UDP──► Windows PC (192.168.100.4:18003) ──► WSL2 SITL #3
```

### 5.2 Windows 포트 포워딩 (VIM4 연결 시 필요)

```powershell
# WSL2 IP 확인
$wslIP = (wsl hostname -I).Trim().Split(" ")[0]

# 포트 포워딩 설정
netsh interface portproxy add v4tov4 listenport=18001 listenaddress=0.0.0.0 connectport=18001 connectaddress=$wslIP
netsh interface portproxy add v4tov4 listenport=18002 listenaddress=0.0.0.0 connectport=18002 connectaddress=$wslIP
netsh interface portproxy add v4tov4 listenport=18003 listenaddress=0.0.0.0 connectport=18003 connectaddress=$wslIP

# 확인
netsh interface portproxy show all
```

> **주의**: 포트 포워딩은 TCP만 지원합니다. UDP는 별도 도구(socat 등) 필요.

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
**버전**: 2.1
**테스트 환경**: Windows 11 + WSL2 Ubuntu 22.04 + PX4 v1.15 + QGC 4.x
