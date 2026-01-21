# 군집 드론 시뮬레이션 환경 구축 가이드

## 빠른 시작 (설치 완료 후)

### WSL2에서 시뮬레이션 스크립트 설치

```bash
# 스크립트 다운로드 (WSL2에서 실행)
mkdir -p ~/simulation
curl -o ~/simulation/start_sitl.sh https://raw.githubusercontent.com/kiy0518/humiro_fire_suppression/main/scripts/start_sitl_simulation.sh
chmod +x ~/simulation/start_sitl.sh
```

### 시뮬레이션 실행

```bash
# 단일 드론 (로컬 QGC)
~/simulation/start_sitl.sh

# 단일 드론 + 원격 QGC (Windows IP 지정)
~/simulation/start_sitl.sh 1 192.168.100.4

# 멀티 드론 3대
~/simulation/start_sitl.sh 3

# 멀티 드론 3대 + 원격 QGC
~/simulation/start_sitl.sh 3 192.168.100.4
```

### PX4 콘솔에서 테스트

```bash
# Failsafe 비활성화 후 이륙
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
commander arm -f
commander takeoff
```

---

## 개요

VIM4 미션컴퓨터와 PX4 SITL(Software In The Loop)을 연동하여 군집 드론 시스템을 시뮬레이션합니다.

```
┌──────────────────────────────────────────────────────────┐
│              Windows PC (WSL2 Ubuntu 22.04)              │
│  ┌───────────────────────────────────────────────────┐   │
│  │           Gazebo 시뮬레이터 (3D 환경)              │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐            │   │
│  │  │ Drone 1 │  │ Drone 2 │  │ Drone 3 │            │   │
│  │  └─────────┘  └─────────┘  └─────────┘            │   │
│  └───────────────────────────────────────────────────┘   │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐                   │
│  │PX4 SITL │  │PX4 SITL │  │PX4 SITL │                   │
│  │ :14540  │  │ :14541  │  │ :14542  │                   │
│  └────┬────┘  └────┬────┘  └────┬────┘                   │
└───────┼───────────┼───────────┼──────────────────────────┘
        │ UDP       │ UDP       │ UDP
        │           │           │
┌───────┴───────────┴───────────┴────────────────────────────┐
│                    WiFi 네트워크                            │
└───────┬───────────┬───────────┬────────────────────────────┘
        │           │           │
   ┌────┴────┐ ┌────┴────┐ ┌────┴────┐
   │  VIM4   │ │  VIM4   │ │  VIM4   │
   │ 1번(L)  │ │ 2번(FL) │ │ 3번(FR) │
   │ 리더    │ │ 팔로워L │ │ 팔로워R │
   └─────────┘ └─────────┘ └─────────┘
```

---

## Part 1: Windows PC 설정 (WSL2)

### 1.1 WSL2 설치

PowerShell (관리자 권한)에서 실행:

```powershell
# WSL 설치
wsl --install

# Ubuntu 22.04 설치
wsl --install -d Ubuntu-22.04

# 재부팅 후 WSL 버전 확인
wsl --list --verbose
```

### 1.2 WSL2 GUI 지원 (WSLg)

Windows 11은 WSLg가 기본 포함되어 Gazebo GUI를 바로 사용할 수 있습니다.
Windows 10은 VcXsrv 설치가 필요합니다.

```powershell
# Windows 버전 확인
winver
```

**Windows 10인 경우:**
1. VcXsrv 다운로드: https://sourceforge.net/projects/vcxsrv/
2. XLaunch 실행 → "Disable access control" 체크
3. WSL에서 환경변수 설정:
```bash
echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk "{print \$2}"):0' >> ~/.bashrc
echo 'export LIBGL_ALWAYS_INDIRECT=0' >> ~/.bashrc
source ~/.bashrc
```

### 1.3 WSL2 네트워크 설정 (포트 포워딩)

VIM4가 WSL2 내부 SITL에 접근하려면 포트 포워딩이 필요합니다.

PowerShell (관리자 권한):

```powershell
# WSL2 IP 확인
wsl hostname -I

# 포트 포워딩 설정 (14540, 14541, 14542 - MAVLink UDP)
netsh interface portproxy add v4tov4 listenport=14540 listenaddress=0.0.0.0 connectport=14540 connectaddress=$(wsl hostname -I)
netsh interface portproxy add v4tov4 listenport=14541 listenaddress=0.0.0.0 connectport=14541 connectaddress=$(wsl hostname -I)
netsh interface portproxy add v4tov4 listenport=14542 listenaddress=0.0.0.0 connectport=14542 connectaddress=$(wsl hostname -I)

# 방화벽 규칙 추가
netsh advfirewall firewall add rule name="PX4 SITL" dir=in action=allow protocol=UDP localport=14540-14550
```

---

## Part 2: PX4 + Gazebo 설치 (WSL2 Ubuntu)

### 2.1 의존성 설치

```bash
# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 필수 패키지
sudo apt install -y \
    git \
    wget \
    cmake \
    build-essential \
    python3-pip \
    python3-venv \
    openjdk-11-jdk \
    ninja-build \
    exiftool \
    astyle \
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

# 안정 버전 체크아웃 (권장)
git checkout v1.14.3
git submodule update --init --recursive

# PX4 빌드 도구 설치
bash ./Tools/setup/ubuntu.sh
```

### 2.3 첫 빌드 테스트

```bash
cd ~/PX4-Autopilot

# 단일 드론 테스트
make px4_sitl gazebo-classic

# 성공 시 Gazebo 창에 드론이 표시됨
# 종료: Ctrl+C
```

---

## Part 3: 멀티 드론 시뮬레이션 설정

### 3.1 멀티 드론 실행 스크립트

```bash
# 스크립트 디렉토리 생성
mkdir -p ~/simulation
cd ~/simulation
```

`multi_drone_sitl.sh` 파일 생성:

```bash
#!/bin/bash
# 멀티 드론 PX4 SITL 실행 스크립트

NUM_DRONES=${1:-3}
PX4_DIR=~/PX4-Autopilot

echo "=== Starting $NUM_DRONES drones SITL simulation ==="

# 기존 프로세스 정리
pkill -9 px4 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null

sleep 2

cd $PX4_DIR

# 멀티 드론 실행
# 각 드론의 MAVLink 포트:
# Drone 1: UDP 14540 (sysid=1)
# Drone 2: UDP 14541 (sysid=2)
# Drone 3: UDP 14542 (sysid=3)

Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n $NUM_DRONES -m iris
```

실행 권한 부여:
```bash
chmod +x multi_drone_sitl.sh
```

### 3.2 드론별 위치 설정

`~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh` 수정 또는 환경변수로 초기 위치 설정:

```bash
# 편대 비행 초기 위치 (5m 간격)
# Drone 1 (리더): 0, 0
# Drone 2 (팔로워L): -5, 0
# Drone 3 (팔로워R): +5, 0

export PX4_HOME_LAT=47.397742
export PX4_HOME_LON=8.545594
export PX4_HOME_ALT=488.0
```

### 3.3 실행 및 확인

```bash
# 시뮬레이션 시작
./multi_drone_sitl.sh 3

# 별도 터미널에서 MAVLink 연결 확인
# QGroundControl 또는 mavlink-router로 연결 테스트
```

---

## Part 4: VIM4 연결 설정

### 4.1 VIM4 mavlink-router 설정

각 VIM4에서 PC의 해당 SITL 드론에 연결하도록 설정합니다.

**VIM4 1번 (리더)** - `/etc/mavlink-router/main.conf`:
```ini
[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

[UdpEndpoint PC_SITL]
Mode = Normal
Address = <WINDOWS_PC_IP>
Port = 14540

[UdpEndpoint GCS]
Mode = Normal
Address = 0.0.0.0
Port = 14550
```

**VIM4 2번 (팔로워L)** - `/etc/mavlink-router/main.conf`:
```ini
[UdpEndpoint PC_SITL]
Mode = Normal
Address = <WINDOWS_PC_IP>
Port = 14541
```

**VIM4 3번 (팔로워R)** - `/etc/mavlink-router/main.conf`:
```ini
[UdpEndpoint PC_SITL]
Mode = Normal
Address = <WINDOWS_PC_IP>
Port = 14542
```

### 4.2 VIM4 설정 스크립트

VIM4에서 실행할 SITL 연결 스크립트:

`~/humiro_fire_suppression/scripts/connect_sitl.sh`:

```bash
#!/bin/bash
# SITL 연결 스크립트

PC_IP=${1:-"192.168.0.100"}  # Windows PC IP
DRONE_ID=$(grep DRONE_ID /home/khadas/humiro_fire_suppression/config/device_config.env | cut -d= -f2)

# 포트 계산: 14540 + (DRONE_ID - 1)
SITL_PORT=$((14540 + DRONE_ID - 1))

echo "=== Connecting VIM4 (Drone $DRONE_ID) to SITL ==="
echo "PC IP: $PC_IP"
echo "SITL Port: $SITL_PORT"

# mavlink-router 설정 업데이트
sudo tee /etc/mavlink-router/main.conf > /dev/null << EOF
[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

[UdpEndpoint PC_SITL]
Mode = Normal
Address = $PC_IP
Port = $SITL_PORT

[UdpEndpoint GCS]
Mode = Normal
Address = 0.0.0.0
Port = 14550
EOF

# mavlink-router 재시작
sudo systemctl restart mavlink-router

echo "=== Connection configured ==="
systemctl status mavlink-router --no-pager | head -5
```

### 4.3 연결 테스트

```bash
# VIM4에서 실행
./scripts/connect_sitl.sh 192.168.0.100

# FC 연결 확인
curl http://localhost:5000/api/fc/status
```

---

## Part 5: 시뮬레이션 실행 순서

### 5.1 전체 실행 순서

```
1. [Windows PC] WSL2 Ubuntu 시작
2. [WSL2] 멀티 드론 SITL 실행
3. [VIM4 1,2,3] SITL 연결 스크립트 실행
4. [VIM4 1] 리더 미션 시작
5. [VIM4 2,3] 팔로워 자동 동기화
```

### 5.2 빠른 시작 명령어

**PC (WSL2):**
```bash
cd ~/simulation && ./multi_drone_sitl.sh 3
```

**VIM4 (각 기체):**
```bash
cd ~/humiro_fire_suppression && ./scripts/connect_sitl.sh <PC_IP>
```

---

## Part 6: 문제 해결

### 6.1 Gazebo가 실행되지 않음

```bash
# 그래픽 드라이버 확인
glxinfo | grep "OpenGL renderer"

# Mesa 소프트웨어 렌더링 사용 (느리지만 동작)
export LIBGL_ALWAYS_SOFTWARE=1
```

### 6.2 VIM4에서 FC 연결 안됨

```bash
# 네트워크 연결 확인
ping <PC_IP>

# UDP 포트 열림 확인
nc -vzu <PC_IP> 14540

# mavlink-router 로그 확인
journalctl -u mavlink-router -f
```

### 6.3 WSL2 IP가 바뀜

WSL2는 재시작마다 IP가 변경됩니다. 고정 IP 설정:

`.wslconfig` (Windows 사용자 폴더):
```ini
[wsl2]
networkingMode=mirrored
```

또는 매번 IP 확인:
```bash
wsl hostname -I
```

---

## 다음 단계

1. 시뮬레이션 환경 구축 완료 후
2. 리더-팔로워 코드 구현 (`swarm/` 모듈)
3. 시뮬레이션에서 편대 비행 테스트
4. 실제 기체로 이전

---

**작성일**: 2026-01-20
**버전**: 1.0
