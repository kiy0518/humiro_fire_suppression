#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# PX4-ROS2 XRCE-DDS 완전 자동 설치 스크립트
# Ubuntu 22.04 ARM64 (Khadas VIM4)용
# 
# 이 스크립트는 새로 설치한 Ubuntu VIM4에서 한 번만 실행하면
# 모든 설정이 자동으로 완료됩니다.
#
# 실행 방법:
#   sudo ./001-install_px4_ros2_complete.sh

set -e

# sudo 실행 시에도 실제 사용자 홈 디렉토리 사용
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

echo "=========================================="
echo "PX4-ROS2 XRCE-DDS 완전 자동 설치"
echo "=========================================="
echo "사용자: $REAL_USER"
echo "홈 디렉토리: $REAL_HOME"
echo ""

# 버전 파일 로드
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION_FILE="$SCRIPT_DIR/versions.env"
if [ -f "$VERSION_FILE" ]; then
    source "$VERSION_FILE"
    if [ -n "$VERSION_SAVED_DATE" ]; then
        echo "버전 파일 로드됨 (저장일: $VERSION_SAVED_DATE)"
        echo "  - micro-ROS Agent: ${MICRO_ROS_AGENT_VERSION:0:12:-최신}"
        echo "  - PX4 msgs: ${PX4_MSGS_VERSION:0:12:-최신}"
        echo ""
    fi
fi

# =============================================================================
# 색상 정의
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# =============================================================================
# 패키지 확인 및 설치 함수 (검증 포함)
# =============================================================================

# 설치 실패 카운터
INSTALL_FAILURES=0
MAX_RETRIES=3

# 패키지 설치 여부 확인 함수
check_package() {
    local pkg=$1
    if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
        local version=$(dpkg -l "$pkg" | grep "^ii" | awk '{print $3}')
        echo -e "  ${GREEN}✓${NC} $pkg: $version (설치됨)"
        return 0
    else
        echo -e "  ${RED}✗${NC} $pkg: 미설치"
        return 1
    fi
}

# 패키지 설치 및 검증 함수 (재시도 포함)
install_and_verify() {
    local pkg=$1
    local retry_count=0
    
    # 이미 설치된 경우 스킵
    if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
        local version=$(dpkg -l "$pkg" | grep "^ii" | awk '{print $3}')
        echo -e "  ${GREEN}✓${NC} $pkg: $version (이미 설치됨)"
        return 0
    fi
    
    while [ $retry_count -lt $MAX_RETRIES ]; do
        echo -e "  ${BLUE}→${NC} $pkg 설치 중... (시도 $((retry_count + 1))/$MAX_RETRIES)"
        
        # 설치 실행
        if sudo apt install -y "$pkg" > /tmp/apt_install_$$.log 2>&1; then
            # 설치 검증
            if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
                local version=$(dpkg -l "$pkg" | grep "^ii" | awk '{print $3}')
                echo -e "  ${GREEN}✓${NC} $pkg: $version (설치 완료)"
                rm -f /tmp/apt_install_$$.log
                return 0
            fi
        fi
        
        retry_count=$((retry_count + 1))
        
        if [ $retry_count -lt $MAX_RETRIES ]; then
            echo -e "  ${YELLOW}⚠${NC} $pkg 설치 실패, 재시도 중..."
            sleep 2
        fi
    done
    
    # 최대 재시도 후에도 실패
    echo -e "  ${RED}✗${NC} $pkg 설치 실패 (${MAX_RETRIES}회 시도)"
    echo "    로그: /tmp/apt_install_$$.log"
    INSTALL_FAILURES=$((INSTALL_FAILURES + 1))
    
    # 사용자에게 계속할지 물어봄
    read -p "    계속 진행하시겠습니까? (y/n): " continue_choice
    if [ "$continue_choice" != "y" ] && [ "$continue_choice" != "Y" ]; then
        echo "설치가 중단되었습니다."
        exit 1
    fi
    
    return 1
}

# 패키지 설치 함수 (미설치 시에만 설치) - 기존 호환성 유지
install_if_missing() {
    local pkg=$1
    install_and_verify "$pkg"
}

# 명령어 존재 여부 확인
check_command() {
    local cmd=$1
    if command -v "$cmd" > /dev/null 2>&1; then
        local version=$($cmd --version 2>/dev/null | head -1 || echo "버전 확인 불가")
        echo -e "  ${GREEN}✓${NC} $cmd: $version"
        return 0
    else
        echo -e "  ${RED}✗${NC} $cmd: 미설치"
        return 1
    fi
}

# Git 저장소 클론 및 검증
clone_and_verify() {
    local repo_url=$1
    local target_dir=$2
    local branch=${3:-""}
    local version=${4:-""}
    local retry_count=0
    
    while [ $retry_count -lt $MAX_RETRIES ]; do
        echo -e "  ${BLUE}→${NC} 저장소 클론 중... (시도 $((retry_count + 1))/$MAX_RETRIES)"
        
        # 기존 디렉토리 제거
        rm -rf "$target_dir"
        
        # 클론 실행
        local clone_cmd="git clone"
        [ -n "$branch" ] && clone_cmd="$clone_cmd -b $branch"
        
        if $clone_cmd "$repo_url" "$target_dir" 2>/tmp/git_clone_$$.log; then
            # 버전 체크아웃
            if [ -n "$version" ]; then
                cd "$target_dir"
                if git checkout "$version" 2>/dev/null; then
                    echo -e "  ${GREEN}✓${NC} 저장소 클론 완료 (버전: $version)"
                else
                    echo -e "  ${YELLOW}⚠${NC} 버전 체크아웃 실패, 기본 브랜치 사용"
                fi
                cd - > /dev/null
            else
                echo -e "  ${GREEN}✓${NC} 저장소 클론 완료"
            fi
            
            # 검증
            if [ -d "$target_dir/.git" ]; then
                rm -f /tmp/git_clone_$$.log
                return 0
            fi
        fi
        
        retry_count=$((retry_count + 1))
        
        if [ $retry_count -lt $MAX_RETRIES ]; then
            echo -e "  ${YELLOW}⚠${NC} 클론 실패, 재시도 중..."
            sleep 3
        fi
    done
    
    echo -e "  ${RED}✗${NC} 저장소 클론 실패"
    cat /tmp/git_clone_$$.log 2>/dev/null
    return 1
}

# 빌드 검증 함수
verify_build() {
    local name=$1
    local check_file=$2
    
    if [ -f "$check_file" ]; then
        echo -e "  ${GREEN}✓${NC} $name 빌드 검증 완료"
        return 0
    else
        echo -e "  ${RED}✗${NC} $name 빌드 검증 실패"
        echo "    예상 파일: $check_file"
        return 1
    fi
}

# 서비스 검증 함수
verify_service() {
    local service_name=$1
    local max_wait=10
    local wait_count=0
    
    echo -e "  ${BLUE}→${NC} $service_name 서비스 검증 중..."
    
    # 서비스 시작 대기
    while [ $wait_count -lt $max_wait ]; do
        if systemctl is-active --quiet "$service_name"; then
            echo -e "  ${GREEN}✓${NC} $service_name 서비스 실행 중"
            return 0
        fi
        sleep 1
        wait_count=$((wait_count + 1))
    done
    
    echo -e "  ${YELLOW}⚠${NC} $service_name 서비스가 아직 시작되지 않음 (재부팅 후 시작됨)"
    return 1
}

# 아키텍처 확인
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo "⚠ 경고: ARM64 아키텍처가 아닙니다 ($ARCH)"
    read -p "계속하시겠습니까? (y/n): " CONTINUE
    if [ "$CONTINUE" != "y" ]; then
        exit 1
    fi
fi

# 1. ROS2 Humble 설치
echo "=========================================="
echo "1단계: ROS2 Humble 설치"
echo "=========================================="

if command -v ros2 > /dev/null 2>&1; then
    echo "✓ ROS2가 이미 설치되어 있습니다"
else
    echo "ROS2 Humble 설치 중..."
    
    # 시스템 업데이트
    sudo apt update
    sudo apt install -y software-properties-common curl gnupg lsb-release
    
    # ROS2 저장소 추가
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /tmp/ros.asc
    sudo gpg --dearmor < /tmp/ros.asc | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
    sudo rm -f /tmp/ros.asc
    
    ARCH=$(dpkg --print-architecture)
    CODENAME=$(lsb_release -cs)
    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
    
    # ROS2 Humble 설치
    sudo apt update
    sudo apt install -y ros-humble-desktop
    
    echo "✓ ROS2 Humble 설치 완료"
fi

# 2. 필수 패키지 설치
echo ""
echo "=========================================="
echo "2단계: 필수 패키지 설치"
echo "=========================================="

# 필수 패키지 목록
REQUIRED_PACKAGES=(
    "python3-pip"
    "python3-colcon-common-extensions"
    "git"
    "build-essential"
    "cmake"
    "python3-rosdep"
    "python3-vcstool"
    "ros-humble-rmw-fastrtps-cpp"
    "dnsmasq"
    "netplan.io"
)

echo "패키지 상태 확인 중..."
echo ""

# 미설치 패키지 확인
MISSING_PACKAGES=()
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ! check_package "$pkg"; then
        MISSING_PACKAGES+=("$pkg")
    fi
done

echo ""

# 미설치 패키지가 있으면 설치
if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo "미설치 패키지 설치 중: ${MISSING_PACKAGES[*]}"
    sudo apt update
    sudo apt install -y "${MISSING_PACKAGES[@]}"
    echo ""
    echo "설치 결과 확인:"
    for pkg in "${MISSING_PACKAGES[@]}"; do
        check_package "$pkg"
    done
else
    echo "✓ 모든 필수 패키지가 이미 설치되어 있습니다"
fi

# rosdep 초기화
echo ""
echo "rosdep 초기화 중..."
sudo rosdep init 2>/dev/null || echo "  (이미 초기화됨)"
rosdep update --rosdistro humble

echo ""
echo "✓ 필수 패키지 설치 완료"

# 3. Micro-ROS Agent 빌드
echo ""
echo "=========================================="
echo "3단계: Micro-ROS Agent 빌드"
echo "=========================================="

# Micro-ROS Agent 빌드 디렉토리 확인 및 정리
if [ -d $REAL_HOME/workspaces/micro_ros_ws ]; then
    echo "기존 빌드 디렉토리 확인 중..."
    # 빌드 디렉토리가 있지만 setup.bash가 없거나 손상된 경우 재빌드
    if [ ! -f $REAL_HOME/workspaces/micro_ros_ws/install/setup.bash ]; then
        echo "빌드 디렉토리 정리 중 (재빌드 필요)..."
        cd $REAL_HOME/workspaces/micro_ros_ws
        rm -rf build install log
    fi
fi

if [ -f $REAL_HOME/workspaces/micro_ros_ws/install/setup.bash ] && [ -f $REAL_HOME/workspaces/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent ]; then
    echo "✓ Micro-ROS Agent가 이미 빌드되어 있습니다"
else
    echo "Micro-ROS Agent 빌드 중..."
    
    mkdir -p $REAL_HOME/workspaces/micro_ros_ws/src
    cd $REAL_HOME/workspaces/micro_ros_ws/src
    
    if [ ! -d "micro-ROS-Agent" ]; then
        echo "Micro-ROS-Agent 저장소 클론 중..."
        git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
        
        # 버전 파일에 지정된 버전으로 체크아웃
        if [ -n "$MICRO_ROS_AGENT_VERSION" ]; then
            echo "  지정된 버전으로 체크아웃: $MICRO_ROS_AGENT_VERSION"
            cd micro-ROS-Agent
            git checkout "$MICRO_ROS_AGENT_VERSION" || echo "  ⚠ 버전 체크아웃 실패, 최신 버전 사용"
            cd ..
        fi
    fi
    
    cd $REAL_HOME/workspaces/micro_ros_ws
    
    # 기존 빌드 디렉토리 정리 (경로 문제 방지)
    if [ -d build ] || [ -d install ] || [ -d log ]; then
        echo "기존 빌드 디렉토리 정리 중..."
        rm -rf build install log
    fi
    
    source /opt/ros/humble/setup.bash
    
    # 의존성 설치
    echo "의존성 패키지 설치 중..."
    sudo apt install -y \
        python3-rosinstall-generator \
        python3-wstool \
        libasio-dev \
        libtinyxml2-dev \
        libcunit1-dev \
        ros-humble-micro-ros-msgs
    
    echo "rosdep 의존성 설치 중..."
    rosdep install --from-paths src --ignore-src -y || echo "일부 의존성 설치 실패 (계속 진행)"
    
    # 빌드 (의존성 포함하여 전체 빌드)
    echo "Micro-ROS Agent 빌드 중 (시간이 걸릴 수 있습니다)..."
    colcon build
    
    # 빌드 결과 확인
    if [ -f $REAL_HOME/workspaces/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent ]; then
        echo "✓ Micro-ROS Agent 빌드 완료"
    else
        echo "✗ Micro-ROS Agent 빌드 실패"
        echo "로그 확인: $REAL_HOME/workspaces/micro_ros_ws/log/latest_build/micro_ros_agent"
        exit 1
    fi
fi

# 4. PX4 ROS2 메시지 패키지 빌드
echo ""
echo "=========================================="
echo "4단계: PX4 ROS2 메시지 패키지 빌드"
echo "=========================================="

if [ -f $REAL_HOME/workspaces/px4_ros2_ws/install/setup.bash ]; then
    echo "✓ PX4 ROS2 메시지 패키지가 이미 빌드되어 있습니다"
else
    echo "PX4 ROS2 메시지 패키지 빌드 중..."
    
    mkdir -p $REAL_HOME/workspaces/px4_ros2_ws/src
    cd $REAL_HOME/workspaces/px4_ros2_ws/src
    
    if [ ! -d "px4_msgs" ]; then
        git clone https://github.com/PX4/px4_msgs.git
        cd px4_msgs
        
        # 버전 파일에 지정된 버전으로 체크아웃
        if [ -n "$PX4_MSGS_VERSION" ]; then
            echo "  지정된 버전으로 체크아웃: $PX4_MSGS_VERSION"
            git checkout "$PX4_MSGS_VERSION" || echo "  ⚠ 버전 체크아웃 실패, main 브랜치 사용"
        else
            git checkout main
        fi
        cd ..
    fi
    
    cd $REAL_HOME/workspaces/px4_ros2_ws
    source /opt/ros/humble/setup.bash
    
    rosdep install --from-paths src --ignore-src -r -y || echo "일부 의존성 설치 실패 (계속 진행)"
    
    # 빌드
    colcon build
    
    echo "✓ PX4 ROS2 메시지 패키지 빌드 완료"
fi

# 5. 네트워크 설정 (이더넷 직접 연결)
echo ""
echo "=========================================="
echo "5단계: 네트워크 설정 (PX4-SBC 직접 연결)"
echo "=========================================="

# netplan 설치 확인
echo "네트워크 도구 확인 중..."
if ! command -v netplan > /dev/null 2>&1; then
    echo "  netplan 미설치, 설치 중..."
    sudo apt update
    sudo apt install -y netplan.io
fi
check_command "netplan"

# Netplan을 사용한 영구 네트워크 설정
echo ""
echo "eth0 영구 네트워크 설정 중 (Netplan)..."

# device_config.env에서 네트워크 설정 로드
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/device_config.env"

if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    echo "  ✓ device_config.env 로드됨"
    echo "  - DRONE_ID: $DRONE_ID"
    echo "  - ETH0_IP: $ETH0_IP"
    echo "  - FC_IP: $FC_IP"
    
    # 필수 변수 확인
    if [ -z "$ETH0_IP" ] || [ -z "$FC_IP" ] || [ -z "$DRONE_ID" ]; then
        echo "  ✗ ERROR: device_config.env에 필수 변수가 없습니다."
        echo "    필수 변수: ETH0_IP, FC_IP, DRONE_ID"
        exit 1
    fi
else
    echo "  ✗ ERROR: device_config.env 파일이 없습니다."
    echo "    먼저 device_config.env 파일을 생성하고 설정하세요."
    echo "    예시: cp device_config.env.example device_config.env"
    exit 1
fi

# FC에 할당할 DHCP 범위 계산 (FC_IP 기반)
DHCP_START="$FC_IP"
DHCP_END="$FC_IP"

# netplan 디렉토리 생성
sudo mkdir -p /etc/netplan

# 기존 netplan 설정 백업
if [ -f /etc/netplan/01-netcfg.yaml ]; then
    sudo cp /etc/netplan/01-netcfg.yaml /etc/netplan/01-netcfg.yaml.backup.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
fi

# eth0 고정 IP 설정하는 netplan 파일 생성
sudo tee /etc/netplan/01-eth0-px4.yaml > /dev/null << NETPLAN_EOF
# Drone #$DRONE_ID eth0 네트워크 설정
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - $ETH0_IP/24
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
NETPLAN_EOF

# 기존 WiFi 설정이 networkd를 사용하는 경우 NetworkManager로 변경
if [ -f /etc/netplan/02-wlan0-wifi.yaml ]; then
    if grep -q "renderer: networkd" /etc/netplan/02-wlan0-wifi.yaml 2>/dev/null; then
        echo "  기존 WiFi 설정을 NetworkManager로 업데이트..."
        sudo sed -i 's/renderer: networkd/renderer: NetworkManager/g' /etc/netplan/02-wlan0-wifi.yaml
    fi
fi

# netplan 설정 적용 (eth0만 적용, WiFi 설정은 보존)
echo "Netplan 설정 적용 중..."

# eth0 인터페이스만 직접 설정 (netplan apply 대신 ip 명령어 사용)
# 기존 IP 제거
sudo ip addr flush dev eth0 2>/dev/null || true
# 새 IP 설정
sudo ip addr add $ETH0_IP/24 dev eth0 2>/dev/null || true
sudo ip link set eth0 up 2>/dev/null || true

# netplan generate만 실행 (apply는 재부팅 시 자동 적용)
sudo netplan generate 2>/dev/null || echo "  ⚠ netplan generate 경고 (무시 가능)"

sleep 1

# eth0 IP 확인
CURRENT_ETH0_IP=$(ip addr show eth0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)

if [ "$CURRENT_ETH0_IP" = "$ETH0_IP" ]; then
    echo "✓ eth0 IP 설정 완료: $ETH0_IP"
    echo "  (재부팅 후에도 netplan으로 자동 적용됨)"
else
    echo "⚠ eth0 IP 설정 확인 실패 (현재: $CURRENT_ETH0_IP, 기대: $ETH0_IP)"
    echo "  재부팅 후 자동 적용됩니다"
fi

# dnsmasq 설정
echo "dnsmasq DHCP 서버 설정 중..."

# dnsmasq 설정 디렉토리 생성
sudo mkdir -p /etc/dnsmasq.d

# 기존 설정 백업
sudo cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true

# dnsmasq 기본 설정 (PX4 전용 설정 파일 사용)
sudo tee /etc/dnsmasq.conf > /dev/null << EOF
# PX4-SBC 직접 연결용 DHCP 서버 설정
# 추가 설정은 /etc/dnsmasq.d/px4.conf에서 관리
conf-dir=/etc/dnsmasq.d/,*.conf
EOF

# PX4 전용 설정 파일 (FC에 고정 IP 할당)
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
# Drone #$DRONE_ID DHCP 서버 설정
interface=eth0
bind-interfaces
dhcp-range=$DHCP_START,$DHCP_END,255.255.255.0,12h
dhcp-option=3,$ETH0_IP
dhcp-option=6,8.8.8.8
log-dhcp
EOF

# dnsmasq systemd 서비스 파일 생성
sudo tee /etc/systemd/system/dnsmasq-px4.service > /dev/null << 'DNSMASQ_SERVICE_EOF'
[Unit]
Description=dnsmasq DHCP server for PX4
After=network.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/sbin/dnsmasq --conf-file=/etc/dnsmasq.d/px4.conf --no-daemon
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
DNSMASQ_SERVICE_EOF

# 기존 dnsmasq 프로세스 종료
sudo pkill -TERM dnsmasq 2>/dev/null || true
sleep 1

# systemd 데몬 재로드
sudo systemctl daemon-reload

# dnsmasq 서비스 활성화 및 시작
echo "dnsmasq 서비스 활성화 및 시작 중..."
sudo systemctl enable dnsmasq-px4.service
sudo systemctl restart dnsmasq-px4.service

sleep 2

if systemctl is-active --quiet dnsmasq-px4.service; then
    echo "✓ dnsmasq DHCP 서버 실행 중 (부팅 시 자동 시작됨)"
else
    echo "⚠ dnsmasq 시작 실패"
    echo "상태 확인: sudo systemctl status dnsmasq-px4"
    echo "로그 확인: sudo journalctl -u dnsmasq-px4 -n 50"
fi

# 6. 환경 설정
echo ""
echo "=========================================="
echo "6단계: 환경 설정"
echo "=========================================="

# .bashrc에 추가 (중복 방지)
if ! grep -q "PX4 ROS2" $REAL_HOME/.bashrc; then
    cat >> $REAL_HOME/.bashrc << 'BASHRC_EOF'

# ==========================================
# PX4 ROS2 XRCE-DDS 환경 설정
# ==========================================

# ROS2 Humble 설정
source /opt/ros/humble/setup.bash

# Micro-ROS Agent 작업 공간 설정
if [ -f $REAL_HOME/workspaces/micro_ros_ws/install/setup.bash ]; then
    source $REAL_HOME/workspaces/micro_ros_ws/install/setup.bash
fi

# PX4 ROS2 작업 공간 설정
if [ -f $REAL_HOME/workspaces/px4_ros2_ws/install/setup.bash ]; then
    source $REAL_HOME/workspaces/px4_ros2_ws/install/setup.bash
fi

# ROS2 도메인 ID 설정 (기본값: 1, 단일 기체 사용 시)
# 군집 비행 시 각 기체별로 1, 2, 3으로 설정 필요
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# RMW 구현체 설정 (ARM64에서는 FastRTPS 사용)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Micro-ROS Agent 실행 함수
micro-ros-agent() {
    if [ -f $REAL_HOME/workspaces/micro_ros_ws/install/setup.bash ]; then
        source $REAL_HOME/workspaces/micro_ros_ws/install/setup.bash
        ros2 run micro_ros_agent micro_ros_agent "$@"
    else
        echo "Micro-ROS Agent가 설치되지 않았습니다."
    fi
}
BASHRC_EOF
    echo "✓ 환경 설정 추가 완료"
else
    echo "✓ 환경 설정 이미 추가됨"
fi

# 7. Micro-ROS Agent systemd 서비스 설정
echo ""
echo "=========================================="
echo "7단계: Micro-ROS Agent systemd 서비스 설정"
echo "=========================================="

# 실제 사용자 정보 사용 (sudo 실행 시에도 올바른 경로)
CURRENT_USER="$REAL_USER"
CURRENT_HOME="$REAL_HOME"

# Micro-ROS Agent 실행 스크립트 생성
MICRO_ROS_SCRIPT="/usr/local/bin/micro-ros-agent-service.sh"
sudo tee "$MICRO_ROS_SCRIPT" > /dev/null << SCRIPT_EOF
#!/bin/bash
# Micro-ROS Agent 실행 스크립트

export HOME=$CURRENT_HOME
# ROS_DOMAIN_ID는 systemd 서비스에서 설정됨 (기본값: 1, 단일 기체 사용 시)
# 군집 비행 시 각 기체별로 1, 2, 3으로 설정 필요
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}  # 기본값 1 (단일 기체)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source Micro-ROS Agent workspace (필수!)
if [ -f $CURRENT_HOME/workspaces/micro_ros_ws/install/setup.bash ]; then
    source $CURRENT_HOME/workspaces/micro_ros_ws/install/setup.bash
fi

# Source PX4 ROS2 messages workspace
if [ -f $CURRENT_HOME/workspaces/px4_ros2_ws/install/setup.bash ]; then
    source $CURRENT_HOME/workspaces/px4_ros2_ws/install/setup.bash
fi

# Set library path for Micro-ROS Agent (모든 필요한 경로 포함)
export LD_LIBRARY_PATH=$CURRENT_HOME/workspaces/micro_ros_ws/install/micro_ros_agent/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:\${LD_LIBRARY_PATH}

# Start Micro-ROS Agent directly
exec $CURRENT_HOME/workspaces/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent udp4 --port 8888
SCRIPT_EOF

sudo chmod +x "$MICRO_ROS_SCRIPT"

# systemd 서비스 파일 생성
sudo tee /etc/systemd/system/micro-ros-agent.service > /dev/null << SERVICE_EOF
[Unit]
Description=Micro-ROS Agent for PX4 XRCE-DDS
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER
WorkingDirectory=$CURRENT_HOME
Environment="HOME=$CURRENT_HOME"
ExecStart=/usr/local/bin/micro-ros-agent-service.sh
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SERVICE_EOF

# systemd 데몬 재로드
sudo systemctl daemon-reload

# 기존 프로세스 종료 (있다면)
if pgrep -f "micro_ros_agent" > /dev/null; then
    echo "기존 Micro-ROS Agent 프로세스 종료 중..."
    pkill -f "micro_ros_agent" || true
    sleep 2
fi

# 서비스 활성화 및 시작
echo "Micro-ROS Agent 서비스 활성화 및 시작 중..."
sudo systemctl enable micro-ros-agent.service
sudo systemctl restart micro-ros-agent.service

sleep 3

# 서비스 상태 확인
sleep 3
if systemctl is-active --quiet micro-ros-agent.service; then
    echo "✓ Micro-ROS Agent 서비스 실행 중 (포트 8888)"
    echo "  - 부팅 시 자동 시작됨"
    echo "  - 로그 확인: sudo journalctl -u micro-ros-agent -f"
    
    # 포트 확인
    sleep 1
    if ss -ulnp 2>/dev/null | grep -q ":8888" || netstat -ulnp 2>/dev/null | grep -q ":8888"; then
        echo "  - 포트 8888 리스닝 확인됨"
    else
        echo "  ⚠ 포트 8888이 아직 열리지 않았을 수 있습니다 (잠시 후 확인)"
    fi
else
    echo "⚠ Micro-ROS Agent 서비스 시작 실패"
    echo "상태 확인: sudo systemctl status micro-ros-agent"
    echo "로그 확인: sudo journalctl -u micro-ros-agent -n 50"
    echo ""
    echo "수동 진단:"
    echo "  1. 실행 파일 확인: ls -la $CURRENT_HOME/workspaces/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
    echo "  2. 수동 실행 테스트: sudo -u $CURRENT_USER $MICRO_ROS_SCRIPT"
fi

# 8. 설치 검증
echo ""
echo "=========================================="
echo "8단계: 설치 검증"
echo "=========================================="

VERIFICATION_FAILED=0

# ROS2 확인
if command -v ros2 > /dev/null 2>&1; then
    echo "✓ ROS2 설치 확인됨"
else
    echo "✗ ROS2 설치 확인 실패"
    VERIFICATION_FAILED=1
fi

# Micro-ROS Agent 실행 파일 확인
if [ -f $REAL_HOME/workspaces/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent ]; then
    echo "✓ Micro-ROS Agent 실행 파일 확인됨"
else
    echo "✗ Micro-ROS Agent 실행 파일 없음"
    VERIFICATION_FAILED=1
fi

# PX4 메시지 패키지 확인
if [ -f $REAL_HOME/workspaces/px4_ros2_ws/install/setup.bash ]; then
    echo "✓ PX4 ROS2 메시지 패키지 확인됨"
else
    echo "✗ PX4 ROS2 메시지 패키지 없음"
    VERIFICATION_FAILED=1
fi

# eth0 IP 확인 (device_config.env 값과 비교)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPECTED_ETH0_IP=""
if [ -f "$SCRIPT_DIR/device_config.env" ]; then
    source "$SCRIPT_DIR/device_config.env"
    EXPECTED_ETH0_IP="$ETH0_IP"
fi
if [ -z "$EXPECTED_ETH0_IP" ]; then
    echo "⚠ 경고: device_config.env에서 ETH0_IP를 읽을 수 없습니다."
    EXPECTED_ETH0_IP="(확인 필요)"
fi
CURRENT_ETH0_IP=$(ip addr show eth0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
if [ "$CURRENT_ETH0_IP" = "$EXPECTED_ETH0_IP" ]; then
    echo "✓ eth0 IP 설정 확인됨 ($EXPECTED_ETH0_IP)"
else
    echo "⚠ eth0 IP 설정 확인 실패 (현재: $CURRENT_ETH0_IP, 기대: $EXPECTED_ETH0_IP)"
    echo "  재부팅 후 자동으로 설정됩니다"
fi

# 서비스 상태 확인
if systemctl is-enabled --quiet dnsmasq-px4.service 2>/dev/null; then
    echo "✓ dnsmasq 서비스 활성화됨"
else
    echo "⚠ dnsmasq 서비스 활성화 확인 실패"
fi

if systemctl is-enabled --quiet micro-ros-agent.service 2>/dev/null; then
    echo "✓ micro-ros-agent 서비스 활성화됨"
else
    echo "⚠ micro-ros-agent 서비스 활성화 확인 실패"
fi

# 9. 설치 완료 및 요약
echo ""
echo "=========================================="
echo "기본 설치 완료!"
echo "=========================================="
echo ""

if [ $VERIFICATION_FAILED -eq 0 ]; then
    echo "✓ 모든 기본 구성요소가 정상적으로 설치되었습니다!"
else
    echo "⚠ 일부 구성요소 설치에 문제가 있을 수 있습니다"
    echo "  위의 검증 결과를 확인하세요"
fi

echo ""
echo "설치된 구성요소:"
echo "  ✓ ROS2 Humble"
echo "  ✓ Micro-ROS Agent (XRCE-DDS 브릿지)"
echo "  ✓ PX4 ROS2 메시지 패키지"
echo "  ✓ DHCP 서버 (dnsmasq)"
echo "  ✓ 기본 네트워크 설정"
echo ""
echo "=========================================="
echo "군집 드론 시스템 완성을 위한 추가 설정"
echo "=========================================="
echo ""
echo "┌─────────────────────────────────────────────────────────┐"
echo "│  단계  │  스크립트                    │  설명           │"
echo "├─────────────────────────────────────────────────────────┤"
echo "│   1    │  001-install (완료)          │  ROS2/Agent     │"
echo "│   2    │  002-install_mavlink_router  │  MAVLink 라우팅 │"
echo "│   3    │  003-apply_config            │  설정 적용      │"
echo "│   4    │  004_setup_video_streaming   │  영상 (선택)    │"
echo "│   5    │  101_check_px4_connection    │  연결 확인      │"
echo "└─────────────────────────────────────────────────────────┘"
echo ""
echo "다음 단계 (순서대로 실행):"
echo ""
echo "  1. mavlink-router 설치 (QGC 연결에 필수):"
echo "     ./002-install_mavlink_router.sh"
echo ""
echo "  2. device_config.env 수정 후 설정 적용:"
echo "     nano device_config.env"
echo "     sudo ./003-apply_config.sh"
echo ""
echo "  3. 영상 스트리밍 설정 (선택사항):"
echo "     ./004_setup_video_streaming.sh"
echo ""
echo "  4. 연결 확인:"
echo "     ./101_check_px4_connection.sh"
echo ""
echo "PX4 파라미터 설정 (QGroundControl):"
echo "  ┌─────────────────────────────────────────┐"
echo "  │  UXRCE_DDS_AG_IP = 드론별 MC IP (int32) │"
echo "  │  UXRCE_DDS_PRT = 8888                   │"
echo "  │  UXRCE_DDS_DOM_ID = 0                   │"
echo "  │  UXRCE_DDS_CFG = 1000 (Ethernet)        │"
echo "  │  MAV_SYS_ID = 드론번호 (1/2/3)          │"
echo "  └─────────────────────────────────────────┘"
echo ""
echo "IP 주소 변환 (드론별):"
echo "  Drone #1: ./102_ip_to_decimal.sh 10.0.0.11  # MC IP"
echo "  Drone #2: ./102_ip_to_decimal.sh 10.0.0.21"
echo "  Drone #3: ./102_ip_to_decimal.sh 10.0.0.31"
echo ""
echo "유용한 명령:"
echo "  ./101_check_px4_connection.sh              # 연결 상태 종합 확인"
echo "  ros2 topic list                            # ROS2 토픽 목록"
echo "  sudo systemctl status micro-ros-agent      # XRCE-DDS Agent 상태"
echo "  sudo systemctl status mavlink-router       # MAVLink 라우터 상태"
echo "  sudo journalctl -u mavlink-router -f       # MAVLink 로그"
echo ""
echo "문제 해결:"
echo "  자세한 내용은 README.md의 '문제 해결' 섹션을 참고하세요"
echo ""

