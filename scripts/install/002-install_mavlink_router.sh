#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# mavlink-router 설치 스크립트
# PX4 MAVLink를 다중 목적지로 라우팅 (GCS, ROS2, 로그)
# 
# 사용법:
#   sudo ./002-install_mavlink_router.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# sudo 실행 시에도 실제 사용자 홈 디렉토리 사용
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

echo "=========================================="
echo "mavlink-router 설치 및 설정"
echo "=========================================="
echo ""

# 버전 파일 로드
VERSION_FILE="$SCRIPT_DIR/versions.env"
if [ -f "$VERSION_FILE" ]; then
    source "$VERSION_FILE"
    if [ -n "$VERSION_SAVED_DATE" ] && [ -n "$MAVLINK_ROUTER_VERSION" ]; then
        echo "버전 파일 로드됨 (저장일: $VERSION_SAVED_DATE)"
        echo "  - mavlink-router: ${MAVLINK_ROUTER_VERSION:0:12}"
        echo ""
    fi
fi

# 아키텍처 확인
ARCH=$(uname -m)
echo "시스템 아키텍처: $ARCH"

# =============================================================================
# 색상 정의
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 설치 실패 카운터
INSTALL_FAILURES=0
MAX_RETRIES=3

# 패키지 확인 함수
check_package() {
    local pkg=$1
    if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
        local version=$(dpkg -l "$pkg" | grep "^ii" | awk '{print $3}')
        echo -e "  ${GREEN}✓${NC} $pkg: $version"
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
        
        if sudo apt install -y "$pkg" > /tmp/apt_install_$$.log 2>&1; then
            if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
                local version=$(dpkg -l "$pkg" | grep "^ii" | awk '{print $3}')
                echo -e "  ${GREEN}✓${NC} $pkg: $version (설치 완료)"
                rm -f /tmp/apt_install_$$.log
                return 0
            fi
        fi
        
        retry_count=$((retry_count + 1))
        [ $retry_count -lt $MAX_RETRIES ] && sleep 2
    done
    
    echo -e "  ${RED}✗${NC} $pkg 설치 실패"
    INSTALL_FAILURES=$((INSTALL_FAILURES + 1))
    return 1
}

# 빌드 검증 함수
verify_build() {
    local name=$1
    local check_cmd=$2
    
    if $check_cmd > /dev/null 2>&1; then
        echo -e "  ${GREEN}✓${NC} $name 빌드 검증 완료"
        return 0
    else
        echo -e "  ${RED}✗${NC} $name 빌드 검증 실패"
        return 1
    fi
}

# 1. 의존성 패키지 설치
echo ""
echo "1. 의존성 패키지 확인 및 설치..."

REQUIRED_PACKAGES=(
    "git"
    "meson"
    "ninja-build"
    "pkg-config"
    "gcc"
    "g++"
    "systemd"
    "python3-pip"
)

echo "패키지 상태 확인 중..."
MISSING_PACKAGES=()
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ! check_package "$pkg"; then
        MISSING_PACKAGES+=("$pkg")
    fi
done

echo ""
if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo "미설치 패키지 설치 중..."
    sudo apt update
    
    for pkg in "${MISSING_PACKAGES[@]}"; do
        install_and_verify "$pkg"
    done
    
    # 설치 결과 확인
    echo ""
    echo "설치 결과 검증 중..."
    VERIFY_FAILED=0
    for pkg in "${REQUIRED_PACKAGES[@]}"; do
        if ! check_package "$pkg" > /dev/null 2>&1; then
            VERIFY_FAILED=1
        fi
    done
    
    if [ $VERIFY_FAILED -eq 0 ]; then
        echo -e "${GREEN}✓${NC} 모든 의존성 패키지 설치 완료"
    else
        echo -e "${YELLOW}⚠${NC} 일부 패키지 설치 실패"
    fi
else
    echo -e "${GREEN}✓${NC} 모든 의존성 패키지가 이미 설치되어 있습니다"
fi

# 2. mavlink-router 소스 다운로드 및 빌드
echo ""
echo "2. mavlink-router 소스 다운로드 및 빌드 중..."

MAVLINK_ROUTER_DIR=$REAL_HOME/workspaces/mavlink-router

if [ -d "$MAVLINK_ROUTER_DIR" ]; then
    echo "   기존 디렉토리 업데이트 중..."
    cd "$MAVLINK_ROUTER_DIR"
    git fetch --all || true
else
    echo "   저장소 클론 중..."
    git clone https://github.com/mavlink-router/mavlink-router.git "$MAVLINK_ROUTER_DIR"
    cd "$MAVLINK_ROUTER_DIR"
fi

# 버전 파일에 지정된 버전으로 체크아웃
if [ -n "$MAVLINK_ROUTER_VERSION" ]; then
    echo "   지정된 버전으로 체크아웃: $MAVLINK_ROUTER_VERSION"
    git checkout "$MAVLINK_ROUTER_VERSION" || echo "   ⚠ 버전 체크아웃 실패, 최신 버전 사용"
fi

# 서브모듈 초기화
git submodule update --init --recursive

# 빌드 디렉토리 생성 및 빌드
echo -e "   ${BLUE}→${NC} 빌드 중... (시간이 걸릴 수 있습니다)"
rm -rf build

if meson setup build --buildtype=release > /tmp/meson_$$.log 2>&1; then
    echo -e "   ${GREEN}✓${NC} meson 설정 완료"
else
    echo -e "   ${RED}✗${NC} meson 설정 실패"
    cat /tmp/meson_$$.log
    exit 1
fi

if ninja -C build > /tmp/ninja_$$.log 2>&1; then
    echo -e "   ${GREEN}✓${NC} 빌드 완료"
else
    echo -e "   ${RED}✗${NC} 빌드 실패"
    tail -50 /tmp/ninja_$$.log
    exit 1
fi

# 설치
echo -e "   ${BLUE}→${NC} 설치 중..."
if sudo ninja -C build install > /tmp/ninja_install_$$.log 2>&1; then
    echo -e "   ${GREEN}✓${NC} 설치 완료"
else
    echo -e "   ${RED}✗${NC} 설치 실패"
    cat /tmp/ninja_install_$$.log
    exit 1
fi

# 빌드 검증
echo ""
echo "빌드 검증 중..."
if command -v mavlink-routerd > /dev/null 2>&1; then
    MAVLINK_VER=$(mavlink-routerd --version 2>/dev/null | head -1 || echo "버전 확인 불가")
    echo -e "   ${GREEN}✓${NC} mavlink-routerd 설치 검증 완료: $MAVLINK_VER"
else
    echo -e "   ${RED}✗${NC} mavlink-routerd 실행 파일을 찾을 수 없습니다"
    exit 1
fi

# 임시 파일 정리
rm -f /tmp/meson_$$.log /tmp/ninja_$$.log /tmp/ninja_install_$$.log

# 3. 설정 디렉토리 생성
echo ""
echo "3. 설정 디렉토리 생성 중..."
sudo mkdir -p /etc/mavlink-router

echo "   ✓ 설정 디렉토리 생성 완료"

# 4. 설정 파일 로드 및 기본 설정 생성
echo ""
echo "4. device_config.env 설정 로드 중..."

CONFIG_FILE="$DEVICE_CONFIG"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: $CONFIG_FILE 파일이 없습니다."
    echo "먼저 device_config.env 파일을 설정하세요."
    exit 1
fi

source "$CONFIG_FILE"

# WiFi 서브넷 기반 브로드캐스트 주소 자동 계산
# 예: 192.168.100.31 → 192.168.100.255
WIFI_BROADCAST="${WIFI_IP%.*}.255"

echo ""
echo "┌─ 기체별 수정 항목 ─────────────────────────┐"
echo "  드론 번호: #$DRONE_ID"
echo "  eth0 IP: $ETH0_IP"
echo "  WiFi IP: $WIFI_IP"
echo "└────────────────────────────────────────────┘"
echo ""
echo "┌─ 자동 설정 (수정 불필요) ──────────────────┐"
echo "  FC IP: $FC_IP (DHCP 고정)"
echo "  QGC: 브로드캐스트 ($WIFI_BROADCAST:$QGC_UDP_PORT)"
echo "└────────────────────────────────────────────┘"
echo ""

# mavlink-router 설정 파일 생성
sudo tee /etc/mavlink-router/main.conf > /dev/null << EOF
# mavlink-router 설정 파일
# Drone #$DRONE_ID 설정
# 
# FC (PX4) <-> mavlink-router <-> 다중 목적지
#   - GCS (QGroundControl) via WiFi
#   - ROS2 노드 via localhost
#   - 로그 파일

[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

# ============================================
# FC (PX4) 연결 - Ethernet UDP
# ============================================
[UdpEndpoint FC]
Mode = Normal
Address = $FC_IP
Port = $FC_MAVLINK_PORT

# ============================================
# GCS (QGroundControl) 연결 - WiFi
# ============================================
# GCS (QGroundControl) 브로드캐스트 - 네트워크 내 모든 QGC 수신 가능
[UdpEndpoint GCS]
Mode = Normal
Address = $WIFI_BROADCAST
Port = $QGC_UDP_PORT

# ============================================
# ROS2 노드 연결 - localhost
# ============================================
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = 14551

EOF

echo "   ✓ mavlink-router 설정 파일 생성 완료"

# 5. systemd 서비스 파일 생성
echo ""
echo "5. systemd 서비스 파일 생성 중..."

sudo tee /etc/systemd/system/mavlink-router.service > /dev/null << EOF
[Unit]
Description=MAVLink Router
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/mavlink-routerd
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "   ✓ systemd 서비스 파일 생성 완료"

# 6. 서비스 활성화 및 시작
echo ""
echo "6. mavlink-router 서비스 활성화 및 시작 중..."

sudo systemctl daemon-reload
sudo systemctl enable mavlink-router.service

# 네트워크 설정 전이므로 시작은 나중에
echo "   ⚠ mavlink-router 서비스가 등록되었습니다."
echo "   → 네트워크 설정 후 시작됩니다: sudo systemctl start mavlink-router"

# 7. 설정 정보 저장
echo ""
echo "7. 드론 설정 정보 저장 중..."

# 드론 설정 파일 저장 (003_apply_config.sh에서 완전히 재생성하므로 간략화)
sudo tee /etc/drone-config > /dev/null << EOF
# Drone Configuration (generated by 002_install_mavlink_router.sh)
# 전체 설정은 003_apply_config.sh 실행 후 완성됨
DRONE_ID=$DRONE_ID
DRONE_NUM=$DRONE_ID
MC_ETH_IP=$ETH0_IP
FC_ETH_IP=$FC_IP
MC_WLAN_IP=$WIFI_IP
VIDEO_PORT=$VIDEO_PORT
EOF

echo "   ✓ 드론 설정 정보 저장 완료: /etc/drone-config"

# 8. 설치 검증
echo ""
echo "=========================================="
echo "설치 검증"
echo "=========================================="

if command -v mavlink-routerd > /dev/null 2>&1; then
    echo "✓ mavlink-routerd 실행 파일 확인됨"
    mavlink-routerd --version 2>/dev/null || echo "  (버전 정보 없음)"
else
    echo "✗ mavlink-routerd 실행 파일 없음"
fi

if [ -f /etc/mavlink-router/main.conf ]; then
    echo "✓ 설정 파일 확인됨: /etc/mavlink-router/main.conf"
else
    echo "✗ 설정 파일 없음"
fi

if systemctl is-enabled mavlink-router.service > /dev/null 2>&1; then
    echo "✓ systemd 서비스 활성화됨"
else
    echo "✗ systemd 서비스 활성화 안됨"
fi

echo ""
echo "=========================================="
echo "mavlink-router 설치 완료!"
echo "=========================================="
echo ""
echo "다음 단계:"
echo "  1. 설정 적용: sudo ./003-apply_config.sh"
echo "  2. 서비스 시작: sudo systemctl start mavlink-router"
echo "  3. 상태 확인: sudo systemctl status mavlink-router"
echo ""
echo "설정 파일 위치: /etc/mavlink-router/main.conf"
echo "설정 수정 후: sudo systemctl restart mavlink-router"
echo ""

