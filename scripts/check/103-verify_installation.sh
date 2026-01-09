#!/bin/bash
# =============================================================================
# 103-verify_installation.sh - 전체 시스템 설치 검증
# =============================================================================
# 용도: 설치된 모든 구성요소가 정상적으로 동작하는지 확인
# 실행: bash 103-verify_installation.sh  (sudo 없이!)
#
# 검증 항목:
#   1. 시스템 정보
#   2. 필수 패키지 설치 여부
#   3. ROS2 환경
#   4. Micro-ROS Agent
#   5. PX4 ROS2 메시지 패키지
#   6. MAVLink Router
#   7. 네트워크 설정
#   8. systemd 서비스
#   9. 경로 일관성 검증
#   10. 환경 변수 검증
#   11. 설정 값 일관성 검증
#   12. 파일 권한 검증
#   13. 네트워크 연결 테스트
#   14. 시스템 리소스 확인
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"

# sudo 실행 시에도 실제 사용자 홈 디렉토리 사용
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

# =============================================================================
# 색상 정의
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 검증 결과 카운터
TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0
WARNING_CHECKS=0

# =============================================================================
# 검증 함수들
# =============================================================================

# 검증 결과 출력
check_result() {
    local status=$1
    local message=$2
    local detail=${3:-""}
    
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    
    case $status in
        "pass")
            echo -e "  ${GREEN}✓${NC} $message"
            [ -n "$detail" ] && echo -e "    ${CYAN}→${NC} $detail"
            PASSED_CHECKS=$((PASSED_CHECKS + 1))
            ;;
        "fail")
            echo -e "  ${RED}✗${NC} $message"
            [ -n "$detail" ] && echo -e "    ${RED}→${NC} $detail"
            FAILED_CHECKS=$((FAILED_CHECKS + 1))
            ;;
        "warn")
            echo -e "  ${YELLOW}⚠${NC} $message"
            [ -n "$detail" ] && echo -e "    ${YELLOW}→${NC} $detail"
            WARNING_CHECKS=$((WARNING_CHECKS + 1))
            ;;
    esac
}

# 패키지 설치 확인
check_package() {
    local pkg=$1
    if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
        local version=$(dpkg -l "$pkg" | grep "^ii" | awk '{print $3}')
        check_result "pass" "$pkg" "버전: $version"
        return 0
    else
        check_result "fail" "$pkg" "설치되지 않음"
        return 1
    fi
}

# 명령어 존재 확인
check_command() {
    local cmd=$1
    local desc=${2:-$cmd}
    if command -v "$cmd" > /dev/null 2>&1; then
        local version=$($cmd --version 2>/dev/null | head -1 || echo "")
        check_result "pass" "$desc" "$version"
        return 0
    else
        check_result "fail" "$desc" "명령어를 찾을 수 없음"
        return 1
    fi
}

# 파일 존재 확인
check_file() {
    local file=$1
    local desc=$2
    if [ -f "$file" ]; then
        check_result "pass" "$desc" "$file"
        return 0
    else
        check_result "fail" "$desc" "파일 없음: $file"
        return 1
    fi
}

# 디렉토리 존재 확인
check_directory() {
    local dir=$1
    local desc=$2
    if [ -d "$dir" ]; then
        check_result "pass" "$desc" "$dir"
        return 0
    else
        check_result "fail" "$desc" "디렉토리 없음: $dir"
        return 1
    fi
}

# 서비스 상태 확인
check_service() {
    local service=$1
    local desc=${2:-$service}
    
    if ! systemctl list-unit-files | grep -q "^${service}"; then
        check_result "fail" "$desc" "서비스가 등록되지 않음"
        return 1
    fi
    
    local enabled=$(systemctl is-enabled "$service" 2>/dev/null)
    local active=$(systemctl is-active "$service" 2>/dev/null)
    
    if [ "$active" = "active" ]; then
        check_result "pass" "$desc" "활성화: $enabled, 상태: $active"
        return 0
    elif [ "$enabled" = "enabled" ]; then
        check_result "warn" "$desc" "활성화됨, 현재 비활성 (재부팅 후 시작됨)"
        return 0
    else
        check_result "fail" "$desc" "활성화: $enabled, 상태: $active"
        return 1
    fi
}

# 네트워크 인터페이스 확인
check_network_interface() {
    local interface=$1
    local expected_ip=$2
    local desc=$3
    
    local current_ip=$(ip addr show "$interface" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
    
    if [ -z "$current_ip" ]; then
        check_result "fail" "$desc" "$interface 인터페이스 없음 또는 IP 없음"
        return 1
    elif [ "$current_ip" = "$expected_ip" ]; then
        check_result "pass" "$desc" "$interface: $current_ip"
        return 0
    else
        check_result "warn" "$desc" "현재: $current_ip, 예상: $expected_ip"
        return 0
    fi
}

# 포트 리스닝 확인
check_port() {
    local port=$1
    local desc=$2
    
    if ss -ulnp 2>/dev/null | grep -q ":$port " || netstat -ulnp 2>/dev/null | grep -q ":$port "; then
        check_result "pass" "$desc" "포트 $port 리스닝 중"
        return 0
    else
        check_result "warn" "$desc" "포트 $port가 열려있지 않음"
        return 1
    fi
}

# =============================================================================
# 메인 검증 로직
# =============================================================================

echo ""
echo "  ╔═══════════════════════════════════════════════════════════╗"
echo "  ║                                                           ║"
echo "  ║        클러스터 드론 설치 검증                            ║"
echo "  ║        Cluster Drone Installation Verification            ║"
echo "  ║                                                           ║"
echo "  ╚═══════════════════════════════════════════════════════════╝"
echo ""

# 설정 파일 로드
if [ -f "$DEVICE_CONFIG" ]; then
    source "$DEVICE_CONFIG"
    echo -e "${BLUE}[INFO]${NC} device_config.env 로드됨 (드론 #$DRONE_ID)"
    
    # 필수 변수 확인
    if [ -z "$ETH0_IP" ] || [ -z "$FC_IP" ] || [ -z "$DRONE_ID" ]; then
        echo -e "${RED}[ERROR]${NC} device_config.env에 필수 변수가 없습니다."
        echo "  필수 변수: ETH0_IP, FC_IP, DRONE_ID"
        exit 1
    fi
else
    echo -e "${RED}[ERROR]${NC} device_config.env 파일이 없습니다."
    echo "  파일 경로: $DEVICE_CONFIG"
    echo "  device_config.env 파일을 생성하고 설정하세요."
    exit 1
fi
echo ""

# -----------------------------------------------------------------------------
# 1. 시스템 정보
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[1/14] 시스템 정보${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo -e "  OS: $(lsb_release -ds 2>/dev/null || echo 'Unknown')"
echo -e "  Kernel: $(uname -r)"
echo -e "  Architecture: $(uname -m)"
echo -e "  User: $REAL_USER"
echo -e "  Home: $REAL_HOME"
echo ""

# -----------------------------------------------------------------------------
# 2. 필수 패키지
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[2/14] 필수 패키지${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

REQUIRED_PACKAGES=(
    "python3-pip"
    "python3-colcon-common-extensions"
    "git"
    "build-essential"
    "cmake"
    "dnsmasq"
    "netplan.io"
    "meson"
    "ninja-build"
)

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    check_package "$pkg"
done
echo ""

# -----------------------------------------------------------------------------
# 3. ROS2 환경
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[3/14] ROS2 환경${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ROS2 설치 파일 확인 및 소스
ROS2_SETUP="/opt/ros/humble/setup.bash"
if [ -f "$ROS2_SETUP" ]; then
    check_result "pass" "ROS2 Humble 설치" "/opt/ros/humble"
    # ROS2 환경 소스
    source "$ROS2_SETUP" 2>/dev/null
    # ROS2 CLI 확인 (소스 후)
    if command -v ros2 > /dev/null 2>&1; then
        ROS2_VERSION=$(ros2 --version 2>/dev/null | head -1 || echo "")
        check_result "pass" "ROS2 CLI" "$ROS2_VERSION"
    else
        # 소스 후에도 없으면 직접 경로 확인
        if [ -f "/opt/ros/humble/bin/ros2" ]; then
            check_result "pass" "ROS2 CLI" "설치됨 (/opt/ros/humble/bin/ros2)"
        else
            check_result "fail" "ROS2 CLI" "명령어를 찾을 수 없음"
        fi
    fi
else
    check_result "fail" "ROS2 Humble 설치" "설치 파일 없음: $ROS2_SETUP"
    check_result "fail" "ROS2 CLI" "ROS2가 설치되지 않음"
fi

check_package "ros-humble-rmw-fastrtps-cpp"
echo ""

# -----------------------------------------------------------------------------
# 4. Micro-ROS Agent
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[4/14] Micro-ROS Agent${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 환경 변수 사용
MICRO_ROS_WS="$MICRO_ROS_WS"
check_directory "$MICRO_ROS_WS" "Micro-ROS 워크스페이스"
check_file "$MICRO_ROS_WS/install/setup.bash" "Micro-ROS 빌드 결과"
check_file "$MICRO_ROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" "Micro-ROS Agent 실행 파일"
echo ""

# -----------------------------------------------------------------------------
# 5. PX4 ROS2 메시지
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[5/14] PX4 ROS2 메시지 패키지${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 환경 변수 사용
PX4_ROS2_WS="$PX4_ROS2_WS"
check_directory "$PX4_ROS2_WS" "PX4 ROS2 워크스페이스"
check_file "$PX4_ROS2_WS/install/setup.bash" "PX4 메시지 빌드 결과"
check_directory "$PX4_ROS2_WS/src/px4_msgs" "px4_msgs 소스"
echo ""

# -----------------------------------------------------------------------------
# 6. MAVLink Router
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[6/14] MAVLink Router${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_command "mavlink-routerd" "MAVLink Router"
check_file "/etc/mavlink-router/main.conf" "MAVLink Router 설정"
# 환경 변수 사용
check_directory "$MAVLINK_ROUTER_DIR" "MAVLink Router 소스"
echo ""

# -----------------------------------------------------------------------------
# 7. 네트워크 설정
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[7/14] 네트워크 설정${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_network_interface "eth0" "$ETH0_IP" "eth0 IP 주소"
check_file "/etc/netplan/01-eth0-px4.yaml" "eth0 Netplan 설정"
check_file "/etc/dnsmasq.d/px4.conf" "dnsmasq DHCP 설정"
check_file "/etc/drone-config" "드론 설정 파일"
echo ""

# -----------------------------------------------------------------------------
# 8. systemd 서비스
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[8/14] systemd 서비스${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_service "dnsmasq-px4.service" "dnsmasq DHCP 서버"
check_service "mavlink-router.service" "MAVLink Router"
check_service "micro-ros-agent.service" "Micro-ROS Agent"

# 포트 확인
echo ""
echo -e "  ${BLUE}포트 상태:${NC}"
check_port "8888" "Micro-ROS Agent (UDP 8888)"
# TCP 5790은 선택적 기능 (UDP 사용이 기본)
if ss -tlnp 2>/dev/null | grep -q ":5790 "; then
    check_result "pass" "MAVLink Router TCP (5790)" "포트 5790 리스닝 중 (선택적)"
else
    echo -e "  ${CYAN}ℹ${NC} MAVLink Router TCP (5790)"
    echo -e "    ${CYAN}→${NC} UDP 사용 중 (TCP는 선택적 기능)"
fi
echo ""

# -----------------------------------------------------------------------------
# 9. 경로 일관성 검증
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[9/14] 경로 일관성 검증${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 올바른 기준 경로 (환경 변수 사용)
CORRECT_BASE_PATH="$PROJECT_ROOT"
WRONG_PATH_PATTERN="/home/khadas/Cluster_Drone\|~/projects/Cluster_Drone"

# 경로 검증 함수
verify_path_in_file() {
    local file=$1
    local desc=$2
    
    if [ ! -f "$file" ]; then
        check_result "warn" "$desc" "파일 없음: $file"
        return 1
    fi
    
    # 잘못된 경로 패턴 검색
    if grep -q "$WRONG_PATH_PATTERN" "$file" 2>/dev/null; then
        local wrong_lines=$(grep -n "$WRONG_PATH_PATTERN" "$file" | head -3)
        check_result "fail" "$desc" "잘못된 경로 발견: $WRONG_PATH_PATTERN"
        echo -e "    ${RED}→${NC} $wrong_lines"
        return 1
    fi
    
    # 올바른 경로 확인
    if grep -q "projects/Cluster_Drone\|Cluster_Drone" "$file" 2>/dev/null; then
        check_result "pass" "$desc" "경로 정상"
        return 0
    else
        check_result "pass" "$desc" "경로 참조 없음 (정상)"
        return 0
    fi
}

# 검증할 파일들
echo -e "  ${BLUE}스크립트 파일:${NC}"
verify_path_in_file "$SCRIPTS_RUNTIME/start_micro_ros_agent_wrapper.sh" "micro-ROS wrapper 스크립트"

echo ""
echo -e "  ${BLUE}systemd 서비스 파일:${NC}"
verify_path_in_file "/etc/systemd/system/micro-ros-agent.service" "micro-ros-agent 서비스"
verify_path_in_file "/usr/local/bin/micro-ros-agent-service.sh" "micro-ros-agent 실행 스크립트"

echo ""
echo -e "  ${BLUE}사용자 설정 파일:${NC}"
verify_path_in_file "$REAL_HOME/.bashrc" ".bashrc 환경 설정"

# .bashrc에서 $REAL_HOME 변수 사용 확인 (변수가 확장되지 않아 문제 발생)
if grep -q '\$REAL_HOME' "$REAL_HOME/.bashrc" 2>/dev/null; then
    check_result "fail" ".bashrc 변수 문제" '\$REAL_HOME 사용됨 (절대 경로로 변경 필요)'
else
    check_result "pass" ".bashrc 변수 확인" "절대 경로 사용 중"
fi

# 워크스페이스 경로 존재 확인
echo ""
echo -e "  ${BLUE}워크스페이스 경로:${NC}"
if [ -d "$MICRO_ROS_WS" ]; then
    check_result "pass" "micro_ros_ws 경로" "$MICRO_ROS_WS"
else
    check_result "fail" "micro_ros_ws 경로" "디렉토리 없음: $MICRO_ROS_WS"
fi

if [ -d "$PX4_ROS2_WS" ]; then
    check_result "pass" "px4_ros2_ws 경로" "$PX4_ROS2_WS"
else
    check_result "fail" "px4_ros2_ws 경로" "디렉토리 없음: $PX4_ROS2_WS"
fi

# 잘못된 경로 디렉토리가 존재하는지 확인 (혼란 방지)
# 중복 디렉토리 확인 (옵션)
if [ -d "$REAL_HOME/Cluster_Drone" ] && [ "$REAL_HOME/Cluster_Drone" != "$PROJECT_ROOT" ]; then
    check_result "warn" "중복 디렉토리 경고" "$REAL_HOME/Cluster_Drone 존재 (혼란 가능)"
fi

echo ""

# -----------------------------------------------------------------------------
# 10. 환경 변수 검증
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[10/14] 환경 변수 검증${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# .bashrc에서 환경 변수 확인
check_env_in_bashrc() {
    local var_name=$1
    local expected_value=$2
    
    if grep -q "export ${var_name}=" "$REAL_HOME/.bashrc" 2>/dev/null; then
        local actual_value=$(grep "export ${var_name}=" "$REAL_HOME/.bashrc" | tail -1 | cut -d= -f2)
        if [ -n "$expected_value" ] && [ "$actual_value" != "$expected_value" ]; then
            check_result "warn" "$var_name in .bashrc" "설정됨: $actual_value (예상: $expected_value)"
        else
            check_result "pass" "$var_name in .bashrc" "설정됨: $actual_value"
        fi
        return 0
    else
        check_result "fail" "$var_name in .bashrc" "설정되지 않음"
        return 1
    fi
}

echo -e "  ${BLUE}.bashrc 환경 변수:${NC}"
check_env_in_bashrc "ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"
check_env_in_bashrc "ROS_NAMESPACE" "$ROS_NAMESPACE"
check_env_in_bashrc "RMW_IMPLEMENTATION" "rmw_fastrtps_cpp"

# micro-ros-agent 서비스 환경 변수 확인
echo ""
echo -e "  ${BLUE}micro-ros-agent 서비스 환경 변수:${NC}"
OVERRIDE_FILE="/etc/systemd/system/micro-ros-agent.service.d/override.conf"
if [ -f "$OVERRIDE_FILE" ]; then
    if grep -q "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" "$OVERRIDE_FILE" 2>/dev/null; then
        check_result "pass" "서비스 ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"
    else
        check_result "warn" "서비스 ROS_DOMAIN_ID" "불일치 가능"
    fi
    if grep -q "ROS_NAMESPACE=$ROS_NAMESPACE" "$OVERRIDE_FILE" 2>/dev/null; then
        check_result "pass" "서비스 ROS_NAMESPACE" "$ROS_NAMESPACE"
    else
        check_result "warn" "서비스 ROS_NAMESPACE" "불일치 가능"
    fi
else
    check_result "warn" "서비스 환경 변수 오버라이드" "파일 없음"
fi
echo ""

# -----------------------------------------------------------------------------
# 11. 설정 값 일관성 검증
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[11/14] 설정 값 일관성 검증${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# device_config.env vs /etc/drone-config 비교
echo -e "  ${BLUE}device_config.env ↔ /etc/drone-config:${NC}"

if [ -f "/etc/drone-config" ]; then
    source /etc/drone-config 2>/dev/null
    ETC_DRONE_ID=${DRONE_ID:-""}
    ETC_ETH0_IP=${ETH0_IP:-""}
    ETC_FC_IP=${FC_IP:-""}
    
    # device_config.env 다시 로드
    source "$DEVICE_CONFIG" 2>/dev/null
    
    # 비교
    if [ "$DRONE_ID" = "$ETC_DRONE_ID" ]; then
        check_result "pass" "DRONE_ID 일치" "$DRONE_ID"
    else
        check_result "fail" "DRONE_ID 불일치" "device_config: $DRONE_ID, /etc: $ETC_DRONE_ID"
    fi
    
    if [ "$ETH0_IP" = "$ETC_ETH0_IP" ]; then
        check_result "pass" "ETH0_IP 일치" "$ETH0_IP"
    else
        check_result "fail" "ETH0_IP 불일치" "device_config: $ETH0_IP, /etc: $ETC_ETH0_IP"
    fi
    
    if [ "$FC_IP" = "$ETC_FC_IP" ]; then
        check_result "pass" "FC_IP 일치" "$FC_IP"
    else
        check_result "fail" "FC_IP 불일치" "device_config: $FC_IP, /etc: $ETC_FC_IP"
    fi
else
    check_result "warn" "/etc/drone-config" "파일 없음 (003-apply_config.sh 실행 필요)"
fi

# mavlink-router 설정 확인
echo ""
echo -e "  ${BLUE}mavlink-router 설정:${NC}"
MAVLINK_CONF="/etc/mavlink-router/main.conf"
if [ -f "$MAVLINK_CONF" ]; then
    # FC 엔드포인트 Mode 확인 (Server 모드가 정상)
    FC_MODE=$(grep -A3 "\[UdpEndpoint FC\]" "$MAVLINK_CONF" | grep "Mode" | awk '{print $3}')
    if [ "$FC_MODE" = "Server" ]; then
        check_result "pass" "mavlink FC Mode" "Server (브로드캐스트 수신)"
    else
        check_result "fail" "mavlink FC Mode" "현재: $FC_MODE (예상: Server)"
    fi
    
    # FC Address 확인 (0.0.0.0이 정상)
    FC_ADDR=$(grep -A3 "\[UdpEndpoint FC\]" "$MAVLINK_CONF" | grep "Address" | awk '{print $3}')
    if [ "$FC_ADDR" = "0.0.0.0" ]; then
        check_result "pass" "mavlink FC Address" "0.0.0.0 (모든 인터페이스)"
    else
        check_result "warn" "mavlink FC Address" "$FC_ADDR (권장: 0.0.0.0)"
    fi
    
    # QGC 브로드캐스트 주소 확인
    WIFI_BROADCAST="${WIFI_IP%.*}.255"
    if grep -q "Address = $WIFI_BROADCAST" "$MAVLINK_CONF" 2>/dev/null; then
        check_result "pass" "mavlink QGC 브로드캐스트" "$WIFI_BROADCAST"
    else
        check_result "warn" "mavlink QGC 브로드캐스트" "확인 필요 ($WIFI_BROADCAST)"
    fi
else
    check_result "warn" "mavlink-router 설정" "파일 없음"
fi

# dnsmasq 설정 확인
echo ""
echo -e "  ${BLUE}dnsmasq DHCP 설정:${NC}"
DNSMASQ_CONF="/etc/dnsmasq.d/px4.conf"
if [ -f "$DNSMASQ_CONF" ]; then
    # dnsmasq 프로세스 실행 상태 확인
    if pgrep -f "dnsmasq.*px4.conf" > /dev/null 2>&1; then
        check_result "pass" "dnsmasq 프로세스" "실행 중"
    else
        check_result "fail" "dnsmasq 프로세스" "실행 안 됨"
    fi
    
    # 인터페이스 설정 확인
    if grep -q "interface=eth0" "$DNSMASQ_CONF" 2>/dev/null; then
        check_result "pass" "dnsmasq 인터페이스" "eth0"
    else
        check_result "fail" "dnsmasq 인터페이스" "eth0이 아님"
    fi
    
    # bind-interfaces 설정 확인
    if grep -q "bind-interfaces" "$DNSMASQ_CONF" 2>/dev/null; then
        check_result "pass" "dnsmasq bind-interfaces" "설정됨"
    else
        check_result "warn" "dnsmasq bind-interfaces" "설정 안 됨"
    fi
    
    # DHCP 범위가 FC_IP 하나로 제한되어 있는지 확인
    if grep -q "dhcp-range=$FC_IP,$FC_IP" "$DNSMASQ_CONF" 2>/dev/null; then
        # lease time이 infinite인지 확인
        if grep -q "dhcp-range=$FC_IP,$FC_IP.*infinite" "$DNSMASQ_CONF" 2>/dev/null; then
            check_result "pass" "FC IP 고정 할당" "$FC_IP (단일 IP, 무제한 lease)"
        else
            check_result "warn" "FC IP lease time" "infinite가 아님"
        fi
    elif grep -q "$FC_IP" "$DNSMASQ_CONF" 2>/dev/null; then
        check_result "warn" "DHCP 범위" "FC_IP 포함되나 범위가 넓음"
    else
        check_result "fail" "DHCP 범위" "FC_IP ($FC_IP) 미포함"
    fi
    
    # 게이트웨이 설정 확인 (dhcp-option=3)
    if grep -q "dhcp-option=3,$ETH0_IP" "$DNSMASQ_CONF" 2>/dev/null; then
        check_result "pass" "dnsmasq 게이트웨이" "$ETH0_IP"
    else
        check_result "warn" "dnsmasq 게이트웨이" "설정 확인 필요"
    fi
    
    # DNS 설정 확인 (dhcp-option=6)
    if grep -q "dhcp-option=6,8.8.8.8" "$DNSMASQ_CONF" 2>/dev/null; then
        check_result "pass" "dnsmasq DNS" "8.8.8.8"
    else
        check_result "warn" "dnsmasq DNS" "설정 확인 필요"
    fi
    
    # 실제 DHCP 할당 확인 (leases 파일)
    LEASES_FILE="/var/lib/misc/dnsmasq.leases"
    if [ -f "$LEASES_FILE" ]; then
        if grep -q "$FC_IP" "$LEASES_FILE" 2>/dev/null; then
            FC_LEASE=$(grep "$FC_IP" "$LEASES_FILE" | head -1)
            check_result "pass" "DHCP 할당 확인" "FC에 $FC_IP 할당됨"
        else
            check_result "warn" "DHCP 할당 확인" "FC에 IP 할당 안 됨 (FC가 꺼져있을 수 있음)"
        fi
    else
        check_result "warn" "DHCP leases 파일" "없음 (아직 할당 안 됨)"
    fi
else
    check_result "warn" "dnsmasq 설정" "파일 없음"
fi
echo ""

# -----------------------------------------------------------------------------
# 12. 파일 권한 검증
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[12/14] 파일 권한 검증${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 실행 권한 확인 함수
check_executable() {
    local file=$1
    local desc=$2
    
    if [ ! -f "$file" ]; then
        check_result "warn" "$desc" "파일 없음"
        return 1
    fi
    
    if [ -x "$file" ]; then
        check_result "pass" "$desc" "실행 가능 (+x)"
        return 0
    else
        check_result "fail" "$desc" "실행 권한 없음"
        return 1
    fi
}

# 소유권 확인 함수
check_ownership() {
    local path=$1
    local expected_owner=$2
    local desc=$3
    
    if [ ! -e "$path" ]; then
        check_result "warn" "$desc" "경로 없음"
        return 1
    fi
    
    local actual_owner=$(stat -c '%U' "$path" 2>/dev/null)
    
    if [ "$actual_owner" = "$expected_owner" ]; then
        check_result "pass" "$desc" "소유자: $actual_owner"
        return 0
    else
        check_result "fail" "$desc" "소유자: $actual_owner (예상: $expected_owner)"
        return 1
    fi
}

echo -e "  ${BLUE}스크립트 실행 권한:${NC}"
check_executable "$SCRIPTS_INSTALL/000-install_all.sh" "000-install_all.sh"
check_executable "$SCRIPTS_INSTALL/001-install_px4_ros2_complete.sh" "001-install_px4_ros2_complete.sh"
check_executable "$SCRIPTS_INSTALL/002-install_mavlink_router.sh" "002-install_mavlink_router.sh"
check_executable "$SCRIPTS_INSTALL/003-apply_config.sh" "003-apply_config.sh"
check_executable "$SCRIPTS_RUNTIME/start_micro_ros_agent_wrapper.sh" "start_micro_ros_agent_wrapper.sh"

echo ""
echo -e "  ${BLUE}워크스페이스 소유권:${NC}"
# 환경 변수 사용
check_ownership "$MICRO_ROS_WS" "$REAL_USER" "micro_ros_ws"
check_ownership "$PX4_ROS2_WS" "$REAL_USER" "px4_ros2_ws"
check_ownership "$MAVLINK_ROUTER_DIR" "$REAL_USER" "mavlink-router"
echo ""

# -----------------------------------------------------------------------------
# 13. 네트워크 연결 테스트
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[13/14] 네트워크 연결 테스트${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo -e "  ${BLUE}로컬 인터페이스:${NC}"

# eth0 상태
ETH0_STATUS=$(ip link show eth0 2>/dev/null | grep -o "state [A-Z]*" | awk '{print $2}')
if [ "$ETH0_STATUS" = "UP" ]; then
    check_result "pass" "eth0 상태" "UP"
else
    check_result "warn" "eth0 상태" "${ETH0_STATUS:-DOWN}"
fi

# wlan0 상태
WLAN0_STATUS=$(ip link show wlan0 2>/dev/null | grep -o "state [A-Z]*" | awk '{print $2}')
if [ "$WLAN0_STATUS" = "UP" ]; then
    check_result "pass" "wlan0 상태" "UP"
else
    check_result "warn" "wlan0 상태" "${WLAN0_STATUS:-DOWN/없음}"
fi

echo ""
echo -e "  ${BLUE}FC (PX4) 연결 테스트:${NC}"

# FC ping 테스트 (빠른 테스트)
if ping -c 1 -W 2 "$FC_IP" > /dev/null 2>&1; then
    check_result "pass" "FC ping ($FC_IP)" "응답 있음"
else
    check_result "warn" "FC ping ($FC_IP)" "응답 없음 (FC가 꺼져있거나 연결 안됨)"
fi

echo ""
echo -e "  ${BLUE}외부 연결 테스트:${NC}"

# 인터넷 연결 테스트
if ping -c 1 -W 3 8.8.8.8 > /dev/null 2>&1; then
    check_result "pass" "인터넷 연결" "8.8.8.8 응답"
else
    check_result "warn" "인터넷 연결" "응답 없음 (WiFi 확인 필요)"
fi
echo ""

# -----------------------------------------------------------------------------
# 14. 시스템 리소스 확인
# -----------------------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}[14/14] 시스템 리소스 확인${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 디스크 공간 확인
echo -e "  ${BLUE}디스크 공간:${NC}"
DISK_USAGE=$(df -h / | tail -1 | awk '{print $5}' | tr -d '%')
DISK_AVAIL=$(df -h / | tail -1 | awk '{print $4}')

if [ "$DISK_USAGE" -lt 80 ]; then
    check_result "pass" "루트 디스크 사용량" "${DISK_USAGE}% (여유: $DISK_AVAIL)"
elif [ "$DISK_USAGE" -lt 90 ]; then
    check_result "warn" "루트 디스크 사용량" "${DISK_USAGE}% (여유: $DISK_AVAIL)"
else
    check_result "fail" "루트 디스크 사용량" "${DISK_USAGE}% - 공간 부족!"
fi

# 홈 디렉토리 공간
HOME_DISK_AVAIL=$(df -h "$REAL_HOME" | tail -1 | awk '{print $4}')
echo -e "    ${CYAN}→${NC} 홈 디렉토리 여유 공간: $HOME_DISK_AVAIL"

echo ""
echo -e "  ${BLUE}메모리:${NC}"
MEM_TOTAL=$(free -h | grep Mem | awk '{print $2}')
MEM_USED=$(free -h | grep Mem | awk '{print $3}')
MEM_AVAIL=$(free -h | grep Mem | awk '{print $7}')
MEM_PERCENT=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100}')

if [ "$MEM_PERCENT" -lt 80 ]; then
    check_result "pass" "메모리 사용량" "${MEM_PERCENT}% (사용: $MEM_USED / 전체: $MEM_TOTAL)"
elif [ "$MEM_PERCENT" -lt 90 ]; then
    check_result "warn" "메모리 사용량" "${MEM_PERCENT}% (사용: $MEM_USED / 전체: $MEM_TOTAL)"
else
    check_result "fail" "메모리 사용량" "${MEM_PERCENT}% - 메모리 부족!"
fi
echo -e "    ${CYAN}→${NC} 가용 메모리: $MEM_AVAIL"

echo ""
echo -e "  ${BLUE}CPU:${NC}"
CPU_CORES=$(nproc)
LOAD_AVG=$(cat /proc/loadavg | awk '{print $1, $2, $3}')
echo -e "    ${CYAN}→${NC} CPU 코어: $CPU_CORES"
echo -e "    ${CYAN}→${NC} 부하 평균: $LOAD_AVG"
echo ""

# =============================================================================
# 검증 결과 요약
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${CYAN}검증 결과 요약${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo -e "  총 검증 항목: $TOTAL_CHECKS"
echo -e "  ${GREEN}통과: $PASSED_CHECKS${NC}"
echo -e "  ${YELLOW}경고: $WARNING_CHECKS${NC}"
echo -e "  ${RED}실패: $FAILED_CHECKS${NC}"
echo ""

# 결과에 따른 메시지
if [ $FAILED_CHECKS -eq 0 ]; then
    if [ $WARNING_CHECKS -eq 0 ]; then
        echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo -e "${GREEN}  ✓ 모든 검증 통과! 시스템이 정상적으로 설치되었습니다.${NC}"
        echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    else
        echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo -e "${YELLOW}  ⚠ 검증 완료. 일부 경고가 있지만 사용 가능합니다.${NC}"
        echo -e "${YELLOW}    재부팅 후 모든 서비스가 정상 작동할 것입니다.${NC}"
        echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    fi
else
    echo -e "${RED}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${RED}  ✗ 일부 검증 실패. 위의 실패 항목을 확인하세요.${NC}"
    echo -e "${RED}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "문제 해결 방법:"
    echo "  1. 설치 스크립트 재실행: sudo ./000-install_all.sh"
    echo "  2. 개별 스크립트 실행:"
    echo "     - sudo ./001-install_px4_ros2_complete.sh"
    echo "     - sudo ./002-install_mavlink_router.sh"
    echo "     - sudo ./003-apply_config.sh"
    echo "  3. 로그 확인: sudo journalctl -u <서비스명>"
fi

echo ""
echo "다음 단계:"
echo "  - PX4 연결 확인: ./101-check_px4_connection.sh"
echo "  - 버전 저장: ./104-save_versions.sh"
echo ""

# ROS2 환경 로드 안내 및 현재 세션에 적용 시도
ROS2_ENV_LOADED=false
if [ -f /opt/ros/humble/setup.bash ]; then
    if ! command -v ros2 > /dev/null 2>&1; then
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo -e "${YELLOW}⚠ ROS2 환경 안내${NC}"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "현재 세션에서 ROS2 명령어를 사용하려면 다음을 실행하세요:"
        echo "  ${CYAN}source ~/.bashrc${NC}"
        echo ""
        echo "또는 새 터미널을 열면 자동으로 ROS2 환경이 로드됩니다."
        echo ""
        echo -e "${BLUE}💡 팁:${NC} 이 스크립트는 내부에서 ROS2 환경을 소스하므로"
        echo "   스크립트 내부 검증은 정상 작동합니다."
        echo "   하지만 직접 'ros2' 명령어를 사용할 때는 위 명령을 실행하세요."
        echo ""
    else
        ROS2_ENV_LOADED=true
    fi
fi

# 종료 코드 반환
exit $FAILED_CHECKS
