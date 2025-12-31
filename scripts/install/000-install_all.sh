#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# =============================================================================
# 000-install_all.sh - 클러스터 드론 통합 설치 스크립트
# =============================================================================
# 용도: 모든 설치 스크립트를 순서대로 실행하여 시스템 구성 완료
# 실행: sudo ./000-install_all.sh
#
# 실행 순서:
#   1. device_config.env 설정 확인
#   2. 001-install_px4_ros2_complete.sh - PX4 ROS2 XRCE-DDS 설치
#   3. 002-install_mavlink_router.sh - MAVLink Router 설치
#   4. 004-rebuild_workspaces.sh - 워크스페이스 재빌드 (경로 변경 시)
#   5. 003-apply_config.sh - 설정 적용
#
# 설치 후 설정 변경:
#   1. device_config.env 수정
#   2. sudo ./003-apply_config.sh 실행
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 로그 함수
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 구분선 출력
print_separator() {
    echo ""
    echo "=============================================="
    echo "$1"
    echo "=============================================="
    echo ""
}

# sudo 권한 확인
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        log_error "이 스크립트는 sudo 권한이 필요합니다."
        echo "실행 방법: sudo ./000-install_all.sh"
        exit 1
    fi
}

# device_config.env 확인
check_config() {
    print_separator "1단계: 설정 파일 확인"
    
    CONFIG_FILE="$SCRIPT_DIR/device_config.env"
    VERSION_FILE="$SCRIPT_DIR/versions.env"
    
    if [ ! -f "$CONFIG_FILE" ]; then
        log_error "device_config.env 파일이 없습니다."
        echo "먼저 device_config.env 파일을 설정하세요."
        exit 1
    fi
    
    source "$CONFIG_FILE"
    
    log_info "기체별 수정 항목:"
    echo "  - 드론 번호: #$DRONE_ID"
    echo "  - ROS 네임스페이스: $ROS_NAMESPACE"
    echo "  - eth0 IP: $ETH0_IP"
    echo "  - WiFi IP: $WIFI_IP"
    echo ""
    log_info "자동 설정 (수정 불필요):"
    echo "  - FC IP: $FC_IP (DHCP 고정)"
    echo "  - QGC: 브로드캐스트 (${WIFI_IP%.*}.255:$QGC_UDP_PORT)"
    echo "  - ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo ""
    
    # 버전 파일 확인
    if [ -f "$VERSION_FILE" ]; then
        source "$VERSION_FILE"
        if [ -n "$VERSION_SAVED_DATE" ]; then
            log_info "버전 파일 감지됨 (저장일: $VERSION_SAVED_DATE)"
            echo "  지정된 버전으로 설치됩니다:"
            [ -n "$MICRO_ROS_AGENT_VERSION" ] && echo "  - micro-ROS Agent: ${MICRO_ROS_AGENT_VERSION:0:12}"
            [ -n "$MAVLINK_ROUTER_VERSION" ] && echo "  - mavlink-router: ${MAVLINK_ROUTER_VERSION:0:12}"
            [ -n "$PX4_MSGS_VERSION" ] && echo "  - PX4 msgs: ${PX4_MSGS_VERSION:0:12}"
            echo ""
        fi
    else
        log_info "버전 파일 없음 - 최신 버전으로 설치됩니다"
        echo ""
    fi
    
    read -p "이 설정으로 설치를 진행하시겠습니까? (y/n): " CONFIRM
    if [ "$CONFIRM" != "y" ] && [ "$CONFIRM" != "Y" ]; then
        log_warning "설치가 취소되었습니다."
        echo "device_config.env 파일을 수정한 후 다시 실행하세요."
        exit 0
    fi
    
    log_success "설정 확인 완료"
}

# 버전 저장 함수
save_versions() {
    print_separator "버전 정보 저장"
    
    if [ -f "$SCRIPT_DIR/104-save_versions.sh" ]; then
        chmod +x "$SCRIPT_DIR/104-save_versions.sh"
        bash "$SCRIPT_DIR/104-save_versions.sh"
        log_success "버전 정보가 versions.env에 저장되었습니다"
    else
        log_warning "104-save_versions.sh 파일이 없습니다. 버전 저장을 건너뜁니다."
    fi
}

# 스크립트 실행 함수
run_script() {
    local script_name=$1
    local script_desc=$2
    local is_optional=${3:-false}
    
    print_separator "$script_desc"
    
    if [ ! -f "$SCRIPT_DIR/$script_name" ]; then
        if [ "$is_optional" = true ]; then
            log_warning "$script_name 파일이 없습니다. 건너뜁니다."
            return 0
        else
            log_error "$script_name 파일이 없습니다."
            exit 1
        fi
    fi
    
    chmod +x "$SCRIPT_DIR/$script_name"
    
    log_info "$script_name 실행 중..."
    
    if bash "$SCRIPT_DIR/$script_name"; then
        log_success "$script_name 완료"
        return 0
    else
        log_error "$script_name 실패"
        if [ "$is_optional" = true ]; then
            log_warning "선택적 스크립트이므로 계속 진행합니다."
            return 0
        else
            exit 1
        fi
    fi
}

# 메인 실행
main() {
    clear
    
    echo ""
    echo "  ╔═══════════════════════════════════════════════════════════╗"
    echo "  ║                                                           ║"
    echo "  ║        클러스터 드론 통합 설치 스크립트                   ║"
    echo "  ║        Cluster Drone Installation Script                  ║"
    echo "  ║                                                           ║"
    echo "  ╚═══════════════════════════════════════════════════════════╝"
    echo ""
    
    # sudo 권한 확인
    check_sudo
    
    # 설정 파일 확인
    check_config
    
    # 설치 시작 시간 기록
    START_TIME=$(date +%s)
    
    # 2단계: PX4 ROS2 설치
    run_script "001-install_px4_ros2_complete.sh" "2단계: PX4 ROS2 XRCE-DDS 설치"
    
    # 3단계: MAVLink Router 설치
    run_script "002-install_mavlink_router.sh" "3단계: MAVLink Router 설치"
    
    # 3.5단계: 워크스페이스 재빌드 (경로 변경 시 필요)
    if [ -f "$SCRIPT_DIR/004-rebuild_workspaces.sh" ]; then
        log_info "워크스페이스 재빌드 (새 경로 반영)..."
        bash "$SCRIPT_DIR/004-rebuild_workspaces.sh" || log_warning "워크스페이스 재빌드 실패 (계속 진행)"
    fi
    
    # 4단계: 설정 적용
    run_script "003-apply_config.sh" "4단계: 설정 적용"
    
    # 5단계: 버전 정보 저장
    save_versions
    
    # 설치 완료 시간 계산
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    MINUTES=$((ELAPSED / 60))
    SECONDS=$((ELAPSED % 60))
    
    # 최종 요약
    print_separator "설치 완료!"
    
    source "$SCRIPT_DIR/device_config.env"
    
    echo "설치된 구성요소:"
    echo "  ✓ ROS2 Humble"
    echo "  ✓ Micro-ROS Agent"
    echo "  ✓ PX4 ROS2 메시지 패키지"
    echo "  ✓ MAVLink Router"
    echo "  ✓ 워크스페이스 재빌드 (새 경로 반영)"
    echo "  ✓ 네트워크 설정 (eth0, WiFi)"
    echo "  ✓ dnsmasq DHCP 서버"
    echo ""
    
    echo "자동 시작 서비스:"
    echo "  ✓ dnsmasq-px4.service"
    echo "  ✓ mavlink-router.service"
    echo "  ✓ micro-ros-agent.service"
    echo ""
    
    echo "드론 설정 정보:"
    echo "  - 드론 번호: #$DRONE_ID"
    echo "  - eth0 IP: $ETH0_IP"
    echo "  - FC IP: $FC_IP (DHCP)"
    echo "  - WiFi IP: $WIFI_IP"
    echo "  - QGC: $QGC_IP:$QGC_UDP_PORT"
    echo "  - ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "  - ROS_NAMESPACE: $ROS_NAMESPACE"
    echo ""
    
    echo "PX4 파라미터 설정 (QGroundControl):"
    echo "  ┌──────────────────────────────────────────┐"
    echo "  │  MAV_SYS_ID = $DRONE_ID"
    DECIMAL_IP=$(python3 -c "import struct,socket; print(struct.unpack('!I',socket.inet_aton('$ETH0_IP'))[0])" 2>/dev/null || echo "???")
    echo "  │  UXRCE_DDS_AG_IP = $DECIMAL_IP ($ETH0_IP)"
    echo "  │  UXRCE_DDS_PRT = $XRCE_DDS_PORT"
    echo "  │  UXRCE_DDS_DOM_ID = $ROS_DOMAIN_ID"
    echo "  │  UXRCE_DDS_CFG = 1000 (Ethernet)"
    echo "  └──────────────────────────────────────────┘"
    echo ""
    
    echo "소요 시간: ${MINUTES}분 ${SECONDS}초"
    echo ""
    
    echo "설정 변경 방법:"
    echo "  1. device_config.env 수정"
    echo "  2. sudo ./003-apply_config.sh 실행"
    echo ""
    
    echo "버전 관리:"
    echo "  - 현재 버전 저장됨: versions.env"
    echo "  - 수동 버전 저장: ./104-save_versions.sh"
    echo "  - 동일 버전 설치: versions.env 복사 후 설치 스크립트 실행"
    echo ""
    
    log_warning "설정을 완전히 적용하려면 재부팅이 필요합니다."
    echo ""
    
    read -p "지금 재부팅하시겠습니까? (y/n): " REBOOT_CONFIRM
    if [ "$REBOOT_CONFIRM" = "y" ] || [ "$REBOOT_CONFIRM" = "Y" ]; then
        echo ""
        echo "3초 후 재부팅합니다..."
        sleep 3
        sudo reboot
    else
        echo ""
        echo "다음 명령어로 수동 재부팅하세요:"
        echo "  sudo reboot"
        echo ""
        echo "재부팅 후 연결 확인:"
        echo "  ./101_check_px4_connection.sh"
        echo ""
    fi
}

# 스크립트 실행
main "$@"
