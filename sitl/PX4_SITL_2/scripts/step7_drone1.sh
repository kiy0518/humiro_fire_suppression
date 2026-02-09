#!/bin/bash
# ==============================================================================
# Step 7: 드론 1 실행 스크립트 생성
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step7_drone1() {
    print_header
    echo -e "${BOLD}🚁 Step 7: 드론 1 실행 스크립트 생성${NC}"
    echo -e "${LINE}"
    echo ""
    echo "설정 정보:"
    echo "  - VIM4 IP: 192.168.100.11"
    echo "  - 인스턴스: -i 0"
    echo "  - MAV_SYS_ID: 1"
    echo ""
    
    cat > ~/start_sitl_drone1.sh << 'EOF'
#!/bin/bash
# ==============================================================================
# PX4 SITL for VIM4 연결 (드론 1)
# ==============================================================================

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============ 설정 ============
VIM4_IP="192.168.100.11"
# ==============================

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE} PX4 SITL 시뮬레이션 시작 (드론 1)${NC}"
echo -e "${BLUE}======================================${NC}"
echo -e "  VIM4 IP: ${GREEN}$VIM4_IP${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# 기존 프로세스 정리
echo -e "${YELLOW}[1/3] 기존 프로세스 정리...${NC}"
pkill -9 px4 2>/dev/null || true
pkill -9 gz 2>/dev/null || true
pkill -9 ruby 2>/dev/null || true
pkill -9 MicroXRCEAgent 2>/dev/null || true
sleep 2
echo -e "  ${GREEN}✓${NC} 완료"

# VIM4 연결 테스트
echo -e "${YELLOW}[2/3] VIM4 연결 테스트...${NC}"
if ping -c 1 -W 2 "$VIM4_IP" > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} VIM4 연결 가능: $VIM4_IP"
else
    echo -e "  ${RED}✗${NC} VIM4에 연결할 수 없습니다: $VIM4_IP"
    exit 1
fi

# ROS2 환경 로드
echo -e "${YELLOW}[3/3] ROS2 환경 설정...${NC}"
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0
echo -e "  ${GREEN}✓${NC} ROS_DOMAIN_ID=0"

# 참고: MicroXRCEAgent는 불필요 (rcS에서 uxrce_dds_client가 VIM4로 직접 연결)

# PX4 SITL 시작
echo ""
echo -e "${BLUE}======================================${NC}"
echo -e "${GREEN} PX4 SITL 실행 중...${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

cd ~/PX4-Autopilot

PX4_SYS_AUTOSTART=4001 \
PX4_HOME_LAT=35.905863 \
PX4_HOME_LON=128.802615 \
PX4_HOME_ALT=0 \
./build/px4_sitl_default/bin/px4 -i 0
EOF

    chmod +x ~/start_sitl_drone1.sh
    
    update_status "STEP7_DRONE1"
    step_complete "드론 1 실행 스크립트 생성"
    echo -e "실행 명령: ${GREEN}~/start_sitl_drone1.sh${NC}"
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step7_drone1 "$@"
fi
