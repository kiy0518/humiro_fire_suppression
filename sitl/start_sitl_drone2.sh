#!/bin/bash
# ==============================================================================
# PX4 SITL for VIM4 연결 (드론 2) - 드론 1 실행 후 사용
# ==============================================================================

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============ 설정 ============
VIM4_IP="192.168.100.21"
# ==============================

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE} PX4 SITL 시뮬레이션 시작 (드론 2)${NC}"
echo -e "${BLUE}======================================${NC}"
echo -e "  VIM4 IP: ${GREEN}$VIM4_IP${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# 드론 1이 실행 중인지 확인
echo -e "${YELLOW}[1/3] 드론 1 실행 확인...${NC}"
if pgrep -f "px4.*-i 0" > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} 드론 1 실행 중"
else
    echo -e "  ${RED}✗${NC} 드론 1이 실행되지 않았습니다!"
    echo -e "  ${YELLOW}→${NC} 먼저 ~/start_sitl_drone1.sh 를 실행하세요."
    exit 1
fi

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
PX4_GZ_MODEL_POSE="-10,0,0,0,0,0" \
PX4_HOME_LAT=35.905863 \
PX4_HOME_LON=128.802615 \
PX4_HOME_ALT=0 \
./build/px4_sitl_default/bin/px4 -i 1
