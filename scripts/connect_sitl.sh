#!/bin/bash
# ==============================================================================
# [DEPRECATED] SITL 연결 스크립트
# 003-apply_config.sh가 FC + SITL 듀얼 모드 설정을 자동 생성하므로
# 이 스크립트는 더 이상 필요하지 않습니다.
# mavlink-router 설정은 003-apply_config.sh에서 관리됩니다.
# ==============================================================================
echo "⚠ 이 스크립트는 더 이상 필요하지 않습니다."
echo "  003-apply_config.sh가 FC + SITL 듀얼 모드 설정을 자동 생성합니다."
echo "  현재 mavlink-router 설정에 이미 SITL 엔드포인트가 포함되어 있습니다."
echo ""
echo "  현재 설정 확인: cat /etc/mavlink-router/main.conf"
echo "  설정 재생성:    sudo ./scripts/install/003-apply_config.sh"
exit 0
# ==============================================================================
# (아래 코드는 비활성화됨)
# ==============================================================================

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 설정
PROJECT_DIR="/home/khadas/humiro_fire_suppression"
CONFIG_FILE="$PROJECT_DIR/config/device_config.env"
MAVLINK_CONF="/etc/mavlink-router/main.conf"

# 사용법
usage() {
    echo "사용법: $0 <PC_IP> [--test]"
    echo ""
    echo "인자:"
    echo "  PC_IP    Windows PC의 IP 주소 (예: 192.168.0.100)"
    echo "  --test   연결 테스트만 수행 (설정 변경 안함)"
    echo ""
    echo "예시:"
    echo "  $0 192.168.0.100"
    echo "  $0 192.168.0.100 --test"
    exit 1
}

# 인자 확인
if [ $# -lt 1 ]; then
    usage
fi

PC_IP=$1
TEST_ONLY=false

if [ "$2" == "--test" ]; then
    TEST_ONLY=true
fi

# DRONE_ID 읽기
if [ -f "$CONFIG_FILE" ]; then
    DRONE_ID=$(grep "^DRONE_ID=" "$CONFIG_FILE" | cut -d= -f2)
else
    echo -e "${RED}오류: $CONFIG_FILE 파일을 찾을 수 없습니다.${NC}"
    exit 1
fi

if [ -z "$DRONE_ID" ]; then
    echo -e "${RED}오류: DRONE_ID를 읽을 수 없습니다.${NC}"
    exit 1
fi

# SITL 포트 계산: 14540 + (DRONE_ID - 1)
SITL_PORT=$((14540 + DRONE_ID - 1))

echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}   VIM4 SITL 연결 설정${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""
echo -e "  기체 번호: ${GREEN}$DRONE_ID${NC}"
echo -e "  PC IP:     ${GREEN}$PC_IP${NC}"
echo -e "  SITL 포트: ${GREEN}$SITL_PORT${NC}"
echo ""

# 연결 테스트
echo -e "${YELLOW}[1/4] 네트워크 연결 테스트...${NC}"
if ping -c 1 -W 2 "$PC_IP" > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} PC 연결 가능"
else
    echo -e "  ${RED}✗${NC} PC에 연결할 수 없습니다: $PC_IP"
    echo -e "  ${YELLOW}→ PC의 방화벽 설정을 확인하세요.${NC}"
    exit 1
fi

# UDP 포트 테스트 (nc가 있는 경우)
echo -e "${YELLOW}[2/4] SITL 포트 테스트...${NC}"
if command -v nc &> /dev/null; then
    # UDP는 열림 여부 확인이 어려우므로 간단히 체크
    echo -e "  ${GREEN}✓${NC} UDP 포트 $SITL_PORT 사용 예정"
else
    echo -e "  ${YELLOW}!${NC} nc 명령이 없어 포트 테스트 생략"
fi

# 테스트 모드면 여기서 종료
if [ "$TEST_ONLY" = true ]; then
    echo ""
    echo -e "${GREEN}테스트 완료. 설정은 변경되지 않았습니다.${NC}"
    exit 0
fi

# mavlink-router 설정 백업 및 업데이트
echo -e "${YELLOW}[3/4] mavlink-router 설정 업데이트...${NC}"

# 백업
if [ -f "$MAVLINK_CONF" ]; then
    sudo cp "$MAVLINK_CONF" "${MAVLINK_CONF}.backup.$(date +%Y%m%d_%H%M%S)"
fi

# 새 설정 작성
sudo tee "$MAVLINK_CONF" > /dev/null << EOF
# ==============================================================================
# MAVLink Router 설정 - SITL 시뮬레이션 모드
# 생성: $(date)
# Drone ID: $DRONE_ID
# ==============================================================================

[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

# PX4 SITL 연결 (PC)
[UdpEndpoint PC_SITL]
Mode = Normal
Address = $PC_IP
Port = $SITL_PORT

# GCS 연결 (QGroundControl 등)
[UdpEndpoint GCS]
Mode = Eavesdropping
Address = 0.0.0.0
Port = 14550

# 로컬 GUI 연결
[UdpEndpoint LocalGUI]
Mode = Eavesdropping
Address = 127.0.0.1
Port = 14551
EOF

echo -e "  ${GREEN}✓${NC} 설정 파일 업데이트 완료"

# mavlink-router 재시작
echo -e "${YELLOW}[4/4] mavlink-router 재시작...${NC}"
sudo systemctl restart mavlink-router
sleep 2

# 상태 확인
if systemctl is-active --quiet mavlink-router; then
    echo -e "  ${GREEN}✓${NC} mavlink-router 실행 중"
else
    echo -e "  ${RED}✗${NC} mavlink-router 시작 실패"
    sudo journalctl -u mavlink-router -n 10 --no-pager
    exit 1
fi

echo ""
echo -e "${BLUE}=====================================${NC}"
echo -e "${GREEN}   SITL 연결 설정 완료!${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""
echo -e "다음 단계:"
echo -e "  1. PC에서 SITL 시뮬레이션 시작"
echo -e "     ${YELLOW}./multi_drone_sitl.sh 3${NC}"
echo -e ""
echo -e "  2. 웹 GUI에서 FC 연결 상태 확인"
echo -e "     ${YELLOW}http://localhost:5000${NC}"
echo ""
