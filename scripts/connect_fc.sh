#!/bin/bash
# ==============================================================================
# FC 연결 복원 스크립트
# VIM4를 실제 FC (PX4)에 연결하도록 mavlink-router 설정을 복원합니다.
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

# 드론별 포트 계산
EXTERNAL_PORT=$((16000 + DRONE_ID))
APPLICATION_PORT=$((15000 + DRONE_ID))

echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}   VIM4 FC 연결 복원${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""
echo -e "  기체 번호:      ${GREEN}$DRONE_ID${NC}"
echo -e "  External 포트:  ${GREEN}$EXTERNAL_PORT${NC}"
echo -e "  Application 포트: ${GREEN}$APPLICATION_PORT${NC}"
echo ""

# mavlink-router 설정 백업 및 업데이트
echo -e "${YELLOW}[1/2] mavlink-router 설정 복원...${NC}"

# 백업
if [ -f "$MAVLINK_CONF" ]; then
    sudo cp "$MAVLINK_CONF" "${MAVLINK_CONF}.backup.$(date +%Y%m%d_%H%M%S)"
fi

# FC 모드 설정 작성
sudo tee "$MAVLINK_CONF" > /dev/null << EOF
# Drone #$DRONE_ID mavlink-router 설정 (자동 생성)
# 생성: $(date)
# 모드: FC 연결 (실제 비행)

[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

# FC (PX4) 연결 - 브로드캐스트 수신
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = 14540

# GCS (QGroundControl) 브로드캐스트 - 드론 $DRONE_ID 전용 포트
[UdpEndpoint GCS]
Mode = Normal
Address = 192.168.100.255
Port = 14550

# 외부 테스트/디버깅 도구 (SENDER GUI 등) - 드론 $DRONE_ID 전용 포트
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = $EXTERNAL_PORT

# ROS2 노드 연결 - 드론 $DRONE_ID 전용 포트
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = 14551

# Application Manager 연결 - 드론 $DRONE_ID 전용 포트 (라우터와 충돌 방지)
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = $APPLICATION_PORT
EOF

echo -e "  ${GREEN}✓${NC} 설정 파일 복원 완료"

# mavlink-router 재시작
echo -e "${YELLOW}[2/2] mavlink-router 재시작...${NC}"
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
echo -e "${GREEN}   FC 연결 복원 완료!${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""
echo -e "FC와 연결되면 웹 GUI에서 상태를 확인하세요:"
echo -e "  ${YELLOW}http://localhost:5000${NC}"
echo ""
