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

# FC IP 읽기
FC_IP=$(grep "^FC_IP=" "$CONFIG_FILE" | cut -d= -f2)
FC_IP=${FC_IP:-10.0.0.12}

# GCS 포트 모드 확인
GCS_PORT_MODE=$(grep "^GCS_PORT_MODE=" "$CONFIG_FILE" | cut -d= -f2)
GCS_PORT_MODE=${GCS_PORT_MODE:-unified}

# 드론별 포트 계산
PORT_OFFSET=$(( (DRONE_ID - 1) * 10 ))
EXTERNAL_PORT=$(( 16001 + PORT_OFFSET ))
APPLICATION_PORT=$(( 15001 + PORT_OFFSET ))
TCP_PORT=$(( 5790 + PORT_OFFSET ))

if [ "$GCS_PORT_MODE" = "unified" ]; then
    QGC_PORT=14550
else
    QGC_PORT=$(( 14550 + PORT_OFFSET ))
fi

# WIFI_IP에서 브로드캐스트 IP 계산
WIFI_IP=$(grep "^WIFI_IP=" "$CONFIG_FILE" | cut -d= -f2)
WIFI_IP=${WIFI_IP:-192.168.100.11}
BROADCAST_IP=$(echo "$WIFI_IP" | sed 's/\.[0-9]*$/.255/')

echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}   VIM4 FC 연결 복원${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""
echo -e "  기체 번호:        ${GREEN}$DRONE_ID${NC}"
echo -e "  FC IP:            ${GREEN}$FC_IP${NC}"
echo -e "  QGC 포트:         ${GREEN}$QGC_PORT${NC}"
echo -e "  External 포트:    ${GREEN}$EXTERNAL_PORT${NC}"
echo -e "  Application 포트: ${GREEN}$APPLICATION_PORT${NC}"
echo -e "  TCP 포트:         ${GREEN}$TCP_PORT${NC}"
echo -e "  브로드캐스트:     ${GREEN}$BROADCAST_IP${NC}"
echo ""

# mavlink-router 설정 백업 및 업데이트
echo -e "${YELLOW}[1/3] mavlink-router 설정 복원...${NC}"

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
TcpServerPort = $TCP_PORT
ReportStats = false
MavlinkDialect = common

# FC (PX4) 연결 - 이더넷 (10.0.0.x)
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = 14540

# GCS (QGroundControl) 브로드캐스트 - 드론 $DRONE_ID 전용 포트
[UdpEndpoint GCS]
Mode = Normal
Address = $BROADCAST_IP
Port = $QGC_PORT

# 외부 테스트/디버깅 도구 (SENDER GUI 등) - 드론 $DRONE_ID 전용 포트
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = $EXTERNAL_PORT

# Application Manager 연결 - 드론 $DRONE_ID 전용 포트 (라우터와 충돌 방지)
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = $APPLICATION_PORT
EOF

echo -e "  ${GREEN}✓${NC} 설정 파일 복원 완료"

# DHCP lease 초기화 (이전 MAC 주소의 infinite lease 충돌 방지)
echo -e "${YELLOW}[2/3] DHCP lease 초기화 및 dnsmasq 재시작...${NC}"
sudo truncate -s 0 /var/lib/misc/dnsmasq.leases 2>/dev/null || true
if systemctl is-active --quiet dnsmasq-px4; then
    sudo systemctl restart dnsmasq-px4
    echo -e "  ${GREEN}✓${NC} DHCP lease 초기화 완료"
else
    echo -e "  ${YELLOW}!${NC} dnsmasq-px4 서비스가 없음 (건너뜀)"
fi

# mavlink-router 재시작
echo -e "${YELLOW}[3/3] mavlink-router 재시작...${NC}"
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
