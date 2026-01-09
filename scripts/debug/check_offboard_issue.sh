#!/bin/bash
# OFFBOARD 모드 변경 문제 진단 스크립트

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"

echo "=========================================="
echo "OFFBOARD 모드 변경 문제 진단"
echo "=========================================="
echo ""

source "$DEVICE_CONFIG"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "1. 현재 설정 확인"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  DRONE_ID: $DRONE_ID"
echo "  WIFI_IP: $WIFI_IP"
echo "  FC_IP: $FC_IP"
echo "  ETH0_IP: $ETH0_IP"
echo ""

echo "2. GUI 설정 확인 필요"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  custom_message_sender_gui_v2.py에서 확인:"
echo "    - 대상 IP: $WIFI_IP"
echo "    - 포트: 14550"
echo "    - Target System (FC): $DRONE_ID ⚠ 중요!"
echo "    - System ID (송신자): 255 (GCS 표준)"
echo "    - Component ID (송신자): 190"
echo ""

echo "3. FC PX4 파라미터 확인 필요 (QGC에서)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ETH0_IP_DECIMAL=$(python3 -c "import struct,socket; print(struct.unpack('!I',socket.inet_aton('$ETH0_IP'))[0])" 2>/dev/null || echo "계산 실패")
echo "  다음 파라미터들이 올바르게 설정되어 있어야 합니다:"
echo "    MAV_SYS_ID = $DRONE_ID ⚠ 중요!"
echo "    UXRCE_DDS_AG_IP = $ETH0_IP_DECIMAL ($ETH0_IP)"
echo "    UXRCE_DDS_PRT = $XRCE_DDS_PORT"
echo "    UXRCE_DDS_DOM_ID = $ROS_DOMAIN_ID"
echo "    UXRCE_DDS_CFG = 1000 (Ethernet)"
echo ""

echo "4. mavlink-router 설정"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -f /etc/mavlink-router/main.conf ]; then
    echo -e "  ${GREEN}✓${NC} 설정 파일 존재"
    echo "  내용:"
    cat /etc/mavlink-router/main.conf | sed 's/^/    /'
else
    echo -e "  ${RED}✗${NC} 설정 파일 없음"
fi
echo ""

echo "5. micro-ros-agent 설정"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -f /etc/systemd/system/micro-ros-agent.service.d/override.conf ]; then
    echo -e "  ${GREEN}✓${NC} override.conf 존재"
    echo "  내용:"
    cat /etc/systemd/system/micro-ros-agent.service.d/override.conf | sed 's/^/    /'
else
    echo -e "  ${YELLOW}⚠${NC} override.conf 없음"
fi

ENV_VARS=$(systemctl show micro-ros-agent.service | grep -E "^Environment=")
if [ -n "$ENV_VARS" ]; then
    echo ""
    echo "  실제 환경 변수:"
    echo "$ENV_VARS" | sed 's/^/    /'
fi
echo ""

echo "6. 메인프로그램 실행 상태"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
MAIN_PROCESS=$(ps aux | grep "humiro_fire_suppression" | grep -v grep | grep -v "cursor-server" | grep -v "micro_ros_agent" | grep -E "(build/humiro_fire_suppression|humiro_fire_suppression$)" | head -1)
if [ -n "$MAIN_PROCESS" ]; then
    MAIN_PID=$(echo "$MAIN_PROCESS" | awk '{print $2}')
    echo -e "  ${GREEN}✓${NC} 메인프로그램 실행 중 (PID: $MAIN_PID)"
else
    echo -e "  ${RED}✗${NC} 메인프로그램 실행 안 됨"
fi
echo ""

echo "7. ROS2 토픽 확인"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  다음 명령어로 ROS2 토픽 확인:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ~/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash"
echo "    ros2 topic list | grep fmu"
echo "    ros2 topic echo /fmu/in/vehicle_command --once"
echo "    ros2 topic echo /fmu/in/offboard_control_mode --once"
echo ""

echo "8. 문제 해결 순서"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  1. GUI에서 Target System을 $DRONE_ID로 변경"
echo "  2. 커스텀 메시지 전송"
echo "  3. 메인프로그램 로그 확인 (FIRE_MISSION_START 수신 여부)"
echo "  4. ROS2 토픽 확인 (/fmu/in/vehicle_command, /fmu/in/offboard_control_mode)"
echo "  5. QGC에서 FC 파라미터 확인 (MAV_SYS_ID = $DRONE_ID)"
echo "  6. 그래도 안되면 mavlink-router/micro-ros-agent 재시작:"
echo "     sudo systemctl restart mavlink-router.service"
echo "     sudo systemctl restart micro-ros-agent.service"
echo ""
