#!/bin/bash
# ROS2-FC 연결 문제 진단 스크립트

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"

echo "=========================================="
echo "ROS2-FC 연결 문제 진단"
echo "=========================================="
echo ""

source "$DEVICE_CONFIG"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "1. 현재 설정"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  DRONE_ID: $DRONE_ID"
echo "  ETH0_IP: $ETH0_IP"
echo "  FC_IP: $FC_IP"
echo "  XRCE_DDS_PORT: $XRCE_DDS_PORT"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

echo "2. micro-ros-agent 상태"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
MICRO_ROS_PID=$(pgrep -f "micro_ros_agent" | head -1)
if [ -n "$MICRO_ROS_PID" ]; then
    echo -e "  ${GREEN}✓${NC} 실행 중 (PID: $MICRO_ROS_PID)"
    ps aux | grep "$MICRO_ROS_PID" | grep -v grep | sed 's/^/    /'
else
    echo -e "  ${RED}✗${NC} 실행 안 됨"
fi
echo ""

echo "3. FC PX4 파라미터 확인 필요 (QGC에서)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ETH0_IP_DECIMAL=$(python3 -c "import struct,socket; print(struct.unpack('!I',socket.inet_aton('$ETH0_IP'))[0])" 2>/dev/null || echo "계산 실패")
echo "  다음 파라미터들이 올바르게 설정되어 있어야 합니다:"
echo "    MAV_SYS_ID = $DRONE_ID"
echo "    UXRCE_DDS_AG_IP = $ETH0_IP_DECIMAL ($ETH0_IP) ⚠ 중요!"
echo "    UXRCE_DDS_PRT = $XRCE_DDS_PORT"
echo "    UXRCE_DDS_DOM_ID = $ROS_DOMAIN_ID"
echo "    UXRCE_DDS_CFG = 1000 (Ethernet)"
echo ""
echo "  ⚠ UXRCE_DDS_AG_IP가 1번 기체 IP로 설정되어 있을 가능성이 높습니다!"
echo "     다른 드론 예: DRONE_ID=1, 10.0.0.11 (167772171)"
echo "     현재 기체 IP: $ETH0_IP ($ETH0_IP_DECIMAL)"
echo ""

echo "4. ROS2 토픽 확인 (수동 실행 필요)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  다음 명령어를 실행하여 ROS2 토픽 확인:"
echo ""
echo "  # ROS2 환경 소스"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash"
echo ""
echo "  # 토픽 리스트 확인"
echo "  ros2 topic list | grep fmu"
echo ""
echo "  # FC에서 오는 메시지 확인 (FC 연결 확인)"
echo "  ros2 topic echo /fmu/out/vehicle_status --once"
echo ""
echo "  # FC로 보내는 메시지 확인 (ROS2 → FC)"
echo "  ros2 topic echo /fmu/in/vehicle_command --once"
echo "  ros2 topic echo /fmu/in/offboard_control_mode --once"
echo ""

echo "5. 문제 해결 방법"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  1. QGC에서 FC 파라미터 확인 및 수정:"
echo "     - UXRCE_DDS_AG_IP를 $ETH0_IP_DECIMAL ($ETH0_IP)로 변경"
echo "     - MAV_SYS_ID를 $DRONE_ID로 변경"
echo "     - 파라미터 저장 후 FC 재부팅"
echo ""
echo "  2. ROS2 토픽 확인:"
echo "     - /fmu/out/vehicle_status가 수신되는지 확인"
echo "     - /fmu/in/vehicle_command가 발행되는지 확인"
echo ""
echo "  3. micro-ros-agent 재시작:"
echo "     sudo systemctl restart micro-ros-agent.service"
echo ""
