#!/bin/bash
# 모든 서비스 설정 및 상태 종합 확인

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"

echo "=========================================="
echo "모든 서비스 설정 및 상태 종합 확인"
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
echo "  WIFI_IP: $WIFI_IP"
echo "  XRCE_DDS_PORT: $XRCE_DDS_PORT"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_NAMESPACE: $ROS_NAMESPACE"
echo ""

echo "2. 네트워크 상태"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ETH0_ACTUAL=$(ip addr show eth0 | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
if [ "$ETH0_ACTUAL" = "$ETH0_IP" ]; then
    echo -e "  ${GREEN}✓${NC} eth0 IP: $ETH0_ACTUAL (설정과 일치)"
else
    echo -e "  ${RED}✗${NC} eth0 IP: $ETH0_ACTUAL (설정: $ETH0_IP, 불일치!)"
fi

FC_PING=$(ping -c 1 -W 1 $FC_IP 2>&1 | grep "1 received")
if [ -n "$FC_PING" ]; then
    echo -e "  ${GREEN}✓${NC} FC 연결: $FC_IP (ping 성공)"
else
    echo -e "  ${RED}✗${NC} FC 연결: $FC_IP (ping 실패)"
fi

FC_ARP=$(ip neigh show | grep "$FC_IP")
if [ -n "$FC_ARP" ]; then
    echo -e "  ${GREEN}✓${NC} FC ARP: $FC_ARP"
else
    echo -e "  ${YELLOW}⚠${NC} FC ARP: 없음"
fi
echo ""

echo "3. 포트 리스닝 상태"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
PORTS="14540 14550 14551 8888"
for port in $PORTS; do
    LISTENING=$(ss -ulnp | grep ":$port " || echo "")
    if [ -n "$LISTENING" ]; then
        echo -e "  ${GREEN}✓${NC} 포트 $port: 리스닝 중"
        echo "$LISTENING" | sed 's/^/    /'
    else
        echo -e "  ${RED}✗${NC} 포트 $port: 리스닝 안 됨"
    fi
done
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

MAVLINK_PID=$(pgrep -f "mavlink-routerd" | head -1)
if [ -n "$MAVLINK_PID" ]; then
    echo ""
    echo -e "  ${GREEN}✓${NC} 실행 중 (PID: $MAVLINK_PID)"
    ps aux | grep "$MAVLINK_PID" | grep -v grep | sed 's/^/    /'
else
    echo -e "  ${RED}✗${NC} 실행 안 됨"
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

MICRO_ROS_PID=$(pgrep -f "micro_ros_agent" | head -1)
if [ -n "$MICRO_ROS_PID" ]; then
    echo ""
    echo -e "  ${GREEN}✓${NC} 실행 중 (PID: $MICRO_ROS_PID)"
    ps aux | grep "$MICRO_ROS_PID" | grep -v grep | sed 's/^/    /'
    
    # 포트 확인
    MICRO_ROS_PORT=$(ss -ulnp | grep "$MICRO_ROS_PID" | grep -o ":$XRCE_DDS_PORT " || echo "")
    if [ -n "$MICRO_ROS_PORT" ]; then
        echo -e "    ${GREEN}✓${NC} 포트 $XRCE_DDS_PORT 리스닝 중"
    else
        echo -e "    ${RED}✗${NC} 포트 $XRCE_DDS_PORT 리스닝 안 됨"
    fi
else
    echo -e "  ${RED}✗${NC} 실행 안 됨"
fi
echo ""

echo "6. wrapper 스크립트 확인"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
WRAPPER_SCRIPT="$SCRIPTS_RUNTIME/start_micro_ros_agent_wrapper.sh"
if [ -f "$WRAPPER_SCRIPT" ]; then
    echo -e "  ${GREEN}✓${NC} wrapper 스크립트 존재"
    echo "  포트 설정:"
    grep "udp4 --port" "$WRAPPER_SCRIPT" | sed 's/^/    /'
    echo "  환경 변수:"
    grep -E "ROS_DOMAIN_ID|ROS_NAMESPACE" "$WRAPPER_SCRIPT" | sed 's/^/    /'
else
    echo -e "  ${RED}✗${NC} wrapper 스크립트 없음: $WRAPPER_SCRIPT"
fi
echo ""

echo "7. FC PX4 파라미터 확인 필요 (QGC에서)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ETH0_IP_DECIMAL=$(python3 -c "import struct,socket; print(struct.unpack('!I',socket.inet_aton('$ETH0_IP'))[0])" 2>/dev/null || echo "계산 실패")
echo "  다음 파라미터 확인:"
echo "    MAV_SYS_ID = $DRONE_ID"
echo "    UXRCE_DDS_AG_IP = $ETH0_IP_DECIMAL ($ETH0_IP)"
echo "    UXRCE_DDS_PRT = $XRCE_DDS_PORT"
echo "    UXRCE_DDS_DOM_ID = $ROS_DOMAIN_ID"
echo "    UXRCE_DDS_CFG = 1000 (Ethernet)"
echo ""

echo "8. ROS2 토픽 테스트 (수동 실행)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash"
echo "  ros2 topic list | grep fmu"
echo "  timeout 5 ros2 topic echo /fmu/out/vehicle_status --once"
echo "  timeout 5 ros2 topic echo /fmu/in/vehicle_command --once"
echo ""
