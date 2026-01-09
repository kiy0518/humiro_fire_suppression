#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# PX4 연결 상태 종합 확인 스크립트
# humiro_fire_suppression 프로젝트용

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# sudo 실행 시에도 실제 사용자 홈 디렉토리 사용
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

echo "=========================================="
echo "PX4 연결 상태 종합 확인"
echo "=========================================="
echo ""

# 1. 네트워크 상태
echo "1. 네트워크 상태:"
echo "   eth0 IP:"
ip addr show eth0 | grep "inet " || echo "     (IP 없음)"
echo "   eth0 상태:"
ip link show eth0 | grep -E "state|UP" || echo "     (상태 확인 불가)"

# 2. DHCP 서버 상태
echo ""
echo "2. DHCP 서버 (dnsmasq) 상태:"
if systemctl is-active --quiet dnsmasq-px4; then
    echo "   ✓ dnsmasq 실행 중"
    echo "   최근 DHCP 활동:"
    sudo journalctl -u dnsmasq-px4 -n 20 --no-pager 2>&1 | grep -i "dhcp\|lease\|request" | tail -5 || echo "     (DHCP 활동 없음)"
else
    echo "   ✗ dnsmasq 실행 안됨"
fi

# 3. 연결된 장치 확인
echo ""
echo "3. eth0에 연결된 장치:"

# device_config.env에서 FC_IP 읽기
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
CONFIG_FILE="$DEVICE_CONFIG"
EXPECTED_FC_IP=""
if [ -f "$CONFIG_FILE" ]; then
    EXPECTED_FC_IP=$(grep "^FC_IP=" "$CONFIG_FILE" | cut -d'=' -f2 | cut -d'#' -f1 | tr -d ' \t\r')
fi
if [ -z "$EXPECTED_FC_IP" ]; then
    echo "   ⚠ 경고: device_config.env에서 FC_IP를 읽을 수 없습니다."
    echo "     device_config.env 파일을 확인하세요."
    EXPECTED_FC_IP="(확인 필요)"
fi

NEIGH=$(ip neigh show dev eth0 2>/dev/null)
if [ -n "$NEIGH" ]; then
    FC_FOUND=false
    echo "$NEIGH" | while read line; do
        echo "   $line"
        # FC_IP가 ARP 테이블에 있는지 확인
        if echo "$line" | grep -q "$EXPECTED_FC_IP"; then
            FC_FOUND=true
        fi
    done
    
    # FC IP 확인
    FC_NEIGH=$(ip neigh show dev eth0 | grep "$EXPECTED_FC_IP" 2>/dev/null)
    if [ -n "$FC_NEIGH" ]; then
        FC_STATE=$(echo "$FC_NEIGH" | awk '{print $NF}')
        FC_MAC=$(echo "$FC_NEIGH" | awk '{print $5}')
        
        echo ""
        if [ "$FC_STATE" = "REACHABLE" ] || [ "$FC_STATE" = "STALE" ] || [ "$FC_STATE" = "DELAY" ] || [ "$FC_STATE" = "PROBE" ]; then
            echo "   ✓ FC 연결 확인됨:"
            echo "     - 예상 IP: $EXPECTED_FC_IP"
            if [ -n "$FC_MAC" ] && [ "$FC_MAC" != "FAILED" ]; then
                echo "     - MAC 주소: $FC_MAC"
            fi
            echo "     - 상태: $FC_STATE"
            
            # DHCP leases 파일에서도 확인
            if [ -f /var/lib/misc/dnsmasq.leases ]; then
                if sudo grep -q "$EXPECTED_FC_IP" /var/lib/misc/dnsmasq.leases 2>/dev/null; then
                    FC_HOSTNAME=$(sudo grep "$EXPECTED_FC_IP" /var/lib/misc/dnsmasq.leases | awk '{print $4}')
                    echo "     - 호스트명: ${FC_HOSTNAME:-nuttx}"
                    echo "     - DHCP 할당됨"
                fi
            fi
        else
            echo "   ⚠ FC IP ($EXPECTED_FC_IP)가 ARP 테이블에 있으나 상태가 비정상: $FC_STATE"
            echo "     → FC가 연결되어 있지만 통신이 안 되거나 응답하지 않습니다"
            echo "     → FC를 재부팅하거나 이더넷 케이블을 확인하세요"
        fi
    else
        echo ""
        echo "   ⚠ FC IP ($EXPECTED_FC_IP)가 ARP 테이블에 없음"
        echo "     → FC가 연결되지 않았거나 다른 IP를 사용 중일 수 있습니다"
    fi
else
    echo "   (ARP 테이블에 장치 없음)"
    echo "   → PX4가 연결되지 않았거나 아직 통신하지 않았습니다"
    echo ""
    echo "   예상 FC IP: $EXPECTED_FC_IP"
    echo "   → FC를 켜고 이더넷 케이블을 연결한 후 잠시 기다려주세요"
fi

# 4. Micro-ROS Agent 상태
echo ""
echo "4. Micro-ROS Agent 상태:"
# systemd 서비스 상태 확인
SERVICE_STATUS=$(systemctl is-active micro-ros-agent 2>&1)
if [ "$SERVICE_STATUS" = "active" ]; then
    echo "   ✓ systemd 서비스 실행 중"
elif [ "$SERVICE_STATUS" = "activating" ] || [ "$SERVICE_STATUS" = "auto-restart" ]; then
    echo "   ⚠ systemd 서비스 재시작 중 (문제 가능성)"
    echo "   → 로그 확인: sudo journalctl -u micro-ros-agent -n 30"
else
    echo "   ✗ systemd 서비스 실행 안됨 (상태: $SERVICE_STATUS)"
fi

# 포트 8888이 리스닝 중인지 확인
AGENT_PID=$(netstat -ulnp 2>/dev/null | grep ":8888" | awk '{print $NF}' | cut -d'/' -f1 | head -1)
if [ -z "$AGENT_PID" ]; then
    AGENT_PID=$(ss -ulnp 2>/dev/null | grep ":8888" | grep -oP 'pid=\K[0-9]+' | head -1)
fi
if [ -n "$AGENT_PID" ]; then
    echo "   ✓ Agent 실행 중 (PID: $AGENT_PID)"
    echo "   ✓ 포트 8888 리스닝 중"
else
    # 프로세스 이름으로도 확인
    AGENT_PID=$(pgrep -f "micro_ros_agent" | head -1)
    if [ -n "$AGENT_PID" ]; then
        echo "   ⚠ Agent 프로세스 발견 (PID: $AGENT_PID) - 포트 미확인"
        echo "   포트 확인:"
        netstat -ulnp 2>/dev/null | grep 8888 || ss -ulnp 2>/dev/null | grep 8888 || echo "     ✗ 포트 8888이 열려있지 않음"
        echo "   → Agent가 제대로 시작되지 않았을 수 있습니다"
        echo "   → 로그 확인: sudo journalctl -u micro-ros-agent -n 50"
    else
        echo "   ✗ Agent 실행 안됨"
        echo "   → 서비스 시작: sudo systemctl start micro-ros-agent"
        echo "   → 로그 확인: sudo journalctl -u micro-ros-agent -n 50"
    fi
fi

# 5. ROS2 토픽 확인
echo ""
echo "5. ROS2 토픽 확인:"
source $REAL_HOME/.bashrc 2>/dev/null || true
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f $REAL_HOME/workspaces/px4_ros2_ws/install/setup.bash ]; then
    source $REAL_HOME/workspaces/px4_ros2_ws/install/setup.bash
fi

if command -v ros2 > /dev/null 2>&1; then
    TOPIC_LIST=$(ros2 topic list 2>/dev/null)
    # /fmu/in과 /fmu/out 토픽 개수 각각 계산 (안전하게)
    if [ -n "$TOPIC_LIST" ]; then
        FMU_IN_COUNT=$(echo "$TOPIC_LIST" | grep -c "^/fmu/in/" 2>/dev/null)
        FMU_OUT_COUNT=$(echo "$TOPIC_LIST" | grep -c "^/fmu/out/" 2>/dev/null)
    else
        FMU_IN_COUNT=0
        FMU_OUT_COUNT=0
    fi
    # 숫자로 변환하여 안전하게 계산 (빈 값이면 0으로)
    FMU_IN_COUNT=${FMU_IN_COUNT:-0}
    FMU_OUT_COUNT=${FMU_OUT_COUNT:-0}
    # 숫자인지 확인
    if ! [[ "$FMU_IN_COUNT" =~ ^[0-9]+$ ]]; then
        FMU_IN_COUNT=0
    fi
    if ! [[ "$FMU_OUT_COUNT" =~ ^[0-9]+$ ]]; then
        FMU_OUT_COUNT=0
    fi
    TOTAL_TOPICS=$((FMU_IN_COUNT + FMU_OUT_COUNT))
    
    echo "   PX4 토픽 개수: $TOTAL_TOPICS (in: $FMU_IN_COUNT, out: $FMU_OUT_COUNT)"
    
    if [ "$TOTAL_TOPICS" -gt 0 ] 2>/dev/null; then
        # /fmu/out 토픽이 없으면 경고
        if [ "$FMU_OUT_COUNT" -eq 0 ]; then
            echo "   ⚠ PX4 연결됨 (하지만 /fmu/out 토픽 없음 - PX4에서 데이터 전송 안됨)"
            echo ""
            echo "   문제 진단:"
            echo "     → PX4가 Agent에 연결되지 않았거나"
            echo "     → PX4 파라미터 설정이 잘못되었을 수 있습니다"
            echo "     → PX4 파라미터 확인:"
            echo "       - UXRCE_DDS_CFG = Ethernet"
            echo "       - UXRCE_DDS_AG_IP = VIM4 IP (10진수)"
            echo "       - UXRCE_DDS_PRT = 8888"
            echo "       - UXRCE_DDS_DOM_ID = 0"
        else
            echo "   ✓ PX4 연결됨!"
        fi
        
        echo ""
        echo "   주요 토픽:"
        # /fmu/out 토픽 우선 표시 (PX4 → ROS2)
        FMU_OUT_TOPICS=$(echo "$TOPIC_LIST" | grep "^/fmu/out/" | head -10)
        if [ -n "$FMU_OUT_TOPICS" ]; then
            echo "     [PX4 → ROS2] /fmu/out/ 토픽:"
            echo "$FMU_OUT_TOPICS" | sed 's/^/       /'
        fi
        
        # /fmu/in 토픽 표시 (ROS2 → PX4)
        FMU_IN_TOPICS=$(echo "$TOPIC_LIST" | grep "^/fmu/in/" | head -5)
        if [ -n "$FMU_IN_TOPICS" ]; then
            echo ""
            echo "     [ROS2 → PX4] /fmu/in/ 토픽:"
            echo "$FMU_IN_TOPICS" | sed 's/^/       /'
        fi
    else
        echo "   ✗ PX4 토픽 없음"
    fi
    TOPICS=$TOTAL_TOPICS
else
    echo "   ✗ ROS2 명령어 없음"
    TOPICS=0
    FMU_OUT_COUNT=0
fi

# 6. 종합 진단
echo ""
echo "=========================================="
echo "종합 진단"
echo "=========================================="

ISSUES=0

# 네트워크 확인
if ! ip addr show eth0 | grep -q "10.0.0"; then
    echo "✗ eth0에 IP가 할당되지 않음"
    echo "  해결: sudo ./003-apply_config.sh"
    ISSUES=$((ISSUES + 1))
fi

# DHCP 확인
if ! systemctl is-active --quiet dnsmasq-px4; then
    echo "✗ dnsmasq 실행 안됨"
    echo "  해결: sudo systemctl start dnsmasq-px4"
    ISSUES=$((ISSUES + 1))
fi

# Agent 확인
if [ -z "$AGENT_PID" ]; then
    echo "✗ Micro-ROS Agent 실행 안됨"
    echo "  해결: ./start_micro_ros_agent.sh"
    ISSUES=$((ISSUES + 1))
fi

# PX4 연결 확인
TOPICS_NUM=${TOPICS:-0}
FMU_OUT_COUNT=${FMU_OUT_COUNT:-0}

if [ "$TOPICS_NUM" -eq 0 ] 2>/dev/null; then
    echo "✗ PX4 토픽 없음"
    echo ""
    # device_config.env에서 ETH0_IP 읽기
    CONFIG_FILE="$SCRIPT_DIR/device_config.env"
    ETH0_IP=""
    if [ -f "$CONFIG_FILE" ]; then
        ETH0_IP=$(grep "^ETH0_IP=" "$CONFIG_FILE" | cut -d'=' -f2 | cut -d'#' -f1 | tr -d ' \t\r')
    fi
    if [ -z "$ETH0_IP" ]; then
        echo "     ⚠ 경고: device_config.env에서 ETH0_IP를 읽을 수 없습니다."
        DECIMAL_IP="(확인 필요)"
    else
        # IP를 decimal로 변환
        IFS='.' read -r a b c d <<< "$ETH0_IP"
        DECIMAL_IP=$((a*256*256*256 + b*256*256 + c*256 + d))
    fi
    
    echo "  확인 사항:"
    echo "    1. PX4가 eth0 케이블로 연결되어 있는지 확인"
    echo "    2. PX4를 재부팅하여 DHCP로 IP를 받도록 함"
    echo "    3. PX4 파라미터 확인 (QGroundControl):"
    echo "       - UXRCE_DDS_CFG = Ethernet"
    echo "       - UXRCE_DDS_AG_IP = $DECIMAL_IP ($ETH0_IP)"
    echo "       - UXRCE_DDS_PRT = 8888"
    echo "       - UXRCE_DDS_DOM_ID = 0"
    echo "    4. DHCP 로그 확인: sudo tail -f /var/log/dnsmasq-px4.log"
    ISSUES=$((ISSUES + 1))
elif [ "$FMU_OUT_COUNT" -eq 0 ] 2>/dev/null; then
    echo "✗ PX4에서 데이터 전송 안됨 (/fmu/out 토픽 없음)"
    echo ""
    # device_config.env에서 ETH0_IP 읽기
    CONFIG_FILE="$SCRIPT_DIR/device_config.env"
    ETH0_IP=""
    if [ -f "$CONFIG_FILE" ]; then
        ETH0_IP=$(grep "^ETH0_IP=" "$CONFIG_FILE" | cut -d'=' -f2 | cut -d'#' -f1 | tr -d ' \t\r')
    fi
    if [ -z "$ETH0_IP" ]; then
        echo "     ⚠ 경고: device_config.env에서 ETH0_IP를 읽을 수 없습니다."
        DECIMAL_IP="(확인 필요)"
    else
        # IP를 decimal로 변환
        IFS='.' read -r a b c d <<< "$ETH0_IP"
        DECIMAL_IP=$((a*256*256*256 + b*256*256 + c*256 + d))
    fi
    
    echo "  문제: /fmu/in 토픽은 있지만 /fmu/out 토픽이 없습니다."
    echo "  이는 PX4가 Agent에 연결되지 않았거나 파라미터가 잘못되었음을 의미합니다."
    echo ""
    echo "  확인 사항:"
    echo "    1. PX4 파라미터 확인 (QGroundControl):"
    echo "       - UXRCE_DDS_CFG = Ethernet (반드시 확인!)"
    echo "       - UXRCE_DDS_AG_IP = $DECIMAL_IP ($ETH0_IP) (10진수 형식)"
    echo "       - UXRCE_DDS_PRT = 8888"
    echo "       - UXRCE_DDS_DOM_ID = 0"
    echo "    2. PX4 재부팅 (파라미터 변경 후)"
    echo "    3. Agent 로그 확인: sudo journalctl -u micro-ros-agent -n 50 -f"
    echo "    4. PX4 콘솔에서 확인:"
    echo "       - 'micro_ros_agent' 연결 메시지 확인"
    echo "       - 'XRCE-DDS' 관련 에러 메시지 확인"
    ISSUES=$((ISSUES + 1))
fi

if [ $ISSUES -eq 0 ]; then
    echo "✓ 모든 설정이 정상입니다!"
else
    echo ""
    echo "발견된 문제: $ISSUES 개"
fi

echo ""

