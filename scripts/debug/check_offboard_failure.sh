#!/bin/bash
# OFFBOARD 모드 전환 실패 문제 진단

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"

echo "=========================================="
echo "OFFBOARD 모드 전환 실패 문제 진단"
echo "=========================================="
echo ""

source "$DEVICE_CONFIG"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "1. ROS2 토픽 상태"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash 2>/dev/null

V_STATUS=$(timeout 2 ros2 topic info /fmu/out/vehicle_status_v1 2>&1 | grep "Publisher count" | awk '{print $3}')
V_CMD=$(timeout 2 ros2 topic info /fmu/in/vehicle_command 2>&1 | grep "Publisher count" | awk '{print $3}')
OFFBOARD=$(timeout 2 ros2 topic info /fmu/in/offboard_control_mode 2>&1 | grep "Publisher count" | awk '{print $3}')

if [ "$V_STATUS" = "1" ]; then
    echo -e "  ${GREEN}✓${NC} /fmu/out/vehicle_status_v1: Publisher count = $V_STATUS (FC 연결 성공)"
else
    echo -e "  ${RED}✗${NC} /fmu/out/vehicle_status_v1: Publisher count = $V_STATUS (FC 연결 실패)"
fi

if [ -n "$OFFBOARD" ] && [ "$OFFBOARD" != "0" ]; then
    echo -e "  ${GREEN}✓${NC} /fmu/in/offboard_control_mode: Publisher count = $OFFBOARD (heartbeat 발행 중)"
else
    echo -e "  ${RED}✗${NC} /fmu/in/offboard_control_mode: Publisher count = $OFFBOARD (heartbeat 발행 안 됨)"
fi

if [ -n "$V_CMD" ] && [ "$V_CMD" != "0" ]; then
    echo -e "  ${GREEN}✓${NC} /fmu/in/vehicle_command: Publisher count = $V_CMD (명령 발행 중)"
else
    echo -e "  ${YELLOW}⚠${NC} /fmu/in/vehicle_command: Publisher count = $V_CMD"
fi
echo ""

echo "2. FC 상태 확인"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
FC_STATUS=$(timeout 2 ros2 topic echo /fmu/out/vehicle_status_v1 --once 2>&1 | grep -E "nav_state|arming_state|safety_off" | head -3)
if [ -n "$FC_STATUS" ]; then
    echo "$FC_STATUS" | sed 's/^/    /'
    NAV_STATE=$(echo "$FC_STATUS" | grep "nav_state:" | awk '{print $2}')
    ARMING_STATE=$(echo "$FC_STATUS" | grep "arming_state:" | awk '{print $2}')
    SAFETY_OFF=$(echo "$FC_STATUS" | grep "safety_off:" | awk '{print $2}')
    
    if [ "$NAV_STATE" = "14" ]; then
        echo -e "    ${GREEN}✓${NC} nav_state = 14 (OFFBOARD 모드)"
    else
        echo -e "    ${RED}✗${NC} nav_state = $NAV_STATE (OFFBOARD 모드 아님, 14 필요)"
    fi
    
    if [ "$ARMING_STATE" = "2" ]; then
        echo -e "    ${GREEN}✓${NC} arming_state = 2 (ARMED)"
    else
        echo -e "    ${YELLOW}⚠${NC} arming_state = $ARMING_STATE (ARM 필요: 2)"
    fi
    
    if [ "$SAFETY_OFF" = "true" ]; then
        echo -e "    ${GREEN}✓${NC} safety_off = true"
    else
        echo -e "    ${RED}✗${NC} safety_off = $SAFETY_OFF (안전 스위치 해제 필요)"
    fi
else
    echo -e "  ${RED}✗${NC} FC 상태 확인 실패"
fi
echo ""

echo "3. OFFBOARD 모드 전환 실패 원인"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  가능한 원인:"
echo "    1. FC가 ARM되지 않음 (arming_state != 2)"
echo "    2. 안전 스위치가 활성화됨 (safety_off = false)"
echo "    3. FC의 COM_RCL_EXCEPT 파라미터 설정 문제"
echo "    4. FC의 안전 설정 (CBRK_FLIGHTTERM 등)"
echo "    5. FC가 heartbeat를 받지만 OFFBOARD 모드로 전환하지 않음"
echo ""

echo "4. 해결 방법"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  1. QGC에서 FC 상태 확인:"
echo "     - ARM 가능한지 확인"
echo "     - 안전 스위치 해제 확인"
echo "     - 비행 모드가 MANUAL인지 확인"
echo ""
echo "  2. QGC에서 FC 파라미터 확인:"
echo "     - COM_RCL_EXCEPT = 0 (안전 모드 비활성화)"
echo "     - CBRK_FLIGHTTERM = 0 (비행 터미널 체크 비활성화)"
echo ""
echo "  3. QGC에서 수동으로 OFFBOARD 모드 전환 테스트:"
echo "     - QGC에서 OFFBOARD 모드로 전환 가능한지 확인"
echo "     - 전환 가능하면 ROS2 문제, 불가능하면 FC 설정 문제"
echo ""
