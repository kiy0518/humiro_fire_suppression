#!/bin/bash
# ==============================================================================
# Step 6: rcS 파일 설정
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step6_rcs() {
    print_header
    echo -e "${BOLD}⚙️ Step 6: rcS 파일에 VIM4 연결 설정 추가${NC}"
    echo -e "${LINE}"
    echo ""
    echo "추가될 설정:"
    echo "  - 드론 1: 192.168.100.11 (MAV_SYS_ID=1)"
    echo "  - 드론 2: 192.168.100.21 (MAV_SYS_ID=2)"
    echo "  - 드론 3: 192.168.100.31 (MAV_SYS_ID=3)"
    echo ""
    
    RCS_FILE="$HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS"
    
    if [ ! -f "$RCS_FILE" ]; then
        echo -e "${RED}오류: rcS 파일을 찾을 수 없습니다.${NC}"
        echo -e "${RED}PX4가 설치되어 있는지 확인하세요.${NC}"
        return 1
    fi
    
    # 이미 설정이 있는지 확인 - 있으면 제거
    if grep -q "VIM4 SITL 연결" "$RCS_FILE"; then
        echo -e "${YELLOW}기존 VIM4 설정 제거 중...${NC}"
        sed -i '/# ============ VIM4 SITL/,/# =========================================================/d' "$RCS_FILE"
    fi
    
    echo -e "${CYAN}rcS 파일에 설정 추가 중...${NC}"
    
    cat >> "$RCS_FILE" << 'EOF'

# ============ VIM4 SITL 연결 (인스턴스별 설정) ============
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param set EKF2_EV_CTRL 0
param set EKF2_HGT_REF 1
mavlink stop-all
uxrce_dds_client stop

if [ "$px4_instance" = "0" ]; then
    # 드론 1 -> VIM4 192.168.100.11, MAV_SYS_ID=1
    param set MAV_SYS_ID 1
    mavlink start -u 14540 -o 18001 -t 192.168.100.11 -r 4000000
    uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888 $uxrce_dds_ns
elif [ "$px4_instance" = "1" ]; then
    # 드론 2 -> VIM4 192.168.100.21, MAV_SYS_ID=2
    param set MAV_SYS_ID 2
    mavlink start -u 14541 -o 18002 -t 192.168.100.21 -r 4000000
    uxrce_dds_client start -t udp -h 192.168.100.21 -p 8888 $uxrce_dds_ns
elif [ "$px4_instance" = "2" ]; then
    # 드론 3 -> VIM4 192.168.100.31, MAV_SYS_ID=3
    param set MAV_SYS_ID 3
    mavlink start -u 14542 -o 18003 -t 192.168.100.31 -r 4000000
    uxrce_dds_client start -t udp -h 192.168.100.31 -p 8888 $uxrce_dds_ns
fi
# =========================================================
EOF
    
    echo -e "${CYAN}PX4 다시 빌드 중...${NC}"
    cd ~/PX4-Autopilot
    # 빌드만 수행 (SITL 실행 없이)
    make px4_sitl_default
    
    update_status "STEP6_RCS"
    step_complete_no_wait "rcS 파일 설정"
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step6_rcs "$@"
fi
