#!/bin/bash
# ==============================================================================
# Step 2-2: PX4 빌드 (재부팅 후)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step2_2_px4_postreboot() {
    print_header
    echo -e "${BOLD}🛠️ Step 2-2: PX4 빌드 (재부팅 후)${NC}"
    echo -e "${LINE}"
    echo ""
    echo -e "${YELLOW}⚠️ 이 단계는 약 30분 정도 소요됩니다.${NC}"
    echo ""
    
    if [ ! -d "$HOME/PX4-Autopilot" ]; then
        echo -e "${RED}오류: PX4-Autopilot 폴더가 없습니다.${NC}"
        echo -e "${RED}먼저 Step 2-1을 실행하세요.${NC}"
        echo ""
        echo -e "${YELLOW}아무 키나 누르면 메뉴로 돌아갑니다...${NC}"
        read -n 1 -s
        return 1
    fi
    
    cd ~/PX4-Autopilot
    
    echo -e "${CYAN}PX4 빌드 중... (약 30분)${NC}"
    # 빌드만 수행 (SITL 실행 없이)
    make px4_sitl_default
    
    update_status "STEP2_2_PX4_POSTREBOOT"
    step_complete_no_wait "PX4 빌드"
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step2_2_px4_postreboot "$@"
fi
