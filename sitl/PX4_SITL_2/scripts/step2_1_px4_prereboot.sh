#!/bin/bash
# ==============================================================================
# Step 2-1: PX4 설치 (재부팅 전)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step2_1_px4_prereboot() {
    print_header
    echo -e "${BOLD}🛠️ Step 2-1: PX4 설치 (재부팅 전)${NC}"
    echo -e "${LINE}"
    echo ""
    echo -e "${YELLOW}⚠️ 이 단계는 약 20분 정도 소요됩니다.${NC}"
    echo ""
    echo "진행 순서:"
    echo "  1. PX4-Autopilot 저장소 클론"
    echo "  2. 빌드 도구 설치"
    echo "  3. 재부팅 필요"
    echo ""
    
    # PX4가 이미 설치되어 있는지 확인
    if [ -d "$HOME/PX4-Autopilot" ]; then
        echo -e "${YELLOW}PX4-Autopilot 폴더가 이미 존재합니다.${NC}"
        echo -e "기존 폴더를 사용할까요? (y=사용, n=삭제 후 재설치)"
        read -r response
        if [[ "$response" =~ ^[Nn]$ ]]; then
            echo -e "${CYAN}기존 폴더 삭제 중...${NC}"
            rm -rf "$HOME/PX4-Autopilot"
        fi
    fi
    
    # PX4 클론
    if [ ! -d "$HOME/PX4-Autopilot" ]; then
        echo -e "${CYAN}PX4-Autopilot 클론 중...${NC}"
        cd ~
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    fi
    
    cd ~/PX4-Autopilot
    
    # 빌드 도구 설치
    echo -e "${CYAN}빌드 도구 설치 중... (약 20분)${NC}"
    bash ./Tools/setup/ubuntu.sh
    
    update_status "STEP2_1_PX4_PREREBOOT"
    
    echo ""
    echo -e "${GREEN}${LINE}${NC}"
    echo -e "${GREEN}✓ PX4 설치 (재부팅 전) 완료!${NC}"
    echo -e "${GREEN}${LINE}${NC}"
    echo ""
    echo -e "${YELLOW}${LINE}${NC}"
    echo -e "${YELLOW}⚠️ 재부팅이 필요합니다!${NC}"
    echo -e "${YELLOW}${LINE}${NC}"
    echo ""
    echo -e "재부팅 후 이 스크립트를 다시 실행하고"
    echo -e "${BOLD}2-2${NC} 또는 ${BOLD}b${NC}를 선택하여 계속 진행하세요."
    echo ""
    echo -e "${YELLOW}지금 재부팅하시겠습니까? (y/n)${NC}"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        sudo reboot
    fi
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step2_1_px4_prereboot "$@"
fi
