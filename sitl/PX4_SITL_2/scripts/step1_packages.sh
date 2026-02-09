#!/bin/bash
# ==============================================================================
# Step 1: 기본 패키지 설치
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step1_packages() {
    print_header
    echo -e "${BOLD}📦 Step 1: 기본 패키지 설치${NC}"
    echo -e "${LINE}"
    echo ""
    echo "설치될 패키지:"
    echo "  - git, wget, curl, cmake, build-essential"
    echo "  - python3-pip, python3-venv"
    echo "  - openjdk-11-jdk, ninja-build"
    echo "  - GStreamer 관련 패키지"
    echo "  - net-tools"
    echo ""
    
    echo -e "${CYAN}패키지 목록 업데이트 중...${NC}"
    sudo apt update && sudo apt upgrade -y
    
    echo -e "${CYAN}패키지 설치 중...${NC}"
    sudo apt install -y \
        git wget curl cmake build-essential \
        python3-pip python3-venv \
        openjdk-11-jdk ninja-build \
        exiftool astyle \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        net-tools
    
    update_status "STEP1_PACKAGES"
    step_complete_no_wait "기본 패키지 설치"
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step1_packages "$@"
fi
