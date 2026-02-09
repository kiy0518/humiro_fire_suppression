#!/bin/bash
# ==============================================================================
# Step 5: Micro XRCE-DDS Agent 설치
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step5_xrce() {
    print_header
    echo -e "${BOLD}📡 Step 5: Micro XRCE-DDS Agent 설치${NC}"
    echo -e "${LINE}"
    echo ""
    echo "설치 항목:"
    echo "  - Micro-CDR"
    echo "  - Micro-XRCE-DDS-Client"
    echo "  - Micro-XRCE-DDS-Agent"
    echo ""
    
    echo -e "${CYAN}의존성 패키지 설치 중...${NC}"
    sudo apt install -y libasio-dev libtinyxml2-dev libp11-kit-dev libssl-dev
    
    # Micro-CDR
    echo -e "${CYAN}Micro-CDR 설치 중...${NC}"
    cd ~
    if [ -d "Micro-CDR" ]; then
        rm -rf Micro-CDR
    fi
    git clone https://github.com/eProsima/Micro-CDR.git
    cd Micro-CDR && mkdir -p build && cd build
    cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig
    
    # Micro-XRCE-DDS-Client
    echo -e "${CYAN}Micro-XRCE-DDS-Client 설치 중...${NC}"
    cd ~
    if [ -d "Micro-XRCE-DDS-Client" ]; then
        rm -rf Micro-XRCE-DDS-Client
    fi
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Client.git
    cd Micro-XRCE-DDS-Client && mkdir -p build && cd build
    cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig
    
    # Micro-XRCE-DDS-Agent
    echo -e "${CYAN}Micro-XRCE-DDS-Agent 설치 중...${NC}"
    cd ~
    if [ -d "Micro-XRCE-DDS-Agent" ]; then
        rm -rf Micro-XRCE-DDS-Agent
    fi
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent && mkdir -p build && cd build
    cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig
    
    echo ""
    echo -e "${CYAN}버전 확인:${NC}"
    MicroXRCEAgent --version
    
    update_status "STEP5_XRCE"
    step_complete_no_wait "Micro XRCE-DDS Agent 설치"
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step5_xrce "$@"
fi
