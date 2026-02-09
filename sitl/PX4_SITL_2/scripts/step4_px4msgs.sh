#!/bin/bash
# ==============================================================================
# Step 4: px4_msgs 빌드
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step4_px4msgs() {
    print_header
    echo -e "${BOLD}📨 Step 4: px4_msgs 빌드${NC}"
    echo -e "${LINE}"
    echo ""
    
    # ROS2 환경 로드
    source /opt/ros/humble/setup.bash
    
    # colcon 설치 확인
    if ! command -v colcon &> /dev/null; then
        echo -e "${YELLOW}colcon이 설치되어 있지 않습니다. 설치 중...${NC}"
        sudo apt install -y python3-colcon-common-extensions
    fi
    
    echo -e "${CYAN}px4_msgs 워크스페이스 설정 중...${NC}"
    mkdir -p ~/px4_ros2_ws/src
    cd ~/px4_ros2_ws/src
    
    if [ ! -d "px4_msgs" ]; then
        git clone https://github.com/PX4/px4_msgs.git
    fi
    
    echo -e "${CYAN}px4_msgs 빌드 중...${NC}"
    cd ~/px4_ros2_ws
    colcon build --packages-select px4_msgs
    
    # bashrc에 추가 (중복 방지)
    if ! grep -q "source ~/px4_ros2_ws/install/setup.bash" ~/.bashrc; then
        echo "source ~/px4_ros2_ws/install/setup.bash" >> ~/.bashrc
    fi
    
    source ~/px4_ros2_ws/install/setup.bash
    
    update_status "STEP4_PX4MSGS"
    step_complete_no_wait "px4_msgs 빌드"
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step4_px4msgs "$@"
fi
