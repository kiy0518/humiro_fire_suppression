#!/bin/bash
# ==============================================================================
# Step 3: ROS2 Humble 설치
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

step3_ros2() {
    print_header
    echo -e "${BOLD}🤖 Step 3: ROS2 Humble 설치${NC}"
    echo -e "${LINE}"
    echo ""
    
    echo -e "${CYAN}ROS2 저장소 설정 중...${NC}"
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y universe
    sudo apt update && sudo apt install -y curl
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    echo -e "${CYAN}ROS2 Humble 설치 중...${NC}"
    sudo apt update
    sudo apt install -y ros-humble-desktop
    
    # bashrc에 추가 (중복 방지)
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    source /opt/ros/humble/setup.bash
    
    update_status "STEP3_ROS2"
    step_complete_no_wait "ROS2 Humble 설치"
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    step3_ros2 "$@"
fi
