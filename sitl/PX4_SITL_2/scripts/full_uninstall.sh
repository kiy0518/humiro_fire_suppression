#!/bin/bash
# ==============================================================================
# 전체 초기화 (삭제)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

full_uninstall() {
    print_header
    echo -e "${RED}${BOLD}🗑️ 전체 초기화 (삭제)${NC}"
    echo -e "${LINE}"
    echo ""
    echo -e "${RED}⚠️ 경고: 다음 항목들이 모두 삭제됩니다!${NC}"
    echo ""
    echo "  - PX4-Autopilot"
    echo "  - ROS2 Humble"
    echo "  - Gazebo"
    echo "  - Micro XRCE-DDS Agent"
    echo "  - px4_ros2_ws"
    echo "  - 드론 실행 스크립트"
    echo "  - 모든 설정 파일"
    echo ""
    echo -e "${YELLOW}'yes'를 입력하면 삭제를 진행합니다.${NC}"
    echo -e -n "${BOLD}입력: ${NC}"
    read -r confirm
    
    if [ "$confirm" != "yes" ]; then
        echo -e "${GREEN}취소되었습니다.${NC}"
        sleep 2
        return
    fi
    
    echo ""
    echo -e "${CYAN}[1/9] 프로세스 종료...${NC}"
    # 자기 자신을 제외하고 px4 관련 프로세스 종료
    pgrep -f "px4" | grep -v "^$$\$" | grep -v "install" | xargs -r kill -9 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 ruby 2>/dev/null || true
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    pkill -9 MicroXRCEAgent 2>/dev/null || true
    pkill -9 micro_ros_agent 2>/dev/null || true
    sleep 2
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[2/9] PX4-Autopilot 삭제...${NC}"
    rm -rf "$HOME/PX4-Autopilot" 2>/dev/null || true
    rm -rf "$HOME/px4_ros_com_ros2" 2>/dev/null || true
    rm -rf "$HOME/px4_ros2_ws" 2>/dev/null || true
    rm -rf "$HOME/px4_msgs" 2>/dev/null || true
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[3/9] Micro XRCE-DDS Agent 삭제...${NC}"
    rm -rf "$HOME/Micro-CDR" 2>/dev/null || true
    rm -rf "$HOME/Micro-XRCE-DDS-Client" 2>/dev/null || true
    rm -rf "$HOME/Micro-XRCE-DDS-Agent" 2>/dev/null || true
    sudo rm -f /usr/local/bin/MicroXRCEAgent 2>/dev/null || true
    sudo rm -rf /usr/local/include/uxr 2>/dev/null || true
    sudo rm -rf /usr/local/lib/libmicroxrcedds* 2>/dev/null || true
    sudo rm -rf /usr/local/lib/libmicrocdr* 2>/dev/null || true
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[4/9] Gazebo 완전 삭제...${NC}"
    sudo apt remove --purge -y gz-harmonic 2>/dev/null || true
    sudo apt remove --purge -y 'gz-*' 2>/dev/null || true
    sudo apt remove --purge -y 'libgz-*' 2>/dev/null || true
    sudo apt remove --purge -y 'gazebo*' 2>/dev/null || true
    sudo apt remove --purge -y 'libgazebo*' 2>/dev/null || true
    sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list 2>/dev/null || true
    sudo rm -f /etc/apt/keyrings/pkgs-osrf-archive-keyring.gpg 2>/dev/null || true
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[5/9] ROS2 Humble 완전 삭제...${NC}"
    sudo apt remove --purge -y ros-humble-* 2>/dev/null || true
    sudo apt remove --purge -y ros-dev-tools 2>/dev/null || true
    sudo rm -f /etc/apt/sources.list.d/ros2.list 2>/dev/null || true
    sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || true
    rm -rf "$HOME/ros2_humble" 2>/dev/null || true
    rm -rf "$HOME/colcon_ws" 2>/dev/null || true
    rm -rf "$HOME/ros2_ws" 2>/dev/null || true
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[6/9] 캐시 및 설정 파일 삭제...${NC}"
    rm -rf ~/.gz 2>/dev/null || true
    rm -rf ~/.gazebo 2>/dev/null || true
    rm -rf ~/.ignition 2>/dev/null || true
    rm -rf ~/.ros 2>/dev/null || true
    rm -rf ~/.cache/gz 2>/dev/null || true
    rm -rf ~/.config/gz 2>/dev/null || true
    rm -rf ~/.local/share/gz 2>/dev/null || true
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[7/9] PX4 SITL 스크립트 및 설정 삭제...${NC}"
    rm -f "$HOME/start_sitl_for_vim4.sh" 2>/dev/null || true
    rm -f "$HOME/start_sitl_drone1.sh" 2>/dev/null || true
    rm -f "$HOME/start_sitl_drone2.sh" 2>/dev/null || true
    rm -f "$HOME/start_sitl_drone3.sh" 2>/dev/null || true
    rm -f "$HOME/extras.txt" 2>/dev/null || true
    rm -f "$HOME/.px4_sitl_install_status" 2>/dev/null || true
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[8/9] .bashrc 정리...${NC}"
    if [ -f "$HOME/.bashrc" ]; then
        # PX4, Gazebo, ROS2 관련 모든 줄 제거
        sed -i '/PX4/d' "$HOME/.bashrc"
        sed -i '/px4/d' "$HOME/.bashrc"
        sed -i '/GZ_SIM/d' "$HOME/.bashrc"
        sed -i '/GAZEBO/d' "$HOME/.bashrc"
        sed -i '/gazebo/d' "$HOME/.bashrc"
        sed -i '/gz-sim/d' "$HOME/.bashrc"
        sed -i '/ros\/humble/d' "$HOME/.bashrc"
        sed -i '/ROS_DOMAIN_ID/d' "$HOME/.bashrc"
        sed -i '/ROS_LOCALHOST_ONLY/d' "$HOME/.bashrc"
        sed -i '/colcon_cd/d' "$HOME/.bashrc"
        sed -i '/AMENT/d' "$HOME/.bashrc"
        sed -i '/source.*setup.bash/d' "$HOME/.bashrc"
    fi
    echo -e "  ${GREEN}✓${NC} 완료"
    
    echo -e "${CYAN}[9/9] 시스템 정리...${NC}"
    sudo apt autoremove -y --purge 2>/dev/null || true
    sudo apt autoclean -y 2>/dev/null || true
    sudo ldconfig 2>/dev/null || true
    echo -e "  ${GREEN}✓${NC} 완료"
    
    # 상태 파일 초기화
    rm -f "$STATUS_FILE"
    init_status
    
    echo ""
    echo -e "${GREEN}${LINE}${NC}"
    echo -e "${GREEN}✓ 전체 초기화 완료!${NC}"
    echo -e "${GREEN}${LINE}${NC}"
    echo ""
    echo "삭제됨:"
    echo "  ✓ PX4-Autopilot"
    echo "  ✓ Gazebo"
    echo "  ✓ ROS2 Humble"
    echo "  ✓ Micro XRCE-DDS Agent"
    echo "  ✓ 모든 캐시/설정"
    echo ""
    echo -e "${YELLOW}터미널을 다시 열어주세요.${NC}"
    echo ""
    echo -e "${YELLOW}아무 키나 누르면 메뉴로 돌아갑니다...${NC}"
    read -n 1 -s
}

# 직접 실행 시
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    full_uninstall "$@"
fi
