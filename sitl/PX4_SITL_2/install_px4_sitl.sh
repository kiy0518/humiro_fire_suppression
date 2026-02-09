#!/bin/bash
# ==============================================================================
# PX4 SITL 시뮬레이션 PC 설치 스크립트 (모듈화 버전)
# 단계별로 선택하여 설치를 진행합니다.
# ==============================================================================

# 스크립트 디렉토리 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts"

# 공통 함수 로드
source "$SCRIPT_DIR/common.sh"

# Ctrl+C 처리 - 메뉴로 돌아가기
INTERRUPTED=false
handle_interrupt() {
    INTERRUPTED=true
    echo ""
    echo -e "${YELLOW}작업이 중단되었습니다. 메뉴로 돌아갑니다...${NC}"
    sleep 1
}

# ==============================================================================
# 메인 루프
# ==============================================================================
main() {
    init_status
    
    while true; do
        # 매 루프마다 trap 설정 (Ctrl+C 시 메뉴로 돌아감)
        trap handle_interrupt SIGINT
        INTERRUPTED=false
        print_header
        print_menu
        
        echo -e -n "${BOLD}선택: ${NC}"
        read -r choice
        
        case $choice in
            1)
                source "$SCRIPT_DIR/step1_packages.sh"
                confirm_step || continue
                step1_packages
                ;;
            2-1|21)
                source "$SCRIPT_DIR/step2_1_px4_prereboot.sh"
                confirm_step || continue
                step2_1_px4_prereboot
                ;;
            2-2|22)
                source "$SCRIPT_DIR/step2_2_px4_postreboot.sh"
                confirm_step || continue
                step2_2_px4_postreboot
                ;;
            3)
                source "$SCRIPT_DIR/step3_ros2.sh"
                confirm_step || continue
                step3_ros2
                ;;
            4)
                source "$SCRIPT_DIR/step4_px4msgs.sh"
                confirm_step || continue
                step4_px4msgs
                ;;
            5)
                source "$SCRIPT_DIR/step5_xrce.sh"
                confirm_step || continue
                step5_xrce
                ;;
            6)
                source "$SCRIPT_DIR/step6_rcs.sh"
                confirm_step || continue
                step6_rcs
                ;;
            7)
                source "$SCRIPT_DIR/step7_drone1.sh"
                confirm_step || continue
                step7_drone1
                ;;
            8)
                source "$SCRIPT_DIR/step8_drone2.sh"
                confirm_step || continue
                step8_drone2
                ;;
            9)
                source "$SCRIPT_DIR/step9_drone3.sh"
                confirm_step || continue
                step9_drone3
                ;;
            a|A)
                print_header
                echo -e "${BOLD}🚀 재부팅 전 전체 설치 (1, 2-1)${NC}"
                echo -e "${LINE}"
                echo ""
                echo "실행될 단계:"
                echo "  1. 기본 패키지 설치"
                echo "  2-1. PX4 설치 (재부팅 전)"
                echo ""
                echo -e "${YELLOW}⚠️ 완료 후 재부팅이 필요합니다.${NC}"
                echo -e "${YELLOW}⚠️ 재부팅 후 'b'를 선택하여 나머지 설치를 진행하세요.${NC}"
                echo ""
                confirm_step || continue
                
                source "$SCRIPT_DIR/step1_packages.sh"
                echo -e "${CYAN}━━━ Step 1 시작 ━━━${NC}"
                step1_packages
                
                source "$SCRIPT_DIR/step2_1_px4_prereboot.sh"
                echo -e "${CYAN}━━━ Step 2-1 시작 ━━━${NC}"
                step2_1_px4_prereboot
                ;;
            b|B)
                print_header
                echo -e "${BOLD}🚀 재부팅 후 전체 설치 (2-2, 3, 4, 5, 6)${NC}"
                echo -e "${LINE}"
                echo ""
                echo "실행될 단계:"
                echo "  2-2. PX4 빌드 (재부팅 후)"
                echo "  3. ROS2 Humble 설치"
                echo "  4. px4_msgs 빌드"
                echo "  5. Micro XRCE-DDS Agent 설치"
                echo "  6. rcS 파일 설정"
                echo ""
                confirm_step || continue
                
                source "$SCRIPT_DIR/step2_2_px4_postreboot.sh"
                echo -e "${CYAN}━━━ Step 2-2 시작 ━━━${NC}"
                step2_2_px4_postreboot
                
                source "$SCRIPT_DIR/step3_ros2.sh"
                echo -e "${CYAN}━━━ Step 3 시작 ━━━${NC}"
                step3_ros2
                
                source "$SCRIPT_DIR/step4_px4msgs.sh"
                echo -e "${CYAN}━━━ Step 4 시작 ━━━${NC}"
                step4_px4msgs
                
                source "$SCRIPT_DIR/step5_xrce.sh"
                echo -e "${CYAN}━━━ Step 5 시작 ━━━${NC}"
                step5_xrce
                
                source "$SCRIPT_DIR/step6_rcs.sh"
                echo -e "${CYAN}━━━ Step 6 시작 ━━━${NC}"
                step6_rcs
                ;;
            r|R)
                source "$SCRIPT_DIR/full_uninstall.sh"
                full_uninstall
                ;;
            q|Q) 
                echo -e "${GREEN}설치 스크립트를 종료합니다.${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}잘못된 선택입니다.${NC}"
                sleep 1
                ;;
        esac
    done
}

# 스크립트 실행
main
