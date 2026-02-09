#!/bin/bash
# ==============================================================================
# 공통 함수 및 변수 정의
# ==============================================================================

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'
BOLD='\033[1m'

# 구분선
LINE="━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 진행 상태 파일
STATUS_FILE="$HOME/.px4_sitl_install_status"

# 상태 파일 초기화
init_status() {
    if [ ! -f "$STATUS_FILE" ]; then
        cat > "$STATUS_FILE" << 'EOF'
STEP1_PACKAGES=0
STEP2_1_PX4_PREREBOOT=0
STEP2_2_PX4_POSTREBOOT=0
STEP3_ROS2=0
STEP4_PX4MSGS=0
STEP5_XRCE=0
STEP6_RCS=0
STEP7_DRONE1=0
STEP8_DRONE2=0
STEP9_DRONE3=0
EOF
    fi
    source "$STATUS_FILE"
}

# 상태 업데이트
update_status() {
    local step=$1
    sed -i "s/${step}=0/${step}=1/" "$STATUS_FILE"
    source "$STATUS_FILE"
}

# 상태 표시 아이콘
status_icon() {
    if [ "$1" = "1" ]; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${YELLOW}○${NC}"
    fi
}

# 헤더 출력
print_header() {
    clear
    echo -e "${BLUE}${LINE}${NC}"
    echo -e "${BLUE}${BOLD}   🚁 PX4 SITL 시뮬레이션 PC 설치 스크립트${NC}"
    echo -e "${BLUE}${LINE}${NC}"
    echo ""
}

# 메뉴 출력
print_menu() {
    init_status
    
    echo -e "${CYAN}${BOLD}📋 공통 설정 (모든 PC에서 필수)${NC}"
    echo -e "${LINE}"
    echo -e "  $(status_icon $STEP1_PACKAGES) ${BOLD}1.${NC}   기본 패키지 설치"
    echo -e "  $(status_icon $STEP2_1_PX4_PREREBOOT) ${BOLD}2-1.${NC} PX4 설치 ${YELLOW}(재부팅 전, 약 20분)${NC}"
    echo -e "  $(status_icon $STEP2_2_PX4_POSTREBOOT) ${BOLD}2-2.${NC} PX4 빌드 ${YELLOW}(재부팅 후, 약 30분)${NC}"
    echo -e "  $(status_icon $STEP3_ROS2) ${BOLD}3.${NC}   ROS2 Humble 설치"
    echo -e "  $(status_icon $STEP4_PX4MSGS) ${BOLD}4.${NC}   px4_msgs 빌드"
    echo -e "  $(status_icon $STEP5_XRCE) ${BOLD}5.${NC}   Micro XRCE-DDS Agent 설치"
    echo -e "  $(status_icon $STEP6_RCS) ${BOLD}6.${NC}   rcS 파일에 VIM4 연결 설정 추가"
    echo ""
    echo -e "${MAGENTA}${BOLD}🎮 드론별 실행 스크립트 생성${NC}"
    echo -e "${LINE}"
    echo -e "  $(status_icon $STEP7_DRONE1) ${BOLD}7.${NC} 드론 1 실행 스크립트 생성 ${CYAN}(192.168.100.11)${NC}"
    echo -e "  $(status_icon $STEP8_DRONE2) ${BOLD}8.${NC} 드론 2 실행 스크립트 생성 ${CYAN}(192.168.100.21)${NC}"
    echo -e "  $(status_icon $STEP9_DRONE3) ${BOLD}9.${NC} 드론 3 실행 스크립트 생성 ${CYAN}(192.168.100.31)${NC}"
    echo ""
    echo -e "${GREEN}${BOLD}🚀 전체 설치${NC}"
    echo -e "${LINE}"
    echo -e "  ${BOLD}a.${NC} 재부팅 전 전체 설치 ${CYAN}(1, 2-1)${NC}"
    echo -e "  ${BOLD}b.${NC} 재부팅 후 전체 설치 ${CYAN}(2-2, 3, 4, 5, 6)${NC}"
    echo ""
    echo -e "${LINE}"
    echo -e "  ${BOLD}r.${NC} ${RED}전체 초기화 (삭제)${NC}"
    echo -e "  ${BOLD}q.${NC} 종료"
    echo -e "${LINE}"
    echo ""
}

# 단계 실행 전 확인
confirm_step() {
    echo -e "${YELLOW}이 단계를 실행하시겠습니까? (y/n)${NC}"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo -e "${RED}취소되었습니다.${NC}"
        return 1
    fi
    return 0
}

# 단계 완료 메시지
step_complete() {
    echo ""
    echo -e "${GREEN}${LINE}${NC}"
    echo -e "${GREEN}✓ $1 완료!${NC}"
    echo -e "${GREEN}${LINE}${NC}"
    echo ""
    echo -e "${YELLOW}아무 키나 누르면 메뉴로 돌아갑니다...${NC}"
    read -n 1 -s
}

# 단계 완료 메시지 (키 입력 대기 없음)
step_complete_no_wait() {
    echo ""
    echo -e "${GREEN}${LINE}${NC}"
    echo -e "${GREEN}✓ $1 완료!${NC}"
    echo -e "${GREEN}${LINE}${NC}"
    echo ""
}
