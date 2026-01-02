#!/bin/bash
# =============================================================================
# Humiro Fire Suppression Service Control Script
# =============================================================================
# 개발 중 자동 실행 관리를 위한 스크립트
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SERVICE_NAME="humiro-fire-suppression"
SERVICE_FILE="$PROJECT_ROOT/scripts/runtime/humiro-fire-suppression.service"
SYSTEMD_DIR="/etc/systemd/system"
SYSTEMD_SERVICE="$SYSTEMD_DIR/$SERVICE_NAME.service"
LOG_DIR="$PROJECT_ROOT/logs"

# 색상 출력
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 로그 디렉토리 생성
mkdir -p "$LOG_DIR"

# 함수: 서비스 파일 설치
install_service() {
    echo -e "${BLUE}[설치] 서비스 파일 설치 중...${NC}"
    
    if [ ! -f "$SERVICE_FILE" ]; then
        echo -e "${RED}오류: 서비스 파일을 찾을 수 없습니다: $SERVICE_FILE${NC}"
        exit 1
    fi
    
    sudo cp "$SERVICE_FILE" "$SYSTEMD_SERVICE"
    sudo chmod 644 "$SYSTEMD_SERVICE"
    sudo systemctl daemon-reload
    
    echo -e "${GREEN}✓ 서비스 파일 설치 완료${NC}"
}

# 함수: 서비스 활성화 (부팅 시 자동 시작)
enable_service() {
    echo -e "${BLUE}[활성화] 서비스 활성화 중...${NC}"
    
    if [ ! -f "$SYSTEMD_SERVICE" ]; then
        echo -e "${YELLOW}서비스 파일이 없습니다. 먼저 설치합니다...${NC}"
        install_service
    fi
    
    sudo systemctl enable "$SERVICE_NAME"
    echo -e "${GREEN}✓ 서비스 활성화 완료 (부팅 시 자동 시작)${NC}"
}

# 함수: 서비스 비활성화 (부팅 시 자동 시작 안 함)
disable_service() {
    echo -e "${BLUE}[비활성화] 서비스 비활성화 중...${NC}"
    
    sudo systemctl disable "$SERVICE_NAME"
    echo -e "${GREEN}✓ 서비스 비활성화 완료 (부팅 시 자동 시작 안 함)${NC}"
}

# 함수: 서비스 시작
start_service() {
    echo -e "${BLUE}[시작] 서비스 시작 중...${NC}"
    
    if [ ! -f "$SYSTEMD_SERVICE" ]; then
        echo -e "${YELLOW}서비스 파일이 없습니다. 먼저 설치합니다...${NC}"
        install_service
    fi
    
    sudo systemctl start "$SERVICE_NAME"
    
    sleep 1
    if sudo systemctl is-active --quiet "$SERVICE_NAME"; then
        echo -e "${GREEN}✓ 서비스 시작 완료${NC}"
    else
        echo -e "${RED}✗ 서비스 시작 실패${NC}"
        echo -e "${YELLOW}로그 확인: sudo journalctl -u $SERVICE_NAME -n 50${NC}"
        exit 1
    fi
}

# 함수: 서비스 중지
stop_service() {
    echo -e "${BLUE}[중지] 서비스 중지 중...${NC}"
    
    sudo systemctl stop "$SERVICE_NAME"
    
    sleep 1
    if ! sudo systemctl is-active --quiet "$SERVICE_NAME"; then
        echo -e "${GREEN}✓ 서비스 중지 완료${NC}"
    else
        echo -e "${RED}✗ 서비스 중지 실패${NC}"
        exit 1
    fi
}

# 함수: 서비스 재시작
restart_service() {
    echo -e "${BLUE}[재시작] 서비스 재시작 중...${NC}"
    
    sudo systemctl restart "$SERVICE_NAME"
    
    sleep 1
    if sudo systemctl is-active --quiet "$SERVICE_NAME"; then
        echo -e "${GREEN}✓ 서비스 재시작 완료${NC}"
    else
        echo -e "${RED}✗ 서비스 재시작 실패${NC}"
        echo -e "${YELLOW}로그 확인: sudo journalctl -u $SERVICE_NAME -n 50${NC}"
        exit 1
    fi
}

# 함수: 서비스 상태 확인
status_service() {
    echo -e "${BLUE}[상태] 서비스 상태 확인 중...${NC}"
    echo ""
    
    sudo systemctl status "$SERVICE_NAME" --no-pager
    
    echo ""
    echo -e "${BLUE}로그 파일 위치:${NC}"
    echo "  - 표준 출력: $LOG_DIR/humiro-service.log"
    echo "  - 에러 로그: $LOG_DIR/humiro-service-error.log"
    echo ""
    echo -e "${BLUE}유용한 명령어:${NC}"
    echo "  - 실시간 로그: sudo journalctl -u $SERVICE_NAME -f"
    echo "  - 최근 로그: sudo journalctl -u $SERVICE_NAME -n 50"
    echo "  - 로그 파일: tail -f $LOG_DIR/humiro-service.log"
}

# 함수: 로그 보기
show_logs() {
    local lines=${1:-50}
    
    echo -e "${BLUE}[로그] 최근 $lines 줄 표시${NC}"
    echo ""
    
    if [ -f "$LOG_DIR/humiro-service.log" ]; then
        echo -e "${GREEN}=== 표준 출력 로그 ===${NC}"
        tail -n "$lines" "$LOG_DIR/humiro-service.log"
    else
        echo -e "${YELLOW}표준 출력 로그 파일이 없습니다.${NC}"
    fi
    
    echo ""
    
    if [ -f "$LOG_DIR/humiro-service-error.log" ]; then
        echo -e "${RED}=== 에러 로그 ===${NC}"
        tail -n "$lines" "$LOG_DIR/humiro-service-error.log"
    else
        echo -e "${YELLOW}에러 로그 파일이 없습니다.${NC}"
    fi
}

# 함수: 실시간 로그 보기
follow_logs() {
    echo -e "${BLUE}[로그] 실시간 로그 모니터링 (Ctrl+C로 종료)${NC}"
    echo ""
    
    if [ -f "$LOG_DIR/humiro-service.log" ]; then
        tail -f "$LOG_DIR/humiro-service.log"
    else
        echo -e "${YELLOW}로그 파일이 없습니다. 서비스를 시작하세요.${NC}"
    fi
}

# 함수: 서비스 제거
remove_service() {
    echo -e "${YELLOW}[제거] 서비스 제거 중...${NC}"
    
    if sudo systemctl is-active --quiet "$SERVICE_NAME"; then
        echo -e "${YELLOW}서비스가 실행 중입니다. 먼저 중지합니다...${NC}"
        stop_service
    fi
    
    if sudo systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        disable_service
    fi
    
    if [ -f "$SYSTEMD_SERVICE" ]; then
        sudo rm "$SYSTEMD_SERVICE"
        sudo systemctl daemon-reload
        echo -e "${GREEN}✓ 서비스 파일 제거 완료${NC}"
    else
        echo -e "${YELLOW}서비스 파일이 없습니다.${NC}"
    fi
}

# 함수: 도움말
show_help() {
    echo -e "${BLUE}Humiro Fire Suppression Service Control${NC}"
    echo ""
    echo "사용법: $0 [명령어]"
    echo ""
    echo "명령어:"
    echo "  install      - 서비스 파일 설치 (systemd에 등록)"
    echo "  enable       - 부팅 시 자동 시작 활성화"
    echo "  disable      - 부팅 시 자동 시작 비활성화"
    echo "  start        - 서비스 시작"
    echo "  stop         - 서비스 중지"
    echo "  restart      - 서비스 재시작"
    echo "  status       - 서비스 상태 확인"
    echo "  logs [N]     - 최근 N줄 로그 보기 (기본: 50)"
    echo "  follow       - 실시간 로그 모니터링"
    echo "  remove       - 서비스 제거"
    echo "  help         - 이 도움말 표시"
    echo ""
    echo "예제:"
    echo "  $0 install    # 서비스 설치"
    echo "  $0 enable     # 부팅 시 자동 시작 활성화"
    echo "  $0 start      # 서비스 시작"
    echo "  $0 logs 100   # 최근 100줄 로그 보기"
    echo "  $0 follow     # 실시간 로그 보기"
    echo ""
    echo "개발 중 사용 팁:"
    echo "  - 개발 중에는 'enable' 하지 않고 'start'만 사용하는 것을 권장"
    echo "  - 테스트 후 문제없으면 'enable'로 부팅 시 자동 시작 설정"
    echo "  - 로그는 $LOG_DIR 에 저장됩니다"
}

# 메인 로직
case "${1:-help}" in
    install)
        install_service
        ;;
    enable)
        enable_service
        ;;
    disable)
        disable_service
        ;;
    start)
        start_service
        ;;
    stop)
        stop_service
        ;;
    restart)
        restart_service
        ;;
    status)
        status_service
        ;;
    logs)
        show_logs "${2:-50}"
        ;;
    follow)
        follow_logs
        ;;
    remove)
        remove_service
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo -e "${RED}알 수 없는 명령어: $1${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac

