#!/bin/bash
# ==============================================================================
# PX4 SITL 멀티 드론 시뮬레이션 시작 스크립트
# WSL2 Ubuntu에서 실행
# ==============================================================================

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 기본 설정
NUM_DRONES=${1:-1}
GCS_IP=${2:-""}
PX4_DIR="$HOME/PX4-Autopilot"
BASE_MAV_PORT=14540
GCS_PORT=14550

# 사용법
usage() {
    echo "사용법: $0 [드론수] [GCS_IP]"
    echo ""
    echo "인자:"
    echo "  드론수    시뮬레이션할 드론 수 (기본: 1, 최대: 3)"
    echo "  GCS_IP    QGroundControl이 실행 중인 PC IP (선택사항)"
    echo ""
    echo "예시:"
    echo "  $0                    # 드론 1대, 로컬 QGC"
    echo "  $0 3                  # 드론 3대, 로컬 QGC"
    echo "  $0 3 192.168.100.4    # 드론 3대, 원격 QGC"
    exit 1
}

# 도움말
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    usage
fi

# 드론 수 검증
if [ "$NUM_DRONES" -lt 1 ] || [ "$NUM_DRONES" -gt 3 ]; then
    echo -e "${RED}오류: 드론 수는 1~3 사이여야 합니다.${NC}"
    exit 1
fi

echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}   PX4 SITL 시뮬레이션 시작${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""
echo -e "  드론 수: ${GREEN}$NUM_DRONES${NC}"
echo -e "  GCS IP:  ${GREEN}${GCS_IP:-localhost}${NC}"
echo ""

# PX4 디렉토리 확인
if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}오류: PX4-Autopilot 디렉토리를 찾을 수 없습니다: $PX4_DIR${NC}"
    exit 1
fi

# 기존 프로세스 정리
echo -e "${YELLOW}[1/4] 기존 프로세스 정리...${NC}"
pkill -9 px4 2>/dev/null || true
pkill -9 gz 2>/dev/null || true
pkill -9 ruby 2>/dev/null || true
sleep 2
echo -e "  ${GREEN}✓${NC} 완료"

# 환경변수 설정
echo -e "${YELLOW}[2/4] 환경변수 설정...${NC}"
export DISPLAY=:0
export HEADLESS=1
export PX4_SIM_HOST_ADDR=0.0.0.0
echo -e "  ${GREEN}✓${NC} HEADLESS 모드 활성화"

# MAVLink 포트 포워딩 (socat)
if [ -n "$GCS_IP" ]; then
    echo -e "${YELLOW}[3/4] MAVLink 포트 포워딩 설정...${NC}"

    # socat 설치 확인
    if ! command -v socat &> /dev/null; then
        echo -e "  ${YELLOW}socat 설치 중...${NC}"
        sudo apt install -y socat > /dev/null 2>&1
    fi

    # 기존 socat 종료
    pkill -f "socat.*14550" 2>/dev/null || true

    # 각 드론별 포트 포워딩
    for i in $(seq 1 $NUM_DRONES); do
        LOCAL_PORT=$((BASE_MAV_PORT + i - 1))
        echo -e "  드론 $i: UDP $LOCAL_PORT → $GCS_IP:$GCS_PORT"
    done

    # GCS로 MAVLink 포워딩 (백그라운드)
    socat UDP4-LISTEN:$GCS_PORT,fork,reuseaddr UDP4:$GCS_IP:$GCS_PORT &
    SOCAT_PID=$!
    echo -e "  ${GREEN}✓${NC} 포트 포워딩 시작 (PID: $SOCAT_PID)"
else
    echo -e "${YELLOW}[3/4] 로컬 QGC 연결 모드${NC}"
    echo -e "  ${GREEN}✓${NC} localhost:$GCS_PORT 에서 대기"
fi

# SITL 실행
echo -e "${YELLOW}[4/4] PX4 SITL 시작...${NC}"
cd "$PX4_DIR"

if [ "$NUM_DRONES" -eq 1 ]; then
    # 단일 드론
    echo -e "  단일 드론 모드"
    echo ""
    echo -e "${BLUE}=====================================${NC}"
    echo -e "${GREEN}   시뮬레이션 시작됨!${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo ""
    echo -e "QGC 연결 포트: UDP ${GREEN}$GCS_PORT${NC}"
    echo -e "PX4 콘솔에서 테스트:"
    echo -e "  ${YELLOW}param set COM_RCL_EXCEPT 4${NC}"
    echo -e "  ${YELLOW}param set NAV_RCL_ACT 0${NC}"
    echo -e "  ${YELLOW}commander arm -f${NC}"
    echo -e "  ${YELLOW}commander takeoff${NC}"
    echo ""
    echo -e "종료: ${YELLOW}Ctrl+C${NC}"
    echo ""

    make px4_sitl gz_x500
else
    # 멀티 드론
    echo -e "  멀티 드론 모드 ($NUM_DRONES대)"

    # 멀티 드론 실행 스크립트 확인
    MULTI_SCRIPT="$PX4_DIR/Tools/simulation/gz/sitl_multiple_run.sh"

    if [ -f "$MULTI_SCRIPT" ]; then
        echo ""
        echo -e "${BLUE}=====================================${NC}"
        echo -e "${GREEN}   멀티 드론 시뮬레이션 시작!${NC}"
        echo -e "${BLUE}=====================================${NC}"
        echo ""
        echo -e "드론 MAVLink 포트:"
        for i in $(seq 1 $NUM_DRONES); do
            PORT=$((BASE_MAV_PORT + i - 1))
            echo -e "  드론 $i: UDP ${GREEN}$PORT${NC}"
        done
        echo ""
        echo -e "종료: ${YELLOW}Ctrl+C${NC}"
        echo ""

        bash "$MULTI_SCRIPT" -n $NUM_DRONES -m x500
    else
        # 수동으로 멀티 인스턴스 실행
        echo -e "  멀티 드론 스크립트 없음, 수동 실행..."

        for i in $(seq 1 $NUM_DRONES); do
            INSTANCE=$((i - 1))
            MAV_PORT=$((BASE_MAV_PORT + INSTANCE))

            echo -e "  드론 $i 시작 (instance=$INSTANCE, port=$MAV_PORT)..."

            PX4_SYS_AUTOSTART=4001 \
            PX4_GZ_MODEL_POSE="$((INSTANCE * 3)),0,0,0,0,0" \
            PX4_SIM_MODEL=gz_x500 \
            ./build/px4_sitl_default/bin/px4 -i $INSTANCE &

            sleep 3
        done

        echo ""
        echo -e "${BLUE}=====================================${NC}"
        echo -e "${GREEN}   $NUM_DRONES대 드론 시뮬레이션 시작!${NC}"
        echo -e "${BLUE}=====================================${NC}"
        echo ""
        echo -e "종료: ${YELLOW}pkill -9 px4${NC}"
        echo ""

        # 포그라운드 유지
        wait
    fi
fi
