#!/bin/bash
# ==============================================================================
# PX4 SITL 3대 드론 통합 실행 스크립트
# 사용법: ~/start_sitl_all.sh
# 종료:   Ctrl+C (3개 인스턴스 모두 종료)
# 로그:   ~/sitl_logs/drone{1,2,3}.log
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

VIM4_IPS=("192.168.100.11" "192.168.100.21" "192.168.100.31")
INSTANCES=(0 1 2)
MODEL_POSES=("" "-10,0,0,0,0,0" "10,0,0,0,0,0")
PIDS=()

# 로그 디렉토리
mkdir -p ~/sitl_logs

cleanup() {
    echo ""
    echo -e "${YELLOW}모든 PX4 인스턴스 종료 중...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
        fi
    done
    sleep 1
    pkill -9 px4 2>/dev/null || true
    pkill -9 gz 2>/dev/null || true
    pkill -9 ruby 2>/dev/null || true
    echo -e "${GREEN}종료 완료${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}  PX4 SITL 3대 드론 통합 실행${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# [1] 기존 프로세스 정리
echo -e "${YELLOW}[1/4] 기존 프로세스 정리...${NC}"
pkill -9 px4 2>/dev/null || true
pkill -9 gz 2>/dev/null || true
pkill -9 ruby 2>/dev/null || true
sleep 2
echo -e "  ${GREEN}✓${NC} 완료"

# [2] VIM4 연결 테스트
echo -e "${YELLOW}[2/4] VIM4 연결 테스트...${NC}"
ALL_OK=true
for i in 0 1 2; do
    ip=${VIM4_IPS[$i]}
    drone_num=$((i+1))
    if ping -c 1 -W 2 "$ip" > /dev/null 2>&1; then
        echo -e "  ${GREEN}✓${NC} 드론 $drone_num ($ip)"
    else
        echo -e "  ${RED}✗${NC} 드론 $drone_num ($ip) - 연결 불가"
        ALL_OK=false
    fi
done
if [ "$ALL_OK" = false ]; then
    echo -e "${RED}일부 VIM4에 연결할 수 없습니다. 계속하시겠습니까? (y/N)${NC}"
    read -r ans
    if [ "$ans" != "y" ] && [ "$ans" != "Y" ]; then
        exit 1
    fi
fi

# [3] ROS2 환경
echo -e "${YELLOW}[3/4] ROS2 환경 설정...${NC}"
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0
echo -e "  ${GREEN}✓${NC} ROS_DOMAIN_ID=0"

# [4] PX4 SITL 시작
echo -e "${YELLOW}[4/4] PX4 SITL 인스턴스 시작...${NC}"
echo ""

cd ~/PX4-Autopilot

# 드론 1 (인스턴스 0) - Gazebo 서버도 함께 시작
echo -e "  ${BLUE}드론 1${NC} (인스턴스 0, ${VIM4_IPS[0]}) 시작..."
PX4_SYS_AUTOSTART=4001 PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 ./build/px4_sitl_default/bin/px4 -i 0 > ~/sitl_logs/drone1.log 2>&1 &
PIDS+=($!)
echo -e "  ${GREEN}✓${NC} 드론 1 PID: ${PIDS[0]}"

# Gazebo 초기화 대기
echo -e "  ${YELLOW}Gazebo 초기화 대기 (10초)...${NC}"
sleep 10

# 드론 2 (인스턴스 1)
echo -e "  ${BLUE}드론 2${NC} (인스턴스 1, ${VIM4_IPS[1]}) 시작..."
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-10,0,0,0,0,0" PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 ./build/px4_sitl_default/bin/px4 -i 1 > ~/sitl_logs/drone2.log 2>&1 &
PIDS+=($!)
echo -e "  ${GREEN}✓${NC} 드론 2 PID: ${PIDS[1]}"
sleep 5

# 드론 3 (인스턴스 2)
echo -e "  ${BLUE}드론 3${NC} (인스턴스 2, ${VIM4_IPS[2]}) 시작..."
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="10,0,0,0,0,0" PX4_HOME_LAT=35.905863 PX4_HOME_LON=128.802615 PX4_HOME_ALT=0 ./build/px4_sitl_default/bin/px4 -i 2 > ~/sitl_logs/drone3.log 2>&1 &
PIDS+=($!)
echo -e "  ${GREEN}✓${NC} 드론 3 PID: ${PIDS[2]}"

echo ""
echo -e "${BLUE}======================================${NC}"
echo -e "${GREEN}  3대 드론 SITL 실행 완료!${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""
echo -e "  드론 1 PID: ${PIDS[0]}  로그: ~/sitl_logs/drone1.log"
echo -e "  드론 2 PID: ${PIDS[1]}  로그: ~/sitl_logs/drone2.log"
echo -e "  드론 3 PID: ${PIDS[2]}  로그: ~/sitl_logs/drone3.log"
echo ""
echo -e "  ${YELLOW}실시간 로그: tail -f ~/sitl_logs/drone{1,2,3}.log${NC}"
echo -e "  ${YELLOW}종료: Ctrl+C${NC}"
echo ""

# 프로세스 모니터링 (하나라도 죽으면 알림)
while true; do
    for i in 0 1 2; do
        if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
            echo -e "${RED}[경고] 드론 $((i+1)) (PID ${PIDS[$i]}) 종료됨!${NC}"
            echo -e "  로그 확인: tail -20 ~/sitl_logs/drone$((i+1)).log"
            PIDS[$i]=0
        fi
    done
    # 모두 종료되었으면 스크립트도 종료
    ALL_DEAD=true
    for pid in "${PIDS[@]}"; do
        if [ "$pid" != "0" ] && kill -0 "$pid" 2>/dev/null; then
            ALL_DEAD=false
            break
        fi
    done
    if [ "$ALL_DEAD" = true ]; then
        echo -e "${RED}모든 인스턴스가 종료되었습니다.${NC}"
        exit 1
    fi
    sleep 5
done
