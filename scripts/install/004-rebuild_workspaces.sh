#!/bin/bash
# =============================================================================
# 004-rebuild_workspaces.sh - 워크스페이스 재빌드
# =============================================================================
# 용도: 프로젝트 구조 변경 후 워크스페이스를 새 경로에서 재빌드
# 실행: ./004-rebuild_workspaces.sh (sudo 불필요)
#
# 주의: 이 스크립트는 기존 빌드 결과물을 삭제하고 재빌드합니다
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"

# sudo 실행 시에도 실제 사용자 홈 디렉토리 사용
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

echo "=========================================="
echo "워크스페이스 재빌드"
echo "=========================================="
echo ""
echo "프로젝트 루트: $PROJECT_ROOT"
echo "사용자: $REAL_USER"
echo ""

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ROS2 환경 확인
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}✗${NC} ROS2 Humble이 설치되지 않았습니다."
    echo "  설치: sudo apt install ros-humble-desktop"
    exit 1
fi

source /opt/ros/humble/setup.bash

# -----------------------------------------------------------------------------
# 1. Micro-ROS 워크스페이스 재빌드
# -----------------------------------------------------------------------------
echo -e "${BLUE}[1/3] Micro-ROS 워크스페이스 재빌드${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ ! -d "$MICRO_ROS_WS/src" ]; then
    echo -e "${YELLOW}⚠${NC} Micro-ROS 소스가 없습니다."
    echo "  소스 다운로드 필요:"
    echo "    cd $MICRO_ROS_WS"
    echo "    mkdir -p src"
    echo "    cd src"
    echo "    git clone https://github.com/micro-ROS/micro-ROS-Agent.git"
    echo ""
    read -p "계속하시겠습니까? (y/n): " CONTINUE
    if [ "$CONTINUE" != "y" ] && [ "$CONTINUE" != "Y" ]; then
        exit 1
    fi
else
    echo "  소스 디렉토리: $MICRO_ROS_WS/src"
    
    # 기존 빌드 결과물 삭제
    if [ -d "$MICRO_ROS_WS/build" ] || [ -d "$MICRO_ROS_WS/install" ]; then
        echo "  기존 빌드 결과물 삭제 중..."
        rm -rf "$MICRO_ROS_WS/build" "$MICRO_ROS_WS/install" "$MICRO_ROS_WS/log"
        echo -e "  ${GREEN}✓${NC} 삭제 완료"
    fi
    
    # 재빌드
    echo "  빌드 중... (시간이 걸릴 수 있습니다)"
    cd "$MICRO_ROS_WS"
    colcon build --symlink-install 2>&1 | tee /tmp/micro_ros_build.log | grep -E "Starting|Finished|packages|warnings|errors" || true
    
    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        echo -e "  ${GREEN}✓${NC} Micro-ROS 워크스페이스 빌드 완료"
    else
        echo -e "  ${RED}✗${NC} 빌드 실패. 로그 확인: /tmp/micro_ros_build.log"
        exit 1
    fi
fi

echo ""

# -----------------------------------------------------------------------------
# 2. PX4 ROS2 워크스페이스 재빌드
# -----------------------------------------------------------------------------
echo -e "${BLUE}[2/3] PX4 ROS2 워크스페이스 재빌드${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ ! -d "$PX4_ROS2_WS/src" ]; then
    echo -e "${YELLOW}⚠${NC} PX4 ROS2 소스가 없습니다."
    echo "  소스 다운로드 필요:"
    echo "    cd $PX4_ROS2_WS"
    echo "    mkdir -p src"
    echo "    cd src"
    echo "    git clone https://github.com/PX4/px4_msgs.git -b ros2"
    echo ""
    read -p "계속하시겠습니까? (y/n): " CONTINUE
    if [ "$CONTINUE" != "y" ] && [ "$CONTINUE" != "Y" ]; then
        exit 1
    fi
else
    echo "  소스 디렉토리: $PX4_ROS2_WS/src"
    
    # 기존 빌드 결과물 삭제
    if [ -d "$PX4_ROS2_WS/build" ] || [ -d "$PX4_ROS2_WS/install" ]; then
        echo "  기존 빌드 결과물 삭제 중..."
        rm -rf "$PX4_ROS2_WS/build" "$PX4_ROS2_WS/install" "$PX4_ROS2_WS/log"
        echo -e "  ${GREEN}✓${NC} 삭제 완료"
    fi
    
    # 재빌드
    echo "  빌드 중..."
    cd "$PX4_ROS2_WS"
    colcon build --symlink-install 2>&1 | tee /tmp/px4_ros2_build.log | grep -E "Starting|Finished|packages|warnings|errors" || true
    
    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        echo -e "  ${GREEN}✓${NC} PX4 ROS2 워크스페이스 빌드 완료"
    else
        echo -e "  ${RED}✗${NC} 빌드 실패. 로그 확인: /tmp/px4_ros2_build.log"
        exit 1
    fi
fi

echo ""

# -----------------------------------------------------------------------------
# 3. 소유권 설정
# -----------------------------------------------------------------------------
echo -e "${BLUE}[3/3] 소유권 설정${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

chown -R $REAL_USER:$REAL_USER "$MICRO_ROS_WS" 2>/dev/null || true
chown -R $REAL_USER:$REAL_USER "$PX4_ROS2_WS" 2>/dev/null || true
echo -e "  ${GREEN}✓${NC} 소유권 설정 완료"

echo ""
echo "=========================================="
echo -e "${GREEN}워크스페이스 재빌드 완료!${NC}"
echo "=========================================="
echo ""
echo "다음 단계:"
echo "  1. 환경 변수 로드: source $PROJECT_ROOT/setup_env.sh"
echo "  2. 설정 적용: sudo ./scripts/install/003-apply_config.sh"
echo "  3. 연결 확인: ./scripts/check/101-check_px4_connection.sh"
echo ""
