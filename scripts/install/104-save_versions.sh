#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# =============================================================================
# 104-save_versions.sh - 설치된 패키지 버전 저장
# =============================================================================
# 용도: 현재 설치된 패키지들의 버전을 versions.env에 저장
#       개발 완료 후 안정적인 버전을 기록하여 재현 가능하게 함
#
# 실행: ./104-save_versions.sh
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION_FILE="$SCRIPT_DIR/versions.env"

# sudo 실행 시에도 실제 사용자 홈 디렉토리 사용
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

echo "=========================================="
echo "설치된 패키지 버전 저장"
echo "=========================================="
echo ""

# Git 커밋 해시 가져오기 함수
get_git_hash() {
    local dir=$1
    if [ -d "$dir/.git" ]; then
        cd "$dir" && git rev-parse HEAD 2>/dev/null
    else
        echo ""
    fi
}

# Git 태그 가져오기 함수
get_git_tag() {
    local dir=$1
    if [ -d "$dir/.git" ]; then
        cd "$dir" && git describe --tags --exact-match 2>/dev/null || git rev-parse --short HEAD 2>/dev/null
    else
        echo ""
    fi
}

# -----------------------------------------------------------------------------
# 버전 정보 수집
# -----------------------------------------------------------------------------
echo "버전 정보 수집 중..."

# 시스템 정보
UBUNTU_VER=$(lsb_release -rs 2>/dev/null || echo "unknown")
KERNEL_VER=$(uname -r)
ARCH=$(uname -m)
echo "  시스템: Ubuntu $UBUNTU_VER, Kernel $KERNEL_VER, $ARCH"

# ROS2 버전
ROS2_VER=""
# ROS2 버전 (Humble)
if [ -d "/opt/ros/humble" ]; then
    ROS2_DISTRO="humble"
    ROS2_PKG_VER=$(dpkg -l ros-humble-desktop 2>/dev/null | grep "^ii" | awk '{print $3}')
    if [ -z "$ROS2_PKG_VER" ]; then
        ROS2_PKG_VER=$(dpkg -l ros-humble-ros-base 2>/dev/null | grep "^ii" | awk '{print $3}')
    fi
    ROS2_VER="$ROS2_DISTRO"
    echo "  ROS2: $ROS2_DISTRO (패키지: $ROS2_PKG_VER)"
else
    ROS2_VER=""
    echo "  ROS2: 미설치"
fi

# micro-ROS Agent 버전
MICRO_ROS_VER=""
MICRO_ROS_DIR="$REAL_HOME/workspaces/micro_ros_ws/src/micro-ROS-Agent"
if [ -d "$MICRO_ROS_DIR" ]; then
    MICRO_ROS_VER=$(get_git_hash "$MICRO_ROS_DIR")
    echo "  micro-ROS Agent: ${MICRO_ROS_VER:0:12}"
else
    # 대안: uros 디렉토리 확인
    MICRO_ROS_DIR="$REAL_HOME/workspaces/micro_ros_ws/src/uros"
    if [ -d "$MICRO_ROS_DIR" ]; then
        MICRO_ROS_VER=$(get_git_hash "$MICRO_ROS_DIR")
        echo "  micro-ROS Agent: ${MICRO_ROS_VER:0:12}"
    else
        echo "  micro-ROS Agent: 소스 없음"
    fi
fi

# mavlink-router 버전
MAVLINK_VER=""
MAVLINK_DIR="$REAL_HOME/mavlink-router"
if [ -d "$MAVLINK_DIR" ]; then
    MAVLINK_VER=$(get_git_hash "$MAVLINK_DIR")
    echo "  mavlink-router: ${MAVLINK_VER:0:12}"
else
    echo "  mavlink-router: 소스 없음"
fi

# PX4 msgs 버전
PX4_MSGS_VER=""
PX4_MSGS_DIR="$REAL_HOME/workspaces/px4_ros2_ws/src/px4_msgs"
if [ -d "$PX4_MSGS_DIR" ]; then
    PX4_MSGS_VER=$(get_git_hash "$PX4_MSGS_DIR")
    echo "  PX4 msgs: ${PX4_MSGS_VER:0:12}"
else
    echo "  PX4 msgs: 소스 없음"
fi

# GStreamer 버전
GSTREAMER_VER=$(dpkg -l gstreamer1.0-tools 2>/dev/null | grep "^ii" | awk '{print $3}' || echo "")
if [ -n "$GSTREAMER_VER" ]; then
    echo "  GStreamer: $GSTREAMER_VER"
fi

# dnsmasq 버전
DNSMASQ_VER=$(dpkg -l dnsmasq 2>/dev/null | grep "^ii" | awk '{print $3}' || echo "")
if [ -n "$DNSMASQ_VER" ]; then
    echo "  dnsmasq: $DNSMASQ_VER"
fi

# -----------------------------------------------------------------------------
# 기존 PX4 펌웨어 버전 유지
# -----------------------------------------------------------------------------
EXISTING_PX4_VERSION=""
if [ -f "$VERSION_FILE" ]; then
    EXISTING_PX4_VERSION=$(grep "^PX4_FIRMWARE_VERSION=" "$VERSION_FILE" 2>/dev/null | cut -d'"' -f2)
fi
PX4_FW_VER="${EXISTING_PX4_VERSION:-v1.16.0}"
echo "  PX4 펌웨어: $PX4_FW_VER (참조용)"

# -----------------------------------------------------------------------------
# versions.env 파일 생성
# -----------------------------------------------------------------------------
echo ""
echo "versions.env 파일 저장 중..."

cat > "$VERSION_FILE" << EOF
# =============================================================================
# ClusterDrone - 패키지 버전 관리 파일
# =============================================================================
# 이 파일은 설치된 패키지들의 버전을 관리합니다.
# 
# 사용법:
#   - 새 시스템에 동일한 버전을 설치하려면 이 파일을 복사하세요.
#   - 버전을 비워두면 최신 버전이 설치됩니다.
#   - 버전 저장: ./104-save_versions.sh
# =============================================================================

# -----------------------------------------------------------------------------
# 버전 정보 생성 시간
# -----------------------------------------------------------------------------
VERSION_SAVED_DATE="$(date '+%Y-%m-%d %H:%M:%S')"

# -----------------------------------------------------------------------------
# PX4 펌웨어 버전 (참조용 - FC에 설치됨)
# -----------------------------------------------------------------------------
# PX4 펌웨어는 QGroundControl을 통해 FC에 직접 설치
# 이 버전은 호환성 확인을 위한 참조용
PX4_FIRMWARE_VERSION="$PX4_FW_VER"

# -----------------------------------------------------------------------------
# ROS2 버전
# -----------------------------------------------------------------------------
# ROS2 배포판 (humble, iron, jazzy 등)
ROS2_DISTRO="$ROS2_DISTRO"
# 패키지 버전 (참조용 - apt 저장소에서 관리되어 특정 버전 설치 어려움)
ROS2_PKG_VERSION="$ROS2_PKG_VER"

# -----------------------------------------------------------------------------
# Git 저장소 버전 (commit hash)
# -----------------------------------------------------------------------------
MICRO_ROS_AGENT_VERSION="$MICRO_ROS_VER"
MAVLINK_ROUTER_VERSION="$MAVLINK_VER"
PX4_MSGS_VERSION="$PX4_MSGS_VER"

# -----------------------------------------------------------------------------
# 주요 apt 패키지 버전
# -----------------------------------------------------------------------------
GSTREAMER_VERSION="$GSTREAMER_VER"
DNSMASQ_VERSION="$DNSMASQ_VER"

# -----------------------------------------------------------------------------
# 설치된 시스템 정보
# -----------------------------------------------------------------------------
UBUNTU_VERSION="$UBUNTU_VER"
KERNEL_VERSION="$KERNEL_VER"
ARCHITECTURE="$ARCH"
EOF

echo ""
echo "=========================================="
echo "버전 저장 완료!"
echo "=========================================="
echo ""
echo "저장된 파일: $VERSION_FILE"
echo ""
echo "저장된 버전 정보:"
echo "  - ROS2: $ROS2_DISTRO"
echo "  - micro-ROS Agent: ${MICRO_ROS_VER:0:12}"
echo "  - mavlink-router: ${MAVLINK_VER:0:12}"
echo "  - PX4 msgs: ${PX4_MSGS_VER:0:12}"
echo "  - 시스템: Ubuntu $UBUNTU_VER ($ARCH)"
echo ""
echo "이 버전으로 다른 시스템에 설치하려면:"
echo "  1. versions.env 파일을 새 시스템에 복사"
echo "  2. sudo ./000-install_all.sh 실행"
echo ""
