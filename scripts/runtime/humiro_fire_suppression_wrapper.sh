#!/bin/bash
# =============================================================================
# Humiro Fire Suppression 실행 wrapper 스크립트
# =============================================================================
# systemd 서비스에서 사용하기 위한 환경 설정 포함 실행 스크립트
# =============================================================================

# 프로젝트 루트
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 환경 변수 로드 (set -a로 자동 export → C++ std::getenv()에서 접근 가능)
if [ -f "$PROJECT_ROOT/config/device_config.env" ]; then
    set -a
    source "$PROJECT_ROOT/config/device_config.env"
    set +a
fi

# ROS2 환경 설정
export HOME=/home/khadas
# Domain 1: 편대 비행 토픽 (WiFi 전용)
# FC 통신은 fc_bridge (Domain 0, loopback)에서 처리
export ROS_DOMAIN_ID=1
export ROS_NAMESPACE=${ROS_NAMESPACE:-drone1}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastdds_wifi_only.xml"

# ROS2 환경 로드
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 워크스페이스 로드
MICRO_ROS_WS="$PROJECT_ROOT/workspaces/micro_ros_ws"
PX4_ROS2_WS="$PROJECT_ROOT/workspaces/px4_ros2_ws"

if [ -f "$MICRO_ROS_WS/install/setup.bash" ]; then
    source "$MICRO_ROS_WS/install/setup.bash"
fi

if [ -f "$PX4_ROS2_WS/install/setup.bash" ]; then
    source "$PX4_ROS2_WS/install/setup.bash"
fi

# 라이브러리 경로 설정
if [ -d "$MICRO_ROS_WS/install/micro_ros_agent/lib" ]; then
    export LD_LIBRARY_PATH="$MICRO_ROS_WS/install/micro_ros_agent/lib:$LD_LIBRARY_PATH"
fi

export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"

if [ -d "$PX4_ROS2_WS/install/px4_msgs/lib" ]; then
    export LD_LIBRARY_PATH="$PX4_ROS2_WS/install/px4_msgs/lib:$LD_LIBRARY_PATH"
fi

# 실행 파일 경로
EXECUTABLE="$PROJECT_ROOT/application/build/humiro_fire_suppression"

# 바이너리 무결성 검사
if [ ! -f "$EXECUTABLE" ]; then
    echo "[WRAPPER ERROR] 바이너리 파일 없음: $EXECUTABLE" >&2
    exit 1
fi

FILE_SIZE=$(stat -c%s "$EXECUTABLE" 2>/dev/null || echo "0")
if [ "$FILE_SIZE" -lt 1024 ]; then
    echo "[WRAPPER ERROR] 바이너리 손상 (크기: ${FILE_SIZE} bytes). 빌드 중 전원 차단으로 추정." >&2
    echo "[WRAPPER ERROR] 복구 방법: cd $PROJECT_ROOT/application/build && cmake --build . --clean-first -j\$(nproc)" >&2
    exit 1
fi

# ELF 매직 넘버 확인 (첫 4바이트: 7f 45 4c 46 = \x7fELF)
MAGIC=$(xxd -l 4 -p "$EXECUTABLE" 2>/dev/null)
if [ "$MAGIC" != "7f454c46" ]; then
    echo "[WRAPPER ERROR] 바이너리가 유효한 ELF 파일이 아님 (magic: $MAGIC)" >&2
    exit 1
fi

# 실행
cd "$PROJECT_ROOT/application/build"
exec "$EXECUTABLE" "$@"


