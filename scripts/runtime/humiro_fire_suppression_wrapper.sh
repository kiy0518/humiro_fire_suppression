#!/bin/bash
# =============================================================================
# Humiro Fire Suppression 실행 wrapper 스크립트
# =============================================================================
# systemd 서비스에서 사용하기 위한 환경 설정 포함 실행 스크립트
# =============================================================================

# 프로젝트 루트
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 환경 변수 로드
if [ -f "$PROJECT_ROOT/config/device_config.env" ]; then
    source "$PROJECT_ROOT/config/device_config.env"
fi

# ROS2 환경 설정
export HOME=/home/khadas
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_NAMESPACE=${ROS_NAMESPACE:-drone1}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastdds_eth0_only.xml"

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

# 실행
cd "$PROJECT_ROOT/application/build"
exec "$EXECUTABLE" "$@"


