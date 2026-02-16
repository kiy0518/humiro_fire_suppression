#!/bin/bash
# =============================================================================
# FC Bridge 실행 wrapper 스크립트
# =============================================================================
# Domain 0 (loopback 전용) — /fmu/* 토픽을 UDP IPC로 변환
# micro-ros-agent와 같은 Domain 0에서 실행, 메인앱과 UDP로 통신
# =============================================================================

# 프로젝트 루트
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 환경 변수 로드
if [ -f "$PROJECT_ROOT/config/device_config.env" ]; then
    set -a
    source "$PROJECT_ROOT/config/device_config.env"
    set +a
fi

# ROS2 환경 설정
export HOME=/home/khadas
export ROS_DOMAIN_ID=0
export ROS_NAMESPACE=${ROS_NAMESPACE:-drone${DRONE_ID:-1}}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# micro-ros-agent와 동일한 DDS 프로파일 사용 (eth0+loopback)
# loopback만 사용하면 agent와 DDS discovery 불가
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastdds_agent_eth0.xml"

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
export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"

if [ -d "$PX4_ROS2_WS/install/px4_msgs/lib" ]; then
    export LD_LIBRARY_PATH="$PX4_ROS2_WS/install/px4_msgs/lib:$LD_LIBRARY_PATH"
fi

# IPC 포트 (환경 변수로 오버라이드 가능)
export FC_BRIDGE_STATE_PORT=${FC_BRIDGE_STATE_PORT:-17001}
export FC_BRIDGE_COMMAND_PORT=${FC_BRIDGE_COMMAND_PORT:-17002}

# PX4 토픽 네임스페이스 (FC 모드에서는 비어있음 — micro-ros-agent가 네임스페이스 미사용)
# SITL 편대 등에서 /droneN/fmu/* 형태일 경우 FC_BRIDGE_PX4_NS=/droneN 으로 설정
export FC_BRIDGE_PX4_NS=${FC_BRIDGE_PX4_NS:-}

# 실행 파일 경로
EXECUTABLE="$PROJECT_ROOT/navigation/build/offboard_control/fc_bridge"

echo "========================================"
echo "  FC Bridge Wrapper"
echo "  Domain: ${ROS_DOMAIN_ID}"
echo "  PX4 Topic NS: '${FC_BRIDGE_PX4_NS:-<none>}'"
echo "  FastDDS: eth0+loopback (agent_eth0)"
echo "  State Port: ${FC_BRIDGE_STATE_PORT}"
echo "  Command Port: ${FC_BRIDGE_COMMAND_PORT}"
echo "========================================"

# 실행 (ROS2 노드 네임스페이스 없이 실행 — PX4 토픽 접두사는 FC_BRIDGE_PX4_NS로 제어)
exec "$EXECUTABLE"
