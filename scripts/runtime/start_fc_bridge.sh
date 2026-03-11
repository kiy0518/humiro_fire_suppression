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
# DDS 프로파일 자동 선택 (SITL/FC 모드)
# - FC 모드: loopback만 사용 → eth0 멀티캐스트가 FC 이더넷에 부하주는 것 방지
# - SITL 모드: eth0+loopback 사용 → eth0에 FC 없으므로 부하 문제 없음, DDS 정상 통신
MAVLINK_CONF="/etc/mavlink-router/main.conf"
if [ -f "$MAVLINK_CONF" ] && grep -q '\[UdpEndpoint SITL\]\|\[UdpEndpoint PC_SITL\]' "$MAVLINK_CONF"; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastdds_agent_eth0.xml"
    DDS_MODE="eth0+loopback (SITL)"
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastdds_loopback_only.xml"
    DDS_MODE="loopback only (FC 부하 방지)"
fi

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

# DDS 프로파일 재설정 (source 이후 .bashrc 등에서 덮어쓸 수 있으므로)
MAVLINK_CONF="/etc/mavlink-router/main.conf"
if [ -f "$MAVLINK_CONF" ] && grep -q '\[UdpEndpoint SITL\]\|\[UdpEndpoint PC_SITL\]' "$MAVLINK_CONF"; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastdds_agent_eth0.xml"
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastdds_loopback_only.xml"
fi

# 라이브러리 경로 설정
export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"

if [ -d "$PX4_ROS2_WS/install/px4_msgs/lib" ]; then
    export LD_LIBRARY_PATH="$PX4_ROS2_WS/install/px4_msgs/lib:$LD_LIBRARY_PATH"
fi

# IPC 포트 (환경 변수로 오버라이드 가능)
export FC_BRIDGE_STATE_PORT=${FC_BRIDGE_STATE_PORT:-17001}
export FC_BRIDGE_COMMAND_PORT=${FC_BRIDGE_COMMAND_PORT:-17002}

# PX4 토픽 네임스페이스 자동 감지
# - SITL 모드: PX4 SITL이 /droneN/fmu/* 형태로 토픽 생성 → /drone{DRONE_ID} 설정
# - FC 모드: micro-ros-agent가 네임스페이스 미사용 → 빈값
if [ -z "${FC_BRIDGE_PX4_NS+x}" ]; then
    MAVLINK_CONF="/etc/mavlink-router/main.conf"
    if [ -f "$MAVLINK_CONF" ] && grep -q '\[UdpEndpoint SITL\]\|\[UdpEndpoint PC_SITL\]' "$MAVLINK_CONF"; then
        export FC_BRIDGE_PX4_NS="/drone${DRONE_ID:-1}"
    else
        export FC_BRIDGE_PX4_NS=""
    fi
fi

# 실행 파일 경로
EXECUTABLE="$PROJECT_ROOT/navigation/build/offboard_control/fc_bridge"

echo "========================================"
echo "  FC Bridge Wrapper"
echo "  Domain: ${ROS_DOMAIN_ID}"
echo "  PX4 Topic NS: '${FC_BRIDGE_PX4_NS:-<none>}'"
echo "  FastDDS: ${DDS_MODE}"
echo "  State Port: ${FC_BRIDGE_STATE_PORT}"
echo "  Command Port: ${FC_BRIDGE_COMMAND_PORT}"
echo "========================================"

# DDS 프로파일: micro-ros-agent와 동일하게 eth0_only 사용
# (micro-ros-agent가 실제로 eth0에서 DDS 발행하므로 fc_bridge도 eth0 필요)
FINAL_DDS_PROFILE="$PROJECT_ROOT/config/fastdds_eth0_only.xml"
echo "  FastDDS Profile (final): $FINAL_DDS_PROFILE"

# 실행
unset FASTRTPS_DEFAULT_PROFILES_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE="$FINAL_DDS_PROFILE"
exec "$EXECUTABLE"
