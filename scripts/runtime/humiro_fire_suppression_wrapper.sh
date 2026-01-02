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

# DDS 구현체 선택: FastRTPS는 배터리 메시지 payload size 문제가 있음 (184바이트 > 183바이트)
# CycloneDX가 설치되어 있으면 사용, 없으면 FastRTPS 사용
if command -v ros2 &> /dev/null && ros2 doctor --report 2>&1 | grep -q "cyclonedx"; then
    export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
    echo "  [DEBUG] DDS 구현체: CycloneDX 사용" >&2
else
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    # FastRTPS 설정: payload size 문제 해결 시도 (배터리 메시지 184바이트 > 기본 183바이트)
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastrtps_profile.xml"
    # FastRTPS 환경 변수: payload size 제한 해제 시도
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/fastrtps_profile.xml"
    # ROS2 환경 변수로 FastRTPS 로그 레벨 조정 (에러 무시)
    export RCUTILS_LOGGING_SEVERITY=WARN  # ERROR 메시지 숨김
    echo "  [DEBUG] DDS 구현체: FastRTPS 사용 (payload size 제한 있음, 에러 무시 모드)" >&2
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


