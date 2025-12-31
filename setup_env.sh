#!/bin/bash
# 환경 변수 설정 스크립트
# 사용법: source setup_env.sh

# 프로젝트 루트 디렉토리 (이 스크립트의 위치 기준)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PROJECT_ROOT="$SCRIPT_DIR"

# 주요 디렉토리 경로
export PROJECT_DOCS="$PROJECT_ROOT/docs"
export PROJECT_CONFIG="$PROJECT_ROOT/config"
export PROJECT_SCRIPTS="$PROJECT_ROOT/scripts"
export PROJECT_WORKSPACES="$PROJECT_ROOT/workspaces"
export PROJECT_COMMON="$PROJECT_ROOT/common"
export PROJECT_THERMAL="$PROJECT_ROOT/thermal"
export PROJECT_TARGETING="$PROJECT_ROOT/targeting"
export PROJECT_THROWING="$PROJECT_ROOT/throwing_mechanism"
export PROJECT_NAVIGATION="$PROJECT_ROOT/navigation"
export PROJECT_QGC="$PROJECT_ROOT/qgc_custom"
export PROJECT_DEPLOYMENT="$PROJECT_ROOT/deployment"
export PROJECT_TESTS="$PROJECT_ROOT/tests"
export PROJECT_LOGS="$PROJECT_ROOT/logs"
export PROJECT_DATA="$PROJECT_ROOT/data"

# ROS2 Workspace 경로
export MICRO_ROS_WS="$PROJECT_WORKSPACES/micro_ros_ws"
export PX4_ROS2_WS="$PROJECT_WORKSPACES/px4_ros2_ws"
export MAVLINK_ROUTER_DIR="$PROJECT_WORKSPACES/mavlink-router"

# 설정 파일 경로
export DEVICE_CONFIG="$PROJECT_CONFIG/device_config.env"
export VERSIONS_CONFIG="$PROJECT_CONFIG/versions.env"
export THROWING_PARAMS="$PROJECT_CONFIG/throwing_params.yaml"
export CAMERA_CONFIG="$PROJECT_CONFIG/camera_config.yaml"

# 로그 디렉토리
export LOGS_THERMAL="$PROJECT_LOGS/thermal"
export LOGS_NAVIGATION="$PROJECT_LOGS/navigation"
export LOGS_SYSTEM="$PROJECT_LOGS/system"

# 데이터 디렉토리
export DATA_CALIBRATION="$PROJECT_DATA/calibration"
export DATA_RECORDINGS="$PROJECT_DATA/recordings"
export DATA_THERMAL_IMAGES="$PROJECT_DATA/thermal_images"

# 스크립트 경로
export SCRIPTS_INSTALL="$PROJECT_SCRIPTS/install"
export SCRIPTS_CHECK="$PROJECT_SCRIPTS/check"
export SCRIPTS_UPDATE="$PROJECT_SCRIPTS/update"
export SCRIPTS_RUNTIME="$PROJECT_SCRIPTS/runtime"

# Python 경로 추가
export PYTHONPATH="$PROJECT_COMMON/utils:$PROJECT_COMMON/interfaces:$PROJECT_THERMAL/python:$PROJECT_TARGETING/horizontal_alignment:$PROJECT_TARGETING/thermal_tracking:$PROJECT_TARGETING/trajectory_calc:$PROJECT_THROWING/actuator_control:$PROJECT_THROWING/gpio_interface:$PROJECT_NAVIGATION/formation_control:$PROJECT_NAVIGATION/rtk_positioning:$PROJECT_NAVIGATION/collision_avoidance:$PYTHONPATH"

# ROS2 환경 설정 (설치되어 있는 경우)
if [ -f "$MICRO_ROS_WS/install/setup.bash" ]; then
    source "$MICRO_ROS_WS/install/setup.bash"
fi

if [ -f "$PX4_ROS2_WS/install/setup.bash" ]; then
    source "$PX4_ROS2_WS/install/setup.bash"
fi

# LD_LIBRARY_PATH 설정
if [ -d "$MICRO_ROS_WS/install/micro_ros_agent/lib" ]; then
    export LD_LIBRARY_PATH="$MICRO_ROS_WS/install/micro_ros_agent/lib:$LD_LIBRARY_PATH"
fi

echo "✓ 환경 변수 설정 완료"
echo "  PROJECT_ROOT: $PROJECT_ROOT"
