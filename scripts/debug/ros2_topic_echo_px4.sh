#!/bin/bash
# =============================================================================
# PX4 토픽 echo 스크립트 (올바른 QoS 설정)
# =============================================================================
# PX4는 BEST_EFFORT QoS를 사용하므로 ros2 topic echo도 BEST_EFFORT로 설정해야 함
# =============================================================================

if [ -z "$1" ]; then
    echo "사용법: $0 <토픽명> [옵션]"
    echo ""
    echo "예제:"
    echo "  $0 /fmu/out/vehicle_attitude"
    echo "  $0 /fmu/out/vehicle_status_v1"
    echo "  $0 /fmu/out/battery_status"
    echo "  $0 /fmu/out/vehicle_attitude --once    # 한 번만 출력"
    echo ""
    echo "PX4 토픽 목록:"
    ros2 topic list | grep "^/fmu/"
    exit 1
fi

TOPIC="$1"
shift
OPTIONS="$@"

# sensor_data 프로필은 BEST_EFFORT를 사용 (PX4와 호환)
# 또는 명시적으로 best_effort 지정
ros2 topic echo "$TOPIC" --qos-profile sensor_data $OPTIONS

