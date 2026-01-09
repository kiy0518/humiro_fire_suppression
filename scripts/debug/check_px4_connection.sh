#!/bin/bash
# =============================================================================
# PX4 연결 상태 확인 스크립트
# =============================================================================

# 프로젝트 환경 로드
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# device_config.env 로드
CONFIG_FILE="$PROJECT_ROOT/config/device_config.env"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
else
    echo "WARNING: device_config.env not found, using defaults"
    ETH0_IP="10.0.0.31"
    FC_IP="10.0.0.32"
    XRCE_DDS_PORT="8888"
fi

# IP를 정수로 변환하는 함수
ip_to_int() {
    local ip=$1
    IFS='.' read -r a b c d <<< "$ip"
    echo $((a * 256 * 256 * 256 + b * 256 * 256 + c * 256 + d))
}

EXPECTED_IP_INT=$(ip_to_int "$ETH0_IP")

echo "=== PX4 연결 상태 확인 ==="
echo ""
echo "설정값 (device_config.env):"
echo "  VIM4 eth0 IP: $ETH0_IP"
echo "  FC IP: $FC_IP"
echo "  XRCE-DDS Port: $XRCE_DDS_PORT"
echo "  IP (정수): $EXPECTED_IP_INT"
echo ""

# 1. micro-ROS Agent 실행 상태
echo "[1] micro-ROS Agent 실행 상태:"
if pgrep -f "micro_ros_agent" > /dev/null; then
    echo "  ✓ micro-ROS Agent 실행 중"
    ps aux | grep -E "(micro_ros|uxrce)" | grep -v grep | head -1
else
    echo "  ✗ micro-ROS Agent 실행 안 됨"
    echo "  → 실행: ./scripts/runtime/start_micro_ros_agent_wrapper.sh"
fi
echo ""

# 2. 네트워크 연결 상태
echo "[2] 네트워크 연결 상태:"
CURRENT_ETH0_IP=$(ip addr show eth0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
if [ -n "$CURRENT_ETH0_IP" ]; then
    if [ "$CURRENT_ETH0_IP" = "$ETH0_IP" ]; then
        echo "  ✓ eth0 IP: $CURRENT_ETH0_IP (설정값과 일치)"
    else
        echo "  ⚠ eth0 IP: $CURRENT_ETH0_IP (설정값 $ETH0_IP와 다름)"
    fi
    echo "  → FC IP 확인: $FC_IP"
else
    echo "  ✗ eth0 인터페이스 없음"
fi
echo ""

# 3. micro-ROS Agent 포트 확인
echo "[3] micro-ROS Agent 포트 ($XRCE_DDS_PORT):"
if netstat -un 2>/dev/null | grep -q ":$XRCE_DDS_PORT" || ss -un 2>/dev/null | grep -q ":$XRCE_DDS_PORT"; then
    echo "  ✓ 포트 $XRCE_DDS_PORT 리스닝 중"
else
    echo "  ✗ 포트 $XRCE_DDS_PORT 리스닝 안 됨"
fi
echo ""

# 4. ROS2 토픽 확인
echo "[4] ROS2 토픽 상태:"
PX4_TOPICS=$(ros2 topic list 2>/dev/null | grep "^/fmu/" | wc -l)
if [ "$PX4_TOPICS" -gt 0 ]; then
    echo "  ✓ PX4 토픽 발견: $PX4_TOPICS 개"
    echo "  → 토픽은 존재하지만 메시지가 발행되지 않을 수 있음"
else
    echo "  ✗ PX4 토픽 없음"
fi
echo ""

# 5. 토픽 발행자 확인
echo "[5] 주요 토픽 발행자 확인:"
for topic in "/fmu/out/vehicle_status_v1" "/fmu/out/battery_status" "/fmu/out/vehicle_attitude"; do
    pub_count=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    if [ -n "$pub_count" ]; then
        if [ "$pub_count" -gt 0 ]; then
            echo "  ✓ $topic: Publisher $pub_count 개"
        else
            echo "  ✗ $topic: Publisher 없음"
        fi
    fi
done
echo ""

# 6. 메시지 수신 테스트
echo "[6] 메시지 수신 테스트 (5초, sensor_data QoS):"
echo "  → /fmu/out/vehicle_status_v1 테스트 중..."
timeout 5 ros2 topic echo /fmu/out/vehicle_status_v1 --qos-profile sensor_data --once 2>&1 | head -10
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "  ✓ 메시지 수신 성공"
else
    echo "  ✗ 메시지 수신 실패 (PX4가 데이터를 보내지 않음)"
    echo ""
    echo "  가능한 원인:"
    echo "    1. PX4가 시동이 안 걸림 (가장 가능성 높음)"
    echo "       → QGC에서 PX4 상태는 보이지만 시동이 안 걸렸을 수 있음"
    echo "    2. PX4의 uXRCE-DDS 파라미터 확인 필요:"
    echo "       → uXRCE-DDS_DOM_ID = 0 (확인 필요)"
    echo "       → uxrce_dds_ag_ip = $EXPECTED_IP_INT ($ETH0_IP)"
    echo "       → UXRCE_DDS_PRT = $XRCE_DDS_PORT"
    echo "    3. QoS Durability 불일치 (코드 수정 완료, 재빌드 필요)"
    echo "    4. micro-ROS Agent 재시작 필요"
    echo ""
    echo "  해결 방법:"
    echo "    1. PX4 시동 확인 (QGC에서 확인)"
    echo "    2. QGC에서 uXRCE-DDS_DOM_ID = 0 확인"
    echo "    3. micro-ROS Agent 재시작:"
    echo "       pkill -f micro_ros_agent"
    echo "       ./scripts/runtime/start_micro_ros_agent_wrapper.sh &"
fi
echo ""

echo "=== 확인 완료 ==="
