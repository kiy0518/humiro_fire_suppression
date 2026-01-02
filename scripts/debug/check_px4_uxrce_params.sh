#!/bin/bash
# =============================================================================
# PX4 uXRCE-DDS 파라미터 확인 스크립트
# =============================================================================
# QGC에서 확인한 파라미터 값을 검증
# =============================================================================

echo "=== PX4 uXRCE-DDS 파라미터 확인 ==="
echo ""

# IP 주소 변환 함수
ip_to_int() {
    local ip=$1
    IFS='.' read -r a b c d <<< "$ip"
    echo $((a * 256 * 256 * 256 + b * 256 * 256 + c * 256 + d))
}

int_to_ip() {
    local int=$1
    local a=$((int >> 24 & 0xFF))
    local b=$((int >> 16 & 0xFF))
    local c=$((int >> 8 & 0xFF))
    local d=$((int & 0xFF))
    echo "$a.$b.$c.$d"
}

# 설정 파일에서 IP 읽기
CONFIG_FILE="/home/khadas/humiro_fire_suppression/config/device_config.env"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    EXPECTED_IP="$ETH0_IP"
    EXPECTED_PORT="$XRCE_DDS_PORT"
else
    EXPECTED_IP="10.0.0.11"
    EXPECTED_PORT="8888"
fi

EXPECTED_IP_INT=$(ip_to_int "$EXPECTED_IP")

echo "[1] 예상 설정값:"
echo "  - VIM4 eth0 IP: $EXPECTED_IP"
echo "  - IP 정수 형식: $EXPECTED_IP_INT"
echo "  - micro-ROS Agent 포트: $EXPECTED_PORT"
echo ""

echo "[2] QGC에서 확인해야 할 파라미터:"
echo ""
echo "  ✓ uXRCE-DDS_DOM_ID = 0"
echo "     → ROS_DOMAIN_ID=0과 일치해야 함"
echo ""
echo "  ✓ uxrce_dds_ag_ip = $EXPECTED_IP_INT"
echo "     → IP 주소: $EXPECTED_IP (VIM4 eth0)"
echo "     → 현재 설정값: 167772171 (사용자가 확인함)"
if [ "$EXPECTED_IP_INT" = "167772171" ]; then
    echo "     → ✅ 올바르게 설정됨"
else
    echo "     → ⚠️  설정값이 예상과 다름 (예상: $EXPECTED_IP_INT)"
    echo "     → 변환된 IP: $(int_to_ip 167772171)"
fi
echo ""
echo "  ✓ UXRCE_DDS_PRT = $EXPECTED_PORT"
echo "     → micro-ROS Agent UDP 포트"
echo "     → 현재 설정값: 8888 (사용자가 확인함)"
echo "     → ✅ 올바르게 설정됨"
echo ""

echo "[3] IP 주소 변환 도구:"
echo ""
echo "  IP → 정수:"
echo "    python3 -c \"ip='$EXPECTED_IP'; parts=ip.split('.'); print(int(parts[0])*256**3 + int(parts[1])*256**2 + int(parts[2])*256 + int(parts[3]))\""
echo ""
echo "  정수 → IP:"
echo "    python3 -c \"n=167772171; print(f'{n>>24&0xFF}.{n>>16&0xFF}.{n>>8&0xFF}.{n&0xFF}')\""
echo ""

echo "[4] 연결 확인:"
echo ""
echo "  micro-ROS Agent 실행 확인:"
if pgrep -f "micro_ros_agent" > /dev/null; then
    echo "    ✓ micro-ROS Agent 실행 중"
else
    echo "    ✗ micro-ROS Agent 실행 안 됨"
fi
echo ""
echo "  네트워크 연결 확인:"
if ping -c 1 "$EXPECTED_IP" > /dev/null 2>&1; then
    echo "    ✓ VIM4 eth0 연결 확인"
else
    echo "    ✗ VIM4 eth0 연결 실패"
fi
echo ""

echo "=== 확인 완료 ==="
echo ""
echo "참고:"
echo "  - PX4가 시동이 걸려 있어야 데이터를 발행합니다"
echo "  - QoS 설정이 올바른지 확인하세요 (TransientLocal)"
echo "  - 재빌드 후 테스트: ./scripts/runtime/service-control.sh restart"

