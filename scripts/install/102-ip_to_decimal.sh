#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# IP 주소를 10진수(int32)로 변환하는 스크립트
# PX4 파라미터 설정용
# humiro_fire_suppression 프로젝트용

# 사용법 출력
usage() {
    echo "사용법: $0 <IP주소>"
    echo "예: $0 10.0.0.1"
    exit 1
}

# 인자 확인
if [ -z "$1" ]; then
    echo "오류: IP 주소가 제공되지 않았습니다."
    usage
fi

IP=$1

# IP 주소 유효성 검사
if [[ ! $IP =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
    echo "오류: 유효하지 않은 IP 주소 형식입니다: $IP"
    exit 1
fi

# 각 옥텟이 0-255 범위인지 확인
IFS='.' read -r -a octets <<< "$IP"
for octet in "${octets[@]}"; do
    if [ "$octet" -lt 0 ] || [ "$octet" -gt 255 ]; then
        echo "오류: IP 주소의 각 옥텟은 0-255 범위여야 합니다: $IP"
        exit 1
    fi
done

# Python을 사용하여 IP를 10진수로 변환
DECIMAL=$(python3 -c "import struct, socket; print(struct.unpack('!I', socket.inet_aton('$IP'))[0])" 2>/dev/null)

if [ -z "$DECIMAL" ]; then
    echo "오류: IP 변환 실패"
    exit 1
fi

# signed int32로 변환 (PX4 파라미터용)
# 32-bit signed integer 범위: -2,147,483,648 ~ 2,147,483,647
# unsigned 값이 2,147,483,647보다 크면 signed로 변환
SIGNED=$(python3 -c "val = $DECIMAL; print(val - 2**32 if val > 2147483647 else val)" 2>/dev/null)

echo "=========================================="
echo "IP 주소 변환 결과"
echo "=========================================="
echo "IP 주소:     $IP"
echo "Unsigned:    $DECIMAL"
echo "Signed:      $SIGNED (PX4 파라미터용)"
echo ""
echo "PX4 파라미터 설정:"
echo "  param set UXRCE_DDS_AG_IP $SIGNED"
echo ""
echo "QGroundControl에서:"
echo "  UXRCE_DDS_AG_IP = $SIGNED"
echo "=========================================="
