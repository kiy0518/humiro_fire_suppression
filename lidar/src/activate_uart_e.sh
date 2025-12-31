#!/bin/bash

# VIM4 UART_E 활성화 시도 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   VIM4 UART_E 활성화 시도${NC}"
echo -e "${BLUE}========================================${NC}\n"

# 1. 현재 상태 확인
echo -e "${YELLOW}[1/4] 현재 UART 상태 확인${NC}"
if [ -c "/dev/ttyS4" ]; then
    echo -e "${GREEN}✅ UART_E (/dev/ttyS4) 이미 활성화되어 있습니다!${NC}"
    ls -l /dev/ttyS4
    exit 0
fi

echo -e "활성화된 UART 포트:"
ls -l /dev/ttyS* 2>/dev/null || echo "없음"
echo ""

# 2. UART 드라이버 확인
echo -e "${YELLOW}[2/4] UART 드라이버 확인${NC}"
if lsmod | grep -q meson_uart; then
    echo -e "${GREEN}✅ meson_uart 드라이버가 로드되어 있습니다${NC}"
    lsmod | grep meson_uart
else
    echo -e "${YELLOW}⚠️  meson_uart 드라이버가 로드되지 않았습니다${NC}"
    echo -e "${BLUE}드라이버 로드 시도...${NC}"
    sudo modprobe meson_uart 2>&1 || echo -e "${YELLOW}드라이버 로드 실패 (이미 로드되었거나 다른 방법 필요)${NC}"
fi
echo ""

# 3. Device Tree 확인
echo -e "${YELLOW}[3/4] Device Tree 확인${NC}"
echo -e "시리얼 포트 디바이스 트리:"
find /proc/device-tree -name "*serial*" -o -name "*uart*" 2>/dev/null | head -5
echo ""

# 4. UART_E 활성화 시도
echo -e "${YELLOW}[4/4] UART_E 활성화 시도${NC}"

# 방법 1: 직접 디바이스 생성 시도 (일반적으로 작동하지 않음)
echo -e "${BLUE}방법 1: 디바이스 확인${NC}"
if [ -d "/sys/class/tty/ttyS4" ]; then
    echo -e "${GREEN}✅ /sys/class/tty/ttyS4 디렉토리 존재${NC}"
    ls -l /sys/class/tty/ttyS4 2>/dev/null || echo "디렉토리는 있지만 디바이스 파일 없음"
else
    echo -e "${YELLOW}⚠️  /sys/class/tty/ttyS4 디렉토리가 없습니다${NC}"
fi
echo ""

# 방법 2: VIM4 Overlay 설정 파일 확인
echo -e "${BLUE}방법 2: VIM4 Overlay 설정 파일 확인${NC}"
OVERLAY_ENV_FILE="/boot/dtb/amlogic/kvim4.dtb.overlay.env"
OVERLAY_ENV_REAL="/boot/overlays/kvim4.dtb.overlay.env"

if [ -f "$OVERLAY_ENV_REAL" ]; then
    CONFIG_FILE="$OVERLAY_ENV_REAL"
elif [ -f "$OVERLAY_ENV_FILE" ]; then
    CONFIG_FILE="$OVERLAY_ENV_FILE"
else
    echo -e "${YELLOW}⚠️  Overlay 설정 파일을 찾을 수 없습니다${NC}"
    CONFIG_FILE=""
fi

if [ -n "$CONFIG_FILE" ]; then
    echo -e "${GREEN}✅ Overlay 설정 파일 발견: $CONFIG_FILE${NC}"
    echo -e "${BLUE}현재 내용:${NC}"
    cat "$CONFIG_FILE"
    echo ""
    
    if grep -q "uart_e" "$CONFIG_FILE" 2>/dev/null; then
        echo -e "${GREEN}✅ uart_e가 이미 설정되어 있습니다${NC}"
        echo -e "${YELLOW}재부팅이 필요합니다${NC}"
    else
        echo -e "${YELLOW}⚠️  uart_e가 설정되지 않았습니다${NC}"
        echo -e "${BLUE}활성화하려면:${NC}"
        echo -e "   ${BLUE}./enable_uart_e.sh${NC}"
    fi
fi
echo ""

# 최종 확인
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   최종 확인${NC}"
echo -e "${BLUE}========================================${NC}\n"

if [ -c "/dev/ttyS4" ]; then
    echo -e "${GREEN}✅ 성공! UART_E (/dev/ttyS4)가 활성화되었습니다${NC}"
    ls -l /dev/ttyS4
    echo ""
    echo -e "${YELLOW}권한 설정:${NC}"
    echo -e "   ${BLUE}sudo chmod 666 /dev/ttyS4${NC}"
else
    echo -e "${RED}❌ UART_E (/dev/ttyS4)가 아직 활성화되지 않았습니다${NC}"
    echo ""
    echo -e "${YELLOW}다음 단계:${NC}"
    echo -e "1. ${BLUE}UART_E 활성화 스크립트 실행:${NC}"
    echo -e "   ${BLUE}./enable_uart_e.sh${NC}"
    echo ""
    echo -e "2. ${BLUE}재부팅 후 확인:${NC}"
    echo -e "   ${BLUE}sudo reboot${NC}"
    echo ""
    echo -e "3. ${BLUE}USB-UART 어댑터 사용 (임시 해결책):${NC}"
    echo -e "   ${BLUE}./lidar_test -u /dev/ttyUSB0 0 -n${NC}"
    echo ""
    echo -e "${BLUE}수동 설정 방법:${NC}"
    echo -e "   ${BLUE}sudo nano /boot/dtb/amlogic/kvim4.dtb.overlay.env${NC}"
    echo -e "   또는"
    echo -e "   ${BLUE}sudo nano /boot/overlays/kvim4.dtb.overlay.env${NC}"
    echo ""
    echo -e "${BLUE}추가할 내용:${NC}"
    echo -e "   ${YELLOW}fdt_overlays=uart_e${NC}"
fi

