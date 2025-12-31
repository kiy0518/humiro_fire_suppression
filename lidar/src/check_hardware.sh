#!/bin/bash

# LiDAR 하드웨어 연결 확인 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   LiDAR 하드웨어 연결 확인${NC}"
echo -e "${BLUE}========================================${NC}\n"

# 1. USB-UART 확인
echo -e "${YELLOW}[1/4] USB-UART 어댑터 확인${NC}"
USB_PORTS=$(ls /dev/ttyUSB* 2>/dev/null)
if [ -n "$USB_PORTS" ]; then
    echo -e "${GREEN}✅ USB-UART 포트 발견:${NC}"
    for port in $USB_PORTS; do
        echo -e "   $port"
        if [ -r "$port" ] && [ -w "$port" ]; then
            echo -e "   ${GREEN}   ✓ 읽기/쓰기 권한 있음${NC}"
        else
            echo -e "   ${YELLOW}   ⚠ 권한 없음 (sudo chmod 666 $port 필요)${NC}"
        fi
    done
else
    echo -e "${YELLOW}⚠️  USB-UART 포트를 찾을 수 없습니다${NC}"
fi
echo ""

# 2. GPIO-UART (UART_E) 확인
echo -e "${YELLOW}[2/4] GPIO-UART (UART_E) 확인${NC}"
if [ -c "/dev/ttyS4" ]; then
    echo -e "${GREEN}✅ UART_E 포트 확인: /dev/ttyS4${NC}"
    
    # 권한 확인
    if [ -r "/dev/ttyS4" ] && [ -w "/dev/ttyS4" ]; then
        echo -e "${GREEN}   ✓ 읽기/쓰기 권한 있음${NC}"
    else
        echo -e "${YELLOW}   ⚠ 권한 없음${NC}"
        echo -e "${YELLOW}   다음 명령으로 권한 설정:${NC}"
        echo -e "   ${BLUE}sudo chmod 666 /dev/ttyS4${NC}"
        echo -e "   ${BLUE}또는${NC}"
        echo -e "   ${BLUE}sudo usermod -a -G dialout $USER${NC}"
        echo -e "   ${YELLOW}(로그아웃 후 다시 로그인 필요)${NC}"
    fi
    
    # 포트 정보
    echo -e "${BLUE}   포트 정보:${NC}"
    ls -l /dev/ttyS4
else
    echo -e "${RED}❌ UART_E 포트를 찾을 수 없습니다: /dev/ttyS4${NC}"
    echo -e "${YELLOW}   UART_E가 활성화되어 있는지 확인하세요${NC}"
fi
echo ""

# 3. 카메라 확인
echo -e "${YELLOW}[3/4] 카메라 확인${NC}"
CAMERA_DEVICES=$(ls /dev/video* 2>/dev/null)
if [ -n "$CAMERA_DEVICES" ]; then
    echo -e "${GREEN}✅ 카메라 장치 발견:${NC}"
    for cam in $CAMERA_DEVICES; do
        echo -e "   $cam"
        if [ -r "$cam" ] && [ -w "$cam" ]; then
            echo -e "   ${GREEN}   ✓ 읽기/쓰기 권한 있음${NC}"
        else
            echo -e "   ${YELLOW}   ⚠ 권한 없음${NC}"
        fi
    done
else
    echo -e "${YELLOW}⚠️  카메라 장치를 찾을 수 없습니다${NC}"
    echo -e "${YELLOW}   카메라 없이도 LiDAR 테스트는 가능합니다${NC}"
fi
echo ""

# 4. USB 장치 정보 (LiDAR 확인용)
echo -e "${YELLOW}[4/4] USB 장치 정보${NC}"
USB_INFO=$(lsusb 2>/dev/null)
if [ -n "$USB_INFO" ]; then
    echo -e "${BLUE}연결된 USB 장치:${NC}"
    echo "$USB_INFO" | while IFS= read -r line; do
        if echo "$line" | grep -qiE "serial|uart|ch340|cp210|ftdi"; then
            echo -e "   ${GREEN}$line${NC} (UART 어댑터 가능성)"
        else
            echo "   $line"
        fi
    done
else
    echo -e "${YELLOW}⚠️  USB 장치 정보를 가져올 수 없습니다${NC}"
fi
echo ""

# 요약
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   확인 완료${NC}"
echo -e "${BLUE}========================================${NC}\n"

# 권장 사항
echo -e "${YELLOW}권장 사항:${NC}"
if [ ! -r "/dev/ttyS4" ] || [ ! -w "/dev/ttyS4" ]; then
    echo -e "1. ${YELLOW}UART_E 권한 설정:${NC}"
    echo -e "   ${BLUE}sudo chmod 666 /dev/ttyS4${NC}"
fi

if [ -z "$USB_PORTS" ] && [ ! -c "/dev/ttyS4" ]; then
    echo -e "2. ${YELLOW}LiDAR 하드웨어 연결 확인:${NC}"
    echo -e "   - USB-UART 어댑터 연결"
    echo -e "   - 또는 VIM4 GPIO UART_E 연결"
fi

echo ""
echo -e "${GREEN}테스트 실행:${NC}"
echo -e "   ${BLUE}cd ~/humiro_fire_suppression/lidar/src${NC}"
echo -e "   ${BLUE}./test_lidar.sh${NC}"

