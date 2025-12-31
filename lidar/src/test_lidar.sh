#!/bin/bash

# LiDAR 프로그램 테스트 스크립트
# 사용법: ./test_lidar.sh [옵션]

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 프로젝트 경로
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
LIDAR_TEST="$BUILD_DIR/lidar_test"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   LiDAR 프로그램 테스트 스크립트${NC}"
echo -e "${BLUE}========================================${NC}\n"

# 1. 빌드 확인
echo -e "${YELLOW}[1/5] 빌드 확인...${NC}"
if [ ! -f "$LIDAR_TEST" ]; then
    echo -e "${RED}❌ 실행 파일을 찾을 수 없습니다: $LIDAR_TEST${NC}"
    echo -e "${YELLOW}빌드를 진행합니다...${NC}"
    cd "$SCRIPT_DIR"
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    echo -e "${GREEN}✅ 빌드 완료${NC}\n"
else
    echo -e "${GREEN}✅ 실행 파일 확인됨${NC}\n"
fi

# 2. 시리얼 포트 확인
echo -e "${YELLOW}[2/5] 시리얼 포트 확인...${NC}"

# USB-UART 확인
USB_PORTS=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -n "$USB_PORTS" ]; then
    echo -e "${GREEN}✅ USB-UART 포트 발견: $USB_PORTS${NC}"
    USB_PORT="$USB_PORTS"
else
    echo -e "${YELLOW}⚠️  USB-UART 포트를 찾을 수 없습니다${NC}"
    USB_PORT="/dev/ttyUSB0"
fi

# GPIO-UART (UART_E) 확인
if [ -c "/dev/ttyS4" ]; then
    echo -e "${GREEN}✅ GPIO-UART (UART_E) 포트 확인: /dev/ttyS4${NC}"
    GPIO_PORT="/dev/ttyS4"
    
    # 권한 확인
    if [ -r "/dev/ttyS4" ] && [ -w "/dev/ttyS4" ]; then
        echo -e "${GREEN}✅ UART_E 권한 확인됨${NC}"
    else
        echo -e "${YELLOW}⚠️  UART_E 권한이 없습니다. 설정이 필요합니다:${NC}"
        echo -e "   sudo chmod 666 /dev/ttyS4"
        echo -e "   또는"
        echo -e "   sudo usermod -a -G dialout $USER"
    fi
else
    echo -e "${YELLOW}⚠️  GPIO-UART (UART_E) 포트를 찾을 수 없습니다: /dev/ttyS4${NC}"
    GPIO_PORT="/dev/ttyS4"
fi

echo ""

# 3. 카메라 확인
echo -e "${YELLOW}[3/5] 카메라 확인...${NC}"
CAMERA_DEVICES=$(ls /dev/video* 2>/dev/null | head -1)
if [ -n "$CAMERA_DEVICES" ]; then
    echo -e "${GREEN}✅ 카메라 장치 발견: $CAMERA_DEVICES${NC}"
    CAMERA_ID=0
else
    echo -e "${YELLOW}⚠️  카메라 장치를 찾을 수 없습니다${NC}"
    echo -e "${YELLOW}   카메라 없이 테스트를 진행합니다 (LiDAR만)${NC}"
    CAMERA_ID=0
fi
echo ""

# 4. 테스트 모드 선택
echo -e "${YELLOW}[4/5] 테스트 모드 선택${NC}"
echo -e "1) USB-UART 테스트 ($USB_PORT)"
echo -e "2) GPIO-UART (UART_E) 테스트 (/dev/ttyS4)"
echo -e "3) GPIO-UART (UART_E) 테스트 - GUI 없음 (headless)"
echo -e "4) 도움말 보기"
echo -e "5) 종료"
echo ""
read -p "선택하세요 (1-5): " choice

case $choice in
    1)
        TEST_MODE="usb"
        TEST_PORT="$USB_PORT"
        TEST_CMD="$LIDAR_TEST -u $TEST_PORT $CAMERA_ID"
        ;;
    2)
        TEST_MODE="gpio"
        TEST_PORT="/dev/ttyS4"
        TEST_CMD="$LIDAR_TEST -g $TEST_PORT $CAMERA_ID"
        ;;
    3)
        TEST_MODE="gpio-headless"
        TEST_PORT="/dev/ttyS4"
        TEST_CMD="$LIDAR_TEST -g $TEST_PORT $CAMERA_ID -n"
        echo -e "${YELLOW}⚠️  GUI 없이 실행합니다 (headless 모드)${NC}"
        ;;
    4)
        echo ""
        $LIDAR_TEST -h
        exit 0
        ;;
    5)
        echo -e "${BLUE}테스트를 종료합니다.${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}잘못된 선택입니다.${NC}"
        exit 1
        ;;
esac

# 5. 테스트 실행
echo ""
echo -e "${YELLOW}[5/5] 테스트 실행${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "테스트 모드: $TEST_MODE"
echo -e "포트: $TEST_PORT"
echo -e "카메라 ID: $CAMERA_ID"
echo -e "${BLUE}========================================${NC}"
echo ""

# UART_E 포트 확인
if [ "$TEST_MODE" = "gpio" ] || [ "$TEST_MODE" = "gpio-headless" ]; then
    if [ ! -c "/dev/ttyS4" ]; then
        echo -e "${YELLOW}⚠️  경고: /dev/ttyS4 (UART_E)가 활성화되지 않았습니다${NC}"
        echo -e "${YELLOW}   프로그램은 실행되지만 LiDAR 연결은 실패합니다${NC}"
        echo -e "${YELLOW}   UART_E 활성화: ./activate_uart_e.sh${NC}"
        echo ""
    fi
fi

echo -e "${GREEN}프로그램을 시작합니다...${NC}"
echo -e "${YELLOW}종료하려면 Ctrl+C를 누르세요${NC}"
echo ""

# 실행
cd "$BUILD_DIR"
$TEST_CMD

