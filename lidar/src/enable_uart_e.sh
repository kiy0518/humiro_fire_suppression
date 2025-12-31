#!/bin/bash

# VIM4 UART_E 활성화 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

OVERLAY_ENV_FILE="/boot/dtb/amlogic/kvim4.dtb.overlay.env"
OVERLAY_ENV_REAL="/boot/overlays/kvim4.dtb.overlay.env"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   VIM4 UART_E 활성화${NC}"
echo -e "${BLUE}========================================${NC}\n"

# 1. 현재 UART 상태 확인
echo -e "${YELLOW}[1/4] 현재 UART 상태 확인${NC}"
if [ -c "/dev/ttyS4" ]; then
    echo -e "${GREEN}✅ UART_E (/dev/ttyS4) 이미 활성화되어 있습니다!${NC}"
    ls -l /dev/ttyS4
    echo ""
    echo -e "${BLUE}설정 파일 확인:${NC}"
    cat "$OVERLAY_ENV_FILE" 2>/dev/null || cat "$OVERLAY_ENV_REAL" 2>/dev/null
    exit 0
else
    echo -e "${YELLOW}⚠️  UART_E (/dev/ttyS4)가 활성화되지 않았습니다${NC}"
fi

echo -e "활성화된 UART 포트:"
ls -l /dev/ttyS* 2>/dev/null || echo "없음"
echo ""

# 2. 설정 파일 확인
echo -e "${YELLOW}[2/4] 설정 파일 확인${NC}"

# 실제 파일 경로 확인
if [ -f "$OVERLAY_ENV_REAL" ]; then
    CONFIG_FILE="$OVERLAY_ENV_REAL"
elif [ -f "$OVERLAY_ENV_FILE" ]; then
    CONFIG_FILE="$OVERLAY_ENV_FILE"
else
    echo -e "${RED}❌ 설정 파일을 찾을 수 없습니다:${NC}"
    echo -e "   $OVERLAY_ENV_FILE"
    echo -e "   $OVERLAY_ENV_REAL"
    exit 1
fi

echo -e "${GREEN}✅ 설정 파일 발견: $CONFIG_FILE${NC}"
echo -e "${BLUE}현재 내용:${NC}"
cat "$CONFIG_FILE"
echo ""

# 3. UART_E 활성화
echo -e "${YELLOW}[3/4] UART_E 활성화${NC}"

# 현재 fdt_overlays 값 확인
CURRENT_OVERLAYS=$(grep "^fdt_overlays=" "$CONFIG_FILE" 2>/dev/null | cut -d'=' -f2- | tr -d '\n')

if [ -z "$CURRENT_OVERLAYS" ]; then
    # fdt_overlays가 비어있거나 없음
    echo -e "${BLUE}fdt_overlays가 비어있습니다. uart_e 추가...${NC}"
    NEW_OVERLAYS="uart_e"
elif echo "$CURRENT_OVERLAYS" | grep -q "uart_e"; then
    # 이미 uart_e가 있음
    echo -e "${GREEN}✅ uart_e가 이미 설정되어 있습니다${NC}"
    echo -e "${YELLOW}재부팅이 필요합니다${NC}"
    NEW_OVERLAYS="$CURRENT_OVERLAYS"
else
    # 다른 overlay가 있음, uart_e 추가
    echo -e "${BLUE}기존 overlay에 uart_e 추가...${NC}"
    NEW_OVERLAYS="${CURRENT_OVERLAYS} uart_e"
fi

# 백업 생성
echo -e "${BLUE}백업 생성: ${CONFIG_FILE}.backup${NC}"
sudo cp "$CONFIG_FILE" "${CONFIG_FILE}.backup"

# 파일 수정
echo -e "${BLUE}설정 파일 업데이트...${NC}"
if grep -q "^fdt_overlays=" "$CONFIG_FILE"; then
    # 기존 라인 수정
    sudo sed -i "s/^fdt_overlays=.*/fdt_overlays=${NEW_OVERLAYS}/" "$CONFIG_FILE"
else
    # 새 라인 추가
    echo "fdt_overlays=${NEW_OVERLAYS}" | sudo tee -a "$CONFIG_FILE" > /dev/null
fi

echo -e "${GREEN}✅ 설정 파일 업데이트 완료${NC}"
echo -e "${BLUE}업데이트된 내용:${NC}"
cat "$CONFIG_FILE"
echo ""

# 4. 최종 확인 및 안내
echo -e "${YELLOW}[4/4] 최종 확인${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✅ UART_E 활성화 설정이 완료되었습니다!${NC}"
echo ""
echo -e "${YELLOW}⚠️  중요: 재부팅이 필요합니다${NC}"
echo ""
echo -e "${BLUE}다음 단계:${NC}"
echo -e "1. ${YELLOW}시스템 재부팅:${NC}"
echo -e "   ${BLUE}sudo reboot${NC}"
echo ""
echo -e "2. ${YELLOW}재부팅 후 확인:${NC}"
echo -e "   ${BLUE}ls -l /dev/ttyS4${NC}"
echo ""
echo -e "3. ${YELLOW}권한 설정:${NC}"
echo -e "   ${BLUE}sudo chmod 666 /dev/ttyS4${NC}"
echo ""
echo -e "4. ${YELLOW}테스트:${NC}"
echo -e "   ${BLUE}cd ~/humiro_fire_suppression/lidar/src/build${NC}"
echo -e "   ${BLUE}./lidar_test -g /dev/ttyS4 0 -n${NC}"
echo ""
echo -e "${BLUE}백업 파일: ${CONFIG_FILE}.backup${NC}"
echo -e "${BLUE}(설정을 되돌리려면 백업 파일을 복원하세요)${NC}"
