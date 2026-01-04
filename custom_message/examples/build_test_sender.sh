#!/bin/bash
# 테스트 메시지 송신 프로그램 빌드 스크립트

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CUSTOM_MESSAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "============================================"
echo "  테스트 메시지 송신 프로그램 빌드"
echo "============================================"

# 커스텀 메시지 라이브러리 빌드 확인
if [ ! -f "$CUSTOM_MESSAGE_DIR/build/libcustom_message.a" ]; then
    echo "[1/2] 커스텀 메시지 라이브러리 빌드 중..."
    cd "$CUSTOM_MESSAGE_DIR"
    mkdir -p build
    cd build
    cmake ..
    make
    echo "  ✓ 라이브러리 빌드 완료"
else
    echo "[1/2] 커스텀 메시지 라이브러리 존재 확인"
    echo "  ✓ 라이브러리 존재"
fi

# 테스트 프로그램 빌드
echo ""
echo "[2/2] 테스트 프로그램 컴파일 중..."
cd "$SCRIPT_DIR"

g++ -std=c++17 \
    -I"$CUSTOM_MESSAGE_DIR/include" \
    test_message_sender.cpp \
    -L"$CUSTOM_MESSAGE_DIR/build" \
    -lcustom_message \
    -pthread \
    -o test_message_sender

if [ $? -eq 0 ]; then
    echo "  ✓ 빌드 완료"
    echo ""
    echo "============================================"
    echo "  빌드 성공!"
    echo "============================================"
    echo ""
    echo "실행 방법:"
    echo "  cd $SCRIPT_DIR"
    echo "  ./test_message_sender"
    echo ""
else
    echo "  ✗ 빌드 실패"
    exit 1
fi
