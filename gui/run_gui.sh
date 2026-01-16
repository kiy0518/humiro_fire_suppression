#!/bin/bash
# Humiro Fire Suppression GUI Management Tool 실행 스크립트

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python 환경 확인
if ! command -v python3 &> /dev/null; then
    echo "Python3가 설치되지 않았습니다."
    exit 1
fi

# PyQt6 확인
if ! python3 -c "import PyQt6" 2>/dev/null; then
    echo "PyQt6가 설치되지 않았습니다."
    echo "설치 중..."
    pip3 install PyQt6
fi

# 환경 변수 설정 (Qt 플랫폼 이슈 방지)
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=1

# GUI 실행
echo "Humiro Fire Suppression GUI 시작..."
python3 main.py "$@"
