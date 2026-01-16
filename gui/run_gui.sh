#!/bin/bash
# Humiro Fire Suppression Web GUI 실행 스크립트
# Flask 기반 웹 서버

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python 환경 확인
if ! command -v python3 &> /dev/null; then
    echo "Python3가 설치되지 않았습니다."
    exit 1
fi

# Flask 확인 및 설치
if ! python3 -c "import flask" 2>/dev/null; then
    echo "Flask가 설치되지 않았습니다."
    echo "설치 중..."
    pip3 install flask
fi

# 포트 설정 (기본: 5000)
PORT=${1:-5000}

echo "=================================================="
echo "  Humiro Fire Suppression - Web GUI"
echo "=================================================="
echo ""
echo "웹 브라우저에서 접속하세요:"
echo ""
echo "  로컬:    http://localhost:${PORT}"
echo "  네트워크: http://$(hostname -I | awk '{print $1}'):${PORT}"
echo ""
echo "종료: Ctrl+C"
echo "=================================================="
echo ""

# Flask 웹 서버 실행
python3 app.py
