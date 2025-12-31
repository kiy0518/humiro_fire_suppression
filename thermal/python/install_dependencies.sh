#!/bin/bash
# Thermal+RGB_new.py 실행을 위한 패키지 설치 스크립트
# Ubuntu 22.04 Server 기준

set -e  # 에러 발생 시 중단

echo "=========================================="
echo "Thermal+RGB_new.py 의존성 패키지 설치"
echo "=========================================="
echo ""

# 시스템 업데이트
echo "[1/4] 시스템 패키지 목록 업데이트..."
sudo apt update

# GStreamer 및 RTSP 서버 패키지
echo ""
echo "[2/4] GStreamer 및 RTSP 서버 패키지 설치..."
sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgstrtspserver-1.0-0 \
    gir1.2-gst-rtsp-server-1.0 \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0

# Python 패키지 (시스템 패키지)
echo ""
echo "[3/4] Python 시스템 패키지 설치..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-gi \
    python3-gi-cairo \
    python3-opencv \
    python3-numpy \
    python3-flask

# pip 업그레이드
echo ""
echo "[4/4] pip 업그레이드 및 추가 패키지 확인..."
python3 -m pip install --upgrade pip

# 추가 Python 패키지 (pip로 설치)
echo ""
echo "추가 Python 패키지 설치 (필요시)..."
python3 -m pip install --user \
    opencv-python \
    numpy \
    flask

echo ""
echo "=========================================="
echo "설치 완료!"
echo "=========================================="
echo ""
echo "설치된 주요 패키지:"
echo "  - GStreamer 1.0 (비디오 스트리밍)"
echo "  - GStreamer RTSP 서버"
echo "  - Python 3 (OpenCV, NumPy, Flask)"
echo ""
echo "다음 명령어로 스크립트 실행:"
echo "  python3 /home/khadas/projects/Thermal/Thermal+RGB_new.py"
echo ""
