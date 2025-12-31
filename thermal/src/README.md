# Thermal RGB Streaming - C/C++ 버전

Python으로 작성된 `Thermal+RGB_new.py`를 C/C++로 변환한 프로젝트입니다.

## 프로젝트 구조

```
src/
├── config.h                 # 설정 상수 정의
├── thread_safe_queue.h      # 스레드 안전 큐 템플릿
├── utils.h/cpp              # 유틸리티 함수 (IP 주소, 카메라 정보 등)
├── thermal_data.h            # 열화상 데이터 구조체
├── camera_manager.h/cpp      # 카메라 초기화 및 관리
├── thermal_processor.h/cpp   # 열화상 데이터 처리
├── frame_compositor.h/cpp   # 프레임 합성 (RGB + 열화상)
├── rtsp_server.h/cpp        # RTSP 서버 (GStreamer)
├── http_server.h/cpp         # HTTP 서버 (libmicrohttpd)
├── main.cpp                  # 메인 프로그램
├── CMakeLists.txt           # CMake 빌드 설정
└── Makefile                 # Makefile 빌드 설정
```

## 빌드 방법

### 1. 의존성 설치

```bash
# 시스템 패키지
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-ugly1.0-dev \
    libgstrtspserver-1.0-dev \
    libmicrohttpd-dev
```

### 2. CMake로 빌드

```bash
cd ~/humiro_fire_suppression/thermal/src
mkdir build
cd build
cmake ..
make
```

### 3. Makefile로 빌드

```bash
cd ~/humiro_fire_suppression/thermal/src
make
```

## 실행

```bash
./thermal_rgb_streaming
```

## 주요 기능

- **모듈화된 구조**: 기능별로 파일 분리
- **스레드 안전**: 멀티스레드 환경에서 안전하게 동작
- **저지연 최적화**: 별도 스레드로 전송과 처리 분리
- **RTSP 스트리밍**: QGroundControl, VLC 등에서 접속 가능
- **HTTP 웹 스트리밍**: 웹 브라우저에서 실시간 모니터링

## 설정 변경

`config.h` 파일에서 설정을 변경할 수 있습니다:

- `ENABLE_HTTP_SERVER`: HTTP 서버 활성화/비활성화
- `OUTPUT_WIDTH`, `OUTPUT_HEIGHT`: 출력 해상도
- `OUTPUT_FPS`: 출력 FPS
- `OVERLAY_THERMAL`: 열화상 오버레이 활성화/비활성화
- `ALPHA_THERMAL_LAYER`: 열화상 오버레이 투명도

## 참고 사항

- Python 버전과 동일한 기능을 제공합니다
- C/C++ 버전은 더 나은 성능을 제공할 수 있습니다
- GStreamer와 OpenCV의 C/C++ API를 사용합니다
