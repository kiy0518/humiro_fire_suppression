# RGB + 열화상 데이터 RTSP/HTTP 스트리밍 서버

## 개요

이 프로젝트는 Khadas VIM4에서 RGB 카메라와 열화상 카메라(Lepton 3.5)의 영상을 합성하여 RTSP/HTTP로 스트리밍하는 서버입니다.

### 주요 기능

- **RGB + 열화상 영상 합성**: RGB 영상에 열화상 데이터 오버레이
- **RTSP 스트리밍**: QGroundControl, VLC 등에서 접속 가능
- **HTTP 웹 스트리밍**: 웹 브라우저에서 실시간 모니터링
- **저지연 최적화**: 별도 스레드로 전송 최적화

## 시스템 요구사항

- **OS**: Ubuntu 22.04 Server (또는 호환 Debian 계열)
- **하드웨어**: Khadas VIM4
- **카메라**: 
  - PureThermal (Lepton 3.5) - 열화상 카메라
  - USB Camera - RGB 카메라

## 설치 방법

### 1. 자동 설치 (권장)

```bash
# 설치 스크립트에 실행 권한 부여
chmod +x ~/humiro_fire_suppression/thermal/install_dependencies.sh

# 의존성 패키지 설치
bash ~/humiro_fire_suppression/thermal/install_dependencies.sh
```

### 2. 수동 설치

#### 시스템 패키지 설치

```bash
# 패키지 목록 업데이트
sudo apt update

# GStreamer 및 RTSP 서버
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

# Python 패키지
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
python3 -m pip install --upgrade pip
```

#### 추가 Python 패키지 (필요시)

```bash
python3 -m pip install --user opencv-python numpy flask
```

## 필요한 패키지 목록

### 시스템 패키지 (apt)

| 패키지 | 용도 |
|--------|------|
| `gstreamer1.0-tools` | GStreamer 명령줄 도구 |
| `gstreamer1.0-plugins-base` | GStreamer 기본 플러그인 |
| `gstreamer1.0-plugins-good` | GStreamer 좋은 품질 플러그인 |
| `gstreamer1.0-plugins-bad` | GStreamer 추가 플러그인 |
| `gstreamer1.0-plugins-ugly` | GStreamer 특허 관련 플러그인 |
| `libgstreamer1.0-dev` | GStreamer 개발 헤더 |
| `libgstrtspserver-1.0-0` | GStreamer RTSP 서버 라이브러리 |
| `gir1.2-gst-rtsp-server-1.0` | GStreamer RTSP 서버 Python 바인딩 |
| `gir1.2-gstreamer-1.0` | GStreamer Python 바인딩 |
| `gir1.2-gst-plugins-base-1.0` | GStreamer 플러그인 Python 바인딩 |
| `python3` | Python 3 인터프리터 |
| `python3-pip` | Python 패키지 관리자 |
| `python3-dev` | Python 개발 헤더 |
| `python3-gi` | GObject Introspection (GStreamer 바인딩) |
| `python3-gi-cairo` | GObject Introspection Cairo 지원 |
| `python3-opencv` | OpenCV Python 바인딩 |
| `python3-numpy` | NumPy 수치 연산 라이브러리 |
| `python3-flask` | Flask 웹 프레임워크 |

### Python 패키지 (pip)

| 패키지 | 용도 |
|--------|------|
| `opencv-python` | OpenCV (이미지 처리) |
| `numpy` | NumPy (배열 연산) |
| `flask` | Flask (HTTP 웹 서버) |

### Python 표준 라이브러리 (설치 불필요)

- `sys`, `signal`, `socket`, `subprocess`, `time`, `os`
- `threading`, `queue`, `warnings`, `datetime`

## 실행 방법

### 기본 실행

```bash
python3 ~/humiro_fire_suppression/thermal/Thermal+RGB_new.py
```

### 백그라운드 실행

```bash
# nohup으로 실행
nohup python3 ~/humiro_fire_suppression/thermal/Thermal+RGB_new.py > /tmp/thermal_rgb.log 2>&1 &

# 또는 screen/tmux 사용
screen -S thermal_rgb
python3 ~/humiro_fire_suppression/thermal/Thermal+RGB_new.py
# Ctrl+A, D로 분리
```

### systemd 서비스로 등록 (선택사항)

`/etc/systemd/system/thermal-rgb.service` 파일 생성:

```ini
[Unit]
Description=Thermal+RGB RTSP Streaming Server
After=network.target

[Service]
Type=simple
User=khadas
WorkingDirectory=~/humiro_fire_suppression/thermal
ExecStart=/usr/bin/python3 ~/humiro_fire_suppression/thermal/Thermal+RGB_new.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

서비스 활성화:

```bash
sudo systemctl daemon-reload
sudo systemctl enable thermal-rgb.service
sudo systemctl start thermal-rgb.service
sudo systemctl status thermal-rgb.service
```

## 접속 방법

### RTSP 스트리밍

- **URL**: `rtsp://<VIM4_IP>:8554/stream`
- **클라이언트**: 
  - QGroundControl (QGC)
  - VLC Media Player
  - FFplay: `ffplay rtsp://<VIM4_IP>:8554/stream`

### HTTP 웹 스트리밍

- **URL**: `http://<VIM4_IP>:8080`
- **브라우저**: Chrome, Firefox, Safari 등

## 설정 변경

`Thermal+RGB_new.py` 파일 상단의 설정 변수 수정:

```python
# 웹 서버 활성화/비활성화
ENABLE_HTTP_SERVER = True  # False로 변경 시 웹 서버 비활성화
```

## 문제 해결

### 카메라 인식 실패

- 카메라 연결 확인: `ls -l /dev/video*`
- 카메라 이름 확인: `for dev in /sys/class/video4linux/video*; do echo "$(basename $dev): $(cat $dev/name)"; done`
- 프로세스 충돌 확인: `lsof /dev/video*`

### RTSP 연결 실패

- 방화벽 확인: `sudo ufw status`
- 포트 확인: `netstat -tuln | grep 8554`
- GStreamer 플러그인 확인: `gst-inspect-1.0 rtspclientsink`

## 로그 확인

```bash
# 실시간 로그 확인
tail -f /tmp/thermal_rgb.log

# 또는 systemd 서비스인 경우
sudo journalctl -u thermal-rgb.service -f
```

## 참고 사항

- Lepton 3.5 열화상 카메라는 8-9 FPS로 느려서 타임아웃 경고가 나타날 수 있으나 정상 동작입니다.
- RGB 카메라는 30 FPS로 동작하며 우선순위가 높습니다.

## 라이선스

이 프로젝트는 개인 사용 목적으로 작성되었습니다.
