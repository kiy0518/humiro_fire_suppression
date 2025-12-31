# QGroundControl 비디오 스트리밍 설정 가이드

## 개요
VIM4에서 Thermal + RGB 영상을 QGroundControl(QGC)로 실시간 스트리밍하는 방법입니다.

## 스트리밍 방식
- **프로토콜**: UDP H.264 (RTP)
- **해상도**: 1280x720 (설정 가능)
- **프레임레이트**: 30 FPS
- **비트레이트**: 3000 kbps (3 Mbps)
- **인코더**: VIM4 Amlogic 하드웨어 인코더 (amlvenc) - 빠른 전송 속도

## VIM4 설정

### 1. 환경 변수 설정 (선택사항)
```bash
# QGC IP 주소 설정 (브로드캐스트 또는 특정 IP)
export QGC_IP=192.168.100.255  # 브로드캐스트
# 또는
export QGC_IP=192.168.100.50    # 특정 QGC PC IP

# QGC UDP 포트 설정 (기본값: 5600)
export QGC_UDP_PORT=5600
```

### 2. 스크립트 실행
```bash
cd ~/humiro_fire_suppression/thermal
python3 Thermal+RGB.py
```

### 3. 출력 확인
스크립트 실행 시 다음과 같은 메시지가 표시됩니다:
```
============================================================
Thermal + RGB 스트리밍 서버 시작
============================================================
웹 브라우저: http://<VIM4_IP>:5000
QGC UDP 스트리밍: 192.168.100.255:5600
============================================================
QGC 설정 방법:
1. QGC > Settings > General > Video
2. Video Source: UDP
3. UDP Port: 5600
4. UDP Address: 192.168.100.255 (또는 브로드캐스트)
============================================================
```

## QGroundControl 설정

### 방법 1: UDP 스트리밍 (권장)

1. **QGC 실행**
   - QGroundControl v5.0.8 이상

2. **Settings 열기**
   - 상단 메뉴: `Settings` (톱니바퀴 아이콘)
   - 또는 단축키: `Ctrl + ,`

3. **Video 설정**
   - 왼쪽 메뉴에서 `General` 선택
   - `Video` 섹션으로 스크롤

4. **Video Source 설정**
   - `Video Source`: `UDP Video Stream` 선택
   - `UDP Port`: `5600` 입력
   - `UDP Address`: 
     - 브로드캐스트: `192.168.100.255` (모든 VIM4에서 수신)
     - 또는 특정 IP: `10.0.0.11` (특정 VIM4에서만 수신)

5. **적용**
   - `OK` 또는 `Apply` 클릭
   - QGC를 재시작할 필요 없음

6. **비디오 확인**
   - QGC 메인 화면에서 비디오 스트림이 자동으로 표시됩니다
   - 비디오 창이 보이지 않으면: `View` > `Video` 메뉴 확인

### 방법 2: RTSP 스트리밍 (고급)

RTSP를 사용하려면 코드를 수정하여 RTSP 서버를 추가해야 합니다.

## 네트워크 설정

### 포트 확인
```bash
# VIM4에서 포트가 열려있는지 확인
sudo netstat -ulnp | grep 5600
# 또는
sudo ss -ulnp | grep 5600
```

### 방화벽 설정 (필요시)
```bash
# UFW 방화벽이 활성화된 경우
sudo ufw allow 5600/udp
sudo ufw allow 5000/tcp  # 웹 스트리밍용
```

## 성능 최적화

### 1. 해상도 조정
코드에서 `QGC_STREAM_WIDTH`와 `QGC_STREAM_HEIGHT`를 수정:
```python
QGC_STREAM_WIDTH = 1280   # 낮추면 지연 감소
QGC_STREAM_HEIGHT = 720
```

### 2. 비트레이트 조정
GStreamer 파이프라인에서 `bitrate=3000` 값을 조정:
- 낮추면: 대역폭 절약, 품질 저하
- 높이면: 품질 향상, 대역폭 증가

### 3. FPS 조정
```python
QGC_FPS = 30  # 낮추면 CPU 사용량 감소
```

## 문제 해결

### 비디오가 표시되지 않음
1. **네트워크 확인**
   ```bash
   # VIM4에서 QGC PC로 ping
   ping 192.168.100.50
   ```

2. **포트 확인**
   ```bash
   # VIM4에서 UDP 패킷 전송 확인
   sudo tcpdump -i any -n udp port 5600
   ```

3. **GStreamer 확인**
   ```bash
   # GStreamer 플러그인 확인
   gst-inspect-1.0 x264enc
   gst-inspect-1.0 rtph264pay
   ```

### 지연이 발생함
1. 해상도 낮추기 (1280x720 → 640x480)
2. FPS 낮추기 (30 → 15)
3. 비트레이트 낮추기 (3000 → 1500)

### GStreamer 오류
```bash
# 필요한 플러그인 설치
sudo apt update
sudo apt install -y gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
                     gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
                     gstreamer1.0-libav \
                     gstreamer1.0-omx gstreamer1.0-aml

# VIM4 하드웨어 인코더 확인
gst-inspect-1.0 amlvenc
```

### 인코더 확인
VIM4는 하드웨어 인코더를 사용하므로 매우 빠른 전송 속도를 제공합니다:
- **하드웨어 인코더**: `amlvenc` (Amlogic H.264/H.265)
- **대체 인코더**: `openh264enc` (소프트웨어, 자동 폴백)

## 다중 드론 설정

3대의 드론이 있는 경우:
- 각 VIM4는 다른 UDP 포트 사용
- QGC에서 여러 비디오 소스 설정 가능

예시:
- Drone 1: `QGC_UDP_PORT=5600`
- Drone 2: `QGC_UDP_PORT=5602`
- Drone 3: `QGC_UDP_PORT=5604`

QGC에서 각 포트별로 비디오 소스를 추가 설정합니다.

## 참고사항

- **대역폭**: H.264 스트리밍은 약 3-5 Mbps 사용
- **지연**: 일반적으로 100-300ms (네트워크 상태에 따라 다름)
- **호환성**: QGC v5.0.8 이상 권장
