# QGC UDP 스트리밍 디버깅 가이드

## 문제: RTSP는 작동하지만 QGC UDP 스트리밍이 안 됨

## 디버깅 단계

### 1. UDP 패킷 전송 확인 (VIM4에서)
```bash
# tcpdump 설치
sudo apt install tcpdump

# UDP 패킷 전송 확인 (포트 5600)
sudo tcpdump -i any -n udp port 5600 -v

# 또는 특정 IP로 전송되는지 확인
sudo tcpdump -i any -n "udp port 5600 and host 192.168.100.56" -v
```

### 2. GStreamer 파이프라인 직접 테스트
```bash
# 테스트 패턴으로 UDP 전송 테스트
gst-launch-1.0 videotestsrc pattern=ball ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! \
  x264enc bitrate=3000 ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.100.56 port=5600

# QGC에서 이 테스트 스트림이 보이면 → GStreamer는 정상, 코드 문제
# QGC에서 안 보이면 → QGC 설정 또는 네트워크 문제
```

### 3. QGC 설정 확인
- **Video Source**: UDP h.264 비디오 스트림
- **UDP Port**: 5600
- **UDP URL**: 
  - 브로드캐스트: 비워두거나 `0.0.0.0`
  - 유니캐스트: VIM4 IP (`192.168.100.11`) 또는 비워두기

### 4. 네트워크 연결 확인
```bash
# VIM4에서 QGC PC로 ping
ping 192.168.100.56

# 포트 연결 테스트 (QGC PC에서)
# QGC PC에서 UDP 포트 5600이 열려있는지 확인
```

### 5. 방화벽 확인
```bash
# VIM4 방화벽 확인
sudo ufw status
sudo ufw allow 5600/udp

# QGC PC 방화벽 확인 (Windows)
# Windows 방화벽에서 포트 5600 UDP 허용
```

## 가능한 원인

### 1. GStreamer 파이프라인 문제
- `udpsink` 설정 문제
- 인코더 출력 형식 문제
- RTP 패킷 형식 문제

### 2. QGC 설정 문제
- UDP URL 설정 오류
- 포트 번호 불일치
- 비디오 소스 선택 오류

### 3. 네트워크 문제
- 방화벽 차단
- 라우팅 문제
- 브로드캐스트/멀티캐스트 설정

## 해결 방법

### 방법 1: GStreamer 파이프라인 수정
현재 코드의 파이프라인을 RTSP와 동일한 형식으로 변경

### 방법 2: QGC 설정 재확인
QGC에서 UDP 스트림 수신 설정을 다시 확인

### 방법 3: 네트워크 테스트
tcpdump로 실제 UDP 패킷 전송 확인
