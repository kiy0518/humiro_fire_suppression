# QGC 연결 문제 해결 가이드

## 문제: QGC에서 비디오 스트림이 보이지 않음

### 1. IP 주소 확인

**중요**: QGC의 UDP URL에 입력하는 IP는 **QGC PC의 IP 주소**가 아니라 **VIM4의 IP 주소**입니다!

- ❌ 잘못된 설정: `192.168.100.11:5600` (이건 VIM4 IP)
- ✅ 올바른 설정: QGC PC의 실제 IP 주소

**확인 방법**:
```bash
# VIM4에서 QGC IP 확인 (device_config.env)
cat ~/humiro_fire_suppression/device_config.env | grep QGC_IP

# 또는 QGC PC에서 자신의 IP 확인
ip addr show
# 또는 Windows: ipconfig
```

### 2. QGC 설정 방법

#### 방법 A: UDP URL 사용
- **UDP URL**: `192.168.100.11:5600`
  - `192.168.100.11`: VIM4의 WIFI_IP (비디오를 보내는 서버)
  - `5600`: UDP 포트

#### 방법 B: UDP Address + UDP Port 분리
- **UDP Address**: `192.168.100.11` (VIM4 IP)
- **UDP Port**: `5600`

**⚠️ 주의**: 
- QGC의 "UDP Address" 필드는 **수신할 주소**가 아니라 **송신 서버 주소**를 의미합니다
- 즉, VIM4의 IP 주소를 입력해야 합니다

### 3. 네트워크 연결 확인

```bash
# VIM4에서 QGC PC로 핑 테스트
ping <QGC_PC_IP>

# 예: QGC PC가 192.168.100.50이면
ping 192.168.100.50
```

### 4. UDP 패킷 전송 확인

#### VIM4에서 확인 (서버 측)
```bash
# UDP 패킷이 전송되는지 확인
sudo tcpdump -i any -n udp port 5600 -v

# 또는 간단히
sudo netstat -un | grep 5600
```

**정상적인 경우**: 패킷이 계속 전송되는 것이 보여야 합니다.

#### QGC PC에서 확인 (클라이언트 측)
```bash
# QGC PC에서 UDP 패킷 수신 확인
sudo tcpdump -i any -n udp port 5600 -v
```

### 5. 방화벽 확인

```bash
# VIM4에서 방화벽 상태 확인
sudo ufw status

# UDP 포트 허용 (필요시)
sudo ufw allow 5600/udp

# QGC PC에서도 방화벽 확인 (Windows)
# Windows 방화벽 설정에서 포트 5600 UDP 허용
```

### 6. 서버 실행 상태 확인

서버 실행 시 다음 메시지가 보여야 합니다:
```
✓ UDP H264 스트리밍 서버 초기화 성공
  → 클라이언트(QGC)로 전송: <QGC_IP>:5600
```

그리고 5초마다:
```
스트리밍 서버: 30.0 FPS (UDP, H264) → <QGC_IP>:5600
```

### 7. QGC 설정 재확인

1. **Settings > General > Video**
2. **Video Source**: `UDP h.264 비디오 스트림` 선택
3. **UDP URL**: `192.168.100.11:5600` (VIM4 IP:포트)
   - 또는
   - **UDP Address**: `192.168.100.11` (VIM4 IP)
   - **UDP Port**: `5600`

### 8. 일반적인 문제

#### 문제 1: IP 주소 혼동
- **증상**: 연결이 안 됨
- **원인**: QGC IP와 VIM4 IP를 혼동
- **해결**: 
  - VIM4 IP: `192.168.100.11` (서버, 비디오를 보냄)
  - QGC IP: `192.168.100.50` (클라이언트, 비디오를 받음)
  - QGC 설정에는 **VIM4 IP**를 입력!

#### 문제 2: 포트 불일치
- **증상**: 연결이 안 됨
- **원인**: 서버 포트와 QGC 포트가 다름
- **해결**: 둘 다 `5600`으로 설정 확인

#### 문제 3: 방화벽 차단
- **증상**: 패킷은 보이지만 QGC에서 안 보임
- **원인**: 방화벽이 UDP 패킷 차단
- **해결**: 방화벽에서 포트 5600 UDP 허용

#### 문제 4: 네트워크 분리
- **증상**: 연결이 안 됨
- **원인**: VIM4와 QGC PC가 다른 네트워크
- **해결**: 같은 WiFi 네트워크에 연결 확인

### 9. 디버깅 체크리스트

- [ ] 서버가 실행 중인가?
- [ ] 카메라가 정상 작동하는가?
- [ ] `스트리밍 서버: XX FPS` 메시지가 보이는가?
- [ ] `tcpdump`에서 UDP 패킷이 보이는가?
- [ ] VIM4와 QGC PC가 같은 네트워크인가?
- [ ] QGC 설정의 IP가 VIM4 IP인가?
- [ ] QGC 설정의 포트가 5600인가?
- [ ] 방화벽이 포트를 차단하지 않는가?

### 10. 빠른 테스트

```bash
# 1. 서버 실행
python3 RGB_Camera_Stream.py

# 2. 다른 터미널에서 패킷 확인
sudo tcpdump -i any -n udp port 5600 -c 10

# 3. 패킷이 보이면 서버는 정상, QGC 설정 확인
# 4. 패킷이 안 보이면 서버 문제, 로그 확인
```
