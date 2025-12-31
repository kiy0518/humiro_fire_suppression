# LiDAR 프로그램 문제 해결 가이드

## 문제 1: UART_E 포트가 없음 (`/dev/ttyS4`)

### 증상
```
Failed to open serial port: /dev/ttyS4
Error: No such file or directory
```

### 원인
- UART_E가 디바이스 트리에서 비활성화됨
- 커널 모듈이 로드되지 않음

### 해결 방법

#### 방법 1: UART_E 활성화 스크립트 실행
```bash
cd ~/humiro_fire_suppression/lidar/src
./enable_uart_e.sh
```

#### 방법 2: 수동 활성화
```bash
# UART 드라이버 확인
lsmod | grep uart

# UART 드라이버 로드 시도
sudo modprobe meson_uart

# 확인
ls -l /dev/ttyS4
```

#### 방법 3: 재부팅 후 확인
일부 VIM4 보드는 재부팅 후 UART_E가 자동으로 활성화됩니다.

#### 방법 4: USB-UART 사용 (임시 해결책)
UART_E가 활성화되지 않는 경우, USB-UART 어댑터를 사용할 수 있습니다:
```bash
./lidar_test -u /dev/ttyUSB0 0
```

---

## 문제 2: OpenCV GTK 백엔드 초기화 실패

### 증상
```
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.5.4) ./modules/highgui/src/window_gtk.cpp:635: 
  error: (-2:Unspecified error) Can't initialize GTK backend
```

### 원인
- GUI 환경이 없음 (SSH 접속, headless 서버)
- DISPLAY 환경 변수가 설정되지 않음
- X11 포워딩이 설정되지 않음

### 해결 방법

#### 방법 1: Headless 모드 사용 (권장) ⭐
GUI 없이 실행:
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -g /dev/ttyS4 0 -n
```

또는 테스트 스크립트에서 옵션 3 선택:
```bash
./test_lidar.sh
# 옵션 3 선택: GPIO-UART (UART_E) 테스트 - GUI 없음
```

#### 방법 2: DISPLAY 설정 (X11 포워딩)
SSH로 접속한 경우:
```bash
# SSH 접속 시 X11 포워딩 활성화
ssh -X user@hostname

# 또는
export DISPLAY=:0.0
./lidar_test -g /dev/ttyS4 0
```

#### 방법 3: VNC 사용
원격 데스크톱 환경 사용:
```bash
# VNC 서버 시작
vncserver :1

# DISPLAY 설정
export DISPLAY=:1
./lidar_test -g /dev/ttyS4 0
```

---

## 문제 3: 권한 오류

### 증상
```
Failed to open serial port: Permission denied
```

### 해결 방법
```bash
# UART_E 권한 설정
sudo chmod 666 /dev/ttyS4

# 또는 사용자를 dialout 그룹에 추가 (권장)
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인
```

---

## 문제 4: LiDAR 데이터가 수신되지 않음

### 증상
- LiDAR 연결은 성공하지만 데이터가 수신되지 않음
- "No data received" 메시지

### 해결 방법

1. **하드웨어 연결 확인:**
   ```
   LD19 TX → VIM4 Pin 15 (UART_E RX)
   LD19 RX → VIM4 Pin 16 (UART_E TX)
   LD19 GND → VIM4 Pin 6 (GND)
   LD19 5V → VIM4 Pin 2 (5V)
   ```

2. **전원 확인:**
   - LiDAR 전원 LED 확인
   - 전압 확인 (5V, 350mA)

3. **Baudrate 확인:**
   - LD19는 230400 baud 사용 (코드에서 이미 설정됨)

4. **TX/RX 교차 확인:**
   - TX와 RX가 올바르게 교차 연결되었는지 확인

---

## 문제 5: 카메라를 찾을 수 없음

### 증상
```
Failed to open camera 0
```

### 해결 방법
```bash
# 카메라 확인
ls -l /dev/video*

# 다른 카메라 ID 시도
./lidar_test -g /dev/ttyS4 1  # 카메라 ID 1

# USB 카메라 확인
lsusb | grep -i camera
```

---

## 빠른 진단

### 1. 하드웨어 확인
```bash
cd ~/humiro_fire_suppression/lidar/src
./check_hardware.sh
```

### 2. UART_E 활성화 확인
```bash
ls -l /dev/ttyS4
```

### 3. Headless 모드로 테스트
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -g /dev/ttyS4 0 -n
```

---

## 테스트 시나리오별 해결책

### 시나리오 1: SSH로 접속한 경우
**문제**: GUI 환경 없음
**해결**: Headless 모드 사용
```bash
./lidar_test -g /dev/ttyS4 0 -n
```

### 시나리오 2: UART_E가 활성화되지 않은 경우
**문제**: `/dev/ttyS4` 없음
**해결**: USB-UART 사용 또는 UART_E 활성화
```bash
# USB-UART 사용
./lidar_test -u /dev/ttyUSB0 0 -n

# 또는 UART_E 활성화
./enable_uart_e.sh
```

### 시나리오 3: 카메라 없이 테스트
**문제**: 카메라 없음
**해결**: 카메라 없이도 실행 가능 (LiDAR만 테스트)
```bash
./lidar_test -g /dev/ttyS4 0 -n
# 카메라 오류는 무시하고 LiDAR만 테스트
```

---

## 추가 도움말

- 상세 테스트 가이드: `TEST_GUIDE.md`
- UART_E 설정: `UART_E_SETUP.md`
- 실행 가이드: `RUN_LIDAR.md`

