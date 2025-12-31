# LiDAR 프로그램 테스트 가이드

## 빠른 시작

### 1. 하드웨어 확인
```bash
cd ~/humiro_fire_suppression/lidar/src
./check_hardware.sh
```

### 2. 테스트 실행
```bash
cd ~/humiro_fire_suppression/lidar/src
./test_lidar.sh
```

## 상세 테스트 방법

### 방법 1: 자동 테스트 스크립트 사용 (권장)

```bash
cd ~/humiro_fire_suppression/lidar/src
./test_lidar.sh
```

스크립트가 다음을 자동으로 확인합니다:
- ✅ 빌드 상태
- ✅ 시리얼 포트 확인
- ✅ 카메라 확인
- ✅ 테스트 모드 선택

### 테스트 모드 선택

1. **USB-UART 테스트**: USB-UART 어댑터 사용
2. **GPIO-UART 테스트**: VIM4 UART_E 사용
3. **도움말**: 프로그램 옵션 확인
4. **종료**: 테스트 취소

### 방법 2: 수동 실행

#### USB-UART로 테스트
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -u /dev/ttyUSB0 0
```

#### GPIO-UART (UART_E)로 테스트
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -g /dev/ttyS4 0
```

#### 도움말 보기
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -h
```

## 하드웨어 연결 확인

### 1. 하드웨어 확인 스크립트 실행
```bash
cd ~/humiro_fire_suppression/lidar/src
./check_hardware.sh
```

### 2. 수동 확인

#### USB-UART 확인
```bash
# USB-UART 포트 확인
ls -l /dev/ttyUSB*

# 권한 확인
ls -l /dev/ttyUSB0
```

#### GPIO-UART (UART_E) 확인
```bash
# UART_E 포트 확인
ls -l /dev/ttyS4

# 권한 확인
ls -l /dev/ttyS4

# 권한이 없으면 설정
sudo chmod 666 /dev/ttyS4
```

#### 카메라 확인
```bash
# 카메라 장치 확인
ls -l /dev/video*

# USB 카메라 확인
lsusb | grep -i camera
```

## 테스트 시나리오

### 시나리오 1: 완전한 하드웨어 테스트

**필요한 하드웨어:**
- ✅ LD19 LiDAR (UART_E 연결)
- ✅ USB 카메라

**실행:**
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -g /dev/ttyS4 0
```

**예상 결과:**
- LiDAR 연결 성공
- 360° 거리 스캔 데이터 수신 (10Hz)
- 카메라 영상에 거리 오버레이 표시
- 색상 코딩: 녹색 (9-11m), 빨간색 (<9m), 파란색 (>11m)

### 시나리오 2: USB-UART 테스트

**필요한 하드웨어:**
- ✅ LD19 LiDAR (USB-UART 어댑터 연결)
- ✅ USB 카메라

**실행:**
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -u /dev/ttyUSB0 0
```

### 시나리오 3: 카메라만 테스트 (LiDAR 없음)

**필요한 하드웨어:**
- ✅ USB 카메라만

**실행:**
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -u /dev/ttyUSB0 0
```

**예상 결과:**
- LiDAR 연결 실패 메시지 (정상)
- "Running in camera-only mode..." 메시지
- 카메라 영상만 표시 (거리 오버레이 없음)

## 문제 해결

### 문제 1: "Permission denied" 오류

**증상:**
```
Failed to open serial port: Permission denied
```

**해결:**
```bash
# USB-UART 권한 설정
sudo chmod 666 /dev/ttyUSB0

# GPIO-UART (UART_E) 권한 설정
sudo chmod 666 /dev/ttyS4

# 또는 사용자를 dialout 그룹에 추가 (권장)
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인
```

### 문제 2: "Device not found" 오류

**증상:**
```
Failed to open serial port: No such file or directory
```

**해결:**
```bash
# USB-UART 확인
ls -l /dev/ttyUSB*

# GPIO-UART 확인
ls -l /dev/ttyS4

# USB 장치 확인
lsusb | grep -i serial

# 하드웨어 연결 확인
./check_hardware.sh
```

### 문제 3: "No data received" 오류

**증상:**
- LiDAR 연결은 성공하지만 데이터가 수신되지 않음

**해결:**
1. **하드웨어 연결 확인:**
   - TX/RX 핀 교차 확인
   - 전원 연결 확인 (5V, 350mA)
   - GND 연결 확인

2. **Baudrate 확인:**
   - LD19는 230400 baud 사용
   - 코드에서 이미 설정됨

3. **LiDAR 전원 확인:**
   - 전원 LED 확인
   - 전압 확인 (5V)

### 문제 4: "Camera not found" 오류

**증상:**
```
Failed to open camera 0
```

**해결:**
```bash
# 카메라 확인
ls -l /dev/video*

# 다른 카메라 ID 시도
./lidar_test -g /dev/ttyS4 1  # 카메라 ID 1

# USB 카메라 확인
lsusb | grep -i camera
```

### 문제 5: 프로그램이 빌드되지 않음

**해결:**
```bash
cd ~/humiro_fire_suppression/lidar/src
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## 테스트 체크리스트

### 사전 확인
- [ ] 빌드 완료 (`./lidar_test` 파일 존재)
- [ ] 하드웨어 연결 확인 (`./check_hardware.sh`)
- [ ] 시리얼 포트 권한 설정
- [ ] 카메라 연결 확인 (선택사항)

### 실행 확인
- [ ] 프로그램 시작 성공
- [ ] LiDAR 연결 성공 (또는 실패 메시지 정상)
- [ ] 카메라 열기 성공 (또는 카메라 없음 메시지)
- [ ] 영상 표시 (카메라 연결 시)
- [ ] 거리 오버레이 표시 (LiDAR 연결 시)

### 기능 확인
- [ ] 거리 데이터 수신 (10Hz)
- [ ] 색상 코딩 정상 작동
- [ ] 거리 텍스트 표시
- [ ] Ctrl+C로 정상 종료

## 추가 정보

### 로그 확인
프로그램 실행 중 콘솔에 다음 정보가 표시됩니다:
- 연결 타입 (USB-UART / GPIO-UART)
- 디바이스 경로
- Baudrate
- 카메라 ID
- FPS (프레임 속도)

### 성능 확인
- 목표 FPS: 30 FPS (카메라)
- LiDAR 스캔 속도: 10Hz
- 지연시간: 최소화

### 관련 파일
- 테스트 스크립트: `test_lidar.sh`
- 하드웨어 확인: `check_hardware.sh`
- 실행 가이드: `RUN_LIDAR.md`
- UART_E 설정: `UART_E_SETUP.md`
- 메인 문서: `../README.md`

