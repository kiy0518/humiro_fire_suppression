# LiDAR 프로그램 실행 가이드

## 빌드 완료 ✅

빌드가 완료되었습니다. 실행 파일: `build/lidar_test`

## 실행 방법

### 1. USB-UART 어댑터 사용 (테스트용)

```bash
cd ~/humiro_fire_suppression/lidar/src/build

# 기본 USB-UART 포트 (/dev/ttyUSB0) 사용
./lidar_test -u

# 또는 특정 USB-UART 포트 지정
./lidar_test -u /dev/ttyUSB1

# 카메라 ID 지정 (기본: 0)
./lidar_test -u /dev/ttyUSB0 1
```

### 2. GPIO-UART 사용 (VIM4 UART_E 배포용) ⭐

```bash
cd ~/humiro_fire_suppression/lidar/src/build

# 기본 GPIO-UART 포트 (/dev/ttyS4, UART_E) 사용
./lidar_test -g

# 또는 명시적으로 UART_E 지정
./lidar_test -g /dev/ttyS4

# 카메라 ID 지정
./lidar_test -g /dev/ttyS4 0
```

### 3. 도움말 보기

```bash
./lidar_test -h
# 또는
./lidar_test --help
```

## 실행 전 확인사항

### 1. 시리얼 포트 권한 확인

```bash
# USB-UART 포트 권한 확인
ls -l /dev/ttyUSB*

# 권한이 없으면 설정
sudo chmod 666 /dev/ttyUSB0

# 또는 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER
# (로그아웃 후 다시 로그인 필요)
```

### 2. GPIO-UART 활성화 (VIM4 UART_E)

```bash
# GPIO UART_E 확인 (기본적으로 활성화됨)
ls -l /dev/ttyS4

# 권한 확인 및 설정
sudo chmod 666 /dev/ttyS4
# 또는
sudo usermod -a -G dialout $USER
```

### 3. 카메라 확인

```bash
# USB 카메라 확인
lsusb | grep -i camera

# 비디오 장치 확인
ls -l /dev/video*
```

## 실행 예제

### 예제 1: USB-UART 기본 설정
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -u
```

### 예제 2: GPIO-UART (UART_E) + 카메라 1번
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -g /dev/ttyS4 1
```

### 예제 3: 특정 USB-UART 포트
```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test --usb-uart /dev/ttyUSB1
```

## 프로그램 동작

프로그램이 실행되면:

1. **LiDAR 연결**: 지정된 시리얼 포트로 LD19 LiDAR 연결 시도
2. **카메라 열기**: 지정된 카메라 ID로 비디오 캡처 시작
3. **거리 스캔**: 360° 거리 데이터 수신 (10Hz)
4. **오버레이 표시**: 
   - 거리 라인 (LINE SHAPE)
   - 색상 코딩:
     - 녹색 (9-11m): 최적 거리
     - 빨간색 (<9m): 너무 가까움
     - 파란색 (>11m): 너무 멀음
   - 거리 텍스트 표시

## 종료 방법

- `Ctrl+C`로 프로그램 종료

## 문제 해결

### 문제: "Permission denied" 오류
```bash
# 권한 설정
sudo chmod 666 /dev/ttyUSB0
# 또는
sudo usermod -a -G dialout $USER
```

### 문제: "Device not found" 오류
```bash
# USB 장치 확인
lsusb | grep -i serial

# 시리얼 포트 확인
ls -l /dev/ttyUSB* /dev/ttyS*
```

### 문제: "Camera not found" 오류
```bash
# 카메라 확인
lsusb | grep -i camera
ls -l /dev/video*

# 다른 카메라 ID 시도
./lidar_test -u 1  # 카메라 ID 1
```

### 문제: LiDAR 데이터가 수신되지 않음
- 전원 연결 확인 (5V, 350mA)
- TX/RX 핀 교차 확인
- Baudrate 확인 (230400)
- USB-UART 어댑터 드라이버 확인

## 추가 정보

- 상세 문서: `../README.md`
- 소스 코드: `../src/`
- 빌드 디렉토리: `./build/`

