# VIM4 UART_E 설정 가이드

## 개요

LiDAR는 VIM4의 **UART_E** 포트를 사용합니다.

- **디바이스**: `/dev/ttyS4`
- **핀 매핑**: 
  - TX: Pin 16 (Header 40-pin)
  - RX: Pin 15 (Header 40-pin)
  - GND: Pin 6
  - 5V: Pin 2 (필요시)

## 하드웨어 연결

```
LD19 LiDAR          VIM4 40-Pin Header
─────────────────────────────────────────
GND          ───→   Pin 6  (GND)
5V           ───→   Pin 2  (5V, 필요시)
TX           ───→   Pin 15 (UART_E RX)
RX           ───→   Pin 16 (UART_E TX)
```

## 소프트웨어 설정

### 1. UART_E 활성화 (필수) ⭐

VIM4에서 UART_E를 활성화하려면 `/boot/overlays/kvim4.dtb.overlay.env` 파일을 수정해야 합니다.

#### 자동 활성화 (권장)
```bash
cd ~/humiro_fire_suppression/lidar/src
./enable_uart_e.sh
```

이 스크립트가 자동으로:
- 설정 파일 확인
- `fdt_overlays=uart_e` 추가
- 백업 생성

#### 수동 활성화
```bash
# 설정 파일 편집
sudo nano /boot/overlays/kvim4.dtb.overlay.env

# 다음 내용으로 설정:
fdt_overlays=uart_e
```

**⚠️ 중요: 재부팅 필요**
```bash
sudo reboot
```

### 2. 재부팅 후 UART_E 포트 확인

```bash
# UART_E 디바이스 확인
ls -l /dev/ttyS4

# 출력 예시:
# crw-rw---- 1 root dialout 506, 4 Jan  1  1970 /dev/ttyS4
```

### 3. 권한 설정

```bash
# 방법 1: 직접 권한 설정
sudo chmod 666 /dev/ttyS4

# 방법 2: 사용자를 dialout 그룹에 추가 (권장)
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인 필요
```

### 4. 프로그램 실행

```bash
cd ~/humiro_fire_suppression/lidar/src/build

# UART_E 사용 (기본값)
./lidar_test -g

# 또는 명시적으로 지정
./lidar_test -g /dev/ttyS4

# 카메라 ID 지정
./lidar_test -g /dev/ttyS4 0
```

## 코드 변경 사항

### lidar_config.h

- 기본 GPIO-UART 포트를 `/dev/ttyS1` → `/dev/ttyS4`로 변경
- `createUartEConfig()` 함수 추가
- `VIM4_UART` 네임스페이스에 UART_E 정보 추가

### 사용 예제

```cpp
// 방법 1: 기본 GPIO-UART (자동으로 UART_E 사용)
LidarConfig config = LidarConfig::createGPIOUartConfig();
// → /dev/ttyS4 사용

// 방법 2: UART_E 명시적 사용
LidarConfig config = LidarConfig::createUartEConfig();
// → /dev/ttyS4 사용

// 방법 3: 직접 지정
LidarConfig config = LidarConfig::createGPIOUartConfig("/dev/ttyS4");
```

## 활성화 완료 확인

### 1. 설정 파일 확인

```bash
# 활성화 스크립트 실행 후
cat /boot/overlays/kvim4.dtb.overlay.env

# 출력 예시:
# fdt_overlays=uart_e
```

### 2. 재부팅

```bash
sudo reboot
```

### 3. 재부팅 후 확인

```bash
# UART_E 포트 확인
ls -l /dev/ttyS4

# 권한 확인
groups  # dialout 그룹에 포함되어 있는지 확인
```

### 4. 프로그램 실행

```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -g /dev/ttyS4 0 -n
```

### 3. 문제 해결

**문제: "Permission denied"**
```bash
sudo chmod 666 /dev/ttyS4
# 또는
sudo usermod -a -G dialout $USER
# (로그아웃 후 다시 로그인)
```

**문제: "Device not found"**
```bash
# UART_E가 활성화되어 있는지 확인
ls -l /dev/ttyS*

# 필요시 device tree 확인
cat /proc/device-tree/serial*/status
```

**문제: "No data received"**
- 하드웨어 연결 확인 (TX/RX 교차 확인)
- 전원 확인 (5V, 350mA)
- Baudrate 확인 (230400)
- LiDAR 전원 LED 확인

## 참고

- VIM4 UART_E는 기본적으로 활성화되어 있습니다
- `/dev/ttyS4`는 UART_E에 매핑됩니다
- 다른 UART 포트와 충돌하지 않도록 확인하세요

## 관련 파일

- 설정: `lidar_config.h`
- 인터페이스: `lidar_interface.h/cpp`
- 실행: `main_test.cpp`
- 문서: `README.md`, `RUN_LIDAR.md`

