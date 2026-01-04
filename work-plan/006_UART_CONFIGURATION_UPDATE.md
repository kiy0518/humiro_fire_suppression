# LiDAR UART 연결 설정 업데이트

작성일: 2025-12-31

## 업데이트 요약

LiDAR 모듈이 **USB-UART**와 **VIM4 GPIO UART** 두 가지 연결 방식을 모두 지원하도록 업데이트되었습니다.

### 업데이트된 파일

```
targeting/lidar_integration/
├── README.md                              # ✅ 업데이트 (USB/GPIO 연결 가이드)
└── src/
    ├── lidar_config.h                     # ✅ 신규 (연결 설정 관리)
    ├── lidar_interface.h                  # ✅ 업데이트 (설정 지원)
    ├── lidar_interface.cpp                # ✅ 업데이트 (GPIO UART 지원)
    ├── distance_overlay.h                 # (변경 없음)
    ├── distance_overlay.cpp               # (변경 없음)
    ├── main_test.cpp                      # ✅ 업데이트 (명령행 옵션)
    ├── lidar_connection_example.cpp       # ✅ 신규 (사용 예제)
    └── CMakeLists.txt                     # (변경 없음)
```

## 주요 변경 사항

### 1. 연결 설정 시스템 (`lidar_config.h`)

**새로운 기능**:
- `UartConnectionType` enum: USB_UART, GPIO_UART
- `LidarConfig` 구조체: 연결 설정 관리
- 편리한 팩토리 메서드:
  - `createUSBUartConfig()`: USB-UART 설정
  - `createGPIOUartConfig()`: GPIO-UART 설정

**VIM4 GPIO UART 핀 정보** (참고용):
```cpp
namespace VIM4_UART {
    constexpr int UART_A_TX_PIN = 8;      // GPIO 헤더 Pin 8
    constexpr int UART_A_RX_PIN = 10;     // GPIO 헤더 Pin 10
    constexpr const char* UART_A_DEV = "/dev/ttyS1";
}
```

### 2. LiDAR 인터페이스 업데이트

**새로운 생성자**:
```cpp
// 기존: USB-UART만 지원
LidarInterface(const std::string& device = "/dev/ttyUSB0");

// 신규: 설정 객체로 연결 타입 선택
LidarInterface(const LidarConfig& config);
```

**GPIO UART 특별 설정**:
- `configureGPIOUart()`: GPIO UART 전용 설정 (필요시)
- 에러 처리 개선: 연결 타입별 트러블슈팅 도움말

### 3. 테스트 프로그램 명령행 옵션

**새로운 사용법**:
```bash
# USB-UART (테스트)
./lidar_test                           # 기본: /dev/ttyUSB0
./lidar_test -u /dev/ttyUSB1 0         # USB-UART 포트 1

# GPIO-UART (배포)
./lidar_test --gpio-uart               # 기본: /dev/ttyS1
./lidar_test -g /dev/ttyS1 0           # GPIO-UART 명시

# 도움말
./lidar_test --help
```

**화면 표시**:
- 연결 타입 표시: "UART: USB-UART" 또는 "UART: GPIO-UART"
- 시작 시 설정 정보 출력

## 하드웨어 연결 가이드

### USB-UART 어댑터 (테스트용)

```
LD19 LiDAR          USB-UART 어댑터
  GND     ───────→   GND
  5V      ───────→   5V (또는 외부 전원)
  TX      ───────→   RX
  RX      ───────→   TX
```

**디바이스**: `/dev/ttyUSB0`

**설정**:
```bash
sudo chmod 666 /dev/ttyUSB0
# 또는
sudo usermod -a -G dialout $USER
```

### VIM4 GPIO UART (배포용)

```
VIM4 40핀 헤더       LD19 LiDAR
Pin 2  (5V)     ───→ 5V
Pin 6  (GND)    ───→ GND
Pin 8  (TX)     ───→ RX  (UART_A)
Pin 10 (RX)     ───→ TX  (UART_A)
```

**디바이스**: `/dev/ttyS1`

**설정**:
```bash
ls -l /dev/ttyS1             # 존재 확인
sudo chmod 666 /dev/ttyS1    # 권한 설정
```

## 코드 사용 예제

### 1. USB-UART로 테스트

```cpp
#include "lidar_interface.h"
#include "lidar_config.h"

// 방법 1: 기본 생성자 (USB-UART)
LidarInterface lidar("/dev/ttyUSB0");
lidar.start();

// 방법 2: 설정 객체 사용
LidarConfig config = LidarConfig::createUSBUartConfig("/dev/ttyUSB0");
LidarInterface lidar(config);
lidar.start();
```

### 2. GPIO-UART로 배포

```cpp
#include "lidar_interface.h"
#include "lidar_config.h"

// GPIO UART 설정
LidarConfig config = LidarConfig::createGPIOUartConfig("/dev/ttyS1");
LidarInterface lidar(config);

if (lidar.start()) {
    std::cout << "Type: " << config.getConnectionTypeString() << std::endl;
    // 출력: "Type: GPIO-UART"
}
```

### 3. 런타임 선택

```cpp
bool use_gpio = true;  // 환경 변수 또는 설정 파일에서 읽음

LidarConfig config;
if (use_gpio) {
    config = LidarConfig::createGPIOUartConfig();
} else {
    config = LidarConfig::createUSBUartConfig();
}

LidarInterface lidar(config);
lidar.start();
```

## 빌드 및 테스트

### 빌드
```bash
cd ~/humiro_fire_suppression/targeting/lidar_integration/src
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

### 테스트 (USB-UART)
```bash
# LiDAR 연결 확인
ls -l /dev/ttyUSB0

# 권한 설정
sudo chmod 666 /dev/ttyUSB0

# 테스트 실행
./lidar_test --usb-uart
```

### 실행 (GPIO-UART)
```bash
# UART 확인
ls -l /dev/ttyS1

# 권한 설정
sudo chmod 666 /dev/ttyS1

# 실행
./lidar_test --gpio-uart
```

## 문제 해결

### USB-UART 연결 안됨
```bash
# 디바이스 확인
ls -l /dev/ttyUSB*

# USB 디바이스 확인
lsusb | grep -i serial

# 권한 확인
sudo chmod 666 /dev/ttyUSB0

# 그룹 추가 (재로그인 필요)
sudo usermod -a -G dialout $USER
```

### GPIO-UART 연결 안됨
```bash
# UART 디바이스 확인
ls -l /dev/ttyS*

# 커널 로그 확인
dmesg | grep ttyS

# 권한 설정
sudo chmod 666 /dev/ttyS1

# UART 활성화 확인
dmesg | tail -20
```

## 선택 가이드

| 기준 | USB-UART | GPIO-UART |
|------|----------|-----------|
| **용도** | 개발/테스트 | 배포/실제 운용 |
| **디바이스** | /dev/ttyUSB0 | /dev/ttyS1 |
| **연결** | USB 포트 | GPIO 핀 (8, 10) |
| **안정성** | 보통 | 높음 (고정 연결) |
| **케이블** | USB 케이블 | 직접 와이어링 |
| **전원** | USB 또는 외부 | VIM4 5V 핀 |
| **권장** | 초기 개발 | 최종 제품 |

## 향후 개발 계획

### 단기 (1주)
- [ ] USB-UART로 하드웨어 테스트
- [ ] GPIO UART 연결 테스트
- [ ] 성능 비교 (USB vs GPIO)

### 중기 (1개월)
- [ ] 열화상 시스템과 통합
- [ ] 자동 디바이스 감지 기능
- [ ] 설정 파일 지원 (YAML/JSON)

### 장기 (2-3개월)
- [ ] 핫 스왑 지원 (연결 끊김 시 자동 재연결)
- [ ] ROS2 파라미터로 연결 타입 설정
- [ ] 다중 LiDAR 지원

## 결론

이제 LiDAR 모듈은 두 가지 연결 방식을 모두 지원합니다:

1. **개발 단계**: USB-UART로 빠르게 테스트
2. **배포 단계**: GPIO-UART로 안정적인 운용

**다음 작업**:
1. LD19 LiDAR 하드웨어 도착 대기
2. USB-UART로 초기 테스트
3. GPIO-UART로 최종 배포 준비

---

**작성자**: Claude Code Assistant  
**업데이트 일자**: 2025-12-31  
**상태**: 코드 완료, 하드웨어 테스트 대기
