# LiDAR Distance Measurement - 거리 측정 시스템

LD19 LiDAR를 이용한 열원까지의 거리 확인 (10m 도착 판단)

## 개요

**역할**: 열원까지의 거리를 측정하여 10m 도착 여부 판단

**현재 상태**: ✅ 완료 (1,188 LOC C++, 하드웨어 테스트 대기)

---

## 시스템 정의

### lidar란?
- **거리 측정**: LD19 LiDAR로 360° 스캔
- **10m 판단**: 열원까지 10m 도착 여부 확인
- **거리 오버레이**: 화면에 거리 정보 표시 (색상 코딩)

### lidar가 아닌 것
- ❌ 핫스팟 감지 → `thermal/src/` 담당
- ❌ 핫스팟 트래킹 → `targeting/` 담당
- ❌ 드론 제어 → `targeting/` 담당

---

## 하드웨어

### LDROBOT LD19 LiDAR
- **타입**: 360° ToF 스캔
- **거리**: 0.05m ~ 12m
- **정확도**: ±2cm
- **스캔 속도**: 10Hz (4500 points/sec)
- **인터페이스**: UART 230400 baud
- **전원**: 5V DC, ~350mA

### 연결 방식

#### 1. USB-UART (테스트용)
```
LD19          USB-UART
GND   ───→   GND
5V    ───→   5V
TX    ───→   RX
RX    ───→   TX
```
**디바이스**: `/dev/ttyUSB0`

#### 2. GPIO-UART (배포용, VIM4 UART_E) ⭐
```
LD19          VIM4 40-pin Header
GND   ───→   Pin 6  (GND)
5V    ───→   Pin 2  (5V)
TX    ───→   Pin 15 (UART_E RX)
RX    ───→   Pin 16 (UART_E TX)
```
**디바이스**: `/dev/ttyS4` (UART_E)

---

## 구현 완료

### 파일 구조
```
lidar/
├── src/
│   ├── lidar_config.h          # UART 설정 (USB/GPIO)
│   ├── lidar_interface.h/cpp   # LD19 통신
│   ├── distance_overlay.h/cpp  # 거리 시각화
│   └── main_test.cpp           # 테스트 프로그램
└── README.md
```

### 주요 기능

#### 1. UART 설정 (lidar_config.h)
```cpp
enum class UartConnectionType {
    USB_UART,   // USB-UART 어댑터
    GPIO_UART   // VIM4 GPIO UART
};

struct LidarConfig {
    UartConnectionType connection_type;
    std::string device_path;
    int baudrate;
    
    static LidarConfig createUSBUartConfig(
        const std::string& device = "/dev/ttyUSB0");
    
    static LidarConfig createGPIOUartConfig(
        const std::string& device = "/dev/ttyS1");
};
```

#### 2. LD19 통신 (lidar_interface.cpp)
```cpp
class LidarInterface {
public:
    LidarInterface(const LidarConfig& config);
    
    bool start();
    void stop();
    
    // 최신 스캔 데이터
    bool getLatestScan(LidarScan& scan);
    
    // 특정 각도 범위의 거리
    std::vector<LidarPoint> getRangeData(
        float start_angle, float end_angle);
    
    bool isConnected() const;
};
```

#### 3. 거리 오버레이 (distance_overlay.cpp)
```cpp
class DistanceOverlay {
public:
    DistanceOverlay(float camera_fov = 60.0f, 
                    int image_width = 640, 
                    int image_height = 480);
    
    // 거리 라인 그리기
    void drawDistanceLine(cv::Mat& frame, 
                          const std::vector<LidarPoint>& lidar_data);
    
    // 거리 정보 텍스트
    void drawDistanceInfo(cv::Mat& frame, 
                          const std::vector<LidarPoint>& lidar_data);
    
    // 색상 코딩
    cv::Scalar getColorForDistance(float distance);
};
```

### 색상 코딩
- **녹색** (9-11m): 최적 거리 (발사 준비)
- **빨간색** (<9m): 너무 가까움
- **파란색** (>11m): 너무 멀음

---

## 빌드 및 실행

### 빌드
```bash
cd ~/humiro_fire_suppression/lidar/src
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 실행

#### USB-UART로 테스트
```bash
./lidar_test --usb-uart /dev/ttyUSB0
```

#### GPIO-UART로 배포 (UART_E)
```bash
./lidar_test --gpio-uart /dev/ttyS4
# 또는 기본값 사용 (자동으로 /dev/ttyS4)
./lidar_test -g
```

### 옵션
```
--usb-uart <device>   USB-UART 어댑터 사용
--gpio-uart <device>  VIM4 GPIO UART_E 사용 (기본: /dev/ttyS4)
--camera <id>         카메라 ID (기본: 0)
```

---

## 데이터 흐름

```
[LD19 LiDAR]
    ↓ UART 230400
[LidarInterface]
    ↓ 360° 스캔 데이터
[DistanceOverlay]
    ↓ 거리 라인 + 색상
[영상 출력]
    - 거리 정보 표시
    - 10m 여부 확인
```

---

## 사용 예제

### C++ 코드
```cpp
#include "lidar_interface.h"
#include "distance_overlay.h"

// LiDAR 설정
LidarConfig config = LidarConfig::createGPIOUartConfig();
LidarInterface lidar(config);

// 거리 오버레이
DistanceOverlay overlay(60.0f, 640, 480);

// LiDAR 시작
if (!lidar.start()) {
    std::cerr << "LiDAR 연결 실패" << std::endl;
    return -1;
}

// 메인 루프
cv::Mat frame;
while (true) {
    // 카메라 프레임 받기
    camera >> frame;
    
    // LiDAR 스캔 데이터
    LidarScan scan;
    if (lidar.getLatestScan(scan)) {
        // 거리 오버레이 그리기
        overlay.drawDistanceLine(frame, scan.points);
        overlay.drawDistanceInfo(frame, scan.points);
        
        // 10m 거리 확인
        float front_distance = scan.points[0].distance;  // 정면
        if (front_distance >= 9.0f && front_distance <= 11.0f) {
            std::cout << "10m 도착! 거리: " << front_distance << "m" << std::endl;
        }
    }
    
    cv::imshow("LiDAR", frame);
}
```

---

## 통합 사용 (시나리오)

### 1. 접근 단계
```cpp
// 거리 확인
float distance = getCurrentDistance();

if (distance > 11.0f) {
    std::cout << "10m까지 접근 중..." << std::endl;
} else if (distance >= 9.0f && distance <= 11.0f) {
    std::cout << "10m 도착! GCS 신호 대기" << std::endl;
    // → targeting/ 모듈로 제어 넘김
} else {
    std::cout << "너무 가까움!" << std::endl;
}
```

### 2. GCS 신호 후
```cpp
// targeting/ 모듈 활성화
targeting_manager.onFireCommand(true);

// lidar는 계속 거리 모니터링
while (targeting_manager.isActive()) {
    float distance = getCurrentDistance();
    overlay.drawDistanceLine(frame, lidar_data);
    
    // 거리 이탈 시 경고
    if (distance < 8.0f || distance > 12.0f) {
        std::cerr << "거리 범위 이탈!" << std::endl;
        targeting_manager.onFireCommand(false);
    }
}
```

---

## 화면 오버레이 통합

lidar 오버레이는 **항상 표시**됩니다:

1. **거리 라인** (LINE SHAPE)
   - 화면 상단에 수평선
   - 색상으로 거리 표시

2. **거리 텍스트**
   - 정면 거리 표시 (예: "10.2m")
   - 색상 코딩과 동일

3. **다른 오버레이와 통합**
   - thermal/src: 핫스팟 표시
   - targeting/: 트래킹 상태 (GCS 신호 후)

---

## 문제 해결

### LiDAR 연결 안 됨
```bash
# USB 디바이스 확인
lsusb | grep -i serial

# 권한 확인
sudo chmod 666 /dev/ttyUSB0

# GPIO UART 활성화 (VIM4)
sudo dtoverlay enable uart_a
```

### 데이터 수신 안 됨
- Baudrate 확인 (230400)
- TX/RX 핀 교차 확인
- 전원 확인 (5V, 350mA)

---

## 코드 통계

- **총 LOC**: 1,188 (C++)
- **파일 수**: 7개 (.h/.cpp)
- **상태**: ✅ 완료 (하드웨어 테스트 대기)

---

## 다음 단계

1. ⏳ LD19 하드웨어 도착
2. ⏳ USB-UART 테스트
3. ⏳ GPIO-UART 연동
4. ⏳ 10m 거리 정확도 검증

---

**작성일**: 2025-12-31  
**상태**: 구현 완료 (하드웨어 테스트 대기)  
**우선순위**: Phase 2 ✅
