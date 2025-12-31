# Quick Start Guide - 빠른 시작 가이드

작성일: 2025-12-31  
버전: v4.0 (targeting 재정의)

---

## 프로젝트 현황 (한눈에 보기)

**✅ 완료**:
1. 열화상 핫스팟 감지 (thermal/src/) - 2,665 LOC C++
2. LiDAR 거리 측정 (lidar/) - 1,188 LOC C++

**⏳ 다음 작업**:
3. **Targeting 시스템 (targeting/)** - 핫스팟 트래킹 + 드론 제어

---

## 시스템 정의 (v4.0 변경사항)

### targeting이란? ⭐ (재정의)

**핫스팟 트래킹 + 드론 위치 제어를 통한 정조준**

- thermal/src로부터 핫스팟 위치 수신
- 핫스팟 추적 (Kalman Filter)
- 드론 상하좌우 미세 조정 (PX4)
- 화면 중심에 핫스팟 유지
- GCS 격발 신호로 활성화

### lidar란?

**거리 측정 (10m 도착 판단)**

- LD19 LiDAR로 360° 스캔
- 10m 도착 여부 확인
- 거리 오버레이 (항상 표시)

---

## 화재 진압 시나리오

```
1. 접근 (lidar로 10m 확인)
   ↓
2. 대기 (GCS 격발 신호)
   ↓
3. Targeting 활성화 ⭐
   - 핫스팟 트래킹
   - 드론 정조준
   ↓
4. 발사 (LOCKED 후)
```

---

## 현재 상태

### ✅ thermal/src (완료)

**핫스팟 자동 감지**

```bash
cd ~/humiro_fire_suppression/thermal/src/build
./thermal_rgb_streaming
```

- RTSP: `rtsp://192.168.100.11:8554/stream`
- HTTP: `http://192.168.100.11:8080`
- 핫스팟 자동 감지 ✅
- RGB+Thermal 정합 ✅

### ✅ lidar/ (완료)

**거리 측정 (10m 도착 판단)**

```bash
cd ~/humiro_fire_suppression/lidar/src/build

# USB-UART 테스트
./lidar_test --usb-uart /dev/ttyUSB0

# GPIO-UART 배포
./lidar_test --gpio-uart /dev/ttyS1
```

- 360° 거리 스캔 ✅
- 거리 오버레이 (색상 코딩) ✅
- 10m 확인 ✅

### ⏳ targeting/ (다음 작업)

**핫스팟 트래킹 + 드론 제어**

```
targeting/
├── hotspot_tracker.cpp         ⏳ 핫스팟 추적
├── drone_position_controller.cpp ⏳ 드론 제어
├── targeting_manager.cpp       ⏳ GCS 신호
└── targeting_overlay.cpp       ⏳ 화면 표시
```

---

## 데이터 흐름

```
[thermal/src]  → 핫스팟 감지
      ↓
[lidar/]       → 거리 측정 (10m 확인)
      ↓
[GCS 신호]     → 격발 명령
      ↓
[targeting/]   → 핫스팟 트래킹 + 드론 제어
      ↓
[throwing_mech] → 발사
```

---

## 영상 오버레이

### 항상 표시
- **거리 라인** (lidar/): 녹색 9-11m, 빨간색 <9m, 파란색 >11m
- **핫스팟** (thermal/src/): 위치, 온도

### GCS 신호 후
- **트래킹 상태** (targeting/): "TRACKING ACTIVE", 십자선, "LOCKED"

---

## 빠른 명령어

### 환경 설정
```bash
source ~/humiro_fire_suppression/setup_env.sh
```

### thermal 빌드 & 실행
```bash
cd ~/humiro_fire_suppression/thermal/src
mkdir -p build && cd build
cmake .. && make -j$(nproc)
./thermal_rgb_streaming
```

### lidar 빌드 & 테스트
```bash
cd ~/humiro_fire_suppression/lidar/src
mkdir -p build && cd build
cmake .. && make -j$(nproc)

# 테스트
./lidar_test --usb-uart /dev/ttyUSB0
```

---

## C++ 코드 예제

### 1. 핫스팟 감지 (thermal/src - 이미 구현)
```cpp
// thermal_processor.cpp
cv::minMaxLoc(green, &min_val, &max_val, &min_loc, &max_loc);
data.hotspot_x = RGB_CROP_X + max_loc.x;
data.hotspot_y = RGB_CROP_Y + max_loc.y;
```

### 2. 거리 측정 (lidar/ - 이미 구현)
```cpp
// lidar_interface.cpp
LidarConfig config = LidarConfig::createGPIOUartConfig();
LidarInterface lidar(config);

if (lidar.start()) {
    LidarScan scan;
    lidar.getLatestScan(scan);
    float distance = scan.points[0].distance;  // 정면 거리
}
```

### 3. Targeting (구현 예정)
```cpp
// targeting_manager.cpp
TargetingManager targeting;

// GCS 신호로 활성화
targeting.onFireCommand(true);

while (targeting.isActive()) {
    targeting.update();
    
    if (targeting.isLocked()) {
        std::cout << "정조준 완료! LOCKED" << std::endl;
        // → throwing_mechanism 발사
    }
}
```

---

## 핵심 파일 위치

### 완료
```
thermal/src/
├── thermal_processor.cpp  ✅ 핫스팟 감지
├── frame_compositor.cpp   ✅ 영상 정합
└── rtsp/http_server.cpp   ✅ 스트리밍

lidar/src/
├── lidar_interface.cpp    ✅ LD19 통신
├── distance_overlay.cpp   ✅ 거리 오버레이
└── lidar_config.h         ✅ UART 설정
```

### 예정
```
targeting/
├── hotspot_tracker.cpp         ⏳ 핫스팟 추적
├── drone_position_controller.cpp ⏳ 드론 제어
├── targeting_manager.cpp       ⏳ GCS 신호
└── targeting_overlay.cpp       ⏳ 화면 표시
```

---

## 진행 상황

```
Phase 1: 열화상    [██████████] 100%  ✅
Phase 2: LiDAR     [██████████] 100%  ✅
Phase 3: Targeting [░░░░░░░░░░]   0%  ⏳ 다음
Phase 4: Throwing  [░░░░░░░░░░]   0%

전체: 50% 완료 (4단계 중 2단계)
```

---

## 다음 단계

### 이번 주
1. LD19 LiDAR 하드웨어 테스트
2. targeting/ 구현 시작 준비

### 1주 후
1. targeting/ Phase 1 구현
   - hotspot_tracker.cpp
   - targeting_overlay.cpp
   - targeting_manager.cpp

### 2주 후
1. targeting/ Phase 2 드론 제어
   - drone_position_controller.cpp
   - PX4 연동

---

## v4.0 주요 변경사항

### targeting 재정의 ⭐
- **이전**: targeting = 거리 측정
- **현재**: targeting = 핫스팟 트래킹 + 드론 제어

### 폴더 구조 변경
- `targeting/lidar_integration` → `lidar/`
- `targeting/` (새로 정의)

### 시나리오 반영
- GCS 격발 신호로 targeting 활성화
- 정조준 (LOCKED) 후 발사

---

## 참고 문서

- **상세 계획**: `work-plan/PROJECT_MASTER_PLAN.md`
- **Thermal 가이드**: `thermal/src/README.md`
- **LiDAR 가이드**: `lidar/README.md`
- **Targeting 설계**: `targeting/README.md`
- **개발 가이드**: `CLAUDE.md`

---

**다음 우선순위**: targeting/ 구현 시작

**작성일**: 2025-12-31  
**버전**: v4.0  
**상태**: targeting 재정의 반영 완료
