# Humiro Fire Suppression - 프로젝트 마스터 플랜 v4.0

작성일: 2025-12-31  
**상태**: targeting 재정의 반영 완료

---

## 프로젝트 개요

**목적**: 드론 기반 자동 소화 시스템

**핵심 프로세스**:
1. ✅ 열화상으로 화재 감지 (핫스팟)
2. ✅ LiDAR로 거리 측정 (10m 도착 판단)
3. ⏳ GCS 격발 신호 수신
4. ⏳ 핫스팟 트래킹 + 드론 정조준
5. ⏳ 소화탄 발사

**기술 스택**:
- **개발 언어**: C++17 (우선)
- **플랫폼**: Khadas VIM4 (Ubuntu 22.04 ARM64)
- **빌드**: CMake + Make
- **ROS2**: Humble

---

## 시스템 재정의 (v4.0 핵심 변경)

### targeting의 의미 변경

**이전 (v3.0 - 잘못된 이해)**:
- targeting = 거리 측정 + 탄도 계산

**현재 (v4.0 - 올바른 이해)**:
- **targeting** = **핫스팟 트래킹 + 드론 위치 제어** (정조준)
- **lidar** = **거리 측정** (10m 도착 판단)

### 폴더 구조 변경
```
lidar/        (이전: targeting/lidar_integration)
targeting/    (새로 정의: 핫스팟 트래킹 + 드론 제어)
application/  (통합 애플리케이션, 최신 리팩토링)
thermal/src/  (라이브러리로 변경, thermal_lib)
streaming/src/ (라이브러리로 변경, streaming_lib)
```

---

## 화재 진압 시나리오

### Phase 1: 접근 (자율 비행)
- 열원까지 10m 지점으로 자율 비행
- **lidar/** 로 거리 모니터링
- 10m 도착 확인

### Phase 2: 대기
- 10m 지점 호버링
- **GCS 격발 신호 대기**

### 프로젝트 Phase 3: Targeting 활성화 ⭐ (핵심)
- **GCS 격발 신호 수신** (ROS2 토픽)
- **targeting/** 모듈 활성화 (드론 제어 기능 포함)
- **핫스팟 트래킹 시작** (thermal/src 데이터 사용, 이미 구현됨)
- **드론 상하좌우 미세 조정** (PX4 제어, 프로젝트 Phase 3에서 추가)
- **정조준 유지** (화면 중심에 핫스팟)
- **LOCKED 판정** (오차 < 임계값)

### Phase 4: 발사
- 정조준 완료 (LOCKED) 확인
- **throwing_mechanism/** 발사 실행

---

## 현재 진행 상황

### ✅ Phase 1: 열화상 시스템 (완료 - 100%)

**위치**: `thermal/src/`  
**코드**: 2,665 LOC C++  
**상태**: ✅ 완전 구현 및 테스트 완료

**구현 기능**:
- 핫스팟 자동 감지 (`thermal_processor.cpp`) ✅
- RGB+Thermal 영상 정합 (`frame_compositor.cpp`) ✅
- RTSP/HTTP 스트리밍 ✅
- **개선 예정**: 비동기식 스트리밍 (준비되는 데이터부터 즉시 전송) ⏳
  - 참조: `work-plan/STREAMING_PLAN.md`

**핵심 코드**:
```cpp
// thermal_processor.cpp
bool extract_thermal_data(const cv::Mat& thermal_frame, ThermalData& data) {
    // 녹색 채널에서 최대값 찾기 (핫스팟)
    cv::minMaxLoc(green, &min_val, &max_val, &min_loc, &max_loc);
    
    // 핫스팟 위치
    data.hotspot_x = RGB_CROP_X + max_loc.x;
    data.hotspot_y = RGB_CROP_Y + max_loc.y;
    data.valid = true;
}
```

### ✅ 프로젝트 Phase 2: LiDAR 거리 측정 (완료 - 100%)

**위치**: `lidar/src/` (이전: targeting/lidar_integration)  
**코드**: 1,188 LOC C++  
**상태**: ✅ 완전 구현 (하드웨어 테스트 대기)

**역할 재정의**:
- **거리 측정** (10m 도착 판단)
- **거리 오버레이** (항상 표시)
- ❌ ~~targeting~~ (잘못된 이해)

**구현 기능**:
- LD19 UART 통신 (`lidar_interface.cpp`) ✅
- 360° 거리 스캔 ✅
- 거리 오버레이 (색상 코딩) ✅
  - 녹색 (9-11m): 최적
  - 빨간색 (<9m): 너무 가까움
  - 파란색 (>11m): 너무 멀음

**핵심 코드**:
```cpp
// distance_overlay.cpp
cv::Scalar getColorForDistance(float distance) {
    if (distance >= 9.0f && distance <= 11.0f) {
        return cv::Scalar(0, 255, 0);  // 녹색 (최적)
    } else if (distance < 9.0f) {
        return cv::Scalar(0, 0, 255);  // 빨간색
    } else {
        return cv::Scalar(255, 0, 0);  // 파란색
    }
}
```

### ⏳ 프로젝트 Phase 3: Targeting 시스템 (미구현 - 0%)

**참고**: 이것은 **프로젝트 기능 Phase 3**입니다. 아키텍처 리팩토링의 Phase 3 (ROS2 통신 강화)와는 다릅니다.

**위치**: `targeting/`  
**코드**: 0 LOC (설계 완료)  
**상태**: ⏳ 구현 대기

**역할 재정의** ⭐:
- **핫스팟 트래킹** (Kalman Filter)
- **드론 위치 제어** (PX4 offboard mode)
- **정조준 유지** (화면 중심에 핫스팟)
- **GCS 신호 처리**

**구현 계획** (프로젝트 Phase 3에서 추가할 부분):
```cpp
// drone_position_controller.cpp - 드론 제어 (신규)
class DronePositionController {
    bool adjustPosition(float dx, float dy);  // 상하좌우 미세 조정
    bool holdPosition();                      // 위치 유지
};

// targeting_manager.cpp - 통합 관리 (확장)
class TargetingManager {
    void onFireCommand(bool enable);  // GCS 신호
    bool isLocked() const;            // 정조준 완료
    void update();                    // 메인 루프
};
```

**참고**: `hotspot_tracker`, `targeting_overlay`의 기본 구조는 이미 아키텍처 리팩토링에서 구현되었습니다.
프로젝트 Phase 3에서는 `drone_position_controller`를 추가하고 `targeting_manager`를 확장합니다.

**개발 단계**:
1. Phase 1 (1주): 기본 구조
   - hotspot_tracker.cpp
   - targeting_overlay.cpp
   - targeting_manager.cpp (기본)

2. Phase 2 (1주): 드론 제어
   - drone_position_controller.cpp
   - PX4 연동
   - 미세 조정 로직

3. Phase 3 (1주): GCS 통합
   - ROS2 토픽 구독
   - 활성화/비활성화
   - 전체 시스템 테스트

### ⏳ Phase 4: 발사 메커니즘 (미구현 - 0%)

**위치**: `throwing_mechanism/`  
**코드**: 0 LOC (README만)  
**상태**: ⏳ 설계 단계

**구현 계획**:
- 서보 각도 제어 (고정 각도) - C++
- GPIO 발사 트리거 - C++
- 탄도 계산 불필요 (10m 고정)

---

## 전체 시스템 데이터 흐름

```
┌──────────────────────┐
│  thermal/src/        │  ✅ 핫스팟 감지
└──────────────────────┘
          ↓
    핫스팟 위치 (x, y)
          ↓
┌──────────────────────┐
│  lidar/              │  ✅ 거리 측정
└──────────────────────┘
          ↓
    10m 도착 확인
          ↓
┌──────────────────────┐
│  GCS 격발 신호       │
└──────────────────────┘
          ↓
┌──────────────────────┐
│  targeting/          │  ⏳ 트래킹 + 드론 제어
│  ├─ 핫스팟 추적      │
│  ├─ 오차 계산        │
│  ├─ 드론 미세 조정   │
│  └─ 정조준 유지      │
└──────────────────────┘
          ↓
    정조준 완료 (LOCKED)
          ↓
┌──────────────────────┐
│  throwing_mechanism/ │  ⏳ 발사
└──────────────────────┘
```

---

## 영상 오버레이 통합

### 항상 표시
1. **거리 오버레이** (`lidar/`)
   - LINE SHAPE 표시
   - 색상 코딩 (녹색 9-11m, 빨간색 <9m, 파란색 >11m)
   - 거리 텍스트

2. **핫스팟 감지** (`thermal/src/`)
   - 핫스팟 위치 (원 또는 마커)
   - 온도 정보

### GCS 신호 후에만 표시
3. **트래킹 상태** (`targeting/`)
   - "TRACKING ACTIVE" 텍스트
   - 화면 중심 십자선
   - 오차 벡터 (중심 → 핫스팟)
   - "LOCKED" 표시 (정조준 완료 시)

---

## 개발 로드맵

### ✅ Phase 1: 열화상 (완료)
- [x] 핫스팟 감지 (thermal_processor.cpp)
- [x] RGB+Thermal 정합 (frame_compositor.cpp)
- [x] RTSP/HTTP 스트리밍

### ✅ Phase 2: LiDAR (완료)
- [x] LD19 UART 통신 (lidar_interface.cpp)
- [x] 거리 오버레이 (distance_overlay.cpp)
- [x] USB-UART / GPIO-UART 지원

### ⏳ 프로젝트 Phase 3: Targeting (다음 작업)
- [ ] hotspot_tracker.cpp (핫스팟 추적)
- [ ] drone_position_controller.cpp (드론 제어)
- [ ] targeting_manager.cpp (GCS 신호)
- [ ] targeting_overlay.cpp (화면 표시)

### ⏳ Phase 4: Throwing (추후)
- [ ] servo_controller.cpp
- [ ] fire_trigger.cpp

---

## 진행 상황 요약

```
프로젝트 Phase 1: 열화상       [██████████] 100%  ✅ 완료
프로젝트 Phase 2: LiDAR        [██████████] 100%  ✅ 완료 (HW 대기)
프로젝트 Phase 3: Targeting    [██░░░░░░░░]  20%  ⏳ 부분 완료 (드론 제어 기능 추가 필요)
프로젝트 Phase 4: Throwing     [░░░░░░░░░░]   0%  ⏳ 추후

전체: 55% 완료 (4단계 중 2단계 완료 + 1단계 부분 완료)
코드: 64% 완료 (3,853 / 5,953 LOC)

참고: 아키텍처 리팩토링 Phase 1-3는 모두 완료되었습니다.
```

---

## 코드 통계

### 완료
- `thermal/src/`: 2,665 LOC C++ ✅
- `lidar/`: 1,188 LOC C++ ✅
- **총 완료**: 3,853 LOC

### 예정
- `targeting/`: ~1,200 LOC C++ ⏳
- `throwing_mechanism/`: ~900 LOC C++ ⏳
- **예상 총합**: 5,953 LOC

---

## v4.0 핵심 변경사항

### 1. targeting 의미 재정의
- **이전**: targeting = 거리 측정
- **현재**: targeting = 핫스팟 트래킹 + 드론 제어

### 2. 폴더 구조 변경
- `targeting/lidar_integration` → `lidar/`
- `targeting/` (새로 정의: 핫스팟 트래킹 + 드론 제어)

### 3. 화재 진압 시나리오 반영
- Phase 1-2: 접근 및 대기
- **Phase 3: GCS 신호 → targeting 활성화** ⭐
- Phase 4: 발사

### 4. 영상 오버레이 통합
- 항상 표시: 거리 + 핫스팟
- GCS 신호 후: 트래킹 상태

---

## 다음 우선순위

### 즉시 (이번 주)
1. LD19 LiDAR 하드웨어 테스트
2. targeting/ 구현 시작 준비

### 1주일 내
1. **targeting/** 프로젝트 Phase 3.1 구현
   - drone_position_controller.cpp (드론 제어 기능 추가)
   - targeting_manager.cpp 확장

### 2주 내
1. **targeting/** 프로젝트 Phase 3.2 구현
   - PX4 연동
   - 미세 조정 로직

### 3주 내
1. **targeting/** 프로젝트 Phase 3.3 통합
   - GCS 신호 연동
   - 전체 시스템 테스트

**참고**: targeting/src/의 기본 구조는 이미 아키텍처 리팩토링에서 완료되었습니다.
프로젝트 Phase 3에서는 드론 제어 기능을 추가하여 확장합니다.

---

**작성자**: Claude Code Assistant  
**버전**: v4.0 (targeting 재정의)  
**작성일**: 2025-12-31  
**다음 리뷰**: targeting/ Phase 1 완료 시
