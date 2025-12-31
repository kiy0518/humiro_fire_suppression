# 프로젝트 폴더 재구성 계획

작성일: 2025-12-31

## 문제점

1. **thermal_tracking 중복**
   - `thermal/` 폴더에 이미 열화상 카메라 시스템 완성 (영상 전송 포함)
   - `targeting/thermal_tracking/` 폴더가 비어있고 목적이 불명확

2. **horizontal_alignment 모호함**
   - 짐벌 제어 기능이지만 단독 폴더로 존재
   - 통합 타겟팅 시스템의 일부가 되어야 함

## 재구성 계획

### 기존 구조
```
thermal/
├── src/              # C++ 열화상 시스템 (완성)
└── python/           # 프로토타입

targeting/
├── thermal_tracking/       # 비어있음 (중복)
├── horizontal_alignment/   # 비어있음 (모호함)
├── lidar_integration/      # 완성 (C++)
└── trajectory_calc/        # 비어있음
```

### 새로운 구조
```
thermal/
├── src/              # C++ 열화상 카메라 I/O, 스트리밍 (기존)
├── python/           # 프로토타입 (기존)
└── processing/       # 새로운: 영상 처리, 핫스팟 감지, 추적

targeting/
├── lidar_integration/      # LiDAR 거리 측정 (완성)
├── trajectory_calc/        # 탄도 계산 엔진
└── integrated_targeting/   # 통합 타겟팅 시스템
    ├── fire_tracking/      # 화재 추적 (thermal_tracking 이동)
    ├── gimbal_control/     # 짐벌 제어 (horizontal_alignment 이동)
    └── targeting_core/     # 타겟팅 코어 로직
```

## 폴더별 역할 재정의

### thermal/ (열화상 카메라 시스템)
**역할**: 하드웨어 I/O 및 기본 영상 처리

- `src/` - C++ 카메라 인터페이스, RTSP/HTTP 스트리밍
- `python/` - Python 프로토타입
- `processing/` - **새로운**: 핫스팟 감지, 온도 분석, 추적

**파일 예**:
```cpp
thermal/processing/
├── hotspot_detector.h/cpp     # 핫스팟 감지 알고리즘
├── thermal_analyzer.h/cpp     # 온도 분석
├── fire_tracker.h/cpp         # 화재 추적
└── CMakeLists.txt
```

### targeting/ (타겟팅 시스템)
**역할**: 센서 통합, 탄도 계산, 조준 제어

#### targeting/lidar_integration/ (완성)
- LiDAR 인터페이스 (C++)
- 거리 측정 및 오버레이

#### targeting/trajectory_calc/ (다음 작업)
- 탄도 계산 엔진 (C++)
- 발사각 계산

#### targeting/integrated_targeting/ (통합 시스템)
**역할**: 모든 센서와 제어를 통합

**하위 구조**:
```cpp
targeting/integrated_targeting/
├── fire_tracking/           # 화재 추적 (기존 thermal_tracking)
│   ├── track_manager.h/cpp  # 추적 관리자
│   └── target_filter.h/cpp  # 타겟 필터링
│
├── gimbal_control/          # 짐벌 제어 (기존 horizontal_alignment)
│   ├── gimbal_driver.h/cpp  # 짐벌 드라이버
│   └── stabilizer.h/cpp     # 안정화
│
├── targeting_core/          # 타겟팅 코어
│   ├── targeting_fsm.h/cpp  # 타겟팅 상태 머신
│   ├── sensor_fusion.h/cpp  # 센서 융합
│   └── aim_controller.h/cpp # 조준 제어
│
└── CMakeLists.txt
```

## 이동 계획

### 1단계: 폴더 이름 변경
```bash
# thermal_tracking → integrated_targeting/fire_tracking
mv targeting/thermal_tracking targeting/integrated_targeting
mkdir -p targeting/integrated_targeting/fire_tracking
mkdir -p targeting/integrated_targeting/gimbal_control
mkdir -p targeting/integrated_targeting/targeting_core

# horizontal_alignment → integrated_targeting/gimbal_control
# (내용물을 gimbal_control로 이동)
```

### 2단계: thermal 처리 기능 추가
```bash
mkdir -p thermal/processing
```

### 3단계: README 작성
각 폴더에 역할 설명 추가

## 데이터 흐름

```
[열화상 카메라]
    ↓
[thermal/src/]              # 카메라 I/O, 스트리밍
    ↓
[thermal/processing/]       # 핫스팟 감지, 추적
    ↓ (각도 정보)
[targeting/lidar_integration/]  # 거리 측정
    ↓ (각도 + 거리)
[targeting/trajectory_calc/]    # 발사각 계산
    ↓ (발사각)
[targeting/integrated_targeting/]
    ├─ [fire_tracking/]     # 타겟 추적 관리
    ├─ [gimbal_control/]    # 짐벌 제어
    └─ [targeting_core/]    # 조준 제어
    ↓
[throwing_mechanism/]       # 발사
```

## 장점

1. **명확한 책임 분리**
   - thermal: 카메라 하드웨어 및 기본 처리
   - targeting: 타겟팅 로직 및 제어

2. **중복 제거**
   - thermal과 thermal_tracking 혼동 해소
   - 통합 타겟팅으로 기능 집중

3. **확장성**
   - thermal/processing: 추가 영상 처리 알고리즘
   - targeting/integrated_targeting: 새로운 센서/제어 추가

4. **이해 용이성**
   - 폴더 이름이 기능을 명확히 표현
   - 데이터 흐름이 논리적

## 구현 순서

### 즉시 실행
1. ✅ 재구성 계획 수립
2. ⏳ 폴더 이동 및 재배치
3. ⏳ README.md 작성

### 단기 (1주일)
1. `thermal/processing/` 구조 설계
2. `targeting/integrated_targeting/` 구조 설계
3. CMakeLists.txt 설정

### 중기 (1개월)
1. `thermal/processing/hotspot_detector.cpp` 구현
2. `targeting/integrated_targeting/targeting_core/` 구현
3. 통합 테스트

---

**작성자**: Claude Code Assistant  
**작성일**: 2025-12-31
