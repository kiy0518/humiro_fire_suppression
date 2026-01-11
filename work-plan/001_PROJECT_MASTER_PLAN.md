# Humiro Fire Suppression - 프로젝트 마스터 플랜 v5.0

작성일: 2025-12-31
**최종 수정일**: 2026-01-10
**상태**: 편대 비행 아키텍처 추가 (v5.0)

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

## 시스템 재정의 (v4.1 핵심 변경)

### targeting의 의미 변경

**이전 (v3.0 - 잘못된 이해)**:
- targeting = 거리 측정 + 탄도 계산

**현재 (v4.1 - 올바른 이해)**:
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
- **RGB 카메라 화재 확인 후 목표점까지 자동 이동**
  - 화재 감지된 위치로 자율 비행 시작
  - GPS 좌표 또는 상대 위치 기반 경로 계획
  - 장애물 회피 및 안전 경로 탐색
- 열원까지 10m 지점으로 자율 비행
- **lidar/** 로 거리 모니터링
- 10m 도착 확인

### Phase 2: 대기
- 10m 지점 호버링
- **GCS 격발 신호 대기**

### 프로젝트 Phase 3: Targeting 활성화 ⭐ (핵심)
- **GCS 격발 신호 수신** (ROS2 토픽)
- **targeting/** 모듈 활성화 (드론 제어 기능 포함)
- **10m 지점부터 열화상으로 열 트래킹 시작**
  - 열화상 카메라로 핫스팟 감지 및 추적
  - Kalman Filter 기반 예측 추적
  - 열원 중심으로 자동 조준
- **핫스팟 트래킹 시작** (thermal/src 데이터 사용, 이미 구현됨)
- **드론 상하좌우 미세 조정** (PX4 제어, 프로젝트 Phase 3에서 추가)
  - 열 트래킹 기반 위치 보정
  - 화면 중심에 핫스팟 유지
- **정조준 유지** (화면 중심에 핫스팟)
- **LOCKED 판정** (오차 < 임계값)

### Phase 4: 발사
- 정조준 완료 (LOCKED) 확인
- **throwing_mechanism/** 발사 실행

### Phase 5: 인공지능 기반 화재 감지 (추후 구현)
- **AI 기반 RGB 카메라 화재 감지** (딥러닝 모델 활용)
  - **딥러닝 화재 감지 모델** (CNN/YOLO 기반)
    - 화염 감지 (실시간 객체 탐지)
    - 연기 감지 (시계열 분석 및 패턴 인식)
    - 화재 확률 계산 및 신뢰도 평가
  - **모델 학습 및 최적화**
    - 화재 데이터셋 수집 및 라벨링
    - 전이 학습 (Transfer Learning) 활용
    - 엣지 디바이스 최적화 (모델 경량화, 양자화)
  - **실시간 추론 파이프라인**
    - RGB 영상 프레임별 화재 감지
    - 화재 위치 추정 및 바운딩 박스
    - GPS 좌표 변환 및 목표점 설정
  - **LTE 통신 이중화** (통신 안정성 강화)
    - 기존 WiFi/무선 통신과 병행
    - 통신 장애 시 자동 전환 (failover)
    - 실시간 영상 전송 및 제어 명령 수신
    - 통신 상태 모니터링 및 자동 복구
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
│  RGB Camera + AI      │  ⏳ AI 화재 감지 (Phase 5)
│  ├─ 딥러닝 모델       │
│  ├─ 화염/연기 탐지   │
│  ├─ 화재 확률 계산   │
│  └─ 위치 추정        │
└──────────────────────┘
          ↓
    화재 위치 확인
          ↓
┌──────────────────────┐
│  LTE 통신 이중화      │  ⏳ 통신 이중화 (추후)
│  ├─ WiFi/무선 통신   │
│  ├─ LTE 통신         │
│  └─ 자동 전환        │
└──────────────────────┘
          ↓
┌──────────────────────┐
│  자율 비행 제어      │  ⏳ 목표점 자동 이동 (추후)
│  ├─ 경로 계획        │
│  ├─ 장애물 회피      │
│  └─ GPS/상대 위치    │
└──────────────────────┘
          ↓
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
│  targeting/          │  ⏳ 열 트래킹 + 드론 제어
│  ├─ 열화상 트래킹    │  (10m 지점부터)
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
- [ ] **열화상 기반 열 트래킹** (10m 지점부터 활성화)

### ⏳ Phase 4: Throwing (추후)
- [ ] servo_controller.cpp
- [ ] fire_trigger.cpp

### ⏳ Phase 5: 인공지능 기반 화재 감지 (추후)
- [ ] **AI 기반 화재 감지 모듈**
  - [ ] 딥러닝 화재 감지 모델 개발 (CNN/YOLO)
    - [ ] 화염 감지 모델 학습 및 최적화
    - [ ] 연기 감지 모델 학습 및 최적화
    - [ ] 화재 데이터셋 수집 및 라벨링
    - [ ] 전이 학습 (Transfer Learning) 적용
    - [ ] 엣지 디바이스 최적화 (모델 경량화, 양자화)
  - [ ] 실시간 추론 파이프라인
    - [ ] RGB 영상 프레임별 화재 감지
    - [ ] 화재 위치 추정 및 바운딩 박스
    - [ ] 화재 확률 계산 및 신뢰도 평가
    - [ ] GPS 좌표 변환 및 목표점 설정
  - [ ] 모델 배포 및 관리
    - [ ] ONNX/TensorRT 변환 및 최적화
    - [ ] 모델 버전 관리 및 업데이트
    - [ ] 추론 성능 모니터링
- [ ] **LTE 통신 이중화 모듈**
  - [ ] LTE 모뎀 초기화 및 연결 관리
  - [ ] WiFi/무선 통신과 병행 통신
  - [ ] 통신 상태 모니터링 및 자동 전환 (failover)
  - [ ] 실시간 영상 전송 (LTE 경로)
  - [ ] 제어 명령 수신 및 우선순위 처리
  - [ ] 통신 장애 복구 메커니즘
- [ ] **자율 비행 경로 계획** (AI 화재 감지 연동)
  - [ ] AI 감지 결과 기반 목표점 설정
  - [ ] GPS 좌표 또는 상대 위치 기반 경로 계획
  - [ ] 장애물 회피 알고리즘
  - [ ] 안전 경로 탐색 및 최적화

### ⏳ Phase 6: 편대 비행 제어 (신규 - v5.0) ⭐

**위치**: `navigation/formation_control/`, `communication/`
**코드**: 0 LOC (설계 단계)
**상태**: ⏳ 구현 대기

**시스템 구성**:
- **1호기**: 리더 (Leader) - 경로 계획 및 팔로워 조율
- **2호기**: 팔로워 (Follower) - 리더 추적, 상대 위치 유지
- **3호기**: 팔로워 (Follower) - 리더 추적, 상대 위치 유지

**아키텍처 결정** ⭐:
- **단일 프로젝트** 사용 (별도 프로젝트 분리 안 함)
- 기체별 역할은 `config/device_config.env`로 구분
- 런타임에 역할에 따라 Leader/Follower 객체 생성
- 공통 코드 재사용 (OffboardManager, 센서 통합)

**Phase 6.1: 기체 간 통신 모듈** (1-2주)
```
communication/                    🆕 새로 생성 필요
├── src/
│   ├── inter_drone_comm.cpp      🆕 UDP/TCP 기체 간 통신
│   ├── inter_drone_comm.h
│   ├── formation_message.cpp     🆕 편대 메시지 프로토콜
│   ├── formation_message.h
│   └── mesh_network.cpp          🆕 메쉬 네트워크 (선택)
├── test/
│   └── test_inter_drone_comm.cpp
└── CMakeLists.txt
```

**기능**:
- [ ] 기체 간 위치 정보 공유 (100ms 주기)
- [ ] 리더의 명령 전달 (waypoint, formation shape)
- [ ] 팔로워의 상태 보고 (위치, 배터리, 센서)
- [ ] 네트워크 상태 모니터링
- [ ] UDP 브로드캐스트 / TCP 직접 통신

**메시지 프로토콜**:
```cpp
// formation_message.h
struct FormationPositionMsg {
    uint8_t drone_id;
    float latitude;
    float longitude;
    float altitude;
    float heading;
    uint64_t timestamp;
};

struct FormationCommandMsg {
    uint8_t command_type;  // HOLD, MOVE, RTL
    float param1, param2, param3;
    uint64_t timestamp;
};

struct FormationStatusMsg {
    uint8_t drone_id;
    uint8_t status;        // OK, WARNING, ERROR
    float battery_percent;
    float distance_to_leader;
    uint64_t timestamp;
};
```

**Phase 6.2: 편대 제어 로직** (2-3주)
```
navigation/formation_control/     ⚠️ 기존 빈 폴더 활용
├── src/
│   ├── formation_leader.cpp      🆕 리더 제어기
│   ├── formation_leader.h
│   ├── formation_follower.cpp    🆕 팔로워 제어기
│   ├── formation_follower.h
│   ├── formation_geometry.cpp    🆕 대형 계산
│   ├── formation_geometry.h
│   └── formation_state.cpp       🆕 편대 상태 관리
├── test/
│   ├── test_formation_leader.cpp
│   └── test_formation_follower.cpp
└── CMakeLists.txt
```

**FormationLeader 기능**:
- [ ] 미션 경로 계획 (자율 비행)
- [ ] 편대 대형 선택 (triangle, line, V-shape)
- [ ] 팔로워 위치 계산 및 명령 전송
- [ ] 팔로워 상태 모니터링
- [ ] 비상 상황 처리 (팔로워 이탈 시)

**FormationFollower 기능**:
- [ ] 리더 위치 추적
- [ ] 상대 위치 계산 (리더 기준)
- [ ] 목표 위치 유지 (오차 ±2m)
- [ ] 충돌 회피 (다른 팔로워와)
- [ ] 리더 손실 시 자동 RTL

**FormationGeometry**:
- [ ] 삼각형 대형 (기본)
  - 리더: 선두
  - 2호기: 좌측 10m 후방
  - 3호기: 우측 10m 후방
- [ ] 라인 대형 (옵션)
  - 일렬 종대 배치
- [ ] V-shape 대형 (옵션)
  - V자 편대

**Phase 6.3: 설정 및 통합** (1주)
```
config/
├── device_config.env              ✏️ 수정 필요
│   ├── FORMATION_ROLE=leader|follower
│   ├── FORMATION_POSITION=center|left|right
│   └── LEADER_IP=192.168.100.11
│
├── formation_config.yaml          🆕 새 파일
│   ├── formation_type: triangle
│   ├── spacing: 10.0m
│   ├── altitude_offset: 0.5m
│   └── safety_distance: 5.0m
│
└── communication_config.yaml      🆕 새 파일
    ├── protocol: udp
    ├── port: 14560
    ├── broadcast_interval: 100ms
    └── timeout: 1000ms
```

**application_manager.cpp 수정**:
```cpp
void ApplicationManager::initializeFormation() {
    std::string role = std::getenv("FORMATION_ROLE") ?: "standalone";

    if (role == "leader") {
        formation_controller_ = new FormationLeader(ros2_node_);
        std::cout << "초기화: 리더 모드" << std::endl;
    } else if (role == "follower") {
        formation_controller_ = new FormationFollower(ros2_node_);
        std::cout << "초기화: 팔로워 모드" << std::endl;
    } else {
        std::cout << "초기화: 단독 비행 모드" << std::endl;
    }
}
```

**Phase 6.4: 테스트 및 검증** (1-2주)
- [ ] 지상 시뮬레이션 (3대 VIM4)
- [ ] 단거리 실내 테스트 (optical flow)
- [ ] 실외 GPS 편대 비행
- [ ] 리더 이탈 시나리오
- [ ] 팔로워 이탈 시나리오
- [ ] 통신 장애 복구

**안전 기능**:
- [ ] 최소 안전 거리 유지 (5m)
- [ ] 리더 손실 시 자동 RTL
- [ ] 배터리 부족 시 개별 RTL
- [ ] 통신 끊김 시 호버링 → RTL
- [ ] GPS 손실 시 현재 위치 유지

**개발 우선순위**:
1. **Phase 6.1**: 기체 간 통신 (가장 중요)
2. **Phase 6.2.1**: 기본 리더 기능
3. **Phase 6.2.2**: 기본 팔로워 기능
4. **Phase 6.2.3**: 삼각형 대형
5. **Phase 6.3**: 설정 통합
6. **Phase 6.4**: 테스트

---

## 진행 상황 요약

```
프로젝트 Phase 1: 열화상       [██████████] 100%  ✅ 완료
프로젝트 Phase 2: LiDAR        [██████████] 100%  ✅ 완료 (HW 대기)
프로젝트 Phase 3: Targeting    [██░░░░░░░░]  20%  ⏳ 부분 완료 (드론 제어 기능 추가 필요)
프로젝트 Phase 4: Throwing     [░░░░░░░░░░]   0%  ⏳ 추후
프로젝트 Phase 5: AI 화재 감지 [░░░░░░░░░░]   0%  ⏳ 추후 구현
프로젝트 Phase 6: 편대 비행    [░░░░░░░░░░]   0%  ⏳ 신규 추가 (v5.0)

단독 비행: 44% 완료 (5단계 중 2단계 완료 + 1단계 부분 완료)
편대 비행: 0% 완료 (설계 단계)

참고: 아키텍처 리팩토링 Phase 1-3는 모두 완료되었습니다.
      실내 테스트 모드 (testExeMission) 구현 완료 (v0.8.0)
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
- `fire_detection/` (AI 기반 화재 감지): ~2,000 LOC C++/Python ⏳
  - 딥러닝 모델 학습 및 추론 파이프라인 포함
- `lte_communication/` (LTE 통신 이중화): ~800 LOC C++ ⏳
- `autonomous_navigation/` (자율 비행 경로 계획): ~1,200 LOC C++ ⏳
- `communication/` (기체 간 통신): ~1,500 LOC C++ 🆕
- `navigation/formation_control/` (편대 제어): ~2,500 LOC C++ 🆕
- **예상 총합**: ~14,553 LOC

---

## v5.0 핵심 변경사항 (2026-01-10)

### 1. 편대 비행 시스템 추가 ⭐ (Phase 6)
- **아키텍처**: 단일 프로젝트, 설정 기반 역할 구분
- **통신 모듈**: `communication/` 디렉토리 신규 생성
- **편대 제어**: `navigation/formation_control/` 구현
- **시스템 구성**: 1호기(리더) + 2,3호기(팔로워)
- **대형**: 삼각형, 라인, V-shape

### 2. 실내 테스트 모드 구현 (v0.8.0)
- `testExeMission()` 함수 추가
- QGC FIRE_MISSION_START 메시지 수신 시 자동 실행
- 시퀀스: OFFBOARD → ARM → 5초 대기 → TAKEOFF(1m) → 5초 호버링 → LAND → 3초 대기 → DISARM
- GPS 불필요 (optical flow 사용)
- 2호기 설정 완료 및 백업

### 3. 프로젝트 구조 확장
```
기존:
├── navigation/src/offboard/autonomous/  ✅ 자율 비행 완료
└── (formation_control/, collision_avoidance/ 빈 폴더)

신규:
├── communication/                        🆕 기체 간 통신
└── navigation/formation_control/        🆕 편대 제어 구현
```

---

## v4.1 핵심 변경사항 (2026-01-04)

### 1. targeting 의미 재정의 (v4.0에서 정의)
- **이전**: targeting = 거리 측정
- **현재**: targeting = 핫스팟 트래킹 + 드론 제어

### 2. 폴더 구조 변경 (v4.0에서 정의)
- `targeting/lidar_integration` → `lidar/`
- `targeting/` (새로 정의: 핫스팟 트래킹 + 드론 제어)

### 3. 화재 진압 시나리오 반영 (v4.0에서 정의)
- Phase 1-2: 접근 및 대기
- **Phase 3: GCS 신호 → targeting 활성화** ⭐
- Phase 4: 발사

### 4. 영상 오버레이 통합 (v4.0에서 정의)
- 항상 표시: 거리 + 핫스팟
- GCS 신호 후: 트래킹 상태

### 5. Phase 5 추가: 인공지능 기반 화재 감지 (v4.1 신규) ⭐
- **AI 기반 RGB 카메라 화재 감지** 추가
  - 딥러닝 화재 감지 모델 (CNN/YOLO 기반)
  - 화염 및 연기 실시간 객체 탐지
  - 화재 확률 계산 및 신뢰도 평가
  - 화재 위치 추정 및 GPS 좌표 변환
  - 모델 경량화 및 엣지 디바이스 최적화
- **LTE 통신 이중화** 추가
  - 기존 WiFi/무선 통신과 병행
  - 통신 장애 시 자동 전환 (failover)
  - 실시간 영상 전송 및 제어 명령 수신
- **자율 비행 경로 계획** 추가 (AI 화재 감지 연동)
  - AI 감지 결과 기반 목표점 자동 설정
  - GPS 좌표 또는 상대 위치 기반 경로 계획
  - 장애물 회피 및 안전 경로 탐색
- **열화상 열 트래킹 강화** (Phase 3)
  - 10m 지점부터 열화상으로 열 트래킹 시작
  - Kalman Filter 기반 예측 추적
  - 열원 중심으로 자동 조준

---

## 다음 우선순위

### 즉시 (이번 주)
1. LD19 LiDAR 하드웨어 테스트
2. targeting/ 구현 시작 준비

### 1주일 내
1. **targeting/** 프로젝트 Phase 3.1 구현
   - drone_position_controller.cpp (드론 제어 기능 추가)
   - targeting_manager.cpp 확장
   - 열화상 기반 열 트래킹 (10m 지점부터)

### 2주 내
1. **targeting/** 프로젝트 Phase 3.2 구현
   - PX4 연동
   - 미세 조정 로직
   - 열 트래킹 정조준 시스템

### 3주 내
1. **targeting/** 프로젝트 Phase 3.3 통합
   - GCS 신호 연동
   - 전체 시스템 테스트

### 추후 (장기 계획)
1. **Phase 5: 인공지능 기반 화재 감지**
   - AI 기반 화재 감지 모듈 개발 (딥러닝 모델)
   - 화재 데이터셋 수집 및 모델 학습
   - 실시간 추론 파이프라인 구축
   - 모델 경량화 및 엣지 디바이스 최적화
   - LTE 통신 이중화 시스템 구축
   - AI 감지 결과 기반 자율 비행 경로 계획

**참고**: targeting/src/의 기본 구조는 이미 아키텍처 리팩토링에서 완료되었습니다.
프로젝트 Phase 3에서는 드론 제어 기능을 추가하여 확장합니다.

---

**작성자**: Claude Code Assistant
**버전**: v5.0 (편대 비행 아키텍처 추가)
**작성일**: 2025-12-31
**최종 수정일**: 2026-01-11  
**다음 리뷰**: targeting/ Phase 1 완료 시
