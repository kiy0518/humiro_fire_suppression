# 상세 개발 계획서

**작성일**: 2025-01-XX  
**프로젝트**: Humiro Fire Suppression Drone System  
**버전**: v5.0

---

## 📋 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [현재 진행 상황](#현재-진행-상황)
3. [Phase별 상세 계획](#phase별-상세-계획)
4. [개발 로드맵](#개발-로드맵)
5. [기술적 세부사항](#기술적-세부사항)
6. [통합 계획](#통합-계획)
7. [테스트 계획](#테스트-계획)
8. [리스크 관리](#리스크-관리)

---

## 프로젝트 개요

### 목적
드론 기반 자동 화재 진압 시스템 개발

### 핵심 프로세스
```
1. 열화상으로 화재 감지 (핫스팟) ✅
2. LiDAR로 거리 측정 (10m 도착 판단) ✅
3. GCS 격발 신호 수신 ⏳
4. 핫스팟 트래킹 + 드론 정조준 ⏳
5. 소화탄 발사 ⏳
```

### 기술 스택
- **언어**: C++17 (우선), Python (ROS2 래퍼)
- **플랫폼**: Khadas VIM4 (Ubuntu 22.04 ARM64)
- **미들웨어**: ROS2 Humble
- **비행 제어**: PX4 Firmware v1.16.0
- **통신**: Micro-ROS Agent (XRCE-DDS)
- **이미지 처리**: OpenCV + GStreamer

---

## 현재 진행 상황

### 전체 진행률
```
프로젝트 Phase 1: Thermal System      [██████████] 100%  ✅
프로젝트 Phase 2: LiDAR System        [██████████] 100%  ✅ (HW 테스트 대기)
프로젝트 Phase 3: Targeting System    [██░░░░░░░░]  20%  ⏳ 부분 완료 (드론 제어 기능 추가 필요)
프로젝트 Phase 4: Throwing Mechanism  [░░░░░░░░░░]   0%  ⏳
프로젝트 Phase 5: Navigation          [░░░░░░░░░░]   0%  ⏳

전체: 44% 완료 (5단계 중 2단계 완료 + 1단계 부분 완료)
코드: 48% 완료 (3,853 / 8,000 LOC)

참고: 아키텍처 리팩토링 Phase 1-3는 모두 완료되었습니다.
```

### 완료된 모듈

#### 1. Thermal System (✅ 완료)
- **코드량**: ~2,665 LOC C++
- **기능**: 핫스팟 감지, RGB+Thermal 정합, RTSP/HTTP 스트리밍
- **상태**: 완전 구현 및 테스트 완료

#### 2. LiDAR System (✅ 완료, HW 테스트 대기)
- **코드량**: ~1,188 LOC C++
- **기능**: LD19 통신, 거리 측정, 오버레이
- **상태**: 소프트웨어 완료, 하드웨어 테스트 대기

### 미구현 모듈

#### 3. Targeting System (⏳ 설계 완료)
- **코드량**: 0 LOC (예상 ~1,200 LOC)
- **기능**: 핫스팟 트래킹, 드론 제어, 정조준
- **상태**: 설계 완료, 구현 대기

#### 4. Throwing Mechanism (⏳ 설계 완료)
- **코드량**: 0 LOC (예상 ~900 LOC)
- **기능**: 서보 제어, GPIO 트리거, 발사 제어
- **상태**: 설계 완료, 구현 대기

#### 5. Navigation (⏳ 미설계)
- **코드량**: 0 LOC (예상 ~2,000 LOC)
- **기능**: RTK GPS, 편대 비행, 충돌 회피
- **상태**: 폴더만 존재, 설계 미완료

---

## Phase별 상세 계획

### Phase 1: Thermal System ✅ (완료)

#### 완료된 작업
- [x] 카메라 자동 감지 (USB VID/PID)
- [x] 핫스팟 감지 알고리즘 (녹색 채널 최대값)
- [x] RGB+Thermal 프레임 정합
- [x] 멀티스레드 아키텍처
- [x] RTSP 스트리밍 (GStreamer, H.264)
- [x] HTTP 스트리밍 (MJPEG)
- [x] 성능 최적화 (30 FPS 달성)

#### 기술적 성과
- 30 FPS 실시간 처리
- <100ms 지연시간
- 스레드 안전 큐 구현
- 자동 카메라 감지

#### 남은 작업
- 없음

---

### Phase 2: LiDAR System ✅ (완료, HW 테스트 대기)

#### 완료된 작업
- [x] LD19 UART 통신 인터페이스
- [x] USB-UART / GPIO-UART 지원
- [x] 360° 스캔 데이터 파싱
- [x] 거리 오버레이 (색상 코딩)
- [x] 10m 도착 판단 로직

#### 남은 작업
- [ ] LD19 하드웨어 도착 및 연결
- [ ] USB-UART 실제 테스트
- [ ] GPIO-UART 연동 테스트
- [ ] 10m 거리 정확도 검증
- [ ] ROS2 토픽 통합 (선택사항)

#### 예상 소요 시간
- 하드웨어 도착 후: 1주

---

### 프로젝트 Phase 3: Targeting System ⏳ (다음 우선순위)

**참고**: 이것은 **프로젝트 기능 Phase 3**입니다. 아키텍처 리팩토링의 Phase 3 (ROS2 통신 강화)는 이미 완료되었습니다.

#### 목표
**핫스팟 트래킹 + 드론 위치 제어를 통한 정조준**

**참고**: `targeting/src/`는 이미 아키텍처 리팩토링에서 생성되었으며,
distance_overlay, aim_indicator, hotspot_tracker, targeting_frame_compositor가 구현되어 있습니다.
프로젝트 Phase 3에서는 이를 확장하여 드론 제어 기능(drone_position_controller)을 추가합니다.

#### 핵심 기능
1. **핫스팟 트래킹**
   - Kalman Filter로 위치 예측
   - 화면 중심으로부터의 오차 계산
   - 추적 상태 관리

2. **드론 위치 제어**
   - PX4 Offboard 모드
   - 상하좌우 미세 조정
   - 정조준 유지

3. **GCS 신호 처리**
   - ROS2 토픽 구독
   - 활성화/비활성화 제어
   - LOCKED 상태 판정

#### 개발 단계

##### 프로젝트 Phase 3.1: 드론 제어 기능 추가 (1주)
**목표**: 드론 위치 제어 기능 구현

**참고**: `hotspot_tracker`, `targeting_overlay`의 기본 구조는 이미 아키텍처 리팩토링에서 구현되었습니다.
프로젝트 Phase 3.1에서는 드론 제어 기능을 추가합니다.

**작업 항목**:
- [ ] `drone_position_controller.h/cpp` 구현 (신규)
  - ROS2 노드 초기화
  - PX4 토픽 구독/발행
  - Offboard 모드 활성화
  - 위치 명령 전송
- [ ] `targeting_manager.h/cpp` 확장
  - 드론 제어 기능 통합
  - 상태 머신 확장
  - thermal/src 데이터 수신 (이미 구현됨)

**예상 코드량**: ~500 LOC (신규)

**테스트**:
- 단위 테스트: 드론 제어 로직
- 통합 테스트: PX4 연동
- 시뮬레이션 테스트: Gazebo/PX4 SITL

##### 프로젝트 Phase 3.2: PX4 연동 및 미세 조정 (1주)
**목표**: PX4를 통한 드론 위치 제어 완성

**작업 항목**:
- [ ] PID 제어 로직 추가
  - X축 PID 컨트롤러
  - Y축 PID 컨트롤러
  - 파라미터 튜닝
- [ ] 미세 조정 로직 완성
  - 픽셀 오차 → 미터 변환
  - 속도 제한
  - 안전 체크
- [ ] 드론 제어 기능 통합 및 테스트

**예상 코드량**: ~300 LOC

**테스트**:
- 시뮬레이션 테스트: Gazebo/PX4 SITL
- 실제 드론 테스트: 작은 이동량부터
- 안전 테스트: 비상 정지, 범위 제한

##### 프로젝트 Phase 3.3: GCS 통합 (1주)
**목표**: 전체 시스템 통합 및 최적화

**작업 항목**:
- [ ] ROS2 토픽 통합
  - `/gcs/fire_command` 구독
  - `/targeting/status` 발행
  - `/thermal/hotspot` 구독
- [ ] 활성화/비활성화 제어
  - GCS 신호 처리
  - 상태 전환 로직
  - 안전 체크
- [ ] 전체 시스템 통합
  - thermal + lidar + targeting 통합
  - 오버레이 통합
  - 성능 최적화

**예상 코드량**: ~300 LOC

**테스트**:
- 통합 테스트: 전체 시나리오
- 성능 테스트: 지연시간, CPU 사용률
- 안전 테스트: 모든 안전 로직 검증

#### 전체 예상 소요 시간
- **3주** (프로젝트 Phase 3.1 + 3.2 + 3.3)

#### 예상 코드량
- **~800 LOC C++** (신규, 드론 제어 기능)
- **참고**: targeting/src/의 기본 구조(~400 LOC)는 이미 아키텍처 리팩토링에서 완료됨

---

### Phase 4: Throwing Mechanism ⏳

#### 목표
**10m 고정 거리에서 고정 각도로 소화탄 발사**

#### 핵심 기능
1. **서보 제어**
   - 발사 각도 설정
   - PWM 제어
   - 초기 위치 복귀

2. **GPIO 트리거**
   - 발사 신호
   - 안전 체크
   - 비상 정지

#### 개발 단계

##### Phase 4.1: 하드웨어 준비 (1주)
- [ ] 서보 모터 선정 및 주문
- [ ] GPIO 핀 매핑 확정
- [ ] 수동 각도 테스트 (물리적)
- [ ] 최적 각도 결정

##### Phase 4.2: 소프트웨어 구현 (1주)
- [ ] `servo_controller.h/cpp` 구현
- [ ] `fire_trigger.h/cpp` 구현
- [ ] 안전 로직 구현

##### Phase 4.3: 통합 및 테스트 (1주)
- [ ] `throwing_controller.h/cpp` 구현
- [ ] 전체 시스템 통합
- [ ] 실제 발사 테스트

#### 전체 예상 소요 시간
- **3주** (하드웨어 준비 포함)

#### 예상 코드량
- **~900 LOC C++**

---

### Phase 5: Navigation ⏳

#### 목표
**RTK GPS 기반 정밀 위치 및 편대 비행**

#### 핵심 기능
1. **RTK GPS 정밀 위치**
2. **편대 비행**
3. **충돌 회피**

#### 개발 단계
- 설계 미완료 (추후 계획)

#### 예상 소요 시간
- **4-6주** (설계 후 결정)

#### 예상 코드량
- **~2,000 LOC** (Python/C++)

---

## 개발 로드맵

### 2025년 1월 (현재)
- [x] Phase 1: Thermal System 완료
- [x] Phase 2: LiDAR System 완료
- [ ] Phase 2: LiDAR 하드웨어 테스트
- [ ] Phase 3.1: Targeting 기본 구조 시작

### 2025년 2월
- [ ] Phase 3.1: Targeting 기본 구조 완료
- [ ] Phase 3.2: 드론 제어 연동 완료
- [ ] Phase 3.3: GCS 통합 완료
- [ ] Phase 4.1: Throwing 하드웨어 준비

### 2025년 3월
- [ ] 프로젝트 Phase 4.2: Throwing 소프트웨어 구현
- [ ] 프로젝트 Phase 4.3: Throwing 통합 및 테스트
- [ ] 전체 시스템 통합 테스트

### 2025년 4월 이후
- [ ] 프로젝트 Phase 5: Navigation 설계 및 구현
- [ ] 최종 최적화 및 문서화

---

## 기술적 세부사항

### Targeting System 상세 설계

#### 1. Hotspot Tracker
```cpp
class HotspotTracker {
public:
    HotspotTracker();
    
    // 핫스팟 위치 업데이트
    void updateHotspot(int x, int y, double timestamp);
    
    // 추적 상태
    bool isTracking() const;
    
    // 예측 위치 (Kalman Filter)
    cv::Point getPredictedPosition();
    
    // 화면 중심으로부터의 오차
    cv::Point getOffsetFromCenter(int frame_width, int frame_height);
    
private:
    cv::KalmanFilter kalman_;
    std::deque<cv::Point> history_;
    bool is_tracking_;
    double last_update_time_;
    
    // Kalman Filter 파라미터
    static constexpr double PROCESS_NOISE = 0.03;
    static constexpr double MEASUREMENT_NOISE = 0.3;
};
```

#### 2. Drone Position Controller
```cpp
class DronePositionController {
public:
    DronePositionController();
    
    // PX4 연결
    bool connect();
    
    // 상하좌우 미세 조정
    bool adjustPosition(float dx, float dy);  // 픽셀 오차 → 드론 이동
    
    // 위치 유지
    bool holdPosition();
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    
    // PID 컨트롤러
    PIDController pid_x_;
    PIDController pid_y_;
    
    // 제어 파라미터
    static constexpr float MAX_VELOCITY = 0.5f;  // m/s
    static constexpr float PIXEL_TO_METER = 0.01f;  // 픽셀당 미터
};
```

#### 3. Targeting Manager
```cpp
class TargetingManager {
public:
    TargetingManager();
    
    // GCS 격발 신호 수신
    void onFireCommand(bool enable);
    
    // 메인 루프
    void update();
    
    // 상태 확인
    bool isActive() const;
    bool isLocked() const;  // 정조준 완료
    
private:
    HotspotTracker tracker_;
    DronePositionController drone_controller_;
    
    bool targeting_active_;  // GCS 신호로 활성화
    bool locked_;            // 정조준 완료
    
    // 허용 오차 (픽셀)
    static constexpr int LOCK_THRESHOLD = 10;
    
    // 안전 체크
    bool isSafeToActivate() const;
    void emergencyStop();
};
```

### 데이터 흐름

```
[GCS]
    ↓
격발 신호 (ROS2 토픽: /gcs/fire_command)
    ↓
[TargetingManager]
    ↓ 활성화
    ├─→ [HotspotTracker]
    │       ↓ 핫스팟 위치 받음
    │   [thermal/src/] → /thermal/hotspot
    │       ↓ 오차 계산
    │   (중심 - 핫스팟 위치)
    │
    └─→ [DronePositionController]
            ↓ PX4 명령
        상하좌우 미세 조정
            ↓
        정조준 유지
            ↓
        LOCKED 상태
```

---

## 통합 계획

### 현재 통합 상태
- ✅ Thermal System (독립 실행)
- ✅ LiDAR System (독립 실행)

### 단계별 통합 계획

#### Step 1: Thermal + LiDAR 통합 (1주)
- [ ] 오버레이 통합
  - 거리 오버레이 (lidar)
  - 핫스팟 표시 (thermal)
  - 동시 렌더링
- [ ] 데이터 동기화
  - 프레임 동기화
  - 타임스탬프 맞춤

#### Step 2: Thermal + LiDAR + Targeting 통합 (1주)
- [ ] targeting 오버레이 추가
  - 십자선
  - 트래킹 상태
  - LOCKED 표시
- [ ] 데이터 흐름 통합
  - thermal → targeting
  - lidar → targeting (거리 체크)

#### Step 3: 전체 시스템 통합 (1주)
- [ ] throwing_mechanism 통합
- [ ] 전체 시나리오 테스트
- [ ] 성능 최적화

---

## 테스트 계획

### 단위 테스트
- [ ] HotspotTracker: Kalman Filter 정확도
- [ ] DronePositionController: PID 제어
- [ ] TargetingManager: 상태 머신

### 통합 테스트
- [ ] Thermal + LiDAR: 오버레이 통합
- [ ] Thermal + Targeting: 핫스팟 추적
- [ ] Targeting + PX4: 드론 제어

### 시스템 테스트
- [ ] 전체 시나리오: 화재 감지 → 거리 측정 → 트래킹 → 발사
- [ ] 다중 드론: 편대 비행
- [ ] 안전 테스트: 비상 정지, 범위 이탈

### 하드웨어 테스트
- [ ] LD19 LiDAR: 거리 정확도
- [ ] 서보 모터: 각도 정확도
- [ ] GPIO 트리거: 발사 신호

---

## 리스크 관리

### 기술적 리스크

#### 1. PX4 Offboard 모드 연동 복잡도
- **확률**: 중간
- **영향**: 높음
- **대응**: 
  - 단계적 접근 (시뮬레이션 → 실제)
  - PX4 문서 및 예제 참고
  - 충분한 테스트 시간 확보

#### 2. Kalman Filter 파라미터 튜닝
- **확률**: 높음
- **영향**: 중간
- **대응**:
  - 시뮬레이션으로 초기 파라미터 결정
  - 실제 테스트로 미세 조정
  - 자동 튜닝 알고리즘 고려

#### 3. 하드웨어 지연
- **확률**: 중간
- **영향**: 중간
- **대응**:
  - 하드웨어 주문 일정 관리
  - 대체 하드웨어 검토
  - 소프트웨어 우선 개발

### 일정 리스크

#### 1. 개발 지연
- **확률**: 중간
- **영향**: 중간
- **대응**:
  - 버퍼 시간 확보 (20%)
  - 우선순위 명확화
  - 단계별 마일스톤 설정

#### 2. 하드웨어 테스트 지연
- **확률**: 높음
- **영향**: 낮음
- **대응**:
  - 소프트웨어 우선 개발
  - 시뮬레이션 테스트 활용

---

## 다음 단계

### 즉시 (이번 주)
1. **LD19 LiDAR 하드웨어 테스트**
   - 하드웨어 도착 확인
   - USB-UART 연결 테스트
   - 10m 거리 정확도 검증

2. **targeting/ Phase 3.1 시작**
   - 프로젝트 구조 생성
   - `hotspot_tracker.cpp` 구현 시작
   - `targeting_overlay.cpp` 구현 시작

### 1주일 내
1. **targeting/ Phase 3.1 완료**
   - 기본 구조 완성
   - 단위 테스트 완료
   - thermal/src와 연동 테스트

### 2주 내
1. **targeting/ Phase 3.2 시작**
   - PX4 연동 준비
   - 시뮬레이션 환경 구축
   - 드론 제어 로직 구현

---

**작성자**: Claude Code Assistant  
**버전**: v5.0  
**마지막 업데이트**: 2025-01-XX  
**다음 리뷰**: targeting/ Phase 3.1 완료 시

