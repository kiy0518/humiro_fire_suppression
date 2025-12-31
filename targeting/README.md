# Targeting System - 표적 추적 및 정조준 시스템

핫스팟 트래킹 + 드론 위치 제어를 통한 정조준

## 개요

**역할**: GCS 격발 신호 수신 후, 핫스팟을 추적하며 드론을 상하좌우로 제어하여 정조준 유지

**현재 상태**: 미구현

---

## 시스템 정의

### targeting이란?
- **핫스팟 감지**: thermal/src에서 제공 (이미 완료)
- **핫스팟 트래킹**: 핫스팟 위치 변화 추적 (Kalman Filter)
- **드론 위치 제어**: PX4로 상하좌우 미세 조정 명령
- **정조준 유지**: 화면 중심에 핫스팟 유지

### targeting이 아닌 것
- ❌ 거리 측정 → `lidar/` 폴더 담당
- ❌ 핫스팟 감지 → `thermal/src/` 이미 구현
- ❌ 발사 → `throwing_mechanism/` 담당

---

## 화재 진압 시나리오

### Phase 1: 접근 (자율 비행)
- 열원으로부터 10m 지점까지 자율 비행
- `lidar/`로 거리 확인

### Phase 2: 대기
- 10m 지점 도착
- GCS 격발 신호 대기

### Phase 3: Targeting 활성화 ⭐ (이 모듈의 역할)
- **GCS 격발 신호 수신**
- **핫스팟 트래킹 시작**
- **드론 상하좌우 제어로 정조준**
- 화면 중심에 핫스팟 유지

### Phase 4: 발사
- `throwing_mechanism/` 발사 실행

---

## 구현 계획

### 기본 기능 (우선 구현)

#### 1. Hotspot Tracker
**파일**: `hotspot_tracker.h/cpp`

```cpp
class HotspotTracker {
public:
    HotspotTracker();
    
    // 핫스팟 위치 업데이트 (thermal/src로부터)
    void updateHotspot(int x, int y, double timestamp);
    
    // 추적 상태
    bool isTracking() const;
    
    // 예측 위치 (칼만 필터)
    cv::Point getPredictedPosition();
    
    // 화면 중심으로부터의 오차
    cv::Point getOffsetFromCenter(int frame_width, int frame_height);
    
private:
    cv::KalmanFilter kalman_;
    std::deque<cv::Point> history_;
    bool is_tracking_;
    double last_update_time_;
};
```

#### 2. Drone Position Controller
**파일**: `drone_position_controller.h/cpp`

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
    // PX4 통신
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
};
```

#### 3. Targeting Manager
**파일**: `targeting_manager.h/cpp`

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
};
```

---

## 데이터 흐름

```
[GCS]
    ↓
격발 신호 (ROS2 토픽)
    ↓
[TargetingManager]
    ↓ 활성화
    ├─→ [HotspotTracker]
    │       ↓ 핫스팟 위치 받음
    │   [thermal/src/]
    │       ↓ 오차 계산
    │   (중심 - 핫스팟 위치)
    │
    └─→ [DronePositionController]
            ↓ PX4 명령
        상하좌우 미세 조정
            ↓
        정조준 유지
```

---

## 영상 오버레이 통합

### 화면 표시 (항상)
1. **거리 오버레이** (`lidar/`)
   - 색상 코딩 (녹색: 9-11m, 빨간색: <9m, 파란색: >11m)
   - LINE SHAPE 표시

2. **핫스팟 감지** (`thermal/src/`)
   - 핫스팟 위치 표시
   - 온도 정보

3. **트래킹 상태** (이 모듈)
   - GCS 신호 후에만 표시
   - "TRACKING ACTIVE" 텍스트
   - 정조준 완료 시 "LOCKED" 표시
   - 십자선 (화면 중심)

### 오버레이 구현
**파일**: `targeting_overlay.h/cpp`

```cpp
class TargetingOverlay {
public:
    TargetingOverlay();
    
    // 오버레이 그리기
    void draw(cv::Mat& frame, 
              bool tracking_active, 
              bool locked,
              cv::Point hotspot,
              cv::Point offset);
    
private:
    // 십자선 그리기
    void drawCrosshair(cv::Mat& frame);
    
    // 상태 텍스트
    void drawStatus(cv::Mat& frame, bool tracking, bool locked);
    
    // 오차 벡터
    void drawOffset(cv::Mat& frame, cv::Point center, cv::Point hotspot);
};
```

---

## 구현 단계

### Phase 1: 기본 구조 (1주)
- [ ] `hotspot_tracker.cpp` - 핫스팟 위치 추적
- [ ] `targeting_overlay.cpp` - 화면 오버레이
- [ ] `targeting_manager.cpp` - 기본 상태 관리

### Phase 2: 드론 제어 연동 (1주)
- [ ] `drone_position_controller.cpp` - PX4 연동
- [ ] 상하좌우 미세 조정 테스트
- [ ] 정조준 로직 구현

### Phase 3: GCS 통합 (1주)
- [ ] GCS 격발 신호 ROS2 토픽 구독
- [ ] 활성화/비활성화 제어
- [ ] 전체 시스템 테스트

---

## ROS2 통신

### 구독 토픽
```cpp
// GCS 격발 신호
/gcs/fire_command (std_msgs/msg/Bool)

// thermal/src에서 핫스팟 위치
/thermal/hotspot (custom_msgs/msg/Hotspot)
    - int32 x
    - int32 y
    - float64 temperature
    - float64 timestamp
```

### 발행 토픽
```cpp
// 트래킹 상태
/targeting/status (custom_msgs/msg/TargetingStatus)
    - bool active
    - bool locked
    - float64 offset_x
    - float64 offset_y

// PX4 위치 명령
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
```

---

## 제어 알고리즘

### 1. 오차 계산
```cpp
// 화면 중심
int center_x = frame_width / 2;
int center_y = frame_height / 2;

// 핫스팟 오차
int offset_x = center_x - hotspot_x;
int offset_y = center_y - hotspot_y;
```

### 2. 드론 이동량 계산
```cpp
// 픽셀 오차 → 미터 변환 (카메라 FOV 기준)
float dx_meters = (offset_x / frame_width) * fov_horizontal;
float dy_meters = (offset_y / frame_height) * fov_vertical;

// PID 제어
float move_x = pid_x.calculate(dx_meters);
float move_y = pid_y.calculate(dy_meters);
```

### 3. 정조준 판정
```cpp
// 오차가 임계값 이내
if (abs(offset_x) < LOCK_THRESHOLD && 
    abs(offset_y) < LOCK_THRESHOLD) {
    locked_ = true;
}
```

---

## 안전 고려사항

1. **활성화 조건**:
   - 거리 8-12m 이내
   - GCS 격발 신호 수신
   - 핫스팟 감지됨

2. **비상 정지**:
   - GCS 취소 신호
   - 거리 범위 이탈
   - 핫스팟 소실 (3초 이상)

3. **제어 한계**:
   - 최대 이동 속도: 0.5 m/s
   - 최대 오차 허용: 100 픽셀

---

## 다음 단계

### 1주차: 기본 구현
- [ ] `hotspot_tracker.cpp` 구현
- [ ] `targeting_overlay.cpp` 구현
- [ ] `targeting_manager.cpp` 기본 구조

### 2주차: 드론 제어
- [ ] `drone_position_controller.cpp` 구현
- [ ] PX4 연동 테스트
- [ ] 미세 조정 로직

### 3주차: 통합 테스트
- [ ] GCS 신호 연동
- [ ] 전체 시스템 테스트
- [ ] 안전 로직 검증

---

**작성일**: 2025-12-31  
**상태**: 설계 완료  
**우선순위**: Phase 3 (lidar 다음)
