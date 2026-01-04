# 영상 스트리밍 시스템 개발 계획 (Streaming System Development Plan)

작성일: 2025-01-01  
**상태**: 개선 계획

---

## 개요

영상 및 데이터 스트리밍 시스템의 비동기식 전송 방식을 정의합니다.

**핵심 원칙**: 준비되는 데이터부터 즉시 스트리밍 시작 (비동기식)

---

## 현재 방식 (동기식 - 개선 필요)

### 문제점
- **두 카메라 모두 초기화 완료 후에만 스트리밍 시작**
- RGB 카메라 또는 열화상 카메라 중 하나라도 실패하면 스트리밍이 시작되지 않음
- 사용자가 아무것도 볼 수 없어 시스템 상태 파악이 어려움
- 초기화 지연으로 인한 사용자 경험 저하

### 현재 동작 흐름
```
1. RGB 카메라 초기화 시도
2. 열화상 카메라 초기화 시도
3. 두 카메라 모두 성공 → 스트리밍 시작
4. 하나라도 실패 → 스트리밍 시작 안 함
```

---

## 개선된 방식 (비동기식)

### 핵심 개념
**준비되는 데이터부터 즉시 스트리밍 시작, 이후 준비되는 데이터를 점진적으로 추가**

### 스트리밍 데이터 종류
1. **열화상 영상** (Thermal Video)
2. **RGB 영상** (RGB Video)
3. **전방 거리 정보** (Front Distance - LiDAR)
4. **상태 메시지** (Status Messages)
5. **기타 OSD 정보** (기체 이름, 편대 정보, 배터리 등)

### 비동기식 동작 흐름
```
시작
  ↓
[1단계] 첫 번째 준비된 데이터부터 스트리밍 시작
  ├─ 열화상만 준비 → 열화상만 스트리밍
  ├─ RGB만 준비 → RGB만 스트리밍
  └─ 둘 다 준비 → 통합 영상 스트리밍
  ↓
[2단계] 추가 데이터 준비되면 즉시 추가
  ├─ LiDAR 데이터 준비 → 거리 정보 오버레이 추가
  ├─ 상태 정보 준비 → 상태 메시지 추가
  └─ 기타 OSD 정보 준비 → OSD 오버레이 추가
  ↓
[3단계] 모든 데이터 통합 스트리밍
  └─ 완전한 통합 영상 스트리밍
```

---

## 구현 계획

### Phase 1: 스트리밍 상태 관리 (1일)

#### 1.1 스트리밍 상태 머신
- [ ] 스트리밍 상태 정의
  - `IDLE`: 스트리밍 시작 전
  - `PARTIAL`: 일부 데이터만 스트리밍 중
  - `FULL`: 모든 데이터 스트리밍 중
- [ ] 각 데이터 소스별 준비 상태 추적
  - RGB 카메라: `rgb_ready_`
  - 열화상 카메라: `thermal_ready_`
  - LiDAR: `lidar_ready_`
  - 상태 정보: `status_ready_`

**클래스 구조**:
```cpp
class StreamingState {
public:
    enum class DataSource {
        RGB_CAMERA,
        THERMAL_CAMERA,
        LIDAR,
        STATUS_INFO
    };
    
    void setReady(DataSource source, bool ready);
    bool isReady(DataSource source) const;
    bool hasAnyData() const;  // 하나라도 준비되면 true
    bool hasAllData() const;  // 모두 준비되면 true
    
private:
    std::map<DataSource, bool> source_states_;
};
```

#### 1.2 스트리밍 시작 조건 변경
- [ ] 기존: 두 카메라 모두 준비되어야 시작
- [ ] 변경: 하나라도 준비되면 즉시 시작
- [ ] 준비 상태 모니터링 및 자동 추가

### Phase 2: 점진적 데이터 추가 (2일)

#### 2.1 프레임 컴포지터 개선
- [ ] 부분 데이터 처리 로직
  - RGB만 있을 때: RGB 영상만 표시
  - 열화상만 있을 때: 열화상 영상만 표시
  - 둘 다 있을 때: 통합 영상 표시
- [ ] OSD 오버레이 점진적 추가
  - LiDAR 준비되면 거리 정보 추가
  - 상태 정보 준비되면 상태 메시지 추가

**프레임 컴포지터 수정**:
```cpp
class TargetingFrameCompositor {
public:
    void composeFrame(cv::Mat& output_frame,
                     const cv::Mat* rgb_frame,      // nullptr 가능
                     const cv::Mat* thermal_frame, // nullptr 가능
                     const LidarData* lidar_data,   // nullptr 가능
                     const StatusData* status_data); // nullptr 가능
    
private:
    void drawPlaceholder(cv::Mat& frame, const std::string& message);
    // "RGB 카메라 초기화 중...", "열화상 카메라 초기화 중..." 등
};
```

#### 2.2 플레이스홀더 표시
- [ ] 준비되지 않은 데이터 소스에 대한 안내 메시지
  - "RGB 카메라 초기화 중..."
  - "열화상 카메라 초기화 중..."
  - "LiDAR 연결 중..."
- [ ] 준비 완료 시 자동으로 실제 데이터로 전환

### Phase 3: 스트리밍 최적화 (1일)

#### 3.1 프레임 레이트 관리
- [ ] 각 데이터 소스별 독립적인 프레임 레이트
  - RGB: 30 FPS
  - 열화상: 9 FPS
  - LiDAR: 10 Hz
  - 상태 정보: 1 Hz
- [ ] 데이터가 없을 때도 안정적인 스트리밍 유지

#### 3.2 에러 처리
- [ ] 데이터 소스 실패 시에도 다른 데이터는 계속 스트리밍
- [ ] 재연결 시 자동으로 데이터 추가
- [ ] 사용자에게 명확한 상태 표시

---

## 데이터 흐름

### 스트리밍 데이터 우선순위

#### 1순위: 기본 영상 데이터
- **RGB 영상**: 가장 빠르게 준비되는 경우가 많음
- **열화상 영상**: 초기화 시간이 다소 걸릴 수 있음
- **동작**: 하나라도 준비되면 즉시 스트리밍 시작

#### 2순위: 거리 정보 (LiDAR)
- **전방 거리**: LiDAR 센서 연결 및 초기화 완료 후 추가
- **표시 위치**: 화면 하단 중앙
- **동작**: 준비되면 기존 스트리밍에 오버레이로 추가

#### 3순위: 상태 정보
- **기체 상태**: PX4 연결 및 상태 수신 후 추가
- **소화탄 갯수**: 시스템 초기화 완료 후 추가
- **기타 정보**: 준비되는 대로 점진적 추가
- **표시 위치**: 화면 왼쪽 하단
- **동작**: 준비되면 기존 스트리밍에 오버레이로 추가

---

## 구현 예시

### 스트리밍 시작 로직
```cpp
void startStreamingWhenReady() {
    StreamingState state;
    
    // RGB 카메라 확인
    if (camera_manager && camera_manager->is_rgb_ready()) {
        state.setReady(StreamingState::DataSource::RGB_CAMERA, true);
    }
    
    // 열화상 카메라 확인
    if (camera_manager && camera_manager->is_thermal_ready()) {
        state.setReady(StreamingState::DataSource::THERMAL_CAMERA, true);
    }
    
    // 하나라도 준비되면 스트리밍 시작
    if (state.hasAnyData()) {
        if (!streaming_manager->isRunning()) {
            streaming_manager->start();
            std::cout << "  ✓ 스트리밍 시작 (준비된 데이터: ";
            if (state.isReady(StreamingState::DataSource::RGB_CAMERA)) {
                std::cout << "RGB ";
            }
            if (state.isReady(StreamingState::DataSource::THERMAL_CAMERA)) {
                std::cout << "열화상 ";
            }
            std::cout << ")" << std::endl;
        }
    }
}
```

### 프레임 컴포지션 로직
```cpp
void composeFrameWithPartialData() {
    cv::Mat output_frame(OUTPUT_HEIGHT, OUTPUT_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    
    // RGB 영상이 있으면 추가
    if (rgb_frame && !rgb_frame->empty()) {
        // RGB 영상 처리 및 추가
        cv::Mat rgb_resized;
        cv::resize(*rgb_frame, rgb_resized, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT));
        rgb_resized.copyTo(output_frame);
    } else {
        // RGB 영상이 없으면 플레이스홀더
        drawPlaceholder(output_frame, "RGB 카메라 초기화 중...");
    }
    
    // 열화상 영상이 있으면 오버레이
    if (thermal_frame && !thermal_frame->empty()) {
        thermal_overlay->overlayThermal(output_frame, *thermal_frame);
    } else if (!rgb_frame || rgb_frame->empty()) {
        // RGB도 없고 열화상도 없으면 열화상 플레이스홀더
        drawPlaceholder(output_frame, "열화상 카메라 초기화 중...");
    }
    
    // LiDAR 데이터가 있으면 거리 정보 추가
    if (lidar_data && lidar_data->is_valid) {
        distance_overlay->drawDistanceOverlay(output_frame, *lidar_data);
    }
    
    // 상태 정보가 있으면 상태 오버레이 추가
    if (status_data && status_data->is_valid) {
        status_overlay->draw(output_frame);
    }
    
    // 스트리밍
    streaming_manager->pushFrame(output_frame);
}
```

---

## 사용자 경험 개선

### 이전 방식의 문제
- ❌ 카메라 하나만 실패해도 아무것도 볼 수 없음
- ❌ 초기화 시간이 길어질수록 사용자 대기 시간 증가
- ❌ 시스템 상태 파악이 어려움

### 개선된 방식의 장점
- ✅ 준비되는 데이터부터 즉시 확인 가능
- ✅ 부분 데이터라도 시스템 상태 파악 가능
- ✅ 점진적으로 완전한 영상으로 발전
- ✅ 사용자 대기 시간 최소화

---

## 우선순위

### 높음 (즉시 구현)
1. 스트리밍 시작 조건 변경 (하나라도 준비되면 시작)
2. 부분 데이터 처리 로직 (RGB 또는 열화상만 있을 때)
3. 플레이스홀더 표시 (준비 중인 데이터 소스 안내)

### 중간 (1주 내)
4. LiDAR 데이터 점진적 추가
5. 상태 정보 점진적 추가
6. 재연결 시 자동 데이터 추가

### 낮음 (2주 내)
7. 프레임 레이트 최적화
8. 에러 복구 로직 강화
9. 사용자 피드백 개선

---

## 테스트 시나리오

### 시나리오 1: RGB 카메라만 준비
1. RGB 카메라 초기화 성공
2. 열화상 카메라 초기화 실패
3. **예상 결과**: RGB 영상만 스트리밍 시작
4. 열화상 카메라 재연결 시 자동으로 추가

### 시나리오 2: 열화상 카메라만 준비
1. RGB 카메라 초기화 실패
2. 열화상 카메라 초기화 성공
3. **예상 결과**: 열화상 영상만 스트리밍 시작
4. RGB 카메라 재연결 시 자동으로 추가

### 시나리오 3: 점진적 데이터 추가
1. RGB 카메라만 준비 → RGB만 스트리밍
2. 열화상 카메라 준비 → 통합 영상으로 전환
3. LiDAR 준비 → 거리 정보 오버레이 추가
4. 상태 정보 준비 → 상태 메시지 추가
5. **예상 결과**: 점진적으로 완전한 영상으로 발전

---

## 시작 지점 (구현 순서)

### Step 1: CameraManager 개선 (우선)
**파일**: `thermal/src/camera_manager.h`, `thermal/src/camera_manager.cpp`

1. **개별 카메라 초기화 메서드 추가**
   - `bool initialize_rgb_camera()`: RGB 카메라만 초기화
   - `bool initialize_thermal_camera()`: 열화상 카메라만 초기화
   - 기존 `initialize()`는 두 메서드를 순차 호출하도록 변경

2. **준비 상태 확인 메서드 추가**
   - `bool is_rgb_ready() const`: RGB 카메라 준비 여부
   - `bool is_thermal_ready() const`: 열화상 카메라 준비 여부

3. **초기화 실패 시에도 계속 진행하도록 변경**
   - 하나라도 성공하면 `initialize()`는 true 반환
   - 실패한 카메라는 나중에 재연결 시도

### Step 2: StreamingState 클래스 생성
**파일**: `streaming/src/streaming_state.h`, `streaming/src/streaming_state.cpp`

- 데이터 소스별 준비 상태 추적
- `hasAnyData()`, `hasAllData()` 메서드 구현

### Step 3: main.cpp 수정
**파일**: `application/main.cpp`

1. **카메라 초기화 실패해도 계속 진행**
   - `camera_manager->initialize()` 실패해도 프로그램 종료하지 않음
   - 준비된 카메라만 사용하여 스트리밍 시작

2. **스트리밍 시작 조건 변경**
   - 기존: 카메라 초기화 완료 후 스트리밍 시작
   - 변경: 하나라도 준비되면 즉시 스트리밍 시작

3. **스트리밍 상태 모니터링 루프 추가**
   - 주기적으로 준비 상태 확인
   - 새로 준비된 데이터 소스 자동 추가

### Step 4: TargetingFrameCompositor 개선
**파일**: `targeting/src/targeting_frame_compositor.h`, `targeting/src/targeting_frame_compositor.cpp`

1. **부분 데이터 처리**
   - RGB만 있을 때, 열화상만 있을 때 처리
   - nullptr 체크 추가

2. **플레이스홀더 표시**
   - 준비되지 않은 데이터 소스에 대한 안내 메시지

### Step 5: composite_thread 수정
**파일**: `application/main.cpp` (composite_thread 함수)

- 부분 데이터 처리 로직 적용
- 준비된 데이터만 사용하여 프레임 합성

---

## 구체적인 시작 코드

### 1단계: CameraManager에 메서드 추가

```cpp
// camera_manager.h에 추가
bool initialize_rgb_camera();
bool initialize_thermal_camera();
bool is_rgb_ready() const { return cap_rgb_.isOpened(); }
bool is_thermal_ready() const { return cap_thermal_.isOpened(); }
```

### 2단계: main.cpp에서 초기화 로직 변경

```cpp
// 기존 (line 342-355):
if (!camera_manager->initialize()) {
    // 실패 시 종료
    return 1;
}

// 변경:
camera_manager->initialize_rgb_camera();      // 실패해도 계속
camera_manager->initialize_thermal_camera();  // 실패해도 계속

if (!camera_manager->is_rgb_ready() && !camera_manager->is_thermal_ready()) {
    // 둘 다 실패한 경우에만 종료
    std::cerr << "모든 카메라 초기화 실패" << std::endl;
    return 1;
}
```

### 3단계: 스트리밍 시작 조건 변경

```cpp
// 기존 (line 415-423):
// 스트리밍 서버 시작 (카메라 초기화 완료 후)

// 변경:
// 하나라도 준비되면 스트리밍 시작
if (camera_manager->is_rgb_ready() || camera_manager->is_thermal_ready()) {
    streaming_manager = new StreamingManager();
    if (streaming_manager->initialize(&frame_queue, &web_frame_queue)) {
        streaming_manager->start();
        std::cout << "  ✓ 스트리밍 시작 (";
        if (camera_manager->is_rgb_ready()) std::cout << "RGB ";
        if (camera_manager->is_thermal_ready()) std::cout << "열화상 ";
        std::cout << ")" << std::endl;
    }
}
```

---

## 참고사항

- 스트리밍은 RTSP와 HTTP 두 가지 방식 모두 지원
- 각 데이터 소스는 독립적으로 관리되어야 함
- 데이터 소스 실패가 다른 데이터 소스에 영향을 주지 않아야 함
- 사용자에게 명확한 상태 피드백 제공 필요

---

**작성자**: Claude Code Assistant  
**버전**: v1.1 (구체적인 시작 지점 추가)  
**작성일**: 2025-01-01  
**다음 리뷰**: Phase 1 완료 시
