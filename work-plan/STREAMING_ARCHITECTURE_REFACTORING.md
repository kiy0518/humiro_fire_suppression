# 스트리밍 아키텍처 리팩토링 계획

**작성일**: 2026-01-03  
**목적**: RGB/열화상/OSD 스트리밍 구조 개선 및 안정성 향상

---

## 1. 현재 문제점 분석

### 1.1 심각한 문제: USB 카메라 미연결 시 VIM4 재부팅
**문제**: USB 카메라가 물리적으로 연결되지 않은 상태에서 실행하면 VIM4가 재부팅됨

**원인 분석**:
- `VideoCapture::open()` 호출 시 존재하지 않는 디바이스에 접근
- V4L2 드라이버 레벨에서 예외가 발생하여 시스템 크래시 가능
- 예외 처리 부재로 인한 안전하지 않은 카메라 접근

**영향**:
- 시스템 안정성 저하
- 프로덕션 환경에서 치명적
- 카메라 없이도 OSD 스트리밍은 가능해야 함

### 1.2 코드 구조 문제: main.cpp 과다 집중
**현재 상태**:
- `main.cpp`: **827줄** (과도하게 긴 단일 파일)
- 모든 스레드 함수가 전역 함수로 정의됨
- 비즈니스 로직이 main 함수에 집중

**문제점**:
- 유지보수 어려움
- 테스트 어려움
- 책임 분리 부족
- 코드 재사용성 저하

**main.cpp에 포함된 함수들**:
```cpp
- signal_handler()              // 시그널 핸들러
- rgb_capture_thread()           // RGB 캡처 스레드
- thermal_capture_thread()       // 열화상 캡처 스레드
- lidar_thread()                 // 라이다 스레드
- drawPlaceholder()              // 플레이스홀더 그리기
- ammunition_simulation_thread() // 탄약 시뮬레이션
- drawCameraStatus()             // 카메라 상태 표시
- composite_thread()             // 프레임 합성 스레드
```

### 1.3 스트리밍 룰 불명확
**현재 문제**:
- RGB, 열화상, OSD의 우선순위가 명확하지 않음
- 카메라 상태에 따른 스트리밍 동작이 일관되지 않음
- 에러 복구 메커니즘이 부족

**필요한 룰**:
1. OSD는 항상 최우선 출력
2. 카메라는 비동기 초기화, 실패해도 계속 진행
3. 카메라 상태는 왼쪽 하단에 명확히 표시
4. 스트리밍은 카메라 없이도 시작 가능
5. 물리적인 연결이 없는 경우 usb 연결이벤트 지속적으로 확인후 usb 카메라 확인(또는 주기적으로 연결확인(5초 주기로 확인))
---

## 2. 해결 방안

### 2.1 USB 카메라 예외 처리 강화

#### 2.1.1 디바이스 존재 확인 (사전 검증)
```cpp
// camera_manager.cpp에 추가
bool CameraManager::device_exists(const std::string& device_path) {
    struct stat st;
    if (stat(device_path.c_str(), &st) != 0) {
        return false;
    }
    
    // 추가 검증: 디바이스가 실제로 카메라인지 확인
    // V4L2 ioctl로 확인
    int fd = open(device_path.c_str(), O_RDWR);
    if (fd < 0) {
        return false;
    }
    
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        close(fd);
        return false;
    }
    
    close(fd);
    return (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE);
}
```

#### 2.1.2 try-catch로 VideoCapture 래핑
```cpp
bool CameraManager::safe_open_camera(int camera_id, cv::VideoCapture& cap) {
    try {
        // 디바이스 존재 확인
        std::string device = "/dev/video" + std::to_string(camera_id);
        if (!device_exists(device)) {
            return false;
        }
        
        // 타임아웃 설정 (5초)
        auto start = std::chrono::steady_clock::now();
        cap.open(camera_id, cv::CAP_V4L2);
        
        // 타임아웃 체크
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > 5) {
            cap.release();
            return false;
        }
        
        return cap.isOpened();
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV 예외: " << e.what() << std::endl;
        if (cap.isOpened()) {
            cap.release();
        }
        return false;
    } catch (const std::exception& e) {
        std::cerr << "표준 예외: " << e.what() << std::endl;
        if (cap.isOpened()) {
            cap.release();
        }
        return false;
    } catch (...) {
        std::cerr << "알 수 없는 예외 발생" << std::endl;
        if (cap.isOpened()) {
            cap.release();
        }
        return false;
    }
}
```

#### 2.1.3 카메라 읽기 시 예외 처리
```cpp
bool CameraManager::read_rgb_frame(cv::Mat& frame) {
    if (!cap_rgb_.isOpened()) {
        return false;
    }
    
    try {
        bool success = cap_rgb_.read(frame);
        if (!success || frame.empty()) {
            return false;
        }
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "RGB 프레임 읽기 예외: " << e.what() << std::endl;
        // 재연결 시도
        reconnect_rgb_camera();
        return false;
    } catch (...) {
        std::cerr << "RGB 프레임 읽기 알 수 없는 예외" << std::endl;
        return false;
    }
}
```

### 2.2 코드 구조 개선: 모듈화

#### 2.2.1 새로운 클래스 구조
```
application/
├── main.cpp                    # 진입점만 (100줄 이하)
├── src/
│   ├── application_manager.h   # 애플리케이션 전체 관리
│   ├── application_manager.cpp
│   ├── frame_compositor.h      # 프레임 합성 로직
│   ├── frame_compositor.cpp
│   ├── camera_status_display.h # 카메라 상태 표시
│   ├── camera_status_display.cpp
│   └── thread_manager.h        # 스레드 관리
│   └── thread_manager.cpp
```

#### 2.2.2 ApplicationManager 클래스
```cpp
// application/src/application_manager.h
class ApplicationManager {
public:
    ApplicationManager();
    ~ApplicationManager();
    
    bool initialize();
    void run();
    void shutdown();
    
private:
    // 컴포넌트
    CameraManager* camera_manager_;
    ThermalProcessor* thermal_processor_;
    StatusOverlay* status_overlay_;
    TargetingFrameCompositor* targeting_compositor_;
    StreamingManager* streaming_manager_;
    LidarInterface* lidar_interface_;
    
    // 스레드 관리
    std::thread rgb_capture_thread_;
    std::thread thermal_capture_thread_;
    std::thread lidar_thread_;
    std::thread composite_thread_;
    std::thread camera_init_thread_;
    
    // 상태
    std::atomic<bool> is_running_;
    
    // 내부 메서드
    void start_threads();
    void stop_threads();
    void rgb_capture_loop();
    void thermal_capture_loop();
    void lidar_loop();
    void composite_loop();
    void camera_init_loop();
};
```

#### 2.2.3 FrameCompositor 클래스
```cpp
// application/src/frame_compositor.h
class FrameCompositor {
public:
    FrameCompositor(
        StatusOverlay* status_overlay,
        ThermalOverlay* thermal_overlay,
        TargetingFrameCompositor* targeting_compositor
    );
    
    cv::Mat compose(
        const cv::Mat& rgb_frame,
        const ThermalData& thermal_data,
        bool has_rgb,
        bool has_thermal
    );
    
private:
    StatusOverlay* status_overlay_;
    ThermalOverlay* thermal_overlay_;
    TargetingFrameCompositor* targeting_compositor_;
    CameraStatusDisplay status_display_;
    
    void draw_osd(cv::Mat& frame);
    void draw_camera_status(cv::Mat& frame, bool rgb_ready, bool thermal_ready);
};
```

### 2.3 스트리밍 룰 정의

#### 2.3.1 우선순위 규칙
```
우선순위 1: OSD (StatusOverlay)
  - 항상 최우선 출력
  - 카메라 상태와 무관하게 표시
  - 검은 배경 위에도 표시 가능

우선순위 2: RGB 카메라
  - RGB 프레임이 있으면 배경으로 사용
  - OSD는 RGB 위에 오버레이

우선순위 3: 열화상 카메라
  - RGB가 없을 때만 배경으로 사용
  - RGB가 있으면 오버레이로만 사용 (OVERLAY_THERMAL 옵션)

우선순위 4: 타겟팅 오버레이
  - 라이다, 조준선, hotspot
  - 모든 레이어 위에 표시

우선순위 5: 카메라 상태 메시지
  - 왼쪽 하단에 표시
  - 카메라가 없거나 로딩 중일 때만 표시
```

#### 2.3.2 상태 머신
```cpp
enum class CameraState {
    NOT_INITIALIZED,  // 초기화 전
    INITIALIZING,     // 초기화 중
    READY,            // 준비됨
    CAPTURING,        // 캡처 중
    ERROR,            // 에러
    DISCONNECTED      // 연결 끊김
};

class CameraStateMachine {
public:
    CameraState get_rgb_state() const { return rgb_state_; }
    CameraState get_thermal_state() const { return thermal_state_; }
    
    void update_rgb_state(bool has_frame, bool is_ready);
    void update_thermal_state(bool has_frame, bool is_ready);
    
    std::string get_status_message(CameraState state) const;
    
private:
    CameraState rgb_state_ = CameraState::NOT_INITIALIZED;
    CameraState thermal_state_ = CameraState::NOT_INITIALIZED;
};
```

#### 2.3.3 스트리밍 시작 조건
```cpp
// 스트리밍은 항상 시작 가능 (카메라 없어도)
bool should_start_streaming() {
    // OSD는 항상 있으므로 true
    return true;
}

// 카메라 초기화는 비동기로 진행
void start_streaming_async() {
    // 1. 스트리밍 서버 즉시 시작 (OSD만)
    streaming_manager->start();
    
    // 2. 카메라 초기화는 별도 스레드에서
    camera_init_thread_ = std::thread([this]() {
        initialize_cameras_async();
    });
}
```

---

## 3. 구현 계획

### Phase 1: 예외 처리 강화 (우선순위: 높음)
**기간**: 1일  
**목표**: USB 카메라 미연결 시 재부팅 방지

1. `camera_manager.cpp`에 예외 처리 추가
   - `device_exists()` 함수 구현
   - `safe_open_camera()` 함수 구현
   - 모든 `VideoCapture` 접근에 try-catch 적용

2. 테스트
   - USB 카메라 없이 실행
   - 시스템 안정성 확인

### Phase 2: 코드 모듈화 (우선순위: 중간)
**기간**: 2-3일  
**목표**: main.cpp 분리 및 모듈화

1. `ApplicationManager` 클래스 생성
   - 모든 전역 변수를 멤버 변수로 이동
   - 스레드 함수를 멤버 함수로 이동

2. `FrameCompositor` 클래스 생성
   - 프레임 합성 로직 분리
   - 카메라 상태 표시 로직 분리

3. `main.cpp` 간소화
   - 진입점만 남기기
   - ApplicationManager 사용

### Phase 3: 스트리밍 룰 구현 (우선순위: 중간)
**기간**: 1-2일  
**목표**: 명확한 스트리밍 룰 적용

1. `CameraStateMachine` 구현
   - 상태 관리
   - 상태 메시지 생성

2. 우선순위 규칙 적용
   - FrameCompositor에 우선순위 로직 구현

3. 문서화
   - 스트리밍 룰 문서 작성

---

## 4. 파일 구조 제안

```
humiro_fire_suppression/
├── application/
│   ├── main.cpp                    # 진입점 (100줄 이하)
│   ├── src/
│   │   ├── application_manager.h
│   │   ├── application_manager.cpp
│   │   ├── frame_compositor.h
│   │   ├── frame_compositor.cpp
│   │   ├── camera_status_display.h
│   │   ├── camera_status_display.cpp
│   │   ├── camera_state_machine.h
│   │   ├── camera_state_machine.cpp
│   │   └── thread_manager.h
│   │   └── thread_manager.cpp
│   └── CMakeLists.txt
├── thermal/
│   └── src/
│       ├── camera_manager.h
│       └── camera_manager.cpp      # 예외 처리 강화
├── streaming/
│   └── src/
│       └── streaming_manager.h     # 스트리밍 룰 적용
└── work-plan/
    └── STREAMING_ARCHITECTURE_REFACTORING.md  # 이 문서
```

---

## 5. 예상 효과

### 5.1 안정성 향상
- USB 카메라 미연결 시 재부팅 방지
- 예외 상황에서도 안정적 동작
- 프로덕션 환경 안정성 확보

### 5.2 유지보수성 향상
- main.cpp 간소화 (827줄 → 100줄 이하)
- 책임 분리로 코드 이해도 향상
- 테스트 가능한 구조

### 5.3 확장성 향상
- 새로운 카메라 타입 추가 용이
- 스트리밍 룰 변경 용이
- 모듈별 독립적 개발 가능

---

## 6. 리스크 및 대응 방안

### 6.1 리스크
1. **리팩토링 중 버그 발생 가능**
   - 대응: 단계별 테스트, 기존 동작 보장

2. **성능 저하 가능성**
   - 대응: 프로파일링, 최적화

3. **개발 시간 증가**
   - 대응: 우선순위에 따라 단계적 구현

### 6.2 마이그레이션 전략
1. 기존 코드는 유지 (백업)
2. 새 구조와 병행 개발
3. 단계적 전환
4. 충분한 테스트 후 기존 코드 제거

---

## 7. 체크리스트

### Phase 1: 예외 처리
- [ ] `device_exists()` 구현
- [ ] `safe_open_camera()` 구현
- [ ] 모든 VideoCapture 접근에 예외 처리
- [ ] USB 카메라 없이 실행 테스트
- [ ] 시스템 안정성 확인

### Phase 2: 모듈화
- [ ] ApplicationManager 클래스 생성
- [ ] FrameCompositor 클래스 생성
- [ ] CameraStatusDisplay 클래스 생성
- [ ] main.cpp 간소화
- [ ] 컴파일 및 기본 동작 확인

### Phase 3: 스트리밍 룰
- [ ] CameraStateMachine 구현
- [ ] 우선순위 규칙 적용
- [ ] 상태 메시지 표시
- [ ] 문서화

---

## 8. 참고 사항

- 기존 코드는 `application/main.cpp.backup`으로 백업
- 각 Phase는 독립적으로 테스트 가능해야 함
- 성능 저하가 없어야 함 (현재 수준 유지)
- 기존 기능은 모두 유지되어야 함

---

**다음 단계**: Phase 1 (예외 처리 강화)부터 시작

