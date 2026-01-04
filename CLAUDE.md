# CLAUDE.md - Humiro Fire Suppression 프로그래밍 가이드

**목적**: Claude Code가 이 프로젝트에서 코드를 작성할 때 참고하는 가이드

**버전**: v2.1 (2026-01-04)
**프로젝트 진행률**: 75% (Phase 1 완료)
**총 코드량**: 10,500+ LOC

---

## 프로젝트 개요

**Humiro Fire Suppression System** - 다중 드론 협조를 위한 ROS2 기반 자율 화재 진압 시스템

**핵심 기술**:
- C++17 (모든 핵심 모듈)
- ROS2 Humble
- PX4 Firmware v1.16.0
- OpenCV + GStreamer

**현재 상태** (2026-01-04):
- ✅ 열화상 시스템 (2,665 LOC)
- ✅ LiDAR 거리 측정 (1,188 LOC)
- ✅ 스트리밍 시스템 (800 LOC)
- ✅ 상태 모니터링 OSD (300 LOC)
- ✅ VIM4 자율 제어 (2,260 LOC) - Phase 1 완료
- ⏳ 편대 통신 (다음 구현 목표)

---

## 코딩 표준

### C/C++ 우선 정책 (필수)

**모든 핵심 모듈은 C++17로 작성**:
- ✅ 실시간 처리 → C++ 필수
- ✅ 하드웨어 인터페이스 → C++ 필수
- ✅ 이미지/비디오 처리 → C++ 필수
- ⚠️ ROS2 래퍼 → Python 허용 (C++ 라이브러리 호출만)

**코딩 스타일**:
- C++17 표준, Google C++ Style Guide
- 스마트 포인터 사용 (`std::unique_ptr`, `std::shared_ptr`)
- 멀티스레드: 스레드 안전 큐 사용
- 에러 처리: 명확한 에러 코드 반환

**금지 사항**:
- ❌ 하드코딩된 경로 (환경 변수 사용)
- ❌ Python으로 실시간 처리
- ❌ 불필요한 메모리 복사
- ❌ 불필요한 md파일 생성

---

## 프로젝트 구조

```
humiro_fire_suppression/
├── thermal/                    # ✅ 열화상 시스템 (2,665 LOC)
│   ├── src/
│   │   ├── main.cpp            # 멀티스레드 오케스트레이션
│   │   ├── camera_manager      # 카메라 I/O
│   │   ├── thermal_processor   # 핫스팟 감지
│   │   ├── frame_compositor    # RGB+Thermal 정합
│   │   ├── rtsp_server         # RTSP 스트리밍
│   │   └── http_server         # HTTP 스트리밍
│   └── CMakeLists.txt
│
├── lidar/                      # ✅ LiDAR 시스템 (1,188 LOC)
│   ├── src/
│   │   ├── lidar_interface     # LD19 UART 통신
│   │   ├── distance_overlay    # 거리 시각화
│   │   └── lidar_ros2_publisher # ROS2 발행
│   └── CMakeLists.txt
│
├── navigation/                 # ✅ 자율 제어 (2,260 LOC)
│   ├── src/offboard/
│   │   ├── autonomous/         # Phase 1 핸들러
│   │   │   ├── arm_handler
│   │   │   ├── takeoff_handler
│   │   │   ├── waypoint_handler
│   │   │   ├── distance_adjuster
│   │   │   ├── rtl_handler
│   │   │   └── offboard_manager
│   │   ├── communication/      # ⏳ Phase 2 예정
│   │   └── formation/          # ⏳ Phase 3 예정
│   └── CMakeLists.txt
│
├── osd/                        # ✅ OSD 시스템 (1,500 LOC)
│   ├── src/
│   │   ├── lidar/              # 거리 오버레이
│   │   ├── thermal/            # 열화상 오버레이
│   │   ├── targeting/          # 타겟팅 오버레이
│   │   └── status/             # 상태 모니터링 OSD
│   └── CMakeLists.txt
│
├── application/                # ✅ 통합 애플리케이션 (1,500 LOC)
│   ├── src/
│   │   ├── main.cpp
│   │   └── application_manager
│   └── CMakeLists.txt
│
├── custom_message/             # MAVLink 커스텀 메시지
│   ├── include/custom_message/
│   ├── src/
│   └── CMakeLists.txt
│
├── targeting/                  # ⏳ 타겟팅 (30% 완료)
├── throwing_mechanism/         # ⏳ 발사 메커니즘 (미구현)
└── work-plan/                  # 프로젝트 계획 문서
```

---

## 시스템 아키텍처

### 데이터 흐름

```
열화상 카메라 (thermal/)
    ↓ 핫스팟 위치 + 온도
LiDAR (lidar/)
    ↓ 거리 측정 (10m 확인)
Navigation (navigation/)
    ↓ OFFBOARD 자율 비행
    ├─ GPS 좌표 이동
    ├─ LiDAR 거리 조정 (10m±1m)
    └─ 호버링 (발사 준비)
Targeting (targeting/) ⏳ 구현 예정
    ↓ 핫스팟 추적 + 드론 미세 조정
Throwing Mechanism ⏳ 구현 예정
    └─ GPIO 발사
```

### Phase 1 자율 비행 상태 머신

```
IDLE → GPS 신호 대기
  ↓
ARMING → OFFBOARD 모드 + ARM
  ↓
TAKEOFF → 5m 고도 이륙
  ↓
NAVIGATE → GPS 좌표 이동
  ↓
ADJUST_DISTANCE → LiDAR 10m±1m 조정
  ↓
HOVER → 타겟팅/발사 준비
  ↓
RTL → 자동 복귀/착륙
  ↓
LANDED → 미션 완료
```

### ROS2 토픽

**PX4 → VIM4** (구독):
- `/fmu/out/vehicle_status` - 비행 상태
- `/fmu/out/vehicle_gps_position` - GPS 위치
- `/fmu/out/battery_status` - 배터리

**VIM4 → PX4** (발행):
- `/fmu/in/offboard_control_mode` - OFFBOARD 제어 모드
- `/fmu/in/trajectory_setpoint` - 위치 목표
- `/fmu/in/vehicle_command` - 명령 (ARM, TAKEOFF 등)

**VIM4 센서** (발행):
- `/lidar/front_distance` - 전방 거리
- `/thermal/hotspot` - 핫스팟 정보
- `/offboard/status` - OFFBOARD 상태

---

## 주요 클래스 및 API

### Navigation 시스템

**OffboardManager** (`navigation/src/offboard/autonomous/offboard_manager.h`):
```cpp
class OffboardManager {
public:
    enum class State {
        IDLE,
        ARMING,
        TAKEOFF,
        NAVIGATE,
        ADJUST_DISTANCE,
        HOVER,
        RTL,
        LANDED,
        EMERGENCY_RTL
    };
    
    void run();  // 메인 루프
    void setState(State new_state);
    State getState() const;
};
```

**각 핸들러 인터페이스**:
```cpp
// 공통 인터페이스
class Handler {
public:
    virtual bool execute() = 0;  // true: 성공, false: 실패
    virtual bool isComplete() = 0;
};

// 예: ArmHandler
class ArmHandler : public Handler {
public:
    bool execute() override;      // ARM 명령 전송
    bool isComplete() override;   // ARM 완료 확인
};
```

### Thermal 시스템

**ThermalProcessor** (`thermal/src/thermal_processor.h`):
```cpp
class ThermalProcessor {
public:
    struct Hotspot {
        cv::Point position;
        float temperature;
        int confidence;
    };
    
    std::vector<Hotspot> detectHotspots(const cv::Mat& thermal_frame);
};
```

### LiDAR 시스템

**LidarInterface** (`lidar/src/lidar_interface.h`):
```cpp
class LidarInterface {
public:
    bool init(const std::string& port, int baud_rate);
    float getFrontDistance();  // 전방 거리 (미터)
    std::array<float, 360> getScan360();  // 360도 스캔
};
```

---

## 빌드 방법

### Thermal 시스템
```bash
cd ~/humiro_fire_suppression/thermal/src
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 출력: ./thermal_rgb_streaming
```

### LiDAR 시스템
```bash
cd ~/humiro_fire_suppression/lidar
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 출력: ./lidar_test
```

### Navigation 시스템
```bash
cd ~/humiro_fire_suppression/navigation
./build.sh
# 출력: build/test_arm, build/test_mission 등
```

### Application (통합)
```bash
cd ~/humiro_fire_suppression/application
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 출력: ./humiro_fire_suppression
```

---

## 다음 구현 목표

### Phase 2: 편대 통신 모듈 (3일)

**위치**: `navigation/src/offboard/communication/`

**구현할 클래스**:
```cpp
class FormationMember {
public:
    // ROS2 발행
    void publishStatus();  // /formation/member_status
    
    // ROS2 구독
    void onTargetAssignment(const TargetMsg& msg);  // /formation/target_assignment
    
private:
    rclcpp::Publisher<MemberStatusMsg>::SharedPtr status_pub_;
    rclcpp::Subscription<TargetMsg>::SharedPtr target_sub_;
};
```

**메시지 정의** (`custom_message/`):
```cpp
struct MemberStatusMsg {
    int drone_id;
    float battery_level;
    int ammo_count;
    std::string state;  // "IDLE", "FLYING", "TARGETING", etc.
    float target_distance;  // LiDAR 측정값
};

struct TargetMsg {
    int target_id;
    double latitude;
    double longitude;
    float altitude;
};
```

### Phase 3: 리더 조율 로직 (4일)

**위치**: `navigation/src/offboard/formation/`

**구현할 클래스**:
```cpp
class FormationLeader {
public:
    void analyzeFireZones();  // 화재 지점 분석
    void assignTargets();     // 드론별 목표 할당
    void monitorProgress();   // 진행 상황 모니터링
    
private:
    std::vector<MemberStatusMsg> member_statuses_;
    std::vector<Target> fire_targets_;
};
```

### Phase 5: 발사 메커니즘 (5일)

**위치**: `throwing_mechanism/src/`

**GPIO 제어**:
```cpp
class ThrowingMechanism {
public:
    void fire(int pin_number);  // 1-6
    void reload();
    int getAmmoCount() const;
    
private:
    std::array<bool, 6> fired_status_;
    void triggerGPIO(int pin);
};
```

---

## 개발 규칙

### 언어 선택
- **실시간 처리**: C++ 필수
- **하드웨어 인터페이스**: C++ 필수
- **성능 중요**: C++ 필수
- **ROS2 래퍼**: Python 허용 (C++ 호출만)

### 코드 품질
- 스마트 포인터 사용
- 에러 처리 필수
- 스레드 안전성 보장
- 메모리 누수 방지

### 경로 관리
- ❌ 절대 경로 하드코딩 금지
- ✅ 환경 변수 사용
- ✅ 상대 경로 (실행 파일 기준)

### 테스트
- 모든 핵심 기능에 단위 테스트
- 하드웨어는 모의 객체 사용
- 커밋 전 테스트 통과 확인

---

## 참고 문서

**프로젝트 계획**:
- `work-plan/000_PROJECT_PROGRESS_REPORT.md` - 진행률 보고서
- `work-plan/001_PROJECT_MASTER_PLAN.md` - 마스터 플랜
- `work-plan/013_NEXT_STEPS_FORMATION_CONTROL.md` - 편대 제어 계획

**기술 문서**:
- `docs/technical/LIDAR_TARGETING.md` - LiDAR 통합
- `docs/OFFBOARD_MODE_SETUP.md` - OFFBOARD 모드
- `docs/PX4_NAV_STATE_REFERENCE.md` - PX4 상태

**설치 및 운영** (코드 작성 시 불필요):
- `docs/setup/` - 설치 가이드
- README.md - 프로젝트 개요
