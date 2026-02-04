# CLAUDE.md - Humiro Fire Suppression 프로그래밍 가이드

**목적**: Claude Code가 이 프로젝트에서 코드를 작성할 때 참고하는 가이드

**버전**: v2.3 (2026-02-04)
**프로젝트 진행률**: ~62% (work-plan 기준)
**총 코드량**: 10,500+ LOC

**프로젝트 경로**: `/home/khadas/humiro_fire_suppression`

---

## ⚠️ 필수 확인 사항 (모든 작업 전)

**코드 작성 또는 답변 전에 반드시 `work-plan/` 폴더를 확인하세요!**

```bash
# work-plan 폴더 위치
/home/khadas/humiro_fire_suppression/work-plan/
```

### work-plan 문서 전체 목록

#### 📋 프로젝트 관리
| 파일 | 내용 |
|------|------|
| `000_PROJECT_PROGRESS_REPORT.md` | 프로젝트 진행 현황 |
| `001_PROJECT_MASTER_PLAN.md` | 전체 마스터 플랜 |
| `007_CURRENT_STATUS.md` | 현재 상태 |
| `026_IMPLEMENTATION_GAP_ANALYSIS.md` | 구현 현황 분석 |

#### 🔧 Phase 완료 보고서
| 파일 | 내용 |
|------|------|
| `002_PHASE2_COMPLETE.md` | Phase 2 완료 |
| `006_PHASE3_COMPLETE.md` | Phase 3 완료 |
| `012_20260102_autonomous_phase1_complete.md` | 자율 비행 Phase 1 완료 |

#### 🚁 자율 비행 / Navigation
| 파일 | 내용 |
|------|------|
| `014_VIM4_AUTONOMOUS_CONTROL_PLAN.md` | VIM4 자율 제어 계획 |
| `024_01_OFFBOARD_MODE_SEQUENCE_V2.md` | OFFBOARD 모드 시퀀스 |
| `013_NEXT_STEPS_FORMATION_CONTROL.md` | 편대 제어 다음 단계 |
| `025_COLLISION_AVOIDANCE_DESIGN.md` | 충돌 방지 설계 |
| `025_COLLISION_AVOIDANCE_VERIFICATION.md` | 충돌 방지 검증 |

#### 📡 통신 / 메시지
| 파일 | 내용 |
|------|------|
| `003_ROS2_COMMUNICATION_EXPLANATION.md` | ROS2 통신 설명 |
| `008_ROS2_TOPIC_ARCHITECTURE.md` | ROS2 토픽 아키텍처 |
| `010_ROS2_COMMUNICATION_MODULE.md` | ROS2 통신 모듈 |
| `019_MAVLINK_CUSTOM_MESSAGE.md` | MAVLink 커스텀 메시지 |
| `021_MAVLINK_ARCHITECTURE_REFACTORING.md` | MAVLink 아키텍처 리팩토링 |
| `022_COMMUNICATION_ARCHITECTURE.md` | 통신 아키텍처 |
| `022_COMMUNICATION_ARCHITECTURE_STATUS.md` | 통신 아키텍처 상태 |

#### 🎯 화재 진압 시나리오
| 파일 | 내용 |
|------|------|
| `015_FIRE_SUPPRESSION_SCENARIO.md` | 화재 진압 시나리오 |
| `020_RTK_GPS_COORDINATE_FORMAT.md` | RTK GPS 좌표 형식 |

#### 🖥️ 스트리밍 / GUI
| 파일 | 내용 |
|------|------|
| `009_STATUS_MONITORING_PLAN.md` | 상태 모니터링 계획 |
| `017_STREAMING_ARCHITECTURE_REFACTORING.md` | 스트리밍 아키텍처 리팩토링 |
| `016_QGC_DEVELOPMENT_GUIDE.md` | QGC 개발 가이드 |
| `027_GUI_MANAGEMENT_TOOL_PLAN.md` | GUI 관리 도구 계획 |

#### 🔩 기타
| 파일 | 내용 |
|------|------|
| `001-usb-camera-autosuspend-fix.md` | USB 카메라 자동 절전 수정 |
| `004_REFACTORING_COMPLETE_SUMMARY.md` | 리팩토링 완료 요약 |
| `005_QUICK_START_ROS2_TOPICS.md` | ROS2 토픽 빠른 시작 |
| `011_20260102_px4_msgs_fix.md` | PX4 메시지 수정 |
| `018_README.md` | README |
| `023_SCRIPT_DEVICE_CONFIG_INTEGRATION.md` | 스크립트 기기 설정 통합 |

### ⚠️ 문서 동기화 유지 (필수)
**`work-plan/` 폴더에 문서가 추가되거나 삭제되면 이 CLAUDE.md 파일의 목록도 함께 업데이트하세요!**

```bash
# work-plan 문서 목록 확인
ls -1 /home/khadas/humiro_fire_suppression/work-plan/*.md
```

### 확인 절차
1. **코드 수정 전**: 해당 모듈의 설계 문서 확인
2. **새 기능 추가 전**: 마스터 플랜과 일치 여부 확인
3. **버그 수정 전**: 기존 설계 의도 파악
4. **질문 답변 전**: 최신 진행 상황 확인

---

## 프로젝트 개요

**Humiro Fire Suppression System** - 다중 드론 협조를 위한 ROS2 기반 자율 화재 진압 시스템

**핵심 기술**:
- C++17 (모든 핵심 모듈)
- ROS2 Humble
- PX4 Firmware v1.16.0
- OpenCV + GStreamer

**현재 상태** (2026-02-04):
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


---

## 버전 관리 및 Git 규칙

### 새 세션 시작 시 (필수)

**커밋/태그 작업 전 반드시 최신 버전 확인**:
```bash
cd /home/khadas/humiro_fire_suppression
git fetch origin
git log -1 --oneline                    # 최신 커밋 확인
git tag --sort=-v:refname | grep "^v0" | head -1  # 최신 태그 확인
```

- 현재 버전을 파악한 후 다음 버전 번호 결정
- 예: 현재 v0.11.8이면 → PATCH는 v0.11.9, MINOR는 v0.12.0

### 버전 체계: Semantic Versioning

**형식**: `vMAJOR.MINOR.PATCH`

| 단계 | 버전 변경 | 조건 |
|------|-----------|------|
| **개발 단계** | MINOR (v0.11.x → v0.12.x) | Phase 완료, 주요 기능 추가, 새 모듈 통합 |
| **개발 단계** | PATCH (v0.11.7 → v0.11.8) | 버그 수정, 성능 개선, 작은 개선사항 |
| **정식 출시 후** | MAJOR (v1.x → v2.x) | 하위 호환성이 깨지는 변경, 아키텍처 전면 개편 |
| **정식 출시 후** | MINOR (v1.0.x → v1.1.x) | 새 기능 추가 (하위 호환 유지) |
| **정식 출시 후** | PATCH (v1.0.0 → v1.0.1) | 버그 수정, 문서 업데이트 |

### Git 커밋/태그 메시지 형식

**커밋 메시지와 태그 메시지는 동일한 전체 내용을 포함** (한 줄 요약 금지)

```
날짜: YYYY. MM. DD. HH:MM:SS

**v0.x.x: 제목**

**테스트 메모**
- 기체: X번 기체
- 테스트 내용
  - 항목 1
  - 항목 2

**변경 사항**
- 변경 내용 1
- 변경 내용 2
```

### Git 명령어

**날짜 지정 커밋/태그** (사용자가 날짜 지정 시):
```bash
# 커밋
GIT_AUTHOR_DATE="YYYY-MM-DDTHH:MM:SS" GIT_COMMITTER_DATE="YYYY-MM-DDTHH:MM:SS" git commit -m "전체 메시지"

# 태그 (커밋 메시지와 동일한 전체 내용)
GIT_COMMITTER_DATE="YYYY-MM-DDTHH:MM:SS" git tag -a v0.x.x -m "전체 메시지"

# 푸시
git push origin main
git push origin v0.x.x
```

### GitHub 태그 히스토리 릴리스 노트 작성 (필수)

**모든 태그는 상세한 릴리스 노트를 포함해야 합니다**:

1. **Annotated Tag 사용**:
   - ❌ Lightweight tag 금지: `git tag v0.x.x`
   - ✅ Annotated tag 필수: `git tag -a v0.x.x -m "메시지"`

2. **릴리스 노트 형식**:
   - ⚠️ **`#` 사용 금지**: Git이 `#`으로 시작하는 줄을 주석으로 처리하여 삭제함
   - 섹션 헤더는 `###` 대신 `**볼드**` 형식 사용
   ```bash
   # 메시지 파일 생성
   cat > /tmp/tag_message.txt <<'EOF'
   날짜: YYYY. MM. DD. HH:MM:SS

   **v0.x.x: 제목**

   **추가된 기능**
   - 기능 1 설명
   - 기능 2 설명

   **변경 사항**
   - 변경 내용 1
   - 변경 내용 2

   **버그 수정**
   - 수정 내용 1
   EOF

   # 태그 생성
   git tag -a v0.x.x -F /tmp/tag_message.txt

   # GitHub에 푸시
   git push origin v0.x.x
   ```

3. **GitHub에서 확인**:
   - 저장소 → Tags → 태그 클릭
   - 릴리스 노트가 표시되어야 함
   - 예시: v0.12.7, v0.12.8 참고

4. **기존 태그 수정** (필요 시):
   ```bash
   # 로컬 태그 삭제
   git tag -d v0.x.x

   # 상세한 메시지로 재생성
   git tag -a v0.x.x -F /tmp/tag_message.txt

   # 강제 푸시
   git push origin v0.x.x --force
   ```

### 버전 히스토리 예시

```
# 개발 단계
v0.1.0 - Phase 1 기본 완료 (Arming, Takeoff, RTL)
v0.2.0 - Waypoint + LiDAR 통합
v0.3.0 - 자동 미션 실행 추가
v0.11.8 - MAVLink Wire Format 정렬 및 메시지 ID 통일
v1.0.0 - 첫 정식 출시!

# 정식 출시 후
v1.1.0 - 성능 개선 및 새 기능
v2.0.0 - 차세대 시스템
```

### 기존 태그 처리

기존 v1.x 태그들 (v1.0-px4-msgs-fix, v1.1-autonomous-phase1 등)은 **히스토리 보존**을 위해 그대로 유지합니다.


## 프로젝트 구조

**프로젝트 루트**: `/home/khadas/humiro_fire_suppression`

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

**프로젝트 경로**: `/home/khadas/humiro_fire_suppression`

### Thermal 시스템
```bash
cd /home/khadas/humiro_fire_suppression/thermal/src
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 출력: ./thermal_rgb_streaming
```

### LiDAR 시스템
```bash
cd /home/khadas/humiro_fire_suppression/lidar
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 출력: ./lidar_test
```

### Navigation 시스템
```bash
cd /home/khadas/humiro_fire_suppression/navigation
./build.sh
# 출력: build/test_arm, build/test_mission 등
```

### Application (통합)
```bash
cd /home/khadas/humiro_fire_suppression/application
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
- ✅ 환경 변수 사용 (`PROJECT_ROOT`, `HOME` 등)
- ✅ 상대 경로 (실행 파일 기준)
- **프로젝트 루트**: `/home/khadas/humiro_fire_suppression`

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
