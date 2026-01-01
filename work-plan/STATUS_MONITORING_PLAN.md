# 기체 상태 모니터링 OSD 개발 계획 (Status Monitoring OSD Development Plan)

작성일: 2026-01-01  
**상태**: 계획 단계

---

## 개요

영상 왼쪽 하단에 기체 상태 및 정보를 실시간으로 표시하는 OSD 시스템입니다.

**중요**: 이 기능은 **모든 비행 모드**에서 동작합니다:
- **MANUAL**: 수동 모드
- **STABILIZED**: 안정화 모드
- **POSCTL**: 위치 제어 모드
- **AUTO_MISSION**: PX4 Mission 모드 (QGC에서 미션 업로드 후 PX4가 자동 실행)
- **AUTO_LOITER**: 자동 대기 모드
- **AUTO_RTL**: 자동 복귀 모드
- **AUTO_LAND**: 자동 착륙 모드
- **AUTO_TAKEOFF**: 자동 이륙 모드
- **OFFBOARD**: 오프보드 모드 (VIM4에서 ROS2로 제어하는 자동 제어 시스템)

**표시 위치**: 영상 왼쪽 하단  
**표시 주기**: 30Hz (33ms 간격, 영상 프레임과 동기화)

---

## 표시 항목

### 1. 기체 상태 (Drone Status)

**상태 목록** (PX4 비행 모드 및 커스텀 상태):
1. **대기** (IDLE) - 초기 상태, 명령 대기 (DISARMED 상태)
2. **시동** (ARMING) - 시동 중 (ARMING 상태)
3. **이륙** (TAKEOFF) - 이륙 중 (AUTO_TAKEOFF 모드)
4. **이동중** (NAVIGATING) - 목적지로 이동 중
   - AUTO_MISSION 모드 (QGC Mission 실행 중)
   - OFFBOARD 모드 (자동모드 waypoint 이동 중)
5. **목적지도착** (DESTINATION_REACHED) - 목적지 도착
   - AUTO_MISSION 모드에서 waypoint 도착
   - OFFBOARD 모드에서 목표 위치 도착
6. **격발대기** (FIRE_READY) - 격발 준비 완료, 대기 중
   - 목적지 도착 후 격발 명령 대기
7. **격발중(자동조준)** (FIRING_AUTO_TARGETING) - 자동 타겟팅 및 격발 중
   - OFFBOARD 모드에서 자동 타겟팅 활성화
8. **임무완료** (MISSION_COMPLETE) - 임무 완료
   - AUTO_MISSION 모드에서 모든 waypoint 완료
9. **복귀중** (RETURNING) - 복귀 경로 이동 중
   - AUTO_RTL 모드
   - AUTO_MISSION 모드에서 RTL waypoint 실행
10. **착륙** (LANDING) - 착륙 중
    - AUTO_LAND 모드
    - AUTO_RTL 모드에서 착륙 단계
11. **시동끔** (DISARMED) - 시동 해제 완료

**PX4 비행 모드 매핑**:
- `MAV_MODE_MANUAL_DISARMED` → 대기/시동끔
- `MAV_MODE_MANUAL_ARMED` → 수동 비행 (MANUAL, STABILIZED, POSCTL)
- `MAV_MODE_AUTO_ARMED` + `AUTO_TAKEOFF` → 이륙
- `MAV_MODE_AUTO_ARMED` + `AUTO_MISSION` → 이동중/목적지도착/임무완료
- `MAV_MODE_AUTO_ARMED` + `AUTO_RTL` → 복귀중/착륙
- `MAV_MODE_AUTO_ARMED` + `AUTO_LAND` → 착륙
- `MAV_MODE_AUTO_ARMED` + `AUTO_LOITER` → 대기 (호버링)
- `MAV_MODE_AUTO_ARMED` + `OFFBOARD` → 이동중/격발대기/격발중 (자동모드)

**표시 형식**:
- 텍스트: 상태 이름 (한글 또는 영문)
- 색상 코딩:
  - 대기: 회색 (128, 128, 128)
  - 시동/이륙/착륙: 노란색 (0, 255, 255)
  - 이동중/복귀중: 파란색 (255, 0, 0)
  - 목적지도착/격발대기: 초록색 (0, 255, 0)
  - 격발중: 빨간색 (0, 0, 255)
  - 임무완료: 하늘색 (255, 255, 0)
  - 시동끔: 어두운 회색 (64, 64, 64)

### 2. 소화탄 갯수 (Ammunition Count)

**표시 형식**:
- 텍스트: "소화탄: X/Y" (X = 현재 갯수, Y = 최대 갯수)
- 예: "소화탄: 3/5"
- 색상:
  - 정상 (X > 1): 흰색 (255, 255, 255)
  - 경고 (X = 1): 노란색 (0, 255, 255)
  - 위험 (X = 0): 빨간색 (0, 0, 255)

### 3. 기체 이름 (Drone Name)

**표시 형식**:
- 텍스트: 기체 식별자 (예: "Drone-01", "Alpha-1")
- 색상: 흰색 (255, 255, 255)
- 폰트: 볼드체

### 4. 편대 정보 (Formation Info)

**표시 형식**:
- 텍스트: "편대: X/Y" (X = 현재 편대 번호, Y = 전체 편대 수)
- 예: "편대: 1/3" (3개 편대 중 1번째)
- 색상: 청록색 (255, 255, 0)

### 5. 추가 정보 (Optional)

**배터리 상태** (선택적):
- 텍스트: "배터리: XX%" 또는 "Battery: XX%"
- 색상:
  - 정상 (>50%): 초록색 (0, 255, 0)
  - 경고 (20-50%): 노란색 (0, 255, 255)
  - 위험 (<20%): 빨간색 (0, 0, 255)

**GPS 상태** (선택적):
- 텍스트: "GPS: XX 위성" 또는 "GPS: XX sats"
- 색상:
  - 정상 (≥6): 초록색 (0, 255, 0)
  - 경고 (3-5): 노란색 (0, 255, 255)
  - 위험 (<3): 빨간색 (0, 0, 255)

---

## 레이아웃 설계

```
┌─────────────────────────────────┐
│                                 │
│         영상 영역                │
│                                 │
│                                 │
│  ┌─────────────────────────┐   │
│  │ 기체: Drone-01           │   │  ← 왼쪽 하단
│  │ 상태: 이동중             │   │
│  │ 소화탄: 3/5              │   │
│  │ 편대: 1/3                │   │
│  │ 배터리: 85%              │   │
│  │ GPS: 12 위성             │   │
│  └─────────────────────────┘   │
└─────────────────────────────────┘
```

**위치 계산**:
- 시작 X: `MARGIN_LEFT` (예: 20 픽셀)
- 시작 Y: `frame.rows - MARGIN_BOTTOM - total_height`
- 배경: 반투명 검은색 또는 진회색
- 테두리: 얇은 흰색 선 (선택적)

---

## 구현 계획

### Phase 1: 기본 구조 (3일)

#### 1.1 상태 오버레이 클래스 생성
- [ ] `osd/src/status/status_overlay.h` 생성
- [ ] `osd/src/status/status_overlay.cpp` 생성
- [ ] 기본 클래스 구조 정의

**클래스 구조**:
```cpp
class StatusOverlay {
public:
    enum class DroneStatus {
        IDLE,                    // 대기
        ARMING,                  // 시동
        TAKEOFF,                 // 이륙
        NAVIGATING,              // 이동중
        DESTINATION_REACHED,     // 목적지도착
        FIRE_READY,              // 격발대기
        FIRING_AUTO_TARGETING,   // 격발중(자동조준)
        MISSION_COMPLETE,        // 임무완료
        RETURNING,               // 복귀중
        LANDING,                 // 착륙
        DISARMED                 // 시동끔
    };
    
    // PX4 상태 업데이트 (모든 모드에서 사용)
    void updatePx4State(const std::string& px4_mode, bool is_armed);
    
    // VIM4 자동 제어 시스템 상태 업데이트 (OFFBOARD 모드일 때만 사용)
    void updateAutoControlStatus(DroneStatus status);
    
    // 기타 정보 업데이트
    void setAmmunition(int current, int max);
    void setDroneName(const std::string& name);
    void setFormation(int current, int total);
    void setBattery(int percentage);
    void setGpsSatellites(int count);
    
    // 그리기
    void draw(cv::Mat& frame);
    
private:
    DroneStatus current_status_;
    std::string px4_mode_;       // PX4 비행 모드
    bool is_armed_;              // 시동 상태
    bool is_offboard_;           // OFFBOARD 모드 여부
    
    int ammo_current_, ammo_max_;
    std::string drone_name_;
    int formation_current_, formation_total_;
    int battery_percentage_;
    int gps_satellites_;
    
    // PX4 모드를 커스텀 상태로 변환
    DroneStatus convertPx4ModeToStatus(const std::string& px4_mode, bool is_armed);
    
    cv::Scalar getStatusColor(DroneStatus status);
    std::string getStatusText(DroneStatus status);
    void drawBackground(cv::Mat& frame, int x, int y, int width, int height);
};
```

#### 1.2 기본 그리기 구현
- [ ] 상태 텍스트 그리기
- [ ] 색상 코딩 적용
- [ ] 배경 및 테두리 그리기
- [ ] 위치 계산 로직

### Phase 2: 데이터 연동 (2일)

#### 2.1 PX4 FC 상태 정보 수신 (우선)
- [ ] MAVLink `HEARTBEAT` 메시지 구독
  - 비행 모드 (`custom_mode`, `base_mode`)
  - 시동 상태 (`armed`)
- [ ] ROS2 토픽 `/mavros/state` 구독
  - `mode`: 현재 비행 모드 문자열
  - `armed`: 시동 상태
  - `connected`: FC 연결 상태
- [ ] PX4 비행 모드를 커스텀 상태로 변환
  - `AUTO_MISSION` → 이동중/목적지도착/임무완료
  - `AUTO_RTL` → 복귀중/착륙
  - `AUTO_LAND` → 착륙
  - `OFFBOARD` → 자동모드 상태 확인 필요

#### 2.2 VIM4 자동 제어 시스템 상태 연동 (OFFBOARD 모드일 때)
- [ ] `/auto_mode/status` 토픽 구독 (OFFBOARD 모드에서만 활성화)
- [ ] VIM4 자동 제어 시스템 상태 메시지 파싱
- [ ] VIM4 자동 제어 시스템 상태와 PX4 모드 조합하여 최종 상태 결정

**ROS2 메시지 정의** (필요 시):
```cpp
// custom_msgs/msg/DroneStatus.msg
uint8 status          # DroneStatus enum 값
string drone_name
int32 ammo_current
int32 ammo_max
int32 formation_current
int32 formation_total
int32 battery_percentage
int32 gps_satellites
string px4_mode       # PX4 비행 모드 (예: "AUTO_MISSION", "OFFBOARD")
bool is_armed         # 시동 상태
```

#### 2.3 상태 업데이트 로직
- [ ] PX4 상태 우선 적용 (모든 모드에서)
- [ ] OFFBOARD 모드일 때만 자동모드 상태 추가 확인
- [ ] 상태 변경 시 자동 업데이트
- [ ] 상태 전이 애니메이션 (선택적)

### Phase 3: 통합 및 최적화 (2일)

#### 3.1 프레임 컴포지터 통합
- [ ] `targeting/src/targeting_frame_compositor.cpp`에 통합
- [ ] 다른 오버레이와의 충돌 방지
- [ ] 렌더링 순서 조정

#### 3.2 성능 최적화
- [ ] 텍스트 렌더링 캐싱
- [ ] 배경 그리기 최적화
- [ ] 불필요한 재계산 방지

#### 3.3 시각적 개선
- [ ] 폰트 크기 조정
- [ ] 간격 및 여백 최적화
- [ ] 아이콘 추가 (선택적)

---

## 파일 구조

```
osd/
├── src/
│   ├── status/
│   │   ├── status_overlay.h
│   │   └── status_overlay.cpp
│   ├── lidar/
│   │   ├── distance_overlay.h
│   │   └── distance_overlay.cpp
│   ├── thermal/
│   │   ├── thermal_overlay.h
│   │   └── thermal_overlay.cpp
│   └── targeting/
│       └── ...
├── include/
│   └── status/
│       └── status_overlay.h
└── CMakeLists.txt
```

---

## 데이터 소스

### 1. 상태 정보 (PX4 FC - 모든 모드)
- **소스**: PX4 Flight Controller (MAVLink)
- **전달 방식**: 
  - ROS2 토픽 `/mavros/state` (주)
  - MAVLink `HEARTBEAT` 메시지 (보조)
- **업데이트 주기**: 10Hz (100ms 간격)
- **비행 모드 감지**: 
  - `AUTO_MISSION`: QGC Mission 모드
  - `AUTO_RTL`: 자동 복귀 모드
  - `AUTO_LAND`: 자동 착륙 모드
  - `AUTO_LOITER`: 자동 대기 모드
  - `OFFBOARD`: 오프보드 모드 (자동모드 포함)
  - `MANUAL`, `STABILIZED`, `POSCTL`: 수동 모드

### 1-1. 상태 정보 (VIM4 자동 제어 시스템 - OFFBOARD 모드일 때만)
- **소스**: `navigation/src/auto_mode/state_machine.cpp`
- **전달 방식**: ROS2 토픽 `/auto_mode/status`
- **업데이트 주기**: 상태 변경 시 즉시 + 주기적 (1Hz)
- **활성 조건**: PX4 모드가 `OFFBOARD`일 때만 사용
- **참고**: PX4 Mission 모드(`AUTO_MISSION`)와는 별개의 시스템

### 2. 소화탄 갯수
- **소스**: `throwing_mechanism/src/fire_controller.cpp`
- **전달 방식**: ROS2 토픽 `/ammunition/count` 또는 상태 메시지에 포함
- **업데이트 주기**: 격발 시 즉시 업데이트

### 3. 기체 이름
- **소스**: 설정 파일 또는 ROS2 파라미터
- **전달 방식**: ROS2 파라미터 `/drone/name` 또는 상태 메시지에 포함
- **업데이트 주기**: 시작 시 1회

### 4. 편대 정보
- **소스**: 설정 파일 또는 ROS2 파라미터
- **전달 방식**: ROS2 파라미터 `/formation/current`, `/formation/total`
- **업데이트 주기**: 시작 시 1회 또는 동적 변경 시

### 5. 배터리 상태
- **소스**: PX4 FC (MAVLink `BATTERY_STATUS`)
- **전달 방식**: ROS2 토픽 `/mavros/battery` 또는 상태 메시지에 포함
- **업데이트 주기**: 1Hz

### 6. GPS 상태
- **소스**: PX4 FC (MAVLink `GPS_RAW_INT`)
- **전달 방식**: ROS2 토픽 `/mavros/gpsstatus` 또는 상태 메시지에 포함
- **업데이트 주기**: 1Hz

---

## 시각적 스타일

### 배경
- **색상**: 반투명 검은색 (BGR: 0, 0, 0, alpha: 180/255)
- **모서리**: 둥근 모서리 (반경: 5 픽셀)
- **테두리**: 얇은 흰색 선 (1 픽셀, 선택적)

### 텍스트
- **폰트**: `cv::FONT_HERSHEY_SIMPLEX`
- **크기**: 0.6 (기본), 0.8 (기체 이름)
- **두께**: 1 (기본), 2 (기체 이름)
- **안티앨리어싱**: `cv::LINE_AA` 사용

### 레이아웃
- **행 간격**: 5 픽셀
- **좌측 여백**: 20 픽셀
- **하단 여백**: 20 픽셀
- **항목 간 간격**: 3 픽셀

---

## 구현 예시 코드

```cpp
// status_overlay.cpp
void StatusOverlay::draw(cv::Mat& frame) {
    const int MARGIN_LEFT = 20;
    const int MARGIN_BOTTOM = 20;
    const int LINE_HEIGHT = 25;
    const int PADDING = 10;
    
    // 텍스트 크기 계산
    int baseline = 0;
    cv::Size text_size = cv::getTextSize("기체: Drone-01", 
                                         cv::FONT_HERSHEY_SIMPLEX, 
                                         0.6, 1, &baseline);
    
    int box_width = text_size.width + PADDING * 2;
    int box_height = LINE_HEIGHT * 6 + PADDING * 2;  // 6개 항목
    
    int x = MARGIN_LEFT;
    int y = frame.rows - MARGIN_BOTTOM - box_height;
    
    // 배경 그리기 (둥근 모서리)
    drawBackground(frame, x, y, box_width, box_height);
    
    // 각 항목 그리기
    int line_y = y + PADDING + LINE_HEIGHT;
    
    // 1. 기체 이름
    cv::putText(frame, "기체: " + drone_name_,
                cv::Point(x + PADDING, line_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    line_y += LINE_HEIGHT;
    
    // 2. 상태
    cv::Scalar status_color = getStatusColor(current_status_);
    std::string status_text = "상태: " + getStatusText(current_status_);
    cv::putText(frame, status_text,
                cv::Point(x + PADDING, line_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, status_color, 1, cv::LINE_AA);
    line_y += LINE_HEIGHT;
    
    // 3. 소화탄
    std::ostringstream ammo_oss;
    ammo_oss << "소화탄: " << ammo_current_ << "/" << ammo_max_;
    cv::Scalar ammo_color = (ammo_current_ > 1) ? cv::Scalar(255, 255, 255) :
                           (ammo_current_ == 1) ? cv::Scalar(0, 255, 255) :
                           cv::Scalar(0, 0, 255);
    cv::putText(frame, ammo_oss.str(),
                cv::Point(x + PADDING, line_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, ammo_color, 1, cv::LINE_AA);
    line_y += LINE_HEIGHT;
    
    // 4. 편대
    std::ostringstream formation_oss;
    formation_oss << "편대: " << formation_current_ << "/" << formation_total_;
    cv::putText(frame, formation_oss.str(),
                cv::Point(x + PADDING, line_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    line_y += LINE_HEIGHT;
    
    // 5. 배터리 (선택적)
    if (battery_percentage_ >= 0) {
        std::ostringstream battery_oss;
        battery_oss << "배터리: " << battery_percentage_ << "%";
        cv::Scalar battery_color = (battery_percentage_ > 50) ? cv::Scalar(0, 255, 0) :
                                  (battery_percentage_ > 20) ? cv::Scalar(0, 255, 255) :
                                  cv::Scalar(0, 0, 255);
        cv::putText(frame, battery_oss.str(),
                    cv::Point(x + PADDING, line_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, battery_color, 1, cv::LINE_AA);
        line_y += LINE_HEIGHT;
    }
    
    // 6. GPS (선택적)
    if (gps_satellites_ >= 0) {
        std::ostringstream gps_oss;
        gps_oss << "GPS: " << gps_satellites_ << " 위성";
        cv::Scalar gps_color = (gps_satellites_ >= 6) ? cv::Scalar(0, 255, 0) :
                              (gps_satellites_ >= 3) ? cv::Scalar(0, 255, 255) :
                              cv::Scalar(0, 0, 255);
        cv::putText(frame, gps_oss.str(),
                    cv::Point(x + PADDING, line_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, gps_color, 1, cv::LINE_AA);
    }
}
```

---

## 통합 순서

1. **OSD 모듈에 추가**: `osd/src/status/` 디렉토리 생성 및 구현
2. **프레임 컴포지터 통합**: `targeting/src/targeting_frame_compositor.cpp`에 `StatusOverlay` 추가
3. **PX4 상태 수신**: `/mavros/state` 토픽 구독 및 비행 모드 파싱 (모든 모드 지원)
4. **VIM4 자동 제어 시스템 상태 연동** (선택적): `navigation/src/auto_mode/state_machine.cpp`에서 상태 변경 시 토픽 발행 (OFFBOARD 모드일 때만)
5. **데이터 소스 연동**: 각 데이터 소스에서 ROS2 토픽 발행 또는 파라미터 설정
6. **테스트**: 
   - 각 비행 모드별 표시 확인 (MANUAL, AUTO_MISSION, AUTO_RTL, OFFBOARD 등)
   - QGC Mission 모드에서 상태 표시 확인
   - 자동모드에서 상태 표시 확인

---

## 우선순위

### 높음 (즉시 시작)
1. 기본 상태 표시 (대기, 시동, 이륙, 이동중, 복귀중, 착륙, 시동끔)
2. 기체 이름 표시
3. 소화탄 갯수 표시

### 중간 (1주 내)
4. 목적지도착, 격발대기, 격발중, 임무완료 상태 추가
5. 편대 정보 표시
6. 배터리 상태 표시

### 낮음 (2주 내)
7. GPS 상태 표시
8. 시각적 개선 (아이콘, 애니메이션 등)

---

## 참고사항

### PX4 비행 모드와 상태 매핑

| PX4 모드 | 상태 표시 | 설명 |
|---------|----------|------|
| `MANUAL` | 수동 비행 | 수동 조종 모드 |
| `STABILIZED` | 수동 비행 | 안정화 모드 |
| `POSCTL` | 수동 비행 | 위치 제어 모드 |
| `AUTO_MISSION` | 이동중/목적지도착/임무완료 | QGC Mission 모드 |
| `AUTO_RTL` | 복귀중/착륙 | 자동 복귀 모드 |
| `AUTO_LAND` | 착륙 | 자동 착륙 모드 |
| `AUTO_LOITER` | 대기 | 자동 대기 모드 |
| `AUTO_TAKEOFF` | 이륙 | 자동 이륙 모드 |
| `OFFBOARD` | 자동모드 상태 확인 | 오프보드 모드 (자동모드 포함) |

### PX4 Mission 모드 동작 (AUTO_MISSION)

**참고**: ArduPilot의 AUTO 모드와 유사하지만, PX4에서는 `AUTO_MISSION` 모드입니다.

QGC에서 Mission을 업로드하고 실행하면:
1. PX4가 `AUTO_MISSION` 모드로 전환
2. `/mavros/state` 토픽의 `mode` 필드가 `"AUTO.MISSION"`으로 변경
3. StatusOverlay가 이를 감지하여 "이동중" 상태로 표시
4. Waypoint 도착 시 "목적지도착" 상태로 변경 (MAVLink `MISSION_ITEM_REACHED` 메시지 활용)
5. 모든 Waypoint 완료 시 "임무완료" 상태로 변경

### VIM4 자동 제어 시스템 동작 (OFFBOARD)

VIM4에서 ROS2를 통해 드론을 제어할 때:
1. PX4가 `OFFBOARD` 모드로 전환
2. `/mavros/state` 토픽의 `mode` 필드가 `"OFFBOARD"`로 변경
3. StatusOverlay가 PX4 모드를 감지하고, 추가로 `/auto_mode/status` 토픽을 구독
4. VIM4 자동 제어 시스템의 상태(시동, 이륙, 이동중, 격발대기 등)를 표시
5. PX4 Mission 모드와는 별개로 동작하는 커스텀 자동 제어 시스템

---

**작성자**: Claude Code Assistant  
**버전**: v1.0  
**작성일**: 2025-01-01  
**다음 리뷰**: Phase 1 완료 시

