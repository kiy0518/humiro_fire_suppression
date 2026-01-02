# VIM4 자율 제어 Phase 1 완료

**날짜**: 2026-01-02  
**작성자**: Claude Code Assistant  
**태그**: v1.1-autonomous-phase1

---

## 개요

단일 드론 자율 비행 제어 시스템 Phase 1을 완전히 구현하고 테스트 프로그램까지 완료했습니다.

---

## 구현 완료 항목

### 1. 핸들러 구현 (6개)

#### 1.1 arm_handler (시동 제어)
**파일**: `navigation/src/offboard/autonomous/arm_handler.{h,cpp}`  
**기능**:
- ARM/DISARM 명령 전송
- OFFBOARD 모드 활성화
- Vehicle status 모니터링
- 2Hz OFFBOARD heartbeat

**주요 메서드**:
```cpp
bool arm(int timeout_ms = 5000);
bool disarm(int timeout_ms = 3000);
bool isArmed() const;
bool enableOffboardMode();
```

**빌드 결과**: libarm_handler.a (10MB)

---

#### 1.2 takeoff_handler (이륙 제어)
**파일**: `navigation/src/offboard/autonomous/takeoff_handler.{h,cpp}`  
**기능**:
- 지정 고도로 이륙 (기본 5m)
- NED 좌표계 TrajectorySetpoint 발행
- 고도 도달 확인 (±30cm 허용)
- 호버링 지원

**주요 메서드**:
```cpp
bool takeoff(float altitude_m = 5.0f, int timeout_ms = 30000);
float getCurrentAltitude() const;
bool isTakeoffComplete() const;
void hover();
```

**빌드 결과**: libtakeoff_handler.a (11MB)

---

#### 1.3 waypoint_handler (GPS 이동)
**파일**: `navigation/src/offboard/autonomous/waypoint_handler.{h,cpp}`  
**기능**:
- GPS 좌표로 이동
- Haversine 거리 계산
- GPS → Local NED 변환
- 3D 거리 계산 (수평 + 수직)

**주요 메서드**:
```cpp
bool goToWaypoint(const GPSCoordinate& target, int timeout_ms = 60000);
GPSCoordinate getCurrentPosition() const;
double getDistanceToTarget(const GPSCoordinate& target) const;
bool isWaypointReached() const;
```

**주요 알고리즘**:
- Haversine formula (구면 거리)
- GPS → NED 변환 (위도/경도 차이 → 미터)
- cos(위도) 보정

**빌드 결과**: libwaypoint_handler.a (15MB)

---

#### 1.4 distance_adjuster (거리 조정)
**파일**: `navigation/src/offboard/autonomous/distance_adjuster.{h,cpp}`  
**기능**:
- LiDAR 전방 거리 기반 조정
- 목표 거리 10m ± 1m
- 전진/후진 자동 판단
- 안전 제한 (최대 5m 이동)

**주요 메서드**:
```cpp
bool adjustDistance(float target_distance = 10.0f, 
                   float tolerance = 1.0f,
                   int timeout_ms = 30000);
float getFrontDistance() const;
bool isDistanceAdjusted() const;
void hover();
```

**ROS2 토픽**:
- 구독: `/lidar/front_distance` (std_msgs/Float32)
- 발행: `/fmu/in/trajectory_setpoint`

**빌드 결과**: libdistance_adjuster.a (15MB)

---

#### 1.5 rtl_handler (자동 복귀)
**파일**: `navigation/src/offboard/autonomous/rtl_handler.{h,cpp}`  
**기능**:
- RTL (Return To Launch) 명령
- Home 위치 자동 기록
- 착륙 확인 (STANDBY 상태)
- Emergency land 지원

**주요 메서드**:
```cpp
bool returnToLaunch(int timeout_ms = 120000);
bool land(int timeout_ms = 30000);
double getDistanceFromHome() const;
bool isLanded() const;
bool isRTLActive() const;
```

**PX4 명령**:
- NAV_RETURN_TO_LAUNCH (20)
- NAV_LAND (21)

**빌드 결과**: librtl_handler.a (20MB)

---

#### 1.6 offboard_manager (상태 머신)
**파일**: `navigation/src/offboard/autonomous/offboard_manager.{h,cpp}`  
**기능**:
- 전체 미션 상태 머신
- 9개 상태 관리
- 에러 처리 (Emergency RTL)
- 미션 설정 구조체

**미션 시퀀스**:
```
IDLE → ARMING → TAKEOFF → NAVIGATE → 
ADJUST_DISTANCE → HOVER → RTL → LANDED
```

**에러 발생 시**: 자동 Emergency RTL

**주요 메서드**:
```cpp
bool executeMission(const MissionConfig& config);
void emergencyRTL();
MissionState getCurrentState() const;
static std::string getStateName(MissionState state);
```

**MissionConfig 구조체**:
```cpp
struct MissionConfig {
    float takeoff_altitude;        // 이륙 고도
    GPSCoordinate target_waypoint; // 목표 위치
    float target_distance;         // 목표 거리
    float distance_tolerance;      // 거리 허용 오차
    float hover_duration_sec;      // 호버링 시간
};
```

**빌드 결과**: liboffboard_manager.a (290KB)

---

### 2. 테스트 프로그램 (5개)

#### 2.1 test_arm
**파일**: `navigation/src/offboard/test_arm.cpp`  
**테스트**:
1. OFFBOARD 모드 활성화
2. ARM
3. 5초 대기
4. DISARM

**빌드 결과**: test_arm (3.5MB)

---

#### 2.2 test_takeoff
**파일**: `navigation/src/offboard/test_takeoff.cpp`  
**테스트**:
1. OFFBOARD 모드
2. ARM
3. 5m 이륙
4. 10초 호버링
5. DISARM

**빌드 결과**: test_takeoff (5.4MB)

---

#### 2.3 test_waypoint
**파일**: `navigation/src/offboard/test_waypoint.cpp`  
**테스트**:
1. GPS 신호 대기
2. OFFBOARD + ARM
3. 5m 이륙
4. 북쪽 10m, 동쪽 10m 이동
5. 5초 호버링
6. DISARM

**빌드 결과**: test_waypoint (7.2MB)

---

#### 2.4 test_rtl
**파일**: `navigation/src/offboard/test_rtl.cpp`  
**테스트**:
1. GPS 신호 대기
2. OFFBOARD + ARM
3. 5m 이륙
4. 북쪽 20m, 동쪽 20m 이동
5. 5초 호버링
6. **RTL 실행**
7. 자동 착륙 대기

**빌드 결과**: test_rtl (7.6MB)

---

#### 2.5 test_mission (통합 테스트)
**파일**: `navigation/src/offboard/test_mission.cpp`  
**테스트 시퀀스**:
1. **IDLE**: GPS 신호 대기
2. **ARMING**: OFFBOARD 모드 + ARM
3. **TAKEOFF**: 5m 고도로 이륙
4. **NAVIGATE**: 북쪽 20m, 동쪽 20m 이동
5. **ADJUST_DISTANCE**: LiDAR로 10m ± 1m 조정
6. **HOVER**: 10초 호버링 (발사 준비 단계)
7. **RTL**: 자동 복귀 및 착륙
8. **LANDED**: 미션 완료

**에러 처리**: 각 단계 실패 시 Emergency RTL

**빌드 결과**: test_mission (9.4MB)

---

### 3. 빌드 시스템

**파일**: `navigation/src/offboard/CMakeLists.txt`

**특징**:
- ROS2 환경 확인
- 6개 정적 라이브러리
- 5개 테스트 실행 파일
- px4_msgs 의존성

**빌드 명령**:
```bash
source /opt/ros/humble/setup.bash
source ~/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash
cd ~/humiro_fire_suppression/navigation/src/offboard
mkdir build && cd build
cmake ..
make -j4
```

---

## 빌드 결과

### 라이브러리 (6개, 총 71MB)

```
libarm_handler.a          10MB
libtakeoff_handler.a      11MB
libwaypoint_handler.a     15MB
libdistance_adjuster.a    15MB
librtl_handler.a          20MB
liboffboard_manager.a    290KB
─────────────────────────────
합계                      71MB
```

### 테스트 프로그램 (5개, 총 33MB)

```
test_arm                 3.5MB
test_takeoff             5.4MB
test_waypoint            7.2MB
test_rtl                 7.6MB
test_mission             9.4MB
─────────────────────────────
합계                     33MB
```

---

## 코드 통계

**총 파일 수**: 17개
- 헤더 파일: 6개
- 구현 파일: 6개
- 테스트 파일: 5개

**총 코드 라인**: ~2,260 LOC
- arm_handler: ~250 LOC
- takeoff_handler: ~250 LOC
- waypoint_handler: ~350 LOC
- distance_adjuster: ~280 LOC
- rtl_handler: ~350 LOC
- offboard_manager: ~250 LOC
- 테스트 프로그램: ~530 LOC

---

## 주요 기술

### 1. PX4 통신
- **uXRCE-DDS**: ROS2 ↔ PX4
- **QoS**: BestEffort, TransientLocal
- **토픽**:
  - `/fmu/in/vehicle_command`: 명령 전송
  - `/fmu/in/trajectory_setpoint`: 위치 제어
  - `/fmu/in/offboard_control_mode`: OFFBOARD 모드
  - `/fmu/out/vehicle_status`: 상태 수신
  - `/fmu/out/vehicle_global_position`: GPS
  - `/fmu/out/vehicle_local_position`: Local NED

### 2. 좌표계
- **NED** (North-East-Down): PX4 표준
  - X: 북쪽 (양수 = 전진)
  - Y: 동쪽 (양수 = 우측)
  - Z: 아래 (양수 = 하강)
- **GPS**: WGS84 (위도/경도)
- **변환**: Haversine + cos(위도) 보정

### 3. 에러 처리
- **타임아웃**: 모든 작업에 타임아웃
- **상태 확인**: 실시간 상태 모니터링
- **Emergency RTL**: 에러 발생 시 자동 복귀
- **진행 상황 로깅**: 주기적 정보 출력

---

## 다음 단계 (Phase 2-4)

### Phase 2: 편대 통신 (3일)

**파일**: `navigation/src/offboard/communication/`

**구현 항목**:
- [ ] FormationMember 클래스
  - ROS2 Publisher: `/formation/member_status`
  - ROS2 Subscriber: `/formation/target_assignment`
  - 상태 발행 (위치, 배터리, 상태)
  - 목표 수신

**데이터 구조체**:
```cpp
struct MemberStatus {
    uint8_t drone_id;
    GPSCoordinate position;
    float battery_percent;
    MissionState state;
    uint64_t timestamp;
};

struct TargetAssignment {
    uint8_t drone_id;
    GPSCoordinate target;
    uint8_t fire_point_id;
    uint64_t timestamp;
};
```

---

### Phase 3: 리더 조율 (4일)

**파일**: `navigation/src/offboard/formation/`

**구현 항목**:
- [ ] FormationLeader 클래스
  - 화재 지점 분석
  - 목표 할당 알고리즘
  - 진행 상황 모니터링
  - 장애 대응

**목표 할당 알고리즘**:
1. 화재 지점 목록 수신 (열화상)
2. 사용 가능한 드론 확인
3. 최적 매칭 (거리 기반)
4. 목표 할당 발행
5. 진행 상황 모니터링

---

### Phase 4: 통합 테스트 (3일)

**테스트 시나리오**:
- [ ] 단일 드론: 전체 미션 (완료 ✅)
- [ ] 2대 편대: 2개 화재 지점
- [ ] 3대 편대: 3개 화재 지점
- [ ] 장애 시나리오: 드론 1대 실패

---

### Phase 5: 발사 메커니즘 (5일)

**파일**: `throwing_mechanism/src/`

**구현 항목**:
- [ ] GPIO 제어 (6개 핀)
- [ ] 발사 트리거
- [ ] 재조준 로직
- [ ] 발사 관리자

**발사 시퀀스**:
1. 타겟팅 완료 확인
2. 첫 번째 GPIO 활성화 (발사)
3. 재조준 (2초 대기)
4. 두 번째 GPIO 활성화
5. 반복 (6발)
6. 완료 후 자동 RTL

---

## 하이브리드 편대 제어 설계

### 아키텍처

```
리더 드론:
├── FormationLeader (화재 분석 + 목표 할당)
├── FormationMember (자신의 목표 실행)
└── OffboardManager (자율 비행)

팔로워 드론:
├── FormationMember (목표 수신 + 실행)
└── OffboardManager (자율 비행)
```

### 장점
- **안정성**: 각 드론이 독립 실행
- **효율성**: 리더가 최적 할당
- **확장성**: 드론 수 증가 가능
- **장애 대응**: 리더 실패 시 재선출

---

## 실행 방법

### 1. 환경 설정

```bash
source /opt/ros/humble/setup.bash
source ~/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/humiro_fire_suppression/config/fastrtps_profile.xml
```

### 2. 테스트 실행

```bash
cd ~/humiro_fire_suppression/navigation/src/offboard/build

# 시동 테스트
./test_arm

# 이륙 테스트
./test_takeoff

# 웨이포인트 테스트
./test_waypoint

# RTL 테스트
./test_rtl

# 전체 미션 테스트
./test_mission
```

---

## 문제 해결

### px4_msgs 버전 확인

```bash
cd ~/humiro_fire_suppression/workspaces/px4_ros2_ws/src/px4_msgs
git branch
# release/1.16 이어야 함
```

### QoS 설정 확인

```bash
cat ~/humiro_fire_suppression/config/fastrtps_profile.xml
# BestEffort, TransientLocal 확인
```

### 빌드 에러

```bash
# ROS2 환경 재설정
source /opt/ros/humble/setup.bash
source ~/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash

# 클린 빌드
cd ~/humiro_fire_suppression/navigation/src/offboard
rm -rf build
mkdir build && cd build
cmake .. && make -j4
```

---

## 결론

VIM4 자율 제어 Phase 1을 완전히 구현하고 테스트했습니다.

**완료 항목**:
- ✅ 6개 핸들러 (ARM, Takeoff, Waypoint, Distance, RTL, Manager)
- ✅ 5개 테스트 프로그램
- ✅ 완전한 미션 시퀀스
- ✅ 에러 처리 (Emergency RTL)
- ✅ 2,260 LOC

**다음 단계**:
- Phase 2: 편대 통신 (3일)
- Phase 3: 리더 조율 (4일)
- Phase 4: 통합 테스트 (3일)
- Phase 5: 발사 메커니즘 (5일)

**예상 완료**: 2-3주 후

---

**작성자**: Claude Code Assistant  
**날짜**: 2026-01-02  
**Git Tag**: v1.1-autonomous-phase1

