# 024-01 스웜 화재진압 드론 Offboard 시퀀스 V2

## 문서 정보
- **작업 번호**: 024-01
- **상위 문서**: 024 Offboard 모드 시퀀스 버전2
- **작성일**: 2025-01-16
- **작성자**: 휴미로㈜

---

## 1. 개요

### 1.1 목적
QGC에서 커스텀 메시지 수신 시 3대의 드론(리더 1대, 팔로워 2대)이 협조 비행하여 화재 목표물에 소화탄을 발사하고 복귀하는 Offboard 제어 시퀀스 구현

### 1.2 시스템 구성
```
┌─────────────────────────────────────────────────────────────┐
│                        QGC (지상국)                          │
│         커스텀 메시지 발행: 미션 시작 트리거                    │
└─────────────────┬───────────────────────────────────────────┘
                  │ MAVLink
    ┌─────────────┼─────────────┐
    │             │             │
    ▼             ▼             ▼
┌───────┐    ┌───────┐    ┌───────┐
│ 1번   │    │ 2번   │    │ 3번   │
│ 리더  │◄───│팔로워 │    │팔로워 │
│       │    │ 좌측  │    │ 우측  │
└───┬───┘    └───┬───┘    └───┬───┘
    │            │            │
    └────────────┴────────────┘
              ROS2 토픽 통신
```

### 1.3 기체 번호 확인
```bash
# 각 기체의 번호는 아래 파일에서 확인
cat /home/khadas/config/device_config.env
# VEHICLE_ID=1  (리더)
# VEHICLE_ID=2  (좌측 팔로워)
# VEHICLE_ID=3  (우측 팔로워)
```

---

## 2. 포메이션 구조

### 2.1 정삼각형 포메이션
```
              [목표물]
             /   |   \
           /     |     \
         10m    10m    10m
        /        |        \
 [2번 좌측]      |       [3번 우측]
        \        |        /
         10m    10m    10m
           \     |     /
            [1번 리더]

각도 관계 (목표물 기준):
- 1번 리더: 0° (정면)
- 2번 팔로워: +60° (좌측)
- 3번 팔로워: -60° (우측)

모든 변의 길이: 10m (정삼각형)
```

### 2.2 이동 중 편대 구조
```
비행 방향 →

    [2번 좌측]
         \
          \  10m
           \
            [1번 리더] ─────→ 비행 방향
           /
          /  10m
         /
    [3번 우측]

- 2번: 리더 기준 좌측 후방 45° 위치, 거리 10m
- 3번: 리더 기준 우측 후방 45° 위치, 거리 10m
```

---

## 3. 시퀀스 다이어그램

### 3.1 전체 흐름
```
┌──────────────────┐
│   QGC 커스텀     │
│   메시지 수신    │
└────────┬─────────┘
         │
         ▼
┌────────────────────────────────────────────────────────────────┐
│                        1번 리더 기체                            │
├────────────────────────────────────────────────────────────────┤
│ 1. 시동 (Arming)                                               │
│    └─ 상태 토픽 발행: /leader/status {phase: "ARMING"}         │
│                                                                 │
│ 2. 이륙 (Takeoff)                                              │
│    └─ 상태 토픽 발행: /leader/status {phase: "TAKEOFF"}        │
│                                                                 │
│ 3. 웨이포인트 이동                                              │
│    └─ 상태 토픽 발행: /leader/status {phase: "WAYPOINT"}       │
│    └─ 위치/헤딩 발행: /leader/pose {lat, lon, alt, yaw}        │
│                                                                 │
│ 4. 웨이포인트 도착                                              │
│    └─ 상태 토픽 발행: /leader/status {phase: "ARRIVED"}        │
│                                                                 │
│ 5. 목표물 거리 조정 (LiDAR 10m)                                 │
│    └─ 상태 토픽 발행: /leader/status {phase: "DISTANCE_ADJ"}   │
│                                                                 │
│ 6. 화재진압중 토픽 발행 ★                                      │
│    └─ 상태 토픽 발행: /leader/status {phase: "FIRE_FIGHTING"}  │
│    └─ 조준 위치 발행: /leader/aim_pose {lat, lon, alt, yaw,    │
│                                         target_lat, target_lon}│
│                                                                 │
│ 7. 조준 (열화상 카메라)                                         │
│    └─ 상태 토픽 발행: /leader/status {phase: "AIMING"}         │
│                                                                 │
│ 8. 격발 (6발)                                                   │
│    └─ 상태 토픽 발행: /leader/status {phase: "FIRING",         │
│                                        shot: 1~6}              │
│                                                                 │
│ 9. 복귀 (RTL)                                                   │
│    └─ 상태 토픽 발행: /leader/status {phase: "RETURNING"}      │
│                                                                 │
│ 10. 착륙                                                        │
│    └─ 상태 토픽 발행: /leader/status {phase: "LANDING"}        │
│                                                                 │
│ 11. 시동 해제 (Disarming)                                       │
│    └─ 상태 토픽 발행: /leader/status {phase: "DISARMED"}       │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│                   2번, 3번 팔로워 기체                          │
├────────────────────────────────────────────────────────────────┤
│ 1. 리더 상태 토픽 구독                                          │
│    └─ 구독: /leader/status, /leader/pose, /leader/aim_pose     │
│                                                                 │
│ 2. 리더 시동 확인 (phase: "ARMING")                            │
│    └─ 확인 후 → 자신도 시동                                     │
│                                                                 │
│ 3. 리더 이륙 확인 (phase: "TAKEOFF")                           │
│    └─ 확인 후 → 자신도 이륙                                     │
│                                                                 │
│ 4. 리더 웨이포인트 이동 확인 (phase: "WAYPOINT")               │
│    └─ /leader/pose 구독하여 리더 GPS/헤딩 확인                 │
│    └─ 자신의 편대 위치 계산 (2번: 좌측 후방, 3번: 우측 후방)   │
│    └─ 편대 유지하며 이동                                        │
│                                                                 │
│ 5. 리더 도착 확인 (phase: "ARRIVED")                           │
│    └─ /leader/aim_pose 기반 발사 지점 계산                     │
│    └─ 목표물 기준 정삼각형 위치로 이동                          │
│    └─ 2번: 목표물 기준 +60° 위치                               │
│    └─ 3번: 목표물 기준 -60° 위치                               │
│                                                                 │
│ 6. 리더 화재진압중 확인 (phase: "FIRE_FIGHTING") ★             │
│    └─ 확인 후 → LiDAR 거리 조정 (옵션)                         │
│    └─ 열화상 조준 시작                                          │
│                                                                 │
│ 7. 조준 완료                                                    │
│    └─ 정조준 자세 저장                                          │
│                                                                 │
│ 8. 격발 (6발)                                                   │
│    └─ 리더와 동일 시퀀스 (5초 간격)                            │
│                                                                 │
│ 9. 복귀 (RTL)                                                   │
│    └─ 각자 이륙 지점으로 복귀                                   │
│                                                                 │
│ 10. 착륙                                                        │
│                                                                 │
│ 11. 시동 해제 (Disarming)                                       │
└────────────────────────────────────────────────────────────────┘
```

---

## 4. 상세 시퀀스 - 1번 리더 기체

### 4.1 Phase 1: 시동 (Arming)
```cpp
// 트리거: QGC 커스텀 메시지 수신
void onCustomMessageReceived() {
    // 1. 현재 위치 저장 (복귀 지점)
    home_position_ = getCurrentGpsPosition();
    
    // 2. 상태 토픽 발행
    publishStatus("ARMING");
    
    // 3. 시동 명령
    sendArmCommand();
    
    // 4. 시동 완료 대기
    waitUntilArmed();
}
```

### 4.2 Phase 2: 이륙 (Takeoff)
```cpp
void takeoff() {
    // 1. 상태 토픽 발행
    publishStatus("TAKEOFF");
    
    // 2. 이륙 명령 (고도 10m)
    sendTakeoffCommand(10.0f);
    
    // 3. 이륙 완료 대기
    waitUntilAltitudeReached(10.0f);
}
```

### 4.3 Phase 3: 웨이포인트 이동
```cpp
void flyToWaypoint() {
    // 1. 상태 토픽 발행
    publishStatus("WAYPOINT");
    
    // 2. 실시간 위치/헤딩 발행 시작 (10Hz)
    startPublishingPose();  // /leader/pose 토픽
    
    // 3. QGC에서 받은 웨이포인트로 이동
    flyToPosition(waypoint_lat_, waypoint_lon_, waypoint_alt_);
    
    // 4. 도착 대기
    waitUntilPositionReached();
}
```

### 4.4 Phase 4: 웨이포인트 도착
```cpp
void onWaypointArrived() {
    // 1. 상태 토픽 발행
    publishStatus("ARRIVED");
    
    // 2. 호버링
    holdPosition();
}
```

### 4.5 Phase 5: 목표물 거리 조정
```cpp
void adjustDistance() {
    // 1. 상태 토픽 발행
    publishStatus("DISTANCE_ADJ");
    
    // 2. LiDAR 기반 거리 제어 (목표: 10m ± 5%)
    while (!isDistanceStable()) {
        float distance = getLidarDistance();
        
        if (distance > 10.5f) {
            moveForward(0.4f);  // 전진
        } else if (distance < 9.5f) {
            moveBackward(0.25f);  // 천천히 후진 (회피 기동 금지!)
        } else {
            holdPosition();
        }
    }
    
    // 3. 3초간 안정화 확인
    waitForStabilization(3.0f);
}
```

### 4.6 Phase 6: 화재진압중 토픽 발행 ★ (핵심)
```cpp
void startFireFighting() {
    // 1. 상태 토픽 발행 - 팔로워들이 이 신호를 기다림
    publishStatus("FIRE_FIGHTING");
    
    // 2. 목표물 위치 계산
    target_lat_ = calculateTargetLat(current_lat_, current_yaw_, lidar_distance_);
    target_lon_ = calculateTargetLon(current_lon_, current_yaw_, lidar_distance_);
    
    // 3. 조준 위치 토픽 발행 - 팔로워들이 발사 지점 계산에 사용
    LeaderAimPose aim_pose;
    aim_pose.latitude = current_lat_;
    aim_pose.longitude = current_lon_;
    aim_pose.altitude = current_alt_;
    aim_pose.yaw_deg = current_yaw_;
    aim_pose.distance_to_target = lidar_distance_;
    aim_pose.target_latitude = target_lat_;
    aim_pose.target_longitude = target_lon_;
    aim_pose.aim_locked = false;
    
    publishAimPose(aim_pose);
}
```

### 4.7 Phase 7: 조준
```cpp
void aim() {
    // 1. 상태 토픽 발행
    publishStatus("AIMING");
    
    // 2. 열화상 카메라로 열원 탐지 및 조준
    while (!isAimingStable()) {
        ThermalCenter center = getThermalCenter();
        
        // Yaw 조정 (좌우)
        float yaw_error = center.x - image_center_x_;
        adjustYaw(yaw_error);
        
        // 고도/Pitch 조정 (상하)
        float pitch_error = center.y - image_center_y_;
        adjustAltitude(pitch_error);
    }
    
    // 3. 정조준 자세 저장
    saveAimPose();
    
    // 4. 조준 완료 토픽 발행
    aim_pose_.aim_locked = true;
    publishAimPose(aim_pose_);
}
```

### 4.8 Phase 8: 격발
```cpp
void fire() {
    for (int shot = 1; shot <= 6; shot++) {
        // 1. 상태 토픽 발행
        publishStatus("FIRING", shot);
        
        // 2. 조준 자세 확인/복귀
        if (!isAimPoseValid()) {
            returnToSavedAimPose();
            reAim();
        }
        
        // 3. 안정화 대기 (1초)
        sleep(1.0f);
        
        // 4. 격발
        sendFireSignal(shot);
        
        // 5. 안정화 대기 (4초)
        sleep(4.0f);
    }
}
```

### 4.9 Phase 9: 복귀
```cpp
void returnToHome() {
    // 1. 상태 토픽 발행
    publishStatus("RETURNING");
    
    // 2. RTL 고도로 상승
    flyToAltitude(rtl_altitude_);
    
    // 3. 이륙 지점으로 이동
    flyToPosition(home_position_.lat, home_position_.lon, rtl_altitude_);
    
    // 4. 도착 대기
    waitUntilPositionReached();
}
```

### 4.10 Phase 10: 착륙
```cpp
void land() {
    // 1. 상태 토픽 발행
    publishStatus("LANDING");
    
    // 2. 착륙 명령
    sendLandCommand();
    
    // 3. 착륙 완료 대기
    waitUntilLanded();
}
```

### 4.11 Phase 11: 시동 해제
```cpp
void disarm() {
    // 1. 상태 토픽 발행
    publishStatus("DISARMED");
    
    // 2. 시동 해제 명령
    sendDisarmCommand();
    
    // 3. 임무 완료
    missionComplete_ = true;
}
```

---

## 5. 상세 시퀀스 - 2번, 3번 팔로워 기체

### 5.1 초기화 및 토픽 구독
```cpp
void initialize() {
    // 기체 번호 확인
    vehicle_id_ = loadVehicleIdFromConfig();  // /home/khadas/config/device_config.env
    
    // 리더 토픽 구독
    subscribeToLeaderStatus();   // /leader/status
    subscribeToLeaderPose();     // /leader/pose
    subscribeToLeaderAimPose();  // /leader/aim_pose
    
    // 복귀 지점 저장
    home_position_ = getCurrentGpsPosition();
}
```

### 5.2 Phase 1: 리더 시동 확인 → 자신 시동
```cpp
void onLeaderStatusCallback(const LeaderStatus& msg) {
    if (msg.phase == "ARMING" && !is_armed_) {
        // 리더가 시동 → 자신도 시동
        sendArmCommand();
        waitUntilArmed();
    }
}
```

### 5.3 Phase 2: 리더 이륙 확인 → 자신 이륙
```cpp
void onLeaderStatusCallback(const LeaderStatus& msg) {
    if (msg.phase == "TAKEOFF" && is_armed_ && !is_airborne_) {
        // 리더가 이륙 → 자신도 이륙
        sendTakeoffCommand(10.0f);
        waitUntilAltitudeReached(10.0f);
    }
}
```

### 5.4 Phase 3: 리더 이동 확인 → 편대 비행
```cpp
void onLeaderPoseCallback(const LeaderPose& msg) {
    if (current_phase_ == "WAYPOINT") {
        // 리더 위치/헤딩 기반 편대 위치 계산
        float formation_offset_angle;
        
        if (vehicle_id_ == 2) {
            // 2번: 리더 좌측 후방 45도
            formation_offset_angle = leader_yaw_ + 135.0f;  // 또는 +45도 후방
        } else if (vehicle_id_ == 3) {
            // 3번: 리더 우측 후방 45도
            formation_offset_angle = leader_yaw_ - 135.0f;  // 또는 -45도 후방
        }
        
        // 리더로부터 10m 떨어진 위치 계산
        auto [my_lat, my_lon] = calculateOffsetPosition(
            msg.latitude, msg.longitude,
            10.0f,  // 10m 거리
            formation_offset_angle
        );
        
        // 계산된 위치로 이동 (편대 유지)
        flyToPosition(my_lat, my_lon, msg.altitude);
    }
}
```

### 5.5 Phase 4: 리더 도착 확인 → 발사 지점 이동
```cpp
void onLeaderAimPoseCallback(const LeaderAimPose& msg) {
    if (current_phase_ == "ARRIVED" || current_phase_ == "DISTANCE_ADJ") {
        // 목표물 위치 확인
        target_lat_ = msg.target_latitude;
        target_lon_ = msg.target_longitude;
        
        // 자신의 발사 지점 계산 (정삼각형 포메이션)
        float formation_angle;
        
        if (vehicle_id_ == 2) {
            // 2번: 목표물 기준 리더 헤딩 + 60도 (좌측)
            formation_angle = msg.yaw_deg + 60.0f;
        } else if (vehicle_id_ == 3) {
            // 3번: 목표물 기준 리더 헤딩 - 60도 (우측)
            formation_angle = msg.yaw_deg - 60.0f;
        }
        
        // 목표물로부터 10m 거리의 발사 지점 계산
        auto [fire_lat, fire_lon] = calculateOffsetPosition(
            target_lat_, target_lon_,
            10.0f,  // 10m
            formation_angle
        );
        
        // 발사 지점으로 이동
        flyToPosition(fire_lat, fire_lon, msg.altitude);
        
        // 목표물 방향으로 헤딩 조정
        float target_yaw = calculateBearing(fire_lat, fire_lon, target_lat_, target_lon_);
        setYaw(target_yaw);
    }
}
```

### 5.6 Phase 5: 화재진압중 확인 → 거리 조정 + 조준
```cpp
void onLeaderStatusCallback(const LeaderStatus& msg) {
    if (msg.phase == "FIRE_FIGHTING") {
        // ★ 핵심 트리거: 리더가 화재진압중 상태가 되면 시작
        
        // 옵션: LiDAR 거리 조정 (설정에 따라)
        if (use_lidar_distance_control_) {
            adjustDistanceWithLidar();  // 리더와 동일 로직
        }
        
        // 열화상 조준
        aimWithThermal();
        
        // 정조준 자세 저장
        saveAimPose();
    }
}
```

### 5.7 Phase 6~8: 격발, 복귀, 착륙, 시동 해제
```cpp
// 리더와 동일한 시퀀스
void fireSequence() {
    for (int shot = 1; shot <= 6; shot++) {
        verifyAimPose();
        sleep(1.0f);
        sendFireSignal(shot);
        sleep(4.0f);
    }
}

void returnAndLand() {
    flyToPosition(home_position_.lat, home_position_.lon, rtl_altitude_);
    land();
    disarm();
}
```

---

## 6. ROS2 토픽 정의

### 6.1 리더 → 팔로워 토픽

#### /leader/status
```cpp
// humiro_msgs/LeaderStatus
std_msgs/Header header
string phase          # ARMING, TAKEOFF, WAYPOINT, ARRIVED, 
                      # DISTANCE_ADJ, FIRE_FIGHTING, AIMING, 
                      # FIRING, RETURNING, LANDING, DISARMED
uint8 shot_number     # 격발 시 1~6
bool mission_active   # 미션 진행 중 여부
```

#### /leader/pose
```cpp
// humiro_msgs/LeaderPose
std_msgs/Header header
float64 latitude      # 리더 GPS 위도
float64 longitude     # 리더 GPS 경도
float32 altitude      # 리더 고도 (m)
float32 yaw_deg       # 리더 헤딩 (도)
float32 speed         # 리더 속도 (m/s)
```

#### /leader/aim_pose
```cpp
// humiro_msgs/LeaderAimPose
std_msgs/Header header
float64 latitude              # 리더 위치 위도
float64 longitude             # 리더 위치 경도
float32 altitude              # 리더 고도
float32 yaw_deg               # 리더 헤딩
float32 distance_to_target    # 목표물까지 거리
float64 target_latitude       # 목표물 위도
float64 target_longitude      # 목표물 경도
bool aim_locked               # 조준 완료 여부
```

### 6.2 발행 주기

| 토픽 | 발행 주기 | 비고 |
|-----|---------|------|
| /leader/status | 이벤트 기반 | 상태 변경 시 즉시 발행 |
| /leader/pose | 10Hz | 이동 중 실시간 발행 |
| /leader/aim_pose | 10Hz | 발사 지점 도착 후 발행 |

---

## 7. 핵심 구현 포인트

### 7.1 팔로워의 상태 기반 동작
```cpp
// 핵심: 팔로워는 리더의 상태 토픽을 보고 자신의 동작 결정
switch (leader_status_.phase) {
    case "ARMING":
        doArm();
        break;
    case "TAKEOFF":
        doTakeoff();
        break;
    case "WAYPOINT":
        doFormationFlight();  // 편대 비행
        break;
    case "ARRIVED":
    case "DISTANCE_ADJ":
        moveToFirePosition();  // 발사 지점으로 이동
        break;
    case "FIRE_FIGHTING":     // ★ 핵심 트리거
        startAimingAndFiring();
        break;
    case "RETURNING":
        doReturnToHome();
        break;
}
```

### 7.2 이동 중 편대 유지
```cpp
// 이동 중에는 리더 기준 후방 45도 위치 유지
void maintainFormationDuringFlight() {
    // 2번: 좌측 후방
    // 3번: 우측 후방
    
    // 리더 pose 업데이트마다 자신의 위치 재계산
    // 실시간으로 편대 위치 유지
}
```

### 7.3 발사 지점 도착 후 정삼각형 포메이션
```cpp
// 도착 후에는 목표물 중심 정삼각형 위치
void moveToFirePosition() {
    // 2번: 목표물 기준 +60도
    // 3번: 목표물 기준 -60도
    
    // 목표물을 향해 기수 조정
}
```

### 7.4 안전한 후진 제어 (모든 기체)
```cpp
// 절대 회피 기동 금지!
void adjustDistanceWithLidar() {
    if (distance < 9.5f) {
        // 천천히 직선 후진 (0.25 m/s)
        moveBackward(0.25f);
        // 급격한 회피 기동 절대 금지
    }
}
```

---

## 8. 구현 순서 (권장)

### 8.1 1단계: 리더 기체 구현
1. QGC 커스텀 메시지 수신 핸들러
2. 상태 토픽 발행 기능
3. 시동 → 이륙 → 웨이포인트 이동 시퀀스
4. LiDAR 거리 조정
5. 열화상 조준
6. 격발 시퀀스
7. 복귀 및 착륙

### 8.2 2단계: 팔로워 기체 구현
1. 리더 토픽 구독
2. 상태 기반 동작 FSM
3. 편대 위치 계산 (이동 중)
4. 발사 지점 계산 (정삼각형)
5. LiDAR 거리 조정 (옵션)
6. 열화상 조준
7. 격발 시퀀스
8. 복귀 및 착륙

### 8.3 3단계: 통합 테스트
1. 리더 단독 테스트
2. 팔로워 단독 테스트 (리더 토픽 시뮬레이션)
3. 3대 동시 통합 테스트

---

## 9. 클래스 구조

```cpp
class OffboardManager {
public:
    OffboardManager();
    ~OffboardManager();
    
    // 메인 미션 함수
    void testMission3();  // QGC 커스텀 메시지 수신 시 호출
    
private:
    // ========== 기체 식별 ==========
    uint8_t vehicle_id_;  // /home/khadas/config/device_config.env 에서 로드
    bool is_leader_;      // vehicle_id_ == 1
    
    // ========== 리더 전용 ==========
    void leaderSequence();
    void publishLeaderStatus(const std::string& phase, uint8_t shot = 0);
    void publishLeaderPose();
    void publishLeaderAimPose();
    
    // ========== 팔로워 전용 ==========
    void followerSequence();
    void onLeaderStatusCallback(const LeaderStatus::SharedPtr msg);
    void onLeaderPoseCallback(const LeaderPose::SharedPtr msg);
    void onLeaderAimPoseCallback(const LeaderAimPose::SharedPtr msg);
    void calculateFormationPosition();   // 이동 중 편대
    void calculateFirePosition();        // 발사 지점
    
    // ========== 공통 ==========
    void arm();
    void takeoff();
    void flyToPosition(double lat, double lon, float alt);
    void adjustDistanceWithLidar();
    void aimWithThermal();
    void fire();
    void returnToHome();
    void land();
    void disarm();
    
    // ========== ROS2 통신 ==========
    // Publishers (리더)
    rclcpp::Publisher<LeaderStatus>::SharedPtr leader_status_pub_;
    rclcpp::Publisher<LeaderPose>::SharedPtr leader_pose_pub_;
    rclcpp::Publisher<LeaderAimPose>::SharedPtr leader_aim_pose_pub_;
    
    // Subscribers (팔로워)
    rclcpp::Subscription<LeaderStatus>::SharedPtr leader_status_sub_;
    rclcpp::Subscription<LeaderPose>::SharedPtr leader_pose_sub_;
    rclcpp::Subscription<LeaderAimPose>::SharedPtr leader_aim_pose_sub_;
    
    // ========== 상태 ==========
    std::string current_phase_;
    GpsPosition home_position_;
    AimPose saved_aim_pose_;
    LeaderStatus leader_status_;
    LeaderPose leader_pose_;
    LeaderAimPose leader_aim_pose_;
};
```

---

## 10. 설정 파라미터

```yaml
# config/swarm_fire_suppression.yaml

# 기체 식별 (또는 device_config.env에서 로드)
vehicle_id: 1  # 1: 리더, 2: 좌측 팔로워, 3: 우측 팔로워

# 이동 중 편대 설정
formation_flight:
  distance_from_leader: 10.0     # 리더와의 거리 (m)
  angle_offset_vehicle_2: 135.0  # 2번: 좌측 후방 각도
  angle_offset_vehicle_3: -135.0 # 3번: 우측 후방 각도

# 발사 지점 정삼각형 설정
fire_position:
  distance_from_target: 10.0     # 목표물과의 거리 (m)
  angle_offset_vehicle_2: 60.0   # 2번: +60도
  angle_offset_vehicle_3: -60.0  # 3번: -60도

# 거리 제어
distance_control:
  target_distance: 10.0
  tolerance: 0.5                 # 5%
  forward_speed: 0.4
  backward_speed: 0.25
  stable_time: 3.0

# 조준 제어
aiming_control:
  tolerance: 0.05                # 5%
  stable_time: 1.0

# 격발 제어
firing_control:
  num_shots: 6
  interval: 5.0

# 복귀 설정
rtl:
  altitude: 10.0
  landing_speed: 0.5

# 팔로워 옵션
follower:
  use_lidar_distance_control: true
```

---

## 11. 참고 문서

- **024 Offboard 모드 시퀀스 버전2** (상위 문서)
- **fire_suppression_mission_spec.md** (상세 사양서)
- **PX4 MAVLink 프로토콜**
- **ROS2 통신 가이드**

---

## 12. 체크리스트

### 구현 완료 확인
- [ ] 기체 번호 로드 (/home/khadas/config/device_config.env)
- [ ] QGC 커스텀 메시지 수신
- [ ] 리더 상태 토픽 발행 (/leader/status)
- [ ] 리더 위치 토픽 발행 (/leader/pose)
- [ ] 리더 조준 위치 토픽 발행 (/leader/aim_pose)
- [ ] 팔로워 토픽 구독 및 콜백
- [ ] 이동 중 편대 위치 계산
- [ ] 발사 지점 정삼각형 위치 계산
- [ ] LiDAR 거리 조정 (안전한 후진)
- [ ] 열화상 조준
- [ ] 격발 시퀀스 (6발, 5초 간격)
- [ ] 복귀 및 착륙
- [ ] 통합 테스트

---

**문서 끝**