# 화재진압 드론 스웜 시스템 - 삼각 포메이션 자동 조준 및 격발 임무 (개정판)

## 목차
1. [시스템 개요](#시스템-개요)
2. [기체 구성](#기체-구성)
3. [1번 리더 기체 동작 시퀀스](#1번-리더-기체-동작-시퀀스)
4. [2번/3번 팔로워 기체 동작 시퀀스](#23번-팔로워-기체-동작-시퀀스)
5. [핵심 구현 요구사항](#핵심-구현-요구사항)
6. [ROS2 통신 구조](#ros2-통신-구조)
7. [코드 작성 참고사항](#코드-작성-참고사항)
8. [시퀀스 다이어그램](#시퀀스-다이어그램)
9. [안전 요구사항](#안전-요구사항)
10. [설정 파라미터](#설정-파라미터)
11. [테스트 시나리오](#테스트-시나리오)

---

## 시스템 개요

QGC 미션 모드로 이륙 및 목표지점 이동 후, Offboard 모드로 전환하여 3대의 드론(리더 1대, 팔로워 2대)이 삼각 포메이션을 형성하고 화재 목표물을 향해 자동 조준 및 소화탄 발사를 수행하는 시스템입니다.

### 미션 흐름
```
QGC 미션 (시동 → 이륙 → 목표지점 이동) 
    ↓
Offboard 모드 전환
    ↓
testMission3() 실행
    ↓
임무 완료 후 복귀 및 착륙
```

---

## 기체 구성

### 기체 번호 확인 방법
기체 번호는 `/home/khadas/config/device_config.env` 파일에서 확인할 수 있습니다.

```bash
# device_config.env 파일 예시
VEHICLE_ID=1  # 1: 리더, 2: 좌측 팔로워, 3: 우측 팔로워
```

### 기체 사양

| 기체 번호 | 역할 | 위치 | 센서 구성 |
|---------|------|------|----------|
| 1번 | 리더 | 중앙 정면 | LiDAR + 열화상 카메라 + RTK GPS |
| 2번 | 팔로워 (좌측) | 리더 좌측 대각선 | LiDAR + 열화상 카메라 + RTK GPS |
| 3번 | 팔로워 (우측) | 리더 우측 대각선 | LiDAR + 열화상 카메라 + RTK GPS |

### 포메이션 구조
- **각 기체는 목표물로부터 10m 거리 유지**
- **정삼각형 포메이션 형성** (각 변의 길이 10m)
- 리더-팔로워 간 거리도 10m
- **모든 기체가 전방 LiDAR 장착**

```
           [목표물]
          /   |   \
        /     |     \
      10m    10m    10m
     /        |       \
[2번 좌측]     |      [3번 우측]
     \        |       /
      10m    10m    10m
        \     |     /
         [1번 리더]

각도 관계:
- 2번-목표물-1번: 60도
- 3번-목표물-1번: 60도
- 2번-목표물-3번: 120도
```

---

## 구현 요청사항

### OffboardManager 클래스에 testMission3() 함수 생성

#### 공통 초기 동작 (모든 기체)

1. **현재 위치 고정 및 목표 고도 설정**
   - Offboard 모드 진입 시 현재 GPS 좌표 저장 (복귀 지점)
   - 목표 고도로 상승 또는 유지
   - 호버링 상태에서 다음 단계 준비

---

## 1번 리더 기체 동작 시퀀스

### Phase 1: 거리 조정 (LiDAR 기반)

#### 목표
- LiDAR 센서로 목표물까지 거리 실시간 측정
- 목표 거리: **10m ± 5% (9.5m ~ 10.5m)**

#### 거리 제어 로직

| 조건 | 동작 | 속도 | 비고 |
|-----|------|------|------|
| 거리 > 10.5m | 전진 이동 | 0.3~0.5 m/s | - |
| 거리 < 9.5m | 천천히 후진 | 0.2~0.3 m/s | **급격한 회피 기동 절대 금지** |
| 9.5m ≤ 거리 ≤ 10.5m | 위치 고정 | 0 m/s | 거리 제어 중지 |

#### 중요 사항
- **후진 제어 원칙**:
  - 절대 급격한 회피 기동 금지
  - 직선 후진만 수행
  - 장애물 감지 시 즉시 정지
- 거리 오차 5% 이내 유지 중에도 벗어나면 즉시 재조정
- **거리 안정화 판단**: 연속 3초 이상 오차 범위 내 유지

```cpp
// 의사 코드
while (!distance_stable) {
    float distance = lidar_distance_;
    
    if (distance > 10.5) {
        move_forward(0.4);
    } else if (distance < 9.5) {
        move_backward(0.25);  // 천천히 후진
    } else {
        hold_position();
        if (stable_duration > 3.0s) {
            distance_stable = true;
        }
    }
}
```

---

### Phase 2: 열원 자동 조준

#### 목표
- 열화상 카메라로 열원 탐지
- 열원 중심이 화면 중앙(조준점)에 오도록 기체 자세 제어

#### 조준 제어

| 제어 축 | 목적 | 허용 오차 |
|--------|------|----------|
| Yaw (헤딩) | 좌우 정렬 | ± 5% |
| Pitch/Roll | 상하 정렬 | ± 5% |

#### 정조준 달성 시 저장 정보

```cpp
struct AimPose {
    double latitude;           // 리더 GPS 위도
    double longitude;          // 리더 GPS 경도
    float altitude;            // 리더 고도 (MSL)
    float yaw_deg;            // 리더 헤딩 (도 단위)
    float distance_to_target; // 목표물까지 거리 (m)
    bool valid;               // 데이터 유효성
};
```

#### ROS2 토픽 발행

정조준 완료 시 `/leader_aim_pose` 토픽 발행:

```cpp
humiro_msgs/LeaderAimPose
{
  Header header
  float64 latitude              // 리더 GPS 위도
  float64 longitude             // 리더 GPS 경도
  float32 altitude              // 리더 고도 (MSL)
  float32 yaw_deg               // 리더 헤딩 (도 단위)
  float32 distance_to_target    // 목표물까지 거리 (m)
  float64 target_latitude       // 계산된 목표물 위도
  float64 target_longitude      // 계산된 목표물 경도
  bool aim_locked               // 조준 완료 상태
}
```

---

### Phase 3: 격발 시퀀스

#### 격발 개요
- 총 6발 소화탄 순차 발사
- 격발 간격: **5초** (조준 확인 1초 + 격발 + 안정화 4초)

#### 각 격발 사이클

```
1. 저장된 조준 자세 복귀 확인
   ├─ GPS 위치 오차 < 0.5m
   ├─ 헤딩 오차 < 3도
   ├─ LiDAR 거리 오차 < 5%
   └─ 오차 초과 시 자동 보정 후 재확인
   
2. 열화상으로 정조준 재확인
   └─ 열원 중심 ± 5% 이내
   
3. 조준 안정화 대기 (1초)

4. 격발 신호 전송
   └─ PWM/GPIO/시리얼 인터페이스
   
5. 격발 후 안정화 대기 (4초)

6. 다음 격발 준비
```

#### 격발 순서
**1번 → 2번 → 3번 → 4번 → 5번 → 6번**

#### 격발 상태 토픽 발행

```cpp
humiro_msgs/FiringStatus
{
  Header header
  uint8 current_shot      // 현재 격발 번호 (1~6)
  bool is_firing          // 격발 진행 중
  bool shot_completed     // 해당 격발 완료
}
```

---

### Phase 4: 복귀 및 착륙

#### 복귀 절차
1. 6발 격발 완료 즉시 이륙 지점 GPS 좌표로 복귀
2. 고도 10m에서 이륙 지점 도달 확인
   - GPS 오차 < 1m
   - 연속 2초 이상 유지
3. 자동 착륙 수행
   - 하강 속도: 0.5 m/s
4. 착륙 완료 후 모터 정지
5. 임무 종료

```cpp
// 의사 코드
void returnToHome() {
    // 이륙 지점으로 복귀
    fly_to_position(home_lat_, home_lon_, 10.0);
    
    // 도달 대기
    wait_until_position_reached();
    
    // 착륙
    auto_land(0.5);  // 0.5 m/s 하강
    
    // 모터 정지
    disarm();
}
```

---

## 2번/3번 팔로워 기체 동작 시퀀스

### Phase 1: 리더 기체 정보 구독 및 포메이션 계산

#### ROS2 토픽 구독
- 토픽: `/leader_aim_pose`
- 수신 정보:
  - 리더 GPS 좌표
  - 리더 고도
  - 리더 헤딩
  - 계산된 목표물 위치

#### 자신의 목표 위치 계산 (정삼각형 구조)

**정삼각형 포메이션**:
- 모든 변의 길이: 10m
- 목표물이 상단 꼭지점
- 리더가 하단 꼭지점
- 팔로워들이 좌우 대각선 위치

```cpp
// 목표물 좌표는 리더가 계산하여 전송
target_lat = leader_msg.target_latitude;
target_lon = leader_msg.target_longitude;

// 자신의 포메이션 위치 계산 (목표물 중심 기준)
// 정삼각형 구조: 각 변 10m, 각 꼭지점 각도 60도
float formation_angle;

if (vehicle_id == 2) {
    // 2번 기체: 목표물 기준 리더 헤딩 + 60도 방향 (좌측 대각선)
    formation_angle = leader_msg.yaw_deg + 60.0;
} else if (vehicle_id == 3) {
    // 3번 기체: 목표물 기준 리더 헤딩 - 60도 방향 (우측 대각선)
    formation_angle = leader_msg.yaw_deg - 60.0;
}

// 목표물로부터 10m 거리의 자신의 위치 계산
auto [my_target_lat, my_target_lon] = calculateOffsetPosition(
    target_lat, 
    target_lon,
    10.0,  // 10m 거리 (정삼각형 한 변)
    formation_angle
);

// 고도는 리더와 동일
my_target_alt = leader_msg.altitude;

// 목표물을 향한 헤딩 계산 (목표물 방향으로 기수 향함)
my_target_yaw = calculateBearing(
    my_target_lat, my_target_lon,
    target_lat, target_lon
);
```

```cpp
// 목표물 좌표는 리더가 계산하여 전송
target_lat = leader_msg.target_latitude;
target_lon = leader_msg.target_longitude;

// 자신의 포메이션 위치 계산 (목표물 중심 기준)
float formation_angle;

if (vehicle_id == 2) {
    // 2번 기체: 목표물 기준 리더 헤딩 + 120도 방향 (좌측)
    formation_angle = leader_msg.yaw_deg + 120.0;
} else if (vehicle_id == 3) {
    // 3번 기체: 목표물 기준 리더 헤딩 - 120도 방향 (우측)
    formation_angle = leader_msg.yaw_deg - 120.0;
}

// 목표물로부터 10m 거리의 자신의 위치 계산
auto [my_target_lat, my_target_lon] = calculateOffsetPosition(
    target_lat, 
    target_lon,
    10.0,  // 10m 거리
    formation_angle
);

// 고도는 리더와 동일
my_target_alt = leader_msg.altitude;

// 목표물을 향한 헤딩 계산
my_target_yaw = atan2(
    target_lon - my_target_lon, 
    target_lat - my_target_lat
) * 180.0 / M_PI;
```

#### 포메이션 각도 설명

**정삼각형 구조** (모든 변의 길이 10m):
- 목표물을 꼭지점으로 하는 정삼각형
- 1번(리더)이 아래, 2번/3번이 좌우 대각선 위

| 기체 | 목표물 기준 각도 | 리더 기준 각도 | 비고 |
|-----|--------------|--------------|------|
| 1번 | 0° (정면) | - | 리더 기체 |
| 2번 | 좌측 60° | 리더 헤딩 + 60° | 정삼각형 좌측 |
| 3번 | 우측 60° | 리더 헤딩 - 60° | 정삼각형 우측 |

**중요**: 기존 120도가 아닌 **60도** 각도 사용

---

### Phase 2: 자동 위치 이동

#### 이동 목표
- 계산된 GPS 좌표로 자동 이동
- 목표 고도로 이동
- 목표 헤딩으로 회전

#### 이동 파라미터

| 항목 | 값 | 비고 |
|-----|---|------|
| 이동 속도 | 2~3 m/s | 수평 이동 |
| GPS 오차 허용 | < 1m | 위치 도달 판단 |
| 고도 오차 허용 | < 0.5m | 고도 도달 판단 |
| 헤딩 오차 허용 | < 3° | 헤딩 도달 판단 |
| 안정화 시간 | 2초 | 연속 유지 |

#### 이동 시퀀스

```cpp
// 의사 코드
void moveToFormationPosition() {
    // GPS 좌표로 이동
    fly_to_position(my_target_lat, my_target_lon, my_target_alt);
    
    // 위치 도달 대기
    while (position_error > 1.0 || altitude_error > 0.5) {
        wait();
    }
    
    // 목표 헤딩으로 회전
    set_yaw(my_target_yaw);
    
    // 헤딩 도달 대기
    while (yaw_error > 3.0) {
        wait();
    }
    
    // 안정화 대기 (2초)
    wait(2.0);
    
    // 호버링 상태로 전환
    hold_position();
}
```

---

### Phase 3: 거리 조정 (LiDAR 기반 - 옵션)

#### 옵션 A: LiDAR 사용 (권장)

**설정 활성화 시 동작**

- 전방 LiDAR로 목표물까지 거리 실시간 측정
- 목표 거리: **10m ± 5% (9.5m ~ 10.5m)**

**거리 제어 로직** (리더와 동일)

| 조건 | 동작 | 속도 | 비고 |
|-----|------|------|------|
| 거리 > 10.5m | 전진 이동 | 0.3~0.5 m/s | - |
| 거리 < 9.5m | 천천히 후진 | 0.2~0.3 m/s | **급격한 회피 기동 절대 금지** |
| 9.5m ≤ 거리 ≤ 10.5m | 위치 고정 | 0 m/s | 거리 제어 중지 |

**중요 사항**:
- 절대 급격한 회피 기동 금지
- 직선 후진만 수행
- 장애물 감지 시 즉시 정지
- 거리 안정화 판단: 연속 3초 이상 오차 범위 내 유지

#### 옵션 B: GPS 기반 위치만 사용

**설정 비활성화 시 동작**

- 계산된 GPS 위치에 고정
- LiDAR 거리 제어 생략
- 바로 조준 단계로 진입

#### 구현 방법

```cpp
// 설정 파라미터로 제어
class OffboardManager {
private:
    bool use_lidar_for_followers_ = true;  // LiDAR 사용 여부
    
public:
    void followerDistanceControl() {
        if (use_lidar_for_followers_) {
            // 옵션 A: LiDAR 거리 제어 수행
            performLidarDistanceControl();
        } else {
            // 옵션 B: GPS 위치만 사용
            holdGpsPosition();
        }
    }
};
```

---

### Phase 4: 열원 자동 조준

#### 조준 절차 (리더와 동일)

1. 열화상 카메라로 목표물 탐지
2. 열원 중심이 화면 중앙에 오도록 자세 제어
   - **Yaw(헤딩) 제어**: 좌우 정렬
   - **Pitch/Roll 미세 조정**: 상하 정렬
3. 정조준 기준: 열원 중심 ± 5% 이내
4. 정조준 달성 시 저장:
   - 현재 GPS 좌표
   - 현재 고도
   - 현재 헤딩
   - LiDAR 거리 (옵션 A 사용 시)
   - 조준 완료 플래그 설정

```cpp
struct FollowerAimPose {
    double latitude;
    double longitude;
    float altitude;
    float yaw_deg;
    float distance;  // 옵션 A 사용 시
    bool valid;
};
```

---

### Phase 5: 격발 시퀀스 (리더와 동일)

#### 격발 개요
- 총 6발 소화탄 순차 발사
- 격발 간격: **5초**

#### 각 격발 사이클

```
1. 저장된 조준 자세 복귀 확인
   ├─ GPS 위치 오차 < 0.5m
   ├─ 헤딩 오차 < 3도
   ├─ LiDAR 거리 오차 < 5% (옵션 A 사용 시)
   └─ 오차 초과 시 자동 보정
   
2. 열화상으로 정조준 재확인
   └─ 열원 중심 ± 5% 이내
   
3. 조준 안정화 대기 (1초)

4. 격발 신호 전송
   └─ PWM/GPIO/시리얼
   
5. 격발 후 안정화 대기 (4초)

6. 다음 격발 준비
```

#### 격발 순서
**1번 → 2번 → 3번 → 4번 → 5번 → 6번**

#### 선택사항: 리더와 동기화

리더의 `/leader_firing_status` 토픽 구독하여 동기화 가능:

```cpp
void onLeaderFiringStatus(const humiro_msgs::msg::FiringStatus::SharedPtr msg) {
    if (msg->shot_completed && msg->current_shot < 6) {
        // 리더가 격발 완료하면 팔로워도 다음 격발 준비
        prepareNextShot(msg->current_shot + 1);
    }
}
```

---

### Phase 6: 복귀 및 착륙

#### 복귀 절차 (리더와 동일)

1. 6발 격발 완료 즉시 이륙 지점으로 복귀
2. 고도 10m에서 이륙 지점 도달 확인
   - GPS 오차 < 1m
   - 연속 2초 이상 유지
3. 자동 착륙 수행
   - 하강 속도: 0.5 m/s
4. 착륙 완료 후 모터 정지
5. 임무 종료

---

## 핵심 구현 요구사항

### 1. 안전한 거리 제어 (모든 기체 공통)

#### 후진 제어 원칙
- **천천히 후진**: 0.2~0.3 m/s
- **절대 급격한 회피 기동 금지**
- **직선 후진만 수행**
- 장애물 감지 시 즉시 정지

#### 거리 제어 안정화
- 연속 3초 이상 오차 범위 내 유지

```cpp
void safeBackwardControl(float current_distance, float target_distance) {
    if (current_distance < target_distance * 0.95f) {
        // 천천히 후진
        float backward_speed = -0.25f;  // m/s (음수 = 후진)
        setVelocityBody(backward_speed, 0.0f, 0.0f, current_yaw_);
        
        // 장애물 감지 (후방)
        if (rear_obstacle_detected_) {
            setVelocityBody(0.0f, 0.0f, 0.0f, current_yaw_);
            RCLCPP_WARN(get_logger(), "Rear obstacle detected! Stopping backward movement.");
        }
    }
}
```

---

### 2. 정밀 조준 시스템 (모든 기체 공통)

#### 조준 허용 오차
- 열화상 중심점 기준 ± 5%

#### 조준 완료 시 저장
- GPS 좌표
- 고도
- 헤딩
- LiDAR 거리

#### 매 격발 전 자세 복귀
- 저장된 자세로 자동 복귀
- 오차 검증 후 격발

#### 조준 안정화
- 1초 이상 유지 확인

```cpp
bool checkAimingStability() {
    // 연속 1초 이상 조준 유지 확인
    float image_center_x = image_width_ / 2.0f;
    float image_center_y = image_height_ / 2.0f;
    
    float x_error = abs(thermal_center_.x - image_center_x);
    float y_error = abs(thermal_center_.y - image_center_y);
    
    float x_tolerance = aim_tolerance_ * image_width_;
    float y_tolerance = aim_tolerance_ * image_height_;
    
    if (x_error < x_tolerance && y_error < y_tolerance) {
        auto now = std::chrono::steady_clock::now();
        
        if (!aim_stable_start_time_) {
            aim_stable_start_time_ = now;
        }
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - aim_stable_start_time_.value()
        );
        
        return duration.count() >= 1000;  // 1초
    } else {
        aim_stable_start_time_.reset();
        return false;
    }
}
```

---

### 3. 팔로워 자율 포메이션

#### 팔로워 자율 포메이션
- 리더의 RTK GPS
- 리더의 헤딩
- 계산된 목표물 위치

#### 정삼각형 포메이션 자동 유지
- **2번 기체: 목표물 기준 리더 대비 +60도 위치**
- **3번 기체: 목표물 기준 리더 대비 -60도 위치**
- 모든 변의 길이: 10m

#### 옵션 선택
1. GPS 계산 위치만 사용
2. GPS + LiDAR 거리 제어 병행 (권장)

```cpp
// WGS84 좌표계에서 거리/방위각으로 새 좌표 계산
std::pair<double, double> calculateOffsetPosition(
    double origin_lat, 
    double origin_lon,
    float distance_m, 
    float bearing_deg
) {
    const double R = 6378137.0;  // 지구 반지름 (m)
    
    double lat_rad = origin_lat * M_PI / 180.0;
    double lon_rad = origin_lon * M_PI / 180.0;
    double bearing_rad = bearing_deg * M_PI / 180.0;
    
    double new_lat_rad = asin(
        sin(lat_rad) * cos(distance_m / R) +
        cos(lat_rad) * sin(distance_m / R) * cos(bearing_rad)
    );
    
    double new_lon_rad = lon_rad + atan2(
        sin(bearing_rad) * sin(distance_m / R) * cos(lat_rad),
        cos(distance_m / R) - sin(lat_rad) * sin(new_lat_rad)
    );
    
    return {
        new_lat_rad * 180.0 / M_PI, 
        new_lon_rad * 180.0 / M_PI
    };
}
```

---

### 4. 동기화된 격발 (모든 기체 공통)

#### 격발 간격
- 모든 기체 동일한 5초 간격
- 조준 확인 1초 + 격발 + 안정화 4초

#### 조준 손실 시
- 자동 재조준 후 격발

#### 6발 완료 후
- 즉시 복귀

```cpp
void firingSequence() {
    for (uint8_t shot = 1; shot <= 6; shot++) {
        // 1. 조준 자세 복귀
        returnToAimPose();
        
        // 2. 조준 재확인
        while (!checkAimingStability()) {
            adjustAiming();
        }
        
        // 3. 조준 안정화 대기
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 4. 격발
        sendFireSignal(shot);
        publishFiringStatus(shot, true, false);
        
        // 5. 안정화 대기
        std::this_thread::sleep_for(std::chrono::seconds(4));
        
        // 6. 격발 완료
        publishFiringStatus(shot, false, true);
    }
}
```

---

## ROS2 통신 구조

### 토픽 리스트

#### 1. /leader_aim_pose

**발행자**: 1번 리더 기체  
**구독자**: 2번, 3번 팔로워 기체  
**빈도**: 10Hz  

**메시지 타입**: `humiro_msgs/LeaderAimPose`

```cpp
std_msgs/Header header

# 리더 기체 위치 및 자세
float64 latitude              # 리더 GPS 위도 (도)
float64 longitude             # 리더 GPS 경도 (도)
float32 altitude              # 리더 고도 MSL (m)
float32 yaw_deg               # 리더 헤딩 (도)

# 목표물 정보
float32 distance_to_target    # 목표물까지 거리 (m)
float64 target_latitude       # 계산된 목표물 위도 (도)
float64 target_longitude      # 계산된 목표물 경도 (도)

# 조준 상태
bool aim_locked               # 조준 완료 여부
```

---

#### 2. /leader_firing_status

**발행자**: 1번 리더 기체  
**구독자**: 2번, 3번 팔로워 기체 (선택사항)  
**빈도**: 이벤트 기반  

**메시지 타입**: `humiro_msgs/FiringStatus`

```cpp
std_msgs/Header header

# 격발 정보
uint8 current_shot           # 현재 격발 번호 (1~6)
bool is_firing               # 격발 진행 중
bool shot_completed          # 해당 격발 완료
```

---

#### 3. /vehicle_{id}/mission_status

**발행자**: 모든 기체  
**구독자**: GCS 모니터링 시스템 (선택사항)  
**빈도**: 1Hz  

**메시지 타입**: `humiro_msgs/MissionStatus`

```cpp
std_msgs/Header header

# 기체 정보
uint8 vehicle_id              # 기체 번호 (1/2/3)

# 임무 상태
string mission_phase          # 현재 단계
                              # "INIT", "DISTANCE_CONTROL", "AIMING",
                              # "FIRING", "RETURNING", "LANDING", "COMPLETE"

# 센서 데이터
float32 lidar_distance        # LiDAR 거리 (m)
bool aim_locked               # 조준 상태

# 격발 정보
uint8 shots_fired             # 격발 완료 수 (0~6)
```

---

### ROS2 노드 구조

```cpp
class FireSuppressionNode : public rclcpp::Node {
public:
    FireSuppressionNode(uint8_t vehicle_id);
    
private:
    // Publishers
    rclcpp::Publisher<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_pub_;
    rclcpp::Publisher<humiro_msgs::msg::FiringStatus>::SharedPtr firing_status_pub_;
    rclcpp::Publisher<humiro_msgs::msg::MissionStatus>::SharedPtr mission_status_pub_;
    
    // Subscribers (팔로워용)
    rclcpp::Subscription<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_sub_;
    rclcpp::Subscription<humiro_msgs::msg::FiringStatus>::SharedPtr firing_status_sub_;
    
    // Callbacks
    void leaderAimPoseCallback(const humiro_msgs::msg::LeaderAimPose::SharedPtr msg);
    void leaderFiringStatusCallback(const humiro_msgs::msg::FiringStatus::SharedPtr msg);
    
    // Data
    uint8_t vehicle_id_;
    humiro_msgs::msg::LeaderAimPose latest_leader_pose_;
};
```

---

## 코드 작성 참고사항

### 클래스 구조

```cpp
class OffboardManager {
public:
    OffboardManager(uint8_t vehicle_id);
    ~OffboardManager();
    
    // 메인 미션 함수
    void testMission3();
    
private:
    // ========== 기체 식별 ==========
    uint8_t vehicle_id_;  // 1: 리더, 2: 좌측 팔로워, 3: 우측 팔로워
    
    // 기체 번호는 /home/khadas/config/device_config.env 파일에서 확인
    // 예: VEHICLE_ID=1
    
    // ========== 리더 전용 함수 ==========
    void leaderDistanceControl();
    void leaderThermalAiming();
    void leaderFiringSequence();
    void publishLeaderAimPose();
    
    // ========== 팔로워 전용 함수 ==========
    void followerFormationCalculation();
    void followerMoveToPosition();
    void followerDistanceControl();     // 옵션: LiDAR 사용 시
    void followerThermalAiming();
    void followerFiringSequence();
    
    // ========== 공통 함수 ==========
    // 복귀 및 착륙
    void returnToHome();
    void autoLand();
    
    // 안정성 검증
    bool checkAimingStability();
    bool checkDistanceStability();
    bool checkPositionReached(double target_lat, double target_lon, float target_alt);
    
    // 격발 제어
    void sendFireSignal(uint8_t shot_number);
    void publishFiringStatus(uint8_t shot, bool is_firing, bool completed);
    
    // 위치 제어
    void setPositionGlobal(double lat, double lon, float alt);
    void setVelocityBody(float vx, float vy, float vz, float yaw);
    void setYaw(float yaw_deg);
    
    // 유틸리티
    std::pair<double, double> calculateOffsetPosition(double origin_lat, double origin_lon, 
                                                      float distance_m, float bearing_deg);
    float calculateDistance(double lat1, double lon1, double lat2, double lon2);
    float calculateBearing(double lat1, double lon1, double lat2, double lon2);
    
    // ========== ROS2 Publishers/Subscribers ==========
    rclcpp::Publisher<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_pub_;
    rclcpp::Publisher<humiro_msgs::msg::FiringStatus>::SharedPtr firing_status_pub_;
    rclcpp::Publisher<humiro_msgs::msg::MissionStatus>::SharedPtr mission_status_pub_;
    
    rclcpp::Subscription<humiro_msgs::msg::LeaderAimPose>::SharedPtr leader_aim_sub_;
    rclcpp::Subscription<humiro_msgs::msg::FiringStatus>::SharedPtr firing_status_sub_;
    
    // Callbacks
    void onLeaderAimPose(const humiro_msgs::msg::LeaderAimPose::SharedPtr msg);
    void onLeaderFiringStatus(const humiro_msgs::msg::FiringStatus::SharedPtr msg);
    
    // ========== 센서 데이터 ==========
    float lidar_distance_;
    cv::Point2f thermal_center_;
    double current_lat_, current_lon_;
    float current_alt_, current_yaw_;
    
    // ========== 저장된 조준 자세 ==========
    struct AimPose {
        double latitude;
        double longitude;
        float altitude;
        float yaw_deg;
        float distance;
        bool valid;
        
        AimPose() : valid(false) {}
    };
    AimPose saved_aim_pose_;
    
    // ========== 복귀 지점 ==========
    double home_lat_, home_lon_;
    float home_alt_;
    
    // ========== 팔로워 데이터 ==========
    humiro_msgs::msg::LeaderAimPose leader_pose_;
    double my_target_lat_, my_target_lon_;
    float my_target_alt_, my_target_yaw_;
    
    // ========== 설정 파라미터 ==========
    // 거리 제어
    float target_distance_ = 10.0f;         // 목표 거리 (m)
    float distance_tolerance_ = 0.5f;       // 거리 허용 오차 (m) = 5%
    float forward_speed_ = 0.4f;            // 전진 속도 (m/s)
    float backward_speed_ = 0.25f;          // 후진 속도 (m/s)
    float distance_stable_time_ = 3.0f;     // 거리 안정화 시간 (초)
    
    // 조준 제어
    float aim_tolerance_ = 0.05f;           // 조준 허용 오차 (5%)
    float aim_stable_time_ = 1.0f;          // 조준 안정화 시간 (초)
    
    // 격발 제어
    uint8_t num_shots_ = 6;                 // 총 격발 수
    float shot_interval_ = 5.0f;            // 격발 간격 (초)
    float stabilization_time_ = 1.0f;       // 격발 전 안정화 (초)
    
    // 팔로워 옵션
    bool use_lidar_for_followers_ = true;   // 팔로워 LiDAR 사용 여부
    float formation_angle_offset_ = 60.0f;  // 포메이션 각도 (도) - 정삼각형
    
    // 복귀 설정
    float rtl_altitude_ = 10.0f;            // 복귀 고도 (m)
    float landing_speed_ = 0.5f;            // 착륙 속도 (m/s)
    
    // ========== 안정화 타이머 ==========
    std::optional<std::chrono::steady_clock::time_point> aim_stable_start_time_;
    std::optional<std::chrono::steady_clock::time_point> distance_stable_start_time_;
    
    // ========== 임무 상태 ==========
    std::string mission_phase_;
    uint8_t shots_fired_ = 0;
};
```

---

### 주요 알고리즘

#### 1. 팔로워 포메이션 위치 계산

```cpp
// WGS84 좌표계에서 거리/방위각으로 새 좌표 계산
std::pair<double, double> OffboardManager::calculateOffsetPosition(
    double origin_lat, 
    double origin_lon,
    float distance_m, 
    float bearing_deg
) {
    const double R = 6378137.0;  // 지구 반지름 (m)
    
    // 도 → 라디안 변환
    double lat_rad = origin_lat * M_PI / 180.0;
    double lon_rad = origin_lon * M_PI / 180.0;
    double bearing_rad = bearing_deg * M_PI / 180.0;
    
    // 새 위도 계산
    double new_lat_rad = asin(
        sin(lat_rad) * cos(distance_m / R) +
        cos(lat_rad) * sin(distance_m / R) * cos(bearing_rad)
    );
    
    // 새 경도 계산
    double new_lon_rad = lon_rad + atan2(
        sin(bearing_rad) * sin(distance_m / R) * cos(lat_rad),
        cos(distance_m / R) - sin(lat_rad) * sin(new_lat_rad)
    );
    
    // 라디안 → 도 변환
    return {
        new_lat_rad * 180.0 / M_PI, 
        new_lon_rad * 180.0 / M_PI
    };
}
```

---

#### 2. 안전한 후진 제어

```cpp
void OffboardManager::safeBackwardControl(float current_distance, float target_distance) {
    if (current_distance < target_distance * 0.95f) {
        // 천천히 후진
        float backward_speed = -backward_speed_;  // m/s (음수 = 후진)
        
        // Body 프레임 속도 명령 (전진/후진)
        setVelocityBody(backward_speed, 0.0f, 0.0f, current_yaw_);
        
        RCLCPP_INFO(get_logger(), "Backward control: %.2f m/s", backward_speed);
        
        // 장애물 감지 (후방) - 센서 있는 경우
        if (rear_obstacle_detected_) {
            // 즉시 정지
            setVelocityBody(0.0f, 0.0f, 0.0f, current_yaw_);
            RCLCPP_WARN(get_logger(), "Rear obstacle detected! Stopping.");
            
            // 회피 기동 하지 않음 - 단순 정지만 수행
        }
    } else if (current_distance > target_distance * 1.05f) {
        // 전진
        float forward_speed = forward_speed_;
        setVelocityBody(forward_speed, 0.0f, 0.0f, current_yaw_);
        
        RCLCPP_INFO(get_logger(), "Forward control: %.2f m/s", forward_speed);
    } else {
        // 거리 적정 - 위치 고정
        setVelocityBody(0.0f, 0.0f, 0.0f, current_yaw_);
        
        RCLCPP_INFO(get_logger(), "Distance OK: %.2f m", current_distance);
    }
}
```

---

#### 3. 조준 안정성 검증

```cpp
bool OffboardManager::checkAimingStability() {
    // 이미지 중심 좌표
    float image_center_x = image_width_ / 2.0f;
    float image_center_y = image_height_ / 2.0f;
    
    // 열원 중심과의 오차 계산
    float x_error = abs(thermal_center_.x - image_center_x);
    float y_error = abs(thermal_center_.y - image_center_y);
    
    // 허용 오차 (이미지 크기의 5%)
    float x_tolerance = aim_tolerance_ * image_width_;
    float y_tolerance = aim_tolerance_ * image_height_;
    
    // 오차 범위 내인지 확인
    if (x_error < x_tolerance && y_error < y_tolerance) {
        auto now = std::chrono::steady_clock::now();
        
        // 안정화 시작 시간 기록
        if (!aim_stable_start_time_) {
            aim_stable_start_time_ = now;
            RCLCPP_INFO(get_logger(), "Aiming stability check started");
        }
        
        // 안정화 지속 시간 계산
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - aim_stable_start_time_.value()
        );
        
        // 1초 이상 유지 확인
        bool stable = duration.count() >= (aim_stable_time_ * 1000);
        
        if (stable) {
            RCLCPP_INFO(get_logger(), "Aiming stable for %.1f seconds", 
                       duration.count() / 1000.0);
        }
        
        return stable;
    } else {
        // 오차 벗어남 - 타이머 리셋
        if (aim_stable_start_time_) {
            RCLCPP_WARN(get_logger(), "Aiming stability lost. Error: x=%.1f, y=%.1f", 
                       x_error, y_error);
        }
        aim_stable_start_time_.reset();
        return false;
    }
}
```

---

#### 4. 거리 안정성 검증

```cpp
bool OffboardManager::checkDistanceStability() {
    float lower_bound = target_distance_ * 0.95f;  // 9.5m
    float upper_bound = target_distance_ * 1.05f;  // 10.5m
    
    // 거리 오차 범위 내인지 확인
    if (lidar_distance_ >= lower_bound && lidar_distance_ <= upper_bound) {
        auto now = std::chrono::steady_clock::now();
        
        // 안정화 시작 시간 기록
        if (!distance_stable_start_time_) {
            distance_stable_start_time_ = now;
            RCLCPP_INFO(get_logger(), "Distance stability check started");
        }
        
        // 안정화 지속 시간 계산
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - distance_stable_start_time_.value()
        );
        
        // 3초 이상 유지 확인
        bool stable = duration.count() >= (distance_stable_time_ * 1000);
        
        if (stable) {
            RCLCPP_INFO(get_logger(), "Distance stable at %.2f m for %.1f seconds", 
                       lidar_distance_, duration.count() / 1000.0);
        }
        
        return stable;
    } else {
        // 오차 벗어남 - 타이머 리셋
        if (distance_stable_start_time_) {
            RCLCPP_WARN(get_logger(), "Distance stability lost. Current: %.2f m", 
                       lidar_distance_);
        }
        distance_stable_start_time_.reset();
        return false;
    }
}
```

---

#### 5. 위치 도달 확인

```cpp
bool OffboardManager::checkPositionReached(double target_lat, double target_lon, float target_alt) {
    // GPS 거리 계산 (m)
    float horizontal_distance = calculateDistance(
        current_lat_, current_lon_, 
        target_lat, target_lon
    );
    
    // 고도 오차 계산 (m)
    float altitude_error = abs(current_alt_ - target_alt);
    
    // 허용 오차 확인
    bool position_ok = horizontal_distance < 1.0f;  // 1m
    bool altitude_ok = altitude_error < 0.5f;       // 0.5m
    
    if (position_ok && altitude_ok) {
        RCLCPP_INFO(get_logger(), 
                   "Position reached. H_dist: %.2f m, Alt_err: %.2f m", 
                   horizontal_distance, altitude_error);
        return true;
    }
    
    return false;
}
```

---

#### 6. 두 GPS 좌표 간 거리 계산 (Haversine)

```cpp
float OffboardManager::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6378137.0;  // 지구 반지름 (m)
    
    // 도 → 라디안
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    // 위도/경도 차이
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    
    // Haversine 공식
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2) * sin(dlon / 2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    float distance = R * c;  // 미터 단위
    
    return distance;
}
```

---

#### 7. 방위각 계산

```cpp
float OffboardManager::calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    // 도 → 라디안
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    
    double dlon = lon2_rad - lon1_rad;
    
    // 방위각 계산
    double y = sin(dlon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon);
    
    double bearing_rad = atan2(y, x);
    
    // 라디안 → 도 변환 (0~360)
    float bearing_deg = fmod((bearing_rad * 180.0 / M_PI) + 360.0, 360.0);
    
    return bearing_deg;
}
```

---

## 시퀀스 다이어그램

```
┌──────────────┐
│ QGC Mission  │
└──────┬───────┘
       │ 시동 → 이륙 → 목표지점 이동
       ↓
┌──────────────┐
│  Offboard    │
│  전환        │
└──────┬───────┘
       │
       ↓
┌─────────────────────────────────────────────────────────┐
│              testMission3() 시작                        │
└──────┬──────────────────────────────────────────────────┘
       │
       ↓
┌──────────────────────────────────────────────────────────┐
│  Phase 1: 현재 위치 고정 + 목표 고도                      │
│  - 복귀 지점 저장 (home_lat, home_lon, home_alt)          │
│  - 목표 고도 설정                                         │
└──────┬───────────────────────────────────────────────────┘
       │
       ├─────────────────────┬──────────────────────┬────────────────────┐
       │                     │                      │                    │
       ↓                     ↓                      ↓                    │
┌─────────────┐      ┌─────────────┐       ┌─────────────┐            │
│  1번 리더   │      │ 2번 팔로워  │       │ 3번 팔로워  │            │
└──────┬──────┘      └──────┬──────┘       └──────┬──────┘            │
       │                     │                      │                    │
       ↓                     ↓                      ↓                    │
┌──────────────┐      ┌──────────────┐      ┌──────────────┐          │
│ Phase 2:     │      │ Phase 2:     │      │ Phase 2:     │          │
│ LiDAR 거리   │      │ 리더 토픽    │      │ 리더 토픽    │          │
│ 제어 (10m)   │      │ 구독         │      │ 구독         │          │
│              │      │              │      │              │          │
│ 9.5~10.5m    │      │ /leader_aim  │      │ /leader_aim  │          │
│ 전진/후진    │      │ _pose        │      │ _pose        │          │
│              │      │              │      │              │          │
│ 안정화 3초   │      └──────┬───────┘      └──────┬───────┘          │
└──────┬───────┘             │                      │                  │
       │                     ↓                      ↓                  │
       ↓              ┌──────────────┐      ┌──────────────┐          │
┌──────────────┐      │ Phase 3:     │      │ Phase 3:     │          │
│ Phase 3:     │      │ 포메이션     │      │ 포메이션     │          │
│ 열화상 조준  │      │ 위치 계산    │      │ 위치 계산    │          │
│              │      │              │      │              │          │
│ Yaw/Pitch    │      │ 좌측 +120°  │      │ 우측 -120°  │          │
│ 조정         │      │              │      │              │          │
│              │      │ GPS 이동     │      │ GPS 이동     │          │
│ 안정화 1초   │      └──────┬───────┘      └──────┬───────┘          │
└──────┬───────┘             │                      │                  │
       │                     ↓                      ↓                  │
       ↓              ┌──────────────┐      ┌──────────────┐          │
┌──────────────┐      │ Phase 4:     │      │ Phase 4:     │          │
│ Phase 4:     │      │ (옵션)       │      │ (옵션)       │          │
│ 조준 자세    │      │ LiDAR 거리   │      │ LiDAR 거리   │          │
│ 저장         │      │ 제어         │      │ 제어         │          │
│              │      │              │      │              │          │
│ GPS, 고도,   │      │ 9.5~10.5m    │      │ 9.5~10.5m    │          │
│ 헤딩, 거리   │      │              │      │              │          │
└──────┬───────┘      └──────┬───────┘      └──────┬───────┘          │
       │                     │                      │                  │
       ↓                     ↓                      ↓                  │
┌──────────────┐      ┌──────────────┐      ┌──────────────┐          │
│ ROS2 토픽    │      │ Phase 5:     │      │ Phase 5:     │          │
│ 발행         │      │ 열화상 조준  │      │ 열화상 조준  │          │
│              │      │              │      │              │          │
│ /leader_aim  │      │ 조준 자세    │      │ 조준 자세    │          │
│ _pose        │──────┤ 저장         │      │ 저장         │          │
└──────┬───────┘      └──────┬───────┘      └──────┬───────┘          │
       │                     │                      │                  │
       ↓                     ↓                      ↓                  │
┌──────────────┐      ┌──────────────┐      ┌──────────────┐          │
│ Phase 5:     │      │ Phase 6:     │      │ Phase 6:     │          │
│ 격발 시퀀스  │      │ 격발 시퀀스  │      │ 격발 시퀀스  │          │
│              │      │              │      │              │          │
│ 1번 격발     │      │ 1번 격발     │      │ 1번 격발     │          │
│  ├─조준 확인 │      │  ├─조준 확인 │      │  ├─조준 확인 │          │
│  ├─안정화1초 │      │  ├─안정화1초 │      │  ├─안정화1초 │          │
│  ├─격발      │      │  ├─격발      │      │  ├─격발      │          │
│  └─대기 4초  │      │  └─대기 4초  │      │  └─대기 4초  │          │
│              │      │              │      │              │          │
│ 2~6번 반복   │      │ 2~6번 반복   │      │ 2~6번 반복   │          │
│ (총 30초)    │      │ (총 30초)    │      │ (총 30초)    │          │
└──────┬───────┘      └──────┬───────┘      └──────┬───────┘          │
       │                     │                      │                  │
       ↓                     ↓                      ↓                  │
┌──────────────┐      ┌──────────────┐      ┌──────────────┐          │
│ Phase 6:     │      │ Phase 7:     │      │ Phase 7:     │          │
│ 복귀 및 착륙 │      │ 복귀 및 착륙 │      │ 복귀 및 착륙 │          │
│              │      │              │      │              │          │
│ 이륙 지점    │      │ 이륙 지점    │      │ 이륙 지점    │          │
│ 복귀         │      │ 복귀         │      │ 복귀         │          │
│              │      │              │      │              │          │
│ 고도 10m     │      │ 고도 10m     │      │ 고도 10m     │          │
│              │      │              │      │              │          │
│ 자동 착륙    │      │ 자동 착륙    │      │ 자동 착륙    │          │
│ (0.5 m/s)    │      │ (0.5 m/s)    │      │ (0.5 m/s)    │          │
│              │      │              │      │              │          │
│ 모터 정지    │      │ 모터 정지    │      │ 모터 정지    │          │
└──────┬───────┘      └──────┬───────┘      └──────┬───────┘          │
       │                     │                      │                  │
       └─────────────────────┴──────────────────────┴──────────────────┘
                                   │
                                   ↓
                         ┌──────────────┐
                         │  임무 종료   │
                         └──────────────┘
```

---

## 안전 요구사항

### 1. 충돌 방지

#### 센서 요구사항
- **전방 LiDAR**: 모든 기체 필수
- **후방 센서**: 선택사항 (추천)

#### 안전 거리
- 포메이션 이동 시 기체 간 최소 **5m** 거리 유지
- 장애물 감지 시 즉시 정지

#### 회피 원칙
- **후진 시**: 급격한 회피 기동 금지
- **전진 시**: 장애물 회피 허용
- **측면 이동**: 신중하게 수행

```cpp
void obstacleAvoidance() {
    // 전방 장애물
    if (front_obstacle_distance_ < 3.0f) {
        // 정지 또는 회피
        if (moving_forward_) {
            stop_or_avoid();
        }
    }
    
    // 후방 장애물 (후진 중)
    if (rear_obstacle_distance_ < 3.0f && moving_backward_) {
        // 즉시 정지만 수행 (회피 금지)
        emergency_stop();
    }
}
```

---

### 2. 통신 손실 대응

#### 팔로워 기체

**리더 토픽 미수신 시나리오**

| 상황 | 대응 |
|-----|------|
| 1분간 미수신 | 현재 위치 호버링 |
| 2분간 미수신 | RTL 모드 대기 |
| 토픽 재수신 | 임무 재개 |

```cpp
void checkLeaderCommunication() {
    auto now = std::chrono::steady_clock::now();
    auto time_since_last = now - last_leader_msg_time_;
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(time_since_last);
    
    if (duration.count() > 60) {
        RCLCPP_WARN(get_logger(), "Leader communication lost for %ld seconds", 
                   duration.count());
        
        // 호버링
        hold_position();
        
        if (duration.count() > 120) {
            // RTL 모드
            trigger_rtl();
        }
    }
}
```

---

### 3. 배터리 안전

#### 배터리 모니터링

| 배터리 레벨 | 동작 |
|-----------|------|
| 30% 이하 | 즉시 복귀 시작 |
| 20% 이하 | 비상 복귀 (최단 경로) |
| 10% 이하 | 현장 착륙 |

#### 격발 중 배터리 부족

```cpp
void checkBatteryDuringFiring() {
    if (battery_level_ < 30.0f) {
        if (is_firing_ && current_shot_ < num_shots_) {
            // 현재 격발 완료 후 복귀
            RCLCPP_WARN(get_logger(), "Low battery during firing. Will RTL after shot %d", 
                       current_shot_);
            finish_current_shot_ = true;
            abort_remaining_shots_ = true;
        } else {
            // 즉시 복귀
            immediate_rtl_ = true;
        }
    }
}
```

---

### 4. GPS 정확도

#### RTK Fix 요구사항
- **필수**: RTK Fix 상태 (정확도 < 0.1m)
- GPS 정확도 저하 시 임무 중단

```cpp
void checkGpsAccuracy() {
    // RTK Fix 확인 (GPS Fix Type)
    // 6 = RTK Float, 7 = RTK Fixed
    if (gps_fix_type_ < 6) {
        RCLCPP_ERROR(get_logger(), "GPS accuracy insufficient. Fix type: %d", 
                    gps_fix_type_);
        
        // 임무 중단
        mission_abort_ = true;
        
        // RTL 또는 호버링
        hold_position();
    }
    
    // 수평 정확도 확인 (m)
    if (gps_eph_ > 0.5f) {
        RCLCPP_WARN(get_logger(), "GPS horizontal accuracy degraded: %.2f m", gps_eph_);
    }
}
```

---

## 설정 파라미터

### YAML 설정 파일

```yaml
# config/fire_suppression_mission.yaml

# ========== 기체 식별 ==========
vehicle_id: 1  # 1: 리더, 2: 좌측 팔로워, 3: 우측 팔로워

# ========== 거리 제어 ==========
distance_control:
  target_distance: 10.0        # 목표 거리 (m)
  distance_tolerance: 0.5      # 거리 허용 오차 (m) = 5%
  forward_speed: 0.4           # 전진 속도 (m/s)
  backward_speed: 0.25         # 후진 속도 (m/s)
  distance_stable_time: 3.0    # 거리 안정화 시간 (초)

# ========== 조준 제어 ==========
aiming_control:
  aim_tolerance: 0.05          # 조준 허용 오차 (5%)
  aim_stable_time: 1.0         # 조준 안정화 시간 (초)
  image_width: 640             # 열화상 이미지 너비 (픽셀)
  image_height: 512            # 열화상 이미지 높이 (픽셀)

# ========== 격발 제어 ==========
firing_control:
  num_shots: 6                 # 총 격발 수
  shot_interval: 5.0           # 격발 간격 (초)
  stabilization_time: 1.0      # 격발 전 안정화 (초)
  fire_signal_type: "pwm"      # 격발 신호 타입: "pwm", "gpio", "serial"
  fire_signal_pin: 14          # GPIO 핀 번호 (gpio 사용 시)
  fire_signal_duration: 0.5    # 격발 신호 지속 시간 (초)

# ========== 팔로워 옵션 ==========
follower_options:
  use_lidar_distance_control: true    # 팔로워 LiDAR 거리 제어 사용
  formation_angle_offset: 60.0        # 포메이션 각도 (도) - 정삼각형
  gps_only_mode: false                # GPS만 사용 (LiDAR 무시)

# ========== 복귀 설정 ==========
return_to_launch:
  rtl_altitude: 10.0           # 복귀 고도 (m)
  landing_speed: 0.5           # 착륙 속도 (m/s)
  approach_speed: 3.0          # 복귀 접근 속도 (m/s)

# ========== 안전 설정 ==========
safety:
  min_battery_level: 30.0      # 최소 배터리 레벨 (%)
  emergency_battery_level: 20.0 # 비상 배터리 레벨 (%)
  min_obstacle_distance: 3.0    # 최소 장애물 거리 (m)
  communication_timeout: 60.0   # 통신 타임아웃 (초)
  gps_accuracy_threshold: 0.5   # GPS 정확도 임계값 (m)

# ========== ROS2 토픽 ==========
topics:
  leader_aim_pose: "/leader_aim_pose"
  leader_firing_status: "/leader_firing_status"
  mission_status: "/vehicle_{id}/mission_status"
  lidar_distance: "/lidar/distance"
  thermal_image: "/thermal/image"
  gps_position: "/mavros/global_position/global"

# ========== PX4 MAVLink ==========
mavlink:
  system_id: 1
  component_id: 1
  target_system: 1
  target_component: 1

# ========== 디버그 옵션 ==========
debug:
  enable_logging: true
  log_level: "info"            # "debug", "info", "warn", "error"
  publish_debug_topics: true
  save_flight_log: true
```

---

### 파라미터 로드 예제

```cpp
class OffboardManager {
public:
    void loadParameters() {
        // 기체 ID (환경변수에서도 읽기 가능)
        // /home/khadas/config/device_config.env 파일에서 VEHICLE_ID 확인
        this->declare_parameter("vehicle_id", 1);
        vehicle_id_ = this->get_parameter("vehicle_id").as_int();
        
        // 환경변수에서 읽기 (선택사항)
        const char* env_vehicle_id = std::getenv("VEHICLE_ID");
        if (env_vehicle_id != nullptr) {
            vehicle_id_ = std::atoi(env_vehicle_id);
            RCLCPP_INFO(get_logger(), "Vehicle ID loaded from environment: %d", vehicle_id_);
        }
        
        // 거리 제어
        this->declare_parameter("distance_control.target_distance", 10.0);
        target_distance_ = this->get_parameter("distance_control.target_distance").as_double();
        
        this->declare_parameter("distance_control.distance_tolerance", 0.5);
        distance_tolerance_ = this->get_parameter("distance_control.distance_tolerance").as_double();
        
        this->declare_parameter("distance_control.forward_speed", 0.4);
        forward_speed_ = this->get_parameter("distance_control.forward_speed").as_double();
        
        this->declare_parameter("distance_control.backward_speed", 0.25);
        backward_speed_ = this->get_parameter("distance_control.backward_speed").as_double();
        
        this->declare_parameter("distance_control.distance_stable_time", 3.0);
        distance_stable_time_ = this->get_parameter("distance_control.distance_stable_time").as_double();
        
        // 조준 제어
        this->declare_parameter("aiming_control.aim_tolerance", 0.05);
        aim_tolerance_ = this->get_parameter("aiming_control.aim_tolerance").as_double();
        
        this->declare_parameter("aiming_control.aim_stable_time", 1.0);
        aim_stable_time_ = this->get_parameter("aiming_control.aim_stable_time").as_double();
        
        // 격발 제어
        this->declare_parameter("firing_control.num_shots", 6);
        num_shots_ = this->get_parameter("firing_control.num_shots").as_int();
        
        this->declare_parameter("firing_control.shot_interval", 5.0);
        shot_interval_ = this->get_parameter("firing_control.shot_interval").as_double();
        
        // 팔로워 옵션
        this->declare_parameter("follower_options.use_lidar_distance_control", true);
        use_lidar_for_followers_ = this->get_parameter("follower_options.use_lidar_distance_control").as_bool();
        
        this->declare_parameter("follower_options.formation_angle_offset", 120.0);
        formation_angle_offset_ = this->get_parameter("follower_options.formation_angle_offset").as_double();
        
        // 복귀 설정
        this->declare_parameter("return_to_launch.rtl_altitude", 10.0);
        rtl_altitude_ = this->get_parameter("return_to_launch.rtl_altitude").as_double();
        
        this->declare_parameter("return_to_launch.landing_speed", 0.5);
        landing_speed_ = this->get_parameter("return_to_launch.landing_speed").as_double();
        
        // 안전 설정
        this->declare_parameter("safety.min_battery_level", 30.0);
        min_battery_level_ = this->get_parameter("safety.min_battery_level").as_double();
        
        RCLCPP_INFO(get_logger(), "Parameters loaded. Vehicle ID: %d", vehicle_id_);
    }
};
```

---

## 테스트 시나리오

### 1. 단일 기체 테스트 (리더만)

#### 목적
리더 기체의 기본 기능 검증

#### 테스트 절차

```
1. QGC 미션 설정
   ├─ 이륙 고도: 10m
   ├─ 목표 지점: 화재 시뮬레이터 앞
   └─ Offboard 전환 웨이포인트 추가

2. 거리 제어 테스트
   ├─ LiDAR 데이터 확인
   ├─ 10m 거리 접근 확인
   ├─ 전진/후진 동작 확인
   └─ 안정화 (3초) 확인

3. 조준 테스트
   ├─ 열화상 이미지 확인
   ├─ Yaw 제어 확인
   ├─ 조준 중심 정렬 확인
   └─ 안정화 (1초) 확인

4. 격발 시퀀스 테스트 (신호만)
   ├─ 격발 신호 6회 전송 확인
   ├─ 5초 간격 확인
   ├─ 조준 자세 유지 확인
   └─ ROS2 토픽 발행 확인

5. 복귀 및 착륙
   ├─ 이륙 지점 복귀 확인
   ├─ 고도 10m 유지 확인
   ├─ 착륙 속도 확인
   └─ 안전 착륙 확인
```

#### 검증 항목

| 항목 | 기준 | 결과 |
|-----|------|------|
| LiDAR 거리 오차 | < 5% | ☐ |
| 조준 중심 오차 | < 5% | ☐ |
| 격발 간격 | 5초 ± 0.5초 | ☐ |
| GPS 복귀 오차 | < 1m | ☐ |
| 착륙 정밀도 | < 2m | ☐ |

---

### 2. 팔로워 포메이션 테스트 (GPS만)

#### 목적
팔로워 기체의 포메이션 계산 및 이동 검증

#### 테스트 설정
```yaml
follower_options:
  use_lidar_distance_control: false  # GPS만 사용
  formation_angle_offset: 60.0       # 정삼각형 각도
```

#### 테스트 절차

```
1. 리더 토픽 시뮬레이션
   ├─ /leader_aim_pose 발행
   ├─ 가상 리더 위치 설정
   └─ 목표물 위치 계산

2. 2번 기체 (좌측) 포메이션
   ├─ 목표 위치 계산 확인
   ├─ GPS 이동 확인
   ├─ 헤딩 정렬 확인
   └─ 위치 도달 확인 (< 1m)

3. 3번 기체 (우측) 포메이션
   ├─ 목표 위치 계산 확인
   ├─ GPS 이동 확인
   ├─ 헤딩 정렬 확인
   └─ 위치 도달 확인 (< 1m)

4. 포메이션 정확도 검증
   ├─ 리더-팔로워 거리: 10m ± 0.5m
   ├─ 각도 오차: 60도 ± 5도
   └─ 정삼각형 형태 확인
```

#### 검증 항목

| 항목 | 기준 | 2번 기체 | 3번 기체 |
|-----|------|---------|---------|
| GPS 위치 오차 | < 1m | ☐ | ☐ |
| 헤딩 오차 | < 3도 | ☐ | ☐ |
| 포메이션 각도 | 60도 ± 5도 | ☐ | ☐ |
| 목표물 거리 | 10m ± 0.5m | ☐ | ☐ |
| 리더-팔로워 거리 | 10m ± 0.5m | ☐ | ☐ |

---

### 3. 팔로워 LiDAR 테스트

#### 목적
팔로워 기체의 LiDAR 거리 제어 검증

#### 테스트 설정
```yaml
follower_options:
  use_lidar_distance_control: true  # LiDAR 사용
  formation_angle_offset: 60.0      # 정삼각형 각도
```

#### 테스트 절차

```
1. 포메이션 이동 후 LiDAR 활성화
   ├─ GPS 목표 위치 도달
   ├─ LiDAR 거리 측정 시작
   └─ 거리 제어 모드 진입

2. 거리 제어 검증
   ├─ 전진 동작 확인 (거리 > 10.5m)
   ├─ 후진 동작 확인 (거리 < 9.5m)
   ├─ 위치 고정 확인 (9.5~10.5m)
   └─ 안정화 확인 (3초)

3. 안전 후진 검증
   ├─ 후진 속도: 0.2~0.3 m/s
   ├─ 직선 후진 확인
   ├─ 회피 기동 없음 확인
   └─ 장애물 정지 확인
```

#### 검증 항목

| 항목 | 기준 | 2번 기체 | 3번 기체 |
|-----|------|---------|---------|
| LiDAR 거리 오차 | < 5% | ☐ | ☐ |
| 후진 속도 | 0.2~0.3 m/s | ☐ | ☐ |
| 안정화 시간 | 3초 | ☐ | ☐ |
| 후진 직진성 | 좋음 | ☐ | ☐ |

---

### 4. 통합 테스트 (3대 동시)

#### 목적
전체 시스템의 통합 운용 검증

#### 테스트 절차

```
1. 동시 이륙
   ├─ 3대 기체 동시 시동
   ├─ QGC 미션 동시 실행
   ├─ 동시 이륙 확인
   └─ 목표 지점 도달

2. Offboard 전환
   ├─ 리더 먼저 전환
   ├─ 팔로워 순차 전환 (2초 간격)
   └─ testMission3() 실행

3. 포메이션 형성
   ├─ 리더 거리 제어
   ├─ 팔로워 포메이션 이동
   ├─ 전체 기체 위치 확인
   └─ 삼각 포메이션 완성

4. 조준 및 격발
   ├─ 각 기체 독립 조준
   ├─ 조준 완료 확인
   ├─ 동시 또는 순차 격발
   └─ 6발 완료 확인

5. 동시 복귀
   ├─ 전체 기체 복귀 시작
   ├─ 각자 이륙 지점 복귀
   ├─ 동시 착륙
   └─ 안전 착륙 확인
```

#### 검증 항목

| 항목 | 기준 | 1번 | 2번 | 3번 |
|-----|------|-----|-----|-----|
| 포메이션 정확도 | 오차 < 1m | ☐ | ☐ | ☐ |
| 조준 정확도 | 오차 < 5% | ☐ | ☐ | ☐ |
| 격발 완료 | 6발 | ☐ | ☐ | ☐ |
| 복귀 정확도 | 오차 < 1m | ☐ | ☐ | ☐ |
| 착륙 정확도 | 오차 < 2m | ☐ | ☐ | ☐ |

---

### 5. 비상 상황 테스트

#### 목적
안전 시스템 및 비상 대응 검증

#### 테스트 시나리오

##### A. 통신 손실 (팔로워)
```
1. 정상 운용 중 리더 토픽 중단
2. 1분 후: 팔로워 호버링 확인
3. 2분 후: RTL 모드 확인
4. 토픽 재개: 임무 재개 확인
```

##### B. 배터리 부족
```
1. 격발 중 배터리 30% 이하 시뮬레이션
2. 현재 격발 완료 확인
3. 즉시 복귀 시작 확인
4. 안전 착륙 확인
```

##### C. GPS 정확도 저하
```
1. RTK Fix 손실 시뮬레이션
2. 임무 중단 확인
3. 호버링 또는 RTL 확인
4. Fix 회복: 임무 재개 판단
```

##### D. 장애물 감지
```
1. 후진 중 후방 장애물 시뮬레이션
2. 즉시 정지 확인
3. 회피 기동 없음 확인
4. 대안 경로 찾기 또는 임무 중단
```

---

## 부록

### A. 커스텀 메시지 정의

#### LeaderAimPose.msg
```msg
std_msgs/Header header

# 리더 기체 위치 및 자세
float64 latitude              # 리더 GPS 위도 (도)
float64 longitude             # 리더 GPS 경도 (도)
float32 altitude              # 리더 고도 MSL (m)
float32 yaw_deg               # 리더 헤딩 (도)

# 목표물 정보
float32 distance_to_target    # 목표물까지 거리 (m)
float64 target_latitude       # 계산된 목표물 위도 (도)
float64 target_longitude      # 계산된 목표물 경도 (도)

# 조준 상태
bool aim_locked               # 조준 완료 여부
```

#### FiringStatus.msg
```msg
std_msgs/Header header

# 격발 정보
uint8 current_shot           # 현재 격발 번호 (1~6)
bool is_firing               # 격발 진행 중
bool shot_completed          # 해당 격발 완료
```

#### MissionStatus.msg
```msg
std_msgs/Header header

# 기체 정보
uint8 vehicle_id              # 기체 번호 (1/2/3)

# 임무 상태
string mission_phase          # 현재 단계

# 센서 데이터
float32 lidar_distance        # LiDAR 거리 (m)
bool aim_locked               # 조준 상태

# 격발 정보
uint8 shots_fired             # 격발 완료 수 (0~6)
```

---

### B. 빌드 및 실행

#### CMakeLists.txt 추가

```cmake
# ROS2 메시지 의존성
find_package(humiro_msgs REQUIRED)

# 실행 파일
add_executable(fire_suppression_node src/fire_suppression_node.cpp)

ament_target_dependencies(fire_suppression_node
  rclcpp
  humiro_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
)

install(TARGETS
  fire_suppression_node
  DESTINATION lib/${PROJECT_NAME}
)
```

#### package.xml 추가

```xml
<depend>humiro_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>mavros_msgs</depend>
```

#### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select fire_suppression
source install/setup.bash
```

#### 실행

```bash
# 리더 기체
ros2 run fire_suppression fire_suppression_node --ros-args -p vehicle_id:=1

# 팔로워 기체 (2번)
ros2 run fire_suppression fire_suppression_node --ros-args -p vehicle_id:=2

# 팔로워 기체 (3번)
ros2 run fire_suppression fire_suppression_node --ros-args -p vehicle_id:=3
```

---

### C. 디버깅 팁

#### ROS2 토픽 모니터링

```bash
# 리더 조준 자세 확인
ros2 topic echo /leader_aim_pose

# 격발 상태 확인
ros2 topic echo /leader_firing_status

# 임무 상태 확인 (1번 기체)
ros2 topic echo /vehicle_1/mission_status
```

#### 로그 레벨 변경

```bash
ros2 run fire_suppression fire_suppression_node --ros-args --log-level debug
```

#### RViz 시각화

```bash
rviz2 -d ~/ros2_ws/src/fire_suppression/rviz/fire_suppression.rviz
```

---

### D. 문제 해결

#### 1. 리더 토픽이 발행되지 않음
```bash
# 토픽 리스트 확인
ros2 topic list

# 노드 상태 확인
ros2 node list

# 해결: ROS2 도메인 ID 확인
export ROS_DOMAIN_ID=0
```

#### 2. 팔로워가 포메이션을 형성하지 못함
```bash
# 리더 토픽 수신 확인
ros2 topic echo /leader_aim_pose --once

# 팔로워 로그 확인
journalctl -u fire_suppression_follower_2.service -f

# 해결: GPS 정확도 확인, 계산 로직 검증
```

#### 3. LiDAR 거리 제어가 불안정함
```bash
# LiDAR 데이터 확인
ros2 topic echo /lidar/distance

# 거리 제어 로그 확인
ros2 topic echo /vehicle_1/mission_status | grep lidar_distance

# 해결: PID 게인 튜닝, 필터링 추가
```

#### 4. 조준이 안정되지 않음
```bash
# 열화상 이미지 확인
ros2 topic echo /thermal/image

# 조준 로그 확인
ros2 run rqt_console rqt_console

# 해결: 조준 허용 오차 증가, 안정화 시간 증가
```

---

## 결론

이 문서는 화재진압 드론 스웜 시스템의 삼각 포메이션 자동 조준 및 격발 임무를 위한 상세 사양서입니다. 

### 핵심 구현 포인트
1. **안전한 후진 제어**: 급격한 회피 기동 없이 천천히 직선 후진
2. **팔로워 LiDAR 옵션**: GPS 기반 또는 GPS+LiDAR 병행
3. **정밀한 포메이션 계산**: 리더 정보 기반 자율 위치 계산
4. **조준 안정성 검증**: 연속 1초 이상 유지 확인

### 구현 요청
이 사양서를 바탕으로 **C++ 코드**를 작성해주세요.
- ROS2 통신 구조
- PX4 MAVLink 통신
- 센서 데이터 처리
- 안전 시스템
- 디버깅 로그

---

**문서 버전**: 2.0  
**작성일**: 2025-01-16  
**작성자**: 휴미로㈜ (Humiro Co., Ltd.)