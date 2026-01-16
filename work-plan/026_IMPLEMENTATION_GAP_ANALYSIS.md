# 026 - Work-Plan vs Implementation Gap Analysis Report

## 문서 정보
- **작성일**: 2026-01-16
- **버전**: 1.0.0
- **목적**: work-plan 문서와 실제 구현 코드 간의 차이 분석 및 개선사항 도출

---

## 1. 요약 (Executive Summary)

| 항목 | 완료율 | 상태 |
|------|--------|------|
| **전체 진행률** | ~60% | 진행 중 |
| **완전 구현** | 9개 기능 | 안정 |
| **부분 구현** | 7개 기능 | 보완 필요 |
| **미구현** | 6개 기능 | 작업 필요 |

---

## 2. 상세 분석

### 001 - `isArmed()` 함수 미구현 [CRITICAL]

**위치**: `outdoor_mission_manager.cpp:950-952`

**문제**:
```cpp
bool OutdoorMissionManager::isArmed() {
    // TODO: VehicleStatus에서 확인
    return true;  // 임시
}
```

**계획 (024_01_OFFBOARD_MODE_SEQUENCE_V2.md)**:
- VehicleStatus 토픽 구독하여 arming_state 확인 필요
- 시동 상태 미확인 시 미션 진행 불가해야 함

**현재 상태**: 항상 `true` 반환 (위험)

**영향도**: 🔴 CRITICAL
- 시동 해제 상태에서도 미션 진행 가능 (안전 위험)
- FC 상태와 불일치 발생 가능

**수정 방안**:
```cpp
// VehicleStatus 구독 추가 필요
rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
std::atomic<uint8_t> arming_state_{0};

bool OutdoorMissionManager::isArmed() {
    return arming_state_ == 2;  // ARMING_STATE_ARMED
}
```

---

### 002 - 충돌 회피 시스템 미구현 [CRITICAL]

**계획 문서**: `025_COLLISION_AVOIDANCE_DESIGN.md`

**계획된 기능**:
- 8m 경고, 5m 위험, 10m 안전 거리 임계값
- 5m 고도 차이 안전 판정
- 우선순위 기반 회피 결정
- 실시간 위치 모니터링

**현재 상태**: ❌ 완전 미구현

**영향도**: 🔴 CRITICAL
- 다중 드론 운용 시 충돌 위험
- 스웜 비행 시 안전 보장 불가

**필요 작업**:
1. `CollisionAvoidanceManager` 클래스 생성
2. 타 드론 위치 ROS2 구독
3. 백그라운드 모니터링 스레드
4. holdPosition() 긴급 정지 연동

---

### 003 - 편대 비행 컨트롤러 미구현 [HIGH]

**계획 문서**: `013_NEXT_STEPS_FORMATION_CONTROL.md`

**계획된 기능**:
- FormationMember 클래스
- FormationLeader 클래스
- 목표 할당 알고리즘
- 리더 선출 로직
- 진행 상황 모니터링

**현재 상태**: ⏳ 기본 위치 계산만 구현 (~20%)

**구현된 부분**:
```cpp
// outdoor_mission_manager.cpp:583-613
GpsPosition OutdoorMissionManager::calculateFormationPosition() {
    float angle_offset = (vehicle_id_ == 2) ? 135.0f : -135.0f;
    // ...
}
```

**미구현 부분**:
- 동적 편대 재구성
- 팔로워 상태 모니터링
- 장애 발생 시 리더 교체

**영향도**: 🟠 HIGH

---

### 004 - 거리 제어 PID 미적용 [MEDIUM]

**위치**: `outdoor_mission_manager.cpp:255-328`

**계획 (024_OFFBOARD_MODE_SEQUENCE.md)**:
- PID 컨트롤러 사용
- 부드러운 속도 전이
- 오버슈트 방지

**현재 구현**:
```cpp
if (error > 0) {
    float vx = std::min(config_.forward_speed, error * 0.5f);
    // 단순 비례 제어 (P만 적용)
}
```

**문제점**:
- I, D 항 없음 → 정상상태 오차 가능
- 속도 전이 급격함
- 진동 발생 가능성

**영향도**: 🟡 MEDIUM

**수정 방안**:
```cpp
// PID 컨트롤러 추가
class PIDController {
    float kp = 0.5f, ki = 0.1f, kd = 0.05f;
    float integral = 0.0f, prev_error = 0.0f;

    float calculate(float error, float dt) {
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};
```

---

### 005 - 열화상 조준 Pitch/Roll 미적용 [MEDIUM]

**위치**: `outdoor_mission_manager.cpp:330-391`

**계획 (024_01_OFFBOARD_MODE_SEQUENCE_V2.md)**:
- Yaw 조정 (좌우)
- 고도 조정 (상하)
- Pitch/Roll 미세 조정 (옵션)

**현재 구현**:
```cpp
// Yaw 조정 (좌우)
float yaw_adj = -error_x * 10.0f;

// 고도 조정 (상하)
float alt_adj = -error_y * 0.5f;
```

**미구현**: Pitch/Roll 미세 조정

**영향도**: 🟡 MEDIUM
- 호버링 상태에서는 큰 영향 없음
- 정밀 조준 시 정확도 저하 가능

---

### 006 - 팔로워 LiDAR 거리 제어 미완성 [MEDIUM]

**위치**: `outdoor_mission_manager.cpp:518-524`

**현재 구현**:
```cpp
// 거리 조정 및 조준
current_phase_ = MissionPhase::DISTANCE_ADJ;
// LiDAR 거리 조정 (옵션)  <-- 미구현
// 열화상 조준
current_phase_ = MissionPhase::AIMING;
```

**문제점**:
- 팔로워는 거리 조정 없이 바로 조준 단계로 이동
- 정확한 10m 거리 유지 불가

**영향도**: 🟡 MEDIUM

---

### 007 - 발사 메커니즘 미구현 [HIGH]

**계획 문서**: `001_PROJECT_MASTER_PLAN.md`

**계획된 기능**:
- GPIO 핀 제어 (서보 모터)
- 발사 트리거 신호
- 재장전 로직
- 안전 인터록

**현재 상태**: ❌ 완전 미구현

**현재 구현**:
```cpp
void OutdoorMissionManager::sendFireSignal(uint8_t shot_number) {
    RCLCPP_INFO(node_->get_logger(), "[OutdoorMission] 격발 신호: %d", shot_number);
    if (send_fire_signal_) {
        send_fire_signal_(shot_number);  // 콜백만 호출
    }
}
```

**영향도**: 🟠 HIGH
- 실제 발사 불가
- 테스트는 시뮬레이션으로만 가능

---

### 008 - AI 화재 감지 미구현 [FUTURE]

**계획 문서**: `001_PROJECT_MASTER_PLAN.md` (Phase 5)

**계획된 기능**:
- CNN/YOLO 기반 화재 감지 모델
- 실시간 추론 파이프라인
- 화염/연기 분류
- 데이터셋 구축

**현재 상태**: ❌ 미구현 (Phase 5, 향후 계획)

**영향도**: 🔵 FUTURE

---

### 009 - 워치독 타이머 미적용 [MEDIUM]

**문제**: 미션 각 단계에 타임아웃 없음

**현재 구현**:
```cpp
bool OutdoorMissionManager::waitUntil(std::function<bool()> condition, float timeout_sec) {
    float elapsed = 0.0f;
    while (!condition() && elapsed < timeout_sec && !stop_requested_) {
        sleepMs(100);
        elapsed += 0.1f;
    }
    return condition();
}
```

**문제점**:
- 개별 단계에 waitUntil 미적용 부분 존재
- 무한 루프 가능성

**영향도**: 🟡 MEDIUM

---

### 010 - 지오펜싱 미구현 [MEDIUM]

**계획 문서**: 미명시 (일반적 안전 요구사항)

**현재 상태**: ❌ 미구현

**필요 기능**:
- 비행 금지 구역 설정
- 최대 비행 거리 제한
- 고도 제한

**영향도**: 🟡 MEDIUM

---

### 011 - 배터리 페일세이프 미구현 [MEDIUM]

**계획 문서**: `001_PROJECT_MASTER_PLAN.md`

**계획된 기능**:
- 30% → RTL 경고
- 20% → 긴급 착륙

**현재 상태**: ❌ 미구현

**영향도**: 🟡 MEDIUM

---

### 012 - LTE 통신 페일오버 미구현 [FUTURE]

**계획 문서**: `001_PROJECT_MASTER_PLAN.md` (Phase 5)

**현재 상태**: ❌ 미구현 (향후 계획)

**영향도**: 🔵 FUTURE

---

## 3. 파라미터 불일치

| 파라미터 | 계획 값 | 실제 코드 | 상태 |
|----------|---------|-----------|------|
| 목표 거리 | 10.0m ± 5% | 설정 가능 (기본 10m) | ✅ 일치 |
| 거리 체크 주기 | 10Hz | 100ms (10Hz) | ✅ 일치 |
| 조준 안정 시간 | 1초 | 설정 가능 (기본 1초) | ✅ 일치 |
| 거리 안정 시간 | 3초 | 설정 가능 (기본 3초) | ✅ 일치 |
| 전진 속도 | 0.3-0.5 m/s | 0.4 m/s | ✅ 일치 |
| 후진 속도 | 0.2-0.3 m/s | 0.25 m/s | ✅ 일치 |
| 격발 간격 | 5초 | 설정 가능 (기본 5초) | ✅ 일치 |
| 편대 각도 | 60° | 60° | ✅ 일치 |
| RTL 고도 | 10m | 설정 가능 (기본 10m) | ✅ 일치 |
| 위치 허용 오차 | 수평 1m, 수직 0.5m | 3D 거리 1.0m | ⚠️ 미소 차이 |

---

## 4. 우선순위별 작업 목록

### 🔴 긴급 (CRITICAL) - 즉시 수정 필요

| 번호 | 항목 | 파일 | 예상 소요 |
|------|------|------|-----------|
| 001 | isArmed() 함수 구현 | outdoor_mission_manager.cpp | 2시간 |
| 002 | 충돌 회피 기본 구현 | 신규 파일 필요 | 2일 |

### 🟠 높음 (HIGH) - 단기 해결

| 번호 | 항목 | 파일 | 예상 소요 |
|------|------|------|-----------|
| 003 | 편대 컨트롤러 구현 | 신규 파일 필요 | 3일 |
| 007 | 발사 메커니즘 구현 | GPIO 드라이버 | 2일 |

### 🟡 보통 (MEDIUM) - 중기 개선

| 번호 | 항목 | 파일 | 예상 소요 |
|------|------|------|-----------|
| 004 | 거리 제어 PID 적용 | outdoor_mission_manager.cpp | 4시간 |
| 005 | Pitch/Roll 조준 추가 | outdoor_mission_manager.cpp | 4시간 |
| 006 | 팔로워 LiDAR 제어 | outdoor_mission_manager.cpp | 4시간 |
| 009 | 워치독 타이머 적용 | 전체 미션 함수 | 3시간 |
| 010 | 지오펜싱 구현 | 신규 모듈 | 1일 |
| 011 | 배터리 페일세이프 | application_manager.cpp | 4시간 |

### 🔵 향후 (FUTURE) - 장기 계획

| 번호 | 항목 | 설명 | 예상 소요 |
|------|------|------|-----------|
| 008 | AI 화재 감지 | Phase 5 | 4주 |
| 012 | LTE 통신 페일오버 | Phase 5 | 2주 |

---

## 5. 최적화 권장사항

### 5.1 성능 최적화

1. **열화상 중심 계산 콜백화**
   - 현재: 100ms 폴링
   - 개선: 이벤트 기반 콜백으로 지연 감소

2. **편대 위치 캐싱**
   - 현재: 매 100ms 재계산
   - 개선: 변경 시에만 업데이트

3. **ROS2 메시지 배치 처리**
   - 현재: 개별 토픽 다수 구독
   - 개선: 단일 집계 토픽 사용 고려

### 5.2 코드 품질 개선

1. **에러 복구 메커니즘**
   - 센서 실패 시 대체 로직
   - 통신 두절 시 자율 복귀

2. **상태 기계 명확화**
   - MissionPhase 전이 조건 문서화
   - 각 상태별 진입/탈출 조건 정의

3. **로깅 개선**
   - 디버그 레벨 세분화
   - 파일 로깅 추가

---

## 6. 결론

### 현재 완성도
- **핵심 비행 기능**: 90% 완료
- **스웜 통신**: 85% 완료
- **안전 기능**: 20% 완료
- **발사 시스템**: 0% 완료

### 즉시 조치 필요 사항
1. `isArmed()` 함수 수정 (안전 최우선)
2. 충돌 회피 시스템 기본 구현

### 다음 마일스톤 목표
- 편대 비행 컨트롤러 완성
- 발사 메커니즘 통합
- 통합 테스트 수행

---

## 7. 문서 이력

| 버전 | 날짜 | 작성자 | 변경 내용 |
|------|------|--------|-----------|
| 1.0.0 | 2026-01-16 | Claude | 최초 작성 |

---

*이 문서는 자동 분석 도구를 통해 생성되었으며, 실제 테스트 결과와 차이가 있을 수 있습니다.*
