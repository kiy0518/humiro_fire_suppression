# 다음 단계: 편대 제어 시스템 구현

**작성일**: 2026-01-02  
**우선순위**: 최우선 🔥  
**예상 기간**: 10일

---

## 개요

하이브리드 편대 제어 시스템을 구현합니다:
- **리더**: 화재 지점 분석 + 목표 할당
- **모든 드론**: 할당된 목표 독립 실행

---

## Phase 2: 편대 통신 모듈 (3일)

### 목표
모든 드론이 편대 내에서 상태를 공유하고 목표를 수신할 수 있도록 통신 모듈 구현

### 구현 항목

#### 1. FormationMember 클래스

**파일**: `navigation/src/offboard/communication/formation_member.{h,cpp}`

**기능**:
- ROS2 Publisher: 자신의 상태 발행
- ROS2 Subscriber: 목표 할당 수신
- 주기적 상태 업데이트 (1Hz)

**코드 구조**:
```cpp
class FormationMember {
public:
    FormationMember(rclcpp::Node::SharedPtr node, uint8_t drone_id);
    
    // 상태 발행
    void publishStatus(const MemberStatus& status);
    
    // 목표 수신 확인
    bool hasNewTarget() const;
    TargetAssignment getTarget();
    
    // 상태 업데이트
    void updatePosition(const GPSCoordinate& pos);
    void updateBattery(float percent);
    void updateState(MissionState state);
    
private:
    void targetAssignmentCallback(const TargetAssignment::SharedPtr msg);
    
    rclcpp::Node::SharedPtr node_;
    uint8_t drone_id_;
    
    rclcpp::Publisher<MemberStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<TargetAssignment>::SharedPtr target_sub_;
    
    MemberStatus current_status_;
    TargetAssignment current_target_;
    bool has_new_target_;
};
```

**ROS2 토픽**:
- 발행: `/formation/member_status` (전체 브로드캐스트)
- 구독: `/formation/target_assignment/{drone_id}` (개별 수신)

**예상 코드**: ~300 LOC

---

#### 2. 데이터 구조체

**파일**: `navigation/src/offboard/communication/formation_types.h`

```cpp
// 편대원 상태
struct MemberStatus {
    uint8_t drone_id;           // 드론 ID (1-255)
    GPSCoordinate position;     // 현재 위치
    float battery_percent;      // 배터리 잔량 (0-100)
    uint8_t ammo_count;         // 소화탄 잔량 (0-6)
    MissionState state;         // 현재 상태
    uint8_t target_id;          // 할당된 목표 ID (0=없음)
    uint64_t timestamp;         // 타임스탬프 (ms)
};

// 목표 할당
struct TargetAssignment {
    uint8_t drone_id;           // 대상 드론 ID
    uint8_t target_id;          // 목표 ID
    GPSCoordinate position;     // 목표 위치
    float priority;             // 우선순위 (0-1)
    uint64_t timestamp;         // 타임스탬프
};

// 진행 상황 보고
struct ProgressReport {
    uint8_t drone_id;
    uint8_t target_id;
    ProgressStatus status;      // ASSIGNED, IN_PROGRESS, COMPLETED, FAILED
    uint8_t ammo_used;          // 사용한 소화탄 수
    uint64_t timestamp;
};
```

**예상 코드**: ~100 LOC

---

#### 3. 테스트 프로그램

**파일**: `navigation/src/offboard/test_formation_comm.cpp`

**테스트 시나리오**:
1. 2대 드론 시뮬레이션
2. 드론 1: 상태 발행
3. 드론 2: 상태 수신 확인
4. 리더: 목표 할당 발행
5. 드론 1,2: 목표 수신 확인

**예상 코드**: ~200 LOC

---

### Day 1: 데이터 구조체 + FormationMember 기본 구조

- [ ] formation_types.h 작성
- [ ] formation_member.h 작성
- [ ] formation_member.cpp 기본 구조

---

### Day 2: FormationMember 구현

- [ ] ROS2 Publisher/Subscriber 구현
- [ ] 상태 업데이트 메서드
- [ ] 목표 수신 콜백
- [ ] CMakeLists.txt 업데이트

---

### Day 3: 테스트 및 검증

- [ ] test_formation_comm.cpp 작성
- [ ] 빌드 및 테스트
- [ ] 버그 수정
- [ ] 문서 작성

---

## Phase 3: 리더 조율 로직 (4일)

### 목표
리더 드론이 화재 지점을 분석하고 최적의 목표를 각 드론에 할당

### 구현 항목

#### 1. FormationLeader 클래스

**파일**: `navigation/src/offboard/formation/formation_leader.{h,cpp}`

**기능**:
- 편대원 상태 모니터링
- 화재 지점 목록 관리
- 최적 매칭 알고리즘
- 목표 할당 발행
- 진행 상황 추적

**코드 구조**:
```cpp
class FormationLeader {
public:
    FormationLeader(rclcpp::Node::SharedPtr node, uint8_t leader_id);
    
    // 화재 지점 설정
    void setFirePoints(const std::vector<GPSCoordinate>& points);
    
    // 목표 할당 실행
    bool assignTargets();
    
    // 진행 상황 확인
    std::vector<ProgressReport> getProgress() const;
    
    // 모든 미션 완료 확인
    bool isAllComplete() const;
    
private:
    void memberStatusCallback(const MemberStatus::SharedPtr msg);
    void progressReportCallback(const ProgressReport::SharedPtr msg);
    
    // 최적 매칭 알고리즘
    std::vector<TargetAssignment> computeOptimalAssignment(
        const std::vector<MemberStatus>& members,
        const std::vector<GPSCoordinate>& targets);
    
    // 거리 계산
    double calculateDistance(const GPSCoordinate& a, const GPSCoordinate& b);
    
    rclcpp::Node::SharedPtr node_;
    uint8_t leader_id_;
    
    rclcpp::Subscription<MemberStatus>::SharedPtr member_status_sub_;
    rclcpp::Subscription<ProgressReport>::SharedPtr progress_sub_;
    rclcpp::Publisher<TargetAssignment>::SharedPtr assignment_pub_;
    
    std::map<uint8_t, MemberStatus> member_states_;  // drone_id -> status
    std::vector<GPSCoordinate> fire_points_;
    std::map<uint8_t, ProgressReport> progress_map_;
};
```

**예상 코드**: ~400 LOC

---

#### 2. 최적 매칭 알고리즘

**알고리즘**: Hungarian Algorithm (헝가리안 알고리즘) 간소화 버전

**단계**:
1. 비용 행렬 계산 (거리 기반)
   ```
   Cost[i][j] = distance(drone[i], target[j])
   ```

2. 사용 가능한 드론 필터링
   - 배터리 > 30%
   - 소화탄 > 0
   - 상태 = IDLE 또는 COMPLETED

3. Greedy 할당 (간단한 버전)
   ```
   for each target:
       find nearest available drone
       assign target to drone
       mark drone as assigned
   ```

4. 목표 할당 발행

**예상 코드**: ~150 LOC

---

#### 3. 리더 선출 로직

**파일**: `navigation/src/offboard/formation/leader_election.{h,cpp}`

**알고리즘**: 우선순위 기반
```cpp
Priority = (battery * 0.4) + (ammo * 0.3) + (gps_quality * 0.3)
```

**단계**:
1. 모든 드론이 자신의 우선순위 계산
2. `/formation/priority` 토픽에 발행
3. 가장 높은 우선순위 드론이 리더
4. 타임아웃 (5초) 후 리더 확정

**예상 코드**: ~200 LOC

---

#### 4. 테스트 프로그램

**파일**: `navigation/src/offboard/test_formation_leader.cpp`

**테스트 시나리오**:
1. 3대 드론 + 3개 화재 지점
2. 리더 선출
3. 목표 할당
4. 진행 상황 모니터링
5. 완료 확인

**예상 코드**: ~250 LOC

---

### Day 4: FormationLeader 기본 구조

- [ ] formation_leader.h 작성
- [ ] formation_leader.cpp 기본 구조
- [ ] 편대원 상태 모니터링

---

### Day 5: 최적 매칭 알고리즘

- [ ] 비용 행렬 계산
- [ ] Greedy 할당 알고리즘
- [ ] 목표 할당 발행

---

### Day 6: 리더 선출 로직

- [ ] leader_election.{h,cpp} 작성
- [ ] 우선순위 계산
- [ ] 리더 확정 로직

---

### Day 7: 테스트 및 통합

- [ ] test_formation_leader.cpp 작성
- [ ] 빌드 및 테스트
- [ ] 버그 수정
- [ ] 문서 작성

---

## Phase 4: 통합 테스트 (3일)

### 목표
실제 시나리오에서 편대 제어 시스템 검증

### 테스트 시나리오

#### Scenario 1: 단일 드론 (이미 완료 ✅)
- test_mission.cpp
- 1대 드론, 1개 목표
- 전체 미션 시퀀스

---

#### Scenario 2: 2대 편대

**파일**: `navigation/src/offboard/test_formation_2drones.cpp`

**시나리오**:
1. 드론 1 (리더), 드론 2 (팔로워)
2. 2개 화재 지점 설정
3. 리더 선출
4. 목표 할당
5. 동시 미션 실행
6. 진행 상황 모니터링
7. 모두 RTL

**예상 코드**: ~300 LOC

---

#### Scenario 3: 3대 편대

**파일**: `navigation/src/offboard/test_formation_3drones.cpp`

**시나리오**:
1. 3대 드론 (1 리더 + 2 팔로워)
2. 3개 화재 지점
3. 리더 선출
4. 최적 목표 할당
5. 동시 미션 실행
6. 진행 상황 모니터링
7. 모두 RTL

**예상 코드**: ~350 LOC

---

#### Scenario 4: 장애 시나리오

**파일**: `navigation/src/offboard/test_formation_failure.cpp`

**시나리오**:
1. 3대 드론, 3개 목표
2. 드론 2가 미션 중 배터리 부족
3. 드론 2 RTL
4. 리더가 드론 2의 목표를 재할당
5. 나머지 드론 계속 진행

**예상 코드**: ~350 LOC

---

### Day 8: 2대 편대 테스트

- [ ] test_formation_2drones.cpp 작성
- [ ] 실행 및 디버깅
- [ ] 성능 측정

---

### Day 9: 3대 편대 + 장애 테스트

- [ ] test_formation_3drones.cpp 작성
- [ ] test_formation_failure.cpp 작성
- [ ] 실행 및 디버깅

---

### Day 10: 문서화 및 정리

- [ ] 테스트 결과 문서화
- [ ] README 업데이트
- [ ] 사용 가이드 작성
- [ ] Git commit + tag (v1.2-formation-control)

---

## 전체 일정

```
Week 1:
├── Day 1: FormationMember 기본 구조
├── Day 2: FormationMember 구현
├── Day 3: 통신 테스트
├── Day 4: FormationLeader 기본 구조
├── Day 5: 최적 매칭 알고리즘
├── Day 6: 리더 선출 로직
└── Day 7: 리더 테스트

Week 2:
├── Day 8: 2대 편대 테스트
├── Day 9: 3대 편대 + 장애 테스트
└── Day 10: 문서화 및 정리
```

---

## 예상 코드량

| 모듈 | LOC |
|------|-----|
| formation_types.h | 100 |
| formation_member.{h,cpp} | 300 |
| formation_leader.{h,cpp} | 400 |
| leader_election.{h,cpp} | 200 |
| test_formation_comm.cpp | 200 |
| test_formation_leader.cpp | 250 |
| test_formation_2drones.cpp | 300 |
| test_formation_3drones.cpp | 350 |
| test_formation_failure.cpp | 350 |
| **합계** | **~2,450 LOC** |

---

## 성공 기준

### Phase 2 완료 조건
- [x] FormationMember 클래스 구현
- [x] 상태 발행/수신 동작
- [x] 목표 할당 수신 동작
- [x] test_formation_comm 성공

### Phase 3 완료 조건
- [x] FormationLeader 클래스 구현
- [x] 최적 매칭 알고리즘 동작
- [x] 리더 선출 동작
- [x] test_formation_leader 성공

### Phase 4 완료 조건
- [x] 2대 편대 미션 성공
- [x] 3대 편대 미션 성공
- [x] 장애 시나리오 대응 성공
- [x] 모든 테스트 통과

---

## 다음 다음 단계 (Phase 5)

**발사 메커니즘 통합** (5일)
- GPIO 제어
- 타겟팅 통합
- 발사 관리자
- 재조준 로직

**예상 완료**: Phase 2-4 완료 후 2주

---

**작성일**: 2026-01-02  
**예상 시작**: 2026-01-03  
**예상 완료**: 2026-01-12  
**다음 Git Tag**: v1.2-formation-control

