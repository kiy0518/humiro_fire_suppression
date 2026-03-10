# 031 - Swarm 독립 편대 비행 설계 및 구현

## 문서 정보
- **작성일**: 2026-02-25
- **버전**: 1.0.0
- **소프트웨어 버전**: v0.24.2+
- **목적**: 기존 편대 비행(Formation)과 별개로, 각 기체가 독립적으로 비행하는 Swarm 모드 설계

---

## 1. 배경 및 목적

### 기존 편대 비행 (Formation) 문제점
- 팔로워가 리더의 위치를 10Hz로 실시간 추적하며 따라감
- 리더가 정지하면 팔로워도 정지, 리더가 회전하면 팔로워도 회전
- 편대 게이트(ROTATE 허가, NAVIGATE 허가)가 필요하여 느린 동기화
- 팔로워 수가 증가하면 동기화 지연 누적

### Swarm 모드 목표
- 각 기체가 **독립적으로** 자기 목적지까지 비행
- 리더는 핵심 시점(정렬 완료)에만 팔로워에게 알림
- 편대 게이트 없음 → 각자 독립 타이밍으로 이/착륙
- 기존 cross-track 클램프로 경로 안전 확보 유지

---

## 2. 미션 흐름

```
Phase 1: 미션 전달
  GCS → Leader (CustomMessage 60000)
  Leader → Followers: CMD_SWARM_START (목표GPS + 접근헤딩 + 고도 + 속도)
  팔로워: 접근헤딩 수직선 위에 자기 위치 계산 → 독립 비행 시작

Phase 2: 독립 비행
  모든 기체: OFFBOARD → ARM → TAKEOFF → HOVER → ROTATE → NAVIGATE
  편대 게이트 없음 (각자 독립 타이밍)
  기존 cross-track 클램프로 경로 안전 확보

Phase 3: 리더 정렬
  리더: NAVIGATE → DISTANCE_ADJUST (LiDAR) → 정렬 완료
  리더 → 팔로워: CMD_ALIGN_COMPLETE (리더GPS + 헤딩 + 화점GPS)
  리더: TRACKING_HOVER (조준 + 격발)

Phase 4: 팔로워 진압 위치 이동
  팔로워: HOVER_AT_TARGET (정렬 대기) → CMD_ALIGN_COMPLETE 수신
  팔로워: 진압 위치 계산 → NAVIGATE (2차: 진압 위치로 이동)

Phase 5: 독립 진압
  각 기체: TRACKING_HOVER (자율 조준 + 격발)
  소화탄 소진 → RTL (각자 독립)
```

### 핸들러 전환 순서

**리더:**
```
PREPARE → OFFBOARD → ARM → TAKEOFF → HOVER → ROTATE → NAVIGATE
  → DISTANCE_ADJUST (LiDAR) → HOVER_AT_TARGET → TRACKING_HOVER → RTL
```

**팔로워:**
```
PREPARE → OFFBOARD → ARM → TAKEOFF → HOVER → ROTATE → NAVIGATE(1차: 라인 위치)
  → HOVER_AT_TARGET(정렬 대기, 최대 300초)
  → [CMD_ALIGN_COMPLETE 수신] → NAVIGATE(2차: 진압 위치)
  → TRACKING_HOVER(조준/격발) → RTL
```

---

## 3. 통신 프로토콜

### 새 FormationCommand 상수

| 상수 | 값 | 방향 | 설명 |
|------|---|------|------|
| CMD_SWARM_START | 5 | 리더→팔로워 | 독립 편대 미션 시작 |
| CMD_ALIGN_COMPLETE | 6 | 리더→팔로워 | 리더 정렬 완료, 진압 위치로 이동 |

### CMD_SWARM_START 메시지 필드

| 필드 | 타입 | 설명 |
|------|------|------|
| target_latitude | float64 | 리더 목적지 GPS 위도 |
| target_longitude | float64 | 리더 목적지 GPS 경도 |
| approach_heading | float32 | 접근 헤딩 (rad) — 팔로워 목적지 계산용 |
| takeoff_altitude | float32 | 이륙/비행 고도 (m) |
| flight_speed | float32 | 비행 속도 (m/s) |

### CMD_ALIGN_COMPLETE 메시지 필드

| 필드 | 타입 | 설명 |
|------|------|------|
| target_latitude | float64 | 리더 현재 GPS 위도 |
| target_longitude | float64 | 리더 현재 GPS 경도 |
| approach_heading | float32 | 리더 현재 헤딩 (rad) — 화점 방향 |
| fire_target_latitude | float64 | 추정 화점 GPS 위도 |
| fire_target_longitude | float64 | 추정 화점 GPS 경도 |

### 전송 신뢰성
- 모든 명령 3회 반복 전송 (100ms 간격)
- WiFi 유실 대비

---

## 4. 팔로워 목적지 계산 알고리즘

### Phase 1: 독립 목적지 (1차 NAVIGATE)

팔로워는 리더 목적지에서 **접근 헤딩의 수직 방향**으로 `offset_right` 만큼 이동한 위치를 자기 목적지로 설정:

```
perp_heading = approach_heading + π/2

dest_lat = leader_target_lat + (offset_m × cos(perp_heading)) / 111320
dest_lon = leader_target_lon + (offset_m × sin(perp_heading)) / (111320 × cos(lat))
```

→ 모든 기체가 화점을 향해 같은 라인에 도착

### Phase 4: 진압 위치 (2차 NAVIGATE)

리더의 최종 위치/헤딩을 기준으로, 헤딩 수직 방향에 `offset_right` 배치:

```
perp = leader_heading + π/2

suppress_lat = leader_lat + (offset_m × cos(perp)) / 111320
suppress_lon = leader_lon + (offset_m × sin(perp)) / (111320 × cos(lat))
```

→ 모든 기체가 화점을 향해 일렬로 서게 됨

---

## 5. 수정 파일 목록

| 파일 | 변경 사항 |
|------|----------|
| `humiro_msgs/msg/FormationCommand.msg` | CMD_SWARM_START=5, CMD_ALIGN_COMPLETE=6, 새 필드 3개 |
| `navigation/src/offboard/mission_context.h` | swarm_mode, swarm_alignment_received 등 6개 필드 |
| `navigation/src/offboard/offboard_manager.h` | executeMissionSwarm(), setSwarmSuppressTarget(), swarm_mode_ |
| `navigation/src/offboard/offboard_manager.cpp` | executeMissionSwarm(), setSwarmSuppressTarget(), advanceToNextHandler 수정 |
| `navigation/src/offboard/handlers/hover_at_target_handler.h` | swarm 정렬 대기 로직 (최대 300초) |
| `navigation/src/offboard/formation/formation_controller.h` | startSwarmMission(), notifyAlignmentComplete() 등 |
| `navigation/src/offboard/formation/formation_controller.cpp` | swarm 전체 로직 구현 (~200 LOC) |
| `application/src/application_manager.h` | executeSwarmMission(), readMissionModeString() |
| `application/src/application_manager.cpp` | executeSwarmMission(), CMD_SWARM_START 콜백, 60000 핸들러 수정 |

---

## 6. 설정

### offboard_config.json

```json
{
    "mission_mode": "swarm"
}
```

| 값 | 설명 |
|---|------|
| "solo" | 단독 비행 (기존) |
| "formation" | 편대 비행 — 팔로워가 리더 추적 (기존) |
| "swarm" | 독립 편대 — 각 기체 독립 비행 (신규) |

### device_config.env (팔로워)

기존 설정 그대로 사용:
- `FORMATION_OFFSET_RIGHT`: 리더 기준 우측 오프셋 (cm)
- `FORMATION_OFFSET_BEHIND`: (swarm에서는 미사용)
- `FORMATION_OFFSET_ABOVE`: (swarm에서는 미사용)

---

## 7. 기존 모드와의 호환성

| 항목 | Formation 모드 | Swarm 모드 |
|------|---------------|------------|
| 팔로워 추적 | 10Hz 실시간 (LeaderPose) | 독립 비행 |
| 편대 게이트 | ROTATE/NAVIGATE 허가 | 없음 |
| 리더→팔로워 명령 | CMD_FOLLOW | CMD_SWARM_START |
| 목적지 계산 | 리더 현재 위치 + 오프셋 | 접근 헤딩 수직선 + offset_right |
| SUPPRESS 전환 | CMD_SUPPRESS + 미러링 | CMD_ALIGN_COMPLETE |
| DISTANCE_ADJUST | 리더만 | 리더만 (동일) |
| RTL | CMD_RTL 동기화 | 각자 독립 |
| cross-track 클램프 | 사용 | 사용 (경로 안전) |

---

## 8. 안전 기능

1. **정렬 대기 타임아웃**: 팔로워가 CMD_ALIGN_COMPLETE를 300초 내 미수신 시 ABORT_RTL
2. **소화탄 소진 RTL**: 각 기체 독립적으로 소화탄 소진 시 2초 후 RTL
3. **cross-track 클램프**: 독립 비행 중에도 경로 횡단 방지 유지
4. **heartbeat 모니터링**: 리더 heartbeat 3초 타임아웃 시 HOLD
5. **명령 3회 재전송**: WiFi 유실 대비 신뢰성 확보

---

## 9. 검증 계획

### SITL 3대 테스트
1. offboard_config.json에 `"mission_mode": "swarm"` 설정
2. QGC에서 drone1(리더)에게 60000 미션 전송
3. 확인 사항:
   - [ ] 팔로워가 CMD_SWARM_START 수신 + 독립 목적지 계산 로그
   - [ ] 모든 기체가 독립적으로 이륙/회전/비행
   - [ ] 팔로워가 리더와 같은 라인(수직 오프셋)에 도착
   - [ ] 리더 DISTANCE_ADJUST 완료 시 CMD_ALIGN_COMPLETE 전송
   - [ ] 팔로워가 진압 위치로 재이동
   - [ ] 각 기체가 TRACKING_HOVER → 격발 → RTL 독립 수행

### 기존 모드 호환 테스트
4. offboard_config.json에 `"mission_mode": "formation"` 설정
5. 기존 편대 모드 정상 동작 확인

---

## 10. 멀티 드론 배포 가이드

### 배포 절차 (드론 #1 → 드론 #2)

```bash
# 1. 소스 코드 동기화 (build, install, config 제외 — config는 기체별 고유!)
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' --exclude='.git/' \
  --exclude='config/device_config.env' --exclude='config/app_config*.env' \
  /home/khadas/humiro_fire_suppression/ \
  khadas@192.168.100.21:/home/khadas/humiro_fire_suppression/

# 2. humiro_msgs install 직접 복사 (msg 변경 시 필수)
rsync -avz /home/khadas/humiro_fire_suppression/install/humiro_msgs/ \
  khadas@192.168.100.21:/home/khadas/humiro_fire_suppression/install/humiro_msgs/

# 3. 드론 #2에서 application 빌드 (cmake 캐시 정리 포함)
ssh khadas@192.168.100.21 'rm -f /home/khadas/humiro_fire_suppression/application/build/CMakeCache.txt && \
  /home/khadas/humiro_fire_suppression/application/build.sh'
```

### COLCON_IGNORE 이슈 (주의)

프로젝트의 `build/`와 `install/` 디렉토리에 `COLCON_IGNORE` 파일이 존재함.
이로 인해 `cd /home/khadas/humiro_fire_suppression && colcon build --packages-select humiro_msgs`를 실행해도
colcon이 프로젝트 내부 `build/`, `install/`을 건너뛰고 **`~/build/`, `~/install/`에 빌드 결과를 출력**함.

**결과:** `build.sh`는 `$PROJECT_ROOT/install/humiro_msgs`를 참조하므로, colcon이 `~/install/`에 빌드한 새 헤더를 찾지 못해 컴파일 에러 발생.

**현재 해결책:** 드론 #1에서 빌드된 `install/humiro_msgs/`를 rsync로 직접 복사.

**향후 개선:** 배포 스크립트(`scripts/deploy.sh`) 작성 필요:
- 소스 rsync + install/humiro_msgs rsync + cmake 캐시 정리 + build.sh 자동 실행
- 대상 드론 IP 자동 계산 (DRONE_ID 기반)

### device_config.env 덮어쓰기 사고 (주의!)

**원인:** rsync 시 `config/device_config.env`를 제외하지 않으면, 드론 #1의 설정(DRONE_ID=1, ROLE=Leader)이
다른 기체에 덮어씌워져 모든 기체가 동일 ID/역할로 동작 → **CustomMessage 포트 충돌 + 통신 불가**.

**증상:** 팔로워 기체의 GUI(http://192.168.100.21:5000)에 "1번 기체"로 표시, 커스텀 메시지 미수신.

**해결:** 각 기체의 `config/backup/` 디렉토리에서 원래 설정 복원 후 `003-apply_config.sh` 재실행.

**예방:** rsync 시 반드시 `--exclude='config/device_config.env' --exclude='config/app_config*.env'` 포함.

### 드론 IP 공식

| 드론 | WiFi IP | eth0 IP |
|------|---------|---------|
| #1 | 192.168.100.11 | 10.0.0.11 |
| #2 | 192.168.100.21 | 10.0.0.21 |
| #3 | 192.168.100.31 | 10.0.0.31 |

공식: `192.168.100.{N*10+1}`, `10.0.0.{N*10+1}`

---

## 11. 향후 개선사항

- Swarm 모드에서 팔로워 간 충돌 방지 (현재는 offset_right으로만 분리)
- CMD_ALIGN_COMPLETE 미수신 시 폴백 전략 (현재: 300초 타임아웃 → RTL)
- 팔로워 독립 DISTANCE_ADJUST (현재: 리더만 수행)
- Gate 4 (SUPPRESS→RTL) 동기화 — 모든 기체 동시 RTL 옵션
- 멀티 드론 배포 스크립트 자동화 (`scripts/deploy.sh`)

---

## 문서 이력

| 버전 | 날짜 | 작성자 | 변경 내용 |
|------|------|--------|----------|
| 1.0.0 | 2026-02-25 | Claude | 최초 작성 |
| 1.0.1 | 2026-02-25 | Claude | 멀티 드론 배포 가이드 + COLCON_IGNORE 이슈 추가 |
