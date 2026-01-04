# 커스텀 MAVLink 메시지 설계 및 사용 가이드

**작성일**: 2026-01-03  
**최종 수정일**: 2026-01-03  
**버전**: v2.0  
**목적**: 화재 진압 미션 전용 커스텀 MAVLink 메시지 설계, 정의 및 라이브러리 사용 가이드 (통합 문서)

---

## 개요

이 문서는 QGC와 VIM4 간 화재 진압 미션을 위한 커스텀 MAVLink 메시지 설계를 설명합니다. 기존 MAVLink 표준 메시지는 드론 기본 제어에 사용하고, 커스텀 메시지는 화재 진압 미션에 특화된 기능만 담당합니다.

---

## 역할 분리

### 기존 MAVLink 사용 (표준 메시지)

다음 기능들은 기존 MAVLink 표준 메시지를 사용합니다:

| 기능 | MAVLink 명령/메시지 | 설명 |
|------|-------------------|------|
| ARM/DISARM | `MAV_CMD_COMPONENT_ARM_DISARM` | 시동 켜기/끄기 |
| TAKEOFF | `MAV_CMD_NAV_TAKEOFF` | 이륙 |
| LAND | `MAV_CMD_NAV_LAND` | 착륙 |
| RTL | `MAV_CMD_NAV_RETURN_TO_LAUNCH` | 출발지 복귀 |
| 위치 이동 | `MISSION_ITEM` 또는 `SET_POSITION_TARGET` | 목표 위치로 이동 |
| 배터리 상태 | `BATTERY_STATUS` | 배터리 정보 |
| GPS 위치 | `GLOBAL_POSITION_INT` | GPS 좌표 |
| 자세 정보 | `ATTITUDE` | 롤/피치/요 |
| 속도 | `VFR_HUD` | 속도 정보 |

**장점**:
- QGC에서 기본 지원
- 표준 프로토콜로 안정성 보장
- 추가 개발 불필요

### 커스텀 메시지 사용 (화재 진압 전용)

다음 기능만 커스텀 메시지로 구현:

| 메시지 ID | 메시지 이름 | 방향 | 용도 |
|-----------|------------|------|------|
| 12900 | FIRE_MISSION_START | QGC → VIM4 | 화재 진압 미션 시작 |
| 12901 | FIRE_MISSION_STATUS | VIM4 → QGC | 미션 진행 상태 |
| 12902 | FIRE_LAUNCH_CONTROL | QGC ↔ VIM4 | 발사 제어 |
| 12903 | FIRE_SUPPRESSION_RESULT | VIM4 → QGC | 발사 결과 |

---

## 커스텀 메시지 정의

### 1. FIRE_MISSION_START (ID: 12900)

**방향**: QGC → VIM4  
**용도**: 화재 진압 미션 시작 명령 (GO 버튼)

```cpp
struct FireMissionStart {
    uint8_t target_system;      // 시스템 ID
    uint8_t target_component;  // 컴포넌트 ID
    int32_t target_lat;        // 목표 위도 * 1e7 (RTK GPS 지원)
    int32_t target_lon;        // 목표 경도 * 1e7 (RTK GPS 지원)
    float target_alt;          // 목표 고도 MSL (m)
    uint8_t auto_fire;         // 0=수동, 1=자동
    uint8_t max_projectiles;   // 최대 발사 횟수
    uint8_t reserved[2];       // 예약 필드
};
```

**좌표 표현 방식**:
- `int32_t` 형식: degrees * 1e7 (MAVLink 표준)
- 최소 단위: 0.0000001도 ≈ 1.1cm (적도 기준)
- RTK GPS 정밀도: 센티미터 단위 → int32_t로 충분히 표현 가능
- 예시: 37.5665000° = 375665000 (int32_t)

**사용 시나리오**:
- QGC 사용자가 지도에서 화재 위치를 클릭
- RTK GPS로 정밀한 좌표 획득
- GO 버튼 클릭 시 이 메시지 전송
- VIM4가 미션을 시작하고 자동으로 이륙 및 이동 시작

---

### 2. FIRE_MISSION_STATUS (ID: 12901)

**방향**: VIM4 → QGC  
**용도**: 화재 진압 미션 진행 상태 (주기적 전송, 1-2Hz)

```cpp
struct FireMissionStatus {
    uint8_t phase;               // 현재 미션 단계 (0-6)
    uint8_t progress;            // 진행률 0-100%
    uint8_t remaining_projectiles; // 남은 발사 횟수
    float distance_to_target;    // 목표까지 거리 (m)
    int16_t thermal_max_temp;    // 최대 온도 (°C * 10)
    char status_text[50];        // 상태 메시지 (UTF-8)
};
```

**미션 단계 (FIRE_MISSION_PHASE)**:
- `0`: FIRE_PHASE_IDLE - 미션 시작 대기
- `1`: FIRE_PHASE_NAVIGATING - 목표로 이동 중
- `2`: FIRE_PHASE_SCANNING - 열화상 스캔 중
- `3`: FIRE_PHASE_READY_TO_FIRE - 발사 준비 완료
- `4`: FIRE_PHASE_SUPPRESSING - 화재 진압 중
- `5`: FIRE_PHASE_VERIFYING - 효과 검증 중
- `6`: FIRE_PHASE_COMPLETE - 미션 완료

**전송 주기**:
- Phase 1-2: 1Hz (이동 및 스캔 중)
- Phase 3: 2Hz (발사 준비 대기 중)
- Phase 4-5: 1Hz (진압 및 검증 중)

**목적지 도착 알림**:
- `phase`를 `FIRE_PHASE_NAVIGATING` (1)에서 `FIRE_PHASE_SCANNING` (2)로 변경
- `distance_to_target`을 0 또는 임계값 이하(예: < 1.0m)로 설정
- `status_text`에 "목적지 도착" 또는 "Arrived at target" 메시지 포함
- 즉시 전송 (이벤트 기반)

**목적지 도착 알림 구현**:

```cpp
// VIM4 측: 목적지 도착 감지 및 알림 전송
void send_arrival_notification() {
    FireMissionStatus status;
    
    // Phase 변경: NAVIGATING → SCANNING
    status.phase = static_cast<uint8_t>(FireMissionPhase::FIRE_PHASE_SCANNING);
    
    // 거리 정보
    status.distance_to_target = 0.0f;  // 또는 실제 거리 (< 1.0m)
    
    // 진행률 업데이트
    status.progress = 50;  // 예: 이동 완료 = 50%
    
    // 상태 메시지
    snprintf(status.status_text, sizeof(status.status_text), 
             "목적지 도착");
    
    // 기타 필드
    status.remaining_projectiles = remaining_projectiles;
    status.thermal_max_temp = 0;  // 아직 스캔 전
    
    // 즉시 전송 (이벤트 기반)
    msg_handler.sendFireMissionStatus(status);
}
```

**도착 판정 기준**:
1. **거리 기반**: `distance_to_target < 1.0m` (권장 임계값: 1.0m ~ 2.0m)
2. **Phase 변경 기반**: NAVIGATING (1) → SCANNING (2)
3. **상태 메시지 기반**: "목적지 도착" 포함

**권장**: 거리 기반 + Phase 변경 기반 조합 사용

---

### 3. FIRE_LAUNCH_CONTROL (ID: 12902)

**방향**: QGC ↔ VIM4  
**용도**: 발사 제어 (확인, 중단, 상태 요청)

```cpp
struct FireLaunchControl {
    uint8_t target_system;      // 시스템 ID
    uint8_t target_component;   // 컴포넌트 ID
    uint8_t command;            // 0=확인, 1=중단, 2=상태 요청
    uint8_t reserved[5];        // 예약 필드
};
```

**명령 타입**:
- `0`: FIRE_LAUNCH_CONFIRM - 발사 확인 (수동 모드에서 사용)
- `1`: FIRE_LAUNCH_ABORT - 발사 중단
- `2`: FIRE_LAUNCH_REQUEST_STATUS - 상태 요청

**사용 시나리오**:
- 수동 모드: Phase 3에서 사용자가 발사 확인 버튼 클릭
- 자동 모드: VIM4가 자동으로 발사 (이 메시지 불필요)
- 비상 상황: 언제든지 중단 명령 전송 가능

---

### 4. FIRE_SUPPRESSION_RESULT (ID: 12903)

**방향**: VIM4 → QGC  
**용도**: 발사 후 결과 전송

```cpp
struct FireSuppressionResult {
    uint8_t shot_number;        // 발사 번호
    uint8_t success;            // 0=실패, 1=성공
    uint8_t reserved[6];        // 예약 필드
};
```

**전송 시점**:
- 발사 직후 (Phase 4)

**성공 판정 기준**:
- 발사 메커니즘 동작 여부
- 발사 명령 실행 완료 여부
- (현재 단계에서는 온도 감소 확인 제외)

---

## 최종 워크플로우

### 전체 미션 흐름도

```
┌─────────────────────────────────────────────────────────────────┐
│                    [QGC 사용자]                                 │
│  지도에서 화재 위치 클릭 → GO 버튼 클릭                         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│  [QGC] → FIRE_MISSION_START → [VIM4]                           │
│  • target_lat, target_lon, target_alt                          │
│  • auto_fire (0=수동, 1=자동)                                  │
│  • max_projectiles                                              │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│              [VIM4 자동 실행 - Phase 1: 네비게이션]            │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 1. MAV_CMD_COMPONENT_ARM_DISARM (ARM)                   │  │
│  │    → 시동 켜기                                           │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 2. MAV_CMD_NAV_TAKEOFF                                  │  │
│  │    → 이륙                                                │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 3. GOTO_LOCATION (목표 위치로 이동)                     │  │
│  │    → target_lat, target_lon, target_alt로 이동         │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 4. FIRE_MISSION_STATUS 전송 (1Hz)                       │  │
│  │    phase = 1 (NAVIGATING)                                │  │
│  │    progress = 이동 진행률                                │  │
│  │    distance_to_target = 목표까지 거리                    │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│              [VIM4 자동 실행 - Phase 2: 스캔]                  │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 1. 열화상 카메라 스캔 시작                               │  │
│  │    → 화재 위치 정밀 확인                                 │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 2. FIRE_MISSION_STATUS 전송 (1Hz)                       │  │
│  │    phase = 2 (SCANNING)                                  │  │
│  │    thermal_max_temp = 최대 온도                         │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│          [VIM4 자동 실행 - Phase 3: 발사 준비]                 │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 1. FIRE_MISSION_STATUS 전송 (2Hz)                       │  │
│  │    phase = 3 (READY_TO_FIRE)                             │  │
│  │    status_text = "발사 준비 완료"                        │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
                    ┌───────┴───────┐
                    │               │
            자동 모드?          수동 모드?
            (auto_fire=1)      (auto_fire=0)
                    │               │
                    ↓               ↓
        ┌───────────────────┐  ┌──────────────────────────────┐
        │ 즉시 발사 실행     │  │ [QGC] ← FIRE_MISSION_STATUS │
        │ (Phase 3 → 4)     │  │ (발사 대기)                  │
        └───────────────────┘  └───────────┬──────────────────┘
                                            │
                                            ↓
                            ┌──────────────────────────────┐
                            │ [사용자가 발사 확인 버튼 클릭]│
                            └───────────┬──────────────────┘
                                        │
                                        ↓
                            ┌──────────────────────────────┐
                            │ [QGC] → FIRE_LAUNCH_CONTROL │
                            │ command = 0 (CONFIRM)       │
                            └───────────┬──────────────────┘
                                        │
                                        ↓
┌─────────────────────────────────────────────────────────────────┐
│              [VIM4 실행 - Phase 4: 진압]                        │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 1. 발사 실행                                             │  │
│  │    → 소화탄 발사                                         │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 2. FIRE_MISSION_STATUS 전송 (1Hz)                       │  │
│  │    phase = 4 (SUPPRESSING)                               │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 3. FIRE_SUPPRESSION_RESULT 전송                          │  │
│  │    shot_number = 발사 번호                                │  │
│  │    success = 성공 여부 (발사 메커니즘 동작 여부)           │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│              [VIM4 실행 - Phase 5: 검증]                       │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 1. 발사 결과 확인                                         │  │
│  │    → 발사 메커니즘 동작 여부 확인                         │  │
│  │    (현재 단계에서는 온도 감소 확인 제외)                  │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 2. FIRE_MISSION_STATUS 전송 (1Hz)                       │  │
│  │    phase = 5 (VERIFYING)                                 │  │
│  │    status_text = "발사 완료"                             │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│              [VIM4 실행 - Phase 6: 완료]                       │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 1. FIRE_MISSION_STATUS 전송                              │  │
│  │    phase = 6 (COMPLETE)                                  │  │
│  │    progress = 100%                                       │  │
│  │    status_text = "미션 완료"                             │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 2. MAV_CMD_NAV_RETURN_TO_LAUNCH                          │  │
│  │    → 출발지로 복귀                                       │  │
│  └─────────────────────────────────────────────────────────┘  │
│                            ↓                                    │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │ 3. MAV_CMD_NAV_LAND                                      │  │
│  │    → 착륙                                                │  │
│  └─────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 단계별 상세 흐름

#### Phase 0 → Phase 1: 미션 시작 및 네비게이션

```
[QGC] FIRE_MISSION_START 전송
  ↓
[VIM4] 메시지 수신
  ↓
[VIM4] MAV_CMD_COMPONENT_ARM_DISARM (ARM)
  ↓
[VIM4] MAV_CMD_NAV_TAKEOFF
  ↓
[VIM4] GOTO_LOCATION (target_lat, target_lon, target_alt)
  ↓
[VIM4] FIRE_MISSION_STATUS 전송 (phase=1, progress=0-50%)
  ↓
[QGC] 상태 표시 업데이트
```

#### Phase 1 → Phase 2: 스캔

```
[VIM4] 목표 위치 도착
  ↓
[VIM4] 열화상 카메라 스캔 시작
  ↓
[VIM4] FIRE_MISSION_STATUS 전송 (phase=2, thermal_max_temp)
  ↓
[QGC] 열화상 데이터 표시
```

#### Phase 2 → Phase 3: 발사 준비

```
[VIM4] 스캔 완료
  ↓
[VIM4] FIRE_MISSION_STATUS 전송 (phase=3, 2Hz)
  ↓
[QGC] 발사 준비 상태 표시
  ↓
┌─────────────────┬─────────────────┐
│ 자동 모드       │ 수동 모드       │
│ (auto_fire=1)   │ (auto_fire=0)   │
│                 │                 │
│ 즉시 Phase 4로  │ 사용자 확인 대기 │
│ 진행            │                 │
└─────────────────┴─────────────────┘
```

#### Phase 3 → Phase 4: 발사 (수동 모드)

```
[QGC] 사용자가 발사 확인 버튼 클릭
  ↓
[QGC] FIRE_LAUNCH_CONTROL 전송 (command=0)
  ↓
[VIM4] 메시지 수신 및 발사 실행
  ↓
[VIM4] FIRE_MISSION_STATUS 전송 (phase=4)
  ↓
[VIM4] FIRE_SUPPRESSION_RESULT 전송
```

#### Phase 4 → Phase 5: 검증

```
[VIM4] 발사 완료
  ↓
[VIM4] FIRE_SUPPRESSION_RESULT 전송 (발사 결과)
  ↓
[VIM4] FIRE_MISSION_STATUS 전송 (phase=5)
  ↓
[QGC] 발사 결과 표시
  (현재 단계에서는 온도 감소 확인 제외)
```

#### Phase 5 → Phase 6: 완료 및 복귀

```
[VIM4] 검증 완료
  ↓
[VIM4] FIRE_MISSION_STATUS 전송 (phase=6, progress=100%)
  ↓
[VIM4] MAV_CMD_NAV_RETURN_TO_LAUNCH
  ↓
[VIM4] MAV_CMD_NAV_LAND
  ↓
[QGC] 미션 완료 표시
```

---

## 메시지 전송 타이밍

### 주기적 전송 (VIM4 → QGC)

| 메시지 | Phase | 주기 | 설명 |
|--------|-------|------|------|
| FIRE_MISSION_STATUS | 1 | 1Hz | 이동 중 (distance_to_target 업데이트) |
| FIRE_MISSION_STATUS | 1→2 | 즉시 | **목적지 도착 알림** (이벤트 기반) |
| FIRE_MISSION_STATUS | 2 | 1Hz | 스캔 중 |
| FIRE_MISSION_STATUS | 3 | 2Hz | 발사 준비 대기 중 |
| FIRE_MISSION_STATUS | 4-5 | 1Hz | 진압 및 검증 중 |
| FIRE_MISSION_STATUS | 6 | 1회 | 미션 완료 |

### 이벤트 기반 전송

| 메시지 | 전송 시점 | 설명 |
|--------|----------|------|
| FIRE_MISSION_START | GO 버튼 클릭 | 미션 시작 |
| FIRE_LAUNCH_CONTROL | 발사 확인/중단 | 사용자 명령 |
| FIRE_SUPPRESSION_RESULT | 발사 직후 | 발사 결과 |

---

## 예외 처리

### 비상 상황

```
[언제든지]
  ↓
[QGC] FIRE_LAUNCH_CONTROL 전송 (command=1, ABORT)
  ↓
[VIM4] 발사 중단
  ↓
[VIM4] MAV_CMD_NAV_RETURN_TO_LAUNCH (Emergency RTL)
```

### 통신 장애

- FIRE_MISSION_STATUS가 일정 시간 수신되지 않으면 QGC에서 경고 표시
- VIM4는 자동으로 RTL 실행 (안전 장치)

---

## 구현 참고사항

### VIM4 미션 컨트롤러

```python
async def execute_mission(self):
    """전체 미션 실행"""
    try:
        # Phase 0 → 1: 네비게이션
        self.update_phase(1, "목표로 이동 중")
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await self.drone.action.goto_location(
            self.target_location['lat'],
            self.target_location['lon'],
            self.target_location['alt'],
            0
        )
        
        # 목적지 도착 감지 및 알림
        while True:
            distance = self.get_distance_to_target()
            self.update_status(phase=1, distance=distance)
            
            if distance < 1.0:  # 목적지 도착 임계값
                # 목적지 도착 알림 전송 (즉시)
                self.update_phase(2, "목적지 도착", distance=0.0)
                break
            await asyncio.sleep(0.1)  # 100ms마다 확인
        
        # Phase 1 → 2: 스캔
        self.update_phase(2, "열화상 스캔 중")
        thermal_data = await self.scan_thermal()
        
        # Phase 2 → 3: 발사 준비
        self.update_phase(3, "발사 준비 완료")
        
        if self.auto_fire:
            await self.execute_fire()
        else:
            await self.wait_for_fire_confirm()
        
        # Phase 3 → 4: 진압
        self.update_phase(4, "화재 진압 중")
        result = await self.launch_projectile()
        
        # Phase 4 → 5: 검증
        self.update_phase(5, "발사 완료")
        # (현재 단계에서는 온도 감소 확인 제외)
        
        # Phase 5 → 6: 완료
        self.update_phase(6, "미션 완료")
        await self.drone.action.return_to_launch()
        
    except Exception as e:
        await self.drone.action.return_to_launch()
```

### QGC 플러그인

#### 미션 시작 및 발사 제어

```cpp
void CustomPlugin::sendMissionStart(double lat, double lon, double alt, 
                                   bool autoFire, int maxProjectiles)
{
    mavlink_fire_mission_start_t cmd = {};
    cmd.target_lat = lat * 1e7;
    cmd.target_lon = lon * 1e7;
    cmd.target_alt = alt;
    cmd.auto_fire = autoFire ? 1 : 0;
    cmd.max_projectiles = maxProjectiles;
    
    mavlink_msg_fire_mission_start_encode(...);
    _vehicle->sendMessageOnLinkThreadSafe(...);
}

void CustomPlugin::confirmFire()
{
    mavlink_fire_launch_control_t cmd = {};
    cmd.command = 0;  // confirm
    mavlink_msg_fire_launch_control_encode(...);
    _vehicle->sendMessageOnLinkThreadSafe(...);
}
```

#### 목적지 도착 알림 수신

```cpp
class QGCPlugin {
private:
    CustomMessage msg_handler_;
    uint8_t previous_phase_ = 0;
    
public:
    void setup() {
        msg_handler_.setFireMissionStatusCallback(
            [this](const FireMissionStatus& status) {
                handle_mission_status(status);
            }
        );
    }
    
private:
    void handle_mission_status(const FireMissionStatus& status) {
        // Phase 변경 감지: NAVIGATING → SCANNING = 목적지 도착
        if (status.phase == 2 && previous_phase_ == 1) {
            if (status.distance_to_target < 1.0f) {
                showArrivalNotification(QString::fromUtf8(status.status_text));
            }
        }
        
        // UI 업데이트
        updateStatusDisplay(status);
        
        previous_phase_ = status.phase;
    }
    
    void showArrivalNotification(const QString& message) {
        // QGC 알림 표시
        qgcApp()->showMessage(message, "목적지 도착");
    }
};
```

---

## 라이브러리 사용 가이드

### 빌드

#### 요구사항

- C++17 이상
- CMake 3.10 이상
- Linux (UDP 소켓 지원)
- pthread 라이브러리

#### 빌드 방법

```bash
cd custom_message
mkdir build && cd build
cmake ..
make
```

#### 설치 (선택사항)

```bash
sudo make install
```

### 기본 사용 예제

```cpp
#include "custom_message/custom_message.h"
#include <iostream>

using namespace custom_message;

int main() {
    // 메시지 송수신기 생성
    CustomMessage msg_handler(
        14550,              // 수신 포트
        14550,              // 송신 포트
        "0.0.0.0",          // 바인드 주소
        "127.0.0.1",        // 대상 주소
        1,                  // 시스템 ID
        1                   // 컴포넌트 ID
    );

    // 수신 콜백 등록
    msg_handler.setFireMissionStartCallback([](const FireMissionStart& start) {
        std::cout << "미션 시작 수신: "
                  << "위치: (" << (start.target_lat / 1e7) << ", " 
                  << (start.target_lon / 1e7) << ")"
                  << ", 고도: " << start.target_alt << "m"
                  << ", 자동발사: " << (start.auto_fire ? "예" : "아니오") << std::endl;
    });

    msg_handler.setFireMissionStatusCallback([](const FireMissionStatus& status) {
        std::cout << "미션 상태 수신: "
                  << "단계: " << static_cast<int>(status.phase)
                  << ", 진행률: " << static_cast<int>(status.progress) << "%"
                  << ", 상태: " << status.status_text << std::endl;
    });

    // 메시지 송수신 시작
    if (!msg_handler.start()) {
        std::cerr << "송수신 시작 실패" << std::endl;
        return 1;
    }

    // 메시지 전송 예제
    FireMissionStatus status;
    status.phase = 1;  // NAVIGATING
    status.progress = 50;
    status.remaining_projectiles = 6;
    status.distance_to_target = 100.0f;
    status.thermal_max_temp = 500;  // 50.0°C
    snprintf(status.status_text, sizeof(status.status_text), "목표로 이동 중");

    msg_handler.sendFireMissionStatus(status);

    // 메인 루프
    while (msg_handler.isRunning()) {
        // 통계 정보 확인
        auto stats = msg_handler.getStatistics();
        // ... 작업 수행 ...
    }

    // 종료
    msg_handler.stop();
    return 0;
}
```

### CMake 프로젝트에 통합

```cmake
# CMakeLists.txt
add_subdirectory(custom_message)

target_link_libraries(your_target PRIVATE custom_message)
```

### API 문서

#### 클래스: `CustomMessage`

##### 생성자

```cpp
CustomMessage(
    uint16_t receive_port = 14550,
    uint16_t send_port = 14550,
    const std::string& bind_address = "0.0.0.0",
    const std::string& target_address = "127.0.0.1",
    uint8_t system_id = 1,
    uint8_t component_id = 1
)
```

**매개변수**:
- `receive_port`: UDP 수신 포트
- `send_port`: UDP 송신 포트
- `bind_address`: 바인드 주소 (기본값: "0.0.0.0" - 모든 인터페이스)
- `target_address`: 송신 대상 주소
- `system_id`: MAVLink 시스템 ID
- `component_id`: MAVLink 컴포넌트 ID

##### 메서드

**수신 관련**:
- `bool start()`: 메시지 송수신 시작
- `void stop()`: 메시지 송수신 중지
- `bool isRunning() const`: 송수신 중인지 확인
- `void setFireMissionStartCallback(FireMissionStartCallback callback)`: 미션 시작 콜백 등록
- `void setFireMissionStatusCallback(FireMissionStatusCallback callback)`: 미션 상태 콜백 등록
- `void setFireLaunchControlCallback(FireLaunchControlCallback callback)`: 발사 제어 콜백 등록
- `void setFireSuppressionResultCallback(FireSuppressionResultCallback callback)`: 진압 결과 콜백 등록

**송신 관련**:
- `bool sendFireMissionStart(const FireMissionStart& start)`: 미션 시작 전송
- `bool sendFireMissionStatus(const FireMissionStatus& status)`: 미션 상태 전송
- `bool sendFireLaunchControl(const FireLaunchControl& control)`: 발사 제어 전송
- `bool sendFireSuppressionResult(const FireSuppressionResult& result)`: 진압 결과 전송
- `void setTargetAddress(const std::string& address, uint16_t port)`: 송신 대상 주소 변경

**통계 정보**:
- `Statistics getStatistics() const`: 통계 정보 가져오기
- `void resetStatistics()`: 통계 정보 리셋

##### 통계 정보 구조체

```cpp
struct Statistics {
    uint64_t mission_start_received = 0;
    uint64_t mission_status_received = 0;
    uint64_t launch_control_received = 0;
    uint64_t suppression_result_received = 0;
    uint64_t mission_start_sent = 0;
    uint64_t mission_status_sent = 0;
    uint64_t launch_control_sent = 0;
    uint64_t suppression_result_sent = 0;
    uint64_t unknown_message_count = 0;
    uint64_t parse_error_count = 0;
    uint64_t send_error_count = 0;
};
```

### 예제 프로그램

`custom_message/examples/example_usage.cpp` 파일에 완전한 사용 예제가 포함되어 있습니다.

**컴파일 및 실행**:

```bash
cd custom_message/examples
g++ -std=c++17 -I../include example_usage.cpp -L../build -lcustom_message -pthread -o example_usage
./example_usage
```

---

## 목적지 도착 알림 구현 상세

### 개요

목적지 도착 알림은 `FIRE_MISSION_STATUS` 메시지를 사용하여 전달합니다. 별도의 메시지 타입 없이 기존 상태 메시지로 충분히 구현 가능합니다.

### 구현 방법

#### 1. 목적지 도착 감지

VIM4에서 목적지 도착을 감지하는 방법:

```cpp
// 목적지까지 거리 계산
float distance = calculate_distance_to_target(
    current_lat, current_lon,
    target_lat, target_lon
);

// 목적지 도착 임계값 (예: 1.0m)
const float ARRIVAL_THRESHOLD = 1.0f;

if (distance < ARRIVAL_THRESHOLD) {
    // 목적지 도착
    send_arrival_notification();
}
```

#### 2. 도착 알림 전송

`FIRE_MISSION_STATUS` 메시지를 사용하여 즉시 전송:

```cpp
void send_arrival_notification() {
    FireMissionStatus status;
    
    // Phase 변경: NAVIGATING → SCANNING
    status.phase = static_cast<uint8_t>(FireMissionPhase::FIRE_PHASE_SCANNING);
    
    // 거리 정보
    status.distance_to_target = 0.0f;  // 또는 실제 거리 (< 1.0m)
    
    // 진행률 업데이트
    status.progress = 50;  // 예: 이동 완료 = 50%
    
    // 상태 메시지
    snprintf(status.status_text, sizeof(status.status_text), 
             "목적지 도착");
    
    // 기타 필드
    status.remaining_projectiles = remaining_projectiles;
    status.thermal_max_temp = 0;  // 아직 스캔 전
    
    // 즉시 전송 (이벤트 기반)
    msg_handler.sendFireMissionStatus(status);
}
```

#### 3. QGC에서 도착 알림 수신

QGC에서 목적지 도착을 감지하는 방법:

```cpp
msg_handler.setFireMissionStatusCallback([](const FireMissionStatus& status) {
    // Phase 변경 감지
    if (status.phase == static_cast<uint8_t>(FireMissionPhase::FIRE_PHASE_SCANNING) &&
        previous_phase == static_cast<uint8_t>(FireMissionPhase::FIRE_PHASE_NAVIGATING)) {
        
        // 목적지 도착 알림
        if (status.distance_to_target < 1.0f) {
            showArrivalNotification(status.status_text);
        }
    }
    
    previous_phase = status.phase;
});
```

### 도착 판정 기준

#### 방법 1: 거리 기반

```cpp
const float ARRIVAL_THRESHOLD = 1.0f;  // 1미터 이내

if (distance_to_target < ARRIVAL_THRESHOLD) {
    // 목적지 도착
}
```

#### 방법 2: Phase 변경 기반

```cpp
// Phase가 NAVIGATING에서 SCANNING으로 변경되면 도착으로 판정
if (current_phase == FIRE_PHASE_SCANNING &&
    previous_phase == FIRE_PHASE_NAVIGATING) {
    // 목적지 도착
}
```

#### 방법 3: 상태 메시지 기반

```cpp
if (strstr(status.status_text, "목적지 도착") != nullptr ||
    strstr(status.status_text, "Arrived") != nullptr) {
    // 목적지 도착
}
```

**권장**: 방법 1 + 방법 2 조합 (거리와 Phase 변경 모두 확인)

### 메시지 전송 타이밍

#### 주기적 전송 (Phase 1: 이동 중)

```cpp
// 1Hz로 주기적 전송
while (phase == FIRE_PHASE_NAVIGATING) {
    FireMissionStatus status;
    status.phase = 1;
    status.distance_to_target = calculate_distance();
    status.progress = calculate_progress();
    // ...
    
    msg_handler.sendFireMissionStatus(status);
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
```

#### 이벤트 기반 전송 (목적지 도착)

```cpp
// 목적지 도착 시 즉시 전송
if (distance_to_target < ARRIVAL_THRESHOLD) {
    FireMissionStatus status;
    status.phase = 2;  // SCANNING으로 변경
    status.distance_to_target = 0.0f;
    status.status_text = "목적지 도착";
    // ...
    
    msg_handler.sendFireMissionStatus(status);  // 즉시 전송
}
```

### 주의사항

#### 1. 거리 계산 정확도

- GPS 좌표 기반 거리 계산은 수평 거리만 고려
- 고도 차이는 별도로 확인 필요
- RTK GPS 사용 시 정밀도 향상

#### 2. 임계값 설정

- 너무 작으면: 도착 감지 실패 가능
- 너무 크면: 조기 도착 감지
- 권장: 1.0m ~ 2.0m

#### 3. 네트워크 지연

- UDP 통신 특성상 메시지 손실 가능
- QGC는 Phase 변경을 주기적으로 확인하여 보완

#### 4. 중복 알림 방지

```cpp
bool arrival_notified = false;

if (distance < ARRIVAL_THRESHOLD && !arrival_notified) {
    send_arrival_notification();
    arrival_notified = true;
}
```

### 요약

**목적지 도착 알림은 `FIRE_MISSION_STATUS` 메시지로 충분합니다:**

1. ✅ **Phase 변경**: NAVIGATING (1) → SCANNING (2)
2. ✅ **거리 정보**: `distance_to_target` = 0 또는 < 1.0m
3. ✅ **상태 메시지**: "목적지 도착" 포함
4. ✅ **즉시 전송**: 이벤트 기반 (주기적 전송과 별도)

별도의 메시지 타입 없이 기존 상태 메시지로 구현 가능합니다.

---

## 문제 해결

### 메시지가 수신되지 않음

1. **포트 확인**: 올바른 포트로 전송하는지 확인
2. **방화벽 확인**: UDP 포트가 차단되지 않았는지 확인
3. **네트워크 연결 확인**: 송수신 대상이 같은 네트워크에 있는지 확인
4. **MAVLink Router 확인**: VIM4에서 MAVLink Router가 실행 중인지 확인

### 메시지 전송 실패

1. **대상 주소 확인**: `setTargetAddress()`로 올바른 주소 설정 확인
2. **소켓 상태 확인**: `isRunning()`으로 송수신 상태 확인
3. **통계 정보 확인**: `getStatistics()`로 송신 오류 수 확인

### 파싱 오류 발생

1. **메시지 ID 확인**: 올바른 메시지 ID를 사용하는지 확인
2. **메시지 포맷 확인**: MAVLink 메시지 포맷이 올바른지 확인
3. **체크섬 확인**: 체크섬 계산이 올바른지 확인

---

## 참고 자료

- [MAVLink 공식 문서](https://mavlink.io/)
- [커스텀 메시지 라이브러리](../custom_message/README.md)
- [RTK GPS 좌표 형식 가이드](./020_RTK_GPS_COORDINATE_FORMAT.md)
- [QGC 개발 가이드](./016_QGC_DEVELOPMENT_GUIDE.md)

---

**작성자**: Humiro Fire Suppression Team  
**버전**: v2.1 (목적지 도착 알림 구현 가이드 통합)  
**작성일**: 2026-01-03  
**최종 수정일**: 2026-01-03
