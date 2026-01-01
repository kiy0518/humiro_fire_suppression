# PX4 Navigation State (nav_state) 참조 가이드

작성일: 2026-01-01  
**소스**: PX4 uXRCE-DDS `/fmu/out/vehicle_status_v1` 토픽의 `nav_state` 필드

---

## 개요

PX4의 `nav_state`는 드론의 현재 비행 모드와 상태를 나타내는 uint8 값입니다.  
총 **25개**의 상태 값이 정의되어 있습니다 (0-24).

---

## PX4 nav_state 전체 목록

| nav_state | 모드 이름 | QGC 표시 | 설명 | 카테고리 |
|-----------|----------|---------|------|---------|
| 0 | MANUAL | Manual | 수동 모드 (자동 안정화 없음) | 수동 모드 |
| 1 | ALTCTL | Altitude | 고도 제어 모드 | 수동 모드 |
| 2 | POSCTL | Position | 위치 제어 모드 | 수동 모드 |
| 3 | AUTO_MISSION | Mission | 미션 모드 (waypoint 자동 비행) | 자동 모드 |
| 4 | AUTO_LOITER | Hold | 자동 대기 모드 (호버링) | 자동 모드 |
| 5 | AUTO_RTL | Return to Launch | 자동 복귀 모드 | 자동 모드 |
| 6 | POSITION_SLOW | Position Slow | 느린 위치 제어 모드 | 수동 모드 |
| 7 | FREE5 | - | 예약됨 (FREE5) | 예약 |
| 8 | ALTITUDE_CRUISE | - | 고도 순항 모드 | 수동 모드 |
| 9 | FREE3 | - | 예약됨 (FREE3) | 예약 |
| 10 | ACRO | Acro | 아크로 모드 (자동 안정화 없음) | 수동 모드 |
| 11 | FREE2 | - | 예약됨 (FREE2) | 예약 |
| 12 | DESCEND | - | 강하 모드 | 자동 모드 |
| 13 | TERMINATION | - | 종료 모드 (비상 정지) | 비상 모드 |
| 14 | OFFBOARD | Offboard | 오프보드 모드 (외부 제어) | 자동 모드 |
| 15 | STABILIZED | Stabilized | 안정화 모드 | 수동 모드 |
| 16 | FREE1 | - | 예약됨 (FREE1) | 예약 |
| 17 | AUTO_TAKEOFF | Takeoff | 자동 이륙 모드 | 자동 모드 |
| 18 | AUTO_LAND | Land | 자동 착륙 모드 | 자동 모드 |
| 19 | AUTO_FOLLOW_TARGET | Follow Target | 타겟 추적 모드 | 자동 모드 |
| 20 | AUTO_PRECLAND | Precision Landing | 정밀 착륙 모드 | 자동 모드 |
| 21 | ORBIT | Orbit | 궤도 비행 모드 | 자동 모드 |
| 22 | AUTO_VTOL_TAKEOFF | VTOL Takeoff | VTOL 이륙 모드 | 자동 모드 |
| 23 | EXTERNAL1 | - | 외부 모드 1 (커스텀) | 외부 모드 |
| 24 | EXTERNAL2 | - | 외부 모드 2 (커스텀) | 외부 모드 |

---

## 카테고리별 분류

### 1. 수동 모드 (Manual Modes)
- **0**: MANUAL - 완전 수동
- **1**: ALTCTL - 고도 제어
- **2**: POSCTL - 위치 제어
- **6**: POSITION_SLOW - 느린 위치 제어
- **8**: ALTITUDE_CRUISE - 고도 순항
- **10**: ACRO - 아크로
- **15**: STABILIZED - 안정화

### 2. 자동 모드 (Auto Modes)
- **3**: AUTO_MISSION - 미션
- **4**: AUTO_LOITER - 대기
- **5**: AUTO_RTL - 복귀
- **12**: DESCEND - 강하
- **14**: OFFBOARD - 오프보드
- **17**: AUTO_TAKEOFF - 이륙
- **18**: AUTO_LAND - 착륙
- **19**: AUTO_FOLLOW_TARGET - 타겟 추적
- **20**: AUTO_PRECLAND - 정밀 착륙
- **21**: ORBIT - 궤도
- **22**: AUTO_VTOL_TAKEOFF - VTOL 이륙

### 3. 비상 모드 (Emergency Modes)
- **13**: TERMINATION - 종료 (비상 정지)

### 4. 예약/외부 모드
- **7**: FREE5 - 예약
- **9**: FREE3 - 예약
- **11**: FREE2 - 예약
- **16**: FREE1 - 예약
- **23**: EXTERNAL1 - 외부 모드 1
- **24**: EXTERNAL2 - 외부 모드 2

---

## 주요 상태 설명

### 수동 모드

#### MANUAL (0)
- **설명**: 조종자가 모든 축을 직접 제어
- **자동 안정화**: 없음
- **사용 시기**: 고급 조종, 공중 곡예

#### ALTCTL (1) - Altitude
- **설명**: 고도는 자동 유지, 수평 이동은 조종자 제어
- **자동 안정화**: 고도만
- **사용 시기**: 고도 유지가 필요한 수동 비행

#### POSCTL (2) - Position
- **설명**: 위치와 고도 자동 유지, 조종자 입력으로 이동
- **자동 안정화**: 위치, 고도
- **사용 시기**: GPS 환경에서 안정적인 수동 비행

#### STABILIZED (15)
- **설명**: 자세 안정화, 조종자가 방향과 속도 제어
- **자동 안정화**: 자세
- **사용 시기**: 기본 수동 비행

### 자동 모드

#### AUTO_MISSION (3) - Mission
- **설명**: QGC에서 업로드한 waypoint를 따라 자동 비행
- **사용 시기**: 자동 경로 비행

#### AUTO_LOITER (4) - Hold
- **설명**: 현재 위치에서 호버링 (대기)
- **사용 시기**: 임시 대기, 비행 중단

#### AUTO_RTL (5) - Return to Launch
- **설명**: 이륙 지점으로 자동 복귀
- **사용 시기**: 배터리 부족, 비상 복귀

#### AUTO_TAKEOFF (17) - Takeoff
- **설명**: 지정된 고도까지 자동 이륙
- **사용 시기**: 자동 이륙

#### AUTO_LAND (18) - Land
- **설명**: 현재 위치에서 자동 착륙
- **사용 시기**: 자동 착륙

#### OFFBOARD (14) - Offboard
- **설명**: 외부 컴퓨터(VIM4)에서 ROS2로 제어
- **사용 시기**: 커스텀 자동 제어 시스템

---

## VehicleStatus 메시지 필드

PX4의 `vehicle_status` 메시지에는 `nav_state` 외에도 다른 상태 정보가 포함됩니다:

### 주요 필드
- **`nav_state`** (uint8): 현재 비행 모드 (0-24)
- **`arming_state`** (bool): 시동 상태 (true = ARMED, false = DISARMED)
- **`failsafe`** (bool): 페일세이프 활성화 여부
- **`vehicle_type`** (uint8): 기체 타입 (멀티콥터, 고정익 등)
- **`system_type`** (uint8): 시스템 타입

---

## 상태 전환 흐름

### 일반적인 비행 시퀀스
```
DISARMED (시동 해제)
  ↓
ARMING (시동)
  ↓
AUTO_TAKEOFF (이륙)
  ↓
AUTO_MISSION 또는 OFFBOARD (비행)
  ↓
AUTO_RTL (복귀)
  ↓
AUTO_LAND (착륙)
  ↓
DISARMED (시동 해제)
```

### OFFBOARD 모드 시퀀스
```
DISARMED
  ↓
ARMING
  ↓
OFFBOARD (VIM4 제어 시작)
  ↓
  ├─ NAVIGATING (이동)
  ├─ DESTINATION_REACHED (도착)
  ├─ FIRE_READY (격발 대기)
  ├─ FIRING_AUTO_TARGETING (격발 중)
  └─ MISSION_COMPLETE (임무 완료)
  ↓
AUTO_RTL (복귀)
  ↓
AUTO_LAND (착륙)
  ↓
DISARMED
```

---

## 코드에서의 사용

### nav_state 확인
```cpp
// vehicle_status 메시지에서 nav_state 읽기
uint8_t nav_state = msg->nav_state;

// nav_state를 모드 문자열로 변환
std::string mode = navStateToModeString(nav_state);

// 특정 모드 확인
if (nav_state == 14) {  // OFFBOARD
    // OFFBOARD 모드 처리
}
```

### 시동 상태 확인
```cpp
bool is_armed = msg->arming_state;
if (is_armed) {
    // 시동 ON
} else {
    // 시동 OFF (DISARMED)
}
```

---

## 참고 자료

- [PX4 공식 문서: Flight Modes](https://docs.px4.io/main/en/getting_started/flight_modes.html)
- [PX4 공식 문서: Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- PX4 소스 코드: `src/lib/modes/` 디렉토리

---

**작성자**: Claude Code Assistant  
**버전**: v1.0  
**작성일**: 2026-01-01

