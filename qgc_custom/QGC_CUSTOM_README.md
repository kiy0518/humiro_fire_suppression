# QGC 커스텀 화면 레이아웃 - Humiro Fire Suppression

**작성일**: 2026-01-27
**QGC 버전**: 4.0+
**메시지 버전**: custom_message v4.0 (60000-60011)

---

## 개요

화재 진압 드론 단일 미션 제어를 위한 QGC 커스텀 UI 레이아웃입니다.

**주요 기능**:
- 미션 상태 실시간 모니터링 (FIRE_MISSION_STATUS)
- 목표 지점 관리 및 미션 시작
- 격발 제어 (FIRE_LAUNCH)
- 진압 결과 표시 (FIRE_SUPPRESSION_RESULT)

---

## 메시지 ID 매핑 (v4.0)

### QGC → VIM4 (명령)

| 메시지 이름 | ID | 설명 |
|------------|------|------|
| FIRE_MISSION_START | 60000 | 미션 시작 |
| FIRE_AUTO_AIM | 60001 | 자동 조준 |
| FIRE_LAUNCH | 60002 | 발사 명령 |
| FIRE_RETURN | 60003 | 복귀(RTL) 명령 |

### VIM4 → QGC (상태)

| 메시지 이름 | ID | 설명 |
|------------|------|------|
| FIRE_MISSION_STATUS | 60010 | 미션 상태 |
| FIRE_SUPPRESSION_RESULT | 60011 | 진압 결과 |

---

## 파일 구조

```
qgc_custom/
├── qml/
│   ├── FormationStatusPanel.qml      # 미션 상태 모니터링 패널
│   ├── FirePointPanel.qml            # 목표 지점 및 진압 결과 패널
│   ├── FireControlPanel.qml          # 격발 제어 패널
│   └── HumiroCustomPlugin.qml        # 메인 플러그인
├── plugins/
│   └── (QGC 플러그인 설정 파일)
└── resources/
    └── (아이콘, 이미지 등)
```

---

## MAVLink 메시지 연동

### 수신 메시지

#### FIRE_MISSION_STATUS (ID: 60010)
미션 상태 업데이트 (VIM4 → QGC)

**QML 핸들러**:
```javascript
if (message.id === 60010) {
    formationStatus.updateMissionStatus(
        message.phase,
        message.progress,
        message.remaining_projectiles,
        message.distance_to_target,
        message.thermal_max_temp / 10.0,
        message.status_text
    )
}
```

#### FIRE_SUPPRESSION_RESULT (ID: 60011)
진압 결과 업데이트 (VIM4 → QGC)

**QML 핸들러**:
```javascript
if (message.id === 60011) {
    firePointPanel.updateSuppressionResult(
        message.shot_number,
        message.temp_before / 10.0,
        message.temp_after / 10.0,
        message.success
    )
}
```

---

### 송신 메시지

#### FIRE_MISSION_START (ID: 60000)
미션 시작 명령 (QGC → VIM4)

**메시지 구조**:
```cpp
struct FireMissionStart {
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
    int32_t target_lat;           // Target latitude * 1e7
    int32_t target_lon;           // Target longitude * 1e7
    float target_alt;             // Target altitude MSL (m)
    uint8_t auto_fire;            // 0=manual, 1=auto
    uint8_t max_projectiles;      // Max projectiles to use
};
```

**QML 전송**:
```javascript
function sendFireMissionStart(lat, lon, alt, autoFire, maxProjectiles) {
    var message = activeVehicle.createMAVLinkMessage(60000)
    message.target_system = 1
    message.target_component = 1
    message.target_lat = lat * 1e7
    message.target_lon = lon * 1e7
    message.target_alt = alt
    message.auto_fire = autoFire ? 1 : 0
    message.max_projectiles = maxProjectiles
    activeVehicle.sendMessage(message)
}
```

#### FIRE_AUTO_AIM (ID: 60001)
자동 조준 명령 (QGC → VIM4)

**QML 전송**:
```javascript
function sendFireAutoAim() {
    var message = activeVehicle.createMAVLinkMessage(60001)
    message.target_system = 1
    message.target_component = 1
    activeVehicle.sendMessage(message)
}
```

#### FIRE_LAUNCH (ID: 60002)
발사 명령 (QGC → VIM4)

**QML 전송**:
```javascript
function sendMAVLinkFireCommand(confirm) {
    var message = activeVehicle.createMAVLinkMessage(60002)
    message.target_system = 1
    message.target_component = 1
    activeVehicle.sendMessage(message)
}
```

#### FIRE_RETURN (ID: 60003)
복귀 명령 (QGC → VIM4)

**QML 전송**:
```javascript
function sendFireReturn() {
    var message = activeVehicle.createMAVLinkMessage(60003)
    message.target_system = 1
    message.target_component = 1
    activeVehicle.sendMessage(message)
}
```

---

## 문제 해결

### MAVLink 메시지가 제대로 파싱되지 않음

**문제**: GPS 좌표가 이상한 값으로 수신됨

**원인**: QGC의 `createMAVLinkMessage()`는 MAVLink XML dialect에 정의된 메시지만 정상 동작합니다.
60000번대 커스텀 메시지는 QGC에 정의되지 않아 필드 순서가 엉망이 될 수 있습니다.

**해결방안**:
1. QGC에 커스텀 MAVLink dialect XML 추가
2. 또는 Python 테스트 도구 사용 (`custom_message/test/custom_message_sender_gui_v2.py`)

### 메시지 ID 확인

- 60000-60003: QGC → VIM4 명령
- 60010-60011: VIM4 → QGC 상태
- `custom_message` 라이브러리와 일치하는지 확인

---

## 참고 자료

**VIM4 문서**:
- `custom_message/README.md`: 커스텀 메시지 라이브러리 문서
- `custom_message/include/custom_message/custom_message_type.h`: 메시지 타입 정의

**QGC 문서**:
- [QGC Custom Build](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/custom_build.html)
- [QGC Plugin Development](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/plugin.html)

---

**작성자**: Claude Code Assistant
**버전**: v4.0 (60000번대 MSG ID)
**작성일**: 2026-01-27
