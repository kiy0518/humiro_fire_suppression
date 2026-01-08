# QGC 커스텀 화면 레이아웃 - Humiro Fire Suppression

**작성일**: 2026-01-05
**QGC 버전**: 4.0+
**메시지 버전**: custom_message v3.1 (12900-12904)

---

## 개요

화재 진압 드론 단일 미션 제어를 위한 QGC 커스텀 UI 레이아웃입니다.

**주요 기능**:
- 미션 상태 실시간 모니터링 (FIRE_MISSION_STATUS)
- 목표 지점 관리 및 미션 시작
- 격발 제어 (FIRE_LAUNCH_CONTROL)
- 진압 결과 표시 (FIRE_SUPPRESSION_RESULT)

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

## 설치 방법

### 1. QGC 커스텀 플러그인 디렉토리 확인

QGC 설정에서 커스텀 플러그인 경로를 확인합니다:
- **Windows**: `C:\Users\[User]\AppData\Local\QGroundControl\Custom`
- **Linux**: `~/.local/share/QGroundControl/Custom`
- **macOS**: `~/Library/Application Support/QGroundControl/Custom`

### 2. 파일 복사

```bash
# QGC Custom 디렉토리로 이동
cd ~/.local/share/QGroundControl/Custom

# qml 파일 복사
mkdir -p qml
cp FormationStatusPanel.qml qml/
cp FirePointPanel.qml qml/
cp FireControlPanel.qml qml/
cp HumiroCustomPlugin.qml qml/
```

### 3. QGC 설정

1. QGC 실행
2. **Settings** → **General** → **Custom Plugins**
3. **Enable Custom Plugins** 체크
4. **Custom Plugin Path** 설정
5. QGC 재시작

---

## 화면 구성

### 레이아웃

```
┌────────────────────────────────────────────────────────────┐
│                    QGC Main Window                         │
├──────────────┬─────────────────────┬───────────────────────┤
│              │                     │                       │
│  Mission     │                     │   Target & Result     │
│  Status      │    QGC Map View     │   Panel              │
│  Panel       │                     │                       │
│              │  (Default Map)      │   ┌─────────────────┐ │
│  ┌─────────┐ │                     │   │ Target #1      │ │
│  │ Phase   │ │                     │   │ Target #2      │ │
│  │ Progress│ │                     │   └─────────────────┘ │
│  │ Distance│ │                     │                       │
│  │ Temp    │ │                     │   Fire Control Panel  │
│  └─────────┘ │                     │                       │
│              │                     │   ┌─────────────────┐ │
│              │                     │   │ [발사 확인]    │ │
│              │                     │   │ [발사 중단]    │ │
│              │                     │   │ [상태 요청]    │ │
│              │                     │   └─────────────────┘ │
└──────────────┴─────────────────────┴───────────────────────┘
```

---

## 사용 방법

### 1. 미션 상태 모니터링

**표시 정보**:
- 미션 단계 (IDLE, NAVIGATING, SCANNING, READY_TO_FIRE, SUPPRESSING, VERIFYING, COMPLETE)
- 진행률 (0-100%)
- 남은 소화탄 개수
- 목표까지 거리 (m)
- 열화상 최고 온도 (°C)
- 상태 메시지

**색상 코딩**:
- 🟢 녹색: READY_TO_FIRE, COMPLETE
- 🔵 파란색: NAVIGATING
- 🟡 노란색: SCANNING
- 🔴 빨간색: SUPPRESSING
- 🟣 보라색: VERIFYING

---

### 2. 목표 지점 관리

**기능**:
- **[+ 목표 지점 추가]**: 새 목표 지점 추가 (지도 클릭)
- **[Delete]**: 목표 지점 삭제
- **[미션 시작]**: 선택한 목표 지점으로 미션 시작 (FIRE_MISSION_START 전송)

**목표 지점 추가 방법**:
1. [+ 목표 지점 추가] 버튼 클릭
2. QGC 지도에서 목표 지점 클릭
3. 목표 지점이 목록에 추가됨

**미션 시작**:
1. 목표 지점 목록에서 원하는 목표 선택
2. [미션 시작] 버튼 클릭
3. FIRE_MISSION_START 메시지 전송

---

### 3. 격발 제어

**발사 확인 (CONFIRM)**:
- READY_TO_FIRE 또는 SUPPRESSING 단계에서만 활성화
- FIRE_LAUNCH_CONTROL (command=0) 전송

**발사 중단 (ABORT)**:
- SUPPRESSING 또는 VERIFYING 단계에서만 활성화
- FIRE_LAUNCH_CONTROL (command=1) 전송

**상태 요청 (REQUEST_STATUS)**:
- 언제든지 활성화
- FIRE_LAUNCH_CONTROL (command=2) 전송

---

### 4. 진압 결과

**표시 정보**:
- 발사 번호
- 발사 전 온도 (°C)
- 발사 후 온도 (°C)
- 온도 변화량 (ΔT)
- 성공/실패 여부

**색상 코딩**:
- 🟢 녹색: 성공 (온도 감소)
- 🔴 빨간색: 실패 (온도 변화 없음 또는 증가)

---

## MAVLink 메시지 연동

### 수신 메시지

#### FIRE_MISSION_STATUS (ID: 12901)
미션 상태 업데이트 (VIM4 → QGC)

**메시지 구조**:
```cpp
struct FireMissionStatus {
    uint8_t phase;                // FIRE_MISSION_PHASE (0-6)
    uint8_t progress;             // Progress 0-100%
    uint8_t remaining_projectiles;// Projectiles left
    float distance_to_target;     // Distance to target (m)
    int16_t thermal_max_temp;     // Max temp (°C * 10)
    char status_text[50];         // Status message
};
```

**QML 핸들러**:
```javascript
if (message.id === 12901) {
    formationStatus.updateMissionStatus(
        message.phase,
        message.progress,
        message.remaining_projectiles,
        message.distance_to_target,
        message.thermal_max_temp / 10.0,  // °C * 10 → °C
        message.status_text
    )
}
```

#### FIRE_SUPPRESSION_RESULT (ID: 12903)
진압 결과 업데이트 (VIM4 → QGC)

**메시지 구조**:
```cpp
struct FireSuppressionResult {
    uint8_t shot_number;          // Shot number
    int16_t temp_before;          // Temp before (°C * 10)
    int16_t temp_after;           // Temp after (°C * 10)
    uint8_t success;              // 0=failed, 1=success
};
```

**QML 핸들러**:
```javascript
if (message.id === 12903) {
    firePointPanel.updateSuppressionResult(
        message.shot_number,
        message.temp_before / 10.0,  // °C * 10 → °C
        message.temp_after / 10.0,    // °C * 10 → °C
        message.success
    )
}
```

---

### 송신 메시지

#### FIRE_MISSION_START (ID: 12900)
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
    var message = activeVehicle.createMAVLinkMessage(12900)
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

#### FIRE_LAUNCH_CONTROL (ID: 12902)
격발 제어 명령 (QGC ↔ VIM4)

**메시지 구조**:
```cpp
struct FireLaunchControl {
    uint8_t target_system;        // System ID
    uint8_t target_component;     // Component ID
    uint8_t command;              // 0=CONFIRM, 1=ABORT, 2=REQUEST_STATUS
};
```

**QML 전송**:
```javascript
// CONFIRM (command=0)
function sendMAVLinkFireCommand(confirm) {
    var message = activeVehicle.createMAVLinkMessage(12902)
    message.target_system = 1
    message.target_component = 1
    message.command = confirm ? 0 : 1  // 0=CONFIRM, 1=ABORT
    activeVehicle.sendMessage(message)
}

// REQUEST_STATUS (command=2)
function sendMAVLinkStatusRequest() {
    var message = activeVehicle.createMAVLinkMessage(12902)
    message.target_system = 1
    message.target_component = 1
    message.command = 2  // REQUEST_STATUS
    activeVehicle.sendMessage(message)
}
```

#### FIRE_SET_MODE (ID: 12904)
PX4 비행 모드 설정 (QGC → VIM4)

**메시지 구조**:
```cpp
struct FireSetMode {
    uint8_t target_system;        // System ID (FC)
    uint8_t target_component;     // Component ID (FC)
    uint8_t px4_mode;             // PX4 mode (1-8)
};
```

**PX4 모드 값**:
- 1: MANUAL
- 2: ALTCTL
- 3: POSCTL
- 4: AUTO
- 5: ACRO
- 6: OFFBOARD
- 7: STABILIZED
- 8: RATTITUDE

---

## 메시지 ID 매핑

| 메시지 이름 | ID | 방향 | 설명 |
|------------|----|----|----|
| FIRE_MISSION_START | 12900 | QGC → VIM4 | 미션 시작 |
| FIRE_MISSION_STATUS | 12901 | VIM4 → QGC | 미션 상태 |
| FIRE_LAUNCH_CONTROL | 12902 | QGC ↔ VIM4 | 격발 제어 |
| FIRE_SUPPRESSION_RESULT | 12903 | VIM4 → QGC | 진압 결과 |
| FIRE_SET_MODE | 12904 | QGC → VIM4 | PX4 모드 설정 |

---

## 커스터마이징

### 색상 변경

**FormationStatusPanel.qml**:
```javascript
function getPhaseColor(phase) {
    var colors = [
        "#CCCCCC",  // IDLE
        "#4A90E2",  // NAVIGATING
        "#FFAA00",  // SCANNING
        "#00FF00",  // READY_TO_FIRE
        "#FF5722",  // SUPPRESSING
        "#9C27B0",  // VERIFYING
        "#4CAF50"   // COMPLETE
    ]
    return colors[phase] || "#CCCCCC"
}
```

---

### 레이아웃 변경

**HumiroCustomPlugin.qml**:
```javascript
RowLayout {
    // 왼쪽 패널 너비
    FormationStatusPanel {
        Layout.preferredWidth: 320  // 픽셀 단위
    }

    // 오른쪽 패널 너비
    ColumnLayout {
        Layout.preferredWidth: 320
    }
}
```

---

## 테스트 데이터

개발 중 테스트를 위해 `HumiroCustomPlugin.qml`에 샘플 데이터가 포함되어 있습니다:

```javascript
Component.onCompleted: {
    // 테스트 미션 상태
    formationStatus.updateMissionStatus(1, 45, 5, 25.5, 85.0, "Flying to target")

    // 테스트 목표 지점
    firePointPanel.addFirePoint(1, 37.5672, 126.9788, 0.8)

    // 테스트 진압 결과
    firePointPanel.updateSuppressionResult(1, 120.0, 45.0, 1)  // 성공

    // 격발 제어 상태
    fireControlPanel.updateFireState(3, false)  // FIRE_PHASE_READY_TO_FIRE
}
```

**실제 운용 시 제거 필요**

---

## 문제 해결

### QGC에서 커스텀 플러그인이 보이지 않음

1. **Custom Plugins 활성화 확인**
   - Settings → General → Custom Plugins → Enable

2. **파일 경로 확인**
   - 정확한 경로에 qml 파일 복사 확인

3. **QGC 로그 확인**
   - Console 탭에서 에러 메시지 확인

### MAVLink 메시지가 수신되지 않음

1. **메시지 ID 확인**
   - 12900-12904 범위 확인
   - `custom_message` 라이브러리와 일치하는지 확인

2. **VIM4 MAVLink 브릿지 확인**
   - `custom_message` 라이브러리가 실행 중인지 확인
   - UDP 포트 14550 리스닝 확인

3. **메시지 구조 확인**
   - `custom_message/include/custom_message/custom_message_type.h` 참조
   - 페이로드 크기 및 필드 타입 확인

---

## 추가 기능 (향후)

- [ ] 실시간 지도에 드론 및 목표 지점 표시
- [ ] 드론 경로 히스토리 표시
- [ ] 알림 및 경고 시스템
- [ ] 상세 통계 및 로그 뷰
- [ ] PX4 모드 변경 UI 추가

---

## 참고 자료

**VIM4 문서**:
- `custom_message/README.md`: 커스텀 메시지 라이브러리 문서
- `custom_message/include/custom_message/custom_message_type.h`: 메시지 타입 정의

**QGC 문서**:
- [QGC Custom Build](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/custom_build.html)
- [QGC Plugin Development](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/plugin.html)
- [QML Tutorial](https://doc.qt.io/qt-5/qmlfirststeps.html)

---

**작성자**: Claude Code Assistant
**버전**: v2.0 (custom_message v3.1 기반)
**작성일**: 2026-01-05
