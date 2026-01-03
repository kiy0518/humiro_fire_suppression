# QGC 커스텀 화면 레이아웃 - Humiro Fire Suppression

**작성일**: 2026-01-02
**QGC 버전**: 4.0+

---

## 개요

화재 진압 드론 편대 제어를 위한 QGC 커스텀 UI 레이아웃입니다.

**주요 기능**:
- 편대 상태 실시간 모니터링
- 화재 지점 관리
- 격발 제어

---

## 파일 구조

```
qgc_custom/
├── qml/
│   ├── FormationStatusPanel.qml      # 편대 상태 모니터링 패널
│   ├── FirePointPanel.qml            # 화재 지점 관리 패널
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
│  Formation   │                     │   Fire Point Panel    │
│  Status      │    QGC Map View     │                       │
│  Panel       │                     │   ┌─────────────────┐ │
│              │  (Default Map)      │   │ Target #1       │ │
│  ┌─────────┐ │                     │   │ Target #2       │ │
│  │Drone 1  │ │                     │   │ Target #3       │ │
│  │Drone 2  │ │                     │   └─────────────────┘ │
│  │Drone 3  │ │                     │                       │
│  └─────────┘ │                     │   Fire Control Panel  │
│              │                     │                       │
│              │                     │   ┌─────────────────┐ │
│              │                     │   │ Drone 1 [FIRE]  │ │
│              │                     │   │ Drone 2 [FIRE]  │ │
│              │                     │   │ Drone 3 [STOP]  │ │
│              │                     │   └─────────────────┘ │
└──────────────┴─────────────────────┴───────────────────────┘
```

---

## 사용 방법

### 1. 편대 상태 모니터링

**표시 정보**:
- 드론 ID 및 역할 (리더/팔로워)
- 현재 위치 (GPS 좌표)
- 배터리 잔량
- 소화탄 잔량 (0-6)
- 미션 상태 (IDLE, NAVIGATING, FIRE_READY 등)
- 할당된 목표

**색상 코딩**:
- 🟢 녹색: 정상 (배터리 > 30%)
- 🟡 노란색: 경고 (배터리 20-30%)
- 🔴 빨간색: 위험 (배터리 < 20% 또는 FAILED)

---

### 2. 화재 지점 관리

**기능**:
- **[+ Add Fire Point]**: 새 화재 지점 추가 (지도 클릭)
- **[Delete]**: 화재 지점 삭제
- **우선순위 표시**: HIGH/MEDIUM/LOW
- **진행 상황**: ASSIGNED/IN_PROGRESS/COMPLETED/FAILED

**화재 지점 추가 방법**:
1. [+ Add Fire Point] 버튼 클릭
2. QGC 지도에서 화재 지점 클릭
3. 우선순위 설정
4. 확인

---

### 3. 격발 제어

**개별 격발**:
- 각 드론별 [FIRE] 버튼 클릭
- 격발 중 [STOP] 버튼으로 중지 가능

**전체 격발**:
- **[Fire All]**: 모든 FIRE_READY 드론 동시 격발
- **[Stop All]**: 모든 드론 격발 중지

**격발 가능 조건**:
- 드론 상태가 FIRE_READY 또는 FIRING
- 버튼 비활성화 시 격발 불가

---

## MAVLink 메시지 연동

### 수신 메시지

#### FORMATION_MEMBER_STATUS (ID: 12920)
드론 상태 업데이트 (1Hz)

```javascript
formationStatus.updateDroneStatus(
    droneId,    // 1-255
    lat,        // degrees
    lon,        // degrees
    battery,    // 0-100
    ammo,       // 0-6
    state,      // 0-10
    targetId,   // 0=없음
    isLeader    // true/false
)
```

#### MISSION_PROGRESS (ID: 12923)
화재 지점 진행 상황 업데이트

```javascript
firePointPanel.updateFirePointProgress(
    targetId,       // 화재 지점 ID
    assignedDrone,  // 할당된 드론 ID
    progress        // 0=ASSIGNED, 1=IN_PROGRESS, 2=COMPLETED, 3=FAILED
)
```

---

### 송신 메시지

#### FIRE_COMMAND (ID: 12922)
격발 명령 전송

```javascript
sendMAVLinkFireCommand(
    droneId,  // 1-255
    enable    // true/false
)
```

#### TARGET_ASSIGNMENT (ID: 12921)
목표 할당 전송

```javascript
sendTargetAssignment(
    droneId,   // 1-255
    targetId,  // 화재 지점 ID
    lat,       // degrees
    lon,       // degrees
    priority   // 0.0-1.0
)
```

---

## 커스터마이징

### 색상 변경

**FormationStatusPanel.qml**:
```javascript
// 배경색
color: "#2C2C2C"

// 테두리색
border.color: "#4A90E2"

// 상태별 색상
function getBatteryColor(battery) {
    if (battery < 20) return "#FF0000"  // 빨간색
    if (battery < 30) return "#FFFF00"  // 노란색
    return "#00FF00"  // 녹색
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
    // 테스트 드론 3대
    formationStatus.updateDroneStatus(1, 37.5665, 126.9780, 87, 6, 3, 1, true)
    formationStatus.updateDroneStatus(2, 37.5670, 126.9785, 92, 6, 4, 2, false)
    formationStatus.updateDroneStatus(3, 37.5668, 126.9782, 28, 3, 5, 3, false)

    // 테스트 화재 지점 3개
    firePointPanel.addFirePoint(1, 37.5672, 126.9788, 0.8)
    firePointPanel.addFirePoint(2, 37.5675, 126.9790, 0.5)
    firePointPanel.addFirePoint(3, 37.5670, 126.9785, 0.9)
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
   - 12920-12923 범위 확인

2. **VIM4 MAVLink 브릿지 확인**
   - `navigation/src/mavlink_bridge/` 구현 확인

3. **MAVLink Router 설정 확인**
   - VIM4에서 MAVLink Router 실행 중인지 확인

---

## 추가 기능 (향후)

- [ ] 실시간 지도에 드론 및 화재 지점 표시
- [ ] 드론 경로 히스토리 표시
- [ ] 알림 및 경고 시스템
- [ ] 편대 자동 할당 알고리즘 UI
- [ ] 상세 통계 및 로그 뷰

---

## 참고 자료

**VIM4 문서**:
- `work-plan/QGC_DEVELOPMENT_GUIDE.md`: QGC 개발 가이드
- `work-plan/NEXT_STEPS_FORMATION_CONTROL.md`: 편대 제어 계획
- `work-plan/ROS2_TOPIC_ARCHITECTURE.md`: ROS2 토픽 구조

**QGC 문서**:
- [QGC Custom Build](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/custom_build.html)
- [QGC Plugin Development](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/plugin.html)
- [QML Tutorial](https://doc.qt.io/qt-5/qmlfirststeps.html)

---

**작성자**: Claude Code Assistant
**버전**: v1.0
**작성일**: 2026-01-02
