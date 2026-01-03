# QGC 개발 가이드 - 편대 제어 통신 프로토콜

**작성일**: 2026-01-03
**대상**: QGC 프로그래머
**목적**: 화재 진압 드론 편대 제어 시스템 연동

---

## 목차

1. [시스템 개요](#시스템-개요)
2. [통신 아키텍처](#통신-아키텍처)
3. [편대 제어 프로토콜](#편대-제어-프로토콜)
4. [MAVLink 메시지 정의](#mavlink-메시지-정의)
5. [QGC UI 요구사항](#qgc-ui-요구사항)
6. [개발 우선순위](#개발-우선순위)
7. [예제 코드](#예제-코드)

---

## 시스템 개요

### 화재 진압 드론 시스템 구성

```
┌─────────────────────────────────────────────────────┐
│                       QGC                           │
│  (지상 관제소 - 편대 조율)                           │
│                                                     │
│  - 편대 상태 모니터링                                │
│  - 화재 지점 관리                                    │
│  - 목표 할당 (수동/자동)                             │
│  - 격발 명령                                         │
└────────────────┬────────────────────────────────────┘
                 │
              MAVLink
          (커스텀 메시지)
                 │
    ┌────────────┴────────────┬────────────┐
    │                         │            │
┌───▼────┐              ┌────▼───┐   ┌────▼───┐
│ Drone 1│              │Drone 2 │   │Drone 3 │
│(리더)  │              │        │   │        │
│        │              │        │   │        │
│ VIM4   │              │ VIM4   │   │ VIM4   │
│  └─PX4 │              │  └─PX4 │   │  └─PX4 │
└────────┘              └────────┘   └────────┘
```

### 하이브리드 편대 제어 방식

**리더 드론 (Drone 1)**:
- 화재 지점 분석
- 목표 할당 (자동)
- 편대원 상태 모니터링
- QGC와 직접 통신

**팔로워 드론 (Drone 2, 3, ...)**:
- 할당된 목표 독립 실행
- 자신의 상태 보고
- QGC와 직접 통신

**QGC (지상 관제소)**:
- 모든 드론 상태 모니터링
- 화재 지점 입력/수정
- 목표 할당 확인/변경
- 개별 격발 명령

---

## 통신 아키텍처

### Phase 1: 기본 통신 (현재)

```
QGC ◄──MAVLink──► Drone (VIM4/PX4)
     (표준 메시지)
```

**구현 완료**:
- 시동/이륙/착륙 명령
- 위치/자세 텔레메트리
- 배터리/GPS 상태
- RTSP 영상 스트리밍

---

### Phase 2: 편대 제어 통신 (신규 구현)

```
QGC ◄──MAVLink Custom──► Formation Communication Module ◄──ROS2──► Navigation Module
          (12920-12923)         (VIM4 내부)                      (편대 제어)
```

**통신 흐름**:

1. **VIM4 → QGC** (상태 보고):
   - ROS2 토픽 → Formation Communication Module → MAVLink 메시지 → QGC

2. **QGC → VIM4** (명령):
   - QGC → MAVLink 메시지 → Formation Communication Module → ROS2 토픽

**구현 위치**:
- VIM4: `navigation/src/formation_comm/` (편대 통신 모듈)
- QGC: 커스텀 메시지 핸들러 추가

---

## 편대 제어 프로토콜

### 통신 흐름

#### 1. 편대 상태 보고 (드론 → QGC)

**빈도**: 1Hz (1초마다)
**방향**: 모든 드론 → QGC

**데이터**:
- 드론 ID
- 현재 위치 (GPS)
- 배터리 잔량
- 소화탄 잔량 (0-6)
- 미션 상태
- 할당된 목표 ID

---

#### 2. 목표 할당 (QGC → 드론)

**빈도**: 이벤트 기반
**방향**: QGC → 개별 드론

**데이터**:
- 드론 ID
- 목표 ID
- 목표 위치 (GPS)
- 우선순위

---

#### 3. 격발 명령 (QGC → 드론)

**빈도**: 이벤트 기반
**방향**: QGC → 개별 드론

**데이터**:
- 드론 ID
- 격발 활성화 (true/false)

---

#### 4. 진행 상황 보고 (드론 → QGC)

**빈도**: 상태 변경 시
**방향**: 개별 드론 → QGC

**데이터**:
- 드론 ID
- 목표 ID
- 진행 상태 (ASSIGNED, IN_PROGRESS, COMPLETED, FAILED)
- 사용한 소화탄 수

---

## MAVLink 메시지 정의

### 1. FORMATION_MEMBER_STATUS (드론 → QGC)

**Message ID**: 12920 (임시, MAVLink 공식 할당 필요)

```c
<message id="12920" name="FORMATION_MEMBER_STATUS">
  <description>Formation member status information</description>
  <field type="uint8_t" name="drone_id">Drone ID (1-255)</field>
  <field type="int32_t" name="lat">Latitude (degrees * 1e7)</field>
  <field type="int32_t" name="lon">Longitude (degrees * 1e7)</field>
  <field type="int32_t" name="alt">Altitude MSL (mm)</field>
  <field type="uint8_t" name="battery_percent">Battery remaining (0-100%)</field>
  <field type="uint8_t" name="ammo_count">Ammunition count (0-6)</field>
  <field type="uint8_t" name="mission_state">Mission state (see enum)</field>
  <field type="uint8_t" name="target_id">Assigned target ID (0=none)</field>
  <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
</message>
```

**Mission State Enum**:
```c
<enum name="MISSION_STATE">
  <entry value="0" name="IDLE">Idle</entry>
  <entry value="1" name="ARMING">Arming</entry>
  <entry value="2" name="TAKEOFF">Takeoff</entry>
  <entry value="3" name="NAVIGATING">Navigating</entry>
  <entry value="4" name="DESTINATION_REACHED">Destination reached</entry>
  <entry value="5" name="FIRE_READY">Fire ready</entry>
  <entry value="6" name="FIRING">Firing</entry>
  <entry value="7" name="RETURNING">Returning</entry>
  <entry value="8" name="LANDING">Landing</entry>
  <entry value="9" name="COMPLETED">Mission completed</entry>
  <entry value="10" name="FAILED">Mission failed</entry>
</enum>
```

---

### 2. TARGET_ASSIGNMENT (QGC → 드론)

**Message ID**: 12921 (임시)

```c
<message id="12921" name="TARGET_ASSIGNMENT">
  <description>Target assignment to formation member</description>
  <field type="uint8_t" name="drone_id">Target drone ID</field>
  <field type="uint8_t" name="target_id">Fire point ID</field>
  <field type="int32_t" name="lat">Target latitude (degrees * 1e7)</field>
  <field type="int32_t" name="lon">Target longitude (degrees * 1e7)</field>
  <field type="int32_t" name="alt">Target altitude MSL (mm)</field>
  <field type="float" name="priority">Priority (0.0-1.0)</field>
  <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
</message>
```

---

### 3. FIRE_COMMAND (QGC → 드론)

**Message ID**: 12922 (임시)

```c
<message id="12922" name="FIRE_COMMAND">
  <description>Fire command to drone</description>
  <field type="uint8_t" name="drone_id">Target drone ID</field>
  <field type="uint8_t" name="fire_enable">Fire enable (0=disable, 1=enable)</field>
  <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
</message>
```

---

### 4. MISSION_PROGRESS (드론 → QGC)

**Message ID**: 12923 (임시)

```c
<message id="12923" name="MISSION_PROGRESS">
  <description>Mission progress report</description>
  <field type="uint8_t" name="drone_id">Drone ID</field>
  <field type="uint8_t" name="target_id">Target ID</field>
  <field type="uint8_t" name="progress_status">Progress status (see enum)</field>
  <field type="uint8_t" name="ammo_used">Ammunition used</field>
  <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
</message>
```

**Progress Status Enum**:
```c
<enum name="PROGRESS_STATUS">
  <entry value="0" name="ASSIGNED">Target assigned</entry>
  <entry value="1" name="IN_PROGRESS">Mission in progress</entry>
  <entry value="2" name="COMPLETED">Mission completed</entry>
  <entry value="3" name="FAILED">Mission failed</entry>
</enum>
```

---

## QGC UI 요구사항

### 1. 편대 상태 모니터링 패널

**표시 정보** (드론별):
```
┌─────────────────────────────────────┐
│ 편대 상태 모니터링                   │
├─────────────────────────────────────┤
│ Drone 1 (리더) ✓                    │
│  ├─ 위치: 37.5665°N, 126.9780°E    │
│  ├─ 배터리: 87%                     │
│  ├─ 소화탄: 6/6                     │
│  ├─ 상태: NAVIGATING                │
│  └─ 목표: Target #1                 │
│                                     │
│ Drone 2 ✓                           │
│  ├─ 위치: 37.5670°N, 126.9785°E    │
│  ├─ 배터리: 92%                     │
│  ├─ 소화탄: 6/6                     │
│  ├─ 상태: DESTINATION_REACHED       │
│  └─ 목표: Target #2                 │
│                                     │
│ Drone 3 ⚠                           │
│  ├─ 위치: 37.5668°N, 126.9782°E    │
│  ├─ 배터리: 28% (LOW)               │
│  ├─ 소화탄: 3/6                     │
│  ├─ 상태: FIRE_READY                │
│  └─ 목표: Target #3                 │
└─────────────────────────────────────┘
```

**색상 코딩**:
- ✓ 녹색: 정상
- ⚠ 노란색: 경고 (배터리 < 30%)
- ✗ 빨간색: 에러/실패

---

### 2. 화재 지점 관리 패널

**기능**:
- 화재 지점 추가/삭제
- 지도에 표시
- 우선순위 설정

```
┌─────────────────────────────────────┐
│ 화재 지점 관리                       │
├─────────────────────────────────────┤
│ Target #1 🔥                        │
│  ├─ 위치: 37.5672°N, 126.9788°E    │
│  ├─ 우선순위: 높음                  │
│  ├─ 할당: Drone 1                   │
│  └─ 상태: IN_PROGRESS               │
│                                     │
│ Target #2 🔥                        │
│  ├─ 위치: 37.5675°N, 126.9790°E    │
│  ├─ 우선순위: 중간                  │
│  ├─ 할당: Drone 2                   │
│  └─ 상태: COMPLETED ✓               │
│                                     │
│ Target #3 🔥                        │
│  ├─ 위치: 37.5670°N, 126.9785°E    │
│  ├─ 우선순위: 높음                  │
│  ├─ 할당: Drone 3                   │
│  └─ 상태: ASSIGNED                  │
│                                     │
│ [+ 화재 지점 추가]                   │
└─────────────────────────────────────┘
```

---

### 3. 목표 할당 패널

**기능**:
- 자동 할당 (리더 알고리즘)
- 수동 할당 (GCS 지정)
- 할당 변경

```
┌─────────────────────────────────────┐
│ 목표 할당                            │
├─────────────────────────────────────┤
│ [ ] 자동 할당 (리더)                │
│ [x] 수동 할당 (GCS)                 │
│                                     │
│ Target #1 → Drone 1 [변경]         │
│ Target #2 → Drone 2 [변경]         │
│ Target #3 → Drone 3 [변경]         │
│                                     │
│ [모두 할당] [할당 취소]              │
└─────────────────────────────────────┘
```

---

### 4. 격발 제어 패널

**기능**:
- 개별 드론 격발 명령
- 전체 격발
- 격발 중지

```
┌─────────────────────────────────────┐
│ 격발 제어                            │
├─────────────────────────────────────┤
│ Drone 1: [격발] 상태: FIRE_READY   │
│ Drone 2: [격발] 상태: FIRING...    │
│ Drone 3: [격발] 상태: COMPLETED ✓  │
│                                     │
│ [전체 격발] [전체 중지]              │
└─────────────────────────────────────┘
```

---

### 5. 지도 뷰

**표시 요소**:
- 드론 위치 (아이콘 + ID)
- 화재 지점 (불 아이콘)
- 할당 라인 (드론 → 목표)
- 드론 경로 (이동 궤적)

```
        Target #1 🔥
           ↗
    Drone 1 ●───────┐
                    │
                    │
    Drone 2 ●───→ Target #2 🔥


    Drone 3 ●───→ Target #3 🔥
```

---

## 개발 우선순위

### Phase 1: 기본 통신 (1주)

**VIM4 측**:
- [x] ROS2 토픽 정의 (완료)
- [ ] Formation Communication Module 구현
  - FORMATION_MEMBER_STATUS 발행
  - TARGET_ASSIGNMENT 수신
  - FIRE_COMMAND 수신

**QGC 측**:
- [ ] MAVLink 커스텀 메시지 정의 (XML)
- [ ] FORMATION_MEMBER_STATUS 수신 처리
- [ ] 기본 상태 모니터링 UI

---

### Phase 2: UI 구현 (1주)

**QGC 측**:
- [ ] 편대 상태 모니터링 패널
- [ ] 화재 지점 관리 패널
- [ ] 지도 뷰 (드론 + 목표)

---

### Phase 3: 제어 기능 (1주)

**QGC 측**:
- [ ] 목표 할당 패널
- [ ] TARGET_ASSIGNMENT 발행
- [ ] FIRE_COMMAND 발행
- [ ] 격발 제어 패널

---

### Phase 4: 통합 테스트 (1주)

**통합**:
- [ ] VIM4 ↔ QGC 연동 테스트
- [ ] 2대 편대 시나리오
- [ ] 3대 편대 시나리오
- [ ] 장애 시나리오

---

## 예제 코드

### VIM4: Formation Communication Module

**파일**: `navigation/src/formation_comm/formation_status_publisher.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <mavlink/v2.0/common/mavlink.h>
#include "custom_msgs/msg/member_status.hpp"

class FormationStatusPublisher : public rclcpp::Node {
public:
    FormationStatusPublisher() : Node("formation_status_publisher") {
        status_sub_ = this->create_subscription<custom_msgs::msg::MemberStatus>(
            "/formation/member_status", 10,
            std::bind(&FormationStatusPublisher::statusCallback, this, std::placeholders::_1)
        );
    }

private:
    void statusCallback(const custom_msgs::msg::MemberStatus::SharedPtr msg) {
        mavlink_message_t mavlink_msg;
        mavlink_formation_member_status_t status;

        status.drone_id = msg->drone_id;
        status.lat = msg->position.latitude * 1e7;
        status.lon = msg->position.longitude * 1e7;
        status.alt = msg->position.altitude * 1000;
        status.battery_percent = msg->battery_percent;
        status.ammo_count = msg->ammo_count;
        status.mission_state = msg->state;
        status.target_id = msg->target_id;
        status.timestamp = msg->timestamp;

        mavlink_msg_formation_member_status_encode(
            1, 1, &mavlink_msg, &status
        );

        sendToGCS(mavlink_msg);
    }

    void sendToGCS(const mavlink_message_t& msg) {
        // MAVLink Router를 통해 QGC로 전송
        // UDP 또는 Serial 사용
    }

    rclcpp::Subscription<custom_msgs::msg::MemberStatus>::SharedPtr status_sub_;
};
```

---

### QGC: 상태 수신

**파일**: `src/Vehicle/FormationManager.cc`

```cpp
void FormationManager::handleFormationMemberStatus(
    const mavlink_message_t* message)
{
    mavlink_formation_member_status_t status;
    mavlink_msg_formation_member_status_decode(message, &status);

    // 드론 상태 업데이트
    FormationMember* member = getMember(status.drone_id);
    if (member) {
        member->setPosition(
            status.lat / 1e7,
            status.lon / 1e7,
            status.alt / 1000.0
        );
        member->setBattery(status.battery_percent);
        member->setAmmo(status.ammo_count);
        member->setState(status.mission_state);
        member->setTarget(status.target_id);

        emit memberStatusUpdated(member);
    }
}
```

---

### QGC: 목표 할당 발행

**파일**: `src/Vehicle/FormationManager.cc`

```cpp
void FormationManager::assignTarget(
    uint8_t drone_id,
    uint8_t target_id,
    const QGeoCoordinate& position,
    float priority)
{
    mavlink_message_t msg;
    mavlink_target_assignment_t assignment;

    assignment.drone_id = drone_id;
    assignment.target_id = target_id;
    assignment.lat = position.latitude() * 1e7;
    assignment.lon = position.longitude() * 1e7;
    assignment.alt = position.altitude() * 1000;
    assignment.priority = priority;
    assignment.timestamp = QDateTime::currentMSecsSinceEpoch() * 1000;

    mavlink_msg_target_assignment_encode(
        _vehicle->id(), MAV_COMP_ID_MISSIONPLANNER,
        &msg, &assignment
    );

    _vehicle->sendMessageOnLink(_vehicle->priorityLink(), msg);
}
```

---

### QGC: 격발 명령

**파일**: `src/Vehicle/FormationManager.cc`

```cpp
void FormationManager::sendFireCommand(uint8_t drone_id, bool enable)
{
    mavlink_message_t msg;
    mavlink_fire_command_t fire_cmd;

    fire_cmd.drone_id = drone_id;
    fire_cmd.fire_enable = enable ? 1 : 0;
    fire_cmd.timestamp = QDateTime::currentMSecsSinceEpoch() * 1000;

    mavlink_msg_fire_command_encode(
        _vehicle->id(), MAV_COMP_ID_MISSIONPLANNER,
        &msg, &fire_cmd
    );

    _vehicle->sendMessageOnLink(_vehicle->priorityLink(), msg);
}
```

---

## MAVLink XML 정의

**파일**: `mavlink/message_definitions/v1.0/humiro.xml`

```xml
<?xml version="1.0"?>
<mavlink>
  <include>common.xml</include>

  <enums>
    <enum name="MISSION_STATE">
      <description>Mission state for formation control</description>
      <entry value="0" name="MISSION_STATE_IDLE">
        <description>Idle</description>
      </entry>
      <entry value="1" name="MISSION_STATE_ARMING">
        <description>Arming</description>
      </entry>
      <entry value="2" name="MISSION_STATE_TAKEOFF">
        <description>Takeoff</description>
      </entry>
      <entry value="3" name="MISSION_STATE_NAVIGATING">
        <description>Navigating to target</description>
      </entry>
      <entry value="4" name="MISSION_STATE_DESTINATION_REACHED">
        <description>Destination reached</description>
      </entry>
      <entry value="5" name="MISSION_STATE_FIRE_READY">
        <description>Fire ready</description>
      </entry>
      <entry value="6" name="MISSION_STATE_FIRING">
        <description>Firing</description>
      </entry>
      <entry value="7" name="MISSION_STATE_RETURNING">
        <description>Returning to launch</description>
      </entry>
      <entry value="8" name="MISSION_STATE_LANDING">
        <description>Landing</description>
      </entry>
      <entry value="9" name="MISSION_STATE_COMPLETED">
        <description>Mission completed</description>
      </entry>
      <entry value="10" name="MISSION_STATE_FAILED">
        <description>Mission failed</description>
      </entry>
    </enum>

    <enum name="PROGRESS_STATUS">
      <description>Mission progress status</description>
      <entry value="0" name="PROGRESS_STATUS_ASSIGNED">
        <description>Target assigned</description>
      </entry>
      <entry value="1" name="PROGRESS_STATUS_IN_PROGRESS">
        <description>Mission in progress</description>
      </entry>
      <entry value="2" name="PROGRESS_STATUS_COMPLETED">
        <description>Mission completed</description>
      </entry>
      <entry value="3" name="PROGRESS_STATUS_FAILED">
        <description>Mission failed</description>
      </entry>
    </enum>
  </enums>

  <messages>
    <message id="12920" name="FORMATION_MEMBER_STATUS">
      <description>Formation member status information</description>
      <field type="uint8_t" name="drone_id">Drone ID (1-255)</field>
      <field type="int32_t" name="lat">Latitude (degrees * 1e7)</field>
      <field type="int32_t" name="lon">Longitude (degrees * 1e7)</field>
      <field type="int32_t" name="alt">Altitude MSL (mm)</field>
      <field type="uint8_t" name="battery_percent">Battery remaining (0-100%)</field>
      <field type="uint8_t" name="ammo_count">Ammunition count (0-6)</field>
      <field type="uint8_t" name="mission_state" enum="MISSION_STATE">Mission state</field>
      <field type="uint8_t" name="target_id">Assigned target ID (0=none)</field>
      <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
    </message>

    <message id="12921" name="TARGET_ASSIGNMENT">
      <description>Target assignment to formation member</description>
      <field type="uint8_t" name="drone_id">Target drone ID</field>
      <field type="uint8_t" name="target_id">Fire point ID</field>
      <field type="int32_t" name="lat">Target latitude (degrees * 1e7)</field>
      <field type="int32_t" name="lon">Target longitude (degrees * 1e7)</field>
      <field type="int32_t" name="alt">Target altitude MSL (mm)</field>
      <field type="float" name="priority">Priority (0.0-1.0)</field>
      <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
    </message>

    <message id="12922" name="FIRE_COMMAND">
      <description>Fire command to drone</description>
      <field type="uint8_t" name="drone_id">Target drone ID</field>
      <field type="uint8_t" name="fire_enable">Fire enable (0=disable, 1=enable)</field>
      <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
    </message>

    <message id="12923" name="MISSION_PROGRESS">
      <description>Mission progress report</description>
      <field type="uint8_t" name="drone_id">Drone ID</field>
      <field type="uint8_t" name="target_id">Target ID</field>
      <field type="uint8_t" name="progress_status" enum="PROGRESS_STATUS">Progress status</field>
      <field type="uint8_t" name="ammo_used">Ammunition used</field>
      <field type="uint64_t" name="timestamp">Timestamp (microseconds)</field>
    </message>
  </messages>
</mavlink>
```

---

## 참고 자료

### VIM4 측 문서
- `work-plan/NEXT_STEPS_FORMATION_CONTROL.md`: 편대 제어 구현 계획
- `work-plan/ROS2_TOPIC_ARCHITECTURE.md`: ROS2 토픽 구조
- `work-plan/VIM4_AUTONOMOUS_CONTROL_PLAN.md`: 자율 제어 시스템
- `work-plan/FIRE_SUPPRESSION_SCENARIO.md`: 화재 진압 시나리오 (메시지 흐름)

### 관련 코드
- `navigation/src/offboard/`: 자율 비행 핸들러 (Phase 1 완료)
- `workspaces/mavlink-router/`: MAVLink Router 설정

---

## 연락 및 협업

**VIM4 개발팀**:
- Formation Communication Module: `navigation/src/formation_comm/` (예정)
- Navigation Module: `navigation/src/offboard/`

**프로토콜 조율**:
- MAVLink 메시지 ID 할당
- 메시지 포맷 확정
- 테스트 시나리오 합의

---

**작성자**: Claude Code Assistant
**버전**: v3.0 (정확한 기술 용어 사용)
**작성일**: 2026-01-03
**다음 업데이트**: Formation Communication Module 구현 후
