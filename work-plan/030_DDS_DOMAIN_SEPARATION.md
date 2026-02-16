# 030 DDS 통신 분리 계획: ROS_DOMAIN_ID 기반 FC/편대 격리

**버전**: v1.0
**작성일**: 2026-02-16
**상태**: 구현 중

---

## 문제 정의

다중 드론 편대 비행 시 **DDS 교차오염**으로 인한 통신 두절 반복 발생.
- 근본 원인: 메인 애플리케이션의 DDS 참여자가 eth0+WiFi+loopback 전체를 포함
- `/fmu/*` 토픽이 WiFi를 통해 다른 드론에 노출
- 결과: nav_state 진동, OFFBOARD 이탈, 미션 실패

## 목표

1. **uXRCE-DDS(FC↔VIM4)**: 단독 비행 데이터만 로컬 처리 (외부 노출 차단)
2. **일반 ROS2 DDS**: 편대 비행 토픽만 WiFi로 통신
3. **브릿지**: 필요한 FC 데이터를 로컬 UDP IPC로 전달

## 핵심 요구사항

4가지 운용 모드 모두 지원:

| 모드 | FC | SITL | 단독 | 편대 |
|------|:---:|:----:|:----:|:----:|
| FC + 단독 | O | - | O | - |
| FC + 편대 | O | - | - | O |
| SITL + 단독 | - | O | O | - |
| SITL + 편대 | - | O | - | O |

---

## 새 아키텍처

```
각 VIM4 드론:

┌─────────────────────────────────────────────────────────────────┐
│                                                                   │
│  [Process 1] micro-ros-agent          (기존 유지, 변경 없음)      │
│    Domain 0 │ eth0 + loopback                                    │
│    PX4 FC ↔ ROS2 /droneN/fmu/* 토픽 브릿지                      │
│                                                                   │
│  [Process 2] fc_bridge                (★ 신규)                   │
│    Domain 0 │ loopback 전용 (fastdds_loopback_only.xml)          │
│    /droneN/fmu/out/* 구독 → FC 상태 수신                         │
│    /droneN/fmu/in/*  발행 → FC 명령 송신                         │
│    UDP 127.0.0.1:17001 → 메인앱에 FC 상태 전송 (50Hz)           │
│    UDP 127.0.0.1:17002 ← 메인앱에서 FC 명령 수신                │
│                                                                   │
│  [Process 3] humiro_fire_suppression  (★ 수정)                   │
│    Domain 1 │ WiFi 전용 (fastdds_wifi_only.xml)                  │
│    편대 토픽: /leader_pose, /drone_position, /follower_status 등 │
│    FC 데이터: FCBridgeClient (로컬 UDP)로 수신/송신              │
│    /fmu/* 토픽 접근 완전 제거                                     │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘

데이터 흐름:
  PX4 FC ──(eth0)──▶ micro-ros-agent ──(loopback, Domain 0)──▶ fc_bridge
                                                                    │
                                                              UDP 127.0.0.1
                                                                    │
                                                                    ▼
  다른 드론 ◀──(WiFi, Domain 1)──▶ humiro_fire_suppression (메인앱)
```

---

## 시나리오별 동작

### 시나리오 1: 단독 ↔ 편대 비행 전환

- fc_bridge는 항상 `/droneN/fmu/*` 네임스페이스로 동작 (단독/편대 무관)
- 단독/편대 구분은 메인앱(Domain 1)에서 FormationController 활성화 여부로만 결정
- FC 통신 경로(Domain 0)는 모드 전환과 완전히 무관
- **전환 시 프로세스 재시작 불필요** — offboard_config.json의 mission_mode만 변경

```
단독 비행: fc_bridge(Domain 0) + 메인앱(Domain 1, FormationController 비활성)
편대 비행: fc_bridge(Domain 0) + 메인앱(Domain 1, FormationController 활성)
```

### 시나리오 2: SITL ↔ FC 모드 전환

| 컴포넌트 | SITL 모드 | FC 모드 | 전환 시 변경 |
|----------|-----------|---------|-------------|
| mavlink-router | SITL endpoint (18001) | FC endpoint (14540) | 설정 변경 + 재시작 |
| micro-ros-agent | WiFi+loopback 프로파일 | eth0+loopback 프로파일 | FastDDS 프로파일 변경 + 재시작 |
| **fc_bridge** | **loopback 전용 (변경 없음)** | **loopback 전용 (변경 없음)** | **변경 불필요** |
| **메인앱** | **WiFi 전용 (변경 없음)** | **WiFi 전용 (변경 없음)** | **변경 불필요** |

- fc_bridge는 항상 loopback으로 micro-ros-agent와 통신
- SITL/FC 전환 영향 범위가 mavlink-router + micro-ros-agent로 한정

### 시나리오 3: QGC 연결

- MAVLink Router는 변경 없음 (DDS와 완전히 별개)
- QGC ↔ VIM4 통신: `QGC → WiFi:14550 → mavlink-router → FC:14540` (기존 동일)
- 커스텀 메시지(60000-60003): `QGC → mavlink-router → UDP 15001 → 메인앱` (기존 동일)
- **QGC 연결에 대한 변경 사항 없음**

---

## 서비스 시작 순서

```
1. mavlink-router.service        (MAVLink 라우팅)
2. micro-ros-agent.service       (Domain 0, eth0+loopback)
3. fc-bridge.service             (Domain 0, loopback — NEW)
4. humiro-fire-suppression.service (Domain 1, WiFi)
5. humiro-gui.service            (Flask GUI)
```

---

## 구현 파일

### 신규 파일
| 파일 | 설명 |
|------|------|
| `navigation/src/offboard/bridge/fc_bridge_protocol.h` | IPC 메시지 구조체 |
| `navigation/src/offboard/bridge/fc_bridge_client.h/cpp` | 메인앱 측 IPC 클라이언트 |
| `navigation/src/offboard/bridge/fc_bridge_server.h/cpp` | fc_bridge 측 IPC 서버 |
| `navigation/src/offboard/bridge/fc_bridge_node.cpp` | fc_bridge 실행파일 |
| `config/fastdds_wifi_only.xml` | WiFi 전용 FastDDS 프로파일 |
| `scripts/runtime/start_fc_bridge.sh` | fc_bridge 실행 스크립트 |
| `deployment/systemd/fc-bridge.service` | systemd 서비스 |

### 수정 파일
| 파일 | 설명 |
|------|------|
| `navigation/src/offboard/offboard_manager.h/cpp` | px4_msgs 제거, FCBridgeClient 사용 |
| `ros2/src/status/status_ros2_subscriber.h/cpp` | /fmu/* 구독 제거, FCBridgeClient 사용 |
| `application/src/application_manager.h/cpp` | FCBridgeClient 공유, px4_ns 제거 |
| `humiro_msgs/msg/DronePosition.msg` | battery/GPS 필드 추가 |
| `navigation/src/offboard/CMakeLists.txt` | fc_bridge 타겟 추가 |
| `application/CMakeLists.txt` | 빌드 설정 업데이트 |
| `scripts/runtime/humiro_fire_suppression_wrapper.sh` | Domain 1, WiFi 프로파일 |

---

## 기존 문서와의 관계

| 기존 문서 | 관계 | 설명 |
|-----------|------|------|
| `022_COMMUNICATION_ARCHITECTURE.md` | **유지** | MAVLink 라우팅 변경 없음 |
| `029_PX4_NAMESPACE_FORMATION.md` | **대체** | px4_ns 방식 → fc_bridge 방식 |
| `025_COLLISION_AVOIDANCE_DESIGN.md` | **유지** | Domain 1에서 기존대로 동작 |
| `013_NEXT_STEPS_FORMATION_CONTROL.md` | **유지** | 편대 제어는 Domain 1에서 기존대로 |

---

## 리스크 및 대응

| 리스크 | 대응 |
|--------|------|
| UDP IPC 지연으로 하트비트 누락 | loopback UDP 지연 ~0.1ms, 10Hz에 영향 없음 |
| fc_bridge 크래시 | systemd Restart=always (3초), FCBridgeClient 연결 감시 → 긴급 RTL |
| SITL 모드 호환성 | fc_bridge는 항상 loopback, SITL/FC 전환 무관 |

---

**작성자**: Claude Code Assistant
**마지막 업데이트**: 2026-02-16
