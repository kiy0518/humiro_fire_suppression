# CLAUDE.md - Humiro Fire Suppression 프로그래밍 가이드

**목적**: Claude Code가 이 프로젝트에서 코드를 작성할 때 참고하는 가이드

**버전**: v3.1 (2026-02-08)
**최신 태그**: v0.26.4
**총 코드량**: 12,000+ LOC

**프로젝트 경로**: `/home/khadas/humiro_fire_suppression`

---

## 필수 확인 사항 (모든 작업 전)

**코드 작성 또는 답변 전에 반드시 `work-plan/` 폴더를 확인하세요!**

```bash
# work-plan 폴더 위치
/home/khadas/humiro_fire_suppression/work-plan/
```

### work-plan 문서 전체 목록

#### 프로젝트 관리
| 파일 | 내용 |
|------|------|
| `000_PROJECT_PROGRESS_REPORT.md` | 프로젝트 진행 현황 |
| `001_PROJECT_MASTER_PLAN.md` | 전체 마스터 플랜 |
| `007_CURRENT_STATUS.md` | 현재 상태 |
| `026_IMPLEMENTATION_GAP_ANALYSIS.md` | 구현 현황 분석 |

#### Phase 완료 보고서
| 파일 | 내용 |
|------|------|
| `002_PHASE2_COMPLETE.md` | Phase 2 완료 |
| `006_PHASE3_COMPLETE.md` | Phase 3 완료 |
| `012_20260102_autonomous_phase1_complete.md` | 자율 비행 Phase 1 완료 |

#### 자율 비행 / Navigation
| 파일 | 내용 |
|------|------|
| `014_VIM4_AUTONOMOUS_CONTROL_PLAN.md` | VIM4 자율 제어 계획 |
| `024_01_OFFBOARD_MODE_SEQUENCE_V2.md` | OFFBOARD 모드 시퀀스 |
| `013_NEXT_STEPS_FORMATION_CONTROL.md` | 편대 제어 다음 단계 |
| `025_COLLISION_AVOIDANCE_DESIGN.md` | 충돌 방지 설계 |
| `025_COLLISION_AVOIDANCE_VERIFICATION.md` | 충돌 방지 검증 |
| `028_OFFBOARD_HANDLER_REFACTORING.md` | OFFBOARD 핸들러 리팩토링 계획 |
| `033_LANDING_DETECTION_IMPROVEMENT.md` | 착지 감지 및 DISARM 시퀀스 개선 (vehicle_land_detected 활용) |

#### 통신 / 메시지
| 파일 | 내용 |
|------|------|
| `003_ROS2_COMMUNICATION_EXPLANATION.md` | ROS2 통신 설명 |
| `008_ROS2_TOPIC_ARCHITECTURE.md` | ROS2 토픽 아키텍처 |
| `010_ROS2_COMMUNICATION_MODULE.md` | ROS2 통신 모듈 |
| `019_MAVLINK_CUSTOM_MESSAGE.md` | MAVLink 커스텀 메시지 |
| `021_MAVLINK_ARCHITECTURE_REFACTORING.md` | MAVLink 아키텍처 리팩토링 |
| `022_COMMUNICATION_ARCHITECTURE.md` | 통신 아키텍처 |
| `022_COMMUNICATION_ARCHITECTURE_STATUS.md` | 통신 아키텍처 상태 |
| `029_PX4_NAMESPACE_FORMATION.md` | PX4 네임스페이스 편대 (030으로 대체됨) |
| `030_DDS_DOMAIN_SEPARATION.md` | DDS 도메인 분리 (FC/편대 격리) |

#### 화재 진압 시나리오
| 파일 | 내용 |
|------|------|
| `015_FIRE_SUPPRESSION_SCENARIO.md` | 화재 진압 시나리오 |
| `020_RTK_GPS_COORDINATE_FORMAT.md` | RTK GPS 좌표 형식 |

#### 스트리밍 / GUI
| 파일 | 내용 |
|------|------|
| `009_STATUS_MONITORING_PLAN.md` | 상태 모니터링 계획 |
| `017_STREAMING_ARCHITECTURE_REFACTORING.md` | 스트리밍 아키텍처 리팩토링 |
| `016_QGC_DEVELOPMENT_GUIDE.md` | QGC 개발 가이드 |
| `027_GUI_MANAGEMENT_TOOL_PLAN.md` | GUI 관리 도구 계획 |

#### 기타
| 파일 | 내용 |
|------|------|
| `001-usb-camera-autosuspend-fix.md` | USB 카메라 자동 절전 수정 |
| `004_REFACTORING_COMPLETE_SUMMARY.md` | 리팩토링 완료 요약 |
| `005_QUICK_START_ROS2_TOPICS.md` | ROS2 토픽 빠른 시작 |
| `011_20260102_px4_msgs_fix.md` | PX4 메시지 수정 |
| `018_README.md` | README |
| `023_SCRIPT_DEVICE_CONFIG_INTEGRATION.md` | 스크립트 기기 설정 통합 |

### 문서 동기화 유지 (필수)
**`work-plan/` 폴더에 문서가 추가되거나 삭제되면 이 CLAUDE.md 파일의 목록도 함께 업데이트하세요!**

```bash
# work-plan 문서 목록 확인
ls -1 /home/khadas/humiro_fire_suppression/work-plan/*.md
```

### 확인 절차
1. **코드 수정 전**: 해당 모듈의 설계 문서 확인
2. **새 기능 추가 전**: 마스터 플랜과 일치 여부 확인
3. **버그 수정 전**: 기존 설계 의도 파악
4. **질문 답변 전**: 최신 진행 상황 확인

---

## 프로젝트 개요

**Humiro Fire Suppression System** - 다중 드론 협조를 위한 ROS2 기반 자율 화재 진압 시스템

**핵심 기술**:
- C++17 (모든 핵심 모듈)
- ROS2 Humble
- PX4 Firmware v1.16.0
- OpenCV + GStreamer
- MAVLink 커스텀 메시지 (60000-60003)

**현재 상태** (2026-02-08, v0.15.7):
- 열화상 시스템
- LiDAR 거리 측정
- 스트리밍 시스템 (HTTP/RTSP)
- 상태 모니터링 OSD
- VIM4 자율 제어 (OFFBOARD 미션, velocity 기반 부드러운 선회, 목표지점 호버링)
- MAVLink 커스텀 메시지 (미션 시작/중지/RTL)
- 웹 GUI 관리 도구 (오프보드 설정 페이지 포함)
- ROS2 DDS 기반 편대비행 (리더/팔로워, 고정 경로선 오프셋, SUPPRESS 미러링)

---

## 코딩 표준

### C/C++ 우선 정책 (필수)

**모든 핵심 모듈은 C++17로 작성**:
- 실시간 처리 -> C++ 필수
- 하드웨어 인터페이스 -> C++ 필수
- 이미지/비디오 처리 -> C++ 필수
- ROS2 래퍼 -> Python 허용 (C++ 라이브러리 호출만)

**코딩 스타일**:
- C++17 표준, Google C++ Style Guide
- 스마트 포인터 사용 (`std::unique_ptr`, `std::shared_ptr`)
- 멀티스레드: `std::atomic`, 스레드 안전 큐 사용
- 에러 처리: 명확한 에러 코드 반환

**금지 사항**:
- 하드코딩된 경로 (환경 변수 사용)
- Python으로 실시간 처리
- 불필요한 메모리 복사
- 불필요한 md파일 생성

---

## 버전 관리 및 Git 규칙

### 새 세션 시작 시 (필수)

**커밋/태그 작업 전 반드시 최신 버전 확인**:
```bash
cd /home/khadas/humiro_fire_suppression
git fetch origin
git log -1 --oneline                    # 최신 커밋 확인
git tag --sort=-v:refname | grep "^v0" | head -1  # 최신 태그 확인
```

- 현재 버전을 파악한 후 다음 버전 번호 결정
- 예: 현재 v0.13.3이면 -> PATCH는 v0.13.4, MINOR는 v0.14.0

### 버전 체계: Semantic Versioning

**형식**: `vMAJOR.MINOR.PATCH`

| 단계 | 버전 변경 | 조건 |
|------|-----------|------|
| **개발 단계** | MINOR (v0.13.x -> v0.14.x) | Phase 완료, 주요 기능 추가, 새 모듈 통합 |
| **개발 단계** | PATCH (v0.13.3 -> v0.13.4) | 버그 수정, 성능 개선, 작은 개선사항 |
| **정식 출시 후** | MAJOR (v1.x -> v2.x) | 하위 호환성이 깨지는 변경, 아키텍처 전면 개편 |
| **정식 출시 후** | MINOR (v1.0.x -> v1.1.x) | 새 기능 추가 (하위 호환 유지) |
| **정식 출시 후** | PATCH (v1.0.0 -> v1.0.1) | 버그 수정, 문서 업데이트 |

### Git 커밋/태그 메시지 형식

**커밋 메시지와 태그 메시지는 동일한 전체 내용을 포함** (한 줄 요약 금지)

**시간 규칙**:
- 타임존: **서울 시간 (KST, Asia/Seoul)** 사용
- 날짜에 **실제 분/초까지 정확히 기록** (00:00:00 금지, 커밋 시점의 실제 시간 사용)
- `TZ=Asia/Seoul date "+%Y. %m. %d. %H:%M:%S"` 명령으로 현재 시간 확인

```
날짜: YYYY. MM. DD. HH:MM:SS

**v0.x.x: 제목**

   **테스트 메모**
   - 실기체 테스트 항목 1
   - 실기체 테스트 항목 2

   **추가된 기능**
   - 기능 1 설명
   - 기능 2 설명

   **변경 사항**
   - 변경 내용 1
   - 변경 내용 2

   **버그 수정**
   - 수정 내용 1

   **수정 파일**
   - path/to/file1 - 설명
   - path/to/file2 - 설명

   **알려진 이슈 및 추가 테스트(미수정)**
   - 이전 태그에서 승계된 미수정 이슈
   - 새로 발견된 이슈
```

### Git 명령어

**날짜 지정 커밋/태그** (사용자가 날짜 지정 시):
```bash
# 커밋
GIT_AUTHOR_DATE="YYYY-MM-DDTHH:MM:SS" GIT_COMMITTER_DATE="YYYY-MM-DDTHH:MM:SS" git commit -m "전체 메시지"

# 태그 (커밋 메시지와 동일한 전체 내용)
GIT_COMMITTER_DATE="YYYY-MM-DDTHH:MM:SS" git tag -a v0.x.x -m "전체 메시지"

# 푸시
git push origin main
git push origin v0.x.x
```

### GitHub 태그 히스토리 릴리스 노트 작성 (필수)

**모든 태그는 상세한 릴리스 노트를 포함해야 합니다**:

1. **Annotated Tag 사용**:
   - Lightweight tag 금지: `git tag v0.x.x`
   - Annotated tag 필수: `git tag -a v0.x.x -m "메시지"`

2. **릴리스 노트 형식**:
   - `#` 사용 금지: Git이 `#`으로 시작하는 줄을 주석으로 처리하여 삭제함
   - 섹션 헤더는 `###` 대신 `**볼드**` 형식 사용
   ```bash
   # 메시지 파일 생성
   cat > /tmp/tag_message.txt <<'EOF'
   날짜: YYYY. MM. DD. HH:MM:SS

   **v0.x.x: 제목**

   **테스트 메모**
   - 실기체 테스트 항목 1
   - 실기체 테스트 항목 2

   **추가된 기능**
   - 기능 1 설명
   - 기능 2 설명

   **변경 사항**
   - 변경 내용 1
   - 변경 내용 2

   **버그 수정**
   - 수정 내용 1

   **수정 파일**
   - path/to/file1 - 설명
   - path/to/file2 - 설명

   **알려진 이슈 및 추가 테스트(미수정)**
   - 이전 태그에서 승계된 미수정 이슈
   - 새로 발견된 이슈
   EOF

   # 태그 생성
   git tag -a v0.x.x -F /tmp/tag_message.txt

   # GitHub에 푸시
   git push origin v0.x.x
   ```

3. **기존 태그 수정** (필요 시):
   ```bash
   git tag -d v0.x.x
   git tag -a v0.x.x -F /tmp/tag_message.txt
   git push origin v0.x.x --force
   ```

### 태그 생성 시 필수 작업

1. **CLAUDE.md 최신 태그 업데이트**: 태그 생성 전 반드시 CLAUDE.md 상단의 `**최신 태그**:` 값을 새 태그 버전으로 수정하고 커밋에 포함
2. **알려진 이슈 승계**: 태그 메시지에 `**알려진 이슈 및 추가 테스트(미수정)**` 섹션을 포함하여 이전 태그의 미수정 이슈를 승계. 새로 발견된 이슈 추가, 해결된 이슈 제거. 이전 태그의 알려진 이슈는 `git tag -l --format='%(contents)' <이전태그>` 로 확인

### Git 추적 필수 설정 파일

다음 설정 파일은 기체 공통 요소이므로 **반드시 Git에 커밋하여 관리**:
- `config/custom_params.json` — 기체 프로파일별 FC 파라미터 권장값 (모든 기체 공통 기준)
- `config/offboard_config.json` — 오프보드 미션 설정 (목표 고도, 속도 등 공통 설정)

태그 생성 시 이 파일들의 변경사항도 함께 커밋에 포함할 것.

### 기존 태그 처리

기존 v1.x 태그들 (v1.0-px4-msgs-fix, v1.1-autonomous-phase1 등)은 **히스토리 보존**을 위해 그대로 유지합니다.

---

## 프로젝트 구조

**프로젝트 루트**: `/home/khadas/humiro_fire_suppression`

```
humiro_fire_suppression/
├── application/                # 통합 애플리케이션 (메인 프로그램)
│   ├── main.cpp                # 진입점
│   ├── src/
│   │   ├── application_manager.cpp/h  # 미션 관리, 커스텀 메시지 처리
│   │   └── frame_compositor.cpp/h     # RGB+열화상 합성
│   ├── build.sh
│   └── CMakeLists.txt
│
├── navigation/                 # OFFBOARD 자율 비행
│   └── src/offboard/
│       ├── offboard_manager.cpp/h     # OFFBOARD 미션 제어 (핵심)
│       ├── mission_context.h          # 핸들러 공유 상태
│       ├── bridge/                    # FC Bridge IPC (UDP)
│       │   ├── fc_bridge_protocol.h         # IPC 메시지 구조체
│       │   ├── fc_bridge_client.cpp/h       # 메인앱 측 IPC 클라이언트
│       │   ├── fc_bridge_server.cpp/h       # fc_bridge 측 IPC 서버
│       │   └── fc_bridge_node.cpp           # fc_bridge 실행파일
│       ├── handlers/                  # 상태별 핸들러
│       │   ├── state_handler.h        # 베이스 인터페이스
│       │   ├── prepare_handler.h      # PREPARE (heartbeat)
│       │   ├── offboard_handler.h     # OFFBOARD 모드 전환
│       │   ├── arm_handler.h          # ARM (시동)
│       │   ├── takeoff_handler.h      # TAKEOFF (이륙)
│       │   ├── hover_handler.h        # HOVER (호버링)
│       │   ├── rotate_handler.h       # ROTATE (회전)
│       │   ├── navigate_handler.h     # NAVIGATE (이동)
│       │   ├── hover_at_target_handler.h  # 목표지점 호버링
│       │   └── rtl_handler.h          # RTL (귀환)
│       └── CMakeLists.txt
│
├── custom_message/             # MAVLink 커스텀 메시지 (60000-60003)
│   ├── include/custom_message/
│   │   ├── custom_message.h           # 메시지 수신/파싱
│   │   └── custom_message_type.h      # 메시지 타입 정의
│   ├── src/
│   │   └── custom_message.cpp
│   ├── examples/
│   │   └── test_message_sender.cpp    # 테스트 송신기
│   ├── test/
│   │   └── custom_message_sender_gui_v2.py  # GUI 테스트 도구
│   └── CMakeLists.txt
│
├── ros2/                       # ROS2 통신 모듈
│   └── src/status/
│       └── status_ros2_subscriber.cpp/h  # PX4 상태 구독 (vehicle_status_v1 등)
│
├── thermal/                    # 열화상 시스템
│   ├── src/
│   │   ├── main.cpp                   # 열화상 메인
│   │   ├── camera_manager/            # 카메라 I/O
│   │   ├── thermal_processor/         # 핫스팟 감지
│   │   └── frame_compositor/          # RGB+Thermal 정합
│   └── CMakeLists.txt
│
├── streaming/                  # 스트리밍 시스템
│   └── src/
│       ├── streaming_manager.cpp/h    # 스트리밍 관리
│       ├── http_server.cpp/h          # HTTP 스트리밍
│       └── rtsp_server.cpp/h          # RTSP 스트리밍
│
├── osd/                        # OSD 오버레이 시스템
│   └── src/
│       ├── lidar/
│       │   └── distance_overlay.cpp/h     # LiDAR 거리 오버레이
│       ├── thermal/
│       │   └── thermal_overlay.cpp/h      # 열화상 오버레이
│       ├── targeting/
│       │   ├── aim_indicator.cpp/h        # 조준 표시
│       │   └── hotspot_tracker.cpp/h      # 핫스팟 추적
│       ├── status/
│       │   └── status_overlay.cpp/h       # 상태 모니터링 OSD
│       └── mission/
│           └── mission_overlay.cpp/h      # 미션 오버레이
│
├── lidar/                      # LiDAR 시스템 (LD19)
│   └── src/
│       ├── lidar_interface.cpp/h      # LD19 UART 통신
│       ├── lidar_ros2_publisher.cpp/h # ROS2 발행
│       └── lidar_config.h             # 설정
│
├── gui/                        # 웹 GUI 관리 도구 (Flask)
│   ├── app.py                         # Flask 메인
│   ├── templates/                     # HTML 템플릿
│   │   ├── index.html                 # 대시보드
│   │   ├── router.html                # mavlink-router 관리
│   │   ├── camera.html                # 카메라 설정
│   │   ├── config.html                # 기기 설정
│   │   ├── flight_mode.html           # 비행 모드
│   │   ├── offboard_settings.html     # 오프보드 모드 설정
│   │   ├── mavlink_sender.html        # MAVLink 메시지 송신
│   │   ├── micro_ros.html             # MicroXRCE-DDS 관리
│   │   ├── params.html                # FC 파라미터
│   │   ├── terminal.html              # 웹 터미널
│   │   └── wifi.html                  # WiFi 관리
│   └── utils/
│       ├── config_manager.py          # 설정 관리
│       ├── mavlink_manager.py         # MAVLink 유틸
│       ├── wifi_manager.py            # WiFi 유틸
│       └── system_checker.py          # 시스템 체크
│
├── targeting/                  # 타겟팅 시스템 (구현 중)
│   └── src/
│       └── targeting_frame_compositor.cpp/h
│
├── throwing_mechanism/         # 발사 메커니즘 (미구현)
│
├── scripts/                    # 운영 스크립트
│   ├── install/                       # 설치 스크립트
│   │   ├── 000-install_all.sh
│   │   ├── 001-install_px4_ros2_complete.sh
│   │   ├── 002-install_mavlink_router.sh
│   │   └── 003-apply_config.sh
│   ├── runtime/                       # 런타임 스크립트
│   │   ├── humiro_fire_suppression_wrapper.sh
│   │   ├── start_fc_bridge.sh               # fc_bridge 실행 (Domain 0)
│   │   ├── service-control.sh
│   │   └── start_micro_ros_agent_wrapper.sh
│   ├── debug/                         # 디버그 스크립트
│   │   ├── check_px4_connection.sh
│   │   └── ros2_topic_echo_px4.sh
│   ├── connect_fc.sh                  # FC 연결
│   ├── connect_sitl.sh                # SITL 연결
│   └── start_sitl_simulation.sh       # SITL 시작
│
├── config/                     # 설정 파일
│   ├── device_config.env              # 기기별 설정 (드론ID, IP 등)
│   ├── custom_params.json             # 커스텀 파라미터
│   ├── offboard_config.json           # 오프보드 모드 설정 (목표지점 고도 등)
│   ├── fastdds_eth0_only.xml          # DDS 설정 (micro-ros-agent)
│   ├── fastdds_loopback_only.xml      # DDS 설정 (fc_bridge, Domain 0)
│   ├── fastdds_wifi_only.xml          # DDS 설정 (메인앱, Domain 1)
│   └── fc_params/                     # FC 파라미터 백업
│
├── deployment/                 # 배포 (systemd 서비스)
│   └── systemd/
│       └── fc-bridge.service          # fc_bridge systemd 서비스
│
├── humiro_msgs/                # ROS2 커스텀 메시지 정의
│   └── CMakeLists.txt
│
├── docs/                       # 기술 문서
│   ├── SITL_VIM4_CONNECTION_QUICKSTART.md  # SITL-VIM4 연결 가이드
│   ├── SIMULATION_SETUP_GUIDE.md          # 시뮬레이션 환경 구축
│   ├── OFFBOARD_MODE_SETUP.md             # OFFBOARD 모드 설정
│   ├── PX4_16_UXRCE_DDS_SETUP.md          # uXRCE-DDS 설정
│   ├── PX4_NAV_STATE_REFERENCE.md         # PX4 상태 참조
│   ├── PORT_RULES.md                      # 포트 규칙
│   ├── CUSTOM_MESSAGE_DEBUG_GUIDE.md      # 커스텀 메시지 디버그
│   ├── installation/                      # 설치 가이드 (HTML)
│   ├── setup/                             # 설정 가이드
│   └── technical/                         # 기술 문서
│
├── work-plan/                  # 프로젝트 계획 문서 (29개)
│
├── CLAUDE.md                   # 이 파일
└── CHANGELOG.md                # 변경 이력
```

---

## 시스템 아키텍처

### 데이터 흐름

```
                        MAVLink (UDP)              uXRCE-DDS (UDP)
  SITL PC / FC  ──────────────────────────────────────────────────> VIM4
  (PX4)               ↓                                ↓
              mavlink-router (18001)          MicroXRCE-DDS Agent (8888)
                    ↓                                ↓
          custom_message.cpp               ROS2 토픽 발행
          (60000-60003 파싱)               /fmu/out/vehicle_status_v1
                    ↓                     /fmu/out/vehicle_local_position
          application_manager.cpp         /fmu/out/vehicle_global_position
          (미션 관리)                              ↓
                    ↓                     status_ros2_subscriber.cpp
          offboard_manager.cpp            (FC 상태 구독)
          (OFFBOARD 미션 제어)
                    ↓
          TrajectorySetpoint / VehicleCommand
          (ROS2 → PX4)
```

### 미션 상태 머신

```
IDLE → 대기
  ↓
PREPARING → heartbeat 발행 (2초, OFFBOARD 준비)
  ↓
OFFBOARD → OFFBOARD 모드 전환 명령
  ↓
ARMING → ARM 시동
  ↓
TAKEOFF → 목표 고도 이륙
  ↓
HOVER → 호버링 안정화
  ↓
ROTATE → 목표 방향 회전 (yaw PD 제어)
  ↓
NAVIGATE → 목표 위치 이동 (velocity setpoint + 보간)
  ↓        ↑ (경로 변경 시 부드러운 선회)
HOVER_AT_TARGET → 목표지점 호버링 (5초, position setpoint)
  ↓
RTL → 자동 귀환/착륙
  ↓
LANDED → 미션 완료 → IDLE 리셋
```

**경로 변경 (v0.13.2+)**:
- NAVIGATE 중 새 커스텀 메시지(60000) 수신 시 목표만 업데이트
- 정지 없이 velocity 보간으로 부드러운 곡선 선회 (v0.13.3)

**목표지점 호버링 (v0.13.6+)**:
- NAVIGATE 도착 후 5초간 목표지점에서 호버링 후 RTL
- 목표지점 고도는 `config/offboard_config.json`에서 설정 (GUI 오프보드 설정 페이지)

### ROS2 토픽

**PX4 -> VIM4** (구독):
- `/fmu/out/vehicle_status_v1` - 비행 상태 (nav_state, arming_state)
- `/fmu/out/vehicle_local_position` - 로컬 NED 위치
- `/fmu/out/vehicle_global_position` - GPS 위치
- `/fmu/out/battery_status` - 배터리

**VIM4 -> PX4** (발행):
- `/fmu/in/offboard_control_mode` - OFFBOARD heartbeat
- `/fmu/in/trajectory_setpoint` - 위치/속도 목표
- `/fmu/in/vehicle_command` - 명령 (ARM, TAKEOFF, RTL 등)

**VIM4 센서** (발행):
- `/lidar/front_distance` - 전방 거리
- `/lidar/points` - LiDAR 포인트
- `/thermal/hotspot` - 핫스팟 정보
- `/thermal/max_temperature` - 최대 온도
- `/offboard/status` - OFFBOARD 상태

### MAVLink 커스텀 메시지

| MSG_ID | 이름 | 방향 | 설명 |
|--------|------|------|------|
| 60000 | FIRE_MISSION_START | GCS->VIM4 | 미션 시작 (좌표, 고도, 속도) |
| 60001 | FIRE_MISSION_STATUS | VIM4->GCS | 미션 상태 보고 |
| 60002 | FIRE_MISSION_STOP | GCS->VIM4 | 미션 중지 |
| 60003 | FIRE_MISSION_RTL | GCS->VIM4 | 긴급 RTL |

- 60000은 **중복 허용** (비행 중 경로 변경)
- 60001-60003은 중복 차단

---

## 주요 클래스 및 API

### OffboardManager (`navigation/src/offboard/offboard_manager.h`)

```cpp
enum class MissionState {
    IDLE, PREPARING, OFFBOARD, ARMING,
    TAKEOFF, HOVER, ROTATE, NAVIGATE,
    HOVER_AT_TARGET, RTL, LANDED, ERROR
};

struct MissionConfig {
    float takeoff_altitude = 5.0f;
    float target_altitude = -1.0f;       // 목표지점 고도 (-1이면 takeoff_altitude 사용)
    float flight_speed = 5.0f;
    GPSCoordinate target_waypoint;
    float hover_duration_sec = 3.0f;
};

class OffboardManager {
public:
    bool executeMissionSolo(const MissionConfig& config);     // 단독 미션
    bool executeMissionFormation(const MissionConfig& config); // 편대 미션
    bool updateMissionTarget(const GPSCoordinate& new_target);  // 부드러운 경로 변경
    bool isMissionRunning() const;
    void abortMission();
    void emergencyRTL();
    MissionState getCurrentState() const;
    void resetToIdle();
};
```

**핵심 동작**:
- 10Hz 타이머로 heartbeat + setpoint 발행
- NAVIGATE 상태: velocity setpoint + low-pass filter (alpha=0.08)
- HOVER_AT_TARGET 상태: 목표지점에서 5초 호버링 후 RTL (position setpoint)
- GPS -> 로컬 NED 좌표 변환
- Yaw PD 제어 (K_P=1.5, K_D=0.9)

### ApplicationManager (`application/src/application_manager.h`)

```cpp
class ApplicationManager {
public:
    void initialize();
    void run();                        // 메인 루프
    void handleMissionStart(const FireMissionStart& msg);  // 미션 시작/경로 변경
    void handleMissionStop();
    void handleMissionRTL();
};
```

### CustomMessage (`custom_message/include/custom_message/custom_message.h`)

```cpp
class CustomMessage {
public:
    bool initialize(int port = 15001);
    void setCallback(std::function<void(const ParsedMessage&)> callback);
    void spin();  // 메시지 수신 루프
};
```

### StatusROS2Subscriber (`ros2/src/status/status_ros2_subscriber.h`)

```cpp
class StatusROS2Subscriber {
public:
    void spin();
    bool isFCConnected() const;        // 3초 이내 VehicleStatus 수신 여부
    uint8_t getNavState() const;
    uint8_t getArmingState() const;
};
```

---

## 빌드 방법

**프로젝트 경로**: `/home/khadas/humiro_fire_suppression`

### Application (통합 - 주로 사용)
```bash
cd /home/khadas/humiro_fire_suppression/application/build
cmake .. -DENABLE_ROS2=ON
make -j4
# 출력: ./humiro_fire_suppression
```

### Thermal 시스템
```bash
cd /home/khadas/humiro_fire_suppression/thermal/src
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 출력: ./thermal_rgb_streaming
```

### LiDAR 시스템
```bash
cd /home/khadas/humiro_fire_suppression/lidar/src
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

---

## SITL 시뮬레이션 연결

상세 가이드: `docs/SITL_VIM4_CONNECTION_QUICKSTART.md`

### 빠른 시작 (SITL PC의 PX4 콘솔에서)
```bash
# 파라미터 설정
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0

# MAVLink 연결 (VIM4로)
mavlink stop-all
mavlink start -u 14540 -o 18001 -t 192.168.100.11 -r 4000000

# uXRCE-DDS 연결 (ROS2 토픽)
uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888
```

### VIM4에서
```bash
sudo systemctl restart mavlink-router
cd /home/khadas/humiro_fire_suppression/application/build
./humiro_fire_suppression
```

---

## 개발 규칙

### 언어 선택
- **실시간 처리**: C++ 필수
- **하드웨어 인터페이스**: C++ 필수
- **성능 중요**: C++ 필수
- **ROS2 래퍼**: Python 허용 (C++ 호출만)
- **웹 GUI**: Python (Flask)

### 경로 관리
- 절대 경로 하드코딩 금지
- 환경 변수 사용 (`device_config.env`)
- **프로젝트 루트**: `/home/khadas/humiro_fire_suppression`

### 네트워크 구성

| 용도 | 인터페이스 | 대역 |
|------|-----------|------|
| FC 통신 | eth0 | 10.0.0.x |
| SITL/WiFi | wlan0 | 192.168.100.x |
| ROS2 DDS | localhost | 127.0.0.1 |

| 서비스 | 포트 |
|--------|------|
| mavlink-router FC | 14540 |
| mavlink-router SITL | 18001 |
| mavlink-router GCS | 14550 |
| Application (커스텀 메시지) | 15001 |
| External (테스트) | 16001 |
| MicroXRCE-DDS Agent | 8888 |
| 웹 GUI | 5000 |

---

## 참고 문서

**프로젝트 계획**:
- `work-plan/000_PROJECT_PROGRESS_REPORT.md` - 진행률 보고서
- `work-plan/001_PROJECT_MASTER_PLAN.md` - 마스터 플랜
- `work-plan/013_NEXT_STEPS_FORMATION_CONTROL.md` - 편대 제어 계획

**기술 문서**:
- `docs/SITL_VIM4_CONNECTION_QUICKSTART.md` - SITL-VIM4 연결
- `docs/OFFBOARD_MODE_SETUP.md` - OFFBOARD 모드
- `docs/PX4_NAV_STATE_REFERENCE.md` - PX4 상태
- `docs/PORT_RULES.md` - 포트 규칙
- `docs/CUSTOM_MESSAGE_DEBUG_GUIDE.md` - 커스텀 메시지 디버그
