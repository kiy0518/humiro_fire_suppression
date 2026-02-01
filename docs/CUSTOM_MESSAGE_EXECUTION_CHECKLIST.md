# Custom Message Execution Checklist

커스텀 메시지(FIRE_MISSION_START) 수신부터 미션 실행까지의 전체 플로우 점검 체크리스트.

---

## 메시지 흐름 개요

```
QGC (UDP:15001) → CustomMessage::receiveLoop()
  → parseMAVLinkMessage() → parseFireMissionStart()
  → callback → ApplicationManager::executeMission(start)
  → OffboardManager::executeMission(config)
  → ARMING → TAKEOFF → NAVIGATE → ADJUST_DISTANCE → HOVER → RTL
```

---

## A. 구조체/설정 정합성

| # | 항목 | 상태 | 파일 | 설명 |
|---|------|------|------|------|
| A-1 | `MissionConfig`에 `takeoff_speed` 필드 추가 | [x] | `offboard_manager.h:37` | 완료 |
| A-2 | `MissionConfig`에 `flight_speed` 필드 추가 | [x] | `offboard_manager.h:38` | 완료 |
| A-3 | `MissionConfig`에 `max_projectiles` 필드 추가 | [x] | `offboard_manager.h:44` | 완료 |
| A-4 | `MissionConfig`에 `auto_fire` 필드 추가 | [x] | `offboard_manager.h:43` | 완료 |
| A-5 | `executeMission()`에서 누락 필드 매핑 | [x] | `application_manager.cpp:1340-1347` | 완료 |
| A-6 | `offboard_manager.cpp` 로그에 속도 정보 출력 | [x] | `offboard_manager.cpp:56-65` | 완료 |
| A-7 | 하드코딩 값 제거 (`target_distance`, `tolerance`, `hover`) | [x] | `application_manager.cpp:1349-1354` | 환경변수 오버라이드 (MISSION_TARGET_DISTANCE 등) |

---

## B. 메시지 수신 검증

| # | 항목 | 상태 | 파일 | 설명 |
|---|------|------|------|------|
| B-1 | 위도 범위 검증 (-90 ~ 90) | [x] | `application_manager.cpp:1355` | 미션 거부 처리 |
| B-2 | 경도 범위 검증 (-180 ~ 180) | [x] | `application_manager.cpp:1359` | 미션 거부 처리 |
| B-3 | 고도 범위 검증 (0 ~ 120m) | [x] | `application_manager.cpp:1363` | 미션 거부 처리 |
| B-4 | 속도값 양수 검증 | [x] | `application_manager.cpp:1367-1373` | 경고 + 기본값 대체 |
| B-5 | `max_projectiles` 범위 검증 | [x] | `application_manager.cpp:1377` | 경고 + 기본값 대체 |

---

## C. 상태 관리 안정화

| # | 항목 | 상태 | 파일 | 설명 |
|---|------|------|------|------|
| C-1 | `mission_running_` 리셋을 `finishMission()` 통합 | [x] | `application_manager.cpp/h` | 11곳 → finishMission(true/false) 호출로 통합 |
| C-2 | 미션 실행 중 새 미션 수신 시 처리 정책 | [x] | `application_manager.cpp:1238-1267` | 기존 compare_exchange + FC 상태 체크 유지, finishMission 통합 |
| C-3 | `disableOffboardMode()` 멱등성 보장 | [x] | `offboard_manager.cpp:673` | isOffboardMode() 가드 추가 |
| C-4 | OFFBOARD 모드 활성화 타임아웃 | [x] | `arm_handler.cpp:171-232` | 기존 3초 고정 타임아웃 내장 확인 |
| C-5 | ERROR → IDLE 전환 시 정리 로직 검증 | [x] | `offboard_manager.cpp:659-669` | resetToIdle() → disableOffboardMode() + IDLE 전환, 멱등성 보장 |

---

## D. OSD 메시지 개선

| # | 항목 | 상태 | 파일 | 설명 |
|---|------|------|------|------|
| D-1 | "Mission Start" 메시지에 속도/격발 정보 포함 | [x] | `application_manager.cpp:351-354` | Alt, TkSpd, FltSpd, AUTO/MAN, 발사수 표시 |
| D-2 | 미션 성공/실패 시 OSD 표시 | [x] | `application_manager.cpp:1396-1403` | "Mission Complete" / "Mission FAILED" |

---

## E. 통신 안정성

| # | 항목 | 상태 | 파일 | 설명 |
|---|------|------|------|------|
| E-1 | 메시지 큐 오버플로우 대응 | [x] | `custom_message.cpp:497-509` | front drop 방식 + queue_drop_count 전용 카운터 |
| E-2 | UDP 소켓 에러 시 재시도 로직 | [x] | `custom_message.cpp:97-120` | 소켓 생성 최대 3회 재시도 (1초 간격) |
| E-3 | 중복 메시지 수신 방지 | [x] | `custom_message.cpp:643-658` | sysid별 seq 추적, 커스텀 메시지(60000~60003) 대상 |
| E-4 | 듀얼 포트 (15001/16001) 충돌 검증 | [x] | `application_manager.cpp:1240` | compare_exchange_strong으로 중복 실행 방지 확인 |

---

## F. 실행 전 사전 조건

| # | 항목 | 상태 | 파일 | 설명 |
|---|------|------|------|------|
| F-1 | FC 연결 상태 확인 | [x] | `application_manager.cpp:1238`, `status_ros2_subscriber.h/cpp` | isFCConnected() - 3초 이내 VehicleStatus 수신 확인 |
| F-2 | GPS fix 상태 확인 (fix_type >= 3) | [x] | `application_manager.cpp:1244`, `status_ros2_subscriber.h/cpp` | isGPSFixed() - fix_type >= 3 (3D Fix) |
| F-3 | 배터리 잔량 확인 | [x] | `application_manager.cpp:1250` | 20% 미만 시 미션 거부 + OSD 메시지 |
| F-4 | `OffboardManager` 상태가 IDLE인지 확인 | [x] | `application_manager.cpp:1281-1304` | 기존 상태 판단 로직으로 확인 (IDLE/ERROR/기타) |
| F-5 | 이전 미션 정리 완료 확인 | [x] | `application_manager.cpp:1240-1268` | compare_exchange_strong + FC OFFBOARD 상태 체크로 확인 |

---

## G. DO_SET_MODE 명령 최적화

| # | 항목 | 상태 | 파일 | 설명 |
|---|------|------|------|------|
| G-1 | 명령 10회 반복 전송 → 3회로 축소 | [x] | `application_manager.cpp:562-584` | 3회 x 100ms = 300ms (기존 10회 x 150ms = 1.5초) |
| G-2 | DO_SET_MODE / SET_MODE 핸들러 통합 검토 | [x] | `application_manager.cpp:510-697` | 별개 MSG_ID 처리, 통합 불필요 (현재 구조 유지) |

---

## 진행 상태 요약

- **완료**: A-1~A-7, B-1~B-5, C-1~C-5, D-1~D-2, E-1~E-4, F-1~F-5, G-1~G-2 (총 30개)
- **미완료**: 없음 (전체 완료)
