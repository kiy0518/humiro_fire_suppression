# v0.11.7 변경사항 요약

**날짜**: 2026-01-27
**상태**: 테스트 필요 (testMission3 이상동작 수정 진행중)

---

## 핵심 수정: testMission3 Yaw/Heading 드리프트 문제 해결

### 1. Position/Velocity Setpoint 충돌 해결
**파일**: `offboard_manager.cpp`

- `updateAdjustDistance()`에서 `hover()` 호출 제거
- **이전**: `hover()` (Position) + `publishDistanceSetpoint()` (Velocity) 동시 발행 → PX4 혼란
- **수정**: Velocity setpoint만 발행하여 일관된 제어

### 2. Fixed Yaw 시스템 완성
**파일**: `distance_adjuster.cpp`, `distance_adjuster.h`

- `publishDistanceSetpoint()`: NED 속도 변환 시 `fixed_yaw_` 사용
- `publishVelocitySetpoint()`: yaw를 NaN 대신 고정값 설정
- `hover()`: 고정 yaw 사용
- **효과**: 전진 중에도 이륙 시 캡처된 헤딩 유지

### 3. 이륙 시 Fixed Yaw 전달
**파일**: `offboard_manager.cpp`

- 이륙 완료 시 `distance_adjuster_->setFixedYaw(takeoff_yaw)` 호출
- `updateTakeoff()`에서 매 틱마다 setpoint 발행 (OFFBOARD 신호 유지)

### 4. 착륙 시 Yaw 유지
**파일**: `rtl_handler.cpp`, `rtl_handler.h`

- `land()`, `sendLandCommandOnly()`: LAND 명령에 명시적 yaw 값 전달 (param4)
- `current_yaw_` 변수 추가하여 현재 헤딩 추적
- 착륙 중 회전 방지

### 5. GPS 유틸리티 리팩토링
**새 파일**: `gps_utils.h`

- `haversineDistance()` 공통 함수로 분리
- `rtl_handler.cpp`, `waypoint_handler.cpp` 에서 중복 코드 제거

---

## 수정된 파일 목록

| 파일 | 변경 내용 |
|------|----------|
| `offboard_manager.cpp` | Position/Velocity 충돌 해결, Fixed Yaw 전달 |
| `distance_adjuster.cpp` | Fixed Yaw 시스템 적용 (hover, velocity, NED 변환) |
| `distance_adjuster.h` | Fixed Yaw 관련 메서드/변수 추가 |
| `rtl_handler.cpp` | LAND 명령에 yaw 파라미터 추가, haversine 분리 |
| `rtl_handler.h` | current_yaw_ 변수 추가 |
| `takeoff_handler.cpp` | 헤딩 유지 로직 |
| `takeoff_handler.h` | getTakeoffStartYaw() 메서드 |
| `waypoint_handler.cpp` | haversine 함수 gps_utils.h로 분리 |
| `waypoint_handler.h` | 중복 함수 제거 |
| `outdoor_mission_manager.cpp` | 리팩토링 |
| `gps_utils.h` | 새 파일 - 공통 GPS 유틸리티 |

---

## 알려진 이슈

- testMission3 이상동작 수정 진행중 (추가 테스트 필요)
- 2호기 동시 사용 시 ROS_DOMAIN_ID 충돌 문제 (UXRCE_DDS_NS 파라미터 필요하나 PX4 1.15.0에서 미지원)

---

## 다음 단계

1. 실제 기체에서 testMission3 테스트
2. Yaw 드리프트 현상 확인
3. 필요시 추가 수정
