★ 편대 RTL 복귀 — Cross-Track 오프셋 유지 + 이륙지점 착륙

**작성일**: 2026-03-25
**대상 버전**: v0.26.15+
**우선순위**: HIGH (비행 안전)
**상태**: 설계 완료, 구현 대기

---

**1. 문제**

편대 비행 후 RTL 시, 각 기체가 자기 이륙 위치로 직선 귀환하면 경로 교차 → 충돌 위험.
현재 팔로워 RTL은 `offboard_mgr_->abortMission()` → RTL 핸들러가 `ctx.start_local_x/y`로
직선 복귀. 편대 경로선의 cross-track 오프셋(예: ±10m)이 RTL에서 완전 무시됨.

**요구사항**:
1. 복귀 중 충돌 방지 — 편대 경로선의 cross-track 오프셋(평행 레인) 유지
2. 이륙지점 착륙 — 안전한 바닥이 보장된 실제 이륙 위치에 정확히 착륙

---

**2. 해결: 2단계 NAVIGATE_HOME**

Phase A: 현재 위치 → 오프셋 홈 (평행 레인, 충돌 방지)
Phase B: 오프셋 홈 → 실제 이륙 위치 (안전 착륙)

```
[화점]
 F2     L     F3       현재 위치 (편대)
  |     |     |
  |     |     |        Phase A: 평행 레인 복귀
  |     |     |
  v     v     v
 F2'    L'    F3'      오프셋 홈 (홈 위도에서 cross-track 유지)
  \     |     /
   HOME HOME HOME      Phase B: 실제 이륙 위치에 착륙
```

---

**3. 수정 파일 (6개)**

- `navigation/src/offboard/mission_context.h` — 편대 RTL 필드 3개 추가
- `navigation/src/offboard/offboard_manager.h` — setFormationRtlOffset() 선언
- `navigation/src/offboard/offboard_manager.cpp` — setFormationRtlOffset() 구현
- `navigation/src/offboard/formation/formation_controller.h` — triggerFormationRTL() 선언
- `navigation/src/offboard/formation/formation_controller.cpp` — RTL 전환 시 오프셋 전달
- `navigation/src/offboard/handlers/rtl_handler.h` — 2단계 NAVIGATE_HOME 구현

---

**4. 상세 설계**

**4.1 MissionContext 필드 추가**

`=== 편대 ===` 섹션, `formation_mode` 아래:
```cpp
bool  formation_rtl_offset_valid{false};   // 편대 RTL 오프셋 유효 여부
float formation_rtl_heading_rad{0.0f};     // 고정 경로 heading (rad, NED)
float formation_rtl_cross_offset_m{0.0f};  // cross-track 오프셋 (m, 우측 양수)
```

`reset()`에 초기화 추가.

**4.2 OffboardManager setter**

```cpp
void setFormationRtlOffset(float heading_rad, float cross_offset_m);
```
ctx_에 3개 필드 설정 + 로그 출력.

**4.3 FormationController RTL 전환**

`triggerFormationRTL()` 헬퍼 메서드:
- `follower_phase_ = FollowerPhase::RTL`
- `fixed_heading_set_` 확인 후 `setFormationRtlOffset()` 호출
- `offset_right_cm_` → meter 변환, `lateral_mirrored_` 반영
- `abortMission()` 호출

호출 위치 (2곳):
- `onFormationCommand()` CMD_RTL 처리
- `onHeartbeat()` RTL phase 감지

**4.4 RTL 핸들러 2단계 NAVIGATE_HOME**

새 멤버: `actual_home_x_/y_`, `formation_rtl_`, `phase_a_complete_`

onEnter():
- `actual_home_x/y` = ARM 위치 (실제 이륙 지점)
- 편대 오프셋 유효 시: `home_x/y` = actual + cross-track 오프셋
- 오프셋 < 0.1m 이면 solo RTL 폴백

NAVIGATE_HOME tick:
- 도착 시 `formation_rtl_ && !phase_a_complete_` → Phase A 완료
  - `home_x/y` = `actual_home_x/y` (실제 이륙 위치)
  - MotionProfile 리셋
- Phase B 도착 → DESCEND

---

**5. NED 오프셋 수학**

경로 heading `theta`, cross-track `cross` (우측 양수):
- offset_N = -sin(theta) * cross
- offset_E =  cos(theta) * cross

검증 (formation_controller.cpp line 957-958 일치):
- heading=0 (정북), cross=+10m → (0, +10) 홈 동쪽 10m
- heading=pi/2 (정동), cross=+10m → (-10, 0) 홈 남쪽 10m (경로 우측)

---

**6. 안전 장치**

1. solo RTL 불변: `formation_rtl_offset_valid == false`이면 기존 동작 100% 유지
2. heading 미설정 폴백: `fixed_heading_set_ == false`이면 solo RTL
3. 작은 오프셋 무시: `|cross_offset| < 0.1m`이면 solo RTL
4. 리더는 항상 solo RTL: `setFormationRtlOffset()` 호출 없음
5. 타임아웃: Phase A+B 총 거리 반영

---

**7. 충돌 방지와의 관계**

편대 RTL(proactive) = 1차 방어선: 경로 분리로 충돌 가능성 원천 차단
충돌 방지(reactive) = 2차 방어선: 예상치 못한 근접 시 회피 (미구현, 025 설계 참조)

두 접근은 보완적 관계이며 대체 관계가 아님.

---

**8. 검증**

1. SITL 3기체 편대 → 순차 RTL:
   - 로그: `[RTL] 편대 복귀: Phase A → Phase B` 확인
   - OSD: `HOME:A` → `HOME:B` 전환 확인
   - 각 기체가 평행 레인 복귀 후 이륙 위치에 착륙
2. solo RTL 회귀: 단독 미션 → RTL 기존 동작 동일
3. 리더 RTL: 오프셋 없이 직선 복귀 + 이륙 위치 착륙
