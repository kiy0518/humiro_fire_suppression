# 034. 오프보드 모드 좌표계 혼동 검토

**작성일**: 2026-03-19
**상태**: 검토 완료 — 수정 불필요
**검토 방식**: 5개 에이전트 병렬 코드 검토

---

## 배경

오프보드 모드에서 상대 좌표(현재 위치 기준 delta)와 절대 좌표(홈 기준 NED)가 혼동되어 사용된 곳이 없는지 전수 검토.

**PX4 NED 좌표 체계:**
- **절대 좌표**: 홈 포지션 기준 NED (local_x, local_y, local_z). 시동 시 홈 = (0,0,0)
- **상대 좌표**: 현재 위치 기반 오프셋 (target - current = delta)
- **GPS 좌표**: lat/lon → equirectangular 변환 → NED
- NED Z축: 음수가 위 (5m 상공 = local_z = -5.0)

---

## 검토 범위

| # | 에이전트 | 검토 대상 |
|---|---------|----------|
| 1 | offboard_manager | offboard_manager.cpp/h — GPS→NED 변환, setpoint 계산 |
| 2 | navigate_handler | navigate_handler.h — 이동 핸들러 velocity/position setpoint |
| 3 | rtl_handler | rtl_handler.h — 귀환 4페이즈 (NAVIGATE_HOME/DESCEND/SOFT_LAND/GROUND_DISARM) |
| 4 | 4개 핸들러 | takeoff/hover/rotate/hover_at_target — position/altitude 변환 |
| 5 | GPS변환+formation | gpsToLocalNED(), formation_controller, mission_context |

---

## 검토 결과: 심각한 좌표 혼동 버그 0건

### 핸들러별 Setpoint 검증

| 핸들러 | Setpoint Position | 좌표계 | 평가 |
|--------|------------------|--------|------|
| TAKEOFF | `{start_local_x, start_local_y, z_profile}` | 절대 NED | 올바름 |
| HOVER | `{hover_x, hover_y, hover_z}` | 절대 NED | 올바름 |
| ROTATE | `{hold_x, hold_y, hold_z}` + yawspeed | 절대 NED | 올바름 |
| NAVIGATE | `{profile.pos[0-2]}` (MotionProfile3D) | 절대 NED | 올바름 |
| HOVER_AT_TARGET | `{target_ned_x, target_ned_y, z_profile}` | 절대 NED | 올바름 |
| RTL NAVIGATE_HOME | `{nav_profile pos}` → home(0,0) | 절대 NED | 올바름 |
| RTL DESCEND | `{home_x, home_y, NAN}` + vz | 절대 XY, velocity Z | 올바름 |
| RTL SOFT_LAND | `{home_x, home_y, NAN}` + calcLandingSpeed | 절대 XY, velocity Z | 올바름 |
| DISTANCE_ADJUST | 모든 페이즈 절대 좌표 | 절대 NED | 올바름 |

### 고도 변환 패턴 (모든 핸들러 일관)

```
목표 NED Z = start_local_z - altitude
예: start_local_z=0, altitude=5m → target_z = -5.0 (NED 위쪽)
```

모든 핸들러가 동일한 패턴 사용 → 일관성 확인됨.

### AGL(Above Ground Level) 계산 (RTL handler)

```cpp
float getAGL() {
    // 1순위: MAVLink DISTANCE_SENSOR (설치높이 보정)
    // 2순위: EKF dist_bottom (설치높이 보정)
    // 3순위: 기압계 -(current_z - home_z)
}
```

3가지 소스 모두 상대 고도를 올바르게 반환. 기압계 드리프트는 baro_agl_stable_count로 완화.

---

## GPS→NED 변환 방식 비일관성 (버그 아님, 개선 권장)

코드에 두 가지 GPS→NED 변환 방식이 혼재:

**방식 A — gpsToLocalNED() 함수 (홈 기준):**
```cpp
local_x = (target_lat - home_lat_) * DEG_TO_M_LAT;
local_y = (target_lon - home_lon_) * deg_to_m_lon;
```

**방식 B — updateMissionTarget() 인라인 (현재 위치 기준):**
```cpp
offset_north = (new_target.latitude - cur_lat) * DEG_TO_M_LAT;
target_ned_x_ = ref_local_x + offset_north;
```

**수학적 동치 증명:**
- `ref_local_x`와 `cur_lat`은 모두 PX4 EKF 출력 (VehicleLocalPosition.x, VehicleGlobalPosition.lat)
- EKF 내부에서 local↔global 일관성 보장
- `ref_local_x + (target_GPS - cur_GPS)` ≈ `(target_GPS - home_GPS)` (동일 결과)
- 연속 호출 시에도 각 호출이 독립적이므로 오차 누적 없음

**개선 권장 (우선순위 낮음):**
1. `gpsToLocalNED()` 함수로 통일하면 가독성 향상
2. `gpsToLocalNED()`의 고도 파라미터가 AMSL인지 AGL인지 명확히 문서화

---

## 결론

오프보드 모드 전체 좌표계 사용이 올바르며, 수정이 필요한 버그는 없음.
GPS→NED 변환 방식이 두 가지 혼재하나, 수학적으로 동치이므로 기능상 문제 없음.
