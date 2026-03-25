037 RTL 착륙 모드 — RC(조종기) 의존성 문제 및 향후 개선

## 현재 상태 (v0.26.14+)

RTL 착륙 시퀀스:
```
NAVIGATE_HOME → DESCEND → SOFT_LAND → TOUCHDOWN_IDLE → GROUND_DISARM → COMPLETE
```

TOUCHDOWN_IDLE 페이즈에서 **POSCTL(Position) 모드**로 전환하여 조종기에 제어권을 이양한다.
조종사가 최종 착륙/시동 끄기를 수행하며, PX4 land_detected 감지 시 자동 DISARM으로 전환된다.

## 문제점

**POSCTL 모드는 RC(조종기) 연결이 필수**이다.

- `COM_RCL_EXCEPT=4`: OFFBOARD만 RC loss 예외. POSCTL은 RC 필요.
- RC 미연결 상태에서 POSCTL 전환 시 → PX4가 RC loss failsafe 발동 (설정에 따라 RTL/LAND/Terminate)
- 현재 운용은 **조종기가 항상 연결된 상태**를 전제로 함

### 검토한 대안과 기각 사유

| 대안 | 문제점 |
|------|--------|
| AUTO_LAND | PX4 최저 하강속도 0.6m/s → 45kg 기체 착지 충격이 큼 |
| AUTO_LOITER (Hold) | 지상에서 호버링 유지 → 자동 disarm 불가, 수동 개입 필요 |
| OFFBOARD 유지 | OFFBOARD setpoint 발행 중단 시 failsafe → 별도 로직 필요 |

## 향후 개선 방안 (RC 없는 환경 대응)

### 방안 1: PX4 펌웨어 수정 — AUTO_LAND 최저속도 파라미터화
- `MPC_LAND_SPEED` 파라미터가 최저 0.6m/s 제한 → 이 하한을 낮추거나 제거
- 0.1~0.2m/s 착륙이 가능해지면 AUTO_LAND를 안전하게 사용 가능
- **가장 근본적인 해결책**

### 방안 2: OFFBOARD 모드 유지 + 자체 착지 시퀀스
- SOFT_LAND 이후에도 OFFBOARD를 유지하면서 극저속 하강 + 자체 착지 감지
- 현재 GROUND_DISARM의 OFFBOARD 경로와 유사하나, TOUCHDOWN_IDLE 없이 직접 전환
- RC 불필요, 단 OFFBOARD setpoint 발행을 계속해야 함

### 방안 3: AUTO_LOITER + 타이머 기반 DISARM
- Hold 모드 전환 후 일정 시간 경과 시 FORCE DISARM 명령
- 착지 여부와 무관하게 강제 시동 끄기 → 안전성 검증 필요

## 관련 파일

- `navigation/src/offboard/handlers/rtl_handler.h` — TOUCHDOWN_IDLE 페이즈 (POSCTL 전환)
- `navigation/src/offboard/offboard_manager.cpp` — LANDED 전환 시 Hold 모드 명령

## 우선순위

중간 — 현재 운용은 조종기 연결 전제이므로 즉시 문제 없음.
RC 없는 자율 운용(무인 편대 등) 계획 시 반드시 해결 필요.
