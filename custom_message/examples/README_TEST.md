# 커스텀 메시지 테스트 시스템

## 개요

OSD 커스텀 메시지 표시를 테스트하기 위한 듀얼 포트 시스템입니다.

## 포트 구성

| 포트 | 용도 | 설명 |
|------|------|------|
| **14550** | QGC 연결 (운용) | QGroundControl과의 실제 통신 포트 |
| **15000** | 테스트 전용 | OSD 메시지 테스트용 (미션 실행 없음) |

## 특징

### 14550 포트 (메인 핸들러)
- QGroundControl과 통신
- 실제 미션 실행
- FIRE_MISSION_START 수신 시 자율 비행 시작
- 운용 환경에서 사용

### 15000 포트 (테스트 핸들러)
- 로컬 테스트 전용
- **미션 실행 안 함** (OSD 표시만)
- 콘솔 메시지에 `[TEST PORT]` 접두사 표시
- 안전한 시뮬레이션 환경

## 사용 방법

### 1. 메인 프로그램 실행

```bash
cd /home/khadas/humiro_fire_suppression/application/build
./humiro_fire_suppression
```

프로그램이 시작되면 두 개의 메시지 핸들러가 초기화됩니다:
```
[커스텀 메시지 초기화]
  ✓ 메시지 송수신 시작 (포트 14550)

[테스트 메시지 핸들러 초기화]
  ✓ 테스트 메시지 송수신 시작 (포트 15000)
```

### 2. 테스트 메시지 전송

별도 터미널에서 테스트 프로그램 실행:

```bash
cd /home/khadas/humiro_fire_suppression/custom_message/examples
./test_message_sender
```

### 3. 테스트 메뉴

```
===========================================
  테스트 메시지 송신 프로그램 (15000 포트)
===========================================
1. FIRE_MISSION_START 전송
2. FIRE_LAUNCH_CONTROL (CONFIRM) 전송
3. FIRE_LAUNCH_CONTROL (ABORT) 전송
4. FIRE_LAUNCH_CONTROL (REQUEST_STATUS) 전송
5. FIRE_MISSION_STATUS 전송
6. FIRE_SUPPRESSION_RESULT 전송
0. 종료
===========================================
```

## 테스트 예시

### FIRE_MISSION_START 테스트

1. 테스트 프로그램에서 `1` 선택
2. 메인 프로그램 콘솔에 다음과 같이 출력:

```
[TEST PORT] ========== FIRE_MISSION_START 수신 (15000) ==========
[TEST PORT] 시간: 2026-01-04 15:02:35.123
[TEST PORT] 목표 위치:
[TEST PORT]   - 위도: 37.1234567°
[TEST PORT]   - 경도: 127.1234567°
[TEST PORT]   - 고도: 10.00 m
[TEST PORT] Auto Fire: 예
[TEST PORT] ==========================================
```

3. OSD 화면에 메시지 표시: `[TEST] Mission: (37.1234567, 127.1234567) 10m`
4. **드론은 이륙하지 않음** (테스트 포트는 미션 실행 안 함)

### FIRE_LAUNCH_CONTROL 테스트

1. 테스트 프로그램에서 `2`, `3`, 또는 `4` 선택
2. 메인 프로그램 콘솔에 다음과 같이 출력:

```
[TEST PORT] FIRE_LAUNCH_CONTROL 수신 (15000): CONFIRM
```

3. OSD 화면에 메시지 표시: `[TEST] CONFIRM`

## 실제 운용 vs 테스트

### 실제 운용 (QGC → 14550 포트)

```
QGroundControl (지상국)
         ↓ (FIRE_MISSION_START)
    포트 14550 (메인 핸들러)
         ↓
    ✓ OSD 메시지 표시
    ✓ 자율 비행 시작 (Arming → Takeoff → Navigate)
    ✓ 미션 실행
```

### 테스트 (test_message_sender → 15000 포트)

```
test_message_sender (로컬)
         ↓ (FIRE_MISSION_START)
    포트 15000 (테스트 핸들러)
         ↓
    ✓ OSD 메시지 표시 (with [TEST] prefix)
    ✗ 미션 실행 안 함 (안전)
```

## 안전 기능

1. **미션 실행 분리**: 테스트 포트는 OSD만 표시하고 미션을 실행하지 않음
2. **포트 분리**: 14550(QGC)과 15000(테스트)은 완전히 독립적
3. **명확한 표시**: 테스트 메시지는 `[TEST PORT]` 접두사로 구분
4. **충돌 방지**: 테스트 송신기는 15001번 포트로 수신 (실제로는 사용 안 함)

## 빌드 방법

### 테스트 프로그램만 재빌드

```bash
cd /home/khadas/humiro_fire_suppression/custom_message/examples
bash build_test_sender.sh
```

### 메인 프로그램 재빌드

```bash
cd /home/khadas/humiro_fire_suppression/application/build
make -j$(nproc)
```

## 파일 구조

```
humiro_fire_suppression/
├── application/
│   └── src/
│       ├── application_manager.h     # 듀얼 핸들러 선언
│       └── application_manager.cpp   # 듀얼 핸들러 구현
└── custom_message/
    └── examples/
        ├── test_message_sender.cpp   # 테스트 송신 프로그램
        ├── build_test_sender.sh      # 빌드 스크립트
        └── README_TEST.md            # 이 문서
```

## 코드 수정 포인트

### ApplicationManager::initializeCustomMessage()

```cpp
// 메인 핸들러 (14550 포트 - QGC)
custom_message_handler_ = new custom_message::CustomMessage(
    14550, 14550, "0.0.0.0", target_address, 1, 1
);

// 테스트 핸들러 (15000 포트 - 테스트)
test_message_handler_ = new custom_message::CustomMessage(
    15000, 15000, "0.0.0.0", target_address, 1, 1
);

// 테스트 핸들러는 미션을 실행하지 않음
test_message_handler_->setFireMissionStartCallback(
    [this](const custom_message::FireMissionStart& start) {
        // OSD 메시지만 표시
        // executeMission(start);  // 주석 처리!
    }
);
```

## 트러블슈팅

### 테스트 메시지가 수신되지 않을 때

1. 메인 프로그램이 실행 중인지 확인
2. 포트 15000이 사용 중인지 확인:
   ```bash
   sudo netstat -tulpn | grep 15000
   ```
3. 방화벽 설정 확인 (필요시):
   ```bash
   sudo ufw allow 15000/udp
   ```

### OSD에 메시지가 표시되지 않을 때

1. `status_overlay_`가 초기화되었는지 확인
2. 메인 프로그램 콘솔에서 `[TEST PORT]` 메시지 확인
3. RTSP 스트림이 정상적으로 재생 중인지 확인

## 주의사항

⚠️ **중요**: 테스트 포트(15000)로 메시지를 전송해도 드론이 실제로 이륙하거나 미션을 수행하지 않습니다. 실제 비행 테스트는 반드시 QGC를 통해 14550 포트로 수행해야 합니다.

## 버전 정보

- 생성일: 2026-01-04
- 버전: v0.3.0
- 테스트 포트: 15000 (14551에서 변경)
