# 커스텀 메시지 OSD 표시 디버그 가이드

## 개요

외부 프로그램(Python GUI)에서 커스텀 메시지를 전송하고 VIM4의 OSD에 표시되는 전체 흐름을 단계별로 점검하는 가이드입니다.

---

## 전체 흐름

```
[STEP 0] Python GUI 프로그램
    ↓ UDP 전송 (포트 15000)
[STEP 1] CustomMessage::receiveLoop() - UDP 수신
    ↓ 메시지 버퍼
[STEP 2] CustomMessage::parseMAVLinkMessage() - MAVLink 파싱
    ↓ 메시지 타입 확인
[STEP 3] CustomMessage::parseFireMissionStart() - 메시지 파싱
    ↓ 구조체 추출
[STEP 4] ApplicationManager 콜백 함수 호출
    ↓ status_overlay_->setCustomMessage()
[STEP 5] StatusOverlay::setCustomMessage() - 메시지 설정
    ↓ show_custom_message_ = true
[STEP 6] StatusOverlay::draw() - OSD 렌더링
    ↓ 화면에 표시
[완료] OSD 화면에 메시지 표시
```

---

## 단계별 점검 방법

### STEP 0: Python GUI 프로그램 전송 확인

**확인 사항:**
- Python 프로그램이 실행 중인지 확인
- 대상 IP가 올바른지 확인 (기본값: 192.168.100.11)
- 포트가 15000인지 확인
- "전송" 버튼 클릭 후 로그에 "✓ 전송" 메시지 확인

**로그 예시:**
```
[14:47:15] ✓ FIRE_MISSION_START 전송: 37.1234567°, 127.1234567°, 10.0m, Auto=1
```

**문제 해결:**
- 네트워크 연결 확인: `ping 192.168.100.11`
- 방화벽 확인: `sudo ufw status`

---

### STEP 1: UDP 수신 확인

**확인 위치:** VIM4 로그 파일
```bash
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 1"
```

**정상 출력:**
```
[CustomMessage] [STEP 1] UDP 수신: 20 bytes, 첫 바이트: 0xFD, 포트: 15000
```

**문제 상황:**
- 출력이 없음 → 메시지가 수신되지 않음
  - 네트워크 연결 확인
  - 포트 15000이 다른 프로세스에 의해 사용 중인지 확인: `sudo netstat -tulpn | grep 15000`
  - 프로그램이 실행 중인지 확인: `ps aux | grep humiro_fire_suppression`

- 첫 바이트가 0xFE → MAVLink v1 사용 중 (Python 프로그램을 v2로 수정 필요)
- 첫 바이트가 0xFD가 아님 → 잘못된 메시지 포맷

---

### STEP 2: MAVLink 파싱 확인

**확인 위치:** VIM4 로그 파일
```bash
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 2"
```

**정상 출력:**
```
[CustomMessage] [STEP 2] 메시지 파싱: MSG_ID=12900, payload_len=14
[CustomMessage] [STEP 2] ✓ CRC 검증 통과
```

**문제 상황:**
- `CRC 불일치` → Python 프로그램의 CRC 계산 오류
  - `custom_message_sender_gui_v2.py` 사용 확인
  - CRC 계산 함수 확인

- `알 수 없는 메시지 ID` → 메시지 ID가 잘못됨
  - FIRE_MISSION_START: 12900
  - FIRE_LAUNCH_CONTROL: 12902

---

### STEP 3: 메시지 타입별 파싱 확인

**확인 위치:** VIM4 로그 파일
```bash
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 3"
```

**정상 출력 (FIRE_MISSION_START):**
```
[CustomMessage] [STEP 3] FIRE_MISSION_START 파싱 완료: lat=37.1234567, lon=127.1234567, alt=10
```

**정상 출력 (FIRE_LAUNCH_CONTROL):**
```
[CustomMessage] [STEP 3] FIRE_LAUNCH_CONTROL 파싱 완료: command=0 (CONFIRM)
```

**문제 상황:**
- `페이로드 길이 부족` → Python 프로그램의 페이로드 구조 오류
  - 구조체 크기 확인 필요

---

### STEP 4: 콜백 함수 호출 확인

**확인 위치:** VIM4 로그 파일
```bash
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 4"
```

**정상 출력:**
```
[CustomMessage] [STEP 4] 콜백 함수 호출 시작
[TEST PORT] ========== FIRE_MISSION_START 수신 (15000) ==========
[TEST PORT] 시간: 2026-01-04 14:47:15.123
[TEST PORT] 목표 위치:
[TEST PORT]   - 위도: 37.1234567°
[TEST PORT]   - 경도: 127.1234567°
[TEST PORT]   - 고도: 10.00 m
[TEST PORT] Auto Fire: 예
[TEST PORT] ==========================================
[CustomMessage] [STEP 4] 콜백 함수 호출 완료
```

**문제 상황:**
- `⚠ 콜백 함수가 설정되지 않음!` → `test_message_handler_` 초기화 실패
  - 프로그램 시작 시 "[테스트 메시지 핸들러 초기화]" 메시지 확인
  - "✓ 테스트 메시지 송수신 시작 (포트 15000)" 메시지 확인

---

### STEP 5: StatusOverlay 메시지 설정 확인

**확인 위치:** VIM4 로그 파일
```bash
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 5"
```

**정상 출력:**
```
[StatusOverlay] [STEP 5] setCustomMessage 호출: "[TEST] Mission: (37.1234567, 127.1234567) 10m", timeout=5.0초
[StatusOverlay] [STEP 5] ✓ 커스텀 메시지 설정 완료 (OSD 표시 활성화)
```

**문제 상황:**
- 출력이 없음 → `status_overlay_`가 nullptr
  - `initializeComponents()`에서 `status_overlay_` 초기화 확인

---

### STEP 6: OSD 렌더링 확인

**확인 위치:** VIM4 로그 파일
```bash
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 6"
```

**정상 출력:**
```
[StatusOverlay] [STEP 6] OSD 렌더링: "[TEST] Mission: (37.1234567, 127.1234567) 10m" (프레임 #1)
```

**문제 상황:**
- 출력이 없음 → `show_custom_message_`가 false이거나 `custom_message_`가 비어있음
  - STEP 5 출력 확인
  - 타임아웃 확인

- 타임아웃 메시지 → 메시지 표시 시간이 지나서 자동으로 사라짐 (정상)

---

## 전체 점검 스크립트

```bash
#!/bin/bash
# 전체 흐름 점검 스크립트

echo "=========================================="
echo "커스텀 메시지 OSD 표시 전체 흐름 점검"
echo "=========================================="

LOG_FILE="/home/khadas/humiro_fire_suppression/logs/humiro-service.log"

echo ""
echo "[STEP 0] Python GUI 프로그램 확인"
echo "  → Python 프로그램에서 메시지 전송 후 확인"

echo ""
echo "[STEP 1] UDP 수신 확인:"
tail -20 "$LOG_FILE" | grep "STEP 1" | tail -5 || echo "  → STEP 1 로그 없음"

echo ""
echo "[STEP 2] MAVLink 파싱 확인:"
tail -20 "$LOG_FILE" | grep "STEP 2" | tail -5 || echo "  → STEP 2 로그 없음"

echo ""
echo "[STEP 3] 메시지 파싱 확인:"
tail -20 "$LOG_FILE" | grep "STEP 3" | tail -5 || echo "  → STEP 3 로그 없음"

echo ""
echo "[STEP 4] 콜백 호출 확인:"
tail -30 "$LOG_FILE" | grep -E "STEP 4|TEST PORT" | tail -10 || echo "  → STEP 4 로그 없음"

echo ""
echo "[STEP 5] StatusOverlay 설정 확인:"
tail -20 "$LOG_FILE" | grep "STEP 5" | tail -5 || echo "  → STEP 5 로그 없음"

echo ""
echo "[STEP 6] OSD 렌더링 확인:"
tail -20 "$LOG_FILE" | grep "STEP 6" | tail -5 || echo "  → STEP 6 로그 없음"

echo ""
echo "=========================================="
echo "실시간 모니터링:"
echo "tail -f $LOG_FILE | grep -E 'STEP [0-9]|TEST PORT'"
echo "=========================================="
```

---

## 빠른 점검 명령어

### 실시간 로그 모니터링
```bash
# 모든 단계 확인
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep -E "STEP [0-9]|TEST PORT"

# 특정 단계만 확인
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 1"  # UDP 수신
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 2"  # 파싱
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 3"  # 메시지 파싱
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 4"  # 콜백
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 5"  # OSD 설정
tail -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep "STEP 6"  # 렌더링
```

### 포트 확인
```bash
# 포트 15000 사용 확인
sudo netstat -tulpn | grep 15000

# UDP 포트 리스닝 확인
sudo ss -ulpn | grep 15000
```

### 프로그램 실행 확인
```bash
# 프로세스 확인
ps aux | grep humiro_fire_suppression | grep -v grep

# 서비스 상태 확인
systemctl status humiro-fire-suppression
```

---

## 일반적인 문제 해결

### 문제 1: STEP 1 로그가 없음 (UDP 수신 안 됨)

**원인:**
- 네트워크 연결 문제
- 포트 충돌
- 프로그램이 실행되지 않음

**해결:**
```bash
# 네트워크 확인
ping 192.168.100.11

# 포트 확인
sudo netstat -tulpn | grep 15000

# 프로그램 재시작
./scripts/runtime/service-control.sh restart
```

### 문제 2: STEP 2에서 CRC 불일치

**원인:**
- Python 프로그램이 MAVLink v1 사용
- CRC 계산 오류

**해결:**
- `custom_message_sender_gui_v2.py` 사용 확인
- Python 프로그램 재시작

### 문제 3: STEP 4에서 콜백이 호출되지 않음

**원인:**
- `test_message_handler_` 초기화 실패
- 콜백 함수가 설정되지 않음

**해결:**
- 프로그램 시작 로그에서 "[테스트 메시지 핸들러 초기화]" 확인
- 프로그램 재빌드 및 재시작

### 문제 4: STEP 5 로그는 있지만 OSD에 표시 안 됨

**원인:**
- `status_overlay_`가 nullptr
- OSD 렌더링이 호출되지 않음
- 타임아웃으로 인해 즉시 사라짐

**해결:**
- STEP 6 로그 확인
- 타임아웃 시간 확인 (기본값: 5초)
- RTSP 스트림 확인

---

## 참고

- **프로젝트 경로**: `/home/khadas/humiro_fire_suppression`
- **로그 파일**: `/home/khadas/humiro_fire_suppression/logs/humiro-service.log`
- **에러 로그**: `/home/khadas/humiro_fire_suppression/logs/humiro-service-error.log`
- **테스트 포트**: 15000 (OSD 표시만, 미션 실행 안 함)
- **실제 미션 포트**: 14550 (QGC 표준 포트)

