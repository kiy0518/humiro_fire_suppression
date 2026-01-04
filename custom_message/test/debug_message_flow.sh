#!/bin/bash
# 커스텀 메시지 OSD 표시 전체 흐름 점검 스크립트

LOG_FILE="/home/khadas/humiro_fire_suppression/logs/humiro-service.log"
ERROR_LOG="/home/khadas/humiro_fire_suppression/logs/humiro-service-error.log"

echo "=========================================="
echo "커스텀 메시지 OSD 표시 전체 흐름 점검"
echo "=========================================="
echo ""

# STEP 0: Python GUI 프로그램 확인
echo "[STEP 0] Python GUI 프로그램 확인"
echo "  → Python 프로그램에서 메시지 전송 후 확인"
echo "  → 로그에 '✓ 전송' 메시지 확인"
echo ""

# STEP 1: UDP 수신 확인
echo "[STEP 1] UDP 수신 확인:"
if [ -f "$LOG_FILE" ]; then
    STEP1_COUNT=$(tail -50 "$LOG_FILE" | grep -c "STEP 1")
    if [ "$STEP1_COUNT" -gt 0 ]; then
        echo "  ✓ UDP 수신 로그 발견 ($STEP1_COUNT개)"
        tail -50 "$LOG_FILE" | grep "STEP 1" | tail -3 | sed 's/^/    /'
    else
        echo "  ✗ UDP 수신 로그 없음 (메시지가 수신되지 않음)"
    fi
else
    echo "  ⚠ 로그 파일 없음: $LOG_FILE"
fi
echo ""

# STEP 2: MAVLink 파싱 확인
echo "[STEP 2] MAVLink 파싱 확인:"
if [ -f "$LOG_FILE" ]; then
    STEP2_COUNT=$(tail -50 "$LOG_FILE" | grep -c "STEP 2")
    if [ "$STEP2_COUNT" -gt 0 ]; then
        echo "  ✓ 파싱 로그 발견"
        tail -50 "$LOG_FILE" | grep "STEP 2" | tail -3 | sed 's/^/    /'
        
        # CRC 오류 확인
        CRC_ERROR=$(tail -50 "$LOG_FILE" | grep -c "CRC 불일치")
        if [ "$CRC_ERROR" -gt 0 ]; then
            echo "  ⚠ CRC 오류 발견!"
            tail -50 "$LOG_FILE" | grep "CRC 불일치" | tail -2 | sed 's/^/    /'
        fi
    else
        echo "  ✗ 파싱 로그 없음"
    fi
else
    echo "  ⚠ 로그 파일 없음"
fi
echo ""

# STEP 3: 메시지 파싱 확인
echo "[STEP 3] 메시지 타입별 파싱 확인:"
if [ -f "$LOG_FILE" ]; then
    STEP3_COUNT=$(tail -50 "$LOG_FILE" | grep -c "STEP 3")
    if [ "$STEP3_COUNT" -gt 0 ]; then
        echo "  ✓ 메시지 파싱 로그 발견"
        tail -50 "$LOG_FILE" | grep "STEP 3" | tail -3 | sed 's/^/    /'
    else
        echo "  ✗ 메시지 파싱 로그 없음"
    fi
else
    echo "  ⚠ 로그 파일 없음"
fi
echo ""

# STEP 4: 콜백 호출 확인
echo "[STEP 4] 콜백 함수 호출 확인:"
if [ -f "$LOG_FILE" ]; then
    STEP4_COUNT=$(tail -50 "$LOG_FILE" | grep -c "STEP 4")
    TEST_PORT_COUNT=$(tail -50 "$LOG_FILE" | grep -c "TEST PORT")
    if [ "$STEP4_COUNT" -gt 0 ] || [ "$TEST_PORT_COUNT" -gt 0 ]; then
        echo "  ✓ 콜백 호출 로그 발견"
        tail -50 "$LOG_FILE" | grep -E "STEP 4|TEST PORT" | tail -5 | sed 's/^/    /'
    else
        echo "  ✗ 콜백 호출 로그 없음"
    fi
else
    echo "  ⚠ 로그 파일 없음"
fi
echo ""

# STEP 5: StatusOverlay 설정 확인
echo "[STEP 5] StatusOverlay 메시지 설정 확인:"
if [ -f "$LOG_FILE" ]; then
    STEP5_COUNT=$(tail -50 "$LOG_FILE" | grep -c "STEP 5")
    if [ "$STEP5_COUNT" -gt 0 ]; then
        echo "  ✓ OSD 설정 로그 발견"
        tail -50 "$LOG_FILE" | grep "STEP 5" | tail -3 | sed 's/^/    /'
    else
        echo "  ✗ OSD 설정 로그 없음"
    fi
else
    echo "  ⚠ 로그 파일 없음"
fi
echo ""

# STEP 6: OSD 렌더링 확인
echo "[STEP 6] OSD 렌더링 확인:"
if [ -f "$LOG_FILE" ]; then
    STEP6_COUNT=$(tail -50 "$LOG_FILE" | grep -c "STEP 6")
    if [ "$STEP6_COUNT" -gt 0 ]; then
        echo "  ✓ OSD 렌더링 로그 발견"
        tail -50 "$LOG_FILE" | grep "STEP 6" | tail -3 | sed 's/^/    /'
    else
        echo "  ✗ OSD 렌더링 로그 없음 (OSD가 그려지지 않음)"
    fi
else
    echo "  ⚠ 로그 파일 없음"
fi
echo ""

# 포트 확인
echo "[추가 확인] 포트 15000 상태:"
PORT_CHECK=$(sudo netstat -tulpn 2>/dev/null | grep 15000 || echo "")
if [ -n "$PORT_CHECK" ]; then
    echo "  ✓ 포트 15000 사용 중:"
    echo "$PORT_CHECK" | sed 's/^/    /'
else
    echo "  ✗ 포트 15000 사용 안 함"
fi
echo ""

# 프로그램 실행 확인
echo "[추가 확인] 프로그램 실행 상태:"
PROCESS_CHECK=$(ps aux | grep humiro_fire_suppression | grep -v grep || echo "")
if [ -n "$PROCESS_CHECK" ]; then
    echo "  ✓ 프로그램 실행 중:"
    echo "$PROCESS_CHECK" | sed 's/^/    /'
else
    echo "  ✗ 프로그램 실행 안 함"
fi
echo ""

echo "=========================================="
echo "실시간 모니터링 명령어:"
echo "  tail -f $LOG_FILE | grep -E 'STEP [0-9]|TEST PORT'"
echo "=========================================="

