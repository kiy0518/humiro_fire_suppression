#!/bin/bash
# 테스트 포트(15000) 수신 확인 스크립트

echo "=========================================="
echo "테스트 포트(15000) 수신 확인"
echo "=========================================="

# 1. 포트 사용 확인
echo ""
echo "[1] 포트 15000 사용 중인 프로세스 확인:"
sudo netstat -tulpn | grep 15000 || echo "  → 포트 15000을 사용 중인 프로세스 없음"

# 2. UDP 포트 리스닝 확인
echo ""
echo "[2] UDP 포트 15000 리스닝 확인:"
sudo ss -ulpn | grep 15000 || echo "  → 포트 15000 리스닝 안 함"

# 3. 프로그램 실행 확인
echo ""
echo "[3] humiro_fire_suppression 프로세스 확인:"
ps aux | grep humiro_fire_suppression | grep -v grep || echo "  → 프로그램 실행 중이 아님"

# 4. 로그 확인
echo ""
echo "[4] 최근 로그 확인 (테스트 포트 관련):"
if [ -f /home/khadas/humiro_fire_suppression/logs/humiro-service.log ]; then
    tail -20 /home/khadas/humiro_fire_suppression/logs/humiro-service.log | grep -i "test\|15000" || echo "  → 관련 로그 없음"
else
    echo "  → 로그 파일 없음"
fi

# 5. 테스트 메시지 전송 (간단한 테스트)
echo ""
echo "[5] 테스트 메시지 전송 (로컬):"
echo "  → Python GUI 프로그램에서 메시지 전송 후 위 로그 확인"

echo ""
echo "=========================================="
echo "확인 완료"
echo "=========================================="

