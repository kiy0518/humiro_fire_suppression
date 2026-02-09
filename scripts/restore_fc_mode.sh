#!/bin/bash
# ==============================================================================
# 부팅 시 FC 모드 자동 복원 스크립트
# VIM4가 재부팅되면 항상 FC 모드로 시작하여 실수 방지
# ==============================================================================

# 설정
PROJECT_DIR="/home/khadas/humiro_fire_suppression"
CONFIG_FILE="$PROJECT_DIR/config/device_config.env"
MAVLINK_CONF="/etc/mavlink-router/main.conf"
LOG_FILE="/var/log/fc-mode-restore.log"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
}

log "=== mavlink-router 설정 확인 시작 ==="

# 003-apply_config.sh가 FC + SITL 듀얼 모드 설정을 생성하므로
# 더 이상 모드 전환이 필요 없음. 설정 파일 존재 여부만 확인.

if [ -f "$MAVLINK_CONF" ]; then
    log "mavlink-router 설정 존재 확인 - 변경 없음"
else
    log "mavlink-router 설정 없음 - 003-apply_config.sh 실행 필요"
fi

log "=== mavlink-router 설정 확인 완료 ==="
