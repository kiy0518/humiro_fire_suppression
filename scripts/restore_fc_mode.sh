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

log "=== FC 모드 복원 시작 ==="

# DRONE_ID 읽기
if [ -f "$CONFIG_FILE" ]; then
    DRONE_ID=$(grep "^DRONE_ID=" "$CONFIG_FILE" | cut -d= -f2)
else
    log "오류: $CONFIG_FILE 파일을 찾을 수 없습니다."
    exit 1
fi

if [ -z "$DRONE_ID" ]; then
    log "오류: DRONE_ID를 읽을 수 없습니다."
    exit 1
fi

log "기체 번호: $DRONE_ID"

# 드론별 포트 계산
EXTERNAL_PORT=$((16000 + DRONE_ID))
APPLICATION_PORT=$((15000 + DRONE_ID))

log "External 포트: $EXTERNAL_PORT"
log "Application 포트: $APPLICATION_PORT"

# 현재 모드 확인
if grep -q "PC_SITL\|SITL" "$MAVLINK_CONF" 2>/dev/null; then
    log "현재 SITL 모드 감지됨 - FC 모드로 복원"

    # FC 모드 설정 작성
    cat > "$MAVLINK_CONF" << EOF
# Drone #$DRONE_ID mavlink-router 설정 (자동 생성)
# 생성: $(date)
# 모드: FC 연결 (실제 비행)
# 참고: 부팅 시 자동 복원됨

[General]
TcpServerPort = 5790
ReportStats = false
MavlinkDialect = common

# FC (PX4) 연결 - 브로드캐스트 수신
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = 14540

# GCS (QGroundControl) 브로드캐스트 - 드론 $DRONE_ID 전용 포트
[UdpEndpoint GCS]
Mode = Normal
Address = 192.168.100.255
Port = 14550

# 외부 테스트/디버깅 도구 (SENDER GUI 등) - 드론 $DRONE_ID 전용 포트
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = $EXTERNAL_PORT

# ROS2 노드 연결 - 드론 $DRONE_ID 전용 포트
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = 14551

# Application Manager 연결 - 드론 $DRONE_ID 전용 포트 (라우터와 충돌 방지)
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = $APPLICATION_PORT
EOF

    log "FC 모드 설정 복원 완료"
else
    log "이미 FC 모드입니다 - 변경 없음"
fi

log "=== FC 모드 복원 완료 ==="
