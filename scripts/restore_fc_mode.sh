#!/bin/bash
# ==============================================================================
# 부팅 시 FC 모드 자동 복원 스크립트
# VIM4가 재부팅되면 항상 FC 모드로 시작하여 실수 방지
# ==============================================================================

PROJECT_DIR="/home/khadas/humiro_fire_suppression"
CONFIG_FILE="$PROJECT_DIR/config/device_config.env"
MAVLINK_CONF="/etc/mavlink-router/main.conf"
LOG_FILE="/var/log/fc-mode-restore.log"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
}

log "=== 부팅 시 FC 모드 복원 시작 ==="

# device_config.env에서 설정 읽기
if [ ! -f "$CONFIG_FILE" ]; then
    log "ERROR: $CONFIG_FILE 없음"
    exit 1
fi

source "$CONFIG_FILE"

DRONE_ID=${DRONE_ID:-1}
FC_IP=${FC_IP:-10.0.0.12}
FC_MAVLINK_PORT=${FC_MAVLINK_PORT:-14540}

# DRONE_ID 기반 포트 계산 (003-apply_config.sh와 동일 공식)
PORT_OFFSET=$(( (DRONE_ID - 1) * 10 ))
QGC_PORT=$(( 14550 + PORT_OFFSET ))
ROS2_PORT=$(( 14551 + PORT_OFFSET ))
EXTERNAL_PORT=$(( 16001 + PORT_OFFSET ))
APPLICATION_PORT=$(( 15001 + PORT_OFFSET ))
TCP_PORT=$(( 5790 + PORT_OFFSET ))

# 브로드캐스트 IP 계산
WIFI_IP=${WIFI_IP:-192.168.100.11}
BROADCAST_IP=$(echo "$WIFI_IP" | sed 's/\.[0-9]*$/.255/')

# 현재 모드 확인
if grep -q '\[UdpEndpoint FC\]' "$MAVLINK_CONF" 2>/dev/null && \
   ! grep -q '\[UdpEndpoint SITL\]' "$MAVLINK_CONF" 2>/dev/null && \
   ! grep -q '\[UdpEndpoint PC_SITL\]' "$MAVLINK_CONF" 2>/dev/null; then
    log "이미 FC 모드 - 변경 불필요"
else
    log "SITL 모드 감지 → FC 모드로 복원"

    # FC 모드 main.conf 생성
    cat > /tmp/mavlink_fc_mode.conf << EOF
# Drone #${DRONE_ID} mavlink-router 설정 (자동 생성)
# 생성: $(date)
# 모드: FC 연결 (실제 비행)
# 참고: 부팅 시 자동 복원됨

[General]
TcpServerPort = ${TCP_PORT}
ReportStats = false
MavlinkDialect = common

# FC (PX4) 연결 - 이더넷 (10.0.0.x)
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = ${FC_MAVLINK_PORT}

# GCS (QGroundControl) 브로드캐스트 - 드론 ${DRONE_ID} 전용 포트
[UdpEndpoint GCS]
Mode = Normal
Address = ${BROADCAST_IP}
Port = ${QGC_PORT}

# 외부 테스트/디버깅 도구 (SENDER GUI 등) - 드론 ${DRONE_ID} 전용 포트
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = ${EXTERNAL_PORT}

# ROS2 노드 연결 - 드론 ${DRONE_ID} 전용 포트
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = ${ROS2_PORT}

# Application Manager 연결 - 드론 ${DRONE_ID} 전용 포트 (라우터와 충돌 방지)
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = ${APPLICATION_PORT}
EOF

    cp /tmp/mavlink_fc_mode.conf "$MAVLINK_CONF"
    rm -f /tmp/mavlink_fc_mode.conf
    log "main.conf FC 모드로 복원 완료"
fi

# iptables: FC 모드 규칙 적용 (wlan0 DDS 차단, eth0 허용)
# 기존 규칙 모두 제거
iptables -D INPUT -i eth0 -s "$FC_IP" -p udp --dport 8888 -j DROP 2>/dev/null
iptables -D INPUT -i wlan0 -p udp --dport 8888 -j DROP 2>/dev/null
iptables -D OUTPUT -o eth0 -d "$FC_IP" -p udp -j DROP 2>/dev/null
iptables -D OUTPUT -o wlan0 -p udp --dport 8888 -j DROP 2>/dev/null

# FC 모드: wlan0(SITL) 양방향 차단
iptables -A INPUT -i wlan0 -p udp --dport 8888 -j DROP
iptables -A OUTPUT -o wlan0 -p udp --dport 8888 -j DROP

# 저장 (재부팅 후 iptables-restore용)
iptables-save > /etc/iptables.rules

log "iptables FC 모드 규칙 적용 완료"
log "=== 부팅 시 FC 모드 복원 완료 ==="
