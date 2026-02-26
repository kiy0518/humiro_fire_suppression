#!/bin/bash
# =============================================================================
# 네트워크 연결 모니터링 스크립트
# =============================================================================
# eth0 (FC ↔ SBC) 및 wlan0 (SBC ↔ GCS) 연결 상태를 실시간 감시하고
# 끊김/복구 이벤트를 타임스탬프와 함께 로그 파일에 기록합니다.
#
# 감시 대상:
#   1. eth0 물리 링크 상태 (carrier)
#   2. FC ping 응답 (10.0.0.x)
#   3. FC MAVLink UDP 트래픽 (port 14540)
#   4. FC DDS/XRCE UDP 트래픽 (port 8888)
#   5. wlan0 물리 링크 및 신호 강도
#   6. WiFi 게이트웨이 ping
#   7. CPU/메모리 사용률 (과부하 상관분석용)
#
# 사용법:
#   ./network_monitor.sh              # foreground 실행
#   systemctl start network-monitor   # systemd 서비스로 실행
#
# 로그 파일: /home/khadas/humiro_fire_suppression/logs/network_monitor.log
# =============================================================================

set -euo pipefail

# === 설정 ===
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# device_config.env 로드
if [ -f "$PROJECT_ROOT/config/device_config.env" ]; then
    set -a
    source "$PROJECT_ROOT/config/device_config.env"
    set +a
fi

FC_IP="${FC_IP:-10.0.0.12}"
WIFI_GATEWAY="${WIFI_GATEWAY:-192.168.100.1}"
MAVLINK_PORT="${FC_MAVLINK_PORT:-14540}"
DDS_PORT="${XRCE_DDS_PORT:-8888}"
DRONE_ID="${DRONE_ID:-1}"

LOG_DIR="$PROJECT_ROOT/logs"
LOG_FILE="$LOG_DIR/network_monitor.log"
POLL_INTERVAL=2        # 기본 폴링 주기 (초)
TRAFFIC_CHECK_SEC=1    # 트래픽 감지 대기 (초)
MAX_LOG_SIZE=10485760  # 10MB 로그 로테이션

mkdir -p "$LOG_DIR"

# === 상태 변수 ===
prev_eth0_link="unknown"
prev_fc_ping="unknown"
prev_fc_mavlink="unknown"
prev_fc_dds="unknown"
prev_wlan0_link="unknown"
prev_gw_ping="unknown"
prev_fc_bridge_connected="unknown"

# 끊김 통계
eth0_drop_count=0
fc_ping_drop_count=0
fc_mavlink_drop_count=0
fc_dds_drop_count=0
wlan0_drop_count=0
gw_ping_drop_count=0
start_time=$(date +%s)

# === 함수 ===

log() {
    local level="$1"
    shift
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    local msg="[$timestamp] [DRONE$DRONE_ID] [$level] $*"
    echo "$msg" | tee -a "$LOG_FILE"
}

log_event() {
    log "EVENT" "$@"
}

log_info() {
    log "INFO" "$@"
}

log_warn() {
    log "WARN" "$@"
}

log_stat() {
    log "STAT" "$@"
}

# 로그 로테이션
rotate_log() {
    if [ -f "$LOG_FILE" ] && [ "$(stat -f%z "$LOG_FILE" 2>/dev/null || stat -c%s "$LOG_FILE" 2>/dev/null)" -gt "$MAX_LOG_SIZE" ]; then
        mv "$LOG_FILE" "${LOG_FILE}.1"
        log_info "로그 로테이션 완료"
    fi
}

# eth0 물리 링크 상태
check_eth0_link() {
    local carrier
    carrier=$(cat /sys/class/net/eth0/carrier 2>/dev/null || echo "0")
    if [ "$carrier" = "1" ]; then
        echo "up"
    else
        echo "down"
    fi
}

# wlan0 링크 상태 + 신호 강도
check_wlan0_link() {
    local operstate
    operstate=$(cat /sys/class/net/wlan0/operstate 2>/dev/null || echo "down")
    if [ "$operstate" = "up" ]; then
        echo "up"
    else
        echo "down"
    fi
}

get_wifi_signal() {
    # RSSI (dBm) 추출
    local rssi
    rssi=$(iw dev wlan0 link 2>/dev/null | grep 'signal:' | awk '{print $2}')
    if [ -n "$rssi" ]; then
        echo "$rssi"
    else
        echo "N/A"
    fi
}

get_wifi_tx_bitrate() {
    local bitrate
    bitrate=$(iw dev wlan0 link 2>/dev/null | grep 'tx bitrate:' | awk '{print $3, $4}')
    if [ -n "$bitrate" ]; then
        echo "$bitrate"
    else
        echo "N/A"
    fi
}

# ping 체크 (1회, 타임아웃 1초)
check_ping() {
    local target="$1"
    if ping -c 1 -W 1 "$target" >/dev/null 2>&1; then
        echo "ok"
    else
        echo "fail"
    fi
}

# UDP 트래픽 존재 여부 (지정 포트, 1초간 패킷 수)
check_udp_traffic() {
    local port="$1"
    local iface="$2"
    local count
    # timeout 으로 1초간 캡처, 패킷 수 카운트
    count=$(timeout "$TRAFFIC_CHECK_SEC" cat /proc/net/udp 2>/dev/null | wc -l)
    # ss로 수신 바이트 변화 확인 (더 가벼운 방법)
    local rx_before rx_after
    rx_before=$(cat /sys/class/net/"$iface"/statistics/rx_packets 2>/dev/null || echo "0")
    sleep "$TRAFFIC_CHECK_SEC"
    rx_after=$(cat /sys/class/net/"$iface"/statistics/rx_packets 2>/dev/null || echo "0")
    local diff=$((rx_after - rx_before))
    if [ "$diff" -gt 0 ]; then
        echo "active:${diff}pkt/${TRAFFIC_CHECK_SEC}s"
    else
        echo "silent"
    fi
}

# FC로부터 특정 포트 UDP 수신 패킷 카운트 (iptables 없이 ss 사용)
check_fc_port_traffic() {
    local port="$1"
    local duration="$2"
    # /proc/net/udp에서 로컬 포트 바인딩 확인 + nstat으로 패킷 카운트
    local rx_before rx_after
    rx_before=$(cat /sys/class/net/eth0/statistics/rx_bytes 2>/dev/null || echo "0")
    sleep "$duration"
    rx_after=$(cat /sys/class/net/eth0/statistics/rx_bytes 2>/dev/null || echo "0")
    local diff=$((rx_after - rx_before))
    echo "$diff"
}

# FC MAVLink 트래픽 (eth0, port 14540 수신 확인)
check_fc_mavlink() {
    # ss로 14540 포트에 바인딩된 소켓이 있는지 + eth0 rx 변화
    local bound
    bound=$(ss -ulnp 2>/dev/null | grep ":${MAVLINK_PORT} " | wc -l)
    if [ "$bound" -eq 0 ]; then
        echo "no_listener"
        return
    fi
    # eth0 rx_bytes 변화로 트래픽 판단
    local rx1 rx2
    rx1=$(cat /sys/class/net/eth0/statistics/rx_bytes 2>/dev/null || echo "0")
    sleep 1
    rx2=$(cat /sys/class/net/eth0/statistics/rx_bytes 2>/dev/null || echo "0")
    local diff=$((rx2 - rx1))
    if [ "$diff" -gt 100 ]; then
        echo "active"
    else
        echo "silent"
    fi
}

# FC DDS 트래픽 (port 8888)
check_fc_dds() {
    local bound
    bound=$(ss -ulnp 2>/dev/null | grep ":${DDS_PORT} " | wc -l)
    if [ "$bound" -eq 0 ]; then
        echo "no_listener"
        return
    fi
    # eth0 트래픽은 이미 mavlink에서 체크했으므로 소켓 존재만 확인
    echo "bound"
}

# CPU/메모리 사용률
get_system_load() {
    local cpu_load mem_used
    cpu_load=$(awk '{print $1}' /proc/loadavg)
    mem_used=$(free -m | awk '/^Mem:/{printf "%.0f", $3/$2*100}')
    echo "load=${cpu_load},mem=${mem_used}%"
}

# fc_bridge 프로세스 상태
check_fc_bridge_process() {
    if pgrep -f "fc_bridge" >/dev/null 2>&1; then
        echo "running"
    else
        echo "stopped"
    fi
}

# humiro_fire_suppression CPU 사용률
get_app_cpu() {
    local pid
    pid=$(pgrep -f "humiro_fire_suppression" -o 2>/dev/null || echo "")
    if [ -n "$pid" ]; then
        local cpu
        cpu=$(ps -p "$pid" -o %cpu --no-headers 2>/dev/null | tr -d ' ')
        echo "${cpu}%"
    else
        echo "N/A"
    fi
}

# mavlink-router 프로세스 상태
check_mavlink_router() {
    if pgrep -f "mavlink-routerd" >/dev/null 2>&1; then
        echo "running"
    else
        echo "stopped"
    fi
}

# === 상태 변화 감지 및 이벤트 로깅 ===

check_and_log_transition() {
    local name="$1"
    local current="$2"
    local prev_var_name="$3"
    local drop_var_name="$4"

    eval "local prev=\$$prev_var_name"

    if [ "$current" != "$prev" ]; then
        if [ "$prev" = "unknown" ]; then
            log_info "$name: 초기상태=$current"
        elif [ "$current" = "down" ] || [ "$current" = "fail" ] || [ "$current" = "silent" ] || [ "$current" = "no_listener" ] || [ "$current" = "stopped" ]; then
            eval "$drop_var_name=\$((\$$drop_var_name + 1))"
            eval "local count=\$$drop_var_name"
            log_event "▼ $name 끊김! ($prev → $current) [누적 ${count}회]"
        else
            log_event "▲ $name 복구! ($prev → $current)"
        fi
        eval "$prev_var_name=\"$current\""
    fi
}

# === 주기적 통계 출력 ===

print_periodic_stats() {
    local uptime_sec=$(( $(date +%s) - start_time ))
    local uptime_min=$((uptime_sec / 60))
    local wifi_rssi
    wifi_rssi=$(get_wifi_signal)
    local wifi_bitrate
    wifi_bitrate=$(get_wifi_tx_bitrate)
    local sys_load
    sys_load=$(get_system_load)
    local app_cpu
    app_cpu=$(get_app_cpu)

    log_stat "=== ${uptime_min}분 경과 | 끊김횟수: eth0=${eth0_drop_count} fc_ping=${fc_ping_drop_count} mavlink=${fc_mavlink_drop_count} dds=${fc_dds_drop_count} wlan0=${wlan0_drop_count} gw_ping=${gw_ping_drop_count} | WiFi=${wifi_rssi}dBm(${wifi_bitrate}) | ${sys_load} | app_cpu=${app_cpu} ==="
}

# === 시그널 핸들러 ===

cleanup() {
    log_info "네트워크 모니터링 종료 (SIGTERM/SIGINT)"
    print_periodic_stats
    exit 0
}

trap cleanup SIGTERM SIGINT

# === 메인 루프 ===

log_info "============================================"
log_info "네트워크 모니터링 시작 (DRONE_ID=$DRONE_ID)"
log_info "  FC: $FC_IP (eth0, MAVLink:$MAVLINK_PORT, DDS:$DDS_PORT)"
log_info "  GW: $WIFI_GATEWAY (wlan0)"
log_info "  폴링: ${POLL_INTERVAL}초"
log_info "  로그: $LOG_FILE"
log_info "============================================"

tick=0
STAT_INTERVAL=60  # 60초마다 통계 출력

while true; do
    # 1. eth0 물리 링크
    eth0_link=$(check_eth0_link)
    check_and_log_transition "eth0_link" "$eth0_link" "prev_eth0_link" "eth0_drop_count"

    # 2. FC ping
    fc_ping=$(check_ping "$FC_IP")
    check_and_log_transition "FC_ping($FC_IP)" "$fc_ping" "prev_fc_ping" "fc_ping_drop_count"

    # 3. FC MAVLink (eth0:14540 수신)
    fc_mavlink=$(check_fc_mavlink)
    check_and_log_transition "FC_MAVLink(:$MAVLINK_PORT)" "$fc_mavlink" "prev_fc_mavlink" "fc_mavlink_drop_count"

    # 4. FC DDS (port 8888)
    fc_dds=$(check_fc_dds)
    check_and_log_transition "FC_DDS(:$DDS_PORT)" "$fc_dds" "prev_fc_dds" "fc_dds_drop_count"

    # 5. wlan0 링크
    wlan0_link=$(check_wlan0_link)
    check_and_log_transition "wlan0_link" "$wlan0_link" "prev_wlan0_link" "wlan0_drop_count"

    # 6. 게이트웨이 ping
    gw_ping=$(check_ping "$WIFI_GATEWAY")
    check_and_log_transition "GW_ping($WIFI_GATEWAY)" "$gw_ping" "prev_gw_ping" "gw_ping_drop_count"

    # 7. fc_bridge 프로세스
    fc_bridge_status=$(check_fc_bridge_process)
    check_and_log_transition "fc_bridge" "$fc_bridge_status" "prev_fc_bridge_connected" "fc_ping_drop_count"

    # 주기적 통계 (60초마다)
    tick=$((tick + 1))
    if [ $((tick * POLL_INTERVAL)) -ge "$STAT_INTERVAL" ]; then
        print_periodic_stats
        rotate_log
        tick=0
    fi

    sleep "$POLL_INTERVAL"
done
