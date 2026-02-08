#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# =============================================================================
# 003-apply_config.sh - device_config.env 설정 적용
# =============================================================================
# 용도: device_config.env의 설정을 시스템에 적용
#       설치 후에도 언제든 독립적으로 실행 가능
#
# 실행: sudo ./003-apply_config.sh
#
# 사용 시나리오:
#   1. 최초 설치 시 (000-install_all.sh에서 호출)
#   2. 설정 변경 시 (device_config.env 수정 후 직접 실행)
#   3. 다른 드론으로 복제 시 (device_config.env 변경 후 실행)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
CONFIG_FILE="$DEVICE_CONFIG"

# sudo 실행 시에도 실제 사용자 홈 디렉토리 사용
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

# sudo 권한 확인
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: 이 스크립트는 sudo 권한이 필요합니다."
    echo "실행: sudo ./003-apply_config.sh"
    exit 1
fi

echo "=========================================="
echo "device_config.env 설정 적용"
echo "=========================================="

# -----------------------------------------------------------------------------
# 0. DRONE_ID 기반 자동 설정 생성 (옵션)
# -----------------------------------------------------------------------------
# --auto-generate 옵션이 있거나, DRONE_ID만 있고 다른 설정이 없는 경우 자동 생성
echo ""
echo "[0/10] DRONE_ID 기반 자동 설정 확인..."

# device_config.env에서 DRONE_ID만 읽기
if [ -f "$DEVICE_CONFIG" ]; then
    CURRENT_DRONE_ID=$(grep "^DRONE_ID=" "$DEVICE_CONFIG" 2>/dev/null | cut -d= -f2 | tr -d ' ')
    CURRENT_ETH0_IP=$(grep "^ETH0_IP=" "$DEVICE_CONFIG" 2>/dev/null | cut -d= -f2 | tr -d ' ')

    # DRONE_ID가 있으면 자동 설정 생성 여부 확인
    if [ -n "$CURRENT_DRONE_ID" ]; then
        # DRONE_ID 기반 예상 ETH0_IP 계산
        EXPECTED_ETH0_IP="10.0.0.$((CURRENT_DRONE_ID * 10 + 1))"

        # --auto-generate 옵션이거나 ETH0_IP가 DRONE_ID와 맞지 않으면 자동 생성
        if [ "$1" = "--auto-generate" ] || [ "$2" = "--auto-generate" ] || [ "$CURRENT_ETH0_IP" != "$EXPECTED_ETH0_IP" ]; then
            echo "  DRONE_ID=$CURRENT_DRONE_ID 기반으로 설정 자동 생성 중..."

            # Python으로 ConfigManager 호출하여 설정 재생성
            python3 << PYTHON_EOF
import sys
sys.path.insert(0, "$PROJECT_ROOT/gui")
from utils.config_manager import ConfigManager

cm = ConfigManager("$PROJECT_ROOT")
drone_id = $CURRENT_DRONE_ID

# 현재 GCS_PORT_MODE 유지
gcs_mode = cm.get("GCS_PORT_MODE", "separate")

# DRONE_ID 기반 설정 재생성 (WiFi 설정은 유지)
if cm.regenerate_config_from_drone_id(drone_id, preserve_wifi=True, gcs_port_mode=gcs_mode):
    print(f"  ✓ DRONE_ID={drone_id} 기반 설정 자동 생성 완료")
    config = cm.get_all_config()
    print(f"    - ETH0_IP: {config.get('ETH0_IP')}")
    print(f"    - FC_IP: {config.get('FC_IP')}")
    print(f"    - WIFI_IP: {config.get('WIFI_IP')}")
    print(f"    - ROS_NAMESPACE: {config.get('ROS_NAMESPACE')}")
    print(f"    - ROLE: {config.get('ROLE')}")
else:
    print("  ⚠ 설정 자동 생성 실패")
    sys.exit(1)
PYTHON_EOF

            if [ $? -ne 0 ]; then
                echo "  ⚠ 자동 설정 생성 실패, 기존 설정 사용"
            fi
        else
            echo "  ✓ 설정이 DRONE_ID와 일치함, 자동 생성 건너뜀"
        fi
    fi
fi
echo ""
echo ""
echo "사용자: $REAL_USER"
echo "홈 디렉토리: $REAL_HOME"
echo ""

# 필수 도구 확인
echo "필수 도구 확인 중..."
if ! command -v netplan > /dev/null 2>&1; then
    echo "  netplan 미설치, 설치 중..."
    apt update && apt install -y netplan.io
fi
echo "  ✓ netplan: $(netplan --version 2>/dev/null || echo '설치됨')"
echo ""

# -----------------------------------------------------------------------------
# 1. 설정 파일 로드
# -----------------------------------------------------------------------------
echo "[1/10] 설정 파일 로드..."

if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: $CONFIG_FILE 파일이 없습니다."
    exit 1
fi

source "$CONFIG_FILE"

# WiFi 서브넷 기반 브로드캐스트 주소 자동 계산
# 예: 192.168.100.31 → 192.168.100.255
WIFI_BROADCAST="${WIFI_IP%.*}.255"

echo ""
echo "┌─ 기체별 수정 항목 ─────────────────────────┐"
echo "  드론 번호: #$DRONE_ID"
echo "  역할: $ROLE"
echo "  ROS 네임스페이스: $ROS_NAMESPACE"
echo "  리더 네임스페이스: ${LEADER_NAMESPACE:-drone1}"
echo "  eth0 IP: $ETH0_IP"
echo "  WiFi IP: $WIFI_IP"
echo "├─ 편대 기본값 ──────────────────────────────┤"
echo "  이동: 좌/우=${FORMATION_OFFSET_RIGHT:-0}cm 앞/뒤=${FORMATION_OFFSET_BEHIND:-0}cm 상/하=${FORMATION_OFFSET_ABOVE:-0}cm"
echo "  진압: 거리=${SUPPRESS_DISTANCE:-10}m 각도=${SUPPRESS_ANGLE:-0}°"
echo "└────────────────────────────────────────────┘"
echo ""
echo "┌─ 자동 설정 (수정 불필요) ──────────────────┐"
echo "  FC IP: $FC_IP (DHCP 고정)"
echo "  QGC: 브로드캐스트 ($WIFI_BROADCAST:$QGC_UDP_PORT)"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "└────────────────────────────────────────────┘"
echo ""

# DHCP 범위: FC_IP 하나만 할당 (항상 동일한 IP 보장)
DHCP_RANGE="${FC_IP}"

# -----------------------------------------------------------------------------
# 2. eth0 Netplan 설정
# -----------------------------------------------------------------------------
echo "[2/10] eth0 네트워크 설정..."

mkdir -p /etc/netplan

tee /etc/netplan/01-eth0-px4.yaml > /dev/null << EOF
# Drone #$DRONE_ID eth0 네트워크 설정
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - $ETH0_IP/24
EOF

echo "  ✓ eth0 Netplan 설정 완료"

# -----------------------------------------------------------------------------
# 3. WiFi Netplan 설정 (최대 3개 SSID 지원)
# -----------------------------------------------------------------------------
echo "[3/10] WiFi 네트워크 설정..."

WIFI_INTERFACE_VALUE=${WIFI_INTERFACE:-wlan0}

# 최소 1개의 SSID가 설정되어 있는지 확인
if [ -n "$WIFI_SSID_1" ] && [ "$WIFI_SSID_1" != '""' ] && [ "$WIFI_SSID_1" != "''" ]; then
    
    # 임시 파일 생성
    TEMP_WIFI=$(mktemp)
    
    # 기본 WiFi 설정 작성
    cat > "$TEMP_WIFI" << EOF
# Drone #$DRONE_ID WiFi 네트워크 설정
network:
  version: 2
  renderer: NetworkManager
  wifis:
    $WIFI_INTERFACE_VALUE:
      dhcp4: no
      addresses:
        - $WIFI_IP/24
      routes:
        - to: default
          via: $WIFI_GATEWAY
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
EOF

    # SSID 1 추가 (필수)
    WIFI_SSID_1_CLEAN=$(echo "$WIFI_SSID_1" | tr -d '"' | tr -d "'")
    WIFI_PASSWORD_1_CLEAN=$(echo "$WIFI_PASSWORD_1" | tr -d '"' | tr -d "'")
    echo "        \"$WIFI_SSID_1_CLEAN\":" >> "$TEMP_WIFI"
    echo "          password: \"$WIFI_PASSWORD_1_CLEAN\"" >> "$TEMP_WIFI"
    echo "  - SSID 1: $WIFI_SSID_1_CLEAN"

    # SSID 2 추가 (선택)
    if [ -n "$WIFI_SSID_2" ] && [ "$WIFI_SSID_2" != '""' ] && [ "$WIFI_SSID_2" != "''" ]; then
        WIFI_SSID_2_CLEAN=$(echo "$WIFI_SSID_2" | tr -d '"' | tr -d "'")
        WIFI_PASSWORD_2_CLEAN=$(echo "$WIFI_PASSWORD_2" | tr -d '"' | tr -d "'")
        echo "        \"$WIFI_SSID_2_CLEAN\":" >> "$TEMP_WIFI"
        echo "          password: \"$WIFI_PASSWORD_2_CLEAN\"" >> "$TEMP_WIFI"
        echo "  - SSID 2: $WIFI_SSID_2_CLEAN"
    fi

    # SSID 3 추가 (선택)
    if [ -n "$WIFI_SSID_3" ] && [ "$WIFI_SSID_3" != '""' ] && [ "$WIFI_SSID_3" != "''" ]; then
        WIFI_SSID_3_CLEAN=$(echo "$WIFI_SSID_3" | tr -d '"' | tr -d "'")
        WIFI_PASSWORD_3_CLEAN=$(echo "$WIFI_PASSWORD_3" | tr -d '"' | tr -d "'")
        echo "        \"$WIFI_SSID_3_CLEAN\":" >> "$TEMP_WIFI"
        echo "          password: \"$WIFI_PASSWORD_3_CLEAN\"" >> "$TEMP_WIFI"
        echo "  - SSID 3: $WIFI_SSID_3_CLEAN"
    fi

    # SSID 4 추가 (선택)
    if [ -n "$WIFI_SSID_4" ] && [ "$WIFI_SSID_4" != '""' ] && [ "$WIFI_SSID_4" != "''" ]; then
        WIFI_SSID_4_CLEAN=$(echo "$WIFI_SSID_4" | tr -d '"' | tr -d "'")
        WIFI_PASSWORD_4_CLEAN=$(echo "$WIFI_PASSWORD_4" | tr -d '"' | tr -d "'")
        echo "        \"$WIFI_SSID_4_CLEAN\":" >> "$TEMP_WIFI"
        echo "          password: \"$WIFI_PASSWORD_4_CLEAN\"" >> "$TEMP_WIFI"
        echo "  - SSID 4: $WIFI_SSID_4_CLEAN"
    fi

    # SSID 5 추가 (선택)
    if [ -n "$WIFI_SSID_5" ] && [ "$WIFI_SSID_5" != '""' ] && [ "$WIFI_SSID_5" != "''" ]; then
        WIFI_SSID_5_CLEAN=$(echo "$WIFI_SSID_5" | tr -d '"' | tr -d "'")
        WIFI_PASSWORD_5_CLEAN=$(echo "$WIFI_PASSWORD_5" | tr -d '"' | tr -d "'")
        echo "        \"$WIFI_SSID_5_CLEAN\":" >> "$TEMP_WIFI"
        echo "          password: \"$WIFI_PASSWORD_5_CLEAN\"" >> "$TEMP_WIFI"
        echo "  - SSID 5: $WIFI_SSID_5_CLEAN"
    fi

    # 설정 파일 적용
    mv "$TEMP_WIFI" /etc/netplan/02-wlan0-wifi.yaml
    chmod 600 /etc/netplan/02-wlan0-wifi.yaml
    
    echo "  ✓ WiFi Netplan 설정 완료"
else
    echo "  ⚠ WiFi SSID가 설정되지 않음, 건너뜀"
fi

# -----------------------------------------------------------------------------
# 4. dnsmasq DHCP 설정
# -----------------------------------------------------------------------------
echo "[4/10] dnsmasq DHCP 서버 설정..."

mkdir -p /etc/dnsmasq.d

# dnsmasq 설정 - DHCP 범위를 FC_IP 하나로 제한
tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
# Drone #$DRONE_ID DHCP 서버 설정
# DHCP 범위를 FC_IP 하나로 제한하여 항상 동일한 IP 할당
interface=eth0
bind-interfaces
dhcp-range=$DHCP_RANGE,$DHCP_RANGE,255.255.255.0,infinite
dhcp-option=3,$ETH0_IP
dhcp-option=6,8.8.8.8
log-queries
log-dhcp
log-facility=/var/log/dnsmasq-px4.log
EOF

echo "  ✓ dnsmasq 설정 완료 (FC IP: $FC_IP 고정)"

# -----------------------------------------------------------------------------
# 5. mavlink-router 설정 (DRONE_ID 기반 포트 자동 계산)
# -----------------------------------------------------------------------------
echo "[5/10] mavlink-router 설정..."

if [ -d /etc/mavlink-router ] || command -v mavlink-routerd > /dev/null 2>&1; then
    mkdir -p /etc/mavlink-router

    # DRONE_ID 기반 포트 계산 (10씩 증가)
    # 드론 1: 14550, 14551, 16001, 15001, 5790
    # 드론 2: 14560, 14561, 16011, 15011, 5800
    # 드론 3: 14570, 14571, 16021, 15021, 5810
    PORT_OFFSET=$(( ($DRONE_ID - 1) * 10 ))
    QGC_PORT=$(( 14550 + $PORT_OFFSET ))
    ROS2_PORT=$(( 14551 + $PORT_OFFSET ))
    EXTERNAL_PORT_CALC=$(( 16001 + $PORT_OFFSET ))
    APP_PORT=$(( 15001 + $PORT_OFFSET ))
    TCP_PORT=$(( 5790 + $PORT_OFFSET ))

    # device_config.env의 EXTERNAL_UDP_PORT 사용 (있으면 우선, 없으면 계산값)
    EXTERNAL_PORT_FINAL=${EXTERNAL_UDP_PORT:-$EXTERNAL_PORT_CALC}

    echo "  → DRONE_ID $DRONE_ID 기반 포트 계산:"
    echo "    - TCP: $TCP_PORT"
    echo "    - QGC: $QGC_PORT"
    echo "    - ROS2: $ROS2_PORT"
    echo "    - External: $EXTERNAL_PORT_FINAL"
    echo "    - App: $APP_PORT"

    tee /etc/mavlink-router/main.conf > /dev/null << EOF
# Drone #$DRONE_ID mavlink-router 설정 (자동 생성)

[General]
TcpServerPort = $TCP_PORT
ReportStats = false
MavlinkDialect = common

# FC (PX4) 연결 - 브로드캐스트 수신
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = $FC_MAVLINK_PORT

# GCS (QGroundControl) 브로드캐스트 - 드론 $DRONE_ID 전용 포트
[UdpEndpoint GCS]
Mode = Normal
Address = $WIFI_BROADCAST
Port = $QGC_PORT

# 외부 테스트/디버깅 도구 (SENDER GUI 등) - 드론 $DRONE_ID 전용 포트
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = $EXTERNAL_PORT_FINAL

# ROS2 노드 연결 - 드론 $DRONE_ID 전용 포트
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = $ROS2_PORT

# Application Manager 연결 - 드론 $DRONE_ID 전용 포트 (라우터와 충돌 방지)
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = $APP_PORT
EOF
    echo "  ✓ mavlink-router 설정 완료 (DRONE_ID 기반, 다중 드론 지원)"
else
    echo "  ⚠ mavlink-router 미설치, 건너뜀"
fi

# -----------------------------------------------------------------------------
# 6. micro-ROS Agent wrapper 설정
# -----------------------------------------------------------------------------
echo "[6/10] micro-ROS Agent 설정..."

# 환경 변수 사용 (setup_env.sh에서 로드됨)
WRAPPER_SCRIPT="$SCRIPTS_RUNTIME/start_micro_ros_agent_wrapper.sh"

cat > "$WRAPPER_SCRIPT" << EOF
#!/bin/bash
# Micro-ROS Agent wrapper script
# Generated by 003_apply_config.sh

# 프로젝트 환경 변수 로드
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="\$(cd "\$SCRIPT_DIR/../.." && pwd)"
source "\$PROJECT_ROOT/setup_env.sh"

export HOME=$REAL_HOME
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
# ROS_NAMESPACE는 PX4 uXRCE-DDS와 호환성 문제가 있을 수 있으므로 제거
# PX4 uXRCE-DDS는 기본적으로 네임스페이스를 사용하지 않음
# export ROS_NAMESPACE=$ROS_NAMESPACE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ROS2 환경 로드
source /opt/ros/humble/setup.bash

# 워크스페이스 로드 (환경 변수 사용)
if [ -f "\$MICRO_ROS_WS/install/setup.bash" ]; then
    source "\$MICRO_ROS_WS/install/setup.bash"
fi

if [ -f "\$PX4_ROS2_WS/install/setup.bash" ]; then
    source "\$PX4_ROS2_WS/install/setup.bash"
fi

# 라이브러리 경로
if [ -d "\$MICRO_ROS_WS/install/micro_ros_agent/lib" ]; then
    export LD_LIBRARY_PATH="\$MICRO_ROS_WS/install/micro_ros_agent/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:\${LD_LIBRARY_PATH}"
fi

# micro-ROS Agent 실행
if [ -f "\$MICRO_ROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" ]; then
    exec "\$MICRO_ROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" udp4 --port $XRCE_DDS_PORT
else
    echo "Error: micro_ros_agent not found at \$MICRO_ROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
    exit 1
fi
EOF

chmod +x "$WRAPPER_SCRIPT"
chown $REAL_USER:$REAL_USER "$WRAPPER_SCRIPT"
echo "  ✓ micro-ROS Agent wrapper 설정 완료"

# systemd 서비스 환경변수 오버라이드
mkdir -p /etc/systemd/system/micro-ros-agent.service.d/
tee /etc/systemd/system/micro-ros-agent.service.d/override.conf > /dev/null << EOF
[Service]
Environment="ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
# ROS_NAMESPACE는 PX4 uXRCE-DDS와 호환성 문제가 있을 수 있으므로 제거
# PX4 uXRCE-DDS는 기본적으로 네임스페이스를 사용하지 않음
# Environment="ROS_NAMESPACE=$ROS_NAMESPACE"
EOF

echo "  ✓ systemd 환경변수 설정 완료"

# -----------------------------------------------------------------------------
# 6.5. FastDDS 프로파일 설정 (WiFi 인터페이스 추가 - 드론 간 DDS 통신)
# -----------------------------------------------------------------------------
echo "[6.5/10] FastDDS 프로파일 설정..."

FASTDDS_XML="$PROJECT_ROOT/config/fastdds_eth0_only.xml"

cat > "$FASTDDS_XML" << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>eth0_only_udp</transport_id>
                <type>UDPv4</type>
                <interfaceWhiteList>
                    <address>$ETH0_IP</address>
                    <address>$WIFI_IP</address>
                    <address>127.0.0.1</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="participant_profile" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>eth0_only_udp</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
EOF

chown $REAL_USER:$REAL_USER "$FASTDDS_XML"
echo "  ✓ FastDDS 프로파일 생성 (eth0: $ETH0_IP, WiFi: $WIFI_IP)"

# -----------------------------------------------------------------------------
# 7. 드론 설정 정보 저장 및 .bashrc 업데이트
# -----------------------------------------------------------------------------
echo "[7/10] 설정 저장..."

# /etc/drone-config 저장
tee /etc/drone-config > /dev/null << EOF
# Drone Configuration - Drone #$DRONE_ID
# Generated: $(date)

DRONE_ID=$DRONE_ID
DRONE_NUM=$DRONE_ID
ROS_NAMESPACE=$ROS_NAMESPACE
ETH0_IP=$ETH0_IP
FC_IP=$FC_IP
WIFI_IP=$WIFI_IP
WIFI_GATEWAY=$WIFI_GATEWAY
WIFI_BROADCAST=$WIFI_BROADCAST
QGC_UDP_PORT=$QGC_UDP_PORT
ROS_DOMAIN_ID=$ROS_DOMAIN_ID
XRCE_DDS_PORT=$XRCE_DDS_PORT
FC_MAVLINK_PORT=$FC_MAVLINK_PORT
EOF

echo "  ✓ /etc/drone-config 저장 완료"

# .bashrc 업데이트
BASHRC="$REAL_HOME/.bashrc"
if [ -f "$BASHRC" ]; then
    # 기존 ROS/humiro_fire_suppression 관련 설정 완전히 제거
    # 주의: 기존 .bashrc의 if-fi 쌍을 손상시키지 않도록 주의
    # 마커 주석을 기준으로 정확한 블록만 제거
    
    # 백업 생성 (안전장치)
    cp "$BASHRC" "${BASHRC}.backup.$(date +%Y%m%d_%H%M%S)" 2>/dev/null || true
    
    # 방법 1: 마커 주석이 있는 블록 제거 (가장 안전)
    # humiro_fire_suppression 관련 설정 블록만 제거
    sed -i '/# ROS2 환경 자동 로드 (humiro_fire_suppression)/,/# humiro_fire_suppression 환경 설정 끝/d' "$BASHRC"
    
    # 방법 2: 혼자 있는 ROS 관련 라인들 제거 (마커가 없는 경우)
    # 주의: 기존 .bashrc의 다른 if-fi 쌍을 건드리지 않도록 주의
    # humiro_fire_suppression 관련 설정만 제거 (다른 ROS 설정은 유지)
    sed -i '/^export ROS_DOMAIN_ID=0$/d' "$BASHRC"
    sed -i '/^export ROS_NAMESPACE=drone[0-9]*$/d' "$BASHRC"
    sed -i '/^export RMW_IMPLEMENTATION=rmw_fastrtps_cpp$/d' "$BASHRC"
    sed -i '/^export FASTRTPS_DEFAULT_PROFILES_FILE=.*humiro_fire_suppression/d' "$BASHRC"
    # humiro_fire_suppression 관련 주석이 있는 setup.bash만 제거
    sed -i '/# humiro_fire_suppression/,/^source \/opt\/ros\/humble\/setup.bash$/d' "$BASHRC"
    sed -i '/\$PX4_ROS2_WS\/install\/setup.bash/d' "$BASHRC"
    sed -i '/\$MICRO_ROS_WS\/install\/setup.bash/d' "$BASHRC"
    sed -i '/micro-ros-agent()/,/^}/d' "$BASHRC"
    
    # 연속된 빈 줄 정리 (최대 2개 연속 빈 줄을 1개로)
    sed -i '/^$/{N;/^\n$/d}' "$BASHRC"
    
    # 새 설정 추가 (환경 변수 사용)
    # 주의: 제거 후 반드시 추가하여 ROS 설정이 항상 존재하도록 보장
    cat >> "$BASHRC" << BASHRC_EOF

# ROS2 환경 자동 로드 (humiro_fire_suppression)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/humiro_fire_suppression/config/fastdds_eth0_only.xml

# humiro_fire_suppression 환경 설정 (Generated by 003_apply_config.sh)
if [ -f $PROJECT_ROOT/setup_env.sh ]; then
    source $PROJECT_ROOT/setup_env.sh
fi
# device_config.env에서 DRONE_ID 로드 (ROS2 메시지의 target_system에 사용됨)
if [ -f $PROJECT_ROOT/config/device_config.env ]; then
    source $PROJECT_ROOT/config/device_config.env
    export DRONE_ID=$DRONE_ID
fi
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
# ROS_NAMESPACE는 PX4 uXRCE-DDS와 호환성 문제가 있을 수 있으므로 제거
# PX4 uXRCE-DDS는 기본적으로 네임스페이스를 사용하지 않음
# export ROS_NAMESPACE=$ROS_NAMESPACE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
if [ -f "\$PX4_ROS2_WS/install/setup.bash" ]; then
    source "\$PX4_ROS2_WS/install/setup.bash"
fi
if [ -f "\$MICRO_ROS_WS/install/setup.bash" ]; then
    source "\$MICRO_ROS_WS/install/setup.bash"
fi
# humiro_fire_suppression 환경 설정 끝

BASHRC_EOF
    chown $REAL_USER:$REAL_USER "$BASHRC"
    
    # 문법 검사 및 ROS 설정 확인 (설정이 제대로 추가되었는지 확인)
    if bash -n "$BASHRC" 2>/dev/null; then
        # ROS 설정이 추가되었는지 확인
        if grep -q "ROS2 환경 자동 로드 (humiro_fire_suppression)" "$BASHRC" && \
           grep -q "export ROS_DOMAIN_ID=" "$BASHRC"; then
            echo "  ✓ .bashrc 업데이트 완료 (ROS 환경 재설정)"
        else
            echo "  ⚠ ROS 설정 추가 확인 실패, 백업에서 복구 중..."
            LATEST_BACKUP=$(ls -t "${BASHRC}.backup."* 2>/dev/null | head -1)
            if [ -n "$LATEST_BACKUP" ]; then
                cp "$LATEST_BACKUP" "$BASHRC"
                chown $REAL_USER:$REAL_USER "$BASHRC"
                echo "  ✓ 백업에서 복구 완료"
            fi
        fi
    else
        echo "  ⚠ .bashrc 문법 오류 발생, 백업에서 복구 중..."
        # 백업에서 복구 시도
        LATEST_BACKUP=$(ls -t "${BASHRC}.backup."* 2>/dev/null | head -1)
        if [ -n "$LATEST_BACKUP" ]; then
            cp "$LATEST_BACKUP" "$BASHRC"
            chown $REAL_USER:$REAL_USER "$BASHRC"
            echo "  ✓ 백업에서 복구 완료"
        else
            echo "  ✗ 백업 파일 없음, 수동 확인 필요"
        fi
    fi
fi

# -----------------------------------------------------------------------------
# 8. 워크스페이스 소유권 및 NetworkManager 정리
# -----------------------------------------------------------------------------
echo "[8/10] 워크스페이스 소유권 설정..."

# 워크스페이스 소유권 변경 (root → 사용자, 환경 변수 사용)
if [ -d "$MICRO_ROS_WS" ]; then
    chown -R $REAL_USER:$REAL_USER "$MICRO_ROS_WS"
    echo "  ✓ micro_ros_ws 소유권 설정"
fi

if [ -d "$PX4_ROS2_WS" ]; then
    chown -R $REAL_USER:$REAL_USER "$PX4_ROS2_WS"
    echo "  ✓ px4_ros2_ws 소유권 설정"
fi

if [ -d "$MAVLINK_ROUTER_DIR" ]; then
    chown -R $REAL_USER:$REAL_USER "$MAVLINK_ROUTER_DIR"
    echo "  ✓ mavlink-router 소유권 설정"
fi

# 프로젝트 디렉토리 전체 소유권 확인
chown -R $REAL_USER:$REAL_USER "$PROJECT_WORKSPACES" 2>/dev/null || true

echo ""
echo "[9/10] NetworkManager WiFi 연결 정리..."

# 기존 nmtui/nmcli로 만든 WiFi 연결 삭제 (netplan 설정과 충돌 방지)
# netplan- 으로 시작하는 연결은 유지 (003에서 생성한 것)
if command -v nmcli > /dev/null 2>&1; then
    # netplan-으로 시작하지 않는 WiFi 연결만 삭제
    # set -e 때문에 오류 시 중단 방지를 위해 서브셸 사용
    set +e  # 일시적으로 set -e 비활성화
    WIFI_CONNECTIONS=$(nmcli -t -f NAME,TYPE connection show 2>/dev/null | grep ":802-11-wireless" 2>/dev/null | cut -d: -f1 | grep -v "^netplan-" 2>/dev/null || true)
    set -e  # 다시 활성화
    if [ -n "$WIFI_CONNECTIONS" ]; then
        echo "  nmtui/수동 WiFi 연결 정리 중 (netplan 연결은 유지)..."
        set +e  # 일시적으로 set -e 비활성화
        while IFS= read -r conn; do
            if [ -n "$conn" ]; then
                nmcli connection delete "$conn" 2>/dev/null && \
                    echo "  ✓ 삭제됨: $conn" || \
                    echo "  ⚠ 삭제 실패: $conn"
            fi
        done <<< "$WIFI_CONNECTIONS"
        set -e  # 다시 활성화
    else
        echo "  ✓ 정리할 nmtui 연결 없음 (netplan 연결만 존재)"
    fi
else
    echo "  ⚠ nmcli 없음, 건너뜀"
fi

# -----------------------------------------------------------------------------
# 10. 서비스 활성화, 재시작 및 자동 시작 설정
# -----------------------------------------------------------------------------
echo ""
echo "[10/10] 서비스 설정 및 자동 시작 활성화..."

# Netplan 적용
echo "  네트워크 설정 적용 중..."
netplan apply 2>/dev/null || echo "  ⚠ netplan 적용 실패 (재부팅 후 적용됨)"

# systemd 데몬 재로드
systemctl daemon-reload

# 서비스 활성화 및 재시작 함수
enable_and_restart_service() {
    local service_name=$1
    
    if systemctl list-unit-files | grep -q "^${service_name}"; then
        # 서비스 활성화 (부팅 시 자동 시작)
        systemctl enable "$service_name" 2>/dev/null && \
            echo "  ✓ $service_name 자동 시작 활성화" || \
            echo "  ⚠ $service_name 자동 시작 활성화 실패"
        
        # 서비스 재시작
        systemctl restart "$service_name" 2>/dev/null && \
            echo "  ✓ $service_name 재시작 완료" || \
            echo "  ⚠ $service_name 재시작 실패 (재부팅 후 시작됨)"
    else
        echo "  ⚠ $service_name 서비스 없음, 건너뜀"
    fi
}

# systemd 서비스 파일 업데이트 (새 경로 반영)
echo ""
echo "  systemd 서비스 파일 업데이트 중..."

# micro-ros-agent.service 업데이트
if [ -f "$PROJECT_DEPLOYMENT/systemd/micro-ros-agent.service" ]; then
    # 사용자 이름을 실제 사용자로 대체
    sed "s|%h|$REAL_HOME|g; s|khadas|$REAL_USER|g" "$PROJECT_DEPLOYMENT/systemd/micro-ros-agent.service" | \
        sudo tee /etc/systemd/system/micro-ros-agent.service > /dev/null
    echo "  ✓ micro-ros-agent.service 업데이트 완료"
fi

# mavlink-router.service 업데이트
if [ -f "$PROJECT_DEPLOYMENT/systemd/mavlink-router.service" ]; then
    sudo cp "$PROJECT_DEPLOYMENT/systemd/mavlink-router.service" /etc/systemd/system/
    echo "  ✓ mavlink-router.service 업데이트 완료"
fi

# dnsmasq-px4.service 업데이트
if [ -f "$PROJECT_DEPLOYMENT/systemd/dnsmasq-px4.service" ]; then
    sudo cp "$PROJECT_DEPLOYMENT/systemd/dnsmasq-px4.service" /etc/systemd/system/
    echo "  ✓ dnsmasq-px4.service 업데이트 완료"
fi

echo ""
echo "  서비스 활성화 및 재시작 중..."
enable_and_restart_service "dnsmasq-px4.service"
enable_and_restart_service "mavlink-router.service"
enable_and_restart_service "micro-ros-agent.service"

echo ""
echo "=========================================="
echo "설정 적용 완료!"
echo "=========================================="
echo ""
echo "적용된 설정:"
echo "  - 드론 번호: #$DRONE_ID"
echo "  - 역할: $ROLE"
echo "  - ROS 네임스페이스: $ROS_NAMESPACE"
echo "  - 리더 네임스페이스: $LEADER_NAMESPACE"
echo "  - eth0 IP: $ETH0_IP"
echo "  - WiFi IP: $WIFI_IP"
echo ""
echo "편대 비행 기본값:"
echo "  ┌─ 이동 편대 (NAVIGATE) ─────────────────────┐"
echo "  │  좌/우 오프셋: ${FORMATION_OFFSET_RIGHT:-0} cm"
echo "  │  앞/뒤 오프셋: ${FORMATION_OFFSET_BEHIND:-0} cm"
echo "  │  상/하 오프셋: ${FORMATION_OFFSET_ABOVE:-0} cm"
echo "  ├─ 진압 편대 (SUPPRESS) ─────────────────────┤"
echo "  │  타겟 이격거리: ${SUPPRESS_DISTANCE:-10} m"
echo "  │  배치 각도: ${SUPPRESS_ANGLE:-0}°"
echo "  └────────────────────────────────────────────┘"
echo ""
echo "자동 설정 (변경 불필요):"
echo "  - FC IP: $FC_IP (DHCP 고정)"
echo "  - QGC: 브로드캐스트 ($WIFI_BROADCAST:$QGC_UDP_PORT)"
echo ""
echo "자동 시작 서비스 (부팅 시 자동 실행):"
echo "  - dnsmasq-px4.service"
echo "  - mavlink-router.service"
echo "  - micro-ros-agent.service"
echo ""
echo "PX4 파라미터 설정 (QGroundControl):"
echo "  ┌──────────────────────────────────────────┐"
echo "  │  MAV_SYS_ID = $DRONE_ID"
DECIMAL_IP=$(python3 -c "import struct,socket; print(struct.unpack('!I',socket.inet_aton('$ETH0_IP'))[0])" 2>/dev/null || echo "???")
echo "  │  UXRCE_DDS_AG_IP = $DECIMAL_IP ($ETH0_IP)"
echo "  │  UXRCE_DDS_PRT = $XRCE_DDS_PORT"
echo "  │  UXRCE_DDS_DOM_ID = $ROS_DOMAIN_ID"
echo "  │  UXRCE_DDS_CFG = 1000 (Ethernet)"
echo "  └──────────────────────────────────────────┘"

# -----------------------------------------------------------------------------
# FC 재부팅 함수 (MAVLink PREFLIGHT_REBOOT_SHUTDOWN 명령)
# -----------------------------------------------------------------------------
reboot_fc() {
    echo "  FC 재부팅 명령 전송 중..."

    # pymavlink를 사용하여 FC 재부팅 명령 전송
    # TCP_PORT는 DRONE_ID 기반으로 계산됨 (5790 + (DRONE_ID-1)*10)
    local REBOOT_TCP_PORT=$(( 5790 + (${DRONE_ID:-1} - 1) * 10 ))
    python3 << PYTHON_EOF
import sys
import time
try:
    from pymavlink import mavutil

    # mavlink-router의 TCP 포트에 연결 (드론별 동적 포트)
    master = mavutil.mavlink_connection('tcp:127.0.0.1:${REBOOT_TCP_PORT}', source_system=255, source_component=0)

    # 하트비트 대기 (최대 5초)
    print("    FC 연결 대기 중...")
    msg = master.wait_heartbeat(timeout=5)
    if msg is None:
        print("    ⚠ FC 연결 실패 (하트비트 없음)")
        sys.exit(1)

    print(f"    ✓ FC 연결됨 (sysid={master.target_system}, compid={master.target_component})")

    # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246) 명령 전송
    # param1: 1 = Autopilot 재부팅
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,  # confirmation
        1,  # param1: 1 = reboot autopilot
        0,  # param2
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        0   # param7
    )

    print("    ✓ FC 재부팅 명령 전송 완료")
    time.sleep(1)

except ImportError:
    print("    ⚠ pymavlink 미설치, FC 재부팅 건너뜀")
    sys.exit(1)
except Exception as e:
    print(f"    ⚠ FC 재부팅 실패: {e}")
    sys.exit(1)
PYTHON_EOF

    return $?
}

# -----------------------------------------------------------------------------
# 리부팅 확인
# -----------------------------------------------------------------------------
echo ""
echo "설정을 완전히 적용하려면 재부팅이 필요합니다."
echo ""

# GUI에서 호출된 경우 (--auto-reboot 옵션)
if [ "$1" = "--auto-reboot" ]; then
    echo "자동 재부팅 모드..."
    echo ""

    # FC 먼저 재부팅
    echo "[FC 재부팅]"
    if reboot_fc; then
        echo "  FC 재부팅 시작됨, 10초 대기..."
        sleep 10
    else
        echo "  FC 재부팅 실패, 수동으로 FC를 재부팅하세요."
    fi

    echo ""
    echo "[VIM4 재부팅]"
    echo "  3초 후 VIM4 재부팅..."
    sleep 3
    reboot
    exit 0
fi

# 30초 카운트다운 후 자동 재부팅 (n 입력 시 취소)
echo "30초 후 자동 재부팅됩니다. 취소하려면 'n'을 입력하세요."
echo ""

REBOOT_CANCELLED=false
for i in $(seq 30 -1 1); do
    printf "\r  재부팅까지 %2d초... (n: 취소) " "$i"
    if read -t 1 -n 1 INPUT 2>/dev/null; then
        if [ "$INPUT" = "n" ] || [ "$INPUT" = "N" ]; then
            REBOOT_CANCELLED=true
            break
        fi
    fi
done
echo ""

if [ "$REBOOT_CANCELLED" = false ]; then
    echo ""

    # FC 먼저 재부팅
    echo "[FC 재부팅]"
    if reboot_fc; then
        echo "  FC 재부팅 시작됨, 10초 대기..."
        sleep 10
    else
        echo "  FC 재부팅 실패, 수동으로 FC를 재부팅하세요."
    fi

    echo ""
    echo "[VIM4 재부팅]"
    echo "  3초 후 VIM4 재부팅..."
    sleep 3
    reboot
else
    echo ""
    echo "재부팅이 취소되었습니다."
    echo ""
    echo "다음 단계:"
    echo "  1. sudo reboot (수동 재부팅)"
    echo "  2. ./101_check_px4_connection.sh (연결 확인)"
    echo ""
    echo "설정 변경 방법:"
    echo "  1. device_config.env 수정"
    echo "  2. sudo ./003-apply_config.sh 실행"
    echo ""
fi