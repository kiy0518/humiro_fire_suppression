# 🖥️ PX4 SITL 시뮬레이션 PC 설정 가이드 (드론 1)

**작성일:** 2026-01-23  
**대상 PC:** Ubuntu 22.04 (192.168.100.30)  
**연결 대상:** VIM4 컴패니언 컴퓨터 (192.168.100.11)

---

## ⚠️ 사전 요구사항

> **먼저 `SIMULATION_PC_COMMON_SETUP.md`의 공통 설정을 완료하세요!**
> 
> 공통 설정에서 다음 항목들이 완료되어 있어야 합니다:
> - 기본 패키지 설치
> - PX4 설치 및 빌드
> - ROS2 Humble 설치
> - px4_msgs 빌드
> - Micro XRCE-DDS Agent 설치
> - rcS 파일에 VIM4 연결 설정 추가

---

## 📋 드론 1 설정 순서

1. SITL 실행 스크립트 생성
2. SITL 실행

---

## 1. SITL 실행 스크립트 생성

```bash
cat > ~/start_sitl_drone1.sh << 'EOF'
#!/bin/bash
# ==============================================================================
# PX4 SITL for VIM4 연결 (드론 1)
# ==============================================================================

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============ 설정 ============
VIM4_IP="192.168.100.11"
# ==============================

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE} PX4 SITL 시뮬레이션 시작 (드론 1)${NC}"
echo -e "${BLUE}======================================${NC}"
echo -e "  VIM4 IP: ${GREEN}$VIM4_IP${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# 기존 프로세스 정리
echo -e "${YELLOW}[1/4] 기존 프로세스 정리...${NC}"
pkill -9 px4 2>/dev/null || true
pkill -9 gz 2>/dev/null || true
pkill -9 ruby 2>/dev/null || true
pkill -9 MicroXRCEAgent 2>/dev/null || true
sleep 2
echo -e "  ${GREEN}✓${NC} 완료"

# VIM4 연결 테스트
echo -e "${YELLOW}[2/4] VIM4 연결 테스트...${NC}"
if ping -c 1 -W 2 "$VIM4_IP" > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} VIM4 연결 가능: $VIM4_IP"
else
    echo -e "  ${RED}✗${NC} VIM4에 연결할 수 없습니다: $VIM4_IP"
    exit 1
fi

# ROS2 환경 로드
echo -e "${YELLOW}[3/4] ROS2 환경 설정...${NC}"
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0
echo -e "  ${GREEN}✓${NC} ROS_DOMAIN_ID=0"

# Micro XRCE-DDS Agent 시작 (백그라운드)
echo -e "${YELLOW}[4/4] Micro XRCE-DDS Agent 시작...${NC}"
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 2
echo -e "  ${GREEN}✓${NC} Agent PID: $AGENT_PID"

# PX4 SITL 시작
echo ""
echo -e "${BLUE}======================================${NC}"
echo -e "${GREEN} PX4 SITL 실행 중...${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

cd ~/PX4-Autopilot

PX4_SYS_AUTOSTART=4001 \
PX4_HOME_LAT=35.905863 \
PX4_HOME_LON=128.802615 \
PX4_HOME_ALT=0 \
./build/px4_sitl_default/bin/px4 -i 0
EOF

chmod +x ~/start_sitl_drone1.sh
```

---

## 2. SITL 실행

```bash
~/start_sitl_drone1.sh
```

> ✅ **rcS에 설정된 mavlink 명령이 자동 실행되어 VIM4에 연결됩니다!**

---

## 📊 통신 구조

```
시뮬레이션 PC (192.168.100.30)
    │
    │  PX4 SITL (드론 1)
    │  mavlink start -o 18001 -t 192.168.100.11
    │
    ▼ UDP 18001
    
VIM4 (192.168.100.11)
    │
    │  MAVLink Router
    │  [SITL] :18001 (Server) ← 연결
    │  [GCS] → 192.168.100.255:14550
    │
    ▼ UDP 14550

QGC (브로드캐스트)
```

---

## 🔢 드론 1 설정 정보

| 항목 | 값 |
|------|-----|
| VIM4 IP | 192.168.100.11 |
| SITL 포트 | 18001 |
| MAVLink 로컬 포트 | 14540 |
| 인스턴스 | -i 0 |
| MAV_SYS_ID | 4 |

---

## ✅ 체크리스트

- [ ] 공통 설정 완료 (SIMULATION_PC_COMMON_SETUP.md)
- [ ] 실행 스크립트 생성 완료
- [ ] VIM4 ping 성공
- [ ] SITL 실행 → VIM4 연결 확인
