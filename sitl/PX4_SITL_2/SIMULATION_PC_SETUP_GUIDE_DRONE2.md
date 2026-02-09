# 🖥️ PX4 SITL 시뮬레이션 PC 설정 가이드 (드론 2)

**작성일:** 2026-01-23  
**대상 PC:** Ubuntu 22.04 (192.168.100.6)  
**연결 대상:** VIM4 컴패니언 컴퓨터 드론2 (192.168.100.21)

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

## 📋 드론 2 설정 순서

1. SITL 실행 스크립트 생성
2. SITL 실행

---

## 1. 드론 2 SITL 실행 스크립트 생성

> **⚠️ 중요:** 드론 1이 **먼저 실행된 상태**에서 실행해야 합니다.

```bash
cat > ~/start_sitl_drone2.sh << 'EOF'
#!/bin/bash
# ==============================================================================
# PX4 SITL for VIM4 연결 (드론 2) - 드론 1 실행 후 사용
# ==============================================================================

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============ 설정 ============
VIM4_IP="192.168.100.21"
# ==============================

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE} PX4 SITL 시뮬레이션 시작 (드론 2)${NC}"
echo -e "${BLUE}======================================${NC}"
echo -e "  VIM4 IP: ${GREEN}$VIM4_IP${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# 드론 1이 실행 중인지 확인
echo -e "${YELLOW}[1/2] 드론 1 실행 확인...${NC}"
if pgrep -f "px4.*-i 0" > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} 드론 1 실행 중"
else
    echo -e "  ${RED}✗${NC} 드론 1이 실행되지 않았습니다!"
    echo -e "  ${YELLOW}→${NC} 먼저 ~/start_sitl_drone1.sh 를 실행하세요."
    exit 1
fi

# VIM4 연결 테스트
echo -e "${YELLOW}[2/2] VIM4 연결 테스트...${NC}"
if ping -c 1 -W 2 "$VIM4_IP" > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} VIM4 연결 가능: $VIM4_IP"
else
    echo -e "  ${RED}✗${NC} VIM4에 연결할 수 없습니다: $VIM4_IP"
    exit 1
fi

# PX4 SITL 시작
echo ""
echo -e "${BLUE}======================================${NC}"
echo -e "${GREEN} PX4 SITL 실행 중...${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

cd ~/PX4-Autopilot

PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="-10,0,0,0,0,0" \
PX4_HOME_LAT=35.905863 \
PX4_HOME_LON=128.802615 \
PX4_HOME_ALT=0 \
./build/px4_sitl_default/bin/px4 -i 1
EOF

chmod +x ~/start_sitl_drone2.sh
```

---

## 2. 드론 2 SITL 실행

> **전제조건:** 드론 1이 이미 실행 중이어야 합니다.

```bash
~/start_sitl_drone2.sh
```

> ✅ **rcS의 인스턴스 조건문에 의해 자동으로 VIM4 드론2에 연결됩니다!**

---

## 📊 통신 구조

```
시뮬레이션 PC (192.168.100.6)
    │
    │  PX4 SITL (드론 2, -i 1)
    │  mavlink start -o 18002 -t 192.168.100.21
    │
    ▼ UDP 18002
    
VIM4 드론2 (192.168.100.21)
    │
    │  MAVLink Router
    │  [SITL] :18002 (Server) ← 연결
    │  [GCS] → 192.168.100.255:14550
    │
    ▼ UDP 14550

QGC (브로드캐스트)
```

---

## 🔢 드론 2 설정 정보

| 항목 | 값 |
|------|-----|
| VIM4 IP | 192.168.100.21 |
| SITL 포트 | 18002 |
| MAVLink 로컬 포트 | 14541 |
| 인스턴스 | -i 1 |
| MAV_SYS_ID | 5 |

---

## ✅ 체크리스트

- [ ] 공통 설정 완료 (SIMULATION_PC_COMMON_SETUP.md)
- [ ] 실행 스크립트 생성 완료
- [ ] 드론 1 실행 중 확인
- [ ] VIM4 드론2 ping 성공
- [ ] SITL 실행 → VIM4 연결 확인
