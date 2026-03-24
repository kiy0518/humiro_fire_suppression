# 035. DDS 교차오염 수정 — fc_bridge WiFi DDS 격리

**날짜**: 2026-03-24
**상태**: 완료
**관련 버전**: v0.26.12 이후

---

## 문제

멀티 드론 운용 시 드론 #2, #3의 OSD에 서로의 FC 데이터(roll, pitch, GPS, 배터리)가 번갈아 표시되는 현상 발견.

### 증상
- 드론 #3은 피치 방향으로 기울여 놓았는데 (pitch ≈ -4.7°), 드론 #2의 OSD에도 이 값이 간헐적으로 나타남
- 드론 #2의 OSD에도 자기 값과 드론 #3의 값이 번갈아 표시
- GPS 위성 수, 배터리 전압 등도 동일하게 교차
- 드론 #1은 비교적 안정적 (but 실제로는 3개 publisher가 보임)

### 위험도: 매우 높음
- OSD 교차 표시는 시각적 문제에 불과하지만
- **비행 제어 명령(trajectory_setpoint, vehicle_command)도 다른 드론 FC에 전달 가능**
- 편대비행 중 한 드론의 제어 명령이 다른 드론의 FC에 도달하면 충돌 위험

---

## 근본 원인

### `start_fc_bridge.sh` 97-104행 (수정 전)

```bash
# DDS 프로파일: micro-ros-agent와 동일하게 eth0_only 사용
# (micro-ros-agent가 실제로 eth0에서 DDS 발행하므로 fc_bridge도 eth0 필요)
FINAL_DDS_PROFILE="$PROJECT_ROOT/config/fastdds_eth0_only.xml"
```

이 코드가 앞서 SITL/FC 모드에 따라 설정한 DDS 프로파일을 **무조건 덮어씀**.

### fastdds_eth0_only.xml 내용

```xml
<interfaceWhiteList>
    <address>10.0.0.21</address>       <!-- eth0 -->
    <address>192.168.100.21</address>  <!-- WiFi -->
    <address>127.0.0.1</address>       <!-- loopback -->
</interfaceWhiteList>
```

WiFi 인터페이스가 포함되어 있어 fc_bridge가 WiFi DDS 멀티캐스트에 참여.

### 교차 메커니즘

```
드론 #2 fc_bridge (WiFi DDS, Domain 0)
         ↕ WiFi 239.255.0.1:7400 멀티캐스트
드론 #3 fc_bridge (WiFi DDS, Domain 0)

→ fc_bridge가 상대 드론의 micro-ros-agent가 발행한 /fmu/out/* 토픽을 구독
→ fc_bridge_server가 양쪽 FC 데이터를 번갈아 UDP 17001로 전송
→ OSD에 교차 데이터 표시
```

### 진단 증거

**수정 전 ros2 topic info (드론 #2에서):**
```
/fmu/out/vehicle_attitude:
  Publisher count: 2    ← 자기 + 드론 #3의 micro-ros-agent
  Subscription count: 2 ← 자기 + 드론 #3의 fc_bridge
```

**WiFi DDS 트래픽 (tcpdump):**
```
192.168.100.21 ↔ 192.168.100.31 port 7411 — 대량의 Domain 0 DDS 데이터 교환
```

---

## 수정

### 1. `start_fc_bridge.sh` — DDS 프로파일 변경

```bash
# 수정 전: WiFi 포함
FINAL_DDS_PROFILE="$PROJECT_ROOT/config/fastdds_eth0_only.xml"

# 수정 후: WiFi 제외 (eth0 + loopback만)
FINAL_DDS_PROFILE="$PROJECT_ROOT/config/fastdds_agent_eth0.xml"
```

**왜 loopback_only가 아닌 agent_eth0인가:**
- loopback 인터페이스에는 MULTICAST 플래그가 없음 (`<LOOPBACK,UP,LOWER_UP>`)
- FastDDS 멀티캐스트 발견이 loopback에서 동작하지 않음
- eth0는 MULTICAST 지원 + 각 드론의 FC에만 직결 → 교차 위험 없음

### 2. `003-apply_config.sh` — .bashrc 전역 DDS 프로파일 제거

```bash
# 수정 전: .bashrc에 eth0_only 삽입
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/humiro_fire_suppression/config/fastdds_eth0_only.xml

# 수정 후: 주석으로 변경
# FASTRTPS_DEFAULT_PROFILES_FILE은 각 서비스 스크립트에서 설정 (start_fc_bridge.sh 등)
```

.bashrc에서 전역 설정하면 ros2-daemon 등이 WiFi DDS에 참여하여 불필요한 교차 트래픽 발생.

### 3. 드론 #2, #3 `.bashrc` 수동 수정

동일하게 `FASTRTPS_DEFAULT_PROFILES_FILE` 주석 처리.

---

## DDS 프로파일 정리

| 프로파일 | 인터페이스 | 사용 프로세스 | 용도 |
|---------|-----------|-------------|------|
| `fastdds_loopback_only.xml` | 127.0.0.1 | micro-ros-agent (FC모드) | FC DDS 토픽 loopback 제한 |
| `fastdds_agent_eth0.xml` | eth0 + 127.0.0.1 | fc_bridge, micro-ros-agent (SITL) | DDS 발견용 eth0 + loopback |
| `fastdds_eth0_only.xml` | eth0 + WiFi + 127.0.0.1 | **사용 금지** (교차오염) | — |
| `fastdds_wifi_only.xml` | WiFi | 메인앱 (Domain 1) | 편대 통신 |

---

## 검증 결과

### 수정 후 fc_bridge 환경변수
```
FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_agent_eth0.xml  (WiFi 없음)
```

### 수정 후 ros2 topic info (드론 #2)
```
/fmu/out/vehicle_attitude:
  Publisher count: 1    ← 자기 micro-ros-agent만
  Subscription count: 0 ← fc_bridge만 (loopback 발견 후)
```

### 수정 후 FC 데이터 (각 드론 독립)
| 드론 | roll | pitch | 배터리 | fc_connected |
|------|------|-------|--------|-------------|
| #1 | -0.004 | -0.009 | 23.9V/77% | 1 |
| #2 | -0.031 | -0.035 | 24.2V/82% | 1 |
| #3 | -0.010 | -0.087 | 23.9V/77% | 1 |

드론 #3의 pitch=-0.087 (≈ -5°) — 기울여 놓은 기체 값만 표시 (교차 없음).

---

## 수정 파일

- `scripts/runtime/start_fc_bridge.sh` — DDS 프로파일을 fastdds_agent_eth0.xml로 변경
- `scripts/install/003-apply_config.sh` — .bashrc에 FASTRTPS 전역 설정 삽입 제거
- 드론 #2, #3 `~/.bashrc` — FASTRTPS 전역 설정 주석 처리

---

## 주의사항

1. **`003-apply_config.sh` 재실행 시**: .bashrc에 FASTRTPS 전역 설정이 더 이상 삽입되지 않음. 정상.
2. **새 드론 배포 시**: `fastdds_eth0_only.xml`은 사용하지 않도록 주의. fc_bridge는 반드시 `fastdds_agent_eth0.xml` 사용.
3. **eth0 공유 네트워크**: 만약 드론 간 eth0가 스위치로 연결될 경우, eth0에서도 교차 가능. 현재는 각 드론의 eth0가 FC에만 직접 연결되어 안전.
4. **loopback MULTICAST 미지원**: Linux loopback(lo)에는 기본적으로 MULTICAST 플래그 없음. FastDDS 멀티캐스트 발견이 loopback만으로 동작하지 않으므로, 반드시 eth0 포함 필요.
