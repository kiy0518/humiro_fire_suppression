# 032. uXRCE-DDS 통신 경로 선택 (eth0 UDP / UART)

## 개요

PX4 FC와 VIM4 간 uXRCE-DDS 통신 경로를 eth0(UDP)에서 UART(시리얼)로도 선택 가능하게 변경.
웹 GUI에서 통신 방식을 선택하며, SITL 모드 전환 시에도 정상 동작.

## 현재 구조

```
[PX4 FC] --eth0(UDP)--> [VIM4 micro_ros_agent udp4 --port 8888]
                              ↓
                         [fc_bridge (loopback IPC)]
                              ↓
                         [application (Domain 1)]
```

- FC측: `uxrce_dds_client start -t udp -h 10.0.0.11 -p 8888`
- VIM4측: `micro_ros_agent udp4 --port 8888`
- SITL측: `uxrce_dds_client start -t udp -h 192.168.100.x -p 8888`

## 변경 목표

```
[PX4 FC] --UART(serial)--> [VIM4 micro_ros_agent serial --dev /dev/ttyXXX -b 921600]
    또는
[PX4 FC] --eth0(UDP)------> [VIM4 micro_ros_agent udp4 --port 8888]
```

- FC모드 UDP: 현재와 동일
- FC모드 UART: `micro_ros_agent serial --dev /dev/ttyXXX -b 921600`
- SITL모드: 항상 UDP (시뮬레이션은 네트워크 기반이므로 UART 불가)

## 변경 대상 파일

### 1. config/device_config.env

**추가 변수**:
```bash
XRCE_DDS_TRANSPORT=udp           # udp 또는 serial
XRCE_DDS_SERIAL_DEV=/dev/ttyUSB0 # UART 디바이스 경로
XRCE_DDS_SERIAL_BAUD=921600      # UART 보레이트
XRCE_DDS_PORT=8888               # 기존 (UDP 모드용)
```

### 2. scripts/runtime/start_micro_ros_agent_wrapper.sh (핵심)

**현재** (line 56):
```bash
exec "$AGENT" udp4 --port 8888
```

**변경**:
```bash
# device_config.env에서 통신 방식 로드
source "$PROJECT_ROOT/config/device_config.env"

# SITL 모드면 항상 UDP (시뮬레이션은 네트워크 기반)
if [ "$IS_SITL" = "true" ]; then
    echo "  [SITL] UDP 모드 강제: udp4 --port ${XRCE_DDS_PORT:-8888}"
    exec "$AGENT" udp4 --port ${XRCE_DDS_PORT:-8888}
elif [ "$XRCE_DDS_TRANSPORT" = "serial" ]; then
    SERIAL_DEV="${XRCE_DDS_SERIAL_DEV:-/dev/ttyUSB0}"
    SERIAL_BAUD="${XRCE_DDS_SERIAL_BAUD:-921600}"
    echo "  [FC] UART 모드: serial --dev $SERIAL_DEV -b $SERIAL_BAUD"
    exec "$AGENT" serial --dev "$SERIAL_DEV" -b "$SERIAL_BAUD"
else
    echo "  [FC] UDP 모드: udp4 --port ${XRCE_DDS_PORT:-8888}"
    exec "$AGENT" udp4 --port ${XRCE_DDS_PORT:-8888}
fi
```

- `IS_SITL` 판별: 기존 mavlink-router 설정 파일 체크 로직 재활용 (이미 스크립트에 존재)

### 3. scripts/install/003-apply_config.sh

**템플릿에 추가**:
- XRCE_DDS_TRANSPORT 기본값 `udp`
- XRCE_DDS_SERIAL_DEV 기본값 `/dev/ttyUSB0`
- XRCE_DDS_SERIAL_BAUD 기본값 `921600`

### 4. gui/app.py

**상태 API 수정** (`api_micro_ros_status`, line 2183~2212):
- UDP 모드: 기존 포트 8888 체크
- UART 모드: 프로세스 실행 여부 + 시리얼 디바이스 존재 확인 (`/dev/ttyXXX` 존재 여부)

**SITL/FC 전환 로직** (`api_router_set_sitl_mode`, line 2040~2160):
- 포트 8888 체크: UART 모드에서는 UDP 포트가 없으므로 건너뛰기
- SITL 전환 시: UART→UDP 자동 전환 (wrapper 스크립트 내부에서 처리)
- FC 전환 시: device_config.env 설정대로 동작 (wrapper 스크립트가 판단)

**iptables 규칙** (line 2001~2040):
- UART 모드에서는 DDS UDP 포트 격리 불필요 (시리얼 직접 연결)
- FC 모드 + UART: iptables DDS 관련 규칙 생략

**설정 저장 API 추가**:
- `/api/config/xrce-dds` POST: XRCE_DDS_TRANSPORT, SERIAL_DEV, SERIAL_BAUD 저장
- device_config.env에 반영

### 5. gui/utils/config_manager.py

- `XRCE_DDS_TRANSPORT`, `XRCE_DDS_SERIAL_DEV`, `XRCE_DDS_SERIAL_BAUD` 읽기/쓰기 추가

### 6. gui/templates/micro_ros.html (또는 config.html)

**통신 방식 선택 UI**:
- uXRCE-DDS 통신: UDP / UART 라디오 버튼 (또는 드롭다운)
- UART 선택 시: 시리얼 포트 경로, 보레이트 입력 필드 표시
- 저장 버튼 → device_config.env 반영 + micro-ros-agent 재시작

**상태 표시**:
- 현재 통신 방식 표시 (UDP port:8888 / UART /dev/ttyUSB0 921600)
- 연결 상태 표시 (UDP: 포트 리스닝 / UART: 프로세스 실행 + 디바이스 존재)

## 변경하지 않는 파일

| 파일 | 이유 |
|------|------|
| fc_bridge 관련 | 로컬 UDP IPC(127.0.0.1:17001/17002)이므로 영향 없음 |
| fastdds_*.xml | FastDDS는 DDS 미들웨어 레벨, UART는 MicroXRCE-DDS 전송 레벨이므로 무관 |
| SITL rcS 스크립트 | SITL은 항상 UDP, 변경 불필요 |
| offboard_manager, application_manager | 상위 레이어, 전송 계층과 무관 |

## 구현 순서

1. `config/device_config.env`에 3개 변수 추가 (기본값: udp)
2. `scripts/runtime/start_micro_ros_agent_wrapper.sh` 수정 (핵심 변경)
3. `scripts/install/003-apply_config.sh` 템플릿 업데이트
4. `gui/utils/config_manager.py` 설정 관리 추가
5. `gui/app.py` 상태 API + SITL/FC 전환 로직 수정 + 설정 저장 API
6. `gui/templates/micro_ros.html` 통신 방식 선택/표시 UI

## FC측 참고사항 (PX4 설정)

UART 모드 사용 시 FC측 PX4에서도 uXRCE-DDS 클라이언트를 UART로 설정해야 함:

```bash
# FC 콘솔에서 (nsh)
uxrce_dds_client start -t serial -d /dev/ttyS1 -b 921600
```

- PX4 파라미터 `UXRCE_DDS_CFG`를 해당 UART 포트로 설정
- VIM4의 시리얼 포트와 FC의 시리얼 포트를 물리적으로 연결 필요
- 이 부분은 코드 변경이 아닌 하드웨어/FC 설정이므로 본 작업 범위 외

## 검증 항목

1. **FC 모드 + UDP**: 기존과 동일하게 동작 (회귀 테스트)
2. **FC 모드 + UART**: `micro_ros_agent serial --dev /dev/ttyXXX` 실행 확인
3. **SITL 전환**: UART 설정이어도 SITL에서는 자동으로 UDP 사용 확인
4. **FC 전환**: UART 설정이면 UART로 복원 확인
5. **GUI 상태**: UDP/UART 현재 모드 정확히 표시 확인
6. **시리얼 디바이스 미존재**: 에러 메시지 표시 확인
7. **설정 저장**: GUI에서 저장 → device_config.env 반영 → 재시작 후 적용 확인
