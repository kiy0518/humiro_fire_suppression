# 027 - GUI 관리 도구 계획서

## 문서 정보
- **작성일**: 2026-01-16
- **버전**: 1.0.0
- **목적**: 프로젝트 설정, 상태 모니터링, 체크리스트를 GUI로 관리

---

## 1. 개요

프로젝트 규모가 커지면서 다음과 같은 문제가 발생:
- 설정 파일이 여러 곳에 분산
- 시스템 상태 확인이 어려움
- 체크리스트 관리가 수동
- 빌드/배포 과정이 복잡

**해결책**: 통합 GUI 관리 도구 개발

---

## 2. GUI 구성 요소

### 2.1 대시보드 (Dashboard)

| 항목 | 설명 | 우선순위 |
|------|------|----------|
| 시스템 상태 | FC 연결, ROS2, micro-ros-agent 상태 | 🔴 HIGH |
| 드론 정보 | DRONE_ID, IP 주소, 네트워크 상태 | 🔴 HIGH |
| 서비스 상태 | humiro-fire-suppression, mavlink-router 등 | 🔴 HIGH |
| 토픽 모니터 | 주요 ROS2 토픽 수신 상태 | 🟠 MEDIUM |
| 리소스 사용량 | CPU, 메모리, 디스크 | 🟡 LOW |

### 2.2 설정 관리 (Configuration)

| 항목 | 파일 위치 | 설명 |
|------|-----------|------|
| 드론 설정 | `/home/khadas/humiro_fire_suppression/config/device_config.env` | DRONE_ID, IP 등 |
| FastDDS 설정 | `/home/khadas/humiro_fire_suppression/config/fastrtps_profile.xml` | DDS 프로파일 |
| 미션 설정 | 코드 내 MissionConfig | 거리, 속도, 타임아웃 |
| 서비스 설정 | `/etc/systemd/system/*.service` | 자동 시작 서비스 |
| 네트워크 설정 | `/etc/netplan/*.yaml` | eth0, wlan0 설정 |

### 2.3 비행 모드 관리 (Flight Mode) 🆕

실내 테스트와 야외 비행을 구분하여 FC 파라미터를 자동 설정합니다.

#### 2.3.1 비행 모드 선택

| 모드 | 설명 | 주요 센서 |
|------|------|-----------|
| **실내 테스트** | 옵티컬 플로 + LiDAR 기반 위치 제어 | Optical Flow, LiDAR |
| **야외 비행** | GPS + RTK 기반 위치 제어 | GPS, RTK |

#### 2.3.2 실내 테스트 모드 파라미터

```
[Position Estimator]
EKF2_AID_MASK      = 26       # Optical Flow + Range Finder 사용
EKF2_HGT_MODE      = 2        # Range Sensor (LiDAR)
EKF2_RNG_AID       = 1        # Range Finder 보조 활성화

[Optical Flow]
EKF2_OF_CTRL       = 1        # Optical Flow 활성화
EKF2_OF_DELAY      = 20       # 지연 (ms)
EKF2_OF_QMIN       = 1        # 최소 품질
EKF2_OF_N_MIN      = 0.15     # 최소 노이즈
EKF2_OF_N_MAX      = 0.5      # 최대 노이즈

[Range Finder]
EKF2_RNG_CTRL      = 1        # Range Finder 활성화
EKF2_RNG_A_HMAX    = 5.0      # 최대 고도 (m)
EKF2_RNG_NOISE     = 0.1      # 노이즈 (m)
SENS_EN_RANGEFNDR  = 1        # Range Finder 센서 활성화

[GPS 비활성화]
EKF2_GPS_CTRL      = 0        # GPS 비활성화
GPS_1_CONFIG       = 0        # GPS 포트 비활성화
```

#### 2.3.3 야외 비행 모드 파라미터

```
[Position Estimator]
EKF2_AID_MASK      = 1        # GPS 사용
EKF2_HGT_MODE      = 1        # GPS (또는 0: Barometer)
EKF2_RNG_AID       = 0        # Range Finder 보조 비활성화

[GPS]
EKF2_GPS_CTRL      = 7        # GPS 활성화 (lon/lat/alt/vel)
GPS_1_CONFIG       = 201      # GPS UART 포트
EKF2_GPS_DELAY     = 110      # GPS 지연 (ms)

[Optical Flow 비활성화]
EKF2_OF_CTRL       = 0        # Optical Flow 비활성화

[RTK (선택)]
GPS_1_PROTOCOL     = 1        # uBlox
GPS_UBX_MODE       = 2        # RTK Float/Fixed
```

#### 2.3.4 파라미터 프리셋 저장

| 프리셋 이름 | 용도 | 파일 |
|------------|------|------|
| `indoor_optical_flow.params` | 실내 옵티컬 플로 | config/fc_params/ |
| `outdoor_gps.params` | 야외 GPS | config/fc_params/ |
| `outdoor_rtk.params` | 야외 RTK | config/fc_params/ |
| `custom.params` | 사용자 정의 | config/fc_params/ |

#### 2.3.5 비행 모드 화면 설계

```
┌─────────────────────────────────────────────────────────────────┐
│ Flight Mode                                                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Current Mode: [● Indoor Test]  [○ Outdoor GPS]  [○ Outdoor RTK]│
│                                                                 │
│  ─────────────────────────────────────────────────────────────  │
│  Mode Description:                                              │
│  실내 테스트 모드 - Optical Flow + LiDAR 기반 위치 제어        │
│  GPS 없이 실내에서 호버링 및 위치 제어 테스트 가능              │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│  Parameter Preview:                              [Show All ▼]   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ Parameter              Current    Target    Status      │   │
│  │ ─────────────────────────────────────────────────────── │   │
│  │ EKF2_AID_MASK          1          26        ⚠ Mismatch │   │
│  │ EKF2_HGT_MODE          1          2         ⚠ Mismatch │   │
│  │ EKF2_OF_CTRL           0          1         ⚠ Mismatch │   │
│  │ EKF2_GPS_CTRL          7          0         ⚠ Mismatch │   │
│  │ EKF2_RNG_CTRL          0          1         ⚠ Mismatch │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  [Apply Parameters]  [Read from FC]  [Save as Preset]          │
│                                                                 │
│  ─────────────────────────────────────────────────────────────  │
│  ⚠ Warning: 파라미터 변경 후 FC 재부팅 필요                    │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### 2.3.6 실내/야외 체크리스트 자동 전환

**실내 테스트 체크리스트:**
```
[ ] Optical Flow 센서 연결
[ ] LiDAR 거리 측정 정상
[ ] EKF2 상태 확인 (position valid)
[ ] 바닥 텍스처 충분 (optical flow용)
[ ] 조명 충분
[ ] GPS 비활성화 확인
```

**야외 비행 체크리스트:**
```
[ ] GPS 위성 수 (최소 6개)
[ ] HDOP < 2.0
[ ] GPS Fix 타입 (3D Fix 이상)
[ ] RTK Status (Fixed/Float) - RTK 모드 시
[ ] 지오펜스 설정 확인
[ ] 복귀 지점 설정 확인
```

### 2.4 기체별 자동 설정 (Vehicle Setup) 🆕

각 기체(1번~3번)를 선택하면 관련 설정이 자동으로 적용되고 시스템 재부팅까지 수행합니다.

#### 2.4.1 기체별 설정 값

| 설정 항목 | 1번 기체 (Leader) | 2번 기체 (Follower) | 3번 기체 (Follower) |
|-----------|------------------|---------------------|---------------------|
| **DRONE_ID** | 1 | 2 | 3 |
| **역할** | Leader | Follower (좌측) | Follower (우측) |
| **ETH0_IP (VIM4)** | 10.0.0.11 | 10.0.0.21 | 10.0.0.31 |
| **FC_IP** | 10.0.0.12 | 10.0.0.22 | 10.0.0.32 |
| **WIFI_IP** | 192.168.100.11 | 192.168.100.21 | 192.168.100.31 |
| **ROS_NAMESPACE** | drone1 | drone2 | drone3 |
| **MAV_SYS_ID** | 1 | 2 | 3 |
| **RTSP URL** | rtsp://IP:8554/humiro1 | rtsp://IP:8554/humiro2 | rtsp://IP:8554/humiro3 |

#### 2.4.2 자동 설정 파일 목록

| 파일 | 변경 내용 |
|------|-----------|
| `/home/khadas/humiro_fire_suppression/config/device_config.env` | DRONE_ID, IP 주소 |
| `/etc/netplan/01-eth0-px4.yaml` | ETH0_IP |
| `/etc/netplan/02-wlan0.yaml` | WIFI_IP (선택) |
| `/etc/systemd/system/micro-ros-agent.service.d/override.conf` | ROS_NAMESPACE |
| FC 파라미터 (MAVLink) | MAV_SYS_ID, MAV_COMP_ID |

#### 2.4.3 자동 설정 시퀀스

```
┌─────────────────────────────────────────────────────────────────┐
│ Vehicle Setup Wizard                                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Step 1: 기체 선택                                              │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│     ┌─────────┐    ┌─────────┐    ┌─────────┐                  │
│     │   1번   │    │   2번   │    │   3번   │                  │
│     │ Leader  │    │Follower │    │Follower │                  │
│     │  (중앙) │    │  (좌측) │    │  (우측) │                  │
│     │  [●]    │    │  [○]    │    │  [○]    │                  │
│     └─────────┘    └─────────┘    └─────────┘                  │
│                                                                 │
│  Step 2: 비행 모드 선택                                         │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│     [● 실내 테스트]    [○ 야외 GPS]    [○ 야외 RTK]            │
│                                                                 │
│  Step 3: 설정 확인                                              │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│  변경될 설정:                                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ DRONE_ID:      1                                        │   │
│  │ ETH0_IP:       10.0.0.11                               │   │
│  │ FC_IP:         10.0.0.12                               │   │
│  │ ROS_NAMESPACE: drone1                                   │   │
│  │ MAV_SYS_ID:    1                                        │   │
│  │ Flight Mode:   Indoor (Optical Flow)                    │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  [< 이전]                              [적용 및 재부팅 >]       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### 2.4.4 자동 재부팅 시퀀스

```
적용 및 재부팅 버튼 클릭 시:

1. 설정 파일 백업
   └─ /config/backup/YYYYMMDD_HHMMSS/

2. VIM4 설정 적용
   ├─ device_config.env 수정
   ├─ netplan 설정 수정
   └─ micro-ros-agent override.conf 수정

3. FC 파라미터 적용 (MAVLink)
   ├─ param set MAV_SYS_ID {id}
   ├─ param set EKF2_* (비행 모드별)
   └─ param save

4. FC 재부팅 (MAVLink)
   └─ MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

5. VIM4 재부팅
   └─ sudo reboot

총 소요 시간: 약 60초
```

#### 2.4.5 재부팅 진행 화면

```
┌─────────────────────────────────────────────────────────────────┐
│ Applying Configuration...                                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Progress: [████████████░░░░░░░░░░░░░░░░░░] 40%                │
│                                                                 │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│  [✓] Step 1: 설정 파일 백업 완료                               │
│  [✓] Step 2: VIM4 설정 적용 완료                               │
│  [▶] Step 3: FC 파라미터 적용 중...                            │
│      - MAV_SYS_ID = 1 ✓                                        │
│      - EKF2_AID_MASK = 26 ✓                                    │
│      - EKF2_OF_CTRL = 1 ...                                    │
│  [ ] Step 4: FC 재부팅 대기                                     │
│  [ ] Step 5: VIM4 재부팅                                        │
│                                                                 │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│  ⚠ 재부팅 중에는 전원을 끄지 마세요!                           │
│                                                                 │
│  예상 완료 시간: 45초 후                                        │
│                                                                 │
│                                              [Cancel (Emergency)]│
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### 2.4.6 재부팅 후 검증

```
VIM4 재부팅 후 자동 검증:

1. 네트워크 연결 확인
   ├─ eth0 IP 확인
   └─ FC ping 테스트

2. 서비스 상태 확인
   ├─ micro-ros-agent 실행
   └─ mavlink-router 실행

3. FC 연결 확인
   ├─ MAV_SYS_ID 확인
   └─ 파라미터 검증

4. 결과 표시
   └─ 성공/실패 리포트
```

#### 2.4.7 빠른 전환 버튼 (대시보드)

```
┌─────────────────────────────────────────────────────────────────┐
│ Quick Vehicle Switch                                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Current: 1번 기체 (Leader) - 실내 테스트                       │
│                                                                 │
│  [1번 실내] [1번 야외] [2번 실내] [2번 야외] [3번 실내] [3번 야외]│
│                                                                 │
│  ⚠ 전환 시 FC + VIM4 재부팅 (약 60초 소요)                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 2.3 체크리스트 (Checklist)

#### 2.3.1 비행 전 체크리스트
```
[ ] FC 전원 ON
[ ] FC 핑 테스트 (10.0.0.12)
[ ] micro-ros-agent 실행 확인
[ ] ROS2 토픽 수신 확인
    [ ] /fmu/out/vehicle_status
    [ ] /fmu/out/battery_status
    [ ] /fmu/out/vehicle_gps_position
[ ] QGC 연결 확인
[ ] GPS 위성 수 (최소 6개)
[ ] 배터리 잔량 (최소 80%)
[ ] 카메라 스트리밍 확인
[ ] LiDAR 거리 측정 확인
```

#### 2.3.2 시스템 점검 체크리스트
```
[ ] 서비스 상태 확인
    [ ] micro-ros-agent.service
    [ ] mavlink-router.service
    [ ] humiro-fire-suppression.service (선택)
[ ] 네트워크 연결
    [ ] eth0 (FC 연결)
    [ ] wlan0 (WiFi/QGC)
[ ] 센서 상태
    [ ] RGB 카메라
    [ ] 열화상 카메라
    [ ] LiDAR
```

#### 2.3.3 빌드 체크리스트
```
[ ] 라이브러리 빌드
    [ ] thermal_lib
    [ ] osd_lib
    [ ] targeting_lib
    [ ] streaming_lib
[ ] 메인 애플리케이션 빌드
[ ] 빌드 오류 없음
[ ] 서비스 재시작 (자동실행 시)
```

### 2.4 빌드 & 배포 (Build & Deploy)

| 기능 | 설명 |
|------|------|
| 원클릭 빌드 | 모든 라이브러리 + 메인 앱 빌드 |
| 클린 빌드 | build 폴더 삭제 후 재빌드 |
| 서비스 재시작 | systemctl restart |
| 로그 뷰어 | journalctl 실시간 출력 |
| Git 상태 | 변경사항, 커밋, 태그 |

### 2.5 모니터링 (Monitoring)

| 항목 | 데이터 소스 |
|------|-------------|
| 비행 모드 | /fmu/out/vehicle_status |
| 시동 상태 | /fmu/out/vehicle_status |
| 배터리 | /fmu/out/battery_status |
| GPS | /fmu/out/vehicle_gps_position |
| LiDAR 거리 | /lidar/front_distance |
| 열화상 핫스팟 | /thermal/hotspot |
| 미션 상태 | /offboard/status |

### 2.6 진행률 추적 (Progress Tracker)

| 항목 | 설명 |
|------|------|
| 전체 진행률 | work-plan 기준 완료율 |
| 모듈별 상태 | 각 모듈 구현 상태 |
| 미구현 항목 | GAP Analysis 기반 |
| 다음 할 일 | 우선순위별 작업 목록 |

---

## 3. 기술 스택 제안

### Option A: Python + PyQt5/PySide6
**장점**:
- ROS2 Python 바인딩 사용 가능
- 빠른 개발
- 크로스 플랫폼

**단점**:
- 리소스 사용량 높음

### Option B: Python + Tkinter
**장점**:
- 가벼움
- 기본 설치됨
- 단순함

**단점**:
- 디자인 제한

### Option C: Web 기반 (Flask + HTML/JS)
**장점**:
- 원격 접속 가능
- 반응형 UI
- 브라우저만 있으면 됨

**단점**:
- 네트워크 필요

### 권장: **Option A (PyQt5)** 또는 **Option C (Web)**

---

## 4. 화면 설계

### 4.1 메인 화면 레이아웃

```
┌─────────────────────────────────────────────────────────────────┐
│  Humiro Fire Suppression - Management Tool              [─][□][X]│
├─────────────────────────────────────────────────────────────────┤
│ [Dashboard] [Config] [Checklist] [Build] [Monitor] [Progress]   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │ System Status   │  │ Drone Info      │  │ Network         │ │
│  │                 │  │                 │  │                 │ │
│  │ FC: ● Connected │  │ ID: 1 (Leader)  │  │ eth0:  ● OK     │ │
│  │ ROS2: ● Running │  │ IP: 10.0.0.11   │  │ wlan0: ● OK     │ │
│  │ Agent: ● Active │  │ FC: 10.0.0.12   │  │ QGC:   ● OK     │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ ROS2 Topics                                              │   │
│  │                                                          │   │
│  │ /fmu/out/vehicle_status      ● Receiving (10 Hz)        │   │
│  │ /fmu/out/battery_status      ● Receiving (1 Hz)         │   │
│  │ /fmu/out/vehicle_gps_position ● Receiving (5 Hz)        │   │
│  │ /lidar/front_distance        ● Receiving (10 Hz)        │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ Quick Actions                                            │   │
│  │                                                          │   │
│  │ [Build All] [Restart Service] [View Logs] [Git Status]  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│ Status: Ready                                    v0.8.3 | 16:30 │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 체크리스트 화면

```
┌─────────────────────────────────────────────────────────────────┐
│ Checklist                                                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Pre-Flight Checklist                              [Auto Check] │
│  ─────────────────────────────────────────────────────────────  │
│  [✓] FC Power ON                                                │
│  [✓] FC Ping Test (10.0.0.12)                    Response: 0.5ms│
│  [✓] micro-ros-agent Running                     PID: 1021     │
│  [✓] ROS2 Topics Receiving                                      │
│      [✓] /fmu/out/vehicle_status                 10 Hz         │
│      [✓] /fmu/out/battery_status                 1 Hz          │
│      [✓] /fmu/out/vehicle_gps_position           5 Hz          │
│  [✓] QGC Connected                                              │
│  [✓] GPS Satellites: 12                          Min: 6        │
│  [✓] Battery: 95%                                Min: 80%      │
│  [✓] Camera Streaming                            30 FPS        │
│  [✓] LiDAR Distance                              2.18m         │
│                                                                 │
│  ─────────────────────────────────────────────────────────────  │
│  Result: 10/10 Passed                            [Ready to Fly] │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 4.3 설정 화면

```
┌─────────────────────────────────────────────────────────────────┐
│ Configuration                                                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Device Config (/config/device_config.env)         [Save] [Reset]│
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│  DRONE_ID:        [1    ▼]     (1: Leader, 2-3: Follower)      │
│  ETH0_IP:         [10.0.0.11 ]                                  │
│  FC_IP:           [10.0.0.12 ]                                  │
│  XRCE_DDS_PORT:   [8888      ]                                  │
│                                                                 │
│  Mission Config                                    [Save] [Reset]│
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│  Target Distance:    [10.0  ] m    (±5% tolerance)              │
│  Forward Speed:      [0.4   ] m/s                               │
│  Backward Speed:     [0.25  ] m/s                               │
│  Takeoff Altitude:   [5.0   ] m                                 │
│  RTL Altitude:       [10.0  ] m                                 │
│  Fire Interval:      [5.0   ] sec                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. 구현 계획

### Phase 1: 기본 프레임워크 (1일)
- [ ] PyQt5/Web 기본 구조
- [ ] 메인 윈도우 레이아웃
- [ ] 탭 네비게이션

### Phase 2: 대시보드 (1일)
- [ ] 시스템 상태 표시
- [ ] 네트워크 상태
- [ ] 서비스 상태 (systemctl)

### Phase 3: 체크리스트 (1일)
- [ ] 비행 전 체크리스트 UI
- [ ] 실내/야외 모드별 체크리스트 자동 전환
- [ ] 자동 체크 기능 (ping, topic 확인)
- [ ] 결과 표시

### Phase 4: 설정 관리 (1일)
- [ ] device_config.env 편집
- [ ] 미션 파라미터 편집
- [ ] 저장/로드

### Phase 5: 비행 모드 관리 (1.5일) 🆕
- [ ] 실내/야외 모드 선택 UI
- [ ] FC 파라미터 프리셋 관리
- [ ] 파라미터 읽기/쓰기 (MAVLink)
- [ ] 현재 파라미터 vs 목표 파라미터 비교

### Phase 6: 기체별 자동 설정 (2일) 🆕
- [ ] 기체 선택 위저드 UI
- [ ] 설정 파일 자동 수정
  - [ ] device_config.env
  - [ ] netplan (eth0, wlan0)
  - [ ] micro-ros-agent override.conf
- [ ] FC 파라미터 자동 적용 (MAVLink param set)
- [ ] FC 재부팅 명령 (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
- [ ] VIM4 재부팅 (sudo reboot)
- [ ] 재부팅 후 자동 검증
- [ ] 빠른 전환 버튼

### Phase 7: 빌드 & 모니터링 (1일)
- [ ] 원클릭 빌드
- [ ] 로그 뷰어
- [ ] ROS2 토픽 모니터

### Phase 8: 진행률 추적 (0.5일)
- [ ] work-plan 기반 진행률
- [ ] 미구현 항목 목록

---

## 5.1 통신 두절 시 안전 동작 (Failsafe) 🆕

### 현재 통신 구조

```
┌─────────────┐     WiFi      ┌─────────────┐    eth0     ┌─────────────┐
│     QGC     │◄────────────►│    VIM4     │◄───────────►│   FC (PX4)  │
│  (GCS PC)   │   MAVLink     │ (Companion) │  uXRCE-DDS  │  (Autopilot)│
└─────────────┘               └─────────────┘             └─────────────┘
                                    │
                              mavlink-router
                              (UDP 14550)
```

### 통신 두절 시나리오 및 동작

| 시나리오 | 두절 구간 | FC 동작 | 결과 |
|----------|-----------|---------|------|
| **WiFi 끊김** | QGC ↔ VIM4 | **영향 없음** | FC는 독립적으로 비행 |
| **VIM4 전원 끊김** | VIM4 ↔ FC | **페일세이프 작동** | PX4 COM_OBL_* 파라미터에 따름 |
| **eth0 끊김** | VIM4 ↔ FC | **페일세이프 작동** | PX4 COM_OBL_* 파라미터에 따름 |

### 중요: FC는 독립적으로 비행 가능

**WiFi 통신(QGC↔VIM4)이 끊어져도 FC는 추락하지 않습니다!**

- FC(PX4)는 VIM4와 별개로 자체 센서(GPS, IMU, 기압계)로 비행 제어
- QGC와의 통신은 모니터링 및 명령 전달용
- WiFi 끊김 = QGC에서 상태만 안 보임

### 실제 위험: OFFBOARD 모드에서 VIM4 통신 두절

OFFBOARD 모드에서 VIM4가 명령을 보내지 못하면:

```
PX4 OFFBOARD 타임아웃 동작:
1. 0.5초 동안 setpoint 미수신
2. COM_OBL_RC_ACT 파라미터에 따라 동작
   - 0: Position 모드로 전환 (호버링)
   - 1: Altitude 모드로 전환
   - 2: Manual 모드로 전환
   - 3: Return 모드로 전환 (RTL)
   - 4: Land 모드로 전환
   - 5: Hold 모드로 전환 (Loiter)
```

### FC 페일세이프 파라미터 (PX4)

```
# OFFBOARD 모드 페일세이프
COM_OBL_RC_ACT    = 5    # OFFBOARD 두절 시 → Hold (Loiter)
COM_OF_LOSS_T     = 1.0  # OFFBOARD 두절 판정 시간 (초)

# RC 페일세이프
COM_RC_LOSS_T     = 0.5  # RC 두절 판정 시간 (초)
NAV_RCL_ACT       = 2    # RC 두절 시 → RTL

# 데이터링크 페일세이프
NAV_DLL_ACT       = 0    # 데이터링크 두절 시 → 동작 없음
COM_DL_LOSS_T     = 10   # 데이터링크 두절 판정 시간 (초)

# 배터리 페일세이프
COM_LOW_BAT_ACT   = 3    # 저전압 시 → RTL
BAT_LOW_THR       = 0.15 # 15% 이하 경고
BAT_CRIT_THR      = 0.07 # 7% 이하 긴급 착륙
BAT_EMERGEN_THR   = 0.05 # 5% 이하 즉시 착륙

# GPS 페일세이프
COM_POS_FS_DELAY  = 1    # 위치 손실 판정 시간 (초)
COM_POSCTL_NAVL   = 0    # 위치 손실 시 → Altitude 모드
```

### GUI에 추가할 페일세이프 설정 화면

```
┌─────────────────────────────────────────────────────────────────┐
│ Failsafe Settings                                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  OFFBOARD 모드 페일세이프                       [Read] [Apply]  │
│  ─────────────────────────────────────────────────────────────  │
│  OFFBOARD 두절 시 동작:  [Hold (Loiter)  ▼]                    │
│  두절 판정 시간:         [1.0 ] 초                              │
│                                                                 │
│  RC 페일세이프                                                  │
│  ─────────────────────────────────────────────────────────────  │
│  RC 두절 시 동작:        [RTL           ▼]                     │
│  두절 판정 시간:         [0.5 ] 초                              │
│                                                                 │
│  배터리 페일세이프                                              │
│  ─────────────────────────────────────────────────────────────  │
│  저전압 동작:            [RTL           ▼]                     │
│  경고 임계값:            [15  ] %                               │
│  긴급 착륙 임계값:       [7   ] %                               │
│                                                                 │
│  ─────────────────────────────────────────────────────────────  │
│  💡 권장 설정:                                                  │
│  - 실내 테스트: OFFBOARD 두절 → Land (착륙)                    │
│  - 야외 비행:   OFFBOARD 두절 → RTL (복귀)                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 권장 페일세이프 설정

| 환경 | COM_OBL_RC_ACT | 설명 |
|------|----------------|------|
| **실내 테스트** | 4 (Land) | 그 자리에서 착륙 |
| **야외 비행** | 3 (RTL) | 출발점으로 복귀 후 착륙 |
| **개발 테스트** | 5 (Hold) | 제자리 호버링 (수동 개입 가능) |

---

## 6. 파일 구조

```
humiro_fire_suppression/
├── gui/
│   ├── main.py                    # 메인 진입점
│   ├── requirements.txt           # 의존성
│   ├── ui/
│   │   ├── main_window.py         # 메인 윈도우
│   │   ├── dashboard_tab.py       # 대시보드 탭
│   │   ├── config_tab.py          # 설정 탭
│   │   ├── checklist_tab.py       # 체크리스트 탭
│   │   ├── build_tab.py           # 빌드 탭
│   │   ├── monitor_tab.py         # 모니터링 탭
│   │   └── progress_tab.py        # 진행률 탭
│   ├── utils/
│   │   ├── system_check.py        # 시스템 체크 유틸
│   │   ├── ros2_monitor.py        # ROS2 모니터
│   │   ├── config_manager.py      # 설정 관리
│   │   └── build_manager.py       # 빌드 관리
│   └── resources/
│       ├── icons/
│       └── styles/
```

---

## 7. 예상 소요 시간

| 단계 | 소요 시간 |
|------|-----------|
| Phase 1: 기본 프레임워크 | 1일 |
| Phase 2: 대시보드 | 1일 |
| Phase 3: 체크리스트 | 1일 |
| Phase 4: 설정 관리 | 1일 |
| Phase 5: 비행 모드 관리 🆕 | 1.5일 |
| Phase 6: 기체별 자동 설정 🆕 | 2일 |
| Phase 7: 빌드 & 모니터링 | 1일 |
| Phase 8: 진행률 추적 | 0.5일 |
| **총 예상** | **9일** |

---

## 8. 우선순위

### 🔴 필수 (MVP)
1. 시스템 상태 대시보드
2. 비행 전 체크리스트
3. 설정 파일 편집

### 🟠 권장
4. 원클릭 빌드
5. ROS2 토픽 모니터
6. 로그 뷰어

### 🟡 선택
7. 진행률 추적
8. Git 통합
9. 원격 접속 (Web)

---

## 9. 결론

GUI 관리 도구를 통해:
- 설정 관리 일원화
- 비행 전 체크 자동화
- 시스템 상태 실시간 모니터링
- 개발/배포 효율성 향상

**권장 시작점**: Python + PyQt5로 MVP 구현 후 필요시 Web으로 확장

---

## 10. 문서 이력

| 버전 | 날짜 | 작성자 | 변경 내용 |
|------|------|--------|-----------|
| 1.0.0 | 2026-01-16 | Claude | 최초 작성 |

---

*다음 단계: 사용자 확인 후 구현 시작*
