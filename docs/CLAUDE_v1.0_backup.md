# CLAUDE.md

이 파일은 이 저장소에서 코드 작업 시 Claude Code (claude.ai/code)에게 제공되는 가이드입니다.

## 프로젝트 개요

**Humiro Fire Suppression Drone System** - PX4 비행 제어기를 사용하는 Khadas VIM4 보드에서 실행되는 다중 드론 협조를 위한 ROS2 기반 자율 화재 진압 시스템입니다.

**핵심 기술:**
- ROS2 Humble (미들웨어)
- PX4 Firmware v1.16.0 (비행 제어)
- Micro-ROS Agent (XRCE-DDS 브리지)
- OpenCV + GStreamer (열화상/RGB 처리)
- Python 3 + C++17
- systemd 서비스 관리

**시스템 아키텍처:**
```
PX4 Flight Controller
    ↓ MAVLink/XRCE-DDS
Khadas VIM4 (Ubuntu 22.04 ARM64)
    ├── Thermal Processing (C++, 실시간 스트리밍)
    ├── LiDAR Distance Measurement (ToF/Scanning, ROS2)
    ├── Targeting System (Python, 화재 감지 + 탄도 계산)
    ├── Throwing Mechanism (GPIO, 발사체 배치)
    └── Navigation (Python, RTK GPS, 편대 비행)
```

## 개발 언어 정책

**중요: C/C++ 우선 정책**

이 프로젝트는 모든 핵심 모듈에 대해 **C/C++**을 우선시하여 다음을 보장합니다:
- **실시간 성능** (30 FPS 열화상 스트리밍, 10 Hz LiDAR)
- **낮은 지연시간** (<100ms glass-to-glass)
- **임베디드 ARM 플랫폼에서의 효율적인 리소스 사용**
- **일관된 코드베이스** (기존 열화상 시스템 아키텍처와 일치)

**언어 선택 가이드라인:**

1. **항상 C/C++ 사용:**
   - 열화상 카메라 처리 (기존: 2,665 LOC C++)
   - LiDAR 인터페이스 및 거리 측정
   - 실시간 센서 데이터 처리
   - 이미지/비디오 처리 및 오버레이
   - 성능이 중요한 경로
   - 하드웨어 인터페이스 (GPIO, 시리얼 등)

2. **Python 허용:**
   - ROS2 노드 래퍼 (C++ 라이브러리 호출)
   - 고수준 조정 로직
   - 비실시간 작업 (로깅, 설정)
   - 테스트 및 프로토타이핑

3. **새 기능 구현 시:**
   - 명시적으로 정당화되지 않는 한 기본적으로 C++ 사용
   - 질문: "이것이 실시간 성능이 필요한가?" → C++ 사용
   - 질문: "이것이 하드웨어와 인터페이스하는가?" → C++ 사용
   - 질문: "이것이 기존 C++ 코드의 래퍼인가?" → Python 허용

**예제:**
- ✅ LiDAR 통합: C++ (실시간, 하드웨어 인터페이스)
- ✅ 거리 오버레이: C++ (실시간 이미지 처리)
- ✅ 열화상 스트리밍: C++ (기존, 성능 중요)
- ⚠️ 탄도 계산기: C++ 선호 (성능, C++ 모듈과의 통합)
- ⚠️ 서보 제어기: C++ (하드웨어 GPIO, 타이밍 중요)
- ✅ ROS2 토픽 발행 노드: Python 허용 (C++ 라이브러리 래퍼)
- ✅ 설정 로더: Python 허용 (비실시간)

**코드 표준:**
- C++: C++17 표준, Google C++ Style Guide
- Python: PEP 8 (ROS2 래퍼에만 해당)
- 빌드: C++는 CMake, Python은 setuptools

## 필수 명령어

### 환경 설정 (중요 - 항상 먼저!)

```bash
# 모든 개발 작업 전에 환경 변수 로드
source ~/humiro_fire_suppression/setup_env.sh

# 이것은 다음을 설정합니다: PROJECT_ROOT, MICRO_ROS_WS, PX4_ROS2_WS, PYTHONPATH 등
```

### 설치 및 설정

```bash
# 전체 시스템 설치 (처음)
cd ~/humiro_fire_suppression
sudo ./scripts/install/000-install_all.sh

# ROS2 워크스페이스 재빌드 (git clone 또는 디렉토리 이동 후)
./scripts/install/004-rebuild_workspaces.sh

# 설정 변경 적용
sudo ./scripts/install/003-apply_config.sh
```

### ROS2 워크스페이스 관리

```bash
# Micro-ROS 워크스페이스 재빌드
cd $MICRO_ROS_WS
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

# PX4 ROS2 워크스페이스 재빌드
cd $PX4_ROS2_WS
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

# 특정 패키지만 재빌드
cd $MICRO_ROS_WS
colcon build --packages-select micro_ros_agent
```

### 열화상 시스템 빌드 (C++)

```bash
cd ~/humiro_fire_suppression/thermal/src
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 출력 바이너리: ./thermal_rgb_streaming
# 실행: ./thermal_rgb_streaming
```

### 테스트 및 검증

```bash
# 완전한 설치 검증
./scripts/check/103-verify_installation.sh

# PX4 연결 확인 (네트워크 진단)
./scripts/check/101-check_px4_connection.sh

# ROS2 토픽 테스트
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status

# 실행 중인 노드 확인
ros2 node list
```

### 서비스 관리

```bash
# 서비스 상태 확인
sudo systemctl status micro-ros-agent.service
sudo systemctl status mavlink-router.service
sudo systemctl status dnsmasq-px4.service

# 서비스 재시작
sudo systemctl restart micro-ros-agent.service
sudo systemctl restart mavlink-router.service

# 서비스 로그 보기 (실시간)
sudo journalctl -u micro-ros-agent.service -f

# 마지막 50개 로그 항목 보기
sudo journalctl -u micro-ros-agent.service -n 50
```

### 네트워크 진단

```bash
# 네트워크 인터페이스 확인
ip addr show eth0
ip addr show wlan0

# ARP 테이블 확인 (PX4 IP 찾기)
ip neigh show dev eth0

# 연결 테스트
ping 10.0.0.12  # PX4 비행 제어기
```

## 아키텍처 및 코드 구조

### 디렉토리 구성

```
humiro_fire_suppression/
├── thermal/                    # 열화상 카메라 시스템
│   ├── src/                    # ✅ 완료 (2,665 LOC C++)
│   │   ├── main.cpp            # 멀티스레드 오케스트레이션
│   │   ├── camera_manager.cpp  # 카메라 I/O
│   │   ├── thermal_processor.cpp  # ✅ 핫스팟 감지, 온도 분석
│   │   ├── frame_compositor.cpp   # ✅ RGB+thermal 영상 정합
│   │   ├── rtsp_server.cpp     # GStreamer RTSP 스트리밍
│   │   └── http_server.cpp     # HTTP 웹 스트리밍
│   └── python/                 # 프로토타입 (12버전)
│
├── lidar/                      # 거리 측정 시스템
│   ├── src/                    # ✅ 완료 (1,188 LOC C++)
│   │   ├── lidar_config.h/cpp      # UART 설정 (USB/GPIO)
│   │   ├── lidar_interface.h/cpp   # LD19 LiDAR 통신
│   │   ├── distance_overlay.h/cpp  # 거리 시각화 오버레이
│   │   └── main_test.cpp           # 테스트 프로그램
│   └── README.md               # 거리 측정 가이드
│
├── targeting/                  # 핫스팟 트래킹 및 정조준 시스템
│   ├── hotspot_tracker         # ⏳ 핫스팟 추적 (Kalman Filter)
│   ├── drone_position_controller # ⏳ 드론 위치 제어 (PX4)
│   ├── targeting_manager       # ⏳ GCS 신호 처리 및 통합
│   ├── targeting_overlay       # ⏳ 트래킹 상태 오버레이
│   └── README.md               # 정조준 시스템 설계
│
├── throwing_mechanism/         # 발사 메커니즘 (미구현)
│   └── README.md               # 구현 계획 (고정 각도, 10m)
│
├── navigation/                 # 자율 비행 (미구현)
│   ├── formation_control/      # 다중 드론 편대
│   ├── rtk_positioning/        # RTK GPS
│   └── collision_avoidance/    # 장애물 회피
│
├── workspaces/                 # ROS2 워크스페이스 (Git 제외)
│   ├── micro_ros_ws/           # Micro-ROS Agent (XRCE-DDS)
│   ├── px4_ros2_ws/            # PX4 메시지 정의
│   └── mavlink-router/         # MAVLink 라우팅
│
├── common/                     # 공통 유틸리티
│   ├── utils/                  # 헬퍼 함수
│   ├── interfaces/             # ROS2 메시지/서비스 정의
│   ├── exceptions/             # 커스텀 예외
│   └── constants/              # 시스템 상수
│
├── scripts/                    # 자동화 스크립트
│   ├── install/                # 설치 스크립트
│   ├── check/                  # 검증 스크립트
│   ├── runtime/                # 서비스 래퍼
│   ├── update/                 # 업데이트 스크립트
│   └── maintenance/            # 유지보수 도구
│
├── config/                     # 설정 파일
│   ├── device_config.env       # 드론별 설정
│   ├── versions.env            # 버전 추적
│   └── network/                # 네트워크 템플릿
│
├── deployment/                 # 배포 설정
│   ├── systemd/                # 서비스 정의
│   ├── docker/                 # Docker 설정
│   └── backup/                 # 백업 스크립트
│
├── tests/                      # 테스트 스위트
│   ├── unit/                   # 단위 테스트
│   ├── integration/            # 통합 테스트
│   └── hardware/               # 하드웨어 테스트
│
├── work-plan/                  # 프로젝트 계획 문서
│   ├── PROJECT_MASTER_PLAN.md  # 상세 로드맵
│   ├── QUICK_START_GUIDE.md    # 빠른 시작
│   └── README.md               # 계획 개요
│
└── docs/                       # 문서
```

### 시스템 통신 흐름

**화재 진압 시나리오 (4단계 프로세스):**

```
Phase 1: 접근
  ├─ thermal/src/          → 핫스팟 자동 감지 ✅
  ├─ lidar/                → 거리 측정 (10m 확인) ✅
  └─ Navigation            → 열원까지 10m 지점 자율 비행
       ↓
Phase 2: 대기
  ├─ lidar/                → 10m 도착 확인
  └─ 호버링                → GCS 격발 신호 대기
       ↓
Phase 3: Targeting 활성화 ⭐
  ├─ GCS 격발 신호 수신   → ROS2 토픽
  ├─ targeting/            → 핫스팟 트래킹 시작
  │   ├─ hotspot_tracker         → Kalman Filter 추적
  │   ├─ drone_position_controller → 드론 상하좌우 미세 조정
  │   └─ targeting_manager       → 정조준 유지 (화면 중심)
  └─ LOCKED 판정           → 오차 < 임계값
       ↓
Phase 4: 발사
  └─ throwing_mechanism/   → 소화탄 발사 실행
```

**영상 오버레이 통합:**

```
항상 표시:
  ├─ 거리 오버레이 (lidar/)
  │   ├─ LINE SHAPE 표시
  │   ├─ 색상 코딩: 녹색 (9-11m), 빨간색 (<9m), 파란색 (>11m)
  │   └─ 거리 텍스트
  └─ 핫스팟 감지 (thermal/src/)
      ├─ 핫스팟 위치 (원 또는 마커)
      └─ 온도 정보

GCS 신호 후 표시:
  └─ 트래킹 상태 (targeting/)
      ├─ "TRACKING ACTIVE" 텍스트
      ├─ 화면 중심 십자선
      ├─ 오차 벡터 (중심 → 핫스팟)
      └─ "LOCKED" 표시 (정조준 완료 시)
```

**데이터 흐름:**

```
thermal/src/ (C++, 2,665 LOC)
    ↓
  핫스팟 위치 (x, y) + 온도
    ↓
lidar/ (C++, 1,188 LOC)
    ↓
  거리 측정 (10m 확인)
    ↓
[GCS 격발 신호]
    ↓
targeting/ (C++, 구현 예정)
    ├─ 핫스팟 추적 (Kalman Filter)
    ├─ 오차 계산 (중심 - 핫스팟)
    ├─ 드론 제어 명령 (PX4 offboard)
    └─ 정조준 완료 (LOCKED)
    ↓
throwing_mechanism/ (C++, 구획 예정)
    └─ 발사 (10m 고정, 고정 각도)
```

### 열화상 시스템 아키텍처

**멀티스레드 설계 (30 FPS 실시간):**

```
Main Thread
├── RGB Capture Thread          → rgb_frame_queue
├── Thermal Capture Thread      → thermal_frame_queue
├── Processing Thread
│   ├── Thermal analysis (핫스팟 감지)
│   ├── Frame composition (RGB + 열화상 오버레이)
│   └── Feature extraction
├── RTSP Server Thread          → GStreamer pipeline (H.264, 포트 8554)
└── HTTP Server Thread          → MJPEG stream (포트 8080)
```

**주요 기능:**
- 프레임 전달을 위한 스레드 안전 큐
- 핫스팟 감지를 위한 녹색 채널 최대값 찾기
- 열화상 오버레이를 위한 그라데이션 알파 블렌딩
- USB VID/PID를 통한 열화상/RGB 카메라 자동 감지
- 출력: 640x480 @ 30 FPS 합성 비디오

**카메라 지원:**
- 열화상: FLIR Lepton, Pure Thermal (USB)
- RGB: 표준 USB UVC 카메라

### LiDAR 거리 측정 시스템

**목적:** 정확한 발사체 탄도 계산을 위해 화재 표적까지의 정확한 거리를 측정합니다.

**현재 상태:** ⚠️ **하드웨어 선택됨, 통합 진행 중**

#### 선택된 하드웨어: LDROBOT LiDAR LD19

**LD19 사양:**
- 타입: 360° 스캔 LiDAR (ToF - Time of Flight)
- 범위: 0.05m ~ 12m
- 정확도: ±2cm (일반적)
- 스캔 속도: 10Hz (초당 4500 포인트)
- 각도 해상도: 1° (회전당 360 포인트)
- 시야각 (FOV): 360° (전체 원형 스캔)
- 업데이트 속도: 10Hz
- 인터페이스: UART (230400 baud), PWM
- 전원: 5V DC, ~350mA
- 무게: ~50g
- 크기: 직경 60mm × 높이 40mm
- 비용: ~$70-90
- IP 등급: IP65 (먼지 및 방수)

**✅ LD19가 이 프로젝트에 우수한 이유:**

**장점:**
- ✅ **360° 영역 스캔** - "영역 범위 측정" 요구사항 충족
- ✅ **경량** (50g) - 드론 페이로드에 미미한 영향
- ✅ **소형** (60mm) - 드론에 장착 용이
- ✅ **저비용** ($70-90) - 다중 드론 시스템에 적합
- ✅ **빠른 스캔 속도** (10Hz) - 실시간 영역 커버리지
- ✅ **좋은 범위** (최대 12m) - 화재 진압 거리에 적합
- ✅ **ROS2 지원** - 네이티브 드라이버 사용 가능
- ✅ **IP65 등급** - 실외 작동 가능
- ✅ **저전력** (1.75W) - 배터리 친화적

**대안과의 비교:**
| 기능 | LD19 | TF02-Pro | RPLIDAR A2 |
|---------|------|----------|------------|
| 스캔 타입 | 360° 회전 | 단일 포인트 | 360° 회전 |
| 범위 | 12m | 40m | 12m |
| 무게 | 50g | 35g | 190g |
| 비용 | $70-90 | $50-70 | $150 |
| 영역 커버리지 | ✅ 예 | ❌ 아니오 | ✅ 예 |
| ROS2 지원 | ✅ 네이티브 | ⚠️ 제한적 | ✅ 네이티브 |
| 업데이트 속도 | 10Hz | 1000Hz | 10Hz |
| **최적 용도** | **드론 영역 스캔** | 포인트 측정 | 데스크톱 로봇 |

**대안 대비 LD19 선택 이유:**
- **vs TF02-Pro:** LD19는 드론 회전 없이 즉시 360° 커버리지를 제공하며, 다중 표적 감지를 가능하게 합니다
- **vs RPLIDAR A2:** LD19는 74% 더 가볍고 (50g vs 190g) 47% 더 저렴하며 ($80 vs $150), 유사한 12m 범위를 제공합니다

#### 통합 아키텍처

**시스템 데이터 흐름:**
```
열화상 카메라 (thermal/src/)
    ↓
표적 각도 감지 [θ = 표적 방위각]
    ↓
LD19 360° 스캔 → 거리 배열 [d₀, d₁, d₂, ..., d₃₅₉]
    ↓
각도 θ에서 거리 추출 → d_target
    ↓
3D 위치 계산:
    x = d_target × cos(θ)
    y = d_target × sin(θ)
    z = altitude_diff (RTK GPS에서)
    ↓
탄도 계산기:
    - 발사 각도 (포물선 운동)
    - 발사 속도
    - 안전 체크 (5m < d < 50m)
    ↓
서보 제어 → 발사 각도 설정
    ↓
GPIO 트리거 → 발사체 발사
```

#### ROS2 통합

**LD19 ROS2 드라이버 설치:**

```bash
# 워크스페이스로 이동
cd ~/humiro_fire_suppression/workspaces

# LiDAR 워크스페이스 생성
mkdir -p lidar_ws/src
cd lidar_ws/src

# LD19 ROS2 드라이버 클론
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

# 드라이버 빌드
cd ..
source /opt/ros/humble/setup.bash
colcon build --packages-select ldlidar_stl_ros2

# 워크스페이스 소스
source install/setup.bash
```

**LD19 노드 실행:**

```bash
# 단일 드론
ros2 launch ldlidar_stl_ros2 ld19.launch.py

# 사용자 정의 매개변수로
ros2 launch ldlidar_stl_ros2 ld19.launch.py \
    serial_port:=/dev/ttyUSB0 \
    topic_name:=scan \
    frame_id:=lidar_link \
    range_threshold:=0.05

# 다중 드론 설정
ros2 launch ldlidar_stl_ros2 ld19.launch.py \
    serial_port:=/dev/ttyUSB0 \
    topic_name:=/drone_1/scan \
    frame_id:=drone_1_lidar_link
```

**발행된 ROS2 토픽:**

```bash
# LaserScan 데이터
/drone_X/scan (sensor_msgs/LaserScan)
    - ranges[360]: 거리 배열 (미터)
    - angle_min: 0.0 rad
    - angle_max: 6.28 rad (2π)
    - angle_increment: 0.0174 rad (1°)
    - range_min: 0.05m
    - range_max: 12.0m
    - scan_time: 0.1s (10Hz)

# 토픽 확인
ros2 topic list | grep scan
ros2 topic echo /drone_1/scan
ros2 topic hz /drone_1/scan  # ~10Hz 표시되어야 함
```

#### 하드웨어 설치

**장착 위치:**
- **위치:** 드론 상단 중앙, 전방 향함
- **방향:** 수평면 (0° 피치)
- **높이:** 프로펠러 간섭을 피하기 위해 위쪽
- **진동:** 진동 감소를 위한 고무 댐퍼 사용

**배선:**
```
LD19 커넥터 (JST-PH 2.0mm 8핀)
├── 핀 1: VCC (5V) → Khadas 5V 또는 FC AUX 전원
├── 핀 2: GND → 공통 접지
├── 핀 3: TX (UART) → Khadas USB-UART 변환기
├── 핀 4: RX (UART) → 사용 안 함 (단방향)
├── 핀 5: PWM → 선택적 속도 제어
└── 핀 6-8: 예약됨

권장: USB-UART 어댑터 (CH340, CP2102, FTDI)
Khadas VIM4 USB 포트 → USB-UART → LD19 UART
```

**전원:** 5V DC, 350mA (1.75W)

#### 테스트 및 검증

**1단계: LD19 연결 테스트**

```bash
# USB 장치 확인
lsusb | grep -i "serial\|uart"

# 시리얼 포트 확인
ls -l /dev/ttyUSB*

# 권한 설정
sudo chmod 666 /dev/ttyUSB0

# 드라이버 실행
ros2 launch ldlidar_stl_ros2 ld19.launch.py

# 토픽 검증
ros2 topic hz /scan  # ~10Hz 표시되어야 함
ros2 topic echo /scan --once
```

**2단계: RViz에서 시각화**

```bash
# RViz 실행
rviz2

# LaserScan 디스플레이 추가
# Fixed Frame: lidar_link
# Topic: /scan
# 360° 포인트 클라우드가 보여야 함
```

#### 현재 구현 상태

**✅ 완료:**
- 하드웨어 선택: LDROBOT LD19
- `docs/technical/LIDAR_TARGETING.md`에 전체 문서화
- ROS2 통합 아키텍처 설계
- 표적 거리 추출 알고리즘 설계
- 탄도 계산 공식 정의

**❌ 구현 예정:**
1. LD19 ROS2 드라이버 설치
2. `targeting/lidar_integration/`에서 `TargetDistanceNode` 구현
3. `targeting/trajectory_calc/`에서 `LaunchAngleCalculatorNode` 구현
4. 열화상 + LiDAR + GPS를 결합한 `IntegratedTargetingNode`
5. 하드웨어 장착 및 배선
6. 캘리브레이션 및 테스트

**중요 개발 경로:**
```
[✅] 열화상 카메라 → 핫스팟 각도
        ↓
[❌] LD19 ROS2 드라이버 → 스캔 데이터
        ↓
[❌] TargetDistanceNode → 거리 추출
        ↓
[❌] 3D 위치 계산 → (x, y, z)
        ↓
[❌] LaunchAngleCalculator → 탄도
        ↓
[❌] 서보 제어 → 조준 메커니즘
        ↓
[❌] GPIO 트리거 → 발사체 발사
```

#### 설정

**`config/device_config.env`에 추가:**
```bash
# LiDAR 설정
LIDAR_MODEL="LDROBOT_LD19"
LIDAR_SERIAL_PORT="/dev/ttyUSB0"
LIDAR_BAUD_RATE="230400"
LIDAR_SCAN_TOPIC="/drone_${DRONE_ID}/scan"
LIDAR_FRAME_ID="drone_${DRONE_ID}_lidar_link"

# 거리 임계값
MIN_LAUNCH_DISTANCE="5.0"  # 미터
MAX_LAUNCH_DISTANCE="50.0"  # 미터
TARGET_DISTANCE_TOPIC="/drone_${DRONE_ID}/target_distance"
```

#### 다음 단계

**Phase 1: 하드웨어 설정 (1주)**
1. 드론에 LD19 장착 (상단 중앙, 수평)
2. USB-UART를 통해 Khadas에 연결
3. 전력 소비 및 진동 테스트
4. 360° 스캔 커버리지 검증

**Phase 2: ROS2 통합 (1주)**
1. `ldlidar_stl_ros2` 드라이버 설치
2. 다중 드론 토픽 구성
3. 스캔 속도 및 정확도 테스트
4. `TargetDistanceNode` 구현

**Phase 3: 열화상 통합 (1주)**
1. 열화상 각도 토픽 구독
2. LD19 스캔에서 거리 추출
3. 3D 표적 위치 계산
4. 실제 화재 표적으로 테스트

**Phase 4: 탄도 계산 (1주)**
1. 발사 각도 계산기 구현
2. RTK GPS 고도와 통합
3. 포물선 운동 물리 테스트
4. 테스트 발사로 검증

**Phase 5: 전체 시스템 테스트 (1주)**
1. 엔드투엔드 타겟팅 워크플로우
2. 다중 드론 협조
3. 안전 검증
4. 성능 최적화

#### 참고 문서

- LD19 제품 페이지: https://www.ldrobot.com/product/en/130
- ROS2 드라이버: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
- 전체 통합 가이드: `docs/technical/LIDAR_TARGETING.md`
- 탄도 물리: 포물선 운동 공식
- 안전 프로토콜: 거리 제한, 장애물 감지

### ROS2 통합 패턴

**Micro-ROS Agent 흐름:**
```
PX4 Firmware (임베디드)
    ↓ XRCE-DDS (시리얼/이더넷)
Micro-ROS Agent (브리지)
    ↓ DDS (FastRTPS)
ROS2 Nodes (Humble)
    ↓ Topics/Services
Application Logic
```

**중요 토픽:**
- `/fmu/out/vehicle_attitude` - 드론 방향
- `/fmu/out/vehicle_status` - 비행 상태
- `/fmu/out/vehicle_gps_position` - GPS 위치
- `/drone_X/thermal_camera/image_raw` - 열화상 피드
- `/drone_X/fire_target/hotspot` - 표적 좌표

## 개발 워크플로우

### 새 Python 모듈 추가

1. 적절한 서브시스템에 모듈 생성:
   ```bash
   mkdir -p targeting/new_feature
   touch targeting/new_feature/__init__.py
   touch targeting/new_feature/my_module.py
   ```

2. 모듈은 `setup_env.sh`에 의해 PYTHONPATH에 자동으로 추가됩니다

3. 어디서나 가져오기:
   ```python
   from targeting.new_feature import MyClass
   ```

### 열화상 시스템 수정 (C++)

1. `thermal/src/`에서 소스 파일 편집
2. 재빌드:
   ```bash
   cd thermal/src/build
   make -j$(nproc)
   ```
3. 바이너리 테스트: `./thermal_rgb_streaming`

### ROS2 패키지 수정

1. `workspaces/*/src/`에서 소스 편집
2. 특정 패키지만 재빌드:
   ```bash
   cd $MICRO_ROS_WS
   colcon build --packages-select <package_name>
   source install/setup.bash
   ```
3. 서비스 재시작:
   ```bash
   sudo systemctl restart micro-ros-agent.service
   ```

### 설정 변경 배포

1. 설정 템플릿 편집:
   - `config/device_config.env` - 드론별 설정
   - `deployment/systemd/*.service` - 서비스 정의
   - `config/network/*.yaml` - 네트워크 설정

2. 변경사항 적용:
   ```bash
   sudo ./scripts/install/003-apply_config.sh
   ```

3. 검증:
   ```bash
   ./scripts/check/101-check_px4_connection.sh
   sudo systemctl status micro-ros-agent.service
   ```

### 통신 문제 디버깅

**단계별 진단:**

1. 서비스 상태 확인:
   ```bash
   sudo systemctl status micro-ros-agent.service
   sudo systemctl status mavlink-router.service
   ```

2. 실시간 로그 보기:
   ```bash
   sudo journalctl -u micro-ros-agent.service -f
   ```

3. 네트워크 구성 확인:
   ```bash
   ip addr show eth0  # 10.0.0.X 표시되어야 함
   ip neigh show dev eth0  # PX4가 10.0.0.X+1에 표시되어야 함
   ```

4. ROS2 연결 테스트:
   ```bash
   source $MICRO_ROS_WS/install/setup.bash
   ros2 topic list  # /fmu/* 토픽이 표시되어야 함
   ros2 topic echo /fmu/out/vehicle_status
   ```

5. XRCE-DDS 포트 확인:
   ```bash
   sudo netstat -ulnp | grep 8888
   ```

### 열화상 스트리밍 테스트

1. 열화상 시스템 시작:
   ```bash
   cd ~/humiro_fire_suppression/thermal/src/build
   ./thermal_rgb_streaming
   ```

2. RTSP 스트림 보기:
   ```bash
   # 다른 머신에서
   vlc rtsp://192.168.100.11:8554/stream
   ```

3. HTTP 스트림 보기:
   ```bash
   # 브라우저 열기
   http://192.168.100.11:8080
   ```

## 중요한 개발 규칙

### 경로 관리
- **절대 하드코딩된 경로 사용 금지** - 항상 환경 변수 사용
- `$PROJECT_ROOT`, `$MICRO_ROS_WS`, `$PX4_ROS2_WS` 등 사용
- 이를 통해 프로젝트가 어느 위치에서든 작동할 수 있습니다

### 워크스페이스 재빌드 요구사항
- ROS2 워크스페이스는 빌드 아티팩트에 **절대 경로**를 포함합니다
- `git clone` 또는 프로젝트 디렉토리 이동 후 **반드시 재빌드**:
  ```bash
  ./scripts/install/004-rebuild_workspaces.sh
  ```
- `build/`, `install/`, `log/` 디렉토리는 `.gitignore`에 포함됩니다 (의도적)

### 환경 설정
- **항상 먼저 실행**: `source ~/humiro_fire_suppression/setup_env.sh`
- 필요: PYTHONPATH, ROS2 경로, 프로젝트 변수
- 모든 스크립트는 이를 자동으로 로드하지만, 대화형 작업은 수동 소싱이 필요합니다

### 서비스 구성
- **절대 직접 편집하지 마세요** `/etc/systemd/system/` 파일
- 항상 `deployment/systemd/`의 템플릿 편집
- 다음으로 적용: `sudo ./scripts/install/003-apply_config.sh`
- 스크립트는 경로 대체 및 systemd 재로드를 처리합니다

### Python 모듈 구성
- 모든 서브시스템 디렉토리가 PYTHONPATH에 자동으로 추가됩니다
- 가져오기 패턴: `from subsystem.module import Class`
- sys.path를 수동으로 수정할 필요 없음

### 다중 드론 구성
- 드론별 설정을 위해 `config/device_config.env` 편집
- `DRONE_ID`, `ETH0_IP`, `WIFI_IP`, `ROS_NAMESPACE` 설정
- 구성 스크립트로 적용
- 프로젝트 경로 일관성 유지: `~/humiro_fire_suppression`

## 일반적인 문제 및 해결책

### 문제: "ROS2 토픽을 찾을 수 없음"
**해결책:**
```bash
source $MICRO_ROS_WS/install/setup.bash
sudo systemctl restart micro-ros-agent.service
ros2 daemon stop && ros2 daemon start
```

### 문제: "git clone 후 워크스페이스 빌드 실패"
**해결책:**
```bash
cd $MICRO_ROS_WS
rm -rf build install log
colcon build --symlink-install
```

### 문제: "네트워크에서 PX4를 감지할 수 없음"
**해결책:**
```bash
# eth0 구성 확인
ip addr show eth0

# DHCP 서버 재시작
sudo systemctl restart dnsmasq-px4.service

# ARP 테이블 확인
ip neigh show dev eth0

# 진단 실행
./scripts/check/101-check_px4_connection.sh
```

### 문제: "열화상 카메라를 찾을 수 없음"
**해결책:**
```bash
# USB 장치 나열
lsusb

# 카메라 권한 확인
ls -la /dev/video*

# 사용자를 video 그룹에 추가
sudo usermod -a -G video $USER
```

### 문제: "서비스 시작 실패"
**해결책:**
```bash
# 서비스 로그 확인
sudo journalctl -u micro-ros-agent.service -n 100

# 서비스 파일의 경로 확인
sudo systemctl cat micro-ros-agent.service

# 구성 재적용
sudo ./scripts/install/003-apply_config.sh
```

## 중요한 파일 및 구성

### 구성 파일

**장치 구성** (`config/device_config.env`):
```bash
DRONE_ID=1
ROS_NAMESPACE=drone1
ETH0_IP=10.0.0.11
FC_IP=10.0.0.12
WIFI_IP=192.168.100.11
ROS_DOMAIN_ID=0
XRCE_DDS_PORT=8888
```

**버전 추적** (`config/versions.env`):
- PX4_FIRMWARE_VERSION="v1.16.0"
- ROS2_DISTRO="humble"
- MICRO_ROS_AGENT_VERSION (커밋 해시)

### 서비스 정의

**Micro-ROS Agent** (`deployment/systemd/micro-ros-agent.service`):
- XRCE-DDS를 통해 PX4 ↔ ROS2 브리지
- 실패 시 자동 재시작
- journalctl에 로그 기록

**MAVLink Router** (`deployment/systemd/mavlink-router.service`):
- QGC와 PX4 간 MAVLink 라우팅
- 다중 드론 QGC를 위한 UDP 브로드캐스트

**DHCP Server** (`deployment/systemd/dnsmasq-px4.service`):
- eth0에서 PX4에 DHCP 제공
- 안정적인 FC 주소 지정 보장

## 테스트

### 테스트 실행

```bash
# 단위 테스트 (구현 시)
cd ~/humiro_fire_suppression/tests/unit
python3 -m pytest

# 통합 테스트
cd ~/humiro_fire_suppression/tests/integration
python3 -m pytest

# 하드웨어 테스트 (연결된 하드웨어 필요)
cd ~/humiro_fire_suppression/tests/hardware
python3 test_gpio.py
python3 test_thermal_camera.py
```

### 수동 검증 체크리스트

- [ ] 환경 소싱: `source setup_env.sh`
- [ ] 서비스 실행 중: `sudo systemctl status micro-ros-agent mavlink-router`
- [ ] PX4 연결됨: `./scripts/check/101-check_px4_connection.sh`
- [ ] ROS2 토픽: `ros2 topic list`
- [ ] 열화상 스트리밍: RTSP/HTTP 엔드포인트 테스트
- [ ] 네트워크 구성: `ip addr show eth0 wlan0`

## 성능 고려사항

### 열화상 시스템
- **목표**: 640x480 @ 30 FPS
- **CPU 사용률**: 단일 코어에서 ~40% (C++ 최적화)
- **지연시간**: <100ms glass-to-glass
- **최적화**: 멀티스레딩, 스레드 안전 큐, 최소 복사

### ROS2 통신
- **QoS**: 텔레메트리는 BEST_EFFORT, 명령은 RELIABLE
- **메시지 속도**: 일반적으로 50-100 Hz
- **네트워크**: PX4는 기가비트 이더넷, QGC는 WiFi 5

### 다중 드론 확장
- **현재**: 3개 드론으로 테스트됨
- **제한**: 네트워크 대역폭 (WiFi)이 병목
- **해결책**: 5 GHz WiFi, 필요시 별도 ROS 도메인

## 보안 참고사항

- 드론 간 통신에 SSH 키 사용
- 저장소에 자격 증명 저장 안 함
- 서비스 파일은 환경 변수 사용
- 네트워크 격리 (eth0는 FC 전용)

## 추가 리소스

**문서:**
- 기술 사양: `docs/technical/FIREFIGHTING_SYSTEM.md`
- RTK GPS 가이드: `docs/technical/RTK_PRECISION_HOVERING.md`
- 설치: `docs/installation/3000-INSTALLATION_GUIDE.html`

**외부 링크:**
- PX4 문서: https://docs.px4.io/
- ROS2 Humble: https://docs.ros.org/en/humble/
- Micro-ROS: https://micro.ros.org/

## 빠른 참조 명령어

```bash
# 일일 워크플로우
source ~/humiro_fire_suppression/setup_env.sh
./scripts/check/101-check_px4_connection.sh

# ROS2 빌드
cd $MICRO_ROS_WS && colcon build && source install/setup.bash

# 열화상 빌드
cd ~/humiro_fire_suppression/thermal/src/build && make -j$(nproc)

# 로그 보기
sudo journalctl -u micro-ros-agent.service -f

# 모든 서비스 재시작
sudo systemctl restart micro-ros-agent mavlink-router dnsmasq-px4

# LD19 LiDAR 테스트
ros2 topic hz /drone_1/scan  # 스캔 속도 확인 (약 10Hz여야 함)
ros2 topic echo /drone_1/scan --once  # 스캔 데이터 보기
ros2 topic echo /drone_1/target_distance  # 표적 거리 추출 확인
# ROS2 확인
ros2 topic list && ros2 node list
```

## 규칙

### 핵심 개발 원칙

#### 1. C/C++ 우선 정책 (CRITICAL)

**기본 원칙: 모든 핵심 모듈은 C/C++로 개발해야 합니다.**

**필수 사항:**
- ✅ **새로운 기능 구현 시 기본적으로 C++ 사용**
- ✅ **실시간 성능이 필요한 모든 모듈은 C++ 필수**
- ✅ **하드웨어 인터페이스는 반드시 C++ 사용**
- ✅ **이미지/비디오 처리 및 오버레이는 C++ 사용**
- ✅ **성능이 중요한 경로는 C++ 사용**

**Python 사용 허용 조건:**
- ⚠️ ROS2 노드 래퍼 (C++ 라이브러리를 호출하는 경우만)
- ⚠️ 고수준 조정 로직 (비실시간)
- ⚠️ 설정 파일 로더 (비실시간)
- ⚠️ 테스트 및 프로토타이핑

**언어 선택 의사결정 트리:**
```
새 기능 구현 시작
    ↓
실시간 성능이 필요한가?
    ├─ 예 → C++ 사용
    └─ 아니오 ↓
하드웨어와 인터페이스하는가?
    ├─ 예 → C++ 사용
    └─ 아니오 ↓
이미지/비디오 처리를 하는가?
    ├─ 예 → C++ 사용
    └─ 아니오 ↓
기존 C++ 코드의 래퍼인가?
    ├─ 예 → Python 허용
    └─ 아니오 → C++ 사용 (기본값)
```

**위반 시 조치:**
- Python으로 작성된 성능 중요 모듈은 C++로 리팩토링 필요
- 하드웨어 인터페이스를 Python으로 작성한 경우 즉시 C++로 전환
- 실시간 처리가 필요한 모듈을 Python으로 작성한 경우 C++로 재작성

#### 2. 코드 품질 규칙

**C++ 코딩 표준:**
- C++17 표준 준수
- Google C++ Style Guide 준수
- 모든 헤더 파일에 include guard 사용
- 메모리 안전성: 스마트 포인터 우선 사용 (`std::unique_ptr`, `std::shared_ptr`)
- 예외 안전성: RAII 원칙 준수
- 스레드 안전성: 멀티스레드 환경에서 적절한 동기화 사용

**Python 코딩 표준:**
- PEP 8 스타일 가이드 준수
- 타입 힌트 사용 권장
- ROS2 래퍼에만 사용 (핵심 로직은 C++)

**주석 및 문서화:**
- 모든 공개 API에 문서화 주석 작성
- 복잡한 알고리즘은 인라인 주석으로 설명
- 파일 헤더에 목적과 저자 명시

#### 3. 빌드 및 의존성 규칙

**빌드 시스템:**
- C++: CMake 사용 (최소 버전 3.16)
- Python: setuptools 사용 (ROS2 패키지)
- 모든 의존성은 명시적으로 선언
- 외부 라이브러리는 버전 고정

**의존성 관리:**
- 시스템 패키지는 `scripts/install/`에 설치 스크립트 제공
- Python 패키지는 `requirements.txt` 또는 `setup.py`에 명시
- C++ 라이브러리는 `CMakeLists.txt`에 명시

#### 4. 경로 및 설정 규칙

**경로 관리:**
- ❌ **절대 경로 하드코딩 금지**
- ✅ 환경 변수 사용 (`$PROJECT_ROOT`, `$MICRO_ROS_WS` 등)
- ✅ 상대 경로는 실행 파일 기준으로 계산
- ✅ 설정 파일은 `config/` 디렉토리에 위치

**설정 관리:**
- 모든 설정은 환경 변수 또는 설정 파일로 관리
- 드론별 설정은 `config/device_config.env` 사용
- 서비스 설정은 `deployment/systemd/` 템플릿 사용
- 직접 `/etc/systemd/system/` 파일 편집 금지

#### 5. 테스트 규칙

**테스트 작성:**
- 모든 핵심 기능에 단위 테스트 작성
- 하드웨어 인터페이스는 모의 객체(mock) 사용
- 통합 테스트는 실제 하드웨어 없이도 실행 가능하도록 설계

**테스트 실행:**
- 코드 커밋 전에 모든 테스트 통과 확인
- CI/CD 파이프라인에서 자동 테스트 실행

#### 6. 성능 규칙

**성능 목표:**
- 열화상 스트리밍: 30 FPS @ 640x480 유지
- 지연시간: <100ms glass-to-glass
- CPU 사용률: 단일 코어에서 40% 이하
- 메모리 사용: 불필요한 메모리 할당 최소화

**최적화 원칙:**
- 프로파일링 후 최적화 (premature optimization 금지)
- 멀티스레딩 활용 (스레드 안전 큐 사용)
- 메모리 복사 최소화 (참조 전달 우선)
- 불필요한 동적 할당 피하기

#### 7. 안전성 규칙

**안전 체크:**
- 모든 하드웨어 제어 전에 안전 조건 확인
- 거리 제한: 5m < d < 50m (발사 전 확인)
- 비상 정지 기능 필수 구현
- 예외 상황 처리 (센서 오류, 통신 실패 등)

**에러 처리:**
- 모든 함수는 명확한 에러 코드 반환
- 예외는 최상위 레벨에서만 처리
- 로깅을 통한 에러 추적 가능하도록 구현

#### 8. 버전 관리 규칙

**Git 사용 규칙:**
- 의미 있는 커밋 메시지 작성
- 기능별로 브랜치 분리
- 코드 리뷰 후 메인 브랜치에 병합
- `build/`, `install/`, `log/` 디렉토리는 `.gitignore`에 포함

**버전 관리:**
- 모든 주요 변경사항은 문서화
- API 변경 시 호환성 고려
- 버전 번호는 시맨틱 버저닝 사용

#### 9. 문서화 규칙

**필수 문서:**
- 모든 모듈에 README.md 작성
- 공개 API는 Doxygen 스타일 주석 사용
- 아키텍처 변경은 `work-plan/`에 문서화
- 중요한 결정사항은 `CLAUDE.md`에 기록

**문서 업데이트:**
- 코드 변경 시 관련 문서도 함께 업데이트
- 사용 예제는 실제 작동하는 코드로 제공
- 설정 변경은 설정 가이드에 반영

#### 10. 다중 드론 규칙

**네임스페이스 관리:**
- 모든 ROS2 토픽은 드론 ID를 포함 (`/drone_X/...`)
- 설정 파일에서 `DRONE_ID` 환경 변수 사용
- 하드코딩된 드론 ID 금지

**네트워크 격리:**
- PX4 통신은 전용 이더넷 인터페이스 사용 (eth0)
- QGC 통신은 WiFi 인터페이스 사용 (wlan0)
- 네트워크 격리 유지 (보안)

---

### 개발 시 체크리스트

새 기능을 구현하기 전에 다음을 확인하세요:

- [ ] **언어 선택**: C++가 필요한가? (실시간, 하드웨어, 성능 중요)
- [ ] **경로 관리**: 하드코딩된 경로가 없는가?
- [ ] **설정 관리**: 환경 변수나 설정 파일을 사용하는가?
- [ ] **에러 처리**: 모든 에러 케이스를 처리하는가?
- [ ] **안전 체크**: 하드웨어 제어 전 안전 조건을 확인하는가?
- [ ] **테스트**: 단위 테스트를 작성했는가?
- [ ] **문서화**: README와 주석을 작성했는가?
- [ ] **성능**: 목표 성능을 달성할 수 있는가?

---

### 규칙 위반 시 조치

1. **C++ 우선 정책 위반**: 즉시 C++로 리팩토링
2. **하드코딩된 경로**: 환경 변수로 변경
3. **안전 체크 누락**: 즉시 추가
4. **문서화 누락**: 문서 작성 후 커밋
5. **테스트 누락**: 테스트 작성 후 커밋

---

**이 규칙들은 프로젝트의 일관성, 성능, 안전성을 보장하기 위해 필수입니다.**
