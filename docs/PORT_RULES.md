# 포트 규칙 정의서

## 개요
다중 드론 시스템에서 포트 충돌을 방지하기 위한 포트 할당 규칙

## 포트 계산 공식
```
port = base_port + (DRONE_ID - 1) * 10
```

## 변동 포트 (DRONE_ID 기반)

| 용도 | Base Port | 드론 1 | 드론 2 | 드론 3 | 설정 위치 |
|------|-----------|--------|--------|--------|-----------|
| **TcpServerPort** | 5790 | 5790 | 5800 | 5810 | main.conf |
| **QGC_UDP_PORT** | 14550 | 14550 | 14560 | 14570 | device_config.env, main.conf |
| **ROS2 Port** | 14551 | 14551 | 14561 | 14571 | main.conf |
| **App Port** | 15001 | 15001 | 15011 | 15021 | main.conf, application |
| **External Port** | 16001 | 16001 | 16011 | 16021 | device_config.env, main.conf |

## 고정 포트 (모든 드론 동일)

| 용도 | 포트 | 설정 위치 |
|------|------|-----------|
| **FC_MAVLINK_PORT** | 14540 | device_config.env, main.conf |
| **XRCE_DDS_PORT** | 8888 | device_config.env |
| **RTSP_PORT** | 8554 | device_config.env |
| **HTTP (GUI)** | 5000 | humiro-gui.service |
| **HTTP (Camera)** | 8080 | humiro-fire-suppression |

## 엔드포인트 설명

### mavlink-router (main.conf)
- **FC**: FC(PX4)에서 브로드캐스트 수신 (14540, Server 모드)
- **GCS**: QGroundControl로 브로드캐스트 송신 (변동 포트, Normal 모드)
- **External**: 외부 테스트/디버깅 도구 수신 (변동 포트, Server 모드)
- **ROS2**: 로컬 ROS2 노드 연결 (변동 포트, Normal 모드)
- **Application**: Application Manager 연결 (변동 포트, Normal 모드)

### Application Manager
- **App Port (15001+)**: mavlink-router로부터 MAVLink 메시지 수신
- **External Port (16001+)**: 송신 GUI에서 커스텀 메시지 수신

## IP 주소 규칙

| 네트워크 | 드론 1 | 드론 2 | 드론 3 | 공식 |
|----------|--------|--------|--------|------|
| ETH0 (FC) | 10.0.0.11 | 10.0.0.21 | 10.0.0.31 | 10.0.0.(ID*10+1) |
| FC IP | 10.0.0.12 | 10.0.0.22 | 10.0.0.32 | 10.0.0.(ID*10+2) |
| WiFi | 192.168.100.11 | 192.168.100.21 | 192.168.100.31 | 192.168.100.(ID*10+1) |

## GCS 포트 모드

### separate (기본값)
각 드론이 별도 포트 사용:
- 드론 1: 14550
- 드론 2: 14560
- 드론 3: 14570

### unified
모든 드론이 동일 포트 사용:
- 모든 드론: 14550

## 관련 파일
- `/home/khadas/humiro_fire_suppression/config/device_config.env`
- `/etc/mavlink-router/main.conf`
- `/home/khadas/humiro_fire_suppression/gui/utils/config_manager.py`
- `/home/khadas/humiro_fire_suppression/application/src/application_manager.cpp`
- `/home/khadas/humiro_fire_suppression/scripts/install/002-install_mavlink_router.sh`

## 변경 이력
- 2026-01-19: 포트 규칙 문서 작성 및 확정
