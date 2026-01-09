# 022. 통신 아키텍처 현황 및 설정

**작성일**: 2026-01-09
**버전**: v1.0

---

## 1. 현재 시스템 구성

### 1.1 네트워크 토폴로지

```
┌─────────────┐     10.0.0.0/24     ┌─────────────┐    192.168.100.0/24   ┌─────────────┐
│     FC      │◄───────────────────►│    VIM4     │◄─────────────────────►│   QGC/PC    │
│  (PX4)      │     eth0            │   (SBC)     │      WiFi             │             │
│ 10.0.0.32   │                     │ 10.0.0.31   │   192.168.100.31      │192.168.100.x│
└─────────────┘                     └─────────────┘                       └─────────────┘
```

### 1.2 통신 프로토콜

| 프로토콜 | 용도 | FC → VIM4 | VIM4 → QGC |
|----------|------|-----------|------------|
| **uXRCE-DDS** | ROS2 토픽 통신 | UDP 8888 | ROS2 DDS |
| **MAVLink** | FC 제어/상태 | UDP 14540 | UDP 14550 |

---

## 2. MAVLink Router 설정

### 2.1 설정 파일 위치
```
/etc/mavlink-router/main.conf
```

### 2.2 현재 설정 (2026-01-09)
```ini
[General]
TcpServerPort = 5790
ReportStats = true
MavlinkDialect = common

# FC (PX4) 연결 - Server 모드로 FC에서 오는 메시지 수신
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = 14540

# GCS (QGroundControl/SENDER) - Server 모드로 양방향 통신
[UdpEndpoint GCS]
Mode = Server
Address = 0.0.0.0
Port = 14550

# 로컬 애플리케이션 (메인프로그램 - 커스텀 메시지 처리)
[UdpEndpoint LocalApp]
Mode = Normal
Address = 127.0.0.1
Port = 14551

# ROS2 노드 연결
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = 14552
```

### 2.3 엔드포인트 모드 설명

| 모드 | 동작 | 용도 |
|------|------|------|
| **Server** | 지정 포트에서 수신 대기, 연결된 클라이언트에게만 응답 | FC 수신, 외부 명령 수신 |
| **Normal** | 지정 주소:포트로 전송, UDP 양방향 가능 | 브로드캐스트, 로컬 앱 전달 |

### 2.4 현재 문제점

```
FC → VIM4:14540 (Server) → 수신 OK (Handled: 1147 messages)
VIM4 → QGC (Server mode) → 전송 FAIL (Transmitted: 0)
```

**원인**: GCS 엔드포인트가 Server 모드로 설정되어 있어서:
- 클라이언트(QGC)가 먼저 메시지를 보내야 응답 가능
- QGC가 단순 UDP 리스닝만 하면 FC 상태를 받을 수 없음

### 2.5 해결 방안

#### 방안 A: GCS를 Normal 모드로 변경 (브로드캐스트)
```ini
[UdpEndpoint GCS]
Mode = Normal
Address = 192.168.100.255
Port = 14550
```
- **장점**: QGC가 브로드캐스트 수신으로 FC 상태 확인 가능
- **단점**: 외부에서 VIM4로 메시지 전송 불가 (14550에서 수신 안 함)

#### 방안 B: 입출력 분리 (권장)
```ini
# 외부 메시지 수신용 (입력)
[UdpEndpoint GCS_In]
Mode = Server
Address = 0.0.0.0
Port = 14553

# FC 상태 브로드캐스트용 (출력)
[UdpEndpoint GCS_Out]
Mode = Normal
Address = 192.168.100.255
Port = 14550
```
- **장점**: 입력(14553)과 출력(14550) 분리, 양방향 통신 가능
- **단점**: QGC/SENDER에서 14553 포트로 전송해야 함

#### 방안 C: QGC 설정 변경
- QGC에서 VIM4:14550으로 **능동적 연결** 설정
- Server 모드 유지하면서 양방향 통신

---

## 3. uXRCE-DDS Agent 설정

### 3.1 서비스 파일 위치
```
/etc/systemd/system/micro-ros-agent.service
/etc/systemd/system/micro-ros-agent.service.d/override.conf
```

### 3.2 서비스 설정
```ini
[Unit]
Description=Micro-ROS Agent for PX4 Communication
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=khadas
Group=khadas
WorkingDirectory=/home/khadas/humiro_fire_suppression
Environment="HOME=/home/khadas"
Environment="ROS_DOMAIN_ID=0"
Environment="ROS_NAMESPACE=drone1"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/home/khadas/humiro_fire_suppression/scripts/runtime/start_micro_ros_agent_wrapper.sh
Restart=always
RestartSec=5
```

### 3.3 Override 설정
```ini
[Service]
Environment="ROS_DOMAIN_ID=0"
# ROS_NAMESPACE는 PX4 uXRCE-DDS와 호환성 문제로 제거
```

### 3.4 실행 스크립트
```bash
# /home/khadas/humiro_fire_suppression/scripts/runtime/start_micro_ros_agent_wrapper.sh
exec "$MICRO_ROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" udp4 --port 8888
```

### 3.5 현재 상태
```
● micro-ros-agent.service - Micro-ROS Agent for PX4 Communication
     Active: active (running)

[연결 정보]
- client_key: 0x00000003 (FC 연결됨)
- UDP 포트: 8888
- 토픽 생성: publisher/subscriber 정상 동작
```

---

## 4. FC (PX4) 파라미터 설정

### 4.1 MAVLink 설정 (MAV_2)
| 파라미터 | 값 | 설명 |
|----------|-----|------|
| MAV_2_CONFIG | 1000 | Ethernet |
| MAV_2_UDP_PRT | 14550 | FC 로컬 포트 (송신 소스) |
| MAV_2_REMOTE_PRT | 14540 | VIM4 수신 포트 (목적지) |
| MAV_2_BROADCAST | 1 | 브로드캐스트 활성화 |
| MAV_2_MODE | NORMAL | 일반 모드 |
| MAV_2_RATE | 100000 | 전송 속도 |

### 4.2 uXRCE-DDS 설정
| 파라미터 | 값 | 설명 |
|----------|-----|------|
| UXRCE_DDS_AG_IP | 167772191 | VIM4 IP (10.0.0.31) |
| UXRCE_DDS_PRT | 8888 | Agent 포트 |

---

## 5. 포트 사용 현황

### 5.1 VIM4 리스닝 포트
```
포트      모드      프로세스              용도
14540    Server    mavlink-routerd      FC MAVLink 수신
14550    Server    mavlink-routerd      GCS 양방향 (현재 문제)
14551    Normal    humiro_fire_sup      메인 프로그램 (커스텀 메시지)
14552    Normal    mavlink-routerd      ROS2 노드
8888     UDP       micro_ros_agent      uXRCE-DDS
15000    Server    humiro_fire_sup      테스트용 (OSD)
```

### 5.2 통신 흐름

```
[정상 동작 중]
FC (10.0.0.32:8888) ──uXRCE-DDS──► VIM4:8888 (micro-ros-agent) ──► ROS2 Topics

[문제 발생 중]
FC (브로드캐스트:14540) ──MAVLink──► VIM4:14540 (수신 OK)
                                         │
                                         ▼
                               VIM4:14550 (Server 모드)
                                         │
                                         ✗ QGC로 전송 안 됨
                                         (클라이언트 미연결)
```

---

## 6. 권장 조치사항

### 즉시 조치
1. **GCS 엔드포인트를 Normal 모드로 변경**하여 QGC 연결 복구
2. 외부 메시지 수신이 필요하면 별도 포트(14553) 추가

### 장기 계획
1. QGC 커스터마이징 시 능동적 연결 방식 구현
2. 커스텀 메시지 전용 포트 체계 정립

---

## 7. 서비스 관리 명령어

```bash
# MAVLink Router
sudo systemctl status mavlink-router
sudo systemctl restart mavlink-router
sudo journalctl -u mavlink-router -f

# uXRCE-DDS Agent
sudo systemctl status micro-ros-agent
sudo systemctl restart micro-ros-agent
sudo journalctl -u micro-ros-agent -f

# 설정 파일 편집
sudo nano /etc/mavlink-router/main.conf
sudo systemctl daemon-reload
```
