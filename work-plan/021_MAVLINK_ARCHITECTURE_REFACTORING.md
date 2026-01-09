# MAVLink 아키텍처 표준화 개선 계획

**작성일**: 2026-01-08  
**수정일**: 2026-01-09 (라우터 설정 오류 수정)
**목적**: 표준 MAVLink 아키텍처에 맞게 메시지 라우팅 구조 개선  
**버전**: v2.1

---

## 개요

현재 구현된 MAVLink 메시지 라우팅 방식을 표준 MAVLink 아키텍처에 맞게 개선하는 계획입니다. MAVLink 라우터를 중심으로 한 표준 아키텍처로 변경하여 유지보수성, 확장성, 표준 준수를 향상시킵니다.

---

## 현재 구현 (방식 2) - 변경 전

### 아키텍처

```
QGC/SENDER (외부 PC)
    ↓
    │ WiFi 브로드캐스트 (UDP 14550)
    │
    ▼
VIM4 Custom Message Library (14550 포트 직접 리스닝)
    │
    ├─→ 표준 메시지 → MAVLink Router (127.0.0.1:14551) → FC
    │
    └─→ 커스텀 메시지 → 메인프로그램 (파싱 + 콜백)
```

### 특징

- **Custom Message Library가 14550 포트를 직접 리스닝**
- 표준 메시지를 받아서 MAVLink 라우터(14551)로 전달
- 커스텀 메시지는 파싱 후 메인프로그램으로 전달

### 단점

- ❌ 표준 MAVLink 아키텍처와 다름
- ❌ Custom Library가 중간 단계로 작동 (표준 메시지가 한 단계 더 거침)
- ❌ 포트 충돌 가능성 (14550을 여러 프로그램이 리스닝)

---

## 개선안 (방식 1 - 표준 MAVLink 아키텍처)

### 아키텍처

```
                    QGC (외부 PC, 192.168.100.x)
                              ↑↓
                    WiFi (192.168.100.0/24)
                              ↑↓
┌─────────────────────────────────────────────────────────────┐
│                    VIM4 (SBC)                               │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │              MAVLink Router                             │ │
│  │                                                         │ │
│  │  [FC Endpoint]        [GCS Endpoint]     [LocalApp]     │ │
│  │   Server 모드          Normal 모드        Normal 모드   │ │
│  │   0.0.0.0:14540        브로드캐스트       127.0.0.1     │ │
│  │                        192.168.100.255    :14551        │ │
│  │                        :14550                            │ │
│  └────────────────────────────────────────────────────────┘ │
│         ↑↓                      ↑↓                 ↓        │
│         │                       │                  │        │
│    FC 메시지                QGC↔FC             커스텀       │
│    수신/전달               상태 전달          메시지 전달   │
│         │                       │                  │        │
│         │                       │                  ▼        │
│         │                       │    ┌──────────────────┐   │
│         │                       │    │ Custom Message   │   │
│         │                       │    │ Library (14551)  │   │
│         │                       │    │ 커스텀 메시지만  │   │
│         │                       │    │ 파싱 및 콜백     │   │
│         │                       │    └────────┬─────────┘   │
│         │                       │             │             │
│         │                       │             ▼             │
│         │                       │    ┌──────────────────┐   │
│         │                       │    │  메인 프로그램   │   │
│         │                       │    │ (Application     │   │
│         │                       │    │  Manager)        │   │
│         │                       │    └──────────────────┘   │
│         │                       │                           │
└─────────┼───────────────────────┼───────────────────────────┘
          │                       │
          ↓                       │
   ┌──────────────┐               │
   │  FC (PX4)    │←──────────────┘
   │ 10.0.0.32    │   (표준 메시지는 라우터가 자동 전달)
   │ :14540      │
   └──────────────┘
```

### 메시지 흐름 상세

#### 1. FC → QGC (상태 메시지)
```
FC (10.0.0.32)
    ↓ UDP 14540으로 전송
MAVLink Router (Server 모드로 수신)
    ↓ 모든 엔드포인트로 전달
GCS Endpoint (Normal 모드, 브로드캐스트)
    ↓ 192.168.100.255:14550으로 송신
QGC (WiFi 네트워크의 모든 클라이언트가 수신)
```

#### 2. QGC → FC (표준 명령)
```
QGC
    ↓ 192.168.100.255:14550 또는 VIM4 IP로 전송
MAVLink Router (GCS Normal 모드 - UDP 양방향 수신)
    ↓ FC 엔드포인트로 전달
FC (10.0.0.32:14540)
```

#### 3. QGC → Custom Message Library (커스텀 메시지)
```
QGC
    ↓ 커스텀 메시지 전송
MAVLink Router
    ↓ LocalApp 엔드포인트로 전달
Custom Message Library (127.0.0.1:14551)
    ↓ 메시지 파싱 및 콜백
메인 프로그램
```

### 특징

- **MAVLink 라우터가 중앙 허브 역할**
- **표준 메시지**: 라우터가 자동으로 FC↔QGC 간 전달 (메인프로그램 거치지 않음)
- **커스텀 메시지**: 라우터 → LocalApp(14551) → Custom Message Library → 메인프로그램
- **FC 엔드포인트**: Server 모드로 FC에서 오는 메시지 수신
- **GCS 엔드포인트**: Normal 모드(클라이언트)로 브로드캐스트 + UDP 양방향 수신

---

## MAVLink Router 엔드포인트 모드 설명

### Server 모드
- 지정된 포트에서 **리스닝** (바인딩)
- 외부에서 해당 포트로 보내는 메시지 수신
- 사용 예: FC가 VIM4:14540으로 메시지를 보낼 때 수신

### Normal 모드 (Client)
- 지정된 주소:포트로 메시지 **송신**
- UDP 소켓이므로 **양방향 통신 가능** (상대방 응답 수신)
- 사용 예: FC 상태를 QGC로 브로드캐스트, QGC 응답도 수신

### 중요 사항
> **GCS 엔드포인트를 Server 모드로 설정하면 안 됩니다!**
> Server 모드는 수신만 하고 송신하지 않으므로, FC→QGC 방향 메시지 전달이 불가능합니다.
> 반드시 **Normal 모드 + 브로드캐스트**를 사용해야 합니다.

---

## 변경 사항 상세

### 1. MAVLink 라우터 설정 변경

**파일 경로**: `/etc/mavlink-router/main.conf`

**올바른 설정**:
```ini
# Drone #3 mavlink-router 설정
# 버전: v2.1 - 2026-01-09

[General]
TcpServerPort = 5790
ReportStats = true
MavlinkDialect = common

# FC (PX4) 연결 - Server 모드로 FC에서 오는 메시지 수신
# FC가 VIM4(10.0.0.31:14540)로 MAVLink 메시지를 전송
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = 14540

# GCS (QGroundControl) - Normal 모드 (클라이언트)
# 브로드캐스트로 FC 상태를 QGC로 전달
# UDP 양방향으로 QGC에서 오는 명령도 수신
[UdpEndpoint GCS]
Mode = Normal
Address = 192.168.100.255
Port = 14550

# 로컬 애플리케이션 (Custom Message Library)
# 라우터가 수신한 모든 메시지를 이 엔드포인트로 전달
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

**주의사항**:
- GCS는 **반드시 Normal 모드**를 사용해야 함 (Server 모드 사용 금지)
- FC는 **Server 모드**로 FC에서 오는 메시지 수신

---

### 2. Custom Message Library 수정

**파일 경로**: `custom_message/src/custom_message.cpp`

#### 변경 목표
- MAVLink 라우터로 표준 메시지를 전달하는 코드 제거
- 라우터가 이미 표준 메시지를 FC↔QGC 간 자동 전달하므로 불필요
- Custom Message Library는 커스텀 메시지(50000-50004)만 처리

#### 2.1 생성자에서 mavlink_router_socket_fd_ 관련 코드 제거

**제거할 코드**:
```cpp
// 생성자 초기화 리스트에서 제거
, mavlink_router_socket_fd_(-1)

// 생성자 본문에서 제거
mavlink_router_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
if (mavlink_router_socket_fd_ >= 0) {
    memset(&mavlink_router_addr_, 0, sizeof(mavlink_router_addr_));
    mavlink_router_addr_.sin_family = AF_INET;
    mavlink_router_addr_.sin_port = htons(14551);
    // ... 나머지 코드
}
```

#### 2.2 소멸자에서 mavlink_router_socket_fd_ 관련 코드 제거

**변경 전**:
```cpp
~CustomMessageImpl() {
    stop();
    if (mavlink_router_socket_fd_ >= 0) {
        close(mavlink_router_socket_fd_);
        mavlink_router_socket_fd_ = -1;
    }
}
```

**변경 후**:
```cpp
~CustomMessageImpl() {
    stop();
}
```

#### 2.3 stop() 함수에서 mavlink_router_socket_fd_ 관련 코드 제거

#### 2.4 parseMAVLinkMessage()에서 표준 메시지 전달 로직 제거

**변경 전**:
```cpp
// 표준 메시지를 MAVLink 라우터로 전달
if (!is_custom_message && !is_command_long) {
    if (mavlink_router_socket_fd_ >= 0) {
        sendto(mavlink_router_socket_fd_, buffer, len, 0, ...);
    }
    return;
}
```

**변경 후**:
```cpp
// 표준 메시지는 라우터가 이미 FC로 전달했으므로 무시
// Custom Message Library는 커스텀 메시지만 처리
if (!is_custom_message) {
    return;
}
```

#### 2.5 클래스 멤버 변수에서 제거

```cpp
// 제거할 멤버 변수
int mavlink_router_socket_fd_;
struct sockaddr_in mavlink_router_addr_;
```

---

### 3. Application Manager 수정

**파일 경로**: `application/src/application_manager.cpp`

**변경할 함수**: `initializeCustomMessage()`

**변경 내용**:
```cpp
// CustomMessage 생성
// 라우터의 LocalApp 엔드포인트(14551)를 통해 메시지 수신
custom_message_handler_ = new custom_message::CustomMessage(
    14551,           // 수신 포트: 라우터의 LocalApp 엔드포인트
    14550,           // 송신 포트: QGC 브로드캐스트 (상태 메시지 전송용)
    127.0.0.1,     // 바인드 주소: 로컬 (라우터에서 수신)
    192.168.100.255, // 대상 주소: QGC 브로드캐스트
    system_id,
    component_id
);
```

---

## 검증 체크리스트

### 필수 검증 항목

- [ ] MAVLink 라우터 서비스 정상 실행
- [ ] FC 엔드포인트: Server 모드, 0.0.0.0:14540
- [ ] GCS 엔드포인트: Normal 모드, 192.168.100.255:14550
- [ ] QGC에서 드론 연결 성공 (메인프로그램 없이도 가능)
- [ ] 표준 메시지(ARM/DISARM) 정상 작동
- [ ] 커스텀 메시지 수신 정상 (메인프로그램 실행 시)

### 테스트 명령

```bash
# 라우터 상태 확인
sudo systemctl status mavlink-router.service

# 포트 리스닝 확인
sudo netstat -ulnp | grep mavlink

# 라우터 로그 확인 (메시지 통계)
sudo journalctl -u mavlink-router.service -f

# ROS2 토픽 확인 (DDS 연결)
ros2 topic list | grep fmu
```

---

## 롤백 절차

문제 발생 시:

```bash
# 라우터 설정 복원
sudo cp /etc/mavlink-router/main.conf.backup.* /etc/mavlink-router/main.conf
sudo systemctl restart mavlink-router.service
```

---

## 요약

| 항목 | 설정 |
|------|------|
| **FC 엔드포인트** | Server 모드, 0.0.0.0:14540 |
| **GCS 엔드포인트** | **Normal 모드**, 192.168.100.255:14550 |
| **LocalApp 엔드포인트** | Normal 모드, 127.0.0.1:14551 |
| **Custom Message Library** | 14551에서 수신, 커스텀 메시지만 처리 |

> **핵심**: GCS는 클라이언트(Normal 모드)입니다. Server 모드를 사용하면 FC→QGC 방향 메시지 전달이 불가능합니다.

---

**작성자**: Humiro Fire Suppression Team  
**버전**: v2.1  
**최종 수정일**: 2026-01-09
