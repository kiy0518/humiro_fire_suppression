# MAVLink 아키텍처 표준화 개선 계획

**작성일**: 2026-01-08  
**목적**: 표준 MAVLink 아키텍처에 맞게 메시지 라우팅 구조 개선  
**버전**: v1.0

---

## 개요

현재 구현된 MAVLink 메시지 라우팅 방식을 표준 MAVLink 아키텍처에 맞게 개선하는 계획입니다. MAVLink 라우터를 중심으로 한 표준 아키텍처로 변경하여 유지보수성, 확장성, 표준 준수를 향상시킵니다.

---

## 현재 구현 (방식 2)

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
- MAVLink 라우터 설정 변경 불필요

### 장점

- ✅ 커스텀 메시지 처리 로직이 이미 구현되어 있음
- ✅ MAVLink 라우터 설정 변경 불필요
- ✅ 커스텀 메시지 필터링/파싱이 Custom Library에서 처리
- ✅ 구현이 단순하고 유지보수 용이

### 단점

- ❌ 표준 MAVLink 아키텍처와 다름
- ❌ Custom Library가 중간 단계로 작동 (표준 메시지가 한 단계 더 거침)
- ❌ 포트 충돌 가능성 (14550을 여러 프로그램이 리스닝)
- ❌ 확장성 제한 (다른 애플리케이션이 메시지 수신 어려움)

---

## 개선안 (방식 1 - 표준 MAVLink 아키텍처)

### 아키텍처

```
QGC 또는 SENDER (외부 PC)
    ↓
    │ WiFi 브로드캐스트 (UDP 14550)
    │ 표준 메시지 (빨간색) + 커스텀 메시지 (파란색)
    │
    ▼
┌─────────────────────────────────────┐
│        SBC (VIM4)                   │
│                                      │
│  ┌──────────────────────────────┐   │
│  │  MAVLink Router              │   │
│  │  (14550 포트 리스닝 - Server)│   │
│  └──────────────────────────────┘   │
│           │                          │
│           ├─→ FC (14540)            │
│           │   표준 메시지 (자동 전달)│
│           │                          │
│           └─→ Custom Message        │
│               Library (14551)        │
│               커스텀 메시지 필터링   │
│                    │                 │
│                    ▼                 │
│           ┌─────────────────────┐   │
│           │  메인 프로그램      │   │
│           │  (Application       │   │
│           │   Manager)          │   │
│           │                      │   │
│           │  커스텀 메시지      │   │
│           │      ↓               │   │
│           │  표준 메시지 묶음    │   │
│           └─────────────────────┘   │
│                    │                 │
│                    │ 표준 메시지     │
└────────────────────┼─────────────────┘
                     │
                     ▼
              FC (PX4)
              (10.0.0.12:14540)
```

### 상태 메시지 경로

```
FC (PX4)
    ↓ 상태 메시지 (주황색)
MAVLink Router
    ↓ 브로드캐스트
QGC/SENDER (외부 PC)
```

### 특징

- **MAVLink 라우터가 14550 포트를 직접 리스닝** (Server 모드)
- **표준 메시지**: 라우터가 자동으로 FC로 전달 (메인프로그램 거치지 않음)
- **커스텀 메시지**: 라우터 → Custom Message Library (14551) → 메인프로그램
- **Custom Message Library**: 커스텀 메시지만 필터링 및 파싱하여 콜백 호출
- **메인프로그램**: 커스텀 메시지를 받아서 표준 메시지 시퀀스로 변환하여 FC로 전송
- **상태 메시지**: FC → 라우터 → QGC/SENDER (자동 전달)

### 메시지 처리 예시

**커스텀 메시지 처리**:
```
QGC: FIRE_MISSION_START (커스텀 메시지, ID: 12900)
    ↓ 브로드캐스트 14550
MAVLink Router
    ↓ 로컬 엔드포인트 14551
Custom Message Library (파싱)
    ↓ 콜백 호출
메인프로그램
    ↓ 자동 변환
표준 메시지 시퀀스:
  1. COMMAND_LONG (ARM)
  2. COMMAND_NAV_TAKEOFF
  3. SET_POSITION_TARGET
    ↓
FC (PX4)
```

**표준 메시지 처리**:
```
QGC: COMMAND_LONG (ARM) (표준 메시지, ID: 76)
    ↓ 브로드캐스트 14550
MAVLink Router
    ↓ 자동 라우팅
FC (PX4)
(메인프로그램 거치지 않음)
```

### 장점

- ✅ **표준 MAVLink 아키텍처 준수**
- ✅ 역할 분리 명확: 라우터는 라우팅만, 애플리케이션은 메시지 처리만
- ✅ 확장성: 여러 애플리케이션이 라우터의 다른 로컬 엔드포인트 사용 가능
- ✅ 유지보수성: 표준 패턴이므로 이해 및 수정이 쉬움
- ✅ 성능: 중간 단계 없이 라우터가 직접 라우팅
- ✅ 커스텀 메시지가 FC로 전달되지 않음 (라우터가 처리)

### 단점

- ❌ MAVLink 라우터 설정 변경 필요
- ❌ Custom Library 수정 필요 (포트 변경)

---

## 비교 분석

| 항목 | 현재 구현 (방식 2) | 개선안 (방식 1) |
|------|------------------|----------------|
| **표준 준수** | ❌ 비표준 | ✅ 표준 MAVLink 아키텍처 |
| **MAVLink 라우터 역할** | 부분적 (표준 메시지만) | 완전 (모든 메시지 라우팅) |
| **포트 구조** | Custom Library가 14550 직접 리스닝 | 라우터가 14550 리스닝 |
| **표준 메시지 경로** | QGC → Custom Library → 라우터 → FC | QGC → 라우터 → FC |
| **커스텀 메시지 경로** | QGC → Custom Library → 메인프로그램 | QGC → 라우터 → Custom Library → 메인프로그램 |
| **확장성** | 제한적 | 우수 (여러 애플리케이션 가능) |
| **유지보수성** | 양호 | 우수 (표준 패턴) |
| **설정 변경** | 불필요 | 필요 (라우터 설정) |

---

## 변경 사항 상세 (구현 가이드)

> **중요**: 다음 변경 사항들을 순서대로 정확히 따라야 합니다. 각 단계마다 검증 후 다음 단계로 진행하세요.

---

### 1. MAVLink 라우터 설정 변경

**파일 경로**: `/etc/mavlink-router/main.conf`

**작업 단계**:

1. **백업 생성**
   ```bash
   sudo cp /etc/mavlink-router/main.conf /etc/mavlink-router/main.conf.backup.$(date +%Y%m%d_%H%M%S)
   ```

2. **파일 편집**
   ```bash
   sudo nano /etc/mavlink-router/main.conf
   ```

3. **정확한 변경 내용**:

   **찾을 내용** (현재 설정):
   ```ini
   # GCS (QGroundControl) 브로드캐스트 - 네트워크 내 모든 QGC 수신 가능
   [UdpEndpoint GCS]
   Mode = Normal
   Address = 192.168.100.255
   Port = 14550

   # ROS2 노드 연결
   [UdpEndpoint ROS2]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14551
   ```

   **변경 후**:
   ```ini
   # GCS (QGroundControl) - Server 모드로 변경하여 14550 포트 직접 리스닝
   [UdpEndpoint GCS]
   Mode = Server
   Address = 0.0.0.0
   Port = 14550

   # 로컬 애플리케이션 엔드포인트 추가 (Custom Message Library용)
   [UdpEndpoint LocalApp]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14551

   # ROS2 노드 연결 (포트 변경: 14551 → 14552)
   [UdpEndpoint ROS2]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14552
   ```

4. **설정 검증**
   ```bash
   # 구문 검사
   sudo mavlink-routerd -c /etc/mavlink-router/main.conf --test
   ```

5. **라우터 재시작**
   ```bash
   sudo systemctl restart mavlink-router.service
   sudo systemctl status mavlink-router.service
   ```

6. **검증 방법**:
   - 라우터가 정상적으로 시작되었는지 확인: `sudo systemctl status mavlink-router.service`
   - 로그 확인: `sudo journalctl -u mavlink-router.service -n 50`
   - 포트 리스닝 확인: `sudo netstat -ulnp | grep mavlink-router` (14550, 14551, 14552 포트 확인)
   - QGC 연결 테스트: QGC에서 드론 연결 확인

---

### 2. Custom Message Library 수정

**파일 경로**: `custom_message/src/custom_message.cpp`

#### 2.1 생성자에서 MAVLink 라우터 소켓 관련 코드 제거

**위치**: `CustomMessageImpl` 생성자 (약 52-96번째 줄)

**찾을 코드** (약 68-95번째 줄):
```cpp
        , mavlink_router_socket_fd_(-1)
        ...
        // MAVLink 라우터로 표준 메시지 전달용 소켓 생성
        // MAVLink 라우터의 로컬 엔드포인트: 127.0.0.1:14550 (로컬 서버)
        // 또는 ROS2 엔드포인트: 127.0.0.1:14551 사용 가능
        mavlink_router_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (mavlink_router_socket_fd_ >= 0) {
            memset(&mavlink_router_addr_, 0, sizeof(mavlink_router_addr_));
            mavlink_router_addr_.sin_family = AF_INET;
            mavlink_router_addr_.sin_port = htons(14551);  // ROS2 엔드포인트 (로컬)
            if (inet_aton("127.0.0.1", &mavlink_router_addr_.sin_addr) == 0) {
                std::cerr << "[CustomMessage] MAVLink 라우터 주소 설정 실패" << std::endl;
                close(mavlink_router_socket_fd_);
                mavlink_router_socket_fd_ = -1;
            } else {
                std::cout << "[CustomMessage] MAVLink 라우터 전달 설정: 127.0.0.1:14551 (ROS2 엔드포인트)" << std::endl;
            }
        }
```

**변경**: **전체 삭제** (68번째 줄의 `mavlink_router_socket_fd_(-1),` 포함)

**변경 후**:
```cpp
        , send_socket_fd_(-1)
        , running_(false)
        , receive_thread_()
        , sequence_number_(0)
    {
        memset(&target_addr_, 0, sizeof(target_addr_));
        ...
    }
```

#### 2.2 소멸자에서 MAVLink 라우터 소켓 관련 코드 제거

**위치**: `~CustomMessageImpl()` 소멸자 (약 98-104번째 줄)

**찾을 코드**:
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

#### 2.3 `stop()` 함수에서 MAVLink 라우터 소켓 관련 코드 제거

**위치**: `stop()` 함수 (약 106-174번째 줄)

**찾을 코드** (stop() 함수 내부, 다른 close() 호출 근처):
```cpp
        if (mavlink_router_socket_fd_ >= 0) {
            close(mavlink_router_socket_fd_);
            mavlink_router_socket_fd_ = -1;
        }
```

**변경**: **해당 코드 블록 삭제**

#### 2.4 `parseMAVLinkMessage()` 함수에서 표준 메시지 전달 로직 제거

**위치**: `parseMAVLinkMessage()` 함수 내부 (약 478-502번째 줄)

**찾을 코드** (정확한 위치):
```cpp
        // 표준 MAVLink 메시지는 MAVLink 라우터로 전달
        // MAVLink 라우터가 자동으로 FC↔QGC 간 표준 메시지를 전달하므로
        // 이 프로그램은 커스텀 메시지(12900-12903)와 COMMAND_LONG(76)만 처리
        bool is_custom_message = (msg_id >= 12900 && msg_id <= 12903);
        bool is_command_long = (msg_id == 76);
        
        // 표준 메시지는 MAVLink 라우터의 FC 엔드포인트로 전달
        if (!is_custom_message && !is_command_long) {
            // 표준 메시지를 MAVLink 라우터로 전달 (127.0.0.1:14551)
            if (mavlink_router_socket_fd_ >= 0) {
                ssize_t sent = sendto(mavlink_router_socket_fd_, buffer, len, 0,
                                     (struct sockaddr*)&mavlink_router_addr_, sizeof(mavlink_router_addr_));
                if (sent < 0 && running_) {
                    static int forward_error_count = 0;
                    if (forward_error_count++ < 5) {
                        std::cerr << "[CustomMessage] 표준 메시지 전달 실패: " << strerror(errno) << std::endl;
                    }
                } else if (sent == len) {
                    static int forward_count = 0;
                    if (forward_count++ < 5) {
                        std::cout << "[CustomMessage] 표준 메시지 전달: MSG_ID=" << msg_id 
                                  << " -> MAVLink 라우터 (127.0.0.1:14551)" << std::endl;
                    }
                }
            }
            // 표준 메시지는 파싱하지 않고 전달만 함
            return;
        }
```

**변경 후**:
```cpp
        // 표준 MAVLink 메시지는 라우터가 이미 FC로 전달했으므로 여기서는 무시
        // MAVLink 라우터가 자동으로 FC↔QGC 간 표준 메시지를 전달하므로
        // 이 프로그램은 커스텀 메시지(12900-12903)와 COMMAND_LONG(76)만 처리
        bool is_custom_message = (msg_id >= 12900 && msg_id <= 12903);
        bool is_command_long = (msg_id == 76);
        
        // 표준 메시지는 라우터가 처리했으므로 파싱하지 않음
        if (!is_custom_message && !is_command_long) {
            return;
        }
```

#### 2.5 `parseMAVLinkMessage()` 함수에서 COMMAND_LONG 전달 로직 제거

**위치**: `parseMAVLinkMessage()` 함수 내부의 switch 문 (약 567-580번째 줄)

**찾을 코드**:
```cpp
            case 76:  // COMMAND_LONG (표준 MAVLink 메시지)
                parseCommandLong(payload, payload_len);
                // COMMAND_LONG도 MAVLink 라우터로 전달 (FC로 전달되어야 함)
                if (mavlink_router_socket_fd_ >= 0) {
                    ssize_t sent = sendto(mavlink_router_socket_fd_, buffer, len, 0,
                                         (struct sockaddr*)&mavlink_router_addr_, sizeof(mavlink_router_addr_));
                    if (sent == len) {
                        static int cmd_forward_count = 0;
                        if (cmd_forward_count++ < 5) {
                            std::cout << "[CustomMessage] COMMAND_LONG 전달: -> MAVLink 라우터 (127.0.0.1:14551)" << std::endl;
                        }
                    }
                }
                break;
```

**변경 후**:
```cpp
            case 76:  // COMMAND_LONG (표준 MAVLink 메시지)
                // COMMAND_LONG은 라우터가 이미 FC로 전달했으므로 파싱만 수행
                parseCommandLong(payload, payload_len);
                break;
```

#### 2.6 클래스 멤버 변수에서 제거

**위치**: `CustomMessageImpl` 클래스의 private 멤버 변수 섹션 (약 735-760번째 줄)

**찾을 코드**:
```cpp
    int receive_socket_fd_;
    int send_socket_fd_;
    int mavlink_router_socket_fd_;  // MAVLink 라우터로 표준 메시지 전달용
    ...
    struct sockaddr_in target_addr_;
    struct sockaddr_in mavlink_router_addr_;  // MAVLink 라우터 주소 (127.0.0.1:14540)
```

**변경 후**:
```cpp
    int receive_socket_fd_;
    int send_socket_fd_;
    ...
    struct sockaddr_in target_addr_;
```

#### 2.7 헤더 파일 기본값 변경 (선택 사항)

**파일 경로**: `custom_message/include/custom_message/custom_message.h`

**위치**: `CustomMessage` 생성자 선언 (약 84-91번째 줄)

**찾을 코드**:
```cpp
    CustomMessage(
        uint16_t receive_port = 14550,
        uint16_t send_port = 14550,
        const std::string& bind_address = "0.0.0.0",
        ...
    );
```

**변경 후** (선택 사항 - 기본값 변경):
```cpp
    CustomMessage(
        uint16_t receive_port = 14551,  // 기본값 변경: 라우터의 로컬 엔드포인트
        uint16_t send_port = 14550,
        const std::string& bind_address = "127.0.0.1",  // 기본값 변경: 로컬 바인드
        ...
    );
```

**참고**: 이 변경은 선택 사항입니다. Application Manager에서 명시적으로 포트를 지정하므로 필수는 아닙니다.

#### 2.8 컴파일 및 검증

```bash
cd /home/khadas/humiro_fire_suppression/custom_message
mkdir -p build && cd build
cmake ..
make

# 컴파일 오류 확인
# 경고는 무시 가능하지만, 오류는 반드시 수정 필요
```

**검증 방법**:
- 컴파일 성공 확인
- 링크 오류 없음 확인
- 생성된 라이브러리 파일 확인

---

### 3. Application Manager 수정

**파일 경로**: `application/src/application_manager.cpp`

**위치**: `ApplicationManager::initializeCustomMessage()` 함수 내부 (약 240-249번째 줄)

**찾을 코드** (정확한 위치):
```cpp
        // CustomMessage 생성
        custom_message_handler_ = new custom_message::CustomMessage(
            mavlink_port,  // 수신 포트
            mavlink_port,  // 송신 포트
            "0.0.0.0",  // 바인드 주소 (모든 인터페이스)
            target_address,  // 대상 주소 (QGC 또는 브로드캐스트)
            1,  // 시스템 ID
            1   // 컴포넌트 ID
        );
```

**변경 후**:
```cpp
        // CustomMessage 생성
        // MAVLink 라우터의 로컬 엔드포인트(14551)를 통해 메시지 수신
        // 라우터가 14550 포트를 리스닝하고, 커스텀 메시지를 14551로 전달
        custom_message_handler_ = new custom_message::CustomMessage(
            14551,  // 수신 포트: 라우터의 로컬 엔드포인트 (고정값)
            mavlink_port,  // 송신 포트: QGC로 전송 시 브로드캐스트 (14550)
            "127.0.0.1",  // 바인드 주소: 로컬 인터페이스만 (라우터에서 수신)
            target_address,  // 대상 주소: QGC 또는 브로드캐스트 (변경 없음)
            1,  // 시스템 ID
            1   // 컴포넌트 ID
        );
```

**검증**:
```bash
cd /home/khadas/humiro_fire_suppression/application
mkdir -p build && cd build
cmake ..
make
```

---

### 4. 문서 업데이트

**파일 경로**: `work-plan/019_MAVLINK_CUSTOM_MESSAGE.md`

**수정할 섹션**: "메시지 필터링 및 데이터 경로" (약 995번째 줄부터)

**변경 내용**:
1. 전체 아키텍처 다이어그램 업데이트
2. 포트 구성표 업데이트 (14551: 로컬 애플리케이션)
3. 메시지 경로 설명 업데이트

### 4. 문서 업데이트

**파일**: `work-plan/019_MAVLINK_CUSTOM_MESSAGE.md`

**변경 사항**:
- 데이터 경로 도식화 업데이트
- 메시지 필터링 정책 설명 업데이트
- 포트 구성표 업데이트

---

## 구현 계획 (단계별 실행 가이드)

> **중요**: 다음 단계를 **순서대로** 진행하고, 각 단계마다 **반드시 검증** 후 다음 단계로 진행하세요.

---

### Phase 1: 준비 작업

**목적**: 변경 전 상태 백업 및 테스트 환경 준비

1. **현재 코드 백업**
   ```bash
   cd /home/khadas/humiro_fire_suppression/custom_message/src
   cp custom_message.cpp custom_message.cpp.backup.$(date +%Y%m%d_%H%M%S)
   ```

2. **MAVLink 라우터 설정 백업**
   ```bash
   sudo cp /etc/mavlink-router/main.conf /etc/mavlink-router/main.conf.backup.$(date +%Y%m%d_%H%M%S)
   ```

3. **Application Manager 백업**
   ```bash
   cd /home/khadas/humiro_fire_suppression/application/src
   cp application_manager.cpp application_manager.cpp.backup.$(date +%Y%m%d_%H%M%S)
   ```

4. **현재 동작 확인** (변경 전 기준점 설정)
   - QGC 연결 확인
   - 표준 메시지 테스트 (ARM/DISARM)
   - 커스텀 메시지 테스트

**검증**: 모든 백업 파일이 생성되었는지 확인

---

### Phase 2: MAVLink 라우터 설정 변경

**목적**: 라우터가 14550 포트를 직접 리스닝하도록 변경

**위치**: "변경 사항 상세" 섹션의 "1. MAVLink 라우터 설정 변경" 참조

**검증 방법**:
1. 라우터 서비스 상태 확인
   ```bash
   sudo systemctl status mavlink-router.service
   # 상태가 "active (running)"이어야 함
   ```

2. 포트 리스닝 확인
   ```bash
   sudo netstat -ulnp | grep mavlink-router
   # 출력에 14550, 14551, 14552 포트가 있어야 함
   ```

3. 로그 확인
   ```bash
   sudo journalctl -u mavlink-router.service -n 50 --no-pager
   # 오류 메시지가 없어야 함
   ```

4. QGC 연결 테스트
   - QGC에서 드론 연결 시도
   - HEARTBEAT 메시지 수신 확인
   - **이 단계에서 QGC 연결이 안 되면 롤백하고 문제 해결 후 진행**

---

### Phase 3: Custom Message Library 수정

**목적**: 라우터의 로컬 엔드포인트(14551)를 통해 메시지 수신하도록 변경

**위치**: "변경 사항 상세" 섹션의 "2. Custom Message Library 수정" 참조

**수정 항목 체크리스트**:
- [ ] 2.1 생성자에서 `mavlink_router_socket_fd_` 관련 코드 제거
- [ ] 2.2 소멸자에서 `mavlink_router_socket_fd_` 관련 코드 제거
- [ ] 2.3 `stop()` 함수에서 `mavlink_router_socket_fd_` 관련 코드 제거
- [ ] 2.4 `parseMAVLinkMessage()`에서 표준 메시지 전달 로직 제거 (482-502줄)
- [ ] 2.5 `parseMAVLinkMessage()`에서 COMMAND_LONG 전달 로직 제거 (569-579줄)
- [ ] 2.6 클래스 멤버 변수에서 `mavlink_router_socket_fd_`, `mavlink_router_addr_` 제거

**컴파일 및 검증**:
```bash
cd /home/khadas/humiro_fire_suppression/custom_message
mkdir -p build && cd build
cmake ..
make 2>&1 | tee compile.log
```

**검증 방법**:
1. 컴파일 성공 확인 (오류 없음)
2. 생성된 라이브러리 파일 확인
   ```bash
   ls -lh libcustom_message.a  # 또는 .so 파일
   ```
3. 컴파일 경고 확인 (심각한 경고는 수정)

**중요**: 이 단계에서 컴파일 오류가 발생하면 이전 단계로 롤백

---

### Phase 4: Application Manager 수정

**목적**: CustomMessage 생성 시 포트 변경

**위치**: "변경 사항 상세" 섹션의 "3. Application Manager 수정" 참조

**컴파일 및 검증**:
```bash
cd /home/khadas/humiro_fire_suppression/application
mkdir -p build && cd build
cmake ..
make 2>&1 | tee compile.log
```

**검증 방법**:
1. 컴파일 성공 확인
2. 실행 파일 생성 확인

---

### Phase 5: 통합 테스트

**목적**: 변경 후 전체 시스템 동작 확인

#### 5.1 표준 메시지 테스트 (메인프로그램 실행 전)

1. **메인프로그램 중지** (있는 경우)
   ```bash
   # 실행 중인 메인프로그램 중지
   ```

2. **QGC 연결 테스트**
   - QGC에서 드론 연결
   - HEARTBEAT 메시지 수신 확인
   - **기대 결과**: QGC가 드론을 인식하고 상태 정보 표시

3. **ARM/DISARM 명령 테스트**
   - QGC에서 ARM 명령 전송
   - **기대 결과**: FC가 ARM되고 QGC에 상태 반영
   - DISARM 명령도 동일하게 테스트

4. **SET_MODE 명령 테스트**
   - QGC에서 비행 모드 변경
   - **기대 결과**: FC 모드 변경 확인

**검증**: 메인프로그램 없이도 표준 메시지가 정상 작동하는지 확인

#### 5.2 커스텀 메시지 테스트 (메인프로그램 실행 후)

1. **메인프로그램 시작**
   ```bash
   # Application Manager 실행
   ```

2. **로그 확인**
   ```bash
   # CustomMessage 시작 로그 확인
   # "메시지 송수신 시작 (수신 포트: 14551, ..." 메시지 확인
   ```

3. **커스텀 메시지 수신 테스트**
   - QGC 또는 테스트 프로그램에서 커스텀 메시지 전송
   - **기대 결과**: 메인프로그램에서 메시지 수신 및 콜백 호출 확인

4. **표준 메시지와 동시 테스트**
   - 표준 메시지(ARM)와 커스텀 메시지 동시 전송
   - **기대 결과**: 둘 다 정상 작동

#### 5.3 에러 처리 테스트

1. **라우터 중지 시나리오**
   ```bash
   sudo systemctl stop mavlink-router.service
   ```
   - **기대 결과**: 
     - 표준 메시지 전송 실패 (정상)
     - 커스텀 메시지 전송 실패 (정상)
     - 에러 로그 확인

2. **라우터 재시작**
   ```bash
   sudo systemctl start mavlink-router.service
   ```
   - **기대 결과**: 정상 복구

---

### Phase 6: 문서 업데이트

**위치**: "변경 사항 상세" 섹션의 "4. 문서 업데이트" 참조

**수정 파일**: `work-plan/019_MAVLINK_CUSTOM_MESSAGE.md`

**업데이트 내용**:
1. 아키텍처 다이어그램 업데이트
2. 포트 구성표 업데이트
3. 메시지 경로 설명 업데이트
4. 변경 사항 요약 추가

---

## 검증 체크리스트

변경 완료 후 다음 항목을 모두 확인하세요:

### 필수 검증 항목

- [ ] MAVLink 라우터가 정상 실행 중 (3개 포트 리스닝: 14550, 14551, 14552)
- [ ] QGC에서 드론 연결 성공 (메인프로그램 없이도 가능)
- [ ] 표준 메시지(ARM/DISARM) 정상 작동 (메인프로그램 없이도 가능)
- [ ] 커스텀 메시지 정상 수신 (메인프로그램 실행 시)
- [ ] 컴파일 오류 없음
- [ ] 런타임 오류 없음 (로그 확인)

### 추가 검증 항목

- [ ] ROS2 노드도 정상 작동하는지 확인 (14552 포트 사용)
- [ ] 성능 테스트 (메시지 지연 시간 확인)
- [ ] 장시간 동작 테스트 (메모리 누수 확인)

---

## 문제 해결 가이드

### 문제 1: 라우터 재시작 실패

**증상**: `sudo systemctl restart mavlink-router.service` 실패

**해결**:
```bash
# 설정 파일 구문 검사
sudo mavlink-routerd -c /etc/mavlink-router/main.conf --test

# 로그 확인
sudo journalctl -u mavlink-router.service -n 100

# 백업으로 복원
sudo cp /etc/mavlink-router/main.conf.backup.* /etc/mavlink-router/main.conf
```

### 문제 2: 컴파일 오류 (mavlink_router_socket_fd_ 관련)

**증상**: `mavlink_router_socket_fd_` 관련 컴파일 오류

**해결**:
- 모든 `mavlink_router_socket_fd_` 참조 제거 확인
- 모든 `mavlink_router_addr_` 참조 제거 확인
- 멤버 변수 선언 확인

### 문제 3: QGC 연결 실패

**증상**: 라우터 변경 후 QGC에서 드론 연결 안 됨

**해결**:
1. 라우터 서비스 상태 확인
2. 포트 14550 리스닝 확인
3. 방화벽 확인
4. 네트워크 연결 확인

### 문제 4: 커스텀 메시지 수신 안 됨

**증상**: 메인프로그램 실행 후 커스텀 메시지 수신 안 됨

**해결**:
1. 라우터가 14551 포트로 메시지 전송하는지 확인
2. CustomMessage가 14551 포트 리스닝하는지 확인
3. 로그에서 메시지 수신 로그 확인
4. 네트워크 인터페이스 확인 (127.0.0.1)

---

## 롤백 절차

문제 발생 시 즉시 롤백:

### 1단계: 라우터 설정 복원
```bash
sudo cp /etc/mavlink-router/main.conf.backup.* /etc/mavlink-router/main.conf
sudo systemctl restart mavlink-router.service
sudo systemctl status mavlink-router.service
```

### 2단계: Custom Message Library 복원
```bash
cd /home/khadas/humiro_fire_suppression/custom_message/src
cp custom_message.cpp.backup.* custom_message.cpp
cd ../build
make
```

### 3단계: Application Manager 복원
```bash
cd /home/khadas/humiro_fire_suppression/application/src
cp application_manager.cpp.backup.* application_manager.cpp
cd ../build
make
```

### 4단계: 동작 확인
- QGC 연결 확인
- 표준/커스텀 메시지 테스트

---

## 예상 효과

### 긍정적 효과

1. **표준 준수**
   - MAVLink 표준 아키텍처 준수로 호환성 향상
   - 다른 MAVLink 시스템과의 통합 용이

2. **확장성 향상**
   - 여러 애플리케이션이 라우터의 다른 로컬 엔드포인트 사용 가능
   - ROS2 노드도 별도 포트로 메시지 수신 가능

3. **유지보수성 향상**
   - 표준 패턴이므로 이해 및 수정이 쉬움
   - 새로운 개발자 온보딩 용이

4. **성능 개선**
   - 표준 메시지가 중간 단계 없이 직접 라우팅
   - 지연 시간 감소

### 주의 사항

1. **마이그레이션 기간**
   - 기존 시스템과의 호환성 확인 필요
   - 단계적 배포 권장

2. **테스트 필요**
   - 모든 메시지 타입에 대한 테스트 필요
   - 실제 비행 테스트 전 시뮬레이션 테스트 권장

---

## 롤백 계획

문제 발생 시 롤백 절차:

1. **MAVLink 라우터 설정 복원**
   ```bash
   sudo cp /etc/mavlink-router/main.conf.backup /etc/mavlink-router/main.conf
   sudo systemctl restart mavlink-router.service
   ```

2. **코드 복원**
   ```bash
   cd custom_message/src
   cp custom_message.cpp.backup custom_message.cpp
   # 재컴파일
   ```

3. **Application Manager 복원**
   - 이전 버전으로 롤백

---

## 참고 자료

- [MAVLink Router 공식 문서](https://github.com/mavlink-router/mavlink-router)
- [MAVLink 프로토콜 문서](https://mavlink.io/)
- [커스텀 MAVLink 메시지 설계 가이드](./019_MAVLINK_CUSTOM_MESSAGE.md)

---

---

## 요약

이 문서는 MAVLink 아키텍처를 표준 방식으로 변경하는 상세 구현 가이드입니다. 

**핵심 변경 사항**:
1. MAVLink 라우터가 14550 포트를 직접 리스닝 (Server 모드)
2. Custom Message Library가 라우터의 로컬 엔드포인트(14551)를 통해 메시지 수신
3. 표준 메시지는 라우터가 자동으로 FC로 전달 (Custom Library 거치지 않음)
4. 커스텀 메시지는 라우터 → Custom Library → 메인프로그램 경로

**구현 시 주의사항**:
- 순서대로 단계별 진행 (Phase 1 → 2 → 3 → 4 → 5 → 6)
- 각 단계마다 반드시 검증 후 다음 단계 진행
- 문제 발생 시 즉시 롤백

**AI 구현 시 참고**:
- 이 문서의 "변경 사항 상세" 섹션에 정확한 코드 위치와 변경 내용이 명시되어 있음
- 각 코드 블록의 "찾을 코드"와 "변경 후"를 정확히 따라야 함
- 컴파일 오류 발생 시 체크리스트 확인 후 누락된 부분 확인

---

**작성자**: Humiro Fire Suppression Team  
**버전**: v2.0 (상세 구현 가이드)  
**작성일**: 2026-01-08  
**최종 수정일**: 2026-01-08  
**상태**: 계획 단계 (구현 대기)
