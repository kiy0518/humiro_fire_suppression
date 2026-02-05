/**
 * @file custom_message.cpp
 * @brief 커스텀 MAVLink 메시지 송수신 라이브러리 구현
 */

#include "custom_message/custom_message.h"
#include <iostream>
#include <cstring>
#include <exception>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <queue>
#include <condition_variable>
#include <unordered_map>

namespace custom_message {

// MAVLink 헤더 구조체 (패딩 없음)
struct __attribute__((packed)) MAVLinkHeader {
    uint8_t magic;          // 0xFD (MAVLink 2.0)
    uint8_t len;            // 페이로드 길이
    uint8_t incompat_flags; // 호환성 플래그
    uint8_t compat_flags;   // 호환성 플래그
    uint8_t seq;            // 시퀀스 번호
    uint8_t sysid;          // 시스템 ID
    uint8_t compid;         // 컴포넌트 ID
    uint8_t msgid;          // 메시지 ID (하위 8비트)
    uint8_t msgid_ext[2];   // 메시지 ID 확장 (상위 16비트)
};

// MAVLink 2.0 메시지 최대 크기
constexpr size_t MAVLINK_MAX_PACKET_LEN = 280;
constexpr size_t MAVLINK_HEADER_LEN = 10;
constexpr size_t MAVLINK_CHECKSUM_LEN = 2;
constexpr uint8_t MAVLINK_MAGIC = 0xFD;

// 간단한 CRC16 계산 (MAVLink 2.0)
uint16_t calculateCRC16(const uint8_t* data, size_t len, uint16_t initial_crc = 0xFFFF) {
    uint16_t crc = initial_crc;

    for (size_t i = 0; i < len; i++) {
        uint8_t tmp = data[i] ^ (crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    return crc;
}

class CustomMessageImpl {
public:
    CustomMessageImpl(
        uint16_t receive_port,
        uint16_t send_port,
        const std::string& bind_address,
        const std::string& target_address,
        uint8_t system_id,
        uint8_t component_id
    )
        : receive_port_(receive_port)
        , send_port_(send_port)
        , bind_address_(bind_address)
        , target_address_(target_address)
        , system_id_(system_id)
        , component_id_(component_id)
        , receive_socket_fd_(-1)
        , send_socket_fd_(-1)
        , epoll_fd_(-1)
        , running_(false)
        , receive_thread_()
        , sequence_number_(0)
    {
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(send_port_);
        if (inet_aton(target_address_.c_str(), &target_addr_.sin_addr) == 0) {
            std::cerr << "[CustomMessage] 잘못된 대상 주소: " << target_address_ << std::endl;
        }
    }

    ~CustomMessageImpl() {
        stop();
    }

    bool start() {
        if (running_) {
            return true;
        }

        // 소켓 생성 (최대 3회 재시도)
        for (int attempt = 1; attempt <= 3; attempt++) {
            receive_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
            if (receive_socket_fd_ >= 0) break;
            std::cerr << "[CustomMessage] 수신 소켓 생성 실패 (시도 " << attempt << "/3): "
                      << strerror(errno) << std::endl;
            if (attempt < 3) std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (receive_socket_fd_ < 0) {
            std::cerr << "[CustomMessage] 수신 소켓 생성 최종 실패" << std::endl;
            return false;
        }

        for (int attempt = 1; attempt <= 3; attempt++) {
            send_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
            if (send_socket_fd_ >= 0) break;
            std::cerr << "[CustomMessage] 송신 소켓 생성 실패 (시도 " << attempt << "/3): "
                      << strerror(errno) << std::endl;
            if (attempt < 3) std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (send_socket_fd_ < 0) {
            std::cerr << "[CustomMessage] 송신 소켓 생성 최종 실패" << std::endl;
            close(receive_socket_fd_);
            receive_socket_fd_ = -1;
            return false;
        }

        // 소켓 옵션 설정 (재사용 허용)
        int reuse = 1;
        if (setsockopt(receive_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            std::cerr << "[CustomMessage] SO_REUSEADDR 설정 실패: " << strerror(errno) << std::endl;
            close(receive_socket_fd_);
            close(send_socket_fd_);
            receive_socket_fd_ = -1;
            send_socket_fd_ = -1;
            return false;
        }
        
        // 수신 버퍼 크기 증가 (메시지 손실 방지)
        // 기본값은 보통 212992 bytes (약 208KB)이지만, 더 크게 설정하여 메시지 손실 방지
        int recv_buf_size = 2 * 1024 * 1024;  // 1MB → 2MB로 증가 (더 많은 메시지 버퍼링)
        if (setsockopt(receive_socket_fd_, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size)) < 0) {
            std::cerr << "[CustomMessage] SO_RCVBUF 설정 실패: " << strerror(errno) << std::endl;
            // 경고만 출력하고 계속 진행 (기본값 사용)
        } else {
            std::cout << "[CustomMessage] 수신 버퍼 크기 설정: " << recv_buf_size << " bytes (2MB)" << std::endl;
        }
        
        // 송신 버퍼 크기도 증가 (일관성 유지)
        int send_buf_size = 2 * 1024 * 1024;  // 2MB
        if (setsockopt(send_socket_fd_, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size)) < 0) {
            std::cerr << "[CustomMessage] SO_SNDBUF 설정 실패: " << strerror(errno) << std::endl;
        } else {
            std::cout << "[CustomMessage] 송신 버퍼 크기 설정: " << send_buf_size << " bytes (2MB)" << std::endl;
        }
        
        // Non-blocking 모드 설정 (메시지 손실 방지)
        // Non-blocking으로 설정하면 recvfrom()이 즉시 반환되어 파싱 중에도 메시지 수신 가능
        int flags = fcntl(receive_socket_fd_, F_GETFL, 0);
        if (flags < 0) {
            std::cerr << "[CustomMessage] F_GETFL 실패: " << strerror(errno) << std::endl;
        } else {
            if (fcntl(receive_socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
                std::cerr << "[CustomMessage] O_NONBLOCK 설정 실패: " << strerror(errno) << std::endl;
                // 경고만 출력하고 계속 진행 (블로킹 모드 사용)
            } else {
                std::cout << "[CustomMessage] Non-blocking 모드 활성화 (메시지 손실 방지)" << std::endl;
            }
        }

        // 수신 소켓 바인드
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(receive_port_);

        if (bind_address_ == "0.0.0.0") {
            addr.sin_addr.s_addr = INADDR_ANY;
        } else {
            if (inet_aton(bind_address_.c_str(), &addr.sin_addr) == 0) {
                std::cerr << "[CustomMessage] 잘못된 바인드 주소: " << bind_address_ << std::endl;
                close(receive_socket_fd_);
                close(send_socket_fd_);
                receive_socket_fd_ = -1;
                send_socket_fd_ = -1;
                return false;
            }
        }

        if (bind(receive_socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "[CustomMessage] 바인드 실패: " << strerror(errno) << std::endl;
            close(receive_socket_fd_);
            close(send_socket_fd_);
            receive_socket_fd_ = -1;
            send_socket_fd_ = -1;
            return false;
        }

        // epoll 생성
        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ < 0) {
            std::cerr << "[CustomMessage] epoll 생성 실패: " << strerror(errno) << std::endl;
            close(receive_socket_fd_);
            close(send_socket_fd_);
            receive_socket_fd_ = -1;
            send_socket_fd_ = -1;
            return false;
        }
        
        // 수신 소켓을 epoll에 등록
        struct epoll_event ev;
        ev.events = EPOLLIN | EPOLLET;  // Edge-triggered 모드 (더 효율적)
        ev.data.fd = receive_socket_fd_;
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, receive_socket_fd_, &ev) < 0) {
            std::cerr << "[CustomMessage] epoll_ctl 실패: " << strerror(errno) << std::endl;
            close(epoll_fd_);
            epoll_fd_ = -1;
            close(receive_socket_fd_);
            close(send_socket_fd_);
            receive_socket_fd_ = -1;
            send_socket_fd_ = -1;
            return false;
        }
        
        running_ = true;
        receive_thread_ = std::thread(&CustomMessageImpl::receiveLoop, this);
        process_thread_ = std::thread(&CustomMessageImpl::processLoop, this);

        std::cout << "[CustomMessage] 메시지 송수신 시작 (표준 MAVLink 아키텍처, epoll 기반 이벤트 처리)" << std::endl;
        std::cout << "[CustomMessage]   - 수신 포트: " << receive_port_ << " (라우터의 로컬 엔드포인트)" << std::endl;
        std::cout << "[CustomMessage]   - 송신 포트: " << send_port_ << " (QGC 브로드캐스트)" << std::endl;
        std::cout << "[CustomMessage]   - 바인드 주소: " << bind_address_ << std::endl;
        std::cout << "[CustomMessage]   - 대상 주소: " << target_address_ << std::endl;
        return true;
    }

    void stop() {
        if (!running_) {
            return;
        }

        running_ = false;

        if (epoll_fd_ >= 0) {
            close(epoll_fd_);
            epoll_fd_ = -1;
        }
        
        if (receive_socket_fd_ >= 0) {
            close(receive_socket_fd_);
            receive_socket_fd_ = -1;
        }

        if (send_socket_fd_ >= 0) {
            close(send_socket_fd_);
            send_socket_fd_ = -1;
        }

        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        
        if (process_thread_.joinable()) {
            process_thread_.join();
        }

        std::cout << "[CustomMessage] 메시지 송수신 중지" << std::endl;
    }

    bool isRunning() const {
        return running_;
    }

    void setFireMissionStartCallback(FireMissionStartCallback callback) {
        mission_start_callback_ = callback;
    }

    void setFireAutoAimCallback(FireAutoAimCallback callback) {
        auto_aim_callback_ = callback;
    }

    void setFireLaunchCallback(FireLaunchCallback callback) {
        launch_callback_ = callback;
    }

    void setFireReturnCallback(FireReturnCallback callback) {
        return_callback_ = callback;
    }

    void setCommandLongCallback(CommandLongCallback callback) {
        command_long_callback_ = callback;
    }

    void setSetModeCallback(SetModeCallback callback) {
        set_mode_callback_ = callback;
    }

    bool sendFireMissionStart(const FireMissionStart& start) {
        return sendMessage(MessageType::FIRE_MISSION_START, &start, sizeof(FireMissionStart));
    }

    bool sendFireAutoAim(const FireAutoAim& aim) {
        return sendMessage(MessageType::FIRE_AUTO_AIM, &aim, sizeof(FireAutoAim));
    }

    bool sendFireLaunch(const FireLaunch& launch) {
        return sendMessage(MessageType::FIRE_LAUNCH, &launch, sizeof(FireLaunch));
    }

    bool sendFireReturn(const FireReturn& ret) {
        return sendMessage(MessageType::FIRE_RETURN, &ret, sizeof(FireReturn));
    }

    void setTargetAddress(const std::string& address, uint16_t port) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_address_ = address;
        send_port_ = port;
        target_addr_.sin_port = htons(port);
        if (inet_aton(address.c_str(), &target_addr_.sin_addr) == 0) {
            std::cerr << "[CustomMessage] 잘못된 대상 주소: " << address << std::endl;
        }
    }

    CustomMessage::Statistics getStatistics() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return stats_;
    }

    void resetStatistics() {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = CustomMessage::Statistics();
    }

private:
    bool sendMessage(MessageType msg_type, const void* payload, size_t payload_len) {
        if (send_socket_fd_ < 0 || !running_) {
            stats_.send_error_count++;
            return false;
        }

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        size_t total_len = MAVLINK_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN;

        if (total_len > MAVLINK_MAX_PACKET_LEN) {
            stats_.send_error_count++;
            return false;
        }

        // MAVLink 헤더 구성
        MAVLinkHeader* header = reinterpret_cast<MAVLinkHeader*>(buffer);
        header->magic = MAVLINK_MAGIC;
        header->len = payload_len;
        header->incompat_flags = 0;
        header->compat_flags = 0;
        header->seq = sequence_number_++;
        header->sysid = system_id_;
        header->compid = component_id_;

        uint32_t msg_id = static_cast<uint32_t>(msg_type);
        header->msgid = msg_id & 0xFF;
        header->msgid_ext[0] = (msg_id >> 8) & 0xFF;
        header->msgid_ext[1] = (msg_id >> 16) & 0xFF;

        // 페이로드 복사
        memcpy(buffer + MAVLINK_HEADER_LEN, payload, payload_len);

        // CRC_EXTRA 값 (메시지 ID별로 다름)
        // NOTE: 60000번대 사용 (50001/50002는 CUBEPILOT/HERELINK와 충돌)
        uint8_t crc_extra = 0;
        switch (msg_id) {
            case 60000: crc_extra = 100; break;  // FIRE_MISSION_START (미션 스타트)
            case 60001: crc_extra = 101; break;  // AUTO_AIM (자동조준)
            case 60002: crc_extra = 102; break;  // FIRE_COMMAND (발사)
            case 60003: crc_extra = 103; break;  // RETURN_TO_LAUNCH (복귀)
            default: crc_extra = 0; break;
        }

        // 체크섬 계산 (헤더 + 페이로드 + CRC_EXTRA)
        uint16_t crc = calculateCRC16(buffer + 1, MAVLINK_HEADER_LEN - 1 + payload_len);
        // CRC_EXTRA 추가
        uint8_t crc_extra_byte = crc_extra;
        crc = calculateCRC16(&crc_extra_byte, 1, crc);
        buffer[MAVLINK_HEADER_LEN + payload_len] = crc & 0xFF;
        buffer[MAVLINK_HEADER_LEN + payload_len + 1] = (crc >> 8) & 0xFF;

        // UDP 전송
        std::lock_guard<std::mutex> lock(target_mutex_);
        ssize_t sent = sendto(send_socket_fd_, buffer, total_len, 0,
                              (struct sockaddr*)&target_addr_, sizeof(target_addr_));

        if (sent < 0) {
            stats_.send_error_count++;
            return false;
        }

        // 통계 업데이트
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        switch (msg_type) {
            case MessageType::FIRE_MISSION_START:
                stats_.mission_start_sent++;
                break;
            case MessageType::FIRE_AUTO_AIM:
                stats_.auto_aim_sent++;
                break;
            case MessageType::FIRE_LAUNCH:
                stats_.launch_sent++;
                break;
            case MessageType::FIRE_RETURN:
                stats_.return_sent++;
                break;
            default:
                break;
        }

        return true;
    }

    void receiveLoop() {
        const int MAX_EVENTS = 50;  // 10 → 50으로 증가 (한 번에 더 많은 이벤트 처리)
        struct epoll_event events[MAX_EVENTS];
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        static int receive_count = 0;

        while (running_) {
            // epoll_wait로 이벤트 대기 (타임아웃: 50ms로 단축 - 더 빠른 반응)
            // 데이터가 도착했을 때만 깨어남 (CPU 효율적)
            int nfds = epoll_wait(epoll_fd_, events, MAX_EVENTS, 50);
            
            if (nfds < 0) {
                if (errno == EINTR) {
                    continue;  // 인터럽트 시 재시도
                }
                if (running_) {
                    std::cerr << "[CustomMessage] epoll_wait 오류: " << strerror(errno) << std::endl;
                }
                continue;
            }
            
            if (nfds == 0) {
                // 타임아웃 (정상, 계속 대기)
                continue;
            }
            
            // 이벤트 처리 (Edge-triggered 모드이므로 모든 데이터를 읽어야 함)
            for (int i = 0; i < nfds; i++) {
                if (events[i].data.fd == receive_socket_fd_) {
                    // Edge-triggered 모드: 가능한 모든 데이터를 읽어야 함
                    while (running_) {
                        ssize_t received = recvfrom(receive_socket_fd_, buffer, sizeof(buffer), 0,
                                                   (struct sockaddr*)&client_addr, &client_addr_len);
                        
                        if (received < 0) {
                            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                // 더 이상 읽을 데이터가 없음 (정상)
                                break;
                            } else if (errno == EINTR) {
                                continue;  // 인터럽트 시 재시도
                            } else {
                                if (running_) {
                                    std::cerr << "[CustomMessage] 수신 오류: " << strerror(errno) << std::endl;
                                }
                                break;
                            }
                        }
                        
                        if (received == 0) {
                            // 빈 패킷
                            continue;
                        }
                        
                        receive_count++;
                        // 디버그 출력: 커스텀 메시지(60000-60003)와 COMMAND_LONG(76)은 항상 출력
                        // 60000: 미션스타트, 60001: 자동조준, 60002: 발사, 60003: 복귀
                        if (buffer[0] == 0xFD && static_cast<size_t>(received) >= MAVLINK_HEADER_LEN) {
                            // 헤더에서 MSG_ID 확인 (buffer[7] = msgid, buffer[8] = msgid_ext[0], buffer[9] = msgid_ext[1])
                            uint32_t msg_id_check = buffer[7] | (static_cast<uint32_t>(buffer[8]) << 8) |
                                                   (static_cast<uint32_t>(buffer[9]) << 16);
                            bool is_important = (msg_id_check == 76) || (msg_id_check >= 60000 && msg_id_check <= 60003);
                            if (is_important || receive_count <= 10) {
                                std::cout << "[CustomMessage] [STEP 1] UDP 수신 (epoll): " << received << " bytes, "
                                          << "첫 바이트: 0x" << std::hex << static_cast<int>(buffer[0]) 
                                          << std::dec << ", MSG_ID=" << msg_id_check << ", 포트: " << receive_port_ 
                                          << ", 큐 크기: " << message_queue_.size() << std::endl;
                            }
                        } else if (receive_count <= 10) {
                            std::cout << "[CustomMessage] [STEP 1] UDP 수신 (epoll): " << received << " bytes, "
                                      << "첫 바이트: 0x" << std::hex << static_cast<int>(buffer[0]) 
                                      << std::dec << ", 포트: " << receive_port_ << std::endl;
                        }
                        
                        // 통계 업데이트 (수신 성공)
                        {
                            std::lock_guard<std::mutex> lock(stats_mutex_);
                            stats_.messages_received++;
                        }

                        // received는 양수이므로 size_t로 캐스팅하여 비교 (경고 해결)
                        if (static_cast<size_t>(received) < MAVLINK_HEADER_LEN) {
                            std::lock_guard<std::mutex> lock(stats_mutex_);
                            stats_.parse_error_count++;
                            if (receive_count <= 5) {
                                std::cerr << "[CustomMessage] 헤더 길이 부족: " << received << " < " << MAVLINK_HEADER_LEN << std::endl;
                            }
                            continue;
                        }

                        // 메시지를 큐에 넣어서 별도 스레드에서 처리 (메시지 손실 방지)
                        {
                            std::lock_guard<std::mutex> lock(message_queue_mutex_);
                            const size_t MAX_QUEUE_SIZE = 5000;  // 1000 → 5000으로 증가 (메시지 손실 방지)
                            if (message_queue_.size() >= MAX_QUEUE_SIZE) {
                                // 오래된 메시지 드롭 (front drop) - 새 메시지 우선
                                message_queue_.pop();
                                {
                                    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                                    stats_.queue_drop_count++;
                                    if (stats_.queue_drop_count % 100 == 1) {
                                        std::cerr << "[CustomMessage] 큐 오버플로우: 오래된 메시지 드롭 (총 "
                                                  << stats_.queue_drop_count << "건)" << std::endl;
                                    }
                                }
                            }
                            std::vector<uint8_t> msg(buffer, buffer + received);
                            message_queue_.push(std::move(msg));
                            message_queue_cv_.notify_one();
                        }
                    }
                }
            }
        }
    }

    void processLoop() {
        // 메시지 큐에서 메시지를 꺼내서 파싱 및 콜백 호출
        // 배치 처리로 성능 향상: 여러 메시지를 한 번에 처리
        const size_t BATCH_SIZE = 10;  // 한 번에 처리할 메시지 수
        std::vector<std::vector<uint8_t>> batch;
        batch.reserve(BATCH_SIZE);
        
        while (running_) {
            // 배치 수집
            {
                std::unique_lock<std::mutex> lock(message_queue_mutex_);
                // 큐에 메시지가 있거나 running_이 false가 될 때까지 대기
                message_queue_cv_.wait(lock, [this] { 
                    return !message_queue_.empty() || !running_; 
                });
                
                if (!running_ && message_queue_.empty()) {
                    break;
                }
                
                // 배치 크기만큼 또는 큐가 빌 때까지 메시지 수집
                batch.clear();
                while (batch.size() < BATCH_SIZE && !message_queue_.empty()) {
                    batch.push_back(std::move(message_queue_.front()));
                    message_queue_.pop();
                }
            }
            
            // 배치 처리: 여러 메시지를 순차적으로 처리
            for (auto& msg : batch) {
                if (!msg.empty()) {
                    try {
                        parseMAVLinkMessage(msg.data(), msg.size());
                    } catch (const std::exception& e) {
                        std::lock_guard<std::mutex> lock(stats_mutex_);
                        stats_.parse_error_count++;
                        static int exception_count = 0;
                        exception_count++;
                        if (exception_count <= 10) {  // 5 → 10으로 증가
                            std::cerr << "[CustomMessage] 메시지 파싱 예외 발생 (무시하고 계속): " << e.what() << std::endl;
                        }
                    } catch (...) {
                        std::lock_guard<std::mutex> lock(stats_mutex_);
                        stats_.parse_error_count++;
                        static int unknown_exception_count = 0;
                        unknown_exception_count++;
                        if (unknown_exception_count <= 10) {  // 5 → 10으로 증가
                            std::cerr << "[CustomMessage] 알 수 없는 예외 발생 (무시하고 계속)" << std::endl;
                        }
                    }
                }
            }
            
            // 배치 처리 후 짧은 지연 (너무 빠른 처리로 인한 CPU 과부하 방지)
            if (batch.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    // 표준 MAVLink 메시지를 mavlink-router(14550)로 포워딩 → FC로 전달
    void forwardToFC(const uint8_t* buffer, size_t len) {
        if (send_socket_fd_ < 0 || !running_ || len == 0) return;

        std::lock_guard<std::mutex> lock(target_mutex_);
        ssize_t sent = sendto(send_socket_fd_, buffer, len, 0,
                              (struct sockaddr*)&target_addr_, sizeof(target_addr_));
        if (sent > 0) {
            std::cout << "[CustomMessage] → FC 포워딩 완료: " << len << " bytes → "
                      << target_address_ << ":" << send_port_ << std::endl;
        } else {
            std::cerr << "[CustomMessage] ✗ FC 포워딩 실패: " << strerror(errno) << std::endl;
        }
    }

    void parseMAVLinkMessage(const uint8_t* buffer, size_t len) {
        // 안전성 검증: 버퍼가 nullptr이거나 길이가 0이면 즉시 반환
        if (!buffer || len == 0) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        // 디버그: 수신된 메시지의 첫 바이트 확인
        if (buffer[0] != MAVLINK_MAGIC) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            if (stats_.parse_error_count % 10 == 1) {  // 10개마다 한 번씩만 출력
                std::cerr << "[CustomMessage] 잘못된 MAVLink magic: 0x" 
                          << std::hex << static_cast<int>(buffer[0]) 
                          << " (예상: 0x" << static_cast<int>(MAVLINK_MAGIC) << ")" << std::dec << std::endl;
            }
            return;
        }

        // 최소 길이 검증 (헤더 + 최소 페이로드 + CRC)
        if (len < MAVLINK_HEADER_LEN + MAVLINK_CHECKSUM_LEN) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        const MAVLinkHeader* header = reinterpret_cast<const MAVLinkHeader*>(buffer);
        size_t payload_len = header->len;

        // 페이로드 길이 검증 (헤더 + 페이로드 + CRC)
        if (len < MAVLINK_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        uint32_t msg_id = header->msgid | (static_cast<uint32_t>(header->msgid_ext[0]) << 8) |
                         (static_cast<uint32_t>(header->msgid_ext[1]) << 16);

        const uint8_t* payload = buffer + MAVLINK_HEADER_LEN;

        // MSG_ID=0인 메시지는 무시 (잘못된 메시지)
        if (msg_id == 0) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        // 중복 메시지 방지: 커스텀 메시지(60001~60003)에 대해 sysid별 seq 추적
        // 60000(미션 시작)은 중복 허용 - 비행 중 목표 좌표 변경 가능
        bool is_custom_message = (msg_id >= 60000 && msg_id <= 60003);
        bool is_mission_start = (msg_id == 60000);
        if (is_custom_message && !is_mission_start) {
            uint8_t sysid = header->sysid;
            uint8_t seq = header->seq;
            auto it = last_seq_by_sysid_.find(sysid);
            if (it != last_seq_by_sysid_.end() && it->second == seq) {
                duplicate_drop_count_++;
                if (duplicate_drop_count_ % 10 == 1) {
                    std::cout << "[CustomMessage] 중복 메시지 드롭: sysid=" << static_cast<int>(sysid)
                              << ", seq=" << static_cast<int>(seq)
                              << ", MSG_ID=" << msg_id << " (총 " << duplicate_drop_count_ << "건)" << std::endl;
                }
                return;
            }
            last_seq_by_sysid_[sysid] = seq;
        }

        // 표준 MAVLink 메시지는 라우터가 이미 FC로 전달했으므로 여기서는 무시
        // 커스텀 메시지: 60000(미션스타트), 60001(자동조준), 60002(발사), 60003(복귀)
        // 표준 메시지: COMMAND_LONG(76), SET_MODE(11)
        bool is_command_long = (msg_id == 76);
        bool is_set_mode = (msg_id == 11);

        // 표준 메시지는 라우터가 처리했으므로 파싱하지 않음
        if (!is_custom_message && !is_command_long && !is_set_mode) {
            return;
        }

        // COMMAND_LONG 메시지 디버그 출력
        if (msg_id == 76) {
            static int command_long_count = 0;
            command_long_count++;
            if (command_long_count <= 5) {
                std::cout << "[CustomMessage] [STEP 2] COMMAND_LONG 메시지 수신: MSG_ID=" << msg_id
                          << ", payload_len=" << payload_len << std::endl;
            }
        }

        static int parse_count = 0;
        parse_count++;
        // 커스텀 메시지(60000-60003)와 COMMAND_LONG(76)은 항상 출력
        bool is_important = (msg_id == 76) || (msg_id >= 60000 && msg_id <= 60003);
        if (is_important || parse_count <= 10) {
            std::cout << "[CustomMessage] [STEP 2] 메시지 파싱: MSG_ID=" << msg_id 
                      << ", payload_len=" << payload_len << std::endl;
        }

        // CRC_EXTRA 값 (메시지 ID별로 다름) - MAVLink 공식 값
        // 참고: https://mavlink.io/ko/guide/serialization.html#crc_extra
        // NOTE: 60000번대 사용 (50001/50002는 CUBEPILOT/HERELINK와 충돌)
        uint8_t crc_extra = 0;
        switch (msg_id) {
            case 11:    crc_extra = 89; break;   // SET_MODE (표준 MAVLink)
            case 76:    crc_extra = 152; break;  // COMMAND_LONG (표준 MAVLink)
            case 60000: crc_extra = 100; break;  // FIRE_MISSION_START (미션 스타트)
            case 60001: crc_extra = 101; break;  // AUTO_AIM (자동조준)
            case 60002: crc_extra = 102; break;  // FIRE_COMMAND (발사)
            case 60003: crc_extra = 103; break;  // RETURN_TO_LAUNCH (복귀)
            default: crc_extra = 0; break;
        }

        // 체크섬 검증 (헤더 + 페이로드 + CRC_EXTRA)
        // 버퍼 범위 검증 (이미 위에서 검증했지만 안전을 위해 다시 확인)
        if (MAVLINK_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN > len) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }
        
        uint16_t received_crc = buffer[MAVLINK_HEADER_LEN + payload_len] |
                               (static_cast<uint16_t>(buffer[MAVLINK_HEADER_LEN + payload_len + 1]) << 8);
        uint16_t calculated_crc = calculateCRC16(buffer + 1, MAVLINK_HEADER_LEN - 1 + payload_len);
        // CRC_EXTRA 추가
        uint8_t crc_extra_byte = crc_extra;
        calculated_crc = calculateCRC16(&crc_extra_byte, 1, calculated_crc);

        if (received_crc != calculated_crc) {
            // 커스텀 메시지(60000-60003)는 CRC 무시하고 처리 (자체 CRC_EXTRA 사용)
            if (is_custom_message) {
                std::cout << "[CustomMessage] [STEP 2] ⚠ CRC 불일치 무시 (MSG_ID=" << msg_id
                          << "): 수신=0x" << std::hex << received_crc
                          << ", 계산=0x" << calculated_crc << std::dec
                          << ", CRC_EXTRA=" << static_cast<int>(crc_extra) << std::endl;
                // CRC 무시하고 계속 진행
            } else {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.parse_error_count++;
                // CRC 불일치 로그는 처음 몇 개만 출력 (너무 많은 로그 방지)
                static int crc_error_count = 0;
                crc_error_count++;
                if (crc_error_count <= 5) {
                    std::cerr << "[CustomMessage] [STEP 2] ✗ CRC 불일치: 수신=0x" << std::hex << received_crc
                              << ", 계산=0x" << calculated_crc
                              << ", MSG_ID=" << std::dec << msg_id << std::endl;
                }
                return;
            }
        }

        if (parse_count <= 10) {
            std::cout << "[CustomMessage] [STEP 2] ✓ CRC 검증 통과" << std::endl;
        }

        // 메시지 타입별 파싱 (이미 위에서 유효성 검사 완료)
        switch (msg_id) {
            case 11:  // SET_MODE (표준 MAVLink 메시지)
                parseSetMode(payload, payload_len);
                // 표준 메시지를 mavlink-router(14550)로 포워딩 → FC로 전달
                forwardToFC(buffer, len);
                break;

            case 76:  // COMMAND_LONG (표준 MAVLink 메시지)
                parseCommandLong(payload, payload_len);
                // 표준 메시지를 mavlink-router(14550)로 포워딩 → FC로 전달
                forwardToFC(buffer, len);
                break;

            case 60000:  // 미션 스타트
                std::cout << "[CustomMessage] ★ 60000 미션스타트 수신 (sysid="
                          << static_cast<int>(header->sysid) << ", compid="
                          << static_cast<int>(header->compid) << ")" << std::endl;
                parseFireMissionStart(payload, payload_len);
                break;

            case 60001:  // 자동조준
                std::cout << "[CustomMessage] ★ 60001 자동조준 수신 (sysid="
                          << static_cast<int>(header->sysid) << ", compid="
                          << static_cast<int>(header->compid) << ")" << std::endl;
                parseFireAutoAim(payload, payload_len);
                break;

            case 60002:  // 발사
                std::cout << "[CustomMessage] ★ 60002 발사 수신 (sysid="
                          << static_cast<int>(header->sysid) << ", compid="
                          << static_cast<int>(header->compid) << ")" << std::endl;
                parseFireLaunch(payload, payload_len);
                break;

            case 60003:  // 복귀
                std::cout << "[CustomMessage] ★ 60003 복귀 수신 (sysid="
                          << static_cast<int>(header->sysid) << ", compid="
                          << static_cast<int>(header->compid) << ")" << std::endl;
                parseFireReturn(payload, payload_len);
                break;

            default:
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    stats_.unknown_message_count++;
                    if (stats_.unknown_message_count % 10 == 1) {  // 10개마다 한 번씩만 출력
                        std::cerr << "[CustomMessage] 알 수 없는 메시지 ID: " << msg_id << std::endl;
                    }
                }
                break;
        }
    }

    void parseFireMissionStart(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireMissionStart)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            std::cerr << "[CustomMessage] FIRE_MISSION_START 페이로드 길이 부족: " 
                      << len << " < " << sizeof(FireMissionStart) << std::endl;
            return;
        }

        FireMissionStart start;
        memcpy(&start, payload, sizeof(FireMissionStart));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.mission_start_received++;
        }

        std::cout << "[CustomMessage] [STEP 3] FIRE_MISSION_START 파싱 완료: "
                  << "lat=" << (start.target_lat / 1e7) << ", lon=" << (start.target_lon / 1e7)
                  << ", alt=" << start.target_alt << std::endl;

        if (mission_start_callback_) {
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 시작" << std::endl;
            mission_start_callback_(start);
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] [STEP 4] ⚠ 콜백 함수가 설정되지 않음!" << std::endl;
        }
    }

    // 60001 자동조준 처리 (target_system, target_component만 포함)
    void parseFireAutoAim(const uint8_t* payload, size_t len) {
        if (len < 2) {
            std::cerr << "[CustomMessage] FIRE_AUTO_AIM 페이로드 길이 부족: "
                      << len << " < 2" << std::endl;
            return;
        }

        uint8_t target_system = payload[0];
        uint8_t target_component = payload[1];

        std::cout << "[CustomMessage] ★★★ 60001 자동조준 명령 수신 완료! ★★★" << std::endl;
        std::cout << "[CustomMessage]   target_system=" << static_cast<int>(target_system)
                  << ", target_component=" << static_cast<int>(target_component) << std::endl;

        // FireAutoAim 구조체로 콜백 호출
        if (auto_aim_callback_) {
            FireAutoAim aim;
            aim.target_system = target_system;
            aim.target_component = target_component;

            std::cout << "[CustomMessage] [STEP 4] 자동조준 콜백 호출" << std::endl;
            auto_aim_callback_(aim);
            std::cout << "[CustomMessage] [STEP 4] 자동조준 콜백 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] ⚠ 자동조준 콜백이 설정되지 않음!" << std::endl;
        }
    }

    // 60002 발사 명령 처리 (target_system, target_component만 포함)
    void parseFireLaunch(const uint8_t* payload, size_t len) {
        if (len < 2) {
            std::cerr << "[CustomMessage] FIRE_LAUNCH 페이로드 길이 부족: "
                      << len << " < 2" << std::endl;
            return;
        }

        uint8_t target_system = payload[0];
        uint8_t target_component = payload[1];

        std::cout << "[CustomMessage] ★★★ 60002 발사 명령 수신 완료! ★★★" << std::endl;
        std::cout << "[CustomMessage]   target_system=" << static_cast<int>(target_system)
                  << ", target_component=" << static_cast<int>(target_component) << std::endl;

        // FireLaunch 구조체로 콜백 호출
        if (launch_callback_) {
            FireLaunch launch;
            launch.target_system = target_system;
            launch.target_component = target_component;

            std::cout << "[CustomMessage] [STEP 4] 발사 콜백 호출" << std::endl;
            launch_callback_(launch);
            std::cout << "[CustomMessage] [STEP 4] 발사 콜백 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] ⚠ 발사 콜백이 설정되지 않음!" << std::endl;
        }
    }

    // 60003 복귀 명령 처리 (target_system, target_component만 포함)
    void parseFireReturn(const uint8_t* payload, size_t len) {
        if (len < 2) {
            std::cerr << "[CustomMessage] FIRE_RETURN 페이로드 길이 부족: "
                      << len << " < 2" << std::endl;
            return;
        }

        uint8_t target_system = payload[0];
        uint8_t target_component = payload[1];

        std::cout << "[CustomMessage] ★★★ 60003 복귀 명령 수신 완료! ★★★" << std::endl;
        std::cout << "[CustomMessage]   target_system=" << static_cast<int>(target_system)
                  << ", target_component=" << static_cast<int>(target_component) << std::endl;

        // FireReturn 구조체로 콜백 호출
        if (return_callback_) {
            FireReturn ret;
            ret.target_system = target_system;
            ret.target_component = target_component;

            std::cout << "[CustomMessage] [STEP 4] 복귀 콜백 호출" << std::endl;
            return_callback_(ret);
            std::cout << "[CustomMessage] [STEP 4] 복귀 콜백 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] ⚠ 복귀 콜백이 설정되지 않음!" << std::endl;
        }
    }

    void parseSetMode(const uint8_t* payload, size_t len) {
        // SET_MODE MAVLink v2 Wire Format (크기 내림차순 재배열):
        // custom_mode(4) + target_system(1) + base_mode(1) = 6 bytes
        const size_t SET_MODE_PAYLOAD_LEN = 6;
        if (len < SET_MODE_PAYLOAD_LEN) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            std::cerr << "[CustomMessage] SET_MODE 페이로드 길이 부족: "
                      << len << " < " << SET_MODE_PAYLOAD_LEN << std::endl;
            return;
        }

        // Wire format 순서 (큰 필드 먼저)
        uint32_t custom_mode = payload[0] | (static_cast<uint32_t>(payload[1]) << 8) |
                               (static_cast<uint32_t>(payload[2]) << 16) |
                               (static_cast<uint32_t>(payload[3]) << 24);
        uint8_t target_system = payload[4];
        uint8_t base_mode = payload[5];
        uint8_t main_mode = (custom_mode >> 16) & 0xFF;  // Extract main_mode

        std::cout << "[CustomMessage] [STEP 3] SET_MODE 파싱 완료: "
                  << "target_system=" << static_cast<int>(target_system)
                  << ", base_mode=" << static_cast<int>(base_mode)
                  << ", custom_mode=" << custom_mode
                  << " (main_mode=" << static_cast<int>(main_mode) << ")" << std::endl;

        // SET_MODE는 표준 MAVLink 메시지이므로 파싱 후 MAVLink 라우터로도 전달
        // VIM4에서는 메시지를 파싱하고 콜백 호출, 동시에 FC로도 전달

        if (set_mode_callback_) {
            std::cout << "[CustomMessage] [STEP 4] SET_MODE 콜백 함수 호출 시작" << std::endl;
            set_mode_callback_(target_system, base_mode, custom_mode);
            std::cout << "[CustomMessage] [STEP 4] SET_MODE 콜백 함수 호출 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] [STEP 4] ⚠ SET_MODE 콜백 함수가 설정되지 않음!" << std::endl;
        }
    }

    void parseCommandLong(const uint8_t* payload, size_t len) {
        // COMMAND_LONG MAVLink v2 Wire Format (크기 내림차순 재배열):
        // param1(4) + param2(4) + param3(4) + param4(4) + param5(4) + param6(4) + param7(4)
        // + command(2) + target_system(1) + target_component(1) + confirmation(1) = 33 bytes
        // MAVLink v2 trailing zero truncation으로 더 짧을 수 있음
        const size_t COMMAND_LONG_MIN_LEN = 30;  // param1-7(28) + command(2) 까지 필수
        if (len < COMMAND_LONG_MIN_LEN) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            std::cerr << "[CustomMessage] COMMAND_LONG 페이로드 길이 부족: "
                      << len << " < " << COMMAND_LONG_MIN_LEN << std::endl;
            return;
        }

        // param1-7 파싱 (little-endian float, 각 4 bytes, offset 0~27)
        auto parse_float = [](const uint8_t* data, size_t payload_len, size_t offset) -> float {
            if (offset + 4 > payload_len) return 0.0f;
            uint32_t bits = static_cast<uint32_t>(data[offset]) |
                           (static_cast<uint32_t>(data[offset+1]) << 8) |
                           (static_cast<uint32_t>(data[offset+2]) << 16) |
                           (static_cast<uint32_t>(data[offset+3]) << 24);
            float value;
            memcpy(&value, &bits, sizeof(float));
            return value;
        };

        // Wire format 순서 (큰 필드 먼저)
        float param1 = parse_float(payload, len, 0);
        float param2 = parse_float(payload, len, 4);
        float param3 = parse_float(payload, len, 8);
        float param4 = parse_float(payload, len, 12);
        float param5 = parse_float(payload, len, 16);
        float param6 = parse_float(payload, len, 20);
        float param7 = parse_float(payload, len, 24);
        uint16_t command = payload[28] | (static_cast<uint16_t>(payload[29]) << 8);
        uint8_t target_system = (len > 30) ? payload[30] : 0;
        uint8_t target_component = (len > 31) ? payload[31] : 0;
        // confirmation = payload[32] (무시)

        std::cout << "[CustomMessage] [STEP 3] COMMAND_LONG 파싱 완료: "
                  << "target_system=" << static_cast<int>(target_system)
                  << ", target_component=" << static_cast<int>(target_component)
                  << ", command=" << command
                  << ", param1=" << param1 
                  << ", param2=" << param2 << std::endl;

        // COMMAND_LONG은 표준 MAVLink 메시지이므로 파싱 후 MAVLink 라우터로도 전달
        // VIM4에서는 메시지를 파싱하고 콜백 호출, 동시에 FC로도 전달

        if (command_long_callback_) {
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 시작" << std::endl;
            command_long_callback_(target_system, target_component, command, param1, param2, param3, param4, param5, param6, param7);
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] [STEP 4] ⚠ 콜백 함수가 설정되지 않음!" << std::endl;
        }
        
        // COMMAND_LONG도 MAVLink 라우터로 전달 (FC로 전달되어야 함)
        // parseMAVLinkMessage의 buffer를 사용해야 하므로, receiveLoop에서 처리하도록 변경 필요
        // 현재는 parseCommandLong이 호출된 후이므로, receiveLoop에서 처리하는 것이 더 적합
    }


    uint16_t receive_port_;
    uint16_t send_port_;
    std::string bind_address_;
    std::string target_address_;
    uint8_t system_id_;
    uint8_t component_id_;
    int receive_socket_fd_;
    int send_socket_fd_;
    int epoll_fd_;  // epoll 파일 디스크립터
    std::atomic<bool> running_;
    std::thread receive_thread_;
    std::thread process_thread_;  // 메시지 파싱 및 콜백 처리 스레드
    std::atomic<uint8_t> sequence_number_;

    struct sockaddr_in target_addr_;
    mutable std::mutex target_mutex_;
    mutable std::mutex stats_mutex_;
    
    // 메시지 큐 (메시지 손실 방지)
    std::queue<std::vector<uint8_t>> message_queue_;
    std::mutex message_queue_mutex_;
    std::condition_variable message_queue_cv_;

    // 중복 메시지 방지 (sysid -> last seq)
    std::unordered_map<uint8_t, uint8_t> last_seq_by_sysid_;
    uint64_t duplicate_drop_count_{0};

    FireMissionStartCallback mission_start_callback_;
    FireAutoAimCallback auto_aim_callback_;
    FireLaunchCallback launch_callback_;
    FireReturnCallback return_callback_;
    CommandLongCallback command_long_callback_;
    SetModeCallback set_mode_callback_;

    CustomMessage::Statistics stats_;
};

// CustomMessage 구현
CustomMessage::CustomMessage(
    uint16_t receive_port,
    uint16_t send_port,
    const std::string& bind_address,
    const std::string& target_address,
    uint8_t system_id,
    uint8_t component_id
)
    : impl_(std::make_unique<CustomMessageImpl>(
        receive_port, send_port, bind_address, target_address, system_id, component_id))
{
}

CustomMessage::~CustomMessage() {
    stop();
}

bool CustomMessage::start() {
    return impl_->start();
}

void CustomMessage::stop() {
    impl_->stop();
}

bool CustomMessage::isRunning() const {
    return impl_->isRunning();
}

void CustomMessage::setFireMissionStartCallback(FireMissionStartCallback callback) {
    impl_->setFireMissionStartCallback(callback);
}

void CustomMessage::setFireAutoAimCallback(FireAutoAimCallback callback) {
    impl_->setFireAutoAimCallback(callback);
}

void CustomMessage::setFireLaunchCallback(FireLaunchCallback callback) {
    impl_->setFireLaunchCallback(callback);
}

void CustomMessage::setFireReturnCallback(FireReturnCallback callback) {
    impl_->setFireReturnCallback(callback);
}

void CustomMessage::setCommandLongCallback(CommandLongCallback callback) {
    impl_->setCommandLongCallback(callback);
}

void CustomMessage::setSetModeCallback(SetModeCallback callback) {
    impl_->setSetModeCallback(callback);
}

bool CustomMessage::sendFireMissionStart(const FireMissionStart& start) {
    return impl_->sendFireMissionStart(start);
}

bool CustomMessage::sendFireAutoAim(const FireAutoAim& aim) {
    return impl_->sendFireAutoAim(aim);
}

bool CustomMessage::sendFireLaunch(const FireLaunch& launch) {
    return impl_->sendFireLaunch(launch);
}

bool CustomMessage::sendFireReturn(const FireReturn& ret) {
    return impl_->sendFireReturn(ret);
}

void CustomMessage::setTargetAddress(const std::string& address, uint16_t port) {
    impl_->setTargetAddress(address, port);
}

CustomMessage::Statistics CustomMessage::getStatistics() const {
    return impl_->getStatistics();
}

void CustomMessage::resetStatistics() {
    impl_->resetStatistics();
}

} // namespace custom_message
