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
#include <mutex>

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

        // 수신 소켓 생성
        receive_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (receive_socket_fd_ < 0) {
            std::cerr << "[CustomMessage] 수신 소켓 생성 실패: " << strerror(errno) << std::endl;
            return false;
        }

        // 송신 소켓 생성
        send_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (send_socket_fd_ < 0) {
            std::cerr << "[CustomMessage] 송신 소켓 생성 실패: " << strerror(errno) << std::endl;
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

        running_ = true;
        receive_thread_ = std::thread(&CustomMessageImpl::receiveLoop, this);

        std::cout << "[CustomMessage] 메시지 송수신 시작 (수신 포트: " << receive_port_
                  << ", 송신 포트: " << send_port_ << ", 대상: " << target_address_ << ")" << std::endl;
        return true;
    }

    void stop() {
        if (!running_) {
            return;
        }

        running_ = false;

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

        std::cout << "[CustomMessage] 메시지 송수신 중지" << std::endl;
    }

    bool isRunning() const {
        return running_;
    }

    void setFireMissionStartCallback(FireMissionStartCallback callback) {
        mission_start_callback_ = callback;
    }

    void setFireMissionStatusCallback(FireMissionStatusCallback callback) {
        mission_status_callback_ = callback;
    }

    void setFireLaunchControlCallback(FireLaunchControlCallback callback) {
        launch_control_callback_ = callback;
    }

    void setFireSuppressionResultCallback(FireSuppressionResultCallback callback) {
        suppression_result_callback_ = callback;
    }

    void setCommandLongCallback(CommandLongCallback callback) {
        command_long_callback_ = callback;
    }

    bool sendFireMissionStart(const FireMissionStart& start) {
        return sendMessage(MessageType::FIRE_MISSION_START, &start, sizeof(FireMissionStart));
    }

    bool sendFireMissionStatus(const FireMissionStatus& status) {
        return sendMessage(MessageType::FIRE_MISSION_STATUS, &status, sizeof(FireMissionStatus));
    }

    bool sendFireLaunchControl(const FireLaunchControl& control) {
        return sendMessage(MessageType::FIRE_LAUNCH_CONTROL, &control, sizeof(FireLaunchControl));
    }

    bool sendFireSuppressionResult(const FireSuppressionResult& result) {
        return sendMessage(MessageType::FIRE_SUPPRESSION_RESULT, &result, sizeof(FireSuppressionResult));
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
        uint8_t crc_extra = 0;
        switch (msg_id) {
            case 12900: crc_extra = 100; break;  // FIRE_MISSION_START
            case 12901: crc_extra = 101; break;  // FIRE_MISSION_STATUS
            case 12902: crc_extra = 102; break;  // FIRE_LAUNCH_CONTROL
            case 12903: crc_extra = 103; break;  // FIRE_SUPPRESSION_RESULT
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
            case MessageType::FIRE_MISSION_STATUS:
                stats_.mission_status_sent++;
                break;
            case MessageType::FIRE_LAUNCH_CONTROL:
                stats_.launch_control_sent++;
                break;
            case MessageType::FIRE_SUPPRESSION_RESULT:
                stats_.suppression_result_sent++;
                break;
            default:
                break;
        }

        return true;
    }

    void receiveLoop() {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        static int receive_count = 0;

        while (running_) {
            ssize_t received = recvfrom(receive_socket_fd_, buffer, sizeof(buffer), 0,
                                       (struct sockaddr*)&client_addr, &client_addr_len);

            if (received < 0) {
                if (errno == EINTR || errno == EAGAIN) {
                    continue;
                }
                if (running_) {
                    std::cerr << "[CustomMessage] 수신 오류: " << strerror(errno) << std::endl;
                }
                continue;
            }

            // received가 0이면 무시 (빈 패킷)
            if (received == 0) {
                continue;
            }

            receive_count++;
            // 처음 몇 개 메시지에 대해 디버그 출력
            // COMMAND_LONG(76)은 항상 출력
            if (buffer[0] == 0xFD && static_cast<size_t>(received) >= MAVLINK_HEADER_LEN) {
                // 헤더에서 MSG_ID 확인 (buffer[7] = msgid, buffer[8] = msgid_ext[0], buffer[9] = msgid_ext[1])
                uint32_t msg_id_check = buffer[7] | (static_cast<uint32_t>(buffer[8]) << 8) | 
                                       (static_cast<uint32_t>(buffer[9]) << 16);
                if (msg_id_check == 76) {
                    std::cout << "[CustomMessage] [STEP 1] UDP 수신: " << received << " bytes, "
                              << "첫 바이트: 0x" << std::hex << static_cast<int>(buffer[0]) 
                              << std::dec << ", MSG_ID=76 (COMMAND_LONG), 포트: " << receive_port_ << std::endl;
                } else if (receive_count <= 10) {
                    std::cout << "[CustomMessage] [STEP 1] UDP 수신: " << received << " bytes, "
                              << "첫 바이트: 0x" << std::hex << static_cast<int>(buffer[0]) 
                              << std::dec << ", MSG_ID=" << msg_id_check << ", 포트: " << receive_port_ << std::endl;
                }
            } else if (receive_count <= 10) {
                std::cout << "[CustomMessage] [STEP 1] UDP 수신: " << received << " bytes, "
                          << "첫 바이트: 0x" << std::hex << static_cast<int>(buffer[0]) 
                          << std::dec << ", 포트: " << receive_port_ << std::endl;
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

            // 예외 처리: 메시지 파싱 오류가 발생해도 영상 스트림은 계속 작동해야 함
            try {
                parseMAVLinkMessage(buffer, received);
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.parse_error_count++;
                static int exception_count = 0;
                exception_count++;
                if (exception_count <= 5) {
                    std::cerr << "[CustomMessage] 메시지 파싱 예외 발생 (무시하고 계속): " << e.what() << std::endl;
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.parse_error_count++;
                static int unknown_exception_count = 0;
                unknown_exception_count++;
                if (unknown_exception_count <= 5) {
                    std::cerr << "[CustomMessage] 알 수 없는 예외 발생 (무시하고 계속)" << std::endl;
                }
            }
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

        // MSG_ID=0이거나 유효하지 않은 메시지는 조용히 무시 (다른 소스의 잘못된 메시지 또는 네트워크 노이즈)
        // CRC 검증 전에 필터링하여 불필요한 처리 방지
        // COMMAND_LONG(76)도 허용 (ARM/DISARM 명령용)
        if (msg_id == 0 || (msg_id != 76 && (msg_id < 12900 || msg_id > 12903))) {
            // 유효하지 않은 메시지는 통계만 업데이트하고 조용히 무시
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            
            // 필터링된 메시지 디버그 출력 (처음 20개만 출력하여 노이즈 방지)
            static int filtered_count = 0;
            filtered_count++;
            if (filtered_count <= 20) {
                std::cout << "[CustomMessage] [STEP 2] ⚠ 메시지 필터링됨: MSG_ID=" << msg_id 
                          << " (허용 범위: 76 또는 12900-12903)" << std::endl;
            } else if (filtered_count == 21) {
                std::cout << "[CustomMessage] [STEP 2] ⚠ 필터링된 메시지가 많아 이후 로그는 생략합니다" << std::endl;
            }
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
        if (parse_count <= 10) {
            std::cout << "[CustomMessage] [STEP 2] 메시지 파싱: MSG_ID=" << msg_id 
                      << ", payload_len=" << payload_len << std::endl;
        }

        // CRC_EXTRA 값 (메시지 ID별로 다름)
        uint8_t crc_extra = 0;
        switch (msg_id) {
            case 76:    crc_extra = 19;  break;  // COMMAND_LONG (표준 MAVLink)
            case 12900: crc_extra = 100; break;  // FIRE_MISSION_START
            case 12901: crc_extra = 101; break;  // FIRE_MISSION_STATUS
            case 12902: crc_extra = 102; break;  // FIRE_LAUNCH_CONTROL
            case 12903: crc_extra = 103; break;  // FIRE_SUPPRESSION_RESULT
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

        if (parse_count <= 10) {
            std::cout << "[CustomMessage] [STEP 2] ✓ CRC 검증 통과" << std::endl;
        }

        // 메시지 타입별 파싱 (이미 위에서 유효성 검사 완료)
        switch (msg_id) {
            case 76:  // COMMAND_LONG (표준 MAVLink 메시지)
                parseCommandLong(payload, payload_len);
                break;

            case static_cast<uint32_t>(MessageType::FIRE_MISSION_START):
                parseFireMissionStart(payload, payload_len);
                break;

            case static_cast<uint32_t>(MessageType::FIRE_MISSION_STATUS):
                parseFireMissionStatus(payload, payload_len);
                break;

            case static_cast<uint32_t>(MessageType::FIRE_LAUNCH_CONTROL):
                parseFireLaunchControl(payload, payload_len);
                break;

            case static_cast<uint32_t>(MessageType::FIRE_SUPPRESSION_RESULT):
                parseFireSuppressionResult(payload, payload_len);
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

    void parseFireMissionStatus(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireMissionStatus)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            std::cerr << "[CustomMessage] FIRE_MISSION_STATUS 페이로드 길이 부족: " 
                      << len << " < " << sizeof(FireMissionStatus) << std::endl;
            return;
        }

        FireMissionStatus status;
        memcpy(&status, payload, sizeof(FireMissionStatus));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.mission_status_received++;
        }

        std::cout << "[CustomMessage] [STEP 3] FIRE_MISSION_STATUS 파싱 완료: "
                  << "phase=" << static_cast<int>(status.phase) 
                  << ", progress=" << static_cast<int>(status.progress)
                  << "%, distance=" << status.distance_to_target << "m" << std::endl;

        if (mission_status_callback_) {
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 시작" << std::endl;
            mission_status_callback_(status);
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] [STEP 4] ⚠ 콜백 함수가 설정되지 않음!" << std::endl;
        }
    }

    void parseFireLaunchControl(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireLaunchControl)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            std::cerr << "[CustomMessage] FIRE_LAUNCH_CONTROL 페이로드 길이 부족: " 
                      << len << " < " << sizeof(FireLaunchControl) << std::endl;
            return;
        }

        FireLaunchControl control;
        memcpy(&control, payload, sizeof(FireLaunchControl));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.launch_control_received++;
        }

        std::string cmd_name = (control.command == 0) ? "CONFIRM" : 
                              (control.command == 1) ? "ABORT" : 
                              (control.command == 2) ? "REQUEST_STATUS" : "UNKNOWN";
        std::cout << "[CustomMessage] [STEP 3] FIRE_LAUNCH_CONTROL 파싱 완료: command=" 
                  << static_cast<int>(control.command) << " (" << cmd_name << ")" << std::endl;

        if (launch_control_callback_) {
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 시작" << std::endl;
            launch_control_callback_(control);
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] [STEP 4] ⚠ 콜백 함수가 설정되지 않음!" << std::endl;
        }
    }

    void parseFireSuppressionResult(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireSuppressionResult)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            std::cerr << "[CustomMessage] FIRE_SUPPRESSION_RESULT 페이로드 길이 부족: " 
                      << len << " < " << sizeof(FireSuppressionResult) << std::endl;
            return;
        }

        FireSuppressionResult result;
        memcpy(&result, payload, sizeof(FireSuppressionResult));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.suppression_result_received++;
        }

        std::cout << "[CustomMessage] [STEP 3] FIRE_SUPPRESSION_RESULT 파싱 완료: "
                  << "shot=" << static_cast<int>(result.shot_number)
                  << ", temp_before=" << (result.temp_before / 10.0) << "°C"
                  << ", temp_after=" << (result.temp_after / 10.0) << "°C"
                  << ", success=" << (result.success ? "true" : "false") << std::endl;

        if (suppression_result_callback_) {
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 시작" << std::endl;
            suppression_result_callback_(result);
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] [STEP 4] ⚠ 콜백 함수가 설정되지 않음!" << std::endl;
        }
    }

    void parseCommandLong(const uint8_t* payload, size_t len) {
        // COMMAND_LONG 페이로드 구조: target_system(1) + target_component(1) + command(2) + confirmation(1) + param1-7(28) = 33 bytes
        const size_t COMMAND_LONG_PAYLOAD_LEN = 33;
        if (len < COMMAND_LONG_PAYLOAD_LEN) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            std::cerr << "[CustomMessage] COMMAND_LONG 페이로드 길이 부족: " 
                      << len << " < " << COMMAND_LONG_PAYLOAD_LEN << std::endl;
            return;
        }

        uint8_t target_system = payload[0];
        uint8_t target_component = payload[1];
        uint16_t command = payload[2] | (static_cast<uint16_t>(payload[3]) << 8);
        // confirmation은 무시 (payload[4])
        
        // param1-7 파싱 (little-endian float, 각 4 bytes)
        auto parse_float = [](const uint8_t* data) -> float {
            uint32_t bits = static_cast<uint32_t>(data[0]) |
                           (static_cast<uint32_t>(data[1]) << 8) |
                           (static_cast<uint32_t>(data[2]) << 16) |
                           (static_cast<uint32_t>(data[3]) << 24);
            float value;
            memcpy(&value, &bits, sizeof(float));
            return value;
        };
        
        float param1 = parse_float(&payload[5]);
        float param2 = parse_float(&payload[9]);
        float param3 = parse_float(&payload[13]);
        float param4 = parse_float(&payload[17]);
        float param5 = parse_float(&payload[21]);
        float param6 = parse_float(&payload[25]);
        float param7 = parse_float(&payload[29]);

        std::cout << "[CustomMessage] [STEP 3] COMMAND_LONG 파싱 완료: "
                  << "target_system=" << static_cast<int>(target_system)
                  << ", target_component=" << static_cast<int>(target_component)
                  << ", command=" << command
                  << ", param1=" << param1 
                  << ", param2=" << param2 << std::endl;

        // COMMAND_LONG은 표준 MAVLink 메시지이므로 MAVLink 라우터가 자동으로 FC로 전달함
        // VIM4에서는 메시지를 파싱하고 콜백만 호출

        if (command_long_callback_) {
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 시작" << std::endl;
            command_long_callback_(target_system, target_component, command, param1, param2, param3, param4, param5, param6, param7);
            std::cout << "[CustomMessage] [STEP 4] 콜백 함수 호출 완료" << std::endl;
        } else {
            std::cerr << "[CustomMessage] [STEP 4] ⚠ 콜백 함수가 설정되지 않음!" << std::endl;
        }
    }


    uint16_t receive_port_;
    uint16_t send_port_;
    std::string bind_address_;
    std::string target_address_;
    uint8_t system_id_;
    uint8_t component_id_;
    int receive_socket_fd_;
    int send_socket_fd_;
    std::atomic<bool> running_;
    std::thread receive_thread_;
    std::atomic<uint8_t> sequence_number_;

    struct sockaddr_in target_addr_;
    mutable std::mutex target_mutex_;
    mutable std::mutex stats_mutex_;

    FireMissionStartCallback mission_start_callback_;
    FireMissionStatusCallback mission_status_callback_;
    FireLaunchControlCallback launch_control_callback_;
    FireSuppressionResultCallback suppression_result_callback_;
    CommandLongCallback command_long_callback_;

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

void CustomMessage::setFireMissionStatusCallback(FireMissionStatusCallback callback) {
    impl_->setFireMissionStatusCallback(callback);
}

void CustomMessage::setFireLaunchControlCallback(FireLaunchControlCallback callback) {
    impl_->setFireLaunchControlCallback(callback);
}

void CustomMessage::setFireSuppressionResultCallback(FireSuppressionResultCallback callback) {
    impl_->setFireSuppressionResultCallback(callback);
}

void CustomMessage::setCommandLongCallback(CommandLongCallback callback) {
    impl_->setCommandLongCallback(callback);
}

bool CustomMessage::sendFireMissionStart(const FireMissionStart& start) {
    return impl_->sendFireMissionStart(start);
}

bool CustomMessage::sendFireMissionStatus(const FireMissionStatus& status) {
    return impl_->sendFireMissionStatus(status);
}

bool CustomMessage::sendFireLaunchControl(const FireLaunchControl& control) {
    return impl_->sendFireLaunchControl(control);
}

bool CustomMessage::sendFireSuppressionResult(const FireSuppressionResult& result) {
    return impl_->sendFireSuppressionResult(result);
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
