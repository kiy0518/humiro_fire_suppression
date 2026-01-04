/**
 * @file custom_message.cpp
 * @brief 커스텀 MAVLink 메시지 송수신 라이브러리 구현
 */

#include "custom_message/custom_message.h"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <mutex>

namespace custom_message {

// MAVLink 헤더 구조체
struct MAVLinkHeader {
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
uint16_t calculateCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;

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

        // 체크섬 계산 (헤더 + 페이로드)
        uint16_t crc = calculateCRC16(buffer + 1, MAVLINK_HEADER_LEN - 1 + payload_len);
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

            if (received < MAVLINK_HEADER_LEN) {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.parse_error_count++;
                continue;
            }

            parseMAVLinkMessage(buffer, received);
        }
    }

    void parseMAVLinkMessage(const uint8_t* buffer, size_t len) {
        if (buffer[0] != MAVLINK_MAGIC) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        const MAVLinkHeader* header = reinterpret_cast<const MAVLinkHeader*>(buffer);

        uint32_t msg_id = header->msgid | (static_cast<uint32_t>(header->msgid_ext[0]) << 8) |
                         (static_cast<uint32_t>(header->msgid_ext[1]) << 16);

        const uint8_t* payload = buffer + MAVLINK_HEADER_LEN;
        size_t payload_len = header->len;

        // 체크섬 검증
        uint16_t received_crc = buffer[MAVLINK_HEADER_LEN + payload_len] |
                               (static_cast<uint16_t>(buffer[MAVLINK_HEADER_LEN + payload_len + 1]) << 8);
        uint16_t calculated_crc = calculateCRC16(buffer + 1, MAVLINK_HEADER_LEN - 1 + payload_len);

        if (received_crc != calculated_crc) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        // 메시지 타입별 파싱
        switch (msg_id) {
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
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.unknown_message_count++;
                break;
        }
    }

    void parseFireMissionStart(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireMissionStart)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        FireMissionStart start;
        memcpy(&start, payload, sizeof(FireMissionStart));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.mission_start_received++;
        }

        if (mission_start_callback_) {
            mission_start_callback_(start);
        }
    }

    void parseFireMissionStatus(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireMissionStatus)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        FireMissionStatus status;
        memcpy(&status, payload, sizeof(FireMissionStatus));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.mission_status_received++;
        }

        if (mission_status_callback_) {
            mission_status_callback_(status);
        }
    }

    void parseFireLaunchControl(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireLaunchControl)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        FireLaunchControl control;
        memcpy(&control, payload, sizeof(FireLaunchControl));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.launch_control_received++;
        }

        if (launch_control_callback_) {
            launch_control_callback_(control);
        }
    }

    void parseFireSuppressionResult(const uint8_t* payload, size_t len) {
        if (len < sizeof(FireSuppressionResult)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_error_count++;
            return;
        }

        FireSuppressionResult result;
        memcpy(&result, payload, sizeof(FireSuppressionResult));

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.suppression_result_received++;
        }

        if (suppression_result_callback_) {
            suppression_result_callback_(result);
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
