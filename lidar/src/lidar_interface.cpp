#include "lidar_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>

// LD19 프로토콜 상수 (공식 SDK 기준)
#define LD19_HEADER 0x54
#define LD19_VERLEN 0x2C  // 상위 3비트: 패킷 타입(1), 하위 5비트: 포인트 수(12)
#define LD19_POINTS_PER_PACK 12
#define LD19_PACKET_SIZE 47  // Header(1) + VerLen(1) + Speed(2) + StartAngle(2) + Data(36) + EndAngle(2) + Timestamp(2) + CRC(1)

// 패킷 구조체 (공식 SDK 방식, __attribute__((packed)) 사용)
typedef struct __attribute__((packed))
{
    uint16_t distance;
    uint8_t confidence;
} LidarPointStructDef;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[LD19_POINTS_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARFrameTypeDef;

// CRC8 테이블 (LD19 Development Manual V2.5 기준)
static const uint8_t crc_table[256] = {
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,
    0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,0xa9,0xe4,0x33,
    0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,
    0xf5,0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,
    0x3a,0x94,0xd9,0x0e,0x43,0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,
    0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,0x3e,0x73,0xa4,
    0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,
    0x62,0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,
    0xb2,0x1c,0x51,0x86,0xcb,0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,
    0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,0x88,0xc5,0x12,
    0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,
    0xd4,0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,
    0x59,0xf7,0xba,0x6d,0x20,0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,
    0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,0x63,0x2e,0xf9,
    0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,
    0x3f,0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,
    0xef,0x41,0x0c,0xdb,0x96,0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,
    0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,0xeb,0xa6,0x71,
    0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,
    0xb7,0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,
    0x78,0xd6,0x9b,0x4c,0x01,0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,
    0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
};

/**
 * CRC8 계산 함수 (공식 SDK 방식)
 * 참고: LD Robot 공식 SDK (ldlidar_component)
 * 
 * 공식 SDK는 전체 패킷(CRC 제외)을 계산합니다:
 *   - sizeof(LiDARFrameTypeDef) - 1 = 47 - 1 = 46바이트
 *   - 헤더부터 CRC 전까지 모든 바이트 포함
 *   - 초기값은 0
 * 
 * @param data 데이터 버퍼 (전체 패킷, CRC 제외)
 * @param len 데이터 길이 (46바이트: 헤더부터 Timestamp까지)
 * @return 계산된 CRC8 값
 */
static uint8_t calculateCRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc = crc_table[(crc ^ data[i]) & 0xff];
    }
    return crc;
}

LidarInterface::LidarInterface(const std::string& device, int baudrate)
    : config_(LidarConfig::createUSBUartConfig(device))
    , serial_fd_(-1)
    , is_connected_(false)
    , is_running_(false) {
    config_.baudrate = baudrate;
}

LidarInterface::LidarInterface(const LidarConfig& config)
    : config_(config)
    , serial_fd_(-1)
    , is_connected_(false)
    , is_running_(false) {
}

LidarInterface::~LidarInterface() {
    stop();
}

bool LidarInterface::start() {
    if (is_running_.load()) {
        last_error_ = "LiDAR already running";
        return false;
    }
    
    std::cout << "Starting LiDAR with " << config_.getConnectionTypeString() 
              << " (" << config_.device_path << ")" << std::endl;
    
    if (!openSerialPort()) {
        return false;
    }
    
    if (!configureSerialPort()) {
        closeSerialPort();
        return false;
    }
    
    // GPIO UART 특별 설정 (필요시)
    if (config_.connection_type == UartConnectionType::GPIO_UART) {
        if (!configureGPIOUart()) {
            std::cerr << "Warning: GPIO UART special configuration failed" << std::endl;
            // 계속 진행 (선택적 설정)
        }
    }
    
    is_running_ = true;
    is_connected_ = true;
    
    // 수신 스레드 시작
    receive_thread_ = std::thread(&LidarInterface::receiveThread, this);
    
    std::cout << "LiDAR started successfully" << std::endl;
    std::cout << "  Device: " << config_.device_path << std::endl;
    std::cout << "  Type: " << config_.getConnectionTypeString() << std::endl;
    std::cout << "  Baudrate: " << config_.baudrate << std::endl;
    
    return true;
}

void LidarInterface::stop() {
    if (!is_running_.load()) {
        return;
    }
    
    is_running_ = false;
    is_connected_ = false;
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    closeSerialPort();
    std::cout << "LiDAR stopped" << std::endl;
}

bool LidarInterface::openSerialPort() {
    serial_fd_ = open(config_.device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (serial_fd_ < 0) {
        last_error_ = "Failed to open serial port: " + config_.device_path;
        std::cerr << last_error_ << std::endl;
        std::cerr << "Error: " << strerror(errno) << std::endl;
        
        // 도움말 출력
        if (config_.connection_type == UartConnectionType::USB_UART) {
            std::cerr << "\nTroubleshooting USB-UART:" << std::endl;
            std::cerr << "  1. Check if device is connected: ls -l /dev/ttyUSB*" << std::endl;
            std::cerr << "  2. Set permissions: sudo chmod 666 " << config_.device_path << std::endl;
            std::cerr << "  3. Add user to dialout group: sudo usermod -a -G dialout $USER" << std::endl;
        } else {
            std::cerr << "\nTroubleshooting GPIO-UART:" << std::endl;
            std::cerr << "  1. Check if UART is enabled in device tree" << std::endl;
            std::cerr << "  2. Verify device exists: ls -l " << config_.device_path << std::endl;
            std::cerr << "  3. Check permissions: sudo chmod 666 " << config_.device_path << std::endl;
        }
        
        return false;
    }
    
    return true;
}

void LidarInterface::closeSerialPort() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool LidarInterface::configureSerialPort() {
    struct termios options;
    
    if (tcgetattr(serial_fd_, &options) < 0) {
        last_error_ = "Failed to get serial port attributes";
        return false;
    }
    
    // 통신 속도 설정 (230400 baud)
    speed_t baud_const = B230400;
    switch (config_.baudrate) {
        case 9600:   baud_const = B9600; break;
        case 19200:  baud_const = B19200; break;
        case 38400:  baud_const = B38400; break;
        case 57600:  baud_const = B57600; break;
        case 115200: baud_const = B115200; break;
        case 230400: baud_const = B230400; break;
        default:
            std::cerr << "Warning: Unsupported baudrate " << config_.baudrate 
                      << ", using 230400" << std::endl;
            baud_const = B230400;
    }
    
    cfsetispeed(&options, baud_const);
    cfsetospeed(&options, baud_const);
    
    // 8N1 설정 (8 data bits, No parity, 1 stop bit)
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 data bits
    
    // Hardware flow control 비활성화
    options.c_cflag &= ~CRTSCTS;
    
    // Enable receiver, ignore modem control lines
    options.c_cflag |= CREAD | CLOCAL;
    
    // Raw 모드
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    // Read timeout 설정
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;  // 0.1초
    
    // 버퍼 비우기
    tcflush(serial_fd_, TCIFLUSH);
    
    if (tcsetattr(serial_fd_, TCSANOW, &options) < 0) {
        last_error_ = "Failed to set serial port attributes";
        return false;
    }
    
    return true;
}

bool LidarInterface::configureGPIOUart() {
    // GPIO UART 특별 설정 (VIM4 전용)
    // 필요한 경우 여기에 추가 설정 구현
    
    // 예: 버퍼 크기 조정, 우선순위 설정 등
    // 현재는 표준 UART 설정으로 충분
    
    std::cout << "GPIO UART configured" << std::endl;
    return true;
}

void LidarInterface::receiveThread() {
    // 스레드 우선순위를 높임 (realtime priority) - UART overrun 방지
    struct sched_param param;
    param.sched_priority = 50;  // 1~99 (99가 가장 높음, 50은 중간)
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Failed to set realtime priority for LiDAR thread (need sudo?)" << std::endl;
        // 실패해도 계속 진행 (일반 우선순위로 동작)
    } else {
        std::cout << "LiDAR receive thread priority set to realtime (SCHED_FIFO, priority 50)" << std::endl;
    }

    uint8_t buffer[4096];  // 공식 SDK와 동일한 4KB 버퍼 (MAX_ACK_BUF_LEN)
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 100;

    // 패킷 파싱 통계
    static int packets_parsed = 0;
    static int packets_failed = 0;
    static auto last_stats_time = std::chrono::steady_clock::now();

    // pselect() 타임아웃 설정 (5ms로 조정 - UART overrun 최소화)
    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 5 * 1000000;  // 5ms (10ms → 5ms)

    while (is_running_.load()) {
        // pselect()를 사용한 비블로킹 I/O (공식 SDK 방식)
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(serial_fd_, &read_fds);

        int r = pselect(serial_fd_ + 1, &read_fds, NULL, NULL, &timeout, NULL);

        if (r < 0) {
            // pselect() 에러
            if (errno == EINTR) {
                // 인터럽트 발생, 계속 진행
                continue;
            }
            consecutive_errors++;
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                std::cerr << "Too many pselect errors, stopping..." << std::endl;
                is_connected_ = false;
                break;
            }
            continue;
        } else if (r == 0) {
            // 타임아웃, 데이터 없음 (정상)
            continue;
        }

        // 데이터 수신 가능
        if (FD_ISSET(serial_fd_, &read_fds)) {
            int bytes_read = read(serial_fd_, buffer, sizeof(buffer));

            if (bytes_read > 0) {
                consecutive_errors = 0;  // 성공적으로 읽음

                // 패킷 버퍼에 추가
                packet_buffer_.insert(packet_buffer_.end(), buffer, buffer + bytes_read);

                // 패킷 파싱
                while (packet_buffer_.size() >= LD19_PACKET_SIZE) {
                    // 헤더 찾기
                    auto it = std::find(packet_buffer_.begin(), packet_buffer_.end(), static_cast<uint8_t>(LD19_HEADER));

                    if (it == packet_buffer_.end()) {
                        // 헤더 없음, 버퍼 비우기
                        packet_buffer_.clear();
                        break;
                    }

                    // 헤더 위치로 이동
                    packet_buffer_.erase(packet_buffer_.begin(), it);

                    if (packet_buffer_.size() < LD19_PACKET_SIZE) {
                        break;
                    }

                    // 패킷 파싱
                    if (parsePacket(packet_buffer_.data(), LD19_PACKET_SIZE)) {
                        // 파싱 성공, 패킷 제거
                        packets_parsed++;
                        packet_buffer_.erase(packet_buffer_.begin(),
                                            packet_buffer_.begin() + LD19_PACKET_SIZE);
                    } else {
                        // 파싱 실패, 헤더 건너뛰기
                        packets_failed++;
                        packet_buffer_.erase(packet_buffer_.begin());
                    }

                    // 10초마다 통계 출력
                    auto now_stats = std::chrono::steady_clock::now();
                    auto elapsed_stats = std::chrono::duration_cast<std::chrono::seconds>(now_stats - last_stats_time).count();
                    if (elapsed_stats >= 10) {
                        std::cout << "[LiDAR Stats] Parsed: " << packets_parsed
                                  << " Failed: " << packets_failed
                                  << " Buffer size: " << packet_buffer_.size() << " bytes" << std::endl;
                        last_stats_time = now_stats;
                    }
                }
            } else if (bytes_read < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    consecutive_errors++;
                    if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                        std::cerr << "Too many read errors, stopping..." << std::endl;
                        is_connected_ = false;
                        break;
                    }
                }
            }
        }
    }
}

bool LidarInterface::parsePacket(const uint8_t* data, size_t len) {
    // 패킷 크기 검증
    if (len < sizeof(LiDARFrameTypeDef)) {
        return false;
    }
    
    // 패킷 구조체로 캐스팅 (공식 SDK 방식)
    LiDARFrameTypeDef* pkg = (LiDARFrameTypeDef*)data;
    
    // 헤더 및 VerLen 검증
    if (pkg->header != LD19_HEADER || pkg->ver_len != LD19_VERLEN) {
        return false;
    }
    
    // CRC 검증 (공식 SDK 방식)
    // 공식 SDK는 전체 패킷(CRC 제외)을 계산: sizeof(LiDARFrameTypeDef) - 1 = 46바이트
    uint8_t calculated_crc = 0;
    for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef) - 1; i++) {
        calculated_crc = crc_table[(calculated_crc ^ data[i]) & 0xff];
    }
    
    if (calculated_crc != pkg->crc8) {
        // CRC 불일치: 패킷 오류, 무시
        return false;
    }
    
    // 패킷 데이터 읽기 (구조체에서 직접 읽기)
    uint16_t speed = pkg->speed;
    uint16_t start_angle_raw = pkg->start_angle;
    uint16_t end_angle_raw = pkg->end_angle;
    uint16_t timestamp = pkg->timestamp;
    
    // 각도 차이 검증 (공식 SDK 방식)
    // diff = (end_angle / 100 - start_angle / 100 + 360) % 360
    double diff = (end_angle_raw / 100.0 - start_angle_raw / 100.0 + 360.0);
    if (diff >= 360.0) diff -= 360.0;
    
    // 각도 차이가 비정상적으로 큰 경우 필터링
    // 공식 SDK: diff > speed * POINT_PER_PACK / 4500 * 3 / 2
    if (diff > (double)speed * LD19_POINTS_PER_PACK / 4500.0 * 3.0 / 2.0) {
        // 비정상적인 패킷, 무시
        return false;
    }
    
    // 각도 간격 계산 (공식 SDK 방식)
    // diff = ((uint32_t)end_angle + 36000 - (uint32_t)start_angle) % 36000
    uint32_t angle_diff = ((uint32_t)end_angle_raw + 36000 - (uint32_t)start_angle_raw) % 36000;
    float step = angle_diff / (LD19_POINTS_PER_PACK - 1) / 100.0f;  // 도 단위
    float start_angle = (double)start_angle_raw / 100.0;  // 도 단위
    float end_angle = (double)(end_angle_raw % 36000) / 100.0;  // 도 단위
    
    // 12개의 측정 포인트 추출
    LidarScan new_scan;
    new_scan.timestamp = timestamp;  // 패킷의 timestamp 사용

    // 디버깅: 수신된 패킷의 각도 범위 로그 (10초마다)
    static auto last_packet_log = std::chrono::steady_clock::now();
    auto now_packet = std::chrono::steady_clock::now();
    auto elapsed_packet = std::chrono::duration_cast<std::chrono::seconds>(now_packet - last_packet_log).count();
    if (elapsed_packet >= 10) {
        std::cout << "[Packet] start_angle=" << start_angle << "° end_angle=" << end_angle
                  << "° speed=" << speed << std::endl;
        last_packet_log = now_packet;
    }
    
    for (int i = 0; i < LD19_POINTS_PER_PACK; i++) {
        LidarPoint point;
        
        // 거리 읽기 (구조체에서 직접 읽기, mm 단위)
        point.distance = pkg->point[i].distance / 1000.0f;  // mm -> m 변환
        
        // 각도 계산 (공식 SDK 방식)
        // angle = start + i * step
        point.angle = start_angle + i * step;
        if (point.angle >= 360.0f) {
            point.angle -= 360.0f;
        }
        
        // 마지막 포인트는 end_angle로 설정 (공식 SDK 방식: prevent angle invert)
        if (i == LD19_POINTS_PER_PACK - 1) {
            point.angle = end_angle;
        }
        
        // Intensity (신호 강도)
        point.quality = pkg->point[i].confidence;
        
        // 유효 범위 체크 (0.05m ~ 12m)
        // 거리가 0이거나 유효 범위를 벗어나면 무효 데이터
        bool is_valid = (point.distance >= 0.05f && point.distance <= 12.0f);
        
        if (is_valid) {
            new_scan.points.push_back(point);
        }
    }
    
    // 스캔 데이터 업데이트
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        
        // 새 스캔 감지: 각도 기반 + 타임아웃 기반 (패킷 손실 대응)
        bool is_new_scan = false;
        static auto last_scan_time = std::chrono::steady_clock::now();
        auto now_scan = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_scan - last_scan_time).count();

        if (!current_scan_.points.empty() && !new_scan.points.empty()) {
            float last_angle = current_scan_.points.back().angle;
            float first_new_angle = new_scan.points.front().angle;

            // 조건 1: 각도 기반 (공식 SDK 방식)
            bool angle_wrapped = (first_new_angle < 20.0f && last_angle > 340.0f);

            // 조건 2: 타임아웃 기반 (100ms 이상 경과 - 패킷 손실 대응)
            // LD19는 10Hz 회전 = 100ms/회전, 정확히 1회전마다 리셋
            bool timeout_exceeded = (elapsed_ms > 100);

            // 스캔 리셋 (디버그 출력 제거)
            if (angle_wrapped || timeout_exceeded) {
                is_new_scan = true;
                last_scan_time = now_scan;
            }
        } else if (current_scan_.points.empty()) {
            // 첫 스캔
            last_scan_time = now_scan;
        }

        // 새 스캔이 시작되면 이전 데이터 초기화
        if (is_new_scan) {./
            current_scan_.points.clear();
        }

        // 새 포인트를 모두 추가 (공식 SDK는 계속 축적)
        for (const auto& new_point : new_scan.points) {
            current_scan_.points.push_back(new_point);
        }

        // 타임스탬프 업데이트
        current_scan_.timestamp = new_scan.timestamp;
        
        // 각도 순으로 정렬
        std::sort(current_scan_.points.begin(), current_scan_.points.end(),
                  [](const LidarPoint& a, const LidarPoint& b) {
                      return a.angle < b.angle;
                  });
        
        // 최대 360개 포인트 유지 (각도 순으로 정렬된 상태에서)
        if (current_scan_.points.size() > 360) {
            current_scan_.points.erase(current_scan_.points.begin() + 360,
                                      current_scan_.points.end());
        }
    }
    
    return true;
}

bool LidarInterface::getLatestScan(LidarScan& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    if (current_scan_.points.empty()) {
        return false;
    }
    
    scan = current_scan_;
    return true;
}

std::vector<LidarPoint> LidarInterface::getRangeData(float start_angle, float end_angle) {
    std::vector<LidarPoint> result;
    
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    for (const auto& point : current_scan_.points) {
        float angle = point.angle;
        
        // 각도 범위 처리 (0도 근처)
        if (start_angle > end_angle) {
            // 예: 350도 ~ 10도
            if (angle >= start_angle || angle <= end_angle) {
                result.push_back(point);
            }
        } else {
            // 일반 범위
            if (angle >= start_angle && angle <= end_angle) {
                result.push_back(point);
            }
        }
    }
    
    return result;
}

float LidarInterface::getFrontDistance(float tolerance) {
    // 0도(정면)에 가장 가까운 포인트 찾기
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    if (current_scan_.points.empty()) {
        return -1.0f;
    }
    
    float min_angle_diff = 360.0f;
    float front_distance = -1.0f;
    
    for (const auto& point : current_scan_.points) {
        // 각도 차이 계산 (0도에 가장 가까운 것)
        float angle_diff = std::abs(point.angle);
        if (angle_diff > 180.0f) {
            angle_diff = 360.0f - angle_diff;
        }
        
        // 허용 범위 내이고 가장 가까운 포인트
        if (angle_diff <= tolerance && angle_diff < min_angle_diff) {
            min_angle_diff = angle_diff;
            front_distance = point.distance;
        }
    }
    
    // 디버깅: 가장 가까운 포인트가 tolerance를 초과하는 경우
    if (front_distance < 0 && !current_scan_.points.empty()) {
        // 가장 가까운 포인트 찾기 (tolerance 무시)
        float global_min_diff = 360.0f;
        float closest_dist = -1.0f;
        float closest_ang = -1.0f;
        
        for (const auto& point : current_scan_.points) {
            float angle_diff = std::abs(point.angle);
            if (angle_diff > 180.0f) {
                angle_diff = 360.0f - angle_diff;
            }
            
            if (angle_diff < global_min_diff) {
                global_min_diff = angle_diff;
                closest_dist = point.distance;
                closest_ang = point.angle;
            }
        }
        
        // 디버깅 정보 출력 (5초마다)
        static auto last_debug_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_debug_time).count();

        if (elapsed >= 5) {
            std::cout << "LiDAR Debug: Closest point to 0° is at angle=" << closest_ang
                     << "° (diff=" << global_min_diff << "°), distance=" << closest_dist << "m" << std::endl;

            // 각도 분포 샘플 출력 (다양한 각도 범위에서)
            std::vector<float> angle_samples;
            size_t sample_count = std::min(current_scan_.points.size(), size_t(20));
            for (size_t i = 0; i < sample_count; i++) {
                size_t idx = (i * current_scan_.points.size()) / sample_count;
                if (idx < current_scan_.points.size()) {
                    angle_samples.push_back(current_scan_.points[idx].angle);
                }
            }
            std::cout << "LiDAR Debug: Sample angles (20 points): ";
            for (float ang : angle_samples) {
                std::cout << ang << "° ";
            }
            std::cout << std::endl;

            // 각도 범위 확인
            float min_angle = 360.0f, max_angle = 0.0f;
            for (const auto& p : current_scan_.points) {
                if (p.angle < min_angle) min_angle = p.angle;
                if (p.angle > max_angle) max_angle = p.angle;
            }
            std::cout << "LiDAR Debug: Angle range: " << min_angle << "° ~ " << max_angle << "°" << std::endl;
            std::cout << "LiDAR Debug: Total points in scan: " << current_scan_.points.size() << std::endl;
            last_debug_time = now;
        }
    }
    
    return front_distance;
}

size_t LidarInterface::getPointCount() const {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return current_scan_.points.size();
}
