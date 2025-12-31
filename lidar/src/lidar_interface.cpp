#include "lidar_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>

// LD19 프로토콜 상수 (V2.5 Manual 기준)
#define LD19_HEADER 0x54
#define LD19_VERLEN 0x2C  // 상위 3비트: 패킷 타입(1), 하위 5비트: 포인트 수(12)
#define LD19_POINTS_PER_PACK 12
#define LD19_PACKET_SIZE 47  // Header(1) + VerLen(1) + Speed(2) + StartAngle(2) + Data(36) + EndAngle(2) + Timestamp(2) + CRC(1)

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
    uint8_t buffer[1024];
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 100;
    
    while (is_running_.load()) {
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
                    packet_buffer_.erase(packet_buffer_.begin(), 
                                        packet_buffer_.begin() + LD19_PACKET_SIZE);
                } else {
                    // 파싱 실패, 헤더 건너뛰기
                    packet_buffer_.erase(packet_buffer_.begin());
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
        
        usleep(1000);  // 1ms 대기
    }
}

bool LidarInterface::parsePacket(const uint8_t* data, size_t len) {
    // 패킷 크기 및 헤더 검증
    if (len < LD19_PACKET_SIZE || data[0] != LD19_HEADER) {
        return false;
    }
    
    // VerLen 검증 (0x2C = 패킷 타입(1) + 포인트 수(12))
    if (data[1] != LD19_VERLEN) {
        return false;
    }
    
    // 패킷 구조 (LD19 V2.5 Manual 기준):
    // [0] Header (0x54)
    // [1] VerLen (0x2C)
    // [2-3] Speed (LSB, MSB) - degrees per second
    // [4-5] Start angle (LSB, MSB) - 0.01도 단위
    // [6-41] Data (12 points × 3 bytes = 36 bytes)
    // [42-43] End angle (LSB, MSB) - 0.01도 단위
    // [44-45] Timestamp (LSB, MSB) - milliseconds
    // [46] CRC check
    
    // 시작 각도 및 종료 각도 읽기 (little-endian, 0.01도 단위)
    uint16_t start_angle_raw = data[4] | (data[5] << 8);
    uint16_t end_angle_raw = data[42] | (data[43] << 8);
    
    // 각도를 도 단위로 변환
    float start_angle = start_angle_raw / 100.0f;
    float end_angle = end_angle_raw / 100.0f;
    
    // 각도 정규화 (0~360도 범위)
    while (start_angle >= 360.0f) start_angle -= 360.0f;
    while (start_angle < 0.0f) start_angle += 360.0f;
    while (end_angle >= 360.0f) end_angle -= 360.0f;
    while (end_angle < 0.0f) end_angle += 360.0f;
    
    // 각도 간격 계산 (Manual V2.5 공식)
    // step = (end_angle - start_angle) / (len - 1)
    // angle = start_angle + step * i
    float angle_step = 0.0f;
    if (LD19_POINTS_PER_PACK > 1) {
        // 각도 차이 계산 (360도 경계 고려)
        float angle_diff = end_angle - start_angle;
        if (angle_diff > 180.0f) {
            angle_diff -= 360.0f;
        } else if (angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }
        angle_step = angle_diff / (LD19_POINTS_PER_PACK - 1);
    }
    
    // Timestamp 읽기 (little-endian, milliseconds)
    uint16_t timestamp = data[44] | (data[45] << 8);
    
    // 12개의 측정 포인트 추출
    LidarScan new_scan;
    new_scan.timestamp = timestamp;  // 패킷의 timestamp 사용
    
    for (int i = 0; i < LD19_POINTS_PER_PACK; i++) {
        // Data 섹션 시작 위치: 6번째 바이트부터
        int offset = 6 + i * 3;  // 각 포인트는 3바이트 (distance 2바이트 + intensity 1바이트)
        
        if (static_cast<size_t>(offset + 2) >= len) break;
        
        LidarPoint point;
        
        // 거리 읽기 (little-endian, mm 단위)
        uint16_t distance_mm = data[offset] | (data[offset + 1] << 8);
        point.distance = distance_mm / 1000.0f;  // mm -> m 변환
        
        // 각도 계산 (Manual V2.5 공식)
        point.angle = start_angle + angle_step * i;
        
        // 각도 정규화 (0~360도 범위)
        while (point.angle >= 360.0f) point.angle -= 360.0f;
        while (point.angle < 0.0f) point.angle += 360.0f;
        
        // Intensity (신호 강도)
        point.quality = data[offset + 2];
        
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
        
        // 새 포인트 추가
        for (const auto& new_point : new_scan.points) {
            // 중복 제거: 같은 각도(±0.5도)의 포인트가 있으면 업데이트, 없으면 추가
            bool found = false;
            for (auto& existing_point : current_scan_.points) {
                float angle_diff = std::abs(existing_point.angle - new_point.angle);
                if (angle_diff > 180.0f) {
                    angle_diff = 360.0f - angle_diff;
                }
                if (angle_diff < 0.5f) {
                    // 같은 각도 범위의 포인트 업데이트
                    existing_point = new_point;
                    found = true;
                    break;
                }
            }
            if (!found) {
                current_scan_.points.push_back(new_point);
            }
        }
        
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
        
        // 디버깅 정보 출력 (첫 번째 호출 시에만)
        static bool debug_printed = false;
        if (!debug_printed) {
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
            debug_printed = true;
        }
    }
    
    return front_distance;
}

size_t LidarInterface::getPointCount() const {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return current_scan_.points.size();
}
