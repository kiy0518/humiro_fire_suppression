#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <vector>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <map>

// LD19 프로토콜 상수
#define LD19_HEADER 0x54
#define LD19_VERLEN 0x2C
#define LD19_POINTS_PER_PACK 12
#define LD19_PACKET_SIZE 47

struct PacketInfo {
    float start_angle;
    float end_angle;
    float angle_diff;
    int valid_points;
    bool is_reverse;
    bool crosses_boundary;
};

int main(int argc, char* argv[]) {
    const char* device = (argc > 1) ? argv[1] : "/dev/ttyS4";
    int baudrate = (argc > 2) ? atoi(argv[2]) : 230400;
    int max_packets = (argc > 3) ? atoi(argv[3]) : 5000;
    
    std::cout << "=========================================" << std::endl;
    std::cout << "  LD19 Detailed Analysis Tool" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Device: " << device << std::endl;
    std::cout << "Baudrate: " << baudrate << std::endl;
    std::cout << "Max packets: " << max_packets << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << std::endl;
    
    // 시리얼 포트 열기
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "Failed to open " << device << ": " << strerror(errno) << std::endl;
        return 1;
    }
    
    // 시리얼 포트 설정
    struct termios options;
    tcgetattr(fd, &options);
    
    speed_t baud_const = B230400;
    switch (baudrate) {
        case 9600:   baud_const = B9600; break;
        case 19200:  baud_const = B19200; break;
        case 38400:  baud_const = B38400; break;
        case 57600:  baud_const = B57600; break;
        case 115200: baud_const = B115200; break;
        case 230400: baud_const = B230400; break;
    }
    
    cfsetispeed(&options, baud_const);
    cfsetospeed(&options, baud_const);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    
    tcsetattr(fd, TCSANOW, &options);
    
    std::vector<uint8_t> buffer;
    std::vector<PacketInfo> packets;
    int packet_count = 0;
    
    // 각도별 포인트 카운트 (36개 구간: 10도씩)
    int angle_bins[36] = {0};
    
    // 각도 범위별 패킷 통계
    int boundary_crossing = 0;  // 360도 경계 넘어가는 패킷
    int reverse_packets = 0;     // 역방향 패킷
    int large_diff_packets = 0;  // 각도 차이가 60도 이상인 패킷
    int normal_packets = 0;      // 정상 패킷
    
    std::cout << "Collecting data..." << std::endl;
    
    while (packet_count < max_packets) {
        uint8_t read_buf[1024];
        int bytes_read = read(fd, read_buf, sizeof(read_buf));
        
        if (bytes_read > 0) {
            buffer.insert(buffer.end(), read_buf, read_buf + bytes_read);
            
            // 패킷 파싱
            while (buffer.size() >= LD19_PACKET_SIZE && packet_count < max_packets) {
                // 헤더 찾기
                auto it = std::find(buffer.begin(), buffer.end(), static_cast<uint8_t>(LD19_HEADER));
                if (it == buffer.end()) {
                    buffer.clear();
                    break;
                }
                
                buffer.erase(buffer.begin(), it);
                
                if (buffer.size() < LD19_PACKET_SIZE) {
                    break;
                }
                
                // VerLen 검증
                if (buffer[1] != LD19_VERLEN) {
                    buffer.erase(buffer.begin());
                    continue;
                }
                
                // 패킷 파싱
                uint16_t start_angle_raw = buffer[4] | (buffer[5] << 8);
                uint16_t end_angle_raw = buffer[42] | (buffer[43] << 8);
                
                float start_angle = start_angle_raw / 100.0f;
                float end_angle = end_angle_raw / 100.0f;
                
                while (start_angle >= 360.0f) start_angle -= 360.0f;
                while (start_angle < 0.0f) start_angle += 360.0f;
                while (end_angle >= 360.0f) end_angle -= 360.0f;
                while (end_angle < 0.0f) end_angle += 360.0f;
                
                // 각도 차이 계산
                float angle_diff = end_angle - start_angle;
                bool crosses_boundary = false;
                bool is_reverse = false;
                
                // 360도 경계 넘어가는지 확인
                if (start_angle > 270.0f && end_angle < 90.0f) {
                    // 예: 289.55° → 36.04°
                    angle_diff = (360.0f - start_angle) + end_angle;
                    crosses_boundary = true;
                    boundary_crossing++;
                } else if (start_angle < 90.0f && end_angle > 270.0f) {
                    // 예: 36.04° → 289.55° (역방향)
                    angle_diff = -(start_angle + (360.0f - end_angle));
                    is_reverse = true;
                    reverse_packets++;
                } else {
                    // 일반적인 경우
                    if (angle_diff < 0) {
                        is_reverse = true;
                        reverse_packets++;
                    } else {
                        normal_packets++;
                    }
                }
                
                // 각도 차이가 비정상적으로 큰지 확인
                float abs_angle_diff = std::abs(angle_diff);
                if (abs_angle_diff > 60.0f) {
                    large_diff_packets++;
                }
                
                PacketInfo info;
                info.start_angle = start_angle;
                info.end_angle = end_angle;
                info.angle_diff = angle_diff;
                info.is_reverse = is_reverse;
                info.crosses_boundary = crosses_boundary;
                
                // 유효 포인트 계산
                int valid_points = 0;
                float angle_step = (LD19_POINTS_PER_PACK > 1) ? 
                    angle_diff / (LD19_POINTS_PER_PACK - 1) : 0.0f;
                
                for (int i = 0; i < LD19_POINTS_PER_PACK; i++) {
                    int offset = 6 + i * 3;
                    if (offset + 2 >= buffer.size()) break;
                    
                    uint16_t distance_mm = buffer[offset] | (buffer[offset + 1] << 8);
                    float distance = distance_mm / 1000.0f;
                    
                    if (distance >= 0.05f && distance <= 12.0f) {
                        valid_points++;
                        
                        float angle = start_angle + angle_step * i;
                        while (angle >= 360.0f) angle -= 360.0f;
                        while (angle < 0.0f) angle += 360.0f;
                        
                        int bin = static_cast<int>(angle / 10.0f);
                        if (bin >= 0 && bin < 36) {
                            angle_bins[bin]++;
                        }
                    }
                }
                
                info.valid_points = valid_points;
                packets.push_back(info);
                
                packet_count++;
                buffer.erase(buffer.begin(), buffer.begin() + LD19_PACKET_SIZE);
            }
        }
        
        usleep(1000);
    }
    
    close(fd);
    
    // 결과 출력
    std::cout << "\n=========================================" << std::endl;
    std::cout << "  Analysis Results (" << packet_count << " packets)" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << std::endl;
    
    // 패킷 통계
    std::cout << "Packet Statistics:" << std::endl;
    std::cout << "  Normal packets: " << normal_packets 
              << " (" << (normal_packets * 100.0f / packet_count) << "%)" << std::endl;
    std::cout << "  Boundary crossing: " << boundary_crossing 
              << " (" << (boundary_crossing * 100.0f / packet_count) << "%)" << std::endl;
    std::cout << "  Reverse packets: " << reverse_packets 
              << " (" << (reverse_packets * 100.0f / packet_count) << "%)" << std::endl;
    std::cout << "  Large diff (>60°): " << large_diff_packets 
              << " (" << (large_diff_packets * 100.0f / packet_count) << "%)" << std::endl;
    std::cout << std::endl;
    
    // 각도 차이 분포
    std::map<int, int> diff_distribution;
    for (const auto& p : packets) {
        int diff_bin = static_cast<int>(std::abs(p.angle_diff) / 5.0f);  // 5도 단위
        diff_distribution[diff_bin]++;
    }
    
    std::cout << "Angle Difference Distribution (5° bins):" << std::endl;
    for (const auto& pair : diff_distribution) {
        float diff_start = pair.first * 5.0f;
        float diff_end = (pair.first + 1) * 5.0f;
        std::cout << "  " << std::setw(5) << std::fixed << std::setprecision(1) << diff_start
                  << "°~" << std::setw(5) << diff_end << "°: " 
                  << std::setw(5) << pair.second << " packets" << std::endl;
    }
    std::cout << std::endl;
    
    // 각도별 포인트 분포
    std::cout << "Angle Distribution (10° bins):" << std::endl;
    int total_points = 0;
    for (int i = 0; i < 36; i++) {
        total_points += angle_bins[i];
    }
    
    for (int i = 0; i < 36; i++) {
        float angle_start = i * 10.0f;
        float angle_end = (i + 1) * 10.0f;
        
        float percentage = (angle_bins[i] * 100.0f / total_points);
        
        std::cout << std::setw(5) << std::fixed << std::setprecision(1) << angle_start 
                  << "°~" << std::setw(5) << angle_end << "°: "
                  << std::setw(6) << angle_bins[i] << " points (" 
                  << std::setprecision(1) << percentage << "%)";
        
        // 왼쪽 영역 강조
        if ((angle_start >= 180.0f && angle_start < 270.0f) ||
            (angle_start >= 270.0f && angle_start < 360.0f)) {
            if (percentage < 2.0f) {
                std::cout << " ⚠ VERY LOW";
            } else if (percentage < 2.5f) {
                std::cout << " ⚠ LOW";
            }
        }
        
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // 문제 패킷 샘플
    std::cout << "Problem Packet Samples:" << std::endl;
    int sample_count = 0;
    for (const auto& p : packets) {
        if (p.crosses_boundary || p.is_reverse || std::abs(p.angle_diff) > 60.0f) {
            std::cout << "  Start: " << std::setprecision(2) << p.start_angle 
                      << "°, End: " << p.end_angle 
                      << "°, Diff: " << p.angle_diff 
                      << "°, Points: " << p.valid_points;
            if (p.crosses_boundary) std::cout << " [BOUNDARY]";
            if (p.is_reverse) std::cout << " [REVERSE]";
            if (std::abs(p.angle_diff) > 60.0f) std::cout << " [LARGE_DIFF]";
            std::cout << std::endl;
            sample_count++;
            if (sample_count >= 20) break;
        }
    }
    
    std::cout << "\n=========================================" << std::endl;
    
    return 0;
}

