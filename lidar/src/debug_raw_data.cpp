#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <vector>
#include <iomanip>
#include <cmath>
#include <algorithm>

// LD19 프로토콜 상수
#define LD19_HEADER 0x54
#define LD19_VERLEN 0x2C
#define LD19_POINTS_PER_PACK 12
#define LD19_PACKET_SIZE 47

int main(int argc, char* argv[]) {
    const char* device = (argc > 1) ? argv[1] : "/dev/ttyS4";
    int baudrate = (argc > 2) ? atoi(argv[2]) : 230400;
    
    std::cout << "=========================================" << std::endl;
    std::cout << "  LD19 Raw Data Debug Tool" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Device: " << device << std::endl;
    std::cout << "Baudrate: " << baudrate << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;
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
    int packet_count = 0;
    int total_points = 0;
    
    // 각도별 포인트 카운트 (36개 구간: 10도씩)
    int angle_bins[36] = {0};
    
    // 각도별 최소/최대 거리
    float angle_min_dist[36];
    float angle_max_dist[36];
    bool angle_has_data[36] = {false};
    for (int i = 0; i < 36; i++) {
        angle_min_dist[i] = 1000.0f;
        angle_max_dist[i] = 0.0f;
    }
    
    std::cout << "Reading data..." << std::endl;
    std::cout << std::endl;
    
    while (true) {
        uint8_t read_buf[1024];
        int bytes_read = read(fd, read_buf, sizeof(read_buf));
        
        if (bytes_read > 0) {
            buffer.insert(buffer.end(), read_buf, read_buf + bytes_read);
            
            // 패킷 파싱
            while (buffer.size() >= LD19_PACKET_SIZE) {
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
                
                float angle_diff = end_angle - start_angle;
                if (angle_diff > 180.0f) angle_diff -= 360.0f;
                else if (angle_diff < -180.0f) angle_diff += 360.0f;
                
                float angle_step = (LD19_POINTS_PER_PACK > 1) ? 
                    angle_diff / (LD19_POINTS_PER_PACK - 1) : 0.0f;
                
                packet_count++;
                int valid_points = 0;
                
                // 각 포인트 처리
                for (int i = 0; i < LD19_POINTS_PER_PACK; i++) {
                    int offset = 6 + i * 3;
                    if (offset + 2 >= buffer.size()) break;
                    
                    uint16_t distance_mm = buffer[offset] | (buffer[offset + 1] << 8);
                    float distance = distance_mm / 1000.0f;
                    
                    float angle = start_angle + angle_step * i;
                    while (angle >= 360.0f) angle -= 360.0f;
                    while (angle < 0.0f) angle += 360.0f;
                    
                    if (distance >= 0.05f && distance <= 12.0f) {
                        valid_points++;
                        total_points++;
                        
                        // 각도 구간별 통계 (10도씩)
                        int bin = static_cast<int>(angle / 10.0f);
                        if (bin >= 0 && bin < 36) {
                            angle_bins[bin]++;
                            if (!angle_has_data[bin] || distance < angle_min_dist[bin]) {
                                angle_min_dist[bin] = distance;
                            }
                            if (!angle_has_data[bin] || distance > angle_max_dist[bin]) {
                                angle_max_dist[bin] = distance;
                            }
                            angle_has_data[bin] = true;
                        }
                    }
                }
                
                // 100개 패킷마다 통계 출력
                if (packet_count % 100 == 0) {
                    std::cout << "\n[Packet " << packet_count << "] " 
                              << "Start: " << std::fixed << std::setprecision(2) << start_angle 
                              << "°, End: " << end_angle 
                              << "°, Valid points: " << valid_points << std::endl;
                }
                
                buffer.erase(buffer.begin(), buffer.begin() + LD19_PACKET_SIZE);
            }
        }
        
        // 1000개 패킷마다 각도 분포 출력
        if (packet_count > 0 && packet_count % 1000 == 0) {
            std::cout << "\n=========================================" << std::endl;
            std::cout << "  Angle Distribution (after " << packet_count << " packets)" << std::endl;
            std::cout << "  Total points: " << total_points << std::endl;
            std::cout << "=========================================" << std::endl;
            
            // 각도 구간별 출력 (10도씩)
            for (int i = 0; i < 36; i++) {
                float angle_start = i * 10.0f;
                float angle_end = (i + 1) * 10.0f;
                
                std::cout << std::setw(5) << std::fixed << std::setprecision(1) << angle_start 
                          << "°~" << std::setw(5) << angle_end << "°: "
                          << std::setw(6) << angle_bins[i] << " points";
                
                if (angle_has_data[i]) {
                    std::cout << " [dist: " << std::setprecision(2) << angle_min_dist[i] 
                              << "~" << angle_max_dist[i] << "m]";
                }
                
                // 왼쪽 영역 강조 (180~270도, 270~360도)
                if ((angle_start >= 180.0f && angle_start < 270.0f) ||
                    (angle_start >= 270.0f && angle_start < 360.0f)) {
                    if (angle_bins[i] == 0) {
                        std::cout << " ⚠ NO DATA";
                    } else if (angle_bins[i] < 10) {
                        std::cout << " ⚠ LOW";
                    }
                }
                
                std::cout << std::endl;
            }
            
            std::cout << "=========================================" << std::endl;
            std::cout << std::endl;
        }
        
        usleep(1000);
    }
    
    close(fd);
    return 0;
}

