#ifndef LIDAR_INTERFACE_H
#define LIDAR_INTERFACE_H

#include "lidar_config.h"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <termios.h>

/**
 * LDROBOT LD19 LiDAR Interface
 * 
 * LD19 프로토콜:
 * - 360도 스캔 (1도 해상도)
 * - 스캔 속도: 10Hz
 * - 측정 범위: 0.05m ~ 12m
 * - 통신: UART (230400 baud)
 * 
 * 연결 방법:
 * 1. USB-UART 어댑터: /dev/ttyUSB0 (테스트용)
 * 2. VIM4 GPIO UART_E: /dev/ttyS4 (배포용)
 */

struct LidarPoint {
    float angle;      // 각도 (0~360도)
    float distance;   // 거리 (미터)
    uint8_t quality;  // 신호 품질 (0~255)
};

struct LidarScan {
    std::vector<LidarPoint> points;
    uint64_t timestamp;  // 스캔 타임스탬프
};

class LidarInterface {
public:
    /**
     * 생성자 (기본: USB-UART)
     * @param device 시리얼 포트 경로 (예: "/dev/ttyUSB0")
     * @param baudrate 통신 속도 (기본: 230400)
     */
    LidarInterface(const std::string& device = "/dev/ttyUSB0", 
                   int baudrate = 230400);
    
    /**
     * 생성자 (설정 객체 사용)
     * @param config LiDAR 설정
     */
    explicit LidarInterface(const LidarConfig& config);
    
    ~LidarInterface();
    
    /**
     * LiDAR 연결 및 시작
     * @return 성공 시 true
     */
    bool start();
    
    /**
     * LiDAR 중지 및 연결 해제
     */
    void stop();
    
    /**
     * 최신 스캔 데이터 가져오기
     * @param scan 출력 스캔 데이터
     * @return 성공 시 true
     */
    bool getLatestScan(LidarScan& scan);
    
    /**
     * 특정 각도 범위의 거리 데이터 가져오기
     * @param start_angle 시작 각도 (도)
     * @param end_angle 종료 각도 (도)
     * @return 해당 범위의 포인트들
     */
    std::vector<LidarPoint> getRangeData(float start_angle, float end_angle);
    
    /**
     * 정면 거리 가져오기 (0도에 가장 가까운 포인트)
     * @param tolerance 허용 각도 범위 (도, 기본: 1도)
     * @return 정면 거리 (미터), 데이터 없으면 -1
     */
    float getFrontDistance(float tolerance = 1.0f);
    
    /**
     * 현재 스캔 데이터의 포인트 개수 가져오기
     * @return 포인트 개수
     */
    size_t getPointCount() const;
    
    /**
     * LiDAR 연결 상태 확인
     * @return 연결됨 true, 끊김 false
     */
    bool isConnected() const { return is_connected_.load(); }
    
    /**
     * 마지막 에러 메시지 가져오기
     */
    std::string getLastError() const { return last_error_; }
    
    /**
     * 현재 설정 가져오기
     */
    LidarConfig getConfig() const { return config_; }

private:
    // 시리얼 포트 설정
    bool openSerialPort();
    void closeSerialPort();
    bool configureSerialPort();
    
    // GPIO UART 특별 설정 (필요시)
    bool configureGPIOUart();
    
    // 데이터 수신 스레드
    void receiveThread();
    
    // LD19 프로토콜 파싱
    bool parsePacket(const uint8_t* data, size_t len);
    
    // 멤버 변수
    LidarConfig config_;
    int serial_fd_;
    
    std::atomic<bool> is_connected_;
    std::atomic<bool> is_running_;
    
    std::thread receive_thread_;
    
    LidarScan current_scan_;
    mutable std::mutex scan_mutex_;
    
    std::string last_error_;
    
    // LD19 패킷 파싱 버퍼
    std::vector<uint8_t> packet_buffer_;
};

#endif // LIDAR_INTERFACE_H
