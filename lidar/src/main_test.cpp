/**
 * LiDAR Integration Test Program
 * 
 * LD19 LiDAR와 연결하여 거리 데이터를 읽고,
 * 카메라 영상에 라인 형태로 오버레이합니다.
 * 
 * 연결 방법:
 *   1. USB-UART 어댑터 (테스트용): /dev/ttyUSB0
 *   2. VIM4 GPIO UART_E (배포용): /dev/ttyS4
 * 
 * 실행 방법:
 *   ./lidar_test [options] [camera_id]
 * 
 * 옵션:
 *   -u, --usb-uart [device]    USB-UART 사용 (기본: /dev/ttyUSB0)
 *   -g, --gpio-uart [device]   GPIO-UART 사용 (기본: /dev/ttyS4, UART_E)
 *   -n, --no-gui               GUI 없이 실행 (headless 모드)
 *   -h, --help                 도움말 표시
 * 
 * 예:
 *   ./lidar_test                           # USB-UART, 카메라 0
 *   ./lidar_test -u /dev/ttyUSB1 1         # USB-UART 1번, 카메라 1
 *   ./lidar_test -g /dev/ttyS4 0           # GPIO-UART (UART_E), 카메라 0
 *   ./lidar_test --gpio-uart               # GPIO-UART 기본, 카메라 0
 */

#include "lidar_interface.h"
#include "lidar_config.h"
#include "distance_overlay.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>
#include <atomic>
#include <cstring>

std::atomic<bool> running(true);

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received." << std::endl;
    running = false;
}

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options] [camera_id]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -u, --usb-uart [device]    Use USB-UART adapter (default: /dev/ttyUSB0)\n";
    std::cout << "  -g, --gpio-uart [device]   Use VIM4 GPIO UART_E (default: /dev/ttyS4)\n";
    std::cout << "  -n, --no-gui               Run without GUI (headless mode)\n";
    std::cout << "  -h, --help                 Show this help message\n\n";
    std::cout << "Arguments:\n";
    std::cout << "  camera_id                  Camera device ID (default: 0)\n\n";
    std::cout << "Connection Types:\n";
    std::cout << "  USB-UART:  For testing with USB-to-Serial adapter\n";
    std::cout << "  GPIO-UART: For deployment using VIM4 GPIO header pins\n\n";
    std::cout << "VIM4 GPIO UART Connection:\n";
    std::cout << VIM4_UART::DESCRIPTION << "\n\n";
    std::cout << "Examples:\n";
    std::cout << "  " << program_name << "                           # USB-UART default, camera 0\n";
    std::cout << "  " << program_name << " -u /dev/ttyUSB1 1         # USB-UART port 1, camera 1\n";
    std::cout << "  " << program_name << " -g /dev/ttyS4 0           # GPIO-UART (UART_E), camera 0\n";
    std::cout << "  " << program_name << " --gpio-uart               # GPIO-UART default, camera 0\n";
    std::cout << "  " << program_name << " -g -n                     # GPIO-UART, no GUI (headless)\n";
}

int main(int argc, char** argv) {
    // 시그널 핸들러 등록
    signal(SIGINT, signalHandler);
    
    // 기본 설정
    LidarConfig lidar_config = LidarConfig::createUSBUartConfig();
    int camera_id = 0;
    bool use_gui = true;  // GUI 사용 여부
    
    // 명령행 인자 파싱
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "-n" || arg == "--no-gui") {
            use_gui = false;
        }
        else if (arg == "-u" || arg == "--usb-uart") {
            // USB-UART 모드
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                lidar_config = LidarConfig::createUSBUartConfig(argv[++i]);
            } else {
                lidar_config = LidarConfig::createUSBUartConfig();
            }
        }
        else if (arg == "-g" || arg == "--gpio-uart") {
            // GPIO-UART 모드
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                lidar_config = LidarConfig::createGPIOUartConfig(argv[++i]);
            } else {
                lidar_config = LidarConfig::createGPIOUartConfig();
            }
        }
        else if (arg[0] != '-') {
            // 카메라 ID
            camera_id = std::stoi(arg);
        }
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "   LiDAR Integration Test Program" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "LiDAR Connection:" << std::endl;
    std::cout << "  Type: " << lidar_config.getConnectionTypeString() << std::endl;
    std::cout << "  Device: " << lidar_config.device_path << std::endl;
    std::cout << "  Baudrate: " << lidar_config.baudrate << std::endl;
    std::cout << "Camera ID: " << camera_id << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // LiDAR 인터페이스 초기화
    LidarInterface lidar(lidar_config);
    
    if (!lidar.start()) {
        std::cerr << "\nFailed to start LiDAR: " << lidar.getLastError() << std::endl;
        std::cerr << "Running in camera-only mode...\n" << std::endl;
    }
    
    // 카메라 열기
    cv::VideoCapture cap(camera_id);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera " << camera_id << std::endl;
        std::cerr << "Try: ls /dev/video*" << std::endl;
        return -1;
    }
    
    // 카메라 설정
    int frame_width = 640;
    int frame_height = 480;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    
    // 거리 오버레이 초기화
    DistanceOverlay overlay(60.0f, frame_width, frame_height);
    
    // 프레임 카운터
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    cv::Mat frame;
    
    std::cout << "Starting main loop..." << std::endl;
    
    while (running) {
        // 프레임 캡처
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Empty frame" << std::endl;
            break;
        }
        
        // LiDAR 데이터 가져오기 (전체 360도)
        if (lidar.isConnected()) {
            // 전체 360도 범위의 데이터 가져오기
            std::vector<LidarPoint> lidar_data = lidar.getRangeData(0.0f, 360.0f);

            if (!lidar_data.empty()) {
                // 360도 원형 레이더 뷰 그리기
                overlay.drawRadarView(frame, lidar_data);
            }
        } else {
            // LiDAR 연결 안됨 표시
            cv::putText(frame, "LiDAR: Disconnected",
                       cv::Point(frame_width / 2 - 100, frame_height / 2),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 0, 255), 2);
        }
        
        // 연결 타입 표시
        std::string conn_type = "UART: " + lidar_config.getConnectionTypeString();
        cv::putText(frame, conn_type,
                   cv::Point(10, frame_height - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(200, 200, 200), 1);
        
        // FPS 계산 및 표시
        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - start_time).count();
        
        if (elapsed >= 1.0f) {
            float fps = frame_count / elapsed;
            
            char fps_text[50];
            snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", fps);
            cv::putText(frame, fps_text,
                       cv::Point(frame_width - 100, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6,
                       cv::Scalar(255, 255, 255), 2);
            
            frame_count = 0;
            start_time = current_time;
        }
        
        // 화면 표시 (GUI 모드인 경우만)
        if (use_gui) {
            cv::imshow("LiDAR Distance Overlay", frame);
            
            // 키 입력 대기 (1ms)
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {  // ESC 또는 'q'
                break;
            }
        } else {
            // Headless 모드: 콘솔에 정보만 출력
            if (frame_count % 30 == 0) {  // 1초마다 (30 FPS 기준)
                if (lidar.isConnected()) {
                    size_t point_count = lidar.getPointCount();
                    
                    if (point_count > 0) {
                        // 정면 거리 가져오기 (0도에 가장 가까운 포인트)
                        // tolerance를 점진적으로 늘려서 시도
                        float front_distance = lidar.getFrontDistance(1.0f);
                        if (front_distance < 0) {
                            front_distance = lidar.getFrontDistance(5.0f);  // ±5도
                        }
                        if (front_distance < 0) {
                            front_distance = lidar.getFrontDistance(10.0f);  // ±10도
                        }
                        
                        if (front_distance > 0) {
                            std::cout << "LiDAR: " << point_count << " points, Front distance: " 
                                     << front_distance << "m" << std::endl;
                        } else {
                            // 데이터는 있지만 정면(0도) 범위에 없음
                            std::cout << "LiDAR: " << point_count << " points, Front distance: No data (no point near 0°)"
                                     << std::endl;
                        }
                    } else {
                        std::cout << "LiDAR: Connected but no data received yet (waiting for scan data...)" << std::endl;
                    }
                } else {
                    std::cout << "LiDAR: Disconnected" << std::endl;
                }
            }
            
            // 짧은 대기 (CPU 사용률 조절)
            std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 FPS
        }
    }
    
    // 정리
    lidar.stop();
    cap.release();
    if (use_gui) {
        cv::destroyAllWindows();
    }
    
    std::cout << "\nProgram terminated." << std::endl;
    
    return 0;
}
