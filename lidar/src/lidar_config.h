#ifndef LIDAR_CONFIG_H
#define LIDAR_CONFIG_H

#include <string>

/**
 * LiDAR 연결 설정
 */

// 연결 타입
enum class UartConnectionType {
    USB_UART,      // USB-UART 어댑터 (테스트용)
    GPIO_UART      // VIM4 GPIO 헤더 핀 UART (배포용)
};

// 디바이스 경로 설정
struct LidarConfig {
    UartConnectionType connection_type;
    std::string device_path;
    int baudrate;
    
    // 생성자 - 기본값은 USB-UART
    LidarConfig() 
        : connection_type(UartConnectionType::USB_UART)
        , device_path("/dev/ttyUSB0")
        , baudrate(230400) {
    }
    
    // USB-UART 설정으로 초기화
    static LidarConfig createUSBUartConfig(const std::string& device = "/dev/ttyUSB0") {
        LidarConfig config;
        config.connection_type = UartConnectionType::USB_UART;
        config.device_path = device;
        config.baudrate = 230400;
        return config;
    }
    
    // GPIO UART 설정으로 초기화
    // VIM4 GPIO UART_E는 /dev/ttyS4 사용 (기본값)
    static LidarConfig createGPIOUartConfig(const std::string& device = "/dev/ttyS4") {
        LidarConfig config;
        config.connection_type = UartConnectionType::GPIO_UART;
        config.device_path = device;
        config.baudrate = 230400;
        return config;
    }
    
    // UART_E 전용 설정 (VIM4)
    static LidarConfig createUartEConfig() {
        LidarConfig config;
        config.connection_type = UartConnectionType::GPIO_UART;
        config.device_path = "/dev/ttyS4";
        config.baudrate = 230400;
        return config;
    }
    
    // 연결 타입 문자열 반환
    std::string getConnectionTypeString() const {
        switch (connection_type) {
            case UartConnectionType::USB_UART:
                return "USB-UART";
            case UartConnectionType::GPIO_UART:
                return "GPIO-UART";
            default:
                return "Unknown";
        }
    }
};

// VIM4 GPIO UART 핀 정보 (참고용)
namespace VIM4_UART {
    // UART_A (일반적으로 /dev/ttyS1)
    constexpr int UART_A_TX_PIN = 8;   // GPIO 핀 번호 (Header Pin 8)
    constexpr int UART_A_RX_PIN = 10;  // GPIO 핀 번호 (Header Pin 10)
    constexpr const char* UART_A_DEV = "/dev/ttyS1";
    
    // UART_B (일반적으로 /dev/ttyS2, 사용 가능 시)
    constexpr const char* UART_B_DEV = "/dev/ttyS2";
    
    // UART_E (LiDAR 사용, /dev/ttyS4) ⭐
    constexpr int UART_E_TX_PIN = 16;  // GPIO 핀 번호 (Header Pin 16)
    constexpr int UART_E_RX_PIN = 15;  // GPIO 핀 번호 (Header Pin 15)
    constexpr const char* UART_E_DEV = "/dev/ttyS4";
    
    // 설명
    constexpr const char* DESCRIPTION = 
        "VIM4 40-Pin GPIO Header - UART_E (LiDAR 사용):\n"
        "  Pin 16 (TX) - UART_E TX → LiDAR RX\n"
        "  Pin 15 (RX) - UART_E RX → LiDAR TX\n"
        "  Pin 6  (GND) - Ground\n"
        "  Pin 2/4 (5V) - Power (if needed)\n"
        "  Device: /dev/ttyS4";
}

#endif // LIDAR_CONFIG_H
