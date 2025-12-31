/**
 * LiDAR 연결 설정 예제
 * 
 * 이 파일은 코드에서 LiDAR 연결 설정을 사용하는 방법을 보여줍니다.
 */

#include "lidar_interface.h"
#include "lidar_config.h"
#include <iostream>

void example_usb_uart() {
    std::cout << "=== USB-UART 연결 예제 ===" << std::endl;
    
    // 방법 1: 기본 생성자 (USB-UART, /dev/ttyUSB0)
    LidarInterface lidar1;
    
    // 방법 2: 디바이스 경로 지정
    LidarInterface lidar2("/dev/ttyUSB1");
    
    // 방법 3: 설정 객체 사용
    LidarConfig config = LidarConfig::createUSBUartConfig("/dev/ttyUSB0");
    LidarInterface lidar3(config);
    
    // 시작
    if (lidar1.start()) {
        std::cout << "LiDAR started successfully" << std::endl;
        
        // 데이터 읽기
        std::vector<LidarPoint> data = lidar1.getRangeData(330.0f, 30.0f);
        std::cout << "Received " << data.size() << " points" << std::endl;
        
        lidar1.stop();
    }
}

void example_gpio_uart() {
    std::cout << "=== GPIO-UART 연결 예제 ===" << std::endl;
    
    // 방법 1: 기본 GPIO UART (/dev/ttyS1)
    LidarConfig config1 = LidarConfig::createGPIOUartConfig();
    LidarInterface lidar1(config1);
    
    // 방법 2: 다른 UART 포트 지정
    LidarConfig config2 = LidarConfig::createGPIOUartConfig("/dev/ttyS2");
    LidarInterface lidar2(config2);
    
    // 시작
    if (lidar1.start()) {
        std::cout << "GPIO UART LiDAR started" << std::endl;
        std::cout << "Connection: " << lidar1.getConfig().getConnectionTypeString() << std::endl;
        std::cout << "Device: " << lidar1.getConfig().device_path << std::endl;
        
        lidar1.stop();
    }
}

void example_runtime_selection() {
    std::cout << "=== 런타임 연결 타입 선택 예제 ===" << std::endl;
    
    bool use_gpio = true;  // 설정 파일이나 환경 변수에서 읽음
    
    LidarConfig config;
    if (use_gpio) {
        config = LidarConfig::createGPIOUartConfig();
        std::cout << "Using GPIO UART for deployment" << std::endl;
    } else {
        config = LidarConfig::createUSBUartConfig();
        std::cout << "Using USB-UART for testing" << std::endl;
    }
    
    LidarInterface lidar(config);
    lidar.start();
    
    // ... 작업 수행 ...
    
    lidar.stop();
}

int main() {
    std::cout << "LiDAR Connection Examples\n" << std::endl;
    
    // 예제 1: USB-UART
    example_usb_uart();
    std::cout << std::endl;
    
    // 예제 2: GPIO-UART
    example_gpio_uart();
    std::cout << std::endl;
    
    // 예제 3: 런타임 선택
    example_runtime_selection();
    std::cout << std::endl;
    
    return 0;
}
