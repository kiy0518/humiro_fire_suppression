#include <rclcpp/rclcpp.hpp>
#include "autonomous/arm_handler.h"
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  ARM Handler 테스트" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // ROS2 초기화
    rclcpp::init(argc, argv);
    
    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("arm_handler_test");
    
    std::cout << "[메인] ROS2 노드 생성: " << node->get_name() << std::endl;
    std::cout << std::endl;
    
    // ArmHandler 생성
    ArmHandler arm_handler(node);
    
    std::cout << std::endl;
    std::cout << "[메인] ArmHandler 준비 완료!" << std::endl;
    std::cout << "[메인] PX4 상태 수신 대기 중 (5초)..." << std::endl;
    std::cout << std::endl;
    
    // PX4 연결 대기
    for (int i = 0; i < 50; i++) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "  테스트 시작" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // 1. OFFBOARD 모드 활성화
    std::cout << "[테스트 1] OFFBOARD 모드 활성화" << std::endl;
    if (arm_handler.enableOffboardMode()) {
        std::cout << "  ✓ 성공!" << std::endl;
    } else {
        std::cerr << "  ✗ 실패!" << std::endl;
    }
    std::cout << std::endl;
    
    // 2. 시동 걸기
    std::cout << "[테스트 2] 시동 걸기 (ARM)" << std::endl;
    if (arm_handler.arm(5000)) {
        std::cout << "  ✓ 시동 성공!" << std::endl;
    } else {
        std::cerr << "  ✗ 시동 실패!" << std::endl;
    }
    std::cout << std::endl;
    
    // 3. 시동 상태 확인
    std::cout << "[테스트 3] 시동 상태 확인" << std::endl;
    if (arm_handler.isArmed()) {
        std::cout << "  ✓ 시동 ON" << std::endl;
    } else {
        std::cout << "  ✗ 시동 OFF" << std::endl;
    }
    std::cout << std::endl;
    
    // 4. 5초 대기
    std::cout << "[메인] 시동 상태 유지 (5초)..." << std::endl;
    for (int i = 0; i < 50; i++) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << std::endl;
    
    // 5. 시동 끄기
    std::cout << "[테스트 4] 시동 끄기 (DISARM)" << std::endl;
    if (arm_handler.disarm(3000)) {
        std::cout << "  ✓ 시동 끄기 성공!" << std::endl;
    } else {
        std::cerr << "  ✗ 시동 끄기 실패!" << std::endl;
    }
    std::cout << std::endl;
    
    std::cout << "========================================" << std::endl;
    std::cout << "  테스트 완료" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 종료
    rclcpp::shutdown();
    return 0;
}
