#include "arm_handler.h"
#include <iostream>
#include <thread>

// PX4 Vehicle Command 코드
#define VEHICLE_CMD_COMPONENT_ARM_DISARM 400
#define VEHICLE_CMD_DO_SET_MODE 176

ArmHandler::ArmHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
    , is_armed_(false)
    , nav_state_(0)
    , arming_state_(0)
{
    if (!node_) {
        throw std::runtime_error("ArmHandler: node is nullptr");
    }
    
    std::cout << "[ArmHandler] 초기화 중..." << std::endl;
    
    // QoS 설정 (PX4 uXRCE-DDS 호환)
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
    
    // Publishers 생성
    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_profile);
    
    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_profile);
    
    // Subscriber 생성
    vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", qos_profile,
        std::bind(&ArmHandler::vehicleStatusCallback, this, std::placeholders::_1));
    
    std::cout << "[ArmHandler] ✓ ROS2 토픽 초기화 완료" << std::endl;
    std::cout << "[ArmHandler]   - Publisher: /fmu/in/vehicle_command" << std::endl;
    std::cout << "[ArmHandler]   - Publisher: /fmu/in/offboard_control_mode" << std::endl;
    std::cout << "[ArmHandler]   - Subscriber: /fmu/out/vehicle_status" << std::endl;
}

ArmHandler::~ArmHandler() {
    // Offboard 타이머 중지
    if (offboard_timer_) {
        offboard_timer_->cancel();
    }
    std::cout << "[ArmHandler] 종료" << std::endl;
}

void ArmHandler::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    arming_state_ = msg->arming_state;
    nav_state_ = msg->nav_state;
    
    // arming_state: 1 = STANDBY, 2 = ARMED
    is_armed_ = (arming_state_ == 2);
    
    // 디버깅 (10초마다 출력)
    static auto last_print = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 10) {
        std::cout << "[ArmHandler] 상태: arming=" << (int)arming_state_ 
                  << ", nav=" << (int)nav_state_ 
                  << ", armed=" << (is_armed_ ? "ON" : "OFF") << std::endl;
        last_print = now;
    }
}

void ArmHandler::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    
    vehicle_command_pub_->publish(msg);
}

void ArmHandler::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    
    offboard_control_mode_pub_->publish(msg);
}

bool ArmHandler::enableOffboardMode() {
    std::cout << "[ArmHandler] OFFBOARD 모드 활성화 중..." << std::endl;
    
    // Offboard heartbeat 시작 (2Hz)
    // PX4는 Offboard 모드를 유지하려면 지속적인 heartbeat가 필요함
    if (!offboard_timer_) {
        try {
            offboard_timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(500),  // 2Hz
                std::bind(&ArmHandler::publishOffboardControlMode, this));
            
            std::cout << "[ArmHandler] ✓ Offboard heartbeat 시작 (2Hz)" << std::endl;
        } catch (const std::runtime_error& e) {
            // executor 관련 예외는 특별히 처리
            std::string error_msg = e.what();
            if (error_msg.find("already been added to an executor") != std::string::npos) {
                std::cerr << "[ArmHandler] ⚠ 타이머 생성 실패 (executor 충돌): " << e.what() << std::endl;
                std::cerr << "  → 메인 스레드의 executor와 충돌했습니다. 타이머 없이 계속 진행합니다." << std::endl;
                // 타이머 없이도 계속 진행 가능 (수동으로 heartbeat 전송)
            } else {
                std::cerr << "[ArmHandler] ✗ 타이머 생성 실패: " << e.what() << std::endl;
                throw;  // 다른 예외는 다시 throw
            }
        } catch (const std::exception& e) {
            std::cerr << "[ArmHandler] ✗ 타이머 생성 실패: " << e.what() << std::endl;
            throw;  // 예외를 다시 throw하여 호출자에게 전달
        }
    }
    
    // Offboard 모드 명령 전송 (여러 번 전송)
    for (int i = 0; i < 5; i++) {
        // MAV_CMD_DO_SET_MODE (176)
        // param1: 1 (custom mode enable)
        // param2: 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        publishVehicleCommand(VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        try {
            rclcpp::spin_some(node_);
        } catch (const std::exception& e) {
            std::cerr << "[ArmHandler] spin_some 예외 (무시): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ArmHandler] spin_some 알 수 없는 예외 (무시)" << std::endl;
        }
    }
    
    std::cout << "[ArmHandler] ✓ OFFBOARD 모드 명령 전송 완료" << std::endl;
    
    // 모드 변경 대기 (최대 3초)
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < 3000) {
        try {
            rclcpp::spin_some(node_);
        } catch (const std::exception& e) {
            std::cerr << "[ArmHandler] spin_some 예외 (무시): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ArmHandler] spin_some 알 수 없는 예외 (무시)" << std::endl;
        }
        
        // nav_state == 14 (OFFBOARD)
        if (nav_state_ == 14) {
            std::cout << "[ArmHandler] ✓ OFFBOARD 모드 활성화 성공!" << std::endl;
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cerr << "[ArmHandler] ✗ OFFBOARD 모드 활성화 실패 (타임아웃)" << std::endl;
    return false;
}

bool ArmHandler::arm(int timeout_ms) {
    std::cout << "[ArmHandler] 시동 걸기 시작..." << std::endl;
    
    // 이미 시동이 걸려있는지 확인
    if (is_armed_) {
        std::cout << "[ArmHandler] ✓ 이미 시동이 걸려 있습니다" << std::endl;
        return true;
    }
    
    // ARM 명령 전송 (여러 번)
    for (int i = 0; i < 5; i++) {
        // MAV_CMD_COMPONENT_ARM_DISARM (400)
        // param1: 1.0 = ARM, 0.0 = DISARM
        publishVehicleCommand(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        
        std::cout << "[ArmHandler] ARM 명령 전송 (" << (i + 1) << "/5)" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        try {
            rclcpp::spin_some(node_);
        } catch (const std::exception& e) {
            std::cerr << "[ArmHandler] spin_some 예외 (무시): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ArmHandler] spin_some 알 수 없는 예외 (무시)" << std::endl;
        }
    }
    
    // 시동 확인 대기
    std::cout << "[ArmHandler] 시동 상태 확인 중..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {
        try {
            rclcpp::spin_some(node_);
        } catch (const std::exception& e) {
            std::cerr << "[ArmHandler] spin_some 예외 (무시): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ArmHandler] spin_some 알 수 없는 예외 (무시)" << std::endl;
        }
        
        if (is_armed_) {
            std::cout << "[ArmHandler] ✓✓✓ 시동 성공! ✓✓✓" << std::endl;
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cerr << "[ArmHandler] ✗✗✗ 시동 실패 (타임아웃 " << timeout_ms << "ms) ✗✗✗" << std::endl;
    std::cerr << "[ArmHandler]   현재 arming_state: " << (int)arming_state_ << std::endl;
    return false;
}

bool ArmHandler::disarm(int timeout_ms) {
    std::cout << "[ArmHandler] 시동 끄기 시작..." << std::endl;
    
    // 이미 시동이 꺼져있는지 확인
    if (!is_armed_) {
        std::cout << "[ArmHandler] ✓ 이미 시동이 꺼져 있습니다" << std::endl;
        return true;
    }
    
    // DISARM 명령 전송 (여러 번)
    for (int i = 0; i < 5; i++) {
        // MAV_CMD_COMPONENT_ARM_DISARM (400)
        // param1: 0.0 = DISARM
        publishVehicleCommand(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        
        std::cout << "[ArmHandler] DISARM 명령 전송 (" << (i + 1) << "/5)" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        try {
            rclcpp::spin_some(node_);
        } catch (const std::exception& e) {
            std::cerr << "[ArmHandler] spin_some 예외 (무시): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ArmHandler] spin_some 알 수 없는 예외 (무시)" << std::endl;
        }
    }
    
    // 시동 끄기 확인 대기
    std::cout << "[ArmHandler] 시동 끄기 확인 중..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {
        try {
            rclcpp::spin_some(node_);
        } catch (const std::exception& e) {
            std::cerr << "[ArmHandler] spin_some 예외 (무시): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ArmHandler] spin_some 알 수 없는 예외 (무시)" << std::endl;
        }
        
        if (!is_armed_) {
            std::cout << "[ArmHandler] ✓ 시동 끄기 성공!" << std::endl;
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cerr << "[ArmHandler] ✗ 시동 끄기 실패 (타임아웃)" << std::endl;
    return false;
}

bool ArmHandler::isArmed() const {
    return is_armed_;
}
