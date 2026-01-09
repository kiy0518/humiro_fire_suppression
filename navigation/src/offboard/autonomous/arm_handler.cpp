#include "arm_handler.h"
#include <iostream>
#include <thread>
#include <cstdlib>

// PX4 Vehicle Command 코드
#define VEHICLE_CMD_COMPONENT_ARM_DISARM 400
#define VEHICLE_CMD_DO_SET_MODE 176

ArmHandler::ArmHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
    , is_armed_(false)
    , nav_state_(0)
    , arming_state_(0)
{
    // DRONE_ID 환경 변수에서 target_system 읽기 (device_config.env에서 설정됨)
    target_system_ = 1;  // 기본값
    const char* drone_id_env = std::getenv("DRONE_ID");
    if (drone_id_env) {
        try {
            int drone_id = std::stoi(drone_id_env);
            if (drone_id >= 1 && drone_id <= 255) {
                target_system_ = static_cast<uint8_t>(drone_id);
                std::cout << "[ArmHandler] DRONE_ID 환경 변수 사용: target_system=" 
                          << static_cast<int>(target_system_) << std::endl;
            } else {
                std::cerr << "[ArmHandler] ⚠ 잘못된 DRONE_ID 값: " << drone_id 
                          << " (1-255 범위 필요), 기본값 1 사용" << std::endl;
            }
        } catch (...) {
            std::cerr << "[ArmHandler] ⚠ DRONE_ID 환경 변수 파싱 실패, 기본값 1 사용" << std::endl;
        }
    } else {
        std::cout << "[ArmHandler] ⚠ DRONE_ID 환경 변수 없음, 기본값 1 사용" << std::endl;
    }
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
    uint8_t new_nav_state = msg->nav_state;
    uint8_t old_nav_state = nav_state_.load(std::memory_order_acquire);
    
    arming_state_ = msg->arming_state;
    // 메모리 순서 보장을 위해 memory_order_release 사용
    nav_state_.store(new_nav_state, std::memory_order_release);
    
    // arming_state: 1 = STANDBY, 2 = ARMED
    is_armed_ = (arming_state_ == 2);
    
    // [개선 제안 3] PX4 모드 변경 감지 및 heartbeat 자동 관리
    const uint8_t PX4_NAV_STATE_OFFBOARD = 14;
    
    // OFFBOARD 모드로 전환된 경우 (nav_state == 14)
    if (new_nav_state == PX4_NAV_STATE_OFFBOARD && old_nav_state != PX4_NAV_STATE_OFFBOARD) {
        std::cout << "[ArmHandler] ✓ OFFBOARD 모드 전환 성공! (nav_state: " << (int)old_nav_state 
                  << " -> " << (int)new_nav_state << ")" << std::endl;
        // heartbeat가 중단되어 있으면 재개 (선택적)
        // 주의: 외부에서 OFFBOARD 모드로 전환한 경우에만 재개
        // VIM4가 직접 OFFBOARD 모드로 전환한 경우는 이미 heartbeat가 활성화되어 있음
        // 타이머는 사용하지 않으므로 항상 수동 heartbeat 모드
        std::cout << "[ArmHandler] OFFBOARD 모드 감지됨 (외부 전환, 수동 heartbeat 모드)" << std::endl;
    }
    // 다른 모드로 전환된 경우 (OFFBOARD → 다른 모드)
    else if (old_nav_state == PX4_NAV_STATE_OFFBOARD && new_nav_state != PX4_NAV_STATE_OFFBOARD) {
        std::cout << "[ArmHandler] OFFBOARD 모드에서 벗어남 (nav_state: " << (int)old_nav_state 
                  << " -> " << (int)new_nav_state << ")" << std::endl;
        // 타이머는 사용하지 않으므로 별도 처리 불필요
    }
    
    // 디버깅: nav_state 변경 시 항상 출력 (OFFBOARD 모드 전환 디버깅용)
    if (old_nav_state != new_nav_state) {
        std::cout << "[ArmHandler] nav_state 변경: " << (int)old_nav_state 
                  << " -> " << (int)new_nav_state << std::endl;
    }
    
    // 주기적 상태 출력 (10초마다)
    static auto last_print = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 10) {
        std::cout << "[ArmHandler] 상태: arming=" << (int)arming_state_.load() 
                  << ", nav=" << (int)nav_state_.load(std::memory_order_acquire) 
                  << ", armed=" << (is_armed_ ? "ON" : "OFF")
                  << ", heartbeat=" << (offboard_timer_ ? "ON" : "OFF") << std::endl;
        last_print = now;
    }
}

void ArmHandler::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = target_system_;
    msg.target_component = 1;
    msg.source_system = target_system_;
    msg.source_component = 1;
    msg.from_external = true;
    
    vehicle_command_pub_->publish(msg);
}

void ArmHandler::publishOffboardControlMode() {
    try {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        // PX4는 OFFBOARD 모드로 전환하려면 최소한 하나의 제어 모드가 활성화되어 있어야 함
        // position 모드를 활성화하여 OFFBOARD 모드 전환 가능하도록 함
        msg.position = true;   // Position control 활성화 (OFFBOARD 모드 전환에 필요)
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        
        offboard_control_mode_pub_->publish(msg);
        
        // 디버깅: heartbeat 전송 확인 (처음 몇 번만 출력)
        static int heartbeat_debug_count = 0;
        if (heartbeat_debug_count < 3) {
            std::cout << "[ArmHandler] ✓ OffboardControlMode heartbeat 전송 (count=" 
                      << heartbeat_debug_count << ")" << std::endl;
            heartbeat_debug_count++;
        }
    } catch (const std::exception& e) {
        static int error_count = 0;
        if (error_count < 3) {
            std::cerr << "[ArmHandler] ⚠ publishOffboardControlMode 예외: " << e.what() << std::endl;
            error_count++;
        }
    } catch (...) {
        static int error_count = 0;
        if (error_count < 3) {
            std::cerr << "[ArmHandler] ⚠ publishOffboardControlMode 알 수 없는 예외" << std::endl;
            error_count++;
        }
    }
}

bool ArmHandler::enableOffboardMode() {
    try {
        std::cout << "[ArmHandler] OFFBOARD 모드 활성화 중..." << std::endl;
        
        // 중요: executor 충돌 문제로 인해 타이머를 사용하지 않고
        // 항상 수동 heartbeat만 사용하여 안정성 보장
        // 타이머가 이미 있다면 중지
        if (offboard_timer_) {
            try {
                offboard_timer_->cancel();
                offboard_timer_.reset();
            } catch (...) {
                // 예외 무시
            }
        }
        
        std::cout << "[ArmHandler] ✓ 수동 heartbeat 모드 사용 (타이머 없이 안정적 동작)" << std::endl;
        
        // Offboard 모드 명령 전송 (여러 번 전송)
        // PX4는 OFFBOARD 모드로 전환하려면 명령과 함께 heartbeat가 필요함
        for (int i = 0; i < 10; i++) {  // 10번 전송 (1초간)
            // MAV_CMD_DO_SET_MODE (176)
            // param1: 1 (custom mode enable)
            // param2: 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            publishVehicleCommand(VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            
            // heartbeat 전송 (타이머 여부와 관계없이 항상 전송)
            publishOffboardControlMode();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // 중요: spin_some을 호출하지 않음 (executor 충돌 방지)
            // 메인 스레드의 executor가 이미 노드를 처리하고 있으므로
            // 여기서 spin_some을 호출할 필요가 없음
        }
        
        std::cout << "[ArmHandler] ✓ OFFBOARD 모드 명령 전송 완료 (10회)" << std::endl;
        
        // 모드 변경 대기 (최대 5초)
        // PX4는 OFFBOARD 모드로 전환하려면 지속적인 heartbeat가 필요함 (최소 2Hz)
        auto start_time = std::chrono::steady_clock::now();
        int heartbeat_count = 0;
        int iteration = 0;
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start_time).count() < 5000) {
            iteration++;
            
            // heartbeat 전송 (2Hz = 500ms마다, 즉 5번 반복마다)
            if (heartbeat_count % 5 == 0) {  // 100ms * 5 = 500ms
                try {
                    publishOffboardControlMode();
                } catch (const std::exception& e) {
                    std::cerr << "[ArmHandler] ⚠ heartbeat 전송 예외 (무시): " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "[ArmHandler] ⚠ heartbeat 전송 알 수 없는 예외 (무시)" << std::endl;
                }
            }
            heartbeat_count++;
            
            // 중요: spin_some을 호출하지 않음 (executor 충돌 방지)
            // 메인 스레드의 executor가 이미 노드를 처리하고 있으므로
            // 여기서 spin_some을 호출할 필요가 없음
            // vehicleStatusCallback은 메인 executor에서 자동으로 호출됨
            
            // nav_state == 14 (OFFBOARD)
            // 메모리 순서 보장을 위해 memory_order_acquire 사용
            uint8_t current_nav_state = nav_state_.load(std::memory_order_acquire);
            if (current_nav_state == 14) {
                std::cout << "[ArmHandler] ✓ OFFBOARD 모드 활성화 성공! (nav_state=14, iteration=" 
                          << iteration << ")" << std::endl;
                return true;
            }
            
            // 1초마다 진행 상황 출력 (더 자주 확인하도록 간격 단축)
            if (iteration % 5 == 0) {  // 100ms * 5 = 500ms마다 출력
                std::cout << "[ArmHandler] OFFBOARD 모드 대기 중... (nav_state=" 
                          << static_cast<int>(current_nav_state) << ", iteration=" << iteration << ")" << std::endl;
            }
            
            // 더 자주 확인하도록 간격 단축 (50ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        uint8_t final_nav_state = nav_state_.load(std::memory_order_acquire);
        std::cerr << "[ArmHandler] ⚠ OFFBOARD 모드 활성화 타임아웃 (nav_state=" 
                  << static_cast<int>(final_nav_state) << ", 총 " << iteration << "회 시도)" << std::endl;
        std::cerr << "[ArmHandler]   → PX4 파라미터 확인 필요: COM_RCL_EXCEPT=4, CBRK_FLIGHTTERM=1212121" << std::endl;
        return false;
    } catch (const std::runtime_error& e) {
        std::string error_msg = e.what();
        if (error_msg.find("already been added to an executor") != std::string::npos) {
            std::cerr << "[ArmHandler] ⚠ enableOffboardMode executor 충돌 (계속 진행): " << e.what() << std::endl;
            return true;  // executor 충돌은 치명적이지 않으므로 성공으로 처리
        } else {
            std::cerr << "[ArmHandler] ✗ enableOffboardMode 실패: " << e.what() << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ArmHandler] ✗ enableOffboardMode 예외: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[ArmHandler] ✗ enableOffboardMode 알 수 없는 예외 발생" << std::endl;
        return false;
    }
}

bool ArmHandler::disableOffboardMode() {
    std::cout << "[ArmHandler] OFFBOARD 모드 비활성화 중 (heartbeat 중단)..." << std::endl;
    
    // Offboard heartbeat 중지
    // 중요: heartbeat 중단 시 PX4가 자동으로 다른 모드(MANUAL 등)로 전환됨
    // 언제든지 호출 가능 (임무 수행 중간에도 가능)
    if (offboard_timer_) {
        offboard_timer_->cancel();
        offboard_timer_.reset();  // 타이머 리셋
        std::cout << "[ArmHandler] ✓ Offboard heartbeat 중지됨" << std::endl;
        
        // PX4는 heartbeat가 중단되면 자동으로 다른 모드로 전환됨
        // 일반적으로 0.5초(500ms) 이내에 전환됨
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 중요: spin_some을 호출하지 않음 (executor 충돌 방지)
        
    } else {
        std::cout << "[ArmHandler] ✓ Offboard heartbeat가 이미 중지되어 있음" << std::endl;
    }
    
    std::cout << "[ArmHandler] ✓ OFFBOARD 모드 비활성화 완료" << std::endl;
    std::cout << "[ArmHandler]   → PX4가 heartbeat 중단으로 자동으로 다른 모드로 전환됩니다" << std::endl;
    std::cout << "[ArmHandler]   → QGC나 다른 GCS에서 비행 모드 변경이 가능합니다" << std::endl;
    std::cout << "[ArmHandler]   → 위급 상황 시 언제든지 호출 가능 (임무 수행 중간에도 가능)" << std::endl;
    
    return true;
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
        // 중요: spin_some을 호출하지 않음 (executor 충돌 방지)
    }
    
    // 시동 확인 대기
    std::cout << "[ArmHandler] 시동 상태 확인 중..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {
        // 중요: spin_some을 호출하지 않음 (executor 충돌 방지)
        // vehicleStatusCallback은 메인 executor에서 자동으로 호출됨
        
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
        // 중요: spin_some을 호출하지 않음 (executor 충돌 방지)
    }
    
    // 시동 끄기 확인 대기
    std::cout << "[ArmHandler] 시동 끄기 확인 중..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {
        // 중요: spin_some을 호출하지 않음 (executor 충돌 방지)
        // vehicleStatusCallback은 메인 executor에서 자동으로 호출됨
        
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
