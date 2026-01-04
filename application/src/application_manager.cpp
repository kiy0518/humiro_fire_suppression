#include "application_manager.h"
#include "frame_compositor.h"
#include "camera_manager.h"
#include "thermal_processor.h"
#include "thermal_overlay.h"
#include "status_overlay.h"
#include "targeting_frame_compositor.h"
#include "streaming_manager.h"
#include "lidar_interface.h"
#include "lidar_config.h"
#include "utils.h"
#include "config.h"
#include "custom_message/custom_message.h"
#include "custom_message/custom_message_type.h"
#include <iostream>
#include <signal.h>
#include <gst/gst.h>
#include <opencv2/core/utils/logger.hpp>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include "thermal_ros2_publisher.h"
#include "lidar_ros2_publisher.h"
#include "status_ros2_subscriber.h"
#include "../../navigation/src/offboard/autonomous/offboard_manager.h"
#endif

// 전역 시그널 핸들러용 포인터
static ApplicationManager* g_app_manager = nullptr;

static void signal_handler(int sig) {
    if (sig == SIGSEGV || sig == SIGBUS || sig == SIGILL) {
        std::cerr << "\n\n[Fatal Error] Signal " << sig << " received. Exiting safely." << std::endl;
        _exit(1);
    }
    
    std::cout << "\n\n[Shutdown Request]" << std::endl;
    if (g_app_manager) {
        g_app_manager->stop();
    }
}

ApplicationManager::ApplicationManager()
    : camera_manager_(nullptr),
      thermal_processor_(nullptr),
      thermal_overlay_(nullptr),
      status_overlay_(nullptr),
      targeting_compositor_(nullptr),
      streaming_manager_(nullptr),
      lidar_interface_(nullptr),
      frame_compositor_(nullptr),
      custom_message_handler_(nullptr),
      test_message_handler_(nullptr),
      rgb_frame_queue_(RGB_FRAME_QUEUE_SIZE),
      frame_queue_(FRAME_QUEUE_SIZE),
      web_frame_queue_(WEB_FRAME_QUEUE_SIZE),
      is_running_(true),
      rgb_init_done_(false),
      thermal_init_done_(false),
      mission_running_(false)
#ifdef ENABLE_ROS2
      , ros2_node_(nullptr),
      thermal_ros2_publisher_(nullptr),
      lidar_ros2_publisher_(nullptr),
      status_ros2_subscriber_(nullptr)
#endif
{
}

ApplicationManager::~ApplicationManager() {
    shutdown();
}

bool ApplicationManager::initialize(int argc, char* argv[]) {
    // OpenCV 경고 억제
    setenv("OPENCV_FFMPEG_LOGLEVEL", "-8", 0);
    setenv("OPENCV_LOG_LEVEL", "ERROR", 0);
    
#if CV_VERSION_MAJOR >= 4
    #if CV_VERSION_MINOR >= 5
        try {
            cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
        } catch (...) {
        }
    #endif
#endif
    
    std::cout << "=" << std::string(60, '=') << std::endl;
    std::cout << "  RGB + 열화상 데이터 RTSP (저지연 최적화)" << std::endl;
    std::cout << "=" << std::string(60, '=') << std::endl;
    
    // 시그널 핸들러 등록
    g_app_manager = this;
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGSEGV, signal_handler);
    signal(SIGBUS, signal_handler);
    signal(SIGILL, signal_handler);
    
    // GStreamer 초기화
    gst_init(&argc, &argv);
    
    // ROS2 초기화
    initializeROS2(argc, argv);
    
    std::cout << "\n[초기화]" << std::endl;
    
    // 카메라 프로세스 확인
    check_camera_processes();
    kill_existing_processes();
    
#if RESET_USB_CAMERAS_ON_START
    reset_all_usb_cameras();
#endif
    
    // 컴포넌트 초기화
    initializeComponents();
    
    // 스트리밍 초기화
    initializeStreaming();
    
    return true;
}

void ApplicationManager::initializeROS2(int argc, char* argv[]) {
#ifdef ENABLE_ROS2
    std::cout << "\n[ROS2 초기화]" << std::endl;
    try {
        rclcpp::init(argc, argv);
        ros2_node_ = rclcpp::Node::make_shared("humiro_fire_suppression");
        std::cout << "  ✓ ROS2 노드 생성: humiro_fire_suppression" << std::endl;
        std::cout << "  → uXRCE-DDS 토픽 구독 준비 완료" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "  ✗ ROS2 초기화 실패: " << e.what() << std::endl;
        ros2_node_ = nullptr;
    }
#else
    std::cout << "\n[ROS2 비활성화]" << std::endl;
    std::cout << "  ⚠ ROS2가 비활성화되어 있습니다" << std::endl;
#endif
}

void ApplicationManager::initializeComponents() {
    std::cout << "\n[카메라 초기화]" << std::endl;
    std::cout << "  → 비동기 초기화 시작 (OSD는 즉시 출력됩니다)" << std::endl;
    camera_manager_ = new CameraManager();
    
    std::cout << "  ✓ 출력: " << OUTPUT_WIDTH << "x" << OUTPUT_HEIGHT 
              << " @ " << OUTPUT_FPS << "fps" << std::endl;
    
    thermal_processor_ = new ThermalProcessor();
    thermal_overlay_ = new ThermalOverlay();
    
    status_overlay_ = new StatusOverlay();
    status_overlay_->setDroneName("1");
    status_overlay_->setAmmunition(6, 6);
    status_overlay_->setFormation(1, 3);
    status_overlay_->setBattery(100);
    
    targeting_compositor_ = new TargetingFrameCompositor();
    targeting_compositor_->setLidarOrientation(LIDAR_ORIENTATION_OFFSET);
    
    // FrameCompositor 생성
    frame_compositor_ = new FrameCompositor(
        status_overlay_,
        thermal_overlay_,
        targeting_compositor_,
        camera_manager_
    );
    
#ifdef ENABLE_ROS2
    if (ros2_node_) {
        thermal_ros2_publisher_ = new ThermalROS2Publisher(ros2_node_);
        lidar_ros2_publisher_ = new LidarROS2Publisher(ros2_node_);
        std::cout << "  ✓ ROS2 토픽 발행 활성화" << std::endl;
        
        if (status_overlay_) {
            std::cout << "\n[ROS2 상태 구독자 초기화]" << std::endl;
            status_ros2_subscriber_ = new StatusROS2Subscriber(ros2_node_, status_overlay_);
        
        // OffboardManager 생성
        // OffboardManager 생성
        std::cout << "\n[자율 비행 관리자 초기화]" << std::endl;
        offboard_manager_ = new OffboardManager(ros2_node_);
        std::cout << "  ✓ OffboardManager 초기화 완료" << std::endl;
            std::cout << "  ✓ ROS2 상태 구독자 활성화" << std::endl;
        }
    }
#endif
    
    // 라이다 초기화
    std::cout << "\n[라이다 초기화]" << std::endl;
    LidarConfig lidar_config = LidarConfig::createUartEConfig();
    lidar_interface_ = new LidarInterface(lidar_config);
    if (lidar_interface_->start()) {
        std::cout << "  ✓ 라이다 연결 성공: " << lidar_config.device_path << std::endl;
    } else {
        std::cout << "  ⚠ 라이다 연결 실패" << std::endl;
    }
    
    // 커스텀 메시지 초기화
    initializeCustomMessage();
}

void ApplicationManager::initializeCustomMessage() {
    std::cout << "\n[커스텀 메시지 초기화]" << std::endl;
    
    try {
        // 포트 설정 (환경 변수 또는 기본값)
        // 우선순위: QGC_UDP_PORT (device_config.env) > MAVLINK_PORT > 기본값 14550
        uint16_t mavlink_port = 14550;
        const char* port_env = std::getenv("QGC_UDP_PORT");  // device_config.env에서 로드
        if (!port_env) {
            port_env = std::getenv("MAVLINK_PORT");  // 대체 환경 변수
        }
        if (port_env) {
            try {
                mavlink_port = static_cast<uint16_t>(std::stoi(port_env));
                std::cout << "  → 포트 설정: " << mavlink_port 
                          << (std::getenv("QGC_UDP_PORT") ? " (QGC_UDP_PORT)" : " (MAVLINK_PORT)") 
                          << std::endl;
            } catch (...) {
                std::cerr << "  ⚠ 잘못된 포트 환경 변수, 기본값 14550 사용" << std::endl;
            }
        }
        
        // 대상 주소 설정 (환경 변수 또는 기본값)
        // MAVLINK_TARGET 환경 변수가 있으면 사용, 없으면 브로드캐스트 (255.255.255.255)
        std::string target_address = "255.255.255.255";  // 브로드캐스트 (모든 QGC 수신 가능)
        const char* target_env = std::getenv("MAVLINK_TARGET");
        if (target_env) {
            target_address = target_env;
            std::cout << "  → 환경 변수 MAVLINK_TARGET 사용: " << target_address << std::endl;
        }
        
        // CustomMessage 생성
        custom_message_handler_ = new custom_message::CustomMessage(
            mavlink_port,  // 수신 포트
            mavlink_port,  // 송신 포트
            "0.0.0.0",  // 바인드 주소 (모든 인터페이스)
            target_address,  // 대상 주소 (QGC 또는 브로드캐스트)
            1,  // 시스템 ID
            1   // 컴포넌트 ID
        );
        
        // FIRE_MISSION_START 콜백 설정
        custom_message_handler_->setFireMissionStartCallback(
            [this](const custom_message::FireMissionStart& start) {
                // 타임스탬프 생성
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;
                
                // 상세 디버그 출력
                std::cout << "\n[DEBUG] ========== FIRE_MISSION_START 수신 ==========" << std::endl;
                std::cout << "[DEBUG] 시간: " << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                          << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
                std::cout << "[DEBUG] 송신자 정보:" << std::endl;
                std::cout << "[DEBUG]   - target_system: " << static_cast<int>(start.target_system) << std::endl;
                std::cout << "[DEBUG]   - target_component: " << static_cast<int>(start.target_component) << std::endl;
                std::cout << "[DEBUG] 목표 위치:" << std::endl;
                std::cout << "[DEBUG]   - 위도 (lat): " << std::fixed << std::setprecision(7) 
                          << (start.target_lat / 1e7) << "° (" << start.target_lat << " * 1e7)" << std::endl;
                std::cout << "[DEBUG]   - 경도 (lon): " << std::fixed << std::setprecision(7) 
                          << (start.target_lon / 1e7) << "° (" << start.target_lon << " * 1e7)" << std::endl;
                std::cout << "[DEBUG]   - 고도 (alt): " << std::fixed << std::setprecision(2) 
                          << start.target_alt << " m (MSL)" << std::endl;
                std::cout << "[DEBUG] 미션 설정:" << std::endl;
                std::cout << "[DEBUG]   - 자동 발사 (auto_fire): " << (start.auto_fire ? "예" : "아니오") 
                          << " (" << static_cast<int>(start.auto_fire) << ")" << std::endl;
                std::cout << "[DEBUG]   - 최대 발사 횟수 (max_projectiles): "
                          << static_cast<int>(start.max_projectiles) << std::endl;
                std::cout << "[DEBUG] ==============================================" << std::endl;
                
                // OSD 표시
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "Mission Start: (" 
                        << std::fixed << std::setprecision(7) << (start.target_lat / 1e7) << ", "
                        << (start.target_lon / 1e7) << ") Alt:" << start.target_alt << "m";
                    status_overlay_->setCustomMessage(oss.str(), 5.0);

                // 미션 실행
                executeMission(start);
                }
            }
        );
        
        // FIRE_LAUNCH_CONTROL 콜백 설정
        custom_message_handler_->setFireLaunchControlCallback(
            [this](const custom_message::FireLaunchControl& control) {
                // 타임스탬프 생성
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;
                
                // 명령어 이름
                std::string cmd_name;
                std::string cmd_desc;
                switch (control.command) {
                    case 0:  // CONFIRM
                        cmd_name = "CONFIRM";
                        cmd_desc = "발사 확인";
                        break;
                    case 1:  // ABORT
                        cmd_name = "ABORT";
                        cmd_desc = "발사 중단";
                        break;
                    case 2:  // REQUEST_STATUS
                        cmd_name = "REQUEST_STATUS";
                        cmd_desc = "상태 요청";
                        break;
                    default:
                        cmd_name = "UNKNOWN";
                        cmd_desc = "알 수 없는 명령";
                        break;
                }
                
                // 상세 디버그 출력
                std::cout << "\n[DEBUG] ========== FIRE_LAUNCH_CONTROL 수신 ==========" << std::endl;
                std::cout << "[DEBUG] 시간: " << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                          << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
                std::cout << "[DEBUG] 송신자 정보:" << std::endl;
                std::cout << "[DEBUG]   - target_system: " << static_cast<int>(control.target_system) << std::endl;
                std::cout << "[DEBUG]   - target_component: " << static_cast<int>(control.target_component) << std::endl;
                std::cout << "[DEBUG] 명령 정보:" << std::endl;
                std::cout << "[DEBUG]   - command: " << cmd_name << " (" << static_cast<int>(control.command) << ") - " << cmd_desc << std::endl;
                std::cout << "[DEBUG] ==============================================" << std::endl;
                
                // OSD 표시
                if (status_overlay_) {
                    std::string msg;
                    switch (control.command) {
                        case 0:  // CONFIRM
                            msg = "Launch Confirmed";
                            break;
                        case 1:  // ABORT
                            msg = "Launch Aborted";
                            break;
                        case 2:  // REQUEST_STATUS
                            msg = "Status Requested";
                            break;
                        default:
                            msg = "Launch Control";
                            break;
                    }
                    status_overlay_->setCustomMessage(msg, 3.0);
                }
            }
        );
        
        // FIRE_MISSION_STATUS 콜백 설정 (14550 포트)
        custom_message_handler_->setFireMissionStatusCallback(
            [this](const custom_message::FireMissionStatus& status) {
                std::cout << "\n[DEBUG] ========== FIRE_MISSION_STATUS 수신 (14550) ==========" << std::endl;
                std::cout << "[DEBUG] Phase: " << static_cast<int>(status.phase) << std::endl;
                std::cout << "[DEBUG] Progress: " << static_cast<int>(status.progress) << "%" << std::endl;
                std::cout << "[DEBUG] Distance: " << status.distance_to_target << " m" << std::endl;
                std::cout << "[DEBUG] Status: " << status.status_text << std::endl;
                std::cout << "[DEBUG] ==========================================" << std::endl;
                
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "Status: Phase=" << static_cast<int>(status.phase)
                        << ", " << static_cast<int>(status.progress) << "%, "
                        << status.distance_to_target << "m";
                    status_overlay_->setCustomMessage(oss.str(), 5.0);
                }
            }
        );
        
        // FIRE_SUPPRESSION_RESULT 콜백 설정 (14550 포트)
        custom_message_handler_->setFireSuppressionResultCallback(
            [this](const custom_message::FireSuppressionResult& result) {
                std::cout << "\n[DEBUG] ========== FIRE_SUPPRESSION_RESULT 수신 (14550) ==========" << std::endl;
                std::cout << "[DEBUG] Shot Number: " << static_cast<int>(result.shot_number) << std::endl;
                std::cout << "[DEBUG] Temp Before: " << (result.temp_before / 10.0) << "C" << std::endl;
                std::cout << "[DEBUG] Temp After: " << (result.temp_after / 10.0) << "C" << std::endl;
                std::cout << "[DEBUG] Success: " << (result.success ? "Yes" : "No") << std::endl;
                std::cout << "[DEBUG] ==========================================" << std::endl;
                
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "Result: Shot=" << static_cast<int>(result.shot_number)
                        << ", " << (result.temp_before / 10.0) << "C->" << (result.temp_after / 10.0) << "C"
                        << ", " << (result.success ? "Success" : "Failed");
                    status_overlay_->setCustomMessage(oss.str(), 5.0);
                }
            }
        );
        
        // COMMAND_LONG 콜백 설정 (ARM/DISARM 명령용)
        custom_message_handler_->setCommandLongCallback(
            [this](uint8_t target_system, uint8_t target_component, uint16_t command, float param1) {
                std::cout << "\n[DEBUG] ========== COMMAND_LONG 수신 (14550) ==========" << std::endl;
                std::cout << "[DEBUG] target_system: " << static_cast<int>(target_system) << std::endl;
                std::cout << "[DEBUG] target_component: " << static_cast<int>(target_component) << std::endl;
                std::cout << "[DEBUG] command: " << command << std::endl;
                std::cout << "[DEBUG] param1: " << param1 << std::endl;
                std::cout << "[DEBUG] ==========================================" << std::endl;
                
                // MAV_CMD_COMPONENT_ARM_DISARM (400) 처리
                if (command == 400) {
                    // param1 값 확인: 1.0 = ARM, 0.0 = DISARM
                    std::cout << "[DEBUG] param1 값: " << param1 << " (1.0=ARM, 0.0=DISARM)" << std::endl;
                    bool arm = (param1 >= 0.5f);  // 1.0 = ARM, 0.0 = DISARM
                    std::cout << "[DEBUG] ARM/DISARM 명령 판단: " << (arm ? "ARM" : "DISARM") << " (param1 >= 0.5f = " << arm << ")" << std::endl;
                    
#ifdef ENABLE_ROS2
                    if (offboard_manager_ && ros2_node_) {
                        // OffboardManager를 통해 ARM/DISARM 처리
                        // OffboardManager는 arm_handler_를 가지고 있지만, 직접 접근이 어려움
                        // 대신 ROS2 토픽으로 직접 전송
                        auto vehicle_command_pub = ros2_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
                            "/fmu/in/vehicle_command", 10);
                        
                        px4_msgs::msg::VehicleCommand cmd;
                        cmd.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                        cmd.param1 = param1;  // 1.0 = ARM, 0.0 = DISARM
                        cmd.param2 = 0.0f;
                        cmd.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
                        cmd.target_system = target_system;
                        cmd.target_component = target_component;
                        cmd.source_system = 1;
                        cmd.source_component = 1;
                        cmd.from_external = true;
                        
                        vehicle_command_pub->publish(cmd);
                        std::cout << "[DEBUG] ✓ ARM/DISARM 명령 ROS2로 전송 완료 (param1=" << param1 << ")" << std::endl;
                        
                        if (status_overlay_) {
                            std::string msg = arm ? "ARM Command" : "DISARM Command";
                            std::cout << "[DEBUG] OSD 메시지 설정: " << msg << " (arm=" << arm << ", param1=" << param1 << ")" << std::endl;
                            status_overlay_->setCustomMessage(msg, 3.0);
                        }
                    } else {
                        std::cerr << "[DEBUG] ✗ OffboardManager 또는 ROS2 노드가 초기화되지 않음" << std::endl;
                    }
#else
                    std::cerr << "[DEBUG] ✗ ROS2가 비활성화되어 있음" << std::endl;
#endif
                } else {
                    std::cout << "[DEBUG] 알 수 없는 명령: " << command << std::endl;
                }
            }
        );
        
        // 메시지 송수신 시작
        if (custom_message_handler_->start()) {
            std::cout << "  ✓ 커스텀 메시지 송수신 시작 (포트 " << mavlink_port << ")" << std::endl;
            std::cout << "  → 대상 주소: " << target_address << std::endl;
        } else {
            std::cout << "  ⚠ 커스텀 메시지 시작 실패" << std::endl;
        }

        // ===== 테스트용 메시지 핸들러 추가 (15000 포트) =====
        std::cout << "\n[테스트 메시지 핸들러 초기화]" << std::endl;
        
        test_message_handler_ = new custom_message::CustomMessage(
            15000,  // 수신 포트 (테스트용)
            15000,  // 송신 포트 (테스트용)
            "0.0.0.0",  // 바인드 주소
            target_address,  // 대상 주소
            1,  // 시스템 ID
            1   // 컴포넌트 ID
        );
        
        // FIRE_MISSION_START 콜백 설정 (동일 로직)
        test_message_handler_->setFireMissionStartCallback(
            [this](const custom_message::FireMissionStart& start) {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;
                
                std::cout << "\n[TEST PORT] ========== FIRE_MISSION_START 수신 (15000) ==========" << std::endl;
                std::cout << "[TEST PORT] 시간: " << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                          << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
                std::cout << "[TEST PORT] 목표 위치:" << std::endl;
                std::cout << "[TEST PORT]   - 위도: " << std::fixed << std::setprecision(7) 
                          << (start.target_lat / 1e7) << "°" << std::endl;
                std::cout << "[TEST PORT]   - 경도: " << std::fixed << std::setprecision(7) 
                          << (start.target_lon / 1e7) << "°" << std::endl;
                std::cout << "[TEST PORT]   - 고도: " << std::fixed << std::setprecision(2) 
                          << start.target_alt << " m" << std::endl;
                std::cout << "[TEST PORT] Auto Fire: " << (start.auto_fire ? "예" : "아니오") << std::endl;
                std::cout << "[TEST PORT] ==========================================" << std::endl;
                
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "[TEST] Mission: (" 
                        << std::fixed << std::setprecision(7) << (start.target_lat / 1e7) << ", "
                        << (start.target_lon / 1e7) << ") " << start.target_alt << "m";
                    status_overlay_->setCustomMessage(oss.str(), 5.0);
                }
                
                // 미션 실행 (주석 처리 - 테스트용은 OSD만 표시)
                // executeMission(start);
            }
        );
        
        // FIRE_LAUNCH_CONTROL 콜백 설정
        test_message_handler_->setFireLaunchControlCallback(
            [this](const custom_message::FireLaunchControl& control) {
                std::string cmd_name;
                switch (control.command) {
                    case 0: cmd_name = "CONFIRM"; break;
                    case 1: cmd_name = "ABORT"; break;
                    case 2: cmd_name = "REQUEST_STATUS"; break;
                    default: cmd_name = "UNKNOWN"; break;
                }
                
                std::cout << "\n[TEST PORT] FIRE_LAUNCH_CONTROL 수신 (15000): " << cmd_name << std::endl;
                
                if (status_overlay_) {
                    status_overlay_->setCustomMessage("[TEST] " + cmd_name, 3.0);
                }
            }
        );
        
        // FIRE_MISSION_STATUS 콜백 설정
        test_message_handler_->setFireMissionStatusCallback(
            [this](const custom_message::FireMissionStatus& status) {
                std::cout << "\n[TEST PORT] ========== FIRE_MISSION_STATUS 수신 (15000) ==========" << std::endl;
                std::cout << "[TEST PORT] Phase: " << static_cast<int>(status.phase) << std::endl;
                std::cout << "[TEST PORT] Progress: " << static_cast<int>(status.progress) << "%" << std::endl;
                std::cout << "[TEST PORT] Distance: " << status.distance_to_target << " m" << std::endl;
                std::cout << "[TEST PORT] Status: " << status.status_text << std::endl;
                std::cout << "[TEST PORT] ==========================================" << std::endl;
                
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "[TEST] Status: Phase=" << static_cast<int>(status.phase)
                        << ", " << static_cast<int>(status.progress) << "%, "
                        << status.distance_to_target << "m";
                    status_overlay_->setCustomMessage(oss.str(), 5.0);
                }
            }
        );
        
        // FIRE_SUPPRESSION_RESULT 콜백 설정
        test_message_handler_->setFireSuppressionResultCallback(
            [this](const custom_message::FireSuppressionResult& result) {
                std::cout << "\n[TEST PORT] ========== FIRE_SUPPRESSION_RESULT 수신 (15000) ==========" << std::endl;
                std::cout << "[TEST PORT] Shot Number: " << static_cast<int>(result.shot_number) << std::endl;
                std::cout << "[TEST PORT] Temp Before: " << (result.temp_before / 10.0) << "°C" << std::endl;
                std::cout << "[TEST PORT] Temp After: " << (result.temp_after / 10.0) << "°C" << std::endl;
                std::cout << "[TEST PORT] Success: " << (result.success ? "Yes" : "No") << std::endl;
                std::cout << "[TEST PORT] ==========================================" << std::endl;
                
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "[TEST] Result: Shot=" << static_cast<int>(result.shot_number)
                        << ", " << (result.temp_before / 10.0) << "C->" << (result.temp_after / 10.0) << "C"
                        << ", " << (result.success ? "Success" : "Failed");
                    status_overlay_->setCustomMessage(oss.str(), 5.0);
                }
            }
        );
        
        // 테스트 메시지 송수신 시작
        if (test_message_handler_->start()) {
            std::cout << "  ✓ 테스트 메시지 송수신 시작 (포트 15000)" << std::endl;
        } else {
            std::cout << "  ⚠ 테스트 메시지 시작 실패" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "  ✗ 커스텀 메시지 초기화 실패: " << e.what() << std::endl;
        custom_message_handler_ = nullptr;
    }
}

void ApplicationManager::startThreads() {
    std::cout << "\n[스레드 시작]" << std::endl;
    
    // 카메라 초기화 스레드
    camera_init_thread_ = std::thread(&ApplicationManager::cameraInitLoop, this);
    
    // 캡처 스레드
    rgb_capture_thread_ = std::thread(&ApplicationManager::rgbCaptureLoop, this);
    thermal_capture_thread_ = std::thread(&ApplicationManager::thermalCaptureLoop, this);
    lidar_thread_ = std::thread(&ApplicationManager::lidarLoop, this);
    
    // 합성 스레드
    composite_thread_ = std::thread(&ApplicationManager::compositeLoop, this);
    
    // 시뮬레이션 스레드
    ammo_sim_thread_ = std::thread(&ApplicationManager::ammunitionSimulationLoop, this);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void ApplicationManager::run() {
    startThreads();
    
    std::cout << "\n대기 중... (Ctrl+C: 종료)\n" << std::endl;
    
    while (is_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
#ifdef ENABLE_ROS2
        if (status_ros2_subscriber_ && ros2_node_) {
            status_ros2_subscriber_->spin();
        }
        
        if (ros2_node_ && !rclcpp::ok()) {
            std::cerr << "  ⚠ ROS2 노드가 종료되었습니다" << std::endl;
            break;
        }
#endif
    }
}

void ApplicationManager::stop() {
    is_running_ = false;
    if (streaming_manager_) {
        streaming_manager_->stop();
    }
}

void ApplicationManager::shutdown() {
    if (!is_running_) {
        return;  // 이미 종료됨
    }
    
    stop();
    
    std::cout << "\n[종료 중...]" << std::endl;
    stopThreads();
    cleanupComponents();
    cleanupROS2();
}

void ApplicationManager::stopThreads() {
    std::cout << "  → 스레드 종료 대기 중..." << std::endl;
    if (camera_init_thread_.joinable()) camera_init_thread_.join();
    if (rgb_capture_thread_.joinable()) rgb_capture_thread_.join();
    if (thermal_capture_thread_.joinable()) thermal_capture_thread_.join();
    if (lidar_thread_.joinable()) lidar_thread_.join();
    if (composite_thread_.joinable()) composite_thread_.join();
    if (ammo_sim_thread_.joinable()) ammo_sim_thread_.join();
    std::cout << "  ✓ 모든 스레드 종료 완료" << std::endl;
}

void ApplicationManager::cleanupComponents() {
    std::cout << "  → 리소스 정리 중..." << std::endl;
    
    if (streaming_manager_) {
        streaming_manager_->stop();
        delete streaming_manager_;
        streaming_manager_ = nullptr;
    }
    
    if (camera_manager_) {
        delete camera_manager_;
        camera_manager_ = nullptr;
    }
    
    if (lidar_interface_) {
        lidar_interface_->stop();
        delete lidar_interface_;
        lidar_interface_ = nullptr;
    }
    
    if (custom_message_handler_) {
        custom_message_handler_->stop();
        delete custom_message_handler_;
        custom_message_handler_ = nullptr;
    }
    if (test_message_handler_) {
        test_message_handler_->stop();
        delete test_message_handler_;
        test_message_handler_ = nullptr;
    }
    
    if (thermal_processor_) delete thermal_processor_;
    if (thermal_overlay_) delete thermal_overlay_;
    if (status_overlay_) delete status_overlay_;
    if (targeting_compositor_) delete targeting_compositor_;
    if (frame_compositor_) delete frame_compositor_;
    
    thermal_processor_ = nullptr;
    thermal_overlay_ = nullptr;
    status_overlay_ = nullptr;
    targeting_compositor_ = nullptr;
    frame_compositor_ = nullptr;
}

void ApplicationManager::cleanupROS2() {
#ifdef ENABLE_ROS2
    if (offboard_manager_) delete offboard_manager_;
    if (status_ros2_subscriber_) delete status_ros2_subscriber_;
    if (thermal_ros2_publisher_) delete thermal_ros2_publisher_;
    if (lidar_ros2_publisher_) delete lidar_ros2_publisher_;
    
    offboard_manager_ = nullptr;
    status_ros2_subscriber_ = nullptr;
    thermal_ros2_publisher_ = nullptr;
    lidar_ros2_publisher_ = nullptr;
    
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
#endif
}

void ApplicationManager::rgbCaptureLoop() {
    const double frame_interval = 1.0 / RGB_TARGET_FPS;
    auto last_frame_time = std::chrono::steady_clock::now();
    int consecutive_failures = 0;
    const int max_failures = 5;
    
    while (is_running_) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_frame_time).count() / 1000.0;
        
        if (elapsed < frame_interval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>((frame_interval - elapsed) * 1000)));
            continue;
        }
        
        last_frame_time = std::chrono::steady_clock::now();
        
        if (!camera_manager_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        cv::Mat frame;
        if (camera_manager_->read_rgb_frame(frame)) {
            consecutive_failures = 0;
            if (frame.rows >= 200 && frame.cols >= 200) {
                if (frame.rows != OUTPUT_HEIGHT || frame.cols != OUTPUT_WIDTH) {
                    cv::resize(frame, frame, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
                }
                rgb_frame_queue_.push(frame);
            }
        } else {
            consecutive_failures++;
            if (consecutive_failures >= max_failures) {
                std::cout << "  ⚠ RGB 카메라 읽기 실패 " << consecutive_failures 
                          << "회 - 재연결 시도..." << std::endl;
                if (camera_manager_->reconnect_rgb_camera()) {
                    consecutive_failures = 0;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
}

void ApplicationManager::thermalCaptureLoop() {
    int consecutive_failures = 0;
    const int max_failures = 5;
    
    while (is_running_) {
        if (!camera_manager_ || !thermal_processor_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        cv::Mat frame;
        if (camera_manager_->read_thermal_frame(frame)) {
            consecutive_failures = 0;
            thermal_processor_->extract_thermal_data(frame, thermal_data_);
            
#ifdef ENABLE_ROS2
            if (thermal_ros2_publisher_ && ros2_node_) {
                thermal_ros2_publisher_->publishThermalData(thermal_data_);
                if (!frame.empty()) {
                    thermal_ros2_publisher_->publishThermalImage(frame);
                }
            }
#endif
        } else {
            consecutive_failures++;
            if (consecutive_failures >= max_failures) {
                std::cout << "  ⚠ 열화상 읽기 실패 " << consecutive_failures 
                          << "회 - 재연결 시도..." << std::endl;
                if (camera_manager_->reconnect_thermal_camera()) {
                    consecutive_failures = 0;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
}

void ApplicationManager::lidarLoop() {
    while (is_running_) {
        if (!lidar_interface_ || !targeting_compositor_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (lidar_interface_->isConnected()) {
            LidarScan scan;
            if (lidar_interface_->getLatestScan(scan)) {
                targeting_compositor_->setLidarData(scan.points);
                
#ifdef ENABLE_ROS2
                if (lidar_ros2_publisher_ && ros2_node_) {
                    lidar_ros2_publisher_->publishLidarPoints(scan.points);
                    float front_dist = lidar_interface_->getFrontDistance();
                    if (front_dist > 0.0f) {
                        lidar_ros2_publisher_->publishFrontDistance(front_dist);
                    }
                }
#endif
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ApplicationManager::compositeLoop() {
    int frame_count = 0;
    auto last_time = std::chrono::steady_clock::now();
    const double frame_interval = 1.0 / RGB_TARGET_FPS;
    auto last_frame_time = std::chrono::steady_clock::now();
    
    while (is_running_) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_frame_time).count() / 1000.0;
        
        if (elapsed < frame_interval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>((frame_interval - elapsed) * 1000)));
            continue;
        }
        
        last_frame_time = std::chrono::steady_clock::now();
        
        if (!frame_compositor_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // RGB 프레임 가져오기
        cv::Mat rgb_frame;
        bool has_rgb = rgb_frame_queue_.try_pop(rgb_frame, 0);
        
        // 큐에 더 있으면 모두 버리고 최신 것만 사용
        while (!rgb_frame_queue_.empty()) {
            cv::Mat dummy;
            rgb_frame_queue_.try_pop(dummy, 0);
        }
        
        // 열화상 데이터 가져오기
        ThermalData data = thermal_data_.copy();
        bool has_thermal = data.valid && !data.frame.empty();
        
        // 프레임 합성
        cv::Mat output = frame_compositor_->compose(rgb_frame, data, has_rgb, has_thermal);
        
        if (output.empty()) {
            continue;
        }
        
        // BGR → RGB 변환
        cv::Mat composite_rgb;
        cv::cvtColor(output, composite_rgb, cv::COLOR_BGR2RGB);
        
        // 큐에 넣기
        frame_queue_.push(composite_rgb);
        
        if (ENABLE_HTTP_SERVER) {
            web_frame_queue_.push(composite_rgb);
        }
        
        // FPS 계산
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_time).count();
        
        if (elapsed_sec >= 5) {
            double fps = frame_count / static_cast<double>(elapsed_sec);
            std::cout << "  → 합성 FPS: " << fps << " (목표: " << RGB_TARGET_FPS << " FPS)" << std::endl;
            frame_count = 0;
            last_time = now;
        }
    }
}

void ApplicationManager::cameraInitLoop() {
    std::cout << "  [비동기] RGB 카메라 초기화 시작..." << std::endl;
    bool rgb_ok = camera_manager_->initialize_rgb_camera();
    rgb_init_done_ = true;
    if (rgb_ok) {
        std::cout << "  ✓ RGB 카메라 준비 완료" << std::endl;
    } else {
        std::cout << "  ⚠ RGB 카메라 초기화 실패 (나중에 재연결 시도)" << std::endl;
    }
    
    std::cout << "  [비동기] 열화상 카메라 초기화 시작..." << std::endl;
    bool thermal_ok = camera_manager_->initialize_thermal_camera();
    thermal_init_done_ = true;
    if (thermal_ok) {
        std::cout << "  ✓ 열화상 카메라 준비 완료" << std::endl;
    } else {
        std::cout << "  ⚠ 열화상 카메라 초기화 실패 (나중에 재연결 시도)" << std::endl;
    }
}

void ApplicationManager::ammunitionSimulationLoop() {
    int current_ammo = 6;
    const int max_ammo = 6;
    auto last_update = std::chrono::steady_clock::now();
    
    while (is_running_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_update).count();
        
        if (elapsed >= 1) {
            if (current_ammo > 0) {
                current_ammo--;
                if (status_overlay_) {
                    status_overlay_->setAmmunition(current_ammo, max_ammo);
                }
            } else {
                current_ammo = max_ammo;
                if (status_overlay_) {
                    status_overlay_->setAmmunition(current_ammo, max_ammo);
                }
            }
            last_update = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ApplicationManager::initializeStreaming() {
    std::cout << "\n[인코더]" << std::endl;
    if (check_encoder("openh264enc")) {
        std::cout << "  ✓ openh264enc" << std::endl;
    } else if (check_encoder("x264enc")) {
        std::cout << "  ✓ x264enc" << std::endl;
    }
    
    std::cout << "\n[스트리밍 서버 시작]" << std::endl;
    streaming_manager_ = new StreamingManager();
    if (!streaming_manager_->initialize(&frame_queue_, &web_frame_queue_)) {
        std::cerr << "  ✗ 스트리밍 서버 초기화 실패" << std::endl;
        is_running_ = false;
    } else {
        streaming_manager_->start();
        
        std::string rtsp_url = streaming_manager_->getRTSPUrl();
        std::string http_url = streaming_manager_->getHTTPUrl();
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "  ✓ RTSP 서버 시작됨" << std::endl;
        if (ENABLE_HTTP_SERVER && !http_url.empty()) {
            std::cout << "  ✓ HTTP 웹 서버 시작됨" << std::endl;
        }
        std::cout << "  → 준비된 데이터: OSD 상태 정보" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "\n  ★ RTSP URL: " << rtsp_url << std::endl;
        if (!http_url.empty()) {
            std::cout << "  ★ HTTP URL: " << http_url << std::endl;
        }
        std::cout << "  ★ FPS: " << OUTPUT_FPS << std::endl;
        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "  QGC: RTSP URL → " << rtsp_url << std::endl;
        std::cout << "  VLC: vlc " << rtsp_url << std::endl;
        if (!http_url.empty()) {
            std::cout << "  웹 브라우저: " << http_url << std::endl;
        }
        std::cout << std::string(60, '=') << std::endl;
    }
}


void ApplicationManager::executeMission(const custom_message::FireMissionStart& start) {
#ifdef ENABLE_ROS2
    if (!offboard_manager_) {
        std::cerr << "[Error] OffboardManager가 초기화되지 않았습니다" << std::endl;
        return;
    }

    // 중복 실행 방지: 이미 미션이 실행 중이면 무시
    bool expected = false;
    if (!mission_running_.compare_exchange_strong(expected, true)) {
        std::cout << "\n[미션 실행 건너뜀] 이미 미션이 실행 중입니다 (mission_running_=true)" << std::endl;
        return;
    }
    
    // OffboardManager 상태 확인: 이미 미션이 실행 중이면 무시
    MissionState current_state = offboard_manager_->getCurrentState();
    if (current_state != MissionState::IDLE) {
        std::cout << "\n[미션 실행 건너뜀] OffboardManager가 이미 실행 중입니다 (state: " 
                  << OffboardManager::getStateName(current_state) << ")" << std::endl;
        mission_running_.store(false);  // 플래그 리셋
        return;
    }

    std::cout << "\n[미션 실행 시작] Arming -> 이륙 -> 목표 위치 이동" << std::endl;

    // 미션 설정 구성
    MissionConfig config;
    
    // 목표 위치 설정 (lat/lon을 1e7로 나누어 degrees로 변환)
    config.target_waypoint.latitude = start.target_lat / 1e7;
    config.target_waypoint.longitude = start.target_lon / 1e7;
    config.target_waypoint.altitude = start.target_alt;
    
    // 이륙 고도 설정
    config.takeoff_altitude = start.target_alt;
    
    // 목표 거리 설정 (10m)
    config.target_distance = 10.0f;
    config.distance_tolerance = 1.0f;
    config.hover_duration_sec = 5.0f;

    std::cout << "  - 이륙 고도: " << config.takeoff_altitude << " m" << std::endl;
    std::cout << "  - 목표 위치: (" << config.target_waypoint.latitude << ", "
              << config.target_waypoint.longitude << ", " << config.target_waypoint.altitude << ")" << std::endl;
    std::cout << "  - 목표 거리: " << config.target_distance << " m" << std::endl;

    // OffboardManager로 미션 실행 (별도 스레드에서 비동기 실행)
    std::thread mission_thread([this, config, start]() {
        bool success = false;
        try {
            success = offboard_manager_->executeMission(config);
        } catch (const std::exception& e) {
            std::cerr << "[미션 실행 오류] " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[미션 실행 오류] 알 수 없는 예외 발생" << std::endl;
        }

        // 미션 완료/실패 시 플래그 리셋
        mission_running_.store(false);

        if (success) {
            std::cout << "\n[미션 성공] 목표 위치 도착" << std::endl;
            
            // 미션 상태 전송
            if (custom_message_handler_) {
                custom_message::FireMissionStatus status;
                status.phase = static_cast<uint8_t>(custom_message::FireMissionPhase::FIRE_PHASE_READY_TO_FIRE);
                status.progress = 90;
                status.remaining_projectiles = 10;
                status.distance_to_target = 0.0f;
                status.thermal_max_temp = 0;
                std::strncpy(status.status_text, "Arrived at target", sizeof(status.status_text) - 1);
                custom_message_handler_->sendFireMissionStatus(status);
            }
            
            // Auto fire 모드일 경우
            if (start.auto_fire) {
                std::cout << "[자동 발사 모드] 발사 시뮬레이션 시작" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                
                // 발사 결과 전송
                if (custom_message_handler_) {
                    custom_message::FireSuppressionResult result;
                    result.shot_number = 1;
                    result.temp_before = 800;  // 80.0°C
                    result.temp_after = 250;   // 25.0°C
                    result.success = 1;
                    custom_message_handler_->sendFireSuppressionResult(result);
                    std::cout << "[발사 완료] 결과 전송 완료" << std::endl;
                }
            }
            
            // 완료 상태 전송
            if (custom_message_handler_) {
                custom_message::FireMissionStatus status;
                status.phase = static_cast<uint8_t>(custom_message::FireMissionPhase::FIRE_PHASE_COMPLETE);
                status.progress = 100;
                status.remaining_projectiles = 9;
                status.distance_to_target = 0.0f;
                status.thermal_max_temp = 0;
                std::strncpy(status.status_text, "Mission complete", sizeof(status.status_text) - 1);
                custom_message_handler_->sendFireMissionStatus(status);
            }
            
        } else {
            std::cerr << "\n[미션 실패]" << std::endl;
            
            // 실패 상태 전송
            if (custom_message_handler_) {
                custom_message::FireMissionStatus status;
                status.phase = static_cast<uint8_t>(custom_message::FireMissionPhase::FIRE_PHASE_IDLE);
                status.progress = 0;
                status.remaining_projectiles = 10;
                status.distance_to_target = 0.0f;
                status.thermal_max_temp = 0;
                std::strncpy(status.status_text, "Mission failed", sizeof(status.status_text) - 1);
                custom_message_handler_->sendFireMissionStatus(status);
            }
        }
    });
    
    // 스레드 분리 (백그라운드 실행)
    mission_thread.detach();
#else
    std::cout << "[경고] ROS2가 비활성화되어 미션 실행 불가" << std::endl;
#endif
}
