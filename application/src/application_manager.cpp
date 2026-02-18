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
#include <fstream>
#include <signal.h>
#include <gst/gst.h>
#include <opencv2/core/utils/logger.hpp>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include "thermal_ros2_publisher.h"
#include "lidar_ros2_publisher.h"
#include "status_ros2_subscriber.h"
#include "../../navigation/src/offboard/offboard_manager.h"
#include "../../navigation/src/offboard/bridge/fc_bridge_client.h"
#include "../../navigation/src/offboard/bridge/fc_bridge_protocol.h"
#include "../../navigation/src/offboard/formation/formation_controller.h"
#include "../../navigation/src/offboard/collision/collision_avoidance.h"
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
      status_ros2_subscriber_(nullptr),
      formation_controller_(nullptr),
      collision_avoidance_(nullptr)
#endif
{
}

ApplicationManager::~ApplicationManager() {
    shutdown();
}

bool ApplicationManager::initialize(int argc, char* argv[]) {
    // device_config.env 자동 로드 (환경변수 설정)
    {
        std::string config_path = "/home/khadas/humiro_fire_suppression/config/device_config.env";
        std::ifstream config_file(config_path);
        if (config_file.is_open()) {
            std::string line;
            int loaded = 0;
            while (std::getline(config_file, line)) {
                if (line.empty() || line[0] == '#') continue;
                auto eq = line.find('=');
                if (eq == std::string::npos) continue;
                std::string key = line.substr(0, eq);
                std::string val = line.substr(eq + 1);
                // ROS_/RMW_/FASTRTPS_ 환경변수는 wrapper 스크립트에서 관리
                // device_config.env가 덮어쓰지 않도록 스킵
                if (key.rfind("ROS_", 0) == 0 || key.rfind("RMW_", 0) == 0 ||
                    key.rfind("FASTRTPS_", 0) == 0) {
                    continue;
                }
                setenv(key.c_str(), val.c_str(), 1);
                loaded++;
            }
            std::cout << "[Config] device_config.env 로드: " << loaded << "개 항목" << std::endl;
        } else {
            std::cerr << "[Config] device_config.env 없음: " << config_path << std::endl;
        }
    }

    // app_config.env 자동 로드 (프로그램 파라미터)
    // 기체별 파일 우선: app_config{DRONE_ID}.env → app_config.env 폴백
    {
        const char* drone_id_str = std::getenv("DRONE_ID");
        std::string drone_id = drone_id_str ? drone_id_str : "1";
        std::string drone_config_path = "/home/khadas/humiro_fire_suppression/config/app_config" + drone_id + ".env";
        std::string default_config_path = "/home/khadas/humiro_fire_suppression/config/app_config.env";

        std::string app_config_path;
        std::ifstream drone_config_file(drone_config_path);
        if (drone_config_file.is_open()) {
            app_config_path = drone_config_path;
            drone_config_file.close();
        } else {
            app_config_path = default_config_path;
            std::cout << "[Config] app_config" << drone_id << ".env 없음 → app_config.env 폴백" << std::endl;
        }

        std::ifstream app_config_file(app_config_path);
        if (app_config_file.is_open()) {
            std::string line;
            int loaded = 0;
            while (std::getline(app_config_file, line)) {
                if (line.empty() || line[0] == '#') continue;
                auto eq = line.find('=');
                if (eq == std::string::npos) continue;
                std::string key = line.substr(0, eq);
                std::string val = line.substr(eq + 1);
                setenv(key.c_str(), val.c_str(), 1);
                loaded++;
            }
            std::cout << "[Config] " << app_config_path.substr(app_config_path.rfind('/') + 1)
                      << " 로드: " << loaded << "개 항목 (DRONE_ID=" << drone_id << ")" << std::endl;
        } else {
            std::cerr << "[Config] app_config.env 없음: " << app_config_path << std::endl;
        }
    }

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
    
    // ① ROS2 초기화
    initializeROS2(argc, argv);

    // ② OffboardManager 초기화 (ROS2 노드 필요)
    initializeOffboard();

    // ③ 편대 비행 제어기 초기화 (OffboardManager 필요)
    initializeFormation();

    // ④ MAVLink 커스텀 메시지 초기화 (OffboardManager 필요)
    initializeCustomMessage();

    std::cout << "\n[초기화]" << std::endl;

    // 카메라 프로세스 확인
    check_camera_processes();
    kill_existing_processes();

#if RESET_USB_CAMERAS_ON_START
    reset_all_usb_cameras();
#endif

    // ④ 카메라/센서 컴포넌트 초기화
    initializeComponents();

    // ⑤ 스트리밍 초기화
    initializeStreaming();
    
    return true;
}

void ApplicationManager::initializeROS2(int argc, char* argv[]) {
#ifdef ENABLE_ROS2
    std::cout << "\n[ROS2 초기화]" << std::endl;
    try {
        rclcpp::init(argc, argv);

        // DRONE_ID 기반 노드 이름 생성 (다중 드론 충돌 방지)
        int drone_id = 1;
        const char* drone_id_env = std::getenv("DRONE_ID");
        if (drone_id_env) {
            try {
                drone_id = std::stoi(drone_id_env);
            } catch (...) {}
        }
        drone_id_ = static_cast<uint8_t>(drone_id);  // 조기 설정 (Formation, CollisionAvoidance에서 사용)
        std::string node_name = "humiro_fire_suppression_" + std::to_string(drone_id);

        ros2_node_ = rclcpp::Node::make_shared(node_name);
        std::cout << "  ✓ ROS2 노드 생성: " << node_name << " (Domain 1, WiFi)" << std::endl;

        // FCBridgeClient 생성 (FC Bridge IPC, UDP loopback)
        // IPC 포트: 환경 변수 오버라이드 가능
        uint16_t state_port = FC_BRIDGE_STATE_PORT;
        uint16_t command_port = FC_BRIDGE_COMMAND_PORT;
        const char* sp = std::getenv("FC_BRIDGE_STATE_PORT");
        const char* cp = std::getenv("FC_BRIDGE_COMMAND_PORT");
        if (sp) state_port = static_cast<uint16_t>(std::atoi(sp));
        if (cp) command_port = static_cast<uint16_t>(std::atoi(cp));

        fc_bridge_ = std::make_shared<FCBridgeClient>(state_port, command_port);
        if (fc_bridge_->start()) {
            std::cout << "  ✓ FCBridgeClient 시작 (state:" << state_port
                      << ", cmd:" << command_port << ")" << std::endl;
        } else {
            std::cerr << "  ✗ FCBridgeClient 시작 실패!" << std::endl;
        }
        // SITL 모드 감지 (mavlink-router 설정 확인)
        {
            std::ifstream mav_conf("/etc/mavlink-router/main.conf");
            if (mav_conf.is_open()) {
                std::string content((std::istreambuf_iterator<char>(mav_conf)),
                                     std::istreambuf_iterator<char>());
                is_sitl_mode_ = (content.find("[UdpEndpoint SITL]") != std::string::npos) ||
                                 (content.find("[UdpEndpoint PC_SITL]") != std::string::npos);
                std::cout << "  [모드 감지] " << (is_sitl_mode_ ? "SITL" : "FC") << " 모드" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "  ✗ ROS2 초기화 실패: " << e.what() << std::endl;
        ros2_node_ = nullptr;
    }
#else
    std::cout << "\n[ROS2 비활성화]" << std::endl;
    std::cout << "  ⚠ ROS2가 비활성화되어 있습니다" << std::endl;
#endif
}

void ApplicationManager::initializeOffboard() {
#ifdef ENABLE_ROS2
    if (ros2_node_ && fc_bridge_) {
        std::cout << "\n[자율 비행 관리자 초기화]" << std::endl;
        offboard_manager_ = new OffboardManager(ros2_node_, fc_bridge_);
        offboard_manager_->setFireAmmoState(&fire_gpio_index_, FIRE_GPIO_COUNT);
        offboard_manager_->setThermalData(&thermal_data_);
        offboard_manager_->setThermalTrackingMode(true);  // 기본: 자동 모드
        if (lidar_interface_) {
            offboard_manager_->setLidarInterface(lidar_interface_);
        }
        std::cout << "  ✓ OffboardManager 초기화 완료 (FCBridgeClient IPC, 열원추적 ON, LiDAR="
                  << (lidar_interface_ ? "연결" : "없음") << ")" << std::endl;
    }
#endif
}

void ApplicationManager::initializeFormation() {
#ifdef ENABLE_ROS2
    if (ros2_node_ && offboard_manager_) {
        std::cout << "\n[편대 비행 제어기 초기화]" << std::endl;

        // device_config.env에서 ROLE 읽기
        const char* role_env = getenv("ROLE");
        std::string role = role_env ? role_env : "Leader";
        // Follower, Follower_L, Follower_R 모두 FOLLOWER 역할
        FormationRole fm_role = (role.find("Follower") != std::string::npos) ?
            FormationRole::FOLLOWER : FormationRole::LEADER;

        formation_controller_ = new FormationController(
            ros2_node_, offboard_manager_, drone_id_, fm_role);

        if (fm_role == FormationRole::FOLLOWER) {
            // 팔로워: 연속 업데이트 모드 (10Hz LeaderPose 추적 시 ROTATE 리셋 방지)
            if (offboard_manager_) {
                offboard_manager_->setContinuousUpdateMode(true);
            }

            // 오프셋 설정 (device_config.env → 환경변수)
            const char* right_env = getenv("FORMATION_OFFSET_RIGHT");
            const char* behind_env = getenv("FORMATION_OFFSET_BEHIND");
            const char* above_env = getenv("FORMATION_OFFSET_ABOVE");
            int16_t right = right_env ? atoi(right_env) : 0;
            int16_t behind = behind_env ? atoi(behind_env) : 0;
            int16_t above = above_env ? atoi(above_env) : 0;
            formation_controller_->setOffset(right, behind, above);
            std::cout << "  [설정] 편대 오프셋: right=" << right
                      << "cm, behind=" << behind << "cm, above=" << above << "cm" << std::endl;

            // 리더 네임스페이스 설정
            const char* leader_ns_env = getenv("LEADER_NAMESPACE");
            if (leader_ns_env) {
                formation_controller_->setLeaderNamespace(leader_ns_env);
                std::cout << "  [설정] 리더 네임스페이스: " << leader_ns_env << std::endl;
            }

            // 진압 파라미터 설정
            const char* sup_dist = getenv("SUPPRESS_DISTANCE");
            const char* sup_angle = getenv("SUPPRESS_ANGLE");
            int16_t sd = sup_dist ? atoi(sup_dist) : 10;
            int16_t sa = sup_angle ? atoi(sup_angle) : 0;
            formation_controller_->setSuppressParams(sd, sa);
            std::cout << "  [설정] 진압 파라미터: distance=" << sd
                      << "m, angle=" << sa << "°" << std::endl;

            // 팔로워 명령 콜백: CMD_FOLLOW 수신 시 자체 미션 시작
            formation_controller_->setCommandCallback(
                [this](uint8_t cmd, double lat, double lon) {
                    if (cmd == humiro_msgs::msg::FormationCommand::CMD_FOLLOW
                        && !mission_running_.load()) {
                        float alt = formation_controller_->getReceivedAltitude();
                        float spd = formation_controller_->getReceivedSpeed();
                        std::cout << "[FormationController] CMD_FOLLOW → 팔로워 미션 시작"
                                  << " (alt=" << alt << "m, speed=" << spd << "m/s)" << std::endl;
                        custom_message::FireMissionStart start{};
                        start.target_lat = (int32_t)(lat * 1e7);
                        start.target_lon = (int32_t)(lon * 1e7);
                        start.target_alt = alt;
                        start.flight_speed = spd;
                        start.auto_fire = 0;
                        executeMission(start);
                    }
                });
        }

        // 편대 제어 시작
        formation_controller_->start();
        std::cout << "  ✓ FormationController 초기화 완료 (role=" << role << ")" << std::endl;
    }

    // ========== 충돌 방지 비활성화 (SITL 편대 테스트) ==========
    // SITL 멀티 인스턴스는 동일 GPS 홈 위치 → 충돌 오감지 (거리 0m < DANGER 5m)
    // TODO: FC 모드에서는 다시 활성화 필요
    if (ros2_node_ && offboard_manager_) {
        std::cout << "\n[충돌 방지] 비활성화됨 (SITL 편대 오감지 방지)" << std::endl;
    }
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

    // DRONE_ID 환경 변수에서 기체 번호 읽기
    int osd_drone_id = 1;
    const char* osd_drone_id_env = std::getenv("DRONE_ID");
    if (osd_drone_id_env) {
        try {
            osd_drone_id = std::stoi(osd_drone_id_env);
        } catch (...) {
            osd_drone_id = 1;
        }
    }
    status_overlay_->setDroneName(std::to_string(osd_drone_id));
    status_overlay_->setAmmunition(6, 6);
    status_overlay_->setBattery(100);
    std::cout << "  ✓ OSD 기체 ID: " << osd_drone_id << std::endl;
    
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
            status_ros2_subscriber_ = new StatusROS2Subscriber(ros2_node_, status_overlay_, fc_bridge_.get());

            // OFFBOARD 모드 종료 시 mission_running_ 플래그 리셋 콜백 설정
            // PX4는 OFFBOARD heartbeat 수신 시 nav_state를 일시적으로 변경할 수 있으므로,
            // 시동~이륙 시퀀스(PREPARING~HOVER) 중에는 nav_state 변동을 무시한다.
            // 실제 비행 중(ROTATE~HOVER_AT_TARGET)에서만 OFFBOARD 이탈을 감지하여 abort 처리.
            status_ros2_subscriber_->setModeChangeCallback(
                [this](uint8_t old_nav_state, uint8_t new_nav_state) {
                    if (old_nav_state == 14) {  // OFFBOARD에서 다른 모드로 전환
                        // 미션 진행 중 의도적 OFFBOARD 이탈은 무시:
                        // - RTL/LANDED: RTL 명령으로 의도적 OFFBOARD 종료 (PX4가 AUTO_RTL 또는 Position으로 전환)
                        // - PREPARING~HOVER: 시동/이륙 시퀀스 중 PX4 nav_state 일시적 변동
                        if (offboard_manager_) {
                            MissionState ms = offboard_manager_->getCurrentState();
                            if (ms == MissionState::PREPARING || ms == MissionState::OFFBOARD ||
                                ms == MissionState::ARMING || ms == MissionState::TAKEOFF ||
                                ms == MissionState::HOVER ||
                                ms == MissionState::RTL || ms == MissionState::LANDED) {
                                std::cout << "  ★ [모드 변경 콜백] OFFBOARD → nav_state=" << (int)new_nav_state
                                          << " (의도적 전환, 무시: state=" << OffboardManager::getStateName(ms) << ")" << std::endl;
                                return;
                            }
                        }

                        if (new_nav_state == 5) {
                            std::cout << "  ★ [모드 변경 콜백] OFFBOARD → AUTO_RTL: 정상 RTL 전환 (무시)" << std::endl;
                        } else if (new_nav_state == 4) {
                            std::cout << "  ★ [모드 변경 콜백] OFFBOARD → HOLD: 일시정지 (미션 유지)" << std::endl;
                        } else if (new_nav_state == 18) {
                            std::cout << "  ★ [모드 변경 콜백] OFFBOARD → AUTO_LAND: 정상 착륙 전환 (무시)" << std::endl;
                        } else {
                            std::cout << "  ★ [모드 변경 콜백] OFFBOARD → nav_state=" << (int)new_nav_state << " (비정상 종료)" << std::endl;
                            finishMission(true);
                        }
                    }
                }
            );
            std::cout << "  ✓ 모드 변경 콜백 설정 완료 (OFFBOARD 종료 시 미션 플래그 리셋)" << std::endl;
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
}

void ApplicationManager::initializeCustomMessage() {
    std::cout << "\n[커스텀 메시지 초기화]" << std::endl;

    try {
        // DRONE_ID 읽기 (환경 변수) - 클래스 멤버에 저장하여 target_system 검증에 사용
        int drone_id = 1;  // 기본값
        const char* drone_id_env = std::getenv("DRONE_ID");
        if (drone_id_env) {
            try {
                drone_id = std::stoi(drone_id_env);
                std::cout << "  → DRONE_ID: " << drone_id << " (target_system 검증에 사용)" << std::endl;
            } catch (...) {
                std::cerr << "  ⚠ 잘못된 DRONE_ID 환경 변수, 기본값 1 사용" << std::endl;
            }
        }
        drone_id_ = static_cast<uint8_t>(drone_id);  // 클래스 멤버에 저장

        // 드론 ID 기반 포트 계산 (10씩 증가)
        // 드론 1: mavlink=14550, app=15001
        // 드론 2: mavlink=14560, app=15011
        // 드론 3: mavlink=14570, app=15021
        int port_offset = (drone_id - 1) * 10;
        uint16_t mavlink_port = 14550 + port_offset;
        uint16_t app_port = 15001 + port_offset;  // mavlink-router에서 라우팅 받음

        std::cout << "  → 포트 자동 계산 (DRONE_ID=" << drone_id << "):" << std::endl;
        std::cout << "    - QGC 송신 포트: " << mavlink_port << std::endl;
        std::cout << "    - App 수신 포트: " << app_port << std::endl;

        // 환경 변수로 오버라이드 가능
        const char* port_env = std::getenv("QGC_UDP_PORT");
        if (port_env) {
            try {
                mavlink_port = static_cast<uint16_t>(std::stoi(port_env));
                std::cout << "  → QGC_UDP_PORT 환경 변수로 오버라이드: " << mavlink_port << std::endl;
            } catch (...) {
                std::cerr << "  ⚠ 잘못된 QGC_UDP_PORT, 계산된 값 사용" << std::endl;
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
        // DRONE_ID 기반으로 독립 포트 사용 (라우터와 충돌 방지)
        custom_message_handler_ = new custom_message::CustomMessage(
            app_port,  // 수신 포트: DRONE_ID 기반 (드론1=15001, 드론2=15011, 드론3=15021)
            mavlink_port,  // 송신 포트: DRONE_ID 기반 (드론1=14550, 드론2=14560, 드론3=14570)
            "0.0.0.0",  // 바인드 주소: 모든 인터페이스 (외부 접근 가능)
            target_address,  // 대상 주소: QGC 또는 브로드캐스트 (변경 없음)
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
                          << start.target_alt << " m" << std::endl;
                std::cout << "[DEBUG] 미션 설정:" << std::endl;
                std::cout << "[DEBUG]   - 자동 발사 (auto_fire): " << (start.auto_fire ? "예" : "아니오") 
                          << " (" << static_cast<int>(start.auto_fire) << ")" << std::endl;
                std::cout << "[DEBUG]   - 최대 발사 횟수 (max_projectiles): "
                          << static_cast<int>(start.max_projectiles) << std::endl;
                std::cout << "[DEBUG]   - 이륙 속도 (takeoff_speed): " << start.takeoff_speed << " m/s" << std::endl;
                std::cout << "[DEBUG]   - 비행 속도 (flight_speed): " << start.flight_speed << " m/s" << std::endl;
                std::cout << "[DEBUG] ==============================================" << std::endl;

                // target_system 검증 (멀티 드론 환경)
                if (!isTargetedToMe(start.target_system)) {
                    std::cout << "[INFO] 미션 무시 (target_system=" << (int)start.target_system
                              << ", my_id=" << (int)drone_id_ << ")" << std::endl;
                    return;
                }

                // OSD 표시
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "Mission: Alt=" << start.target_alt << "m"
                        << " TkSpd=" << std::fixed << std::setprecision(1) << start.takeoff_speed
                        << " FltSpd=" << start.flight_speed << "m/s"
                        << (start.auto_fire ? " AUTO" : " MAN")
                        << " x" << (int)start.max_projectiles;
                    status_overlay_->setCustomMessage(oss.str(), 5.0);
                }

                // 좌표 체크: lat=0, lon=0이면 실내 테스트 미션
                if (start.target_lat == 0 && start.target_lon == 0) {
                    // std::cout << "[INFO] 실내 테스트 미션 모드 (lat=0, lon=0, alt=" << start.target_alt << ")" << std::endl;
                    // testExeMission3(start);
                    std::cout << "[INFO] 실내 테스트 미션은 비활성화되었습니다" << std::endl;
                } else {
                    std::cout << "[INFO] GPS 기반 미션 모드" << std::endl;
                    executeMission(start);
                }
            }
        );
        
        // FIRE_AUTO_AIM (60001) 콜백 설정 - 자동조준 명령
        custom_message_handler_->setFireAutoAimCallback(
            [this](const custom_message::FireAutoAim& aim) {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;

                std::cout << "\n[DEBUG] ========== FIRE_AUTO_AIM (60001) 수신 ==========" << std::endl;
                std::cout << "[DEBUG] 시간: " << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                          << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
                std::cout << "[DEBUG]   - target_system: " << static_cast<int>(aim.target_system) << std::endl;
                std::cout << "[DEBUG]   - target_component: " << static_cast<int>(aim.target_component) << std::endl;
                std::cout << "[DEBUG] ==============================================" << std::endl;

                // target_system 검증
                if (!isTargetedToMe(aim.target_system)) {
                    std::cout << "[INFO] 자동조준 무시 (target=" << (int)aim.target_system
                              << ", my_id=" << (int)drone_id_ << ")" << std::endl;
                    return;
                }

                if (status_overlay_) {
                    status_overlay_->setCustomMessage("Auto Aim Activated", 3.0);
                }

                // TODO: 자동조준 로직 구현
            }
        );

        // FIRE_LAUNCH (60002) 콜백 설정 - 발사 명령
        custom_message_handler_->setFireLaunchCallback(
            [this](const custom_message::FireLaunch& launch) {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;

                std::cout << "\n[DEBUG] ========== FIRE_LAUNCH (60002) 수신 ==========" << std::endl;
                std::cout << "[DEBUG] 시간: " << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                          << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
                std::cout << "[DEBUG]   - target_system: " << static_cast<int>(launch.target_system) << std::endl;
                std::cout << "[DEBUG]   - target_component: " << static_cast<int>(launch.target_component) << std::endl;
                std::cout << "[DEBUG] ==============================================" << std::endl;

                // target_system 검증
                if (!isTargetedToMe(launch.target_system)) {
                    std::cout << "[INFO] 발사 명령 무시 (target=" << (int)launch.target_system
                              << ", my_id=" << (int)drone_id_ << ")" << std::endl;
                    return;
                }

                int idx = fire_gpio_index_.load();
                if (idx >= FIRE_GPIO_COUNT) {
                    std::cout << "[FIRE_LAUNCH] 모든 GPIO 발사 완료 (" << FIRE_GPIO_COUNT << "/" << FIRE_GPIO_COUNT << ")" << std::endl;
                    if (status_overlay_) {
                        status_overlay_->setCustomMessage("All Rounds Fired!", 3.0);
                    }
                    return;
                }

                int gpio = FIRE_GPIO_PINS[idx];
                fire_gpio_index_.store(idx + 1);
                int remaining = FIRE_GPIO_COUNT - (idx + 1);

                std::cout << "[FIRE_LAUNCH] 발사 " << (idx + 1) << "/" << FIRE_GPIO_COUNT
                          << " → GPIO " << gpio << " (잔탄: " << remaining << ")" << std::endl;

                if (status_overlay_) {
                    std::string msg = "Fire " + std::to_string(idx + 1) + "/" + std::to_string(FIRE_GPIO_COUNT);
                    status_overlay_->setCustomMessage(msg, 2.0);
                    status_overlay_->setAmmunition(remaining, FIRE_GPIO_COUNT);
                }

                // GPIO 제어는 별도 스레드에서 실행 (3초 대기 포함)
                std::thread([this, gpio]() {
                    fireGpioPin(gpio);
                }).detach();
            }
        );

        // FIRE_RETURN (60003) 콜백 설정 - 복귀 명령
        // 60003은 모든 기체가 무조건 RTL (target_system 무시)
        custom_message_handler_->setFireReturnCallback(
            [this](const custom_message::FireReturn& ret) {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;

                std::cout << "\n[DEBUG] ========== FIRE_RETURN (60003) 수신 ==========" << std::endl;
                std::cout << "[DEBUG] 시간: " << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                          << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
                std::cout << "[DEBUG]   - target_system: " << static_cast<int>(ret.target_system)
                          << " (무시 - 모든 기체 RTL)" << std::endl;
                std::cout << "[DEBUG] ==============================================" << std::endl;

                if (status_overlay_) {
                    status_overlay_->setCustomMessage("RTL - All Drones Returning!", 3.0);
                }

                // 모든 기체 즉시 RTL 실행
#ifdef ENABLE_ROS2
                if (offboard_manager_) {
                    std::cout << "[FIRE_RETURN] ★ 60003 수신 → 무조건 RTL (drone_id="
                              << (int)drone_id_ << ")" << std::endl;
                    offboard_manager_->abortMission();
                    std::cout << "[FIRE_RETURN] ✓ abortMission() → RTL 전환 완료" << std::endl;
                    finishMission(false);
                } else {
                    std::cerr << "[FIRE_RETURN] Error: OffboardManager 미초기화" << std::endl;
                }
#else
                std::cerr << "[FIRE_RETURN] ROS2가 비활성화되어 RTL 불가" << std::endl;
#endif
            }
        );

        // COMMAND_LONG 콜백 설정 (ARM/DISARM 명령용)
        custom_message_handler_->setCommandLongCallback(
            [this](uint8_t target_system, uint8_t target_component, uint16_t command, 
                   float param1, float param2, float param3, float param4, 
                   float param5, float param6, float param7) {
                std::cout << "\n[DEBUG] ========== COMMAND_LONG 수신 (14550) ==========" << std::endl;
                std::cout << "[DEBUG] target_system: " << static_cast<int>(target_system) << std::endl;
                std::cout << "[DEBUG] target_component: " << static_cast<int>(target_component) << std::endl;
                std::cout << "[DEBUG] command: " << command << std::endl;
                std::cout << "[DEBUG] param1: " << param1 << std::endl;
                std::cout << "[DEBUG] ==========================================" << std::endl;

                // target_system 검증
                if (!isTargetedToMe(target_system)) {
                    std::cout << "[INFO] COMMAND_LONG 무시 (target=" << (int)target_system
                              << ", my_id=" << (int)drone_id_ << ")" << std::endl;
                    return;
                }

                // MAV_CMD_COMPONENT_ARM_DISARM (400) 처리
                if (command == 400) {
                    // param1 값 확인: 1.0 = ARM, 0.0 = DISARM
                    std::cout << "[DEBUG] param1 값: " << param1 << " (1.0=ARM, 0.0=DISARM)" << std::endl;
                    bool arm = (param1 >= 0.5f);  // 1.0 = ARM, 0.0 = DISARM
                    std::cout << "[DEBUG] ARM/DISARM 명령 판단: " << (arm ? "ARM" : "DISARM") << " (param1 >= 0.5f = " << arm << ")" << std::endl;
                    
#ifdef ENABLE_ROS2
                    if (offboard_manager_ && fc_bridge_) {
                        fc_bridge_->sendVehicleCommand(
                            400,              // MAV_CMD_COMPONENT_ARM_DISARM
                            param1,           // 1.0 = ARM, 0.0 = DISARM
                            0.0f,             // param2
                            0.0f,             // param3
                            target_system     // target_system
                        );
                        std::cout << "[DEBUG] ✓ ARM/DISARM 명령 FCBridge로 전송 완료 (param1=" << param1 << ")" << std::endl;
                        
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
                } else if (command == 176) {  // MAV_CMD_DO_SET_MODE
                    // PX4 DO_SET_MODE 명령 형식:
                    // param1 = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                    // param2 = custom_mode 값 (main_mode << 16) | (sub_mode << 8)
                    // 예: param2 = 65536 (1 << 16, MANUAL), 393216 (6 << 16, OFFBOARD) 등
                    uint32_t custom_mode = static_cast<uint32_t>(param2);
                    uint8_t main_mode = (custom_mode >> 16) & 0xFF;  // Extract main_mode
                    uint8_t sub_mode = (custom_mode >> 8) & 0xFF;    // Extract sub_mode
                    
                    std::cout << "[DEBUG] DO_SET_MODE 수신: param1=" << param1 
                              << ", param2=" << param2 << " (custom_mode=" << custom_mode 
                              << ", main=" << (int)main_mode << ", sub=" << (int)sub_mode << ")" << std::endl;
                    
                    #ifdef ENABLE_ROS2
                    // 중요: OFFBOARD 모드 활성화 중(ARMING 상태)에는 외부 DO_SET_MODE 명령 무시
                    // OFFBOARD 모드 활성화가 완료되기 전에 외부 명령으로 모드가 변경되면 활성화 실패
                    // 단, OFFBOARD 모드로 전환하려는 명령(main_mode=6)은 허용
                    bool should_ignore = false;
                    if (offboard_manager_) {
                        MissionState current_state = offboard_manager_->getCurrentState();
                        const uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
                        
                        // ARMING 상태이고 OFFBOARD가 아닌 모드로 변경하려는 경우만 무시
                        if (current_state == MissionState::ARMING && main_mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
                            std::cout << "[DEBUG] ⚠ DO_SET_MODE 명령 무시 (OFFBOARD 모드 활성화 중: ARMING 상태, 요청 모드=" 
                                      << (int)main_mode << ")" << std::endl;
                            should_ignore = true;
                        } else {
                            std::cout << "[DEBUG] ✓ DO_SET_MODE 명령 허용 (현재 상태: " 
                                      << OffboardManager::getStateName(current_state) << ", 요청 모드=" 
                                      << (int)main_mode << ")" << std::endl;
                        }
                    }
                    
                    // 명령 무시 시 ROS2 전송도 건너뛰기
                    if (should_ignore) {
                        return;  // 명령 무시하고 종료
                    }
                    
                    // [개선 제안 1] OFFBOARD 모드(6)가 아닌 다른 모드로 변경하려는 경우
                    // 현재 미션이 실행 중이면 미션 종료
                    const uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
                    if (main_mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
                        if (offboard_manager_ && offboard_manager_->getCurrentState() != MissionState::IDLE) {
                            std::cout << "[DEBUG] OFFBOARD 모드 비활성화 (외부 모드 변경 요청: main_mode="
                                      << (int)main_mode << ")" << std::endl;
                            finishMission(true);
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        }
                    }
                    #endif
                    
                #ifdef ENABLE_ROS2
                    if (fc_bridge_) {
                        // 중요: FC가 이전 명령을 처리할 시간을 주기 위해 초기 지연 추가
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                        // DO_SET_MODE 명령 전송 (3회 반복, 100ms 간격)
                        int send_count = 3;
                        for (int i = 0; i < send_count; i++) {
                            fc_bridge_->sendVehicleCommand(
                                176,              // MAV_CMD_DO_SET_MODE
                                1.0f,             // param1: Custom mode flag
                                static_cast<float>(main_mode),  // param2: Main mode value
                                0.0f,             // param3
                                target_system     // target_system
                            );

                            if (i == 0) {
                                std::cout << "[DEBUG] DO_SET_MODE 전송 (main_mode=" << (int)main_mode
                                         << ", " << send_count << "회, target=" << (int)target_system << ")" << std::endl;
                            }

                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        
                        if (status_overlay_) {
                            std::ostringstream oss;
                            oss << "Mode Change: " << (int)main_mode;
                            if (sub_mode > 0) {
                                oss << "." << (int)sub_mode;
                            }
                            status_overlay_->setCustomMessage(oss.str(), 3.0);
                        }
                    } else {
                        std::cerr << "[DEBUG] ✗ ROS2 노드가 초기화되지 않음" << std::endl;
                    }
                #else
                    std::cerr << "[DEBUG] ✗ ROS2가 비활성화되어 있음" << std::endl;
                #endif
} else {
                    std::cout << "[DEBUG] 알 수 없는 명령: " << command << std::endl;
                }
            }
        );
        
        // [개선 제안 2] SET_MODE 콜백 설정 (QGC의 SET_MODE 명령 처리용)
        custom_message_handler_->setSetModeCallback(
            [this](uint8_t target_system, uint8_t base_mode, uint32_t custom_mode) {
                std::cout << "\n[DEBUG] ========== SET_MODE 수신 (14550) ==========" << std::endl;
                std::cout << "[DEBUG] target_system: " << static_cast<int>(target_system) << std::endl;
                std::cout << "[DEBUG] base_mode: " << static_cast<int>(base_mode) << std::endl;
                std::cout << "[DEBUG] custom_mode: " << custom_mode << std::endl;
                
                uint8_t main_mode = (custom_mode >> 16) & 0xFF;  // Extract main_mode
                uint8_t sub_mode = (custom_mode >> 8) & 0xFF;     // Extract sub_mode
                std::cout << "[DEBUG] main_mode: " << static_cast<int>(main_mode) << std::endl;
                std::cout << "[DEBUG] sub_mode: " << static_cast<int>(sub_mode) << std::endl;
                std::cout << "[DEBUG] ==========================================" << std::endl;

                // target_system 검증
                if (!isTargetedToMe(target_system)) {
                    std::cout << "[INFO] SET_MODE 무시 (target=" << (int)target_system
                              << ", my_id=" << (int)drone_id_ << ")" << std::endl;
                    return;
                }

                #ifdef ENABLE_ROS2
                // 중요: OFFBOARD 모드 활성화 중(ARMING 상태)에는 외부 SET_MODE 명령 무시
                // OFFBOARD 모드 활성화가 완료되기 전에 외부 명령으로 모드가 변경되면 활성화 실패
                // 단, OFFBOARD 모드로 전환하려는 명령(main_mode=6)은 허용
                bool should_ignore = false;
                if (offboard_manager_) {
                    MissionState current_state = offboard_manager_->getCurrentState();
                    const uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
                    
                    // ARMING 상태이고 OFFBOARD가 아닌 모드로 변경하려는 경우만 무시
                    if (current_state == MissionState::ARMING && main_mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
                        std::cout << "[DEBUG] ⚠ SET_MODE 명령 무시 (OFFBOARD 모드 활성화 중: ARMING 상태, 요청 모드=" 
                                  << (int)main_mode << ")" << std::endl;
                        should_ignore = true;
                    } else {
                        std::cout << "[DEBUG] ✓ SET_MODE 명령 허용 (현재 상태: " 
                                  << OffboardManager::getStateName(current_state) << ", 요청 모드=" 
                                  << (int)main_mode << ")" << std::endl;
                    }
                }
                
                // 명령 무시 시 추가 처리 건너뛰기
                if (should_ignore) {
                    return;  // 명령 무시하고 종료
                }
                
                // [개선 제안 2] OFFBOARD 모드(6)가 아닌 다른 모드로 변경하려는 경우
                // 현재 미션이 실행 중이면 미션 종료
                const uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
                if (main_mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
                    if (offboard_manager_ && offboard_manager_->getCurrentState() != MissionState::IDLE) {
                        std::cout << "[DEBUG] OFFBOARD 모드 비활성화 (SET_MODE 수신: main_mode="
                                  << (int)main_mode << ")" << std::endl;
                        finishMission(true);
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                }
                #endif
                
                // SET_MODE는 이미 MAVLink 라우터로 전달되었으므로 추가 처리 불필요
                // FC가 자동으로 모드 변경을 처리함
                
                if (status_overlay_) {
                    std::ostringstream oss;
                    oss << "SET_MODE: " << (int)main_mode;
                    if (sub_mode > 0) {
                        oss << "." << (int)sub_mode;
                    }
                    status_overlay_->setCustomMessage(oss.str(), 3.0);
                }
            }
        );
        
        // 메시지 송수신 시작
        if (custom_message_handler_->start()) {
            std::cout << "  ✓ 커스텀 메시지 송수신 시작 (수신 포트: " << app_port << ", 송신 포트: " << mavlink_port << ")" << std::endl;
            std::cout << "  → DRONE_ID 기반 독립 포트 사용 (라우터와 충돌 방지)" << std::endl;
            std::cout << "  → 대상 주소: " << target_address << std::endl;
        } else {
            std::cout << "  ⚠ 커스텀 메시지 시작 실패" << std::endl;
        }

        // 테스트 핸들러 비활성화 (16001은 mavlink-router가 Server 모드로 점유)
        test_message_handler_ = nullptr;

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

    // WiFi 상태 폴링 스레드
    wifi_poll_thread_ = std::thread(&ApplicationManager::wifiPollLoop, this);

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
    if (wifi_poll_thread_.joinable()) wifi_poll_thread_.join();
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
    if (collision_avoidance_) {
        collision_avoidance_->stop();
        delete collision_avoidance_;
    }
    if (formation_controller_) {
        formation_controller_->stop();
        delete formation_controller_;
    }
    if (offboard_manager_) delete offboard_manager_;
    if (status_ros2_subscriber_) delete status_ros2_subscriber_;
    if (thermal_ros2_publisher_) delete thermal_ros2_publisher_;
    if (lidar_ros2_publisher_) delete lidar_ros2_publisher_;

    collision_avoidance_ = nullptr;
    formation_controller_ = nullptr;
    offboard_manager_ = nullptr;
    status_ros2_subscriber_ = nullptr;
    thermal_ros2_publisher_ = nullptr;
    lidar_ros2_publisher_ = nullptr;
    if (fc_bridge_) {
        fc_bridge_->stop();
        fc_bridge_.reset();
    }

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
        // RGB 카메라 초기화 완료 대기 (race condition 방지)
        if (!rgb_init_done_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

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
                // 첫 실패 후 10회마다 로그 + 재연결 (로그 스팸 방지)
                if (consecutive_failures == max_failures ||
                    (consecutive_failures % (max_failures * 10)) == 0) {
                    std::cout << "  ⚠ RGB 카메라 읽기 실패 " << consecutive_failures
                              << "회 - 재연결 시도..." << std::endl;
                    if (camera_manager_->reconnect_rgb_camera()) {
                        consecutive_failures = 0;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
        // 열화상 카메라 초기화 완료 대기 (race condition 방지)
        if (!thermal_init_done_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

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
                // 첫 실패 후 10회마다 로그 + 재연결 (로그 스팸 방지)
                if (consecutive_failures == max_failures ||
                    (consecutive_failures % (max_failures * 10)) == 0) {
                    std::cout << "  ⚠ 열화상 읽기 실패 " << consecutive_failures
                              << "회 - 재연결 시도..." << std::endl;
                    if (camera_manager_->reconnect_thermal_camera()) {
                        consecutive_failures = 0;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
    // 시뮬레이션 제거 — 실제 GPIO 발사(60002)로 대체됨
    while (is_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void ApplicationManager::wifiPollLoop() {
    const char* iface = "wlan0";

    while (is_running_) {
        std::string ssid;
        int rssi = 0;

        // SSID 읽기: iwgetid wlan0 -r
        {
            FILE* fp = popen("iwgetid wlan0 -r 2>/dev/null", "r");
            if (fp) {
                char buf[128];
                if (fgets(buf, sizeof(buf), fp)) {
                    ssid = buf;
                    // 개행 제거
                    while (!ssid.empty() && (ssid.back() == '\n' || ssid.back() == '\r'))
                        ssid.pop_back();
                }
                pclose(fp);
            }
        }

        // RSSI 읽기: /proc/net/wireless
        {
            std::ifstream wfile("/proc/net/wireless");
            if (wfile.is_open()) {
                std::string line;
                while (std::getline(wfile, line)) {
                    if (line.find(iface) != std::string::npos) {
                        // 형식: "wlan0: 0000  54.  -50.  -256  ..."
                        // 세 번째 숫자 필드가 signal level (dBm)
                        std::istringstream iss(line);
                        std::string iface_name;
                        int status_val;
                        double link_quality, signal_level;
                        iss >> iface_name >> status_val >> link_quality >> signal_level;
                        rssi = static_cast<int>(signal_level);
                        break;
                    }
                }
            }
        }

        if (status_overlay_ && !ssid.empty()) {
            status_overlay_->setWifiInfo(ssid, rssi);
        }

        // 5초 간격 (100ms 단위로 체크하여 종료 반응성 유지)
        for (int i = 0; i < 50 && is_running_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
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

    // ========== 미션 사전 조건 체크 (F-1~F-5) ==========
    if (status_ros2_subscriber_) {
        // F-1: FC 연결 상태 확인
        if (!status_ros2_subscriber_->isFCConnected()) {
            std::cerr << "[미션 거부] FC 연결 없음 (최근 3초 이내 VehicleStatus 미수신)" << std::endl;
            if (status_overlay_) status_overlay_->setCustomMessage("Mission Rejected: No FC", 5.0);
            return;
        }
        // F-2: GPS fix 상태 확인 (SITL 모드에서는 스킵)
        if (!status_ros2_subscriber_->isGPSFixed()) {
            if (is_sitl_mode_) {
                std::cout << "[미션] GPS 미고정이지만 SITL 모드이므로 스킵 (fix_type="
                          << static_cast<int>(status_ros2_subscriber_->getGPSFixType()) << ")" << std::endl;
            } else {
                std::cerr << "[미션 거부] GPS 미고정 (fix_type="
                          << static_cast<int>(status_ros2_subscriber_->getGPSFixType()) << ", 최소 3 필요)" << std::endl;
                if (status_overlay_) status_overlay_->setCustomMessage("Mission Rejected: No GPS Fix", 5.0);
                return;
            }
        }
        // F-3: 배터리 잔량 확인 (20% 미만 거부)
        float battery = status_ros2_subscriber_->getBatteryRemaining();
        if (battery > 0.0f && battery < 0.2f) {
            std::cerr << "[미션 거부] 배터리 부족 (" << static_cast<int>(battery * 100) << "%, 최소 20% 필요)" << std::endl;
            if (status_overlay_) status_overlay_->setCustomMessage("Mission Rejected: Low Battery", 5.0);
            return;
        }
        std::cout << "[사전 조건 통과] FC=" << (status_ros2_subscriber_->isFCConnected() ? "OK" : "X")
                  << ", GPS=" << static_cast<int>(status_ros2_subscriber_->getGPSFixType())
                  << ", Battery=" << static_cast<int>(battery * 100) << "%" << std::endl;
    }
    // F-4: OffboardManager IDLE 상태 확인 (아래 상태 판단 로직에서 처리)
    // F-5: 이전 미션 정리 완료 확인 (compare_exchange + 상태 판단에서 처리)

    // 중복 실행 방지: 이미 미션이 실행 중이면 무시
    bool expected = false;
    if (!mission_running_.compare_exchange_strong(expected, true)) {
        MissionState current_state = offboard_manager_->getCurrentState();

        std::cout << "\n[미션 중복 체크] mission_running_=true" << std::endl;
        std::cout << "  - OffboardManager 상태: " << OffboardManager::getStateName(current_state) << std::endl;

        // IDLE/ERROR/LANDED/RTL 상태면 이전 미션이 종료된 것으로 간주하고 리셋
        if (current_state == MissionState::IDLE || current_state == MissionState::ERROR ||
            current_state == MissionState::LANDED || current_state == MissionState::RTL) {
            std::cout << "  ★ 이전 미션 종료 상태 → 강제 리셋 후 재시도" << std::endl;
            finishMission(true);
            expected = false;
            if (!mission_running_.compare_exchange_strong(expected, true)) {
                std::cout << "[미션 실행 건너뜀] 리셋 후에도 여전히 미션이 실행 중입니다" << std::endl;
                return;
            }
            std::cout << "  ✓ 리셋 완료, 새 미션 시작 진행" << std::endl;
        } else {
            // 미션 진행 중 → 새 좌표로 경로 변경
            std::cout << "[미션 경로 변경] 미션 진행 중, 새 좌표로 업데이트 (상태="
                      << OffboardManager::getStateName(current_state) << ")" << std::endl;

            MissionConfig config;
            config.target_waypoint.latitude = start.target_lat / 1e7;
            config.target_waypoint.longitude = start.target_lon / 1e7;
            config.target_waypoint.altitude = start.target_alt;
            config.takeoff_altitude = start.target_alt;
            config.target_altitude = readTargetAltitudeFromConfig();
            config.flight_speed = start.flight_speed;
            config.hover_duration_sec = 5.0f;

            // 비행 모드 결정
            bool use_formation = formation_controller_ && readMissionModeFromConfig();
            if (formation_controller_) formation_controller_->setSoloMode(!use_formation);

            // 리더: FormationController 미션 타겟 + 파라미터 업데이트
            if (use_formation && formation_controller_->getRole() == FormationRole::LEADER) {
                formation_controller_->setMissionTarget(
                    config.target_waypoint.latitude, config.target_waypoint.longitude);
                formation_controller_->setMissionParams(config.takeoff_altitude, config.flight_speed);
            }

            // 미션 진행 중 → 내부에서 updateMissionTarget() 실행
            if (use_formation)
                offboard_manager_->executeMissionFormation(config);
            else
                offboard_manager_->executeMissionSolo(config);
            return;
        }
    }

    // 상태 확인 및 리셋
    MissionState current_state = offboard_manager_->getCurrentState();

    std::cout << "\n[미션 시작 전 상태 확인]" << std::endl;
    std::cout << "  - OffboardManager 상태: " << OffboardManager::getStateName(current_state) << std::endl;

    if (current_state == MissionState::IDLE) {
        std::cout << "  → IDLE 상태: 미션 시작 가능" << std::endl;
    } else if (current_state == MissionState::ERROR || current_state == MissionState::LANDED ||
               current_state == MissionState::RTL) {
        std::cout << "  → " << OffboardManager::getStateName(current_state)
                  << " 상태: 리셋 후 미션 시작" << std::endl;
        offboard_manager_->resetToIdle();
    } else {
        std::cout << "  → " << OffboardManager::getStateName(current_state)
                  << " 상태: 미션 진행 중, 시작 불가" << std::endl;
        finishMission(false);
        return;
    }

    std::cout << "\n[미션 실행 시작] Arming -> 이륙 -> 목표 위치 이동" << std::endl;

    // 미션 설정 구성 (새 MissionConfig API)
    MissionConfig config;

    // 목표 위치 설정 (lat/lon을 1e7로 나누어 degrees로 변환)
    config.target_waypoint.latitude = start.target_lat / 1e7;
    config.target_waypoint.longitude = start.target_lon / 1e7;
    config.target_waypoint.altitude = start.target_alt;

    // 이륙/비행 설정
    config.takeoff_altitude = start.target_alt;
    config.flight_speed = start.flight_speed;

    // 목표지점 고도: GUI offboard_config.json에서 읽기 (리더/팔로워 공통)
    config.target_altitude = readTargetAltitudeFromConfig();

    // 호버링 시간 (환경변수로 오버라이드 가능)
    const char* env_hover = std::getenv("MISSION_HOVER_DURATION");
    config.hover_duration_sec = env_hover ? std::atof(env_hover) : 5.0f;

    // === 입력 검증 ===
    if (config.target_waypoint.latitude < -90.0 || config.target_waypoint.latitude > 90.0) {
        std::cerr << "[미션 거부] 위도 범위 초과: " << config.target_waypoint.latitude << std::endl;
        finishMission(false);
        return;
    }
    if (config.target_waypoint.longitude < -180.0 || config.target_waypoint.longitude > 180.0) {
        std::cerr << "[미션 거부] 경도 범위 초과: " << config.target_waypoint.longitude << std::endl;
        finishMission(false);
        return;
    }
    if (config.takeoff_altitude <= 0.0f || config.takeoff_altitude > 120.0f) {
        std::cerr << "[미션 거부] 고도 범위 초과: " << config.takeoff_altitude << " m (허용: 0~120m)" << std::endl;
        finishMission(false);
        return;
    }
    if (config.flight_speed <= 0.0f || config.flight_speed > 15.0f) {
        std::cerr << "[미션 경고] 비행 속도 비정상: " << config.flight_speed << " m/s → 기본값 5.0 적용" << std::endl;
        config.flight_speed = 5.0f;
    }

    std::cout << "  - 이륙 고도: " << config.takeoff_altitude << " m" << std::endl;
    std::cout << "  - 목표지점 고도: " << config.target_altitude << " m"
              << (config.target_altitude < 0 ? " (이륙고도 사용)" : " (offboard_config.json)") << std::endl;
    std::cout << "  - 비행 속도: " << config.flight_speed << " m/s" << std::endl;
    std::cout << "  - 목표 위치: (" << config.target_waypoint.latitude << ", "
              << config.target_waypoint.longitude << "), 고도: " << config.target_waypoint.altitude << " m" << std::endl;
    std::cout << "  - 자동 격발: " << (start.auto_fire ? "ON" : "OFF") << std::endl;

    // 비행 모드 결정: formation_controller 존재 + offboard_config.json의 mission_mode
    bool use_formation = formation_controller_ && readMissionModeFromConfig();
    if (formation_controller_) formation_controller_->setSoloMode(!use_formation);
    std::cout << "  - 비행 모드: " << (use_formation ? "편대비행 (formation)" : "단독비행 (solo)") << std::endl;

    // ★★★ 리더: CMD_FOLLOW 타겟 먼저 설정 (미션 스레드 시작 전!) ★★★
    // mission_running_=true 후 1Hz 타이머가 CMD_FOLLOW 전송할 때
    // mission_target_lat_이 0.0이면 팔로워가 heading을 계산 못함 → 오프셋 회전 안됨
    if (use_formation && formation_controller_ && formation_controller_->getRole() == FormationRole::LEADER) {
        formation_controller_->setMissionTarget(
            config.target_waypoint.latitude, config.target_waypoint.longitude);
        formation_controller_->setMissionParams(config.takeoff_altitude, config.flight_speed);
        formation_controller_->setFormationPhase("NAVIGATE");
        std::cout << "[FormationController] 리더 미션 타겟 사전 설정: ("
                  << config.target_waypoint.latitude << ", " << config.target_waypoint.longitude
                  << ") alt=" << config.takeoff_altitude << "m" << std::endl;
    }

    // OffboardManager로 미션 실행 (별도 스레드에서 비동기 실행)

    std::thread mission_thread([this, config, start, use_formation]() {
        bool success = false;
        try {
            success = use_formation
                ? offboard_manager_->executeMissionFormation(config)
                : offboard_manager_->executeMissionSolo(config);
        } catch (const std::runtime_error& e) {
            // executor 관련 예외는 특별히 처리
            std::string error_msg = e.what();
            if (error_msg.find("already been added to an executor") != std::string::npos) {
                std::cerr << "[미션 실행 오류] ROS2 executor 충돌: " << e.what() << std::endl;
                std::cerr << "  → 메인 스레드의 executor와 충돌했습니다. 미션을 계속 진행합니다." << std::endl;
                // executor 충돌은 치명적이지 않으므로 계속 진행
            } else {
                std::cerr << "[미션 실행 오류] " << e.what() << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[미션 실행 오류] " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[미션 실행 오류] 알 수 없는 예외 발생" << std::endl;
        }

        if (success) {
            std::cout << "\n[미션 성공] 목표 위치 도착" << std::endl;

            // Auto fire 모드일 경우
            if (start.auto_fire) {
                std::cout << "[자동 발사 모드] 발사 시뮬레이션 시작" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                std::cout << "[발사 완료]" << std::endl;
            }

            std::cout << "[미션 완료]" << std::endl;
            if (status_overlay_) {
                status_overlay_->setCustomMessage("Mission Complete", 5.0);
            }
            finishMission(false);
        } else {
            std::cerr << "\n[미션 실패]" << std::endl;
            if (status_overlay_) {
                status_overlay_->setCustomMessage("Mission FAILED", 5.0);
            }
            finishMission(true);
        }
    });

    // 스레드 분리 (백그라운드 실행)
    mission_thread.detach();

    // (리더 미션 타겟은 미션 스레드 시작 전에 이미 설정됨 - race condition 방지)
#else
    std::cout << "[경고] ROS2가 비활성화되어 미션 실행 불가" << std::endl;
#endif
}

void ApplicationManager::finishMission(bool reset_offboard) {
#ifdef ENABLE_ROS2
    mission_running_.store(false);
    if (reset_offboard && offboard_manager_) {
        offboard_manager_->abortMission();
        offboard_manager_->resetToIdle();
    }
    std::cout << "[ApplicationManager] ✓ finishMission() - mission_running_ 리셋"
              << (reset_offboard ? " + OffboardManager 정리" : "") << std::endl;
#else
    mission_running_.store(false);
#endif
}

bool ApplicationManager::isTargetedToMe(uint8_t target_system) const {
    // 브로드캐스트(0 또는 255) 또는 자신의 ID와 일치하면 true
    return target_system == 0 || target_system == 255 || target_system == drone_id_;
}

void ApplicationManager::fireGpioPin(int gpio_num) {
    const std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(gpio_num);
    const std::string export_path = "/sys/class/gpio/export";

    // 1. GPIO export (이미 export 되어있으면 무시)
    {
        std::ofstream f(export_path);
        if (f.is_open()) {
            f << gpio_num;
            f.close();
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // sysfs 안정화 대기

    // 2. direction = out
    {
        std::ofstream f(gpio_path + "/direction");
        if (!f.is_open()) {
            std::cerr << "[GPIO] direction 열기 실패: " << gpio_path << std::endl;
            return;
        }
        f << "out";
    }

    // 3. value = 1 (ON)
    {
        std::ofstream f(gpio_path + "/value");
        if (!f.is_open()) {
            std::cerr << "[GPIO] value 열기 실패: " << gpio_path << std::endl;
            return;
        }
        f << "1";
    }
    std::cout << "[GPIO] " << gpio_num << " ON" << std::endl;

    // 4. 3초 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 5. value = 0 (OFF)
    {
        std::ofstream f(gpio_path + "/value");
        if (f.is_open()) {
            f << "0";
        }
    }
    std::cout << "[GPIO] " << gpio_num << " OFF" << std::endl;
}

float ApplicationManager::readTargetAltitudeFromConfig() const {
    const std::string config_path = "/home/khadas/humiro_fire_suppression/config/offboard_config.json";
    std::ifstream file(config_path);
    if (!file.is_open()) {
        return -1.0f;
    }

    std::string line;
    while (std::getline(file, line)) {
        // "target_altitude": 10.0 형식 파싱
        auto pos = line.find("\"target_altitude\"");
        if (pos != std::string::npos) {
            auto colon_pos = line.find(':', pos);
            if (colon_pos != std::string::npos) {
                std::string value_str = line.substr(colon_pos + 1);
                // 쉼표, 공백 제거
                value_str.erase(std::remove(value_str.begin(), value_str.end(), ','), value_str.end());
                try {
                    float val = std::stof(value_str);
                    if (val > 0.0f) {
                        std::cout << "[Config] offboard target_altitude: " << val << " m" << std::endl;
                        return val;
                    }
                } catch (...) {
                    // 파싱 실패
                }
            }
        }
    }
    return -1.0f;
}

bool ApplicationManager::readMissionModeFromConfig() const {
    const std::string config_path = "/home/khadas/humiro_fire_suppression/config/offboard_config.json";
    std::ifstream file(config_path);
    if (!file.is_open()) {
        std::cout << "[Config] offboard_config.json 없음 → 기본값: formation" << std::endl;
        return true;  // 기본값: formation
    }

    std::string line;
    while (std::getline(file, line)) {
        auto pos = line.find("\"mission_mode\"");
        if (pos != std::string::npos) {
            if (line.find("\"solo\"") != std::string::npos) {
                std::cout << "[Config] mission_mode: solo (단독비행)" << std::endl;
                return false;
            } else {
                std::cout << "[Config] mission_mode: formation (편대비행)" << std::endl;
                return true;
            }
        }
    }
    std::cout << "[Config] mission_mode 필드 없음 → 기본값: formation" << std::endl;
    return true;  // 기본값: formation
}
