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
#include <iostream>
#include <signal.h>
#include <gst/gst.h>
#include <opencv2/core/utils/logger.hpp>

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include "thermal_ros2_publisher.h"
#include "lidar_ros2_publisher.h"
#include "status_ros2_subscriber.h"
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
      rgb_frame_queue_(RGB_FRAME_QUEUE_SIZE),
      frame_queue_(FRAME_QUEUE_SIZE),
      web_frame_queue_(WEB_FRAME_QUEUE_SIZE),
      is_running_(true),
      rgb_init_done_(false),
      thermal_init_done_(false)
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
    if (status_ros2_subscriber_) delete status_ros2_subscriber_;
    if (thermal_ros2_publisher_) delete thermal_ros2_publisher_;
    if (lidar_ros2_publisher_) delete lidar_ros2_publisher_;
    
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

