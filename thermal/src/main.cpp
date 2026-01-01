#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <gst/gst.h>

#include "config.h"
#include "camera_manager.h"
#include "thermal_processor.h"
#include "thermal_basic_overlay.h"
#include "thread_safe_queue.h"
#include "thermal_data.h"
#include "utils.h"

// 라이다 인터페이스 (경로는 빌드 시스템에서 설정)
#include "../../lidar/src/lidar_interface.h"
#include "../../lidar/src/lidar_config.h"

// 타겟팅 프레임 합성 (경로는 빌드 시스템에서 설정)
#include "../../targeting/src/targeting_frame_compositor.h"

// 스트리밍 관리자 (경로는 빌드 시스템에서 설정)
#include "../../streaming/src/streaming_manager.h"

// ROS2 통합 (선택적)
#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include "thermal_ros2_publisher.h"
#include "../../lidar/src/lidar_ros2_publisher.h"
#endif

// 전역 변수
std::atomic<bool> is_running(true);
CameraManager* camera_manager = nullptr;
ThermalProcessor* thermal_processor = nullptr;
ThermalBasicOverlay* thermal_overlay = nullptr;
TargetingFrameCompositor* targeting_compositor = nullptr;
StreamingManager* streaming_manager = nullptr;
LidarInterface* lidar_interface = nullptr;

#ifdef ENABLE_ROS2
rclcpp::Node::SharedPtr ros2_node = nullptr;
ThermalROS2Publisher* thermal_ros2_publisher = nullptr;
LidarROS2Publisher* lidar_ros2_publisher = nullptr;
#endif

ThreadSafeQueue<cv::Mat> rgb_frame_queue(RGB_FRAME_QUEUE_SIZE);
ThreadSafeQueue<cv::Mat> frame_queue(FRAME_QUEUE_SIZE);
ThreadSafeQueue<cv::Mat> web_frame_queue(WEB_FRAME_QUEUE_SIZE);

ThermalData thermal_data;

// 시그널 핸들러
void signal_handler(int sig) {
    std::cout << "\n\n[종료 요청]" << std::endl;
    is_running = false;
    
    // 서버 중지 (비동기적으로 안전하게)
    if (streaming_manager) {
        streaming_manager->stop();
    }
    
    // 카메라와 리소스 정리는 main 함수에서 처리
    // (시그널 핸들러에서는 직접 해제하지 않음 - 다른 스레드에서 사용 중일 수 있음)
}

// RGB 캡처 스레드
void rgb_capture_thread() {
    const double frame_interval = 1.0 / RGB_TARGET_FPS;
    auto last_frame_time = std::chrono::steady_clock::now();
    
    while (is_running) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_frame_time).count() / 1000.0;
        
        if (elapsed < frame_interval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>((frame_interval - elapsed) * 1000)));
            continue;
        }
        
        last_frame_time = std::chrono::steady_clock::now();
        
        if (!camera_manager) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        cv::Mat frame;
        if (camera_manager->read_rgb_frame(frame)) {
            if (frame.rows >= 200 && frame.cols >= 200) {
                if (frame.rows != OUTPUT_HEIGHT || frame.cols != OUTPUT_WIDTH) {
                    cv::resize(frame, frame, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
                }
                rgb_frame_queue.push(frame);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

// 열화상 캡처 스레드
void thermal_capture_thread() {
    int consecutive_failures = 0;
    const int max_failures = 3;
    
    while (is_running) {
        if (!camera_manager || !thermal_processor) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        cv::Mat frame;
        if (camera_manager->read_thermal_frame(frame)) {
            consecutive_failures = 0;
            thermal_processor->extract_thermal_data(frame, thermal_data);
            
#ifdef ENABLE_ROS2
            // ROS2 토픽 발행 (선택적)
            if (thermal_ros2_publisher && ros2_node) {
                thermal_ros2_publisher->publishThermalData(thermal_data);
                if (!frame.empty()) {
                    thermal_ros2_publisher->publishThermalImage(frame);
                }
                // ROS2 스핀 (비동기)
                rclcpp::spin_some(ros2_node);
            }
#endif
        } else {
            consecutive_failures++;
            if (consecutive_failures >= max_failures) {
                std::cout << "  ⚠ 열화상 읽기 실패 " << consecutive_failures 
                          << "회 - 재연결 시도..." << std::endl;
                consecutive_failures = 0;
            }
        }
        
        // Lepton 3.5는 8-9 FPS (약 0.12초 간격)
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
}

// 라이다 데이터 수집 스레드
void lidar_thread() {
    while (is_running) {
        if (!lidar_interface || !targeting_compositor) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (lidar_interface->isConnected()) {
            // 최신 스캔 데이터 가져오기
            LidarScan scan;
            if (lidar_interface->getLatestScan(scan)) {
                // TargetingFrameCompositor에 라이다 데이터 전달
                targeting_compositor->setLidarData(scan.points);
                
#ifdef ENABLE_ROS2
                // ROS2 토픽 발행 (선택적)
                if (lidar_ros2_publisher && ros2_node) {
                    lidar_ros2_publisher->publishLidarPoints(scan.points);
                    // 전방 거리 발행
                    float front_dist = lidar_interface->getFrontDistance();
                    if (front_dist > 0.0f) {
                        lidar_ros2_publisher->publishFrontDistance(front_dist);
                    }
                    // ROS2 스핀 (비동기)
                    rclcpp::spin_some(ros2_node);
                }
#endif
            }
        }
        
        // 라이다는 10Hz이므로 100ms 간격으로 업데이트
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 프레임 합성 스레드
void composite_thread() {
    int frame_count = 0;
    auto last_time = std::chrono::steady_clock::now();
    const double frame_interval = 1.0 / RGB_TARGET_FPS;
    auto last_frame_time = std::chrono::steady_clock::now();
    
    while (is_running) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_frame_time).count() / 1000.0;
        
        if (elapsed < frame_interval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>((frame_interval - elapsed) * 1000)));
            continue;
        }
        
        last_frame_time = std::chrono::steady_clock::now();
        
        if (!thermal_overlay || !targeting_compositor) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // RGB 프레임 가져오기
        cv::Mat rgb_frame;
        if (!rgb_frame_queue.try_pop(rgb_frame, 0)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // 큐에 더 있으면 모두 버리고 최신 것만 사용
        while (!rgb_frame_queue.empty()) {
            cv::Mat dummy;
            rgb_frame_queue.try_pop(dummy, 0);
        }
        
        if (rgb_frame.empty()) {
            continue;
        }
        
        // 프레임 크기 조정
        cv::Mat output;
        if (rgb_frame.rows != OUTPUT_HEIGHT || rgb_frame.cols != OUTPUT_WIDTH) {
            cv::resize(rgb_frame, output, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
        } else {
            output = rgb_frame.clone();
        }
        
        // 열화상 데이터 가져오기 (스레드 안전 복사)
        ThermalData data = thermal_data.copy();
        
        // 기본 오버레이 (열화상 레이어, 로고)
        if (OVERLAY_THERMAL && data.valid && !data.frame.empty()) {
            thermal_overlay->overlayThermal(output, data.frame);
        }
        thermal_overlay->overlayLogo(output);
        
        // 타겟팅 오버레이 (조준, 라이다, hotspot)
        targeting_compositor->compositeTargeting(output, data);
        
        cv::Mat composite = output;
        
        if (composite.empty()) {
            continue;
        }
        
        // BGR → RGB 변환
        cv::Mat composite_rgb;
        cv::cvtColor(composite, composite_rgb, cv::COLOR_BGR2RGB);
        
        // RTSP용 큐에 넣기
        frame_queue.push(composite_rgb);
        
        // 웹 서버용 큐에 넣기
        if (ENABLE_HTTP_SERVER) {
            web_frame_queue.push(composite_rgb);
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

int main(int argc, char* argv[]) {
    // OpenCV 경고 억제 (열화상 카메라 타임아웃은 정상 동작)
    // 환경 변수로 FFMPEG/OpenCV 로그 억제
    setenv("OPENCV_FFMPEG_LOGLEVEL", "-8", 0);
    setenv("OPENCV_LOG_LEVEL", "ERROR", 0);
    
    // OpenCV 로그 레벨 설정 시도 (OpenCV 4.5+)
    #if CV_VERSION_MAJOR >= 4
        #if CV_VERSION_MINOR >= 5
            // OpenCV 4.5+ 버전
            try {
                cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
            } catch (...) {
                // 함수가 없으면 무시
            }
        #endif
    #endif
    
    std::cout << "=" << std::string(60, '=') << std::endl;
    std::cout << "  RGB + 열화상 데이터 RTSP (저지연 최적화)" << std::endl;
    std::cout << "=" << std::string(60, '=') << std::endl;
    
    // 시그널 핸들러 등록
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // GStreamer 초기화
    gst_init(&argc, &argv);
    
    std::cout << "\n[초기화]" << std::endl;
    
    // 카메라 프로세스 확인 (종료하지 않고 확인만)
    check_camera_processes();
    
    // 기존 프로세스 종료
    kill_existing_processes();
    
    // USB 카메라 재연결 (설정에 따라)
    #if RESET_USB_CAMERAS_ON_START
    reset_all_usb_cameras();
    #endif
    
    // 카메라 초기화
    camera_manager = new CameraManager();
    if (!camera_manager->initialize()) {
        std::cerr << "카메라 초기화 실패" << std::endl;
        delete camera_manager;
        return 1;
    }
    std::cout << "  ✓ 출력: " << OUTPUT_WIDTH << "x" << OUTPUT_HEIGHT 
              << " @ " << OUTPUT_FPS << "fps" << std::endl;
    
    // 프로세서 초기화
    thermal_processor = new ThermalProcessor();
    
    // 기본 오버레이 (열화상 레이어, 로고)
    thermal_overlay = new ThermalBasicOverlay();
    
    // 타겟팅 프레임 합성 (조준, 라이다, hotspot)
    targeting_compositor = new TargetingFrameCompositor();
    
#ifdef ENABLE_ROS2
    // ROS2 발행자 초기화
    if (ros2_node) {
        thermal_ros2_publisher = new ThermalROS2Publisher(ros2_node);
        lidar_ros2_publisher = new LidarROS2Publisher(ros2_node);
        std::cout << "  ✓ ROS2 토픽 발행 활성화" << std::endl;
    }
#endif
    
#ifdef ENABLE_ROS2
    // ROS2 발행자 초기화
    if (ros2_node) {
        thermal_ros2_publisher = new ThermalROS2Publisher(ros2_node);
        lidar_ros2_publisher = new LidarROS2Publisher(ros2_node);
        std::cout << "  ✓ ROS2 토픽 발행 활성화" << std::endl;
    }
#endif
    
    // 라이다 오리엔테이션 설정 (각도 오프셋)
    // 라이다의 0도 방향이 실제 전방과 다를 경우 오프셋 설정
    // config.h의 LIDAR_ORIENTATION_OFFSET 값 사용
    targeting_compositor->setLidarOrientation(LIDAR_ORIENTATION_OFFSET);
    if (LIDAR_ORIENTATION_OFFSET != 0.0f) {
        std::cout << "  ✓ 라이다 오리엔테이션 오프셋: " << LIDAR_ORIENTATION_OFFSET << "도" << std::endl;
    }
    
    // 라이다 디스플레이 모드 설정
    targeting_compositor->setLidarDisplayMode(LIDAR_DISPLAY_MODE);
    targeting_compositor->setLidarShowDirectionLines(LIDAR_SHOW_DIRECTION_LINES);
    targeting_compositor->setLidarThreePointTolerance(LIDAR_THREE_POINT_TOLERANCE);
    
    // 라이다 초기화 (VIM4 UART_E 사용)
    std::cout << "\n[라이다 초기화]" << std::endl;
    LidarConfig lidar_config = LidarConfig::createUartEConfig();
    lidar_interface = new LidarInterface(lidar_config);
    if (lidar_interface->start()) {
        std::cout << "  ✓ 라이다 연결 성공: " << lidar_config.device_path << std::endl;
    } else {
        std::cout << "  ⚠ 라이다 연결 실패: " << lidar_interface->getLastError() << std::endl;
        std::cout << "  → 라이다 없이 계속 진행" << std::endl;
    }
    
    // 스레드 시작
    std::cout << "\n[스레드 시작]" << std::endl;
    std::cout << "  → 비동기 카메라 캡처 시작 (RGB, 열화상 독립 스레드)" << std::endl;
    std::thread rgb_thread(rgb_capture_thread);
    std::thread thermal_thread(thermal_capture_thread);
    std::thread lidar_thread_obj(lidar_thread);
    std::thread composite_thread_obj(composite_thread);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 인코더 확인
    std::cout << "\n[인코더]" << std::endl;
    if (check_encoder("openh264enc")) {
        std::cout << "  ✓ openh264enc" << std::endl;
    } else if (check_encoder("x264enc")) {
        std::cout << "  ✓ x264enc" << std::endl;
    }
    
    // 스트리밍 서버 시작
    std::cout << "\n[스트리밍 서버 시작]" << std::endl;
    streaming_manager = new StreamingManager();
    if (!streaming_manager->initialize(&frame_queue, &web_frame_queue)) {
        std::cerr << "  ✗ 스트리밍 서버 초기화 실패" << std::endl;
        is_running = false;
    } else {
        streaming_manager->start();
    }
    
    std::string rtsp_url = streaming_manager->getRTSPUrl();
    std::string http_url = streaming_manager->getHTTPUrl();
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "  ✓ RTSP 서버 시작됨" << std::endl;
    if (ENABLE_HTTP_SERVER && !http_url.empty()) {
        std::cout << "  ✓ HTTP 웹 서버 시작됨" << std::endl;
    }
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
    std::cout << "\n대기 중... (Ctrl+C: 종료)\n" << std::endl;
    
    // 메인 루프
    while (is_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "\n[종료 중...]" << std::endl;
    
    // 스레드 종료 대기
    std::cout << "  → 스레드 종료 대기 중..." << std::endl;
    if (rgb_thread.joinable()) {
        rgb_thread.join();
    }
    if (thermal_thread.joinable()) {
        thermal_thread.join();
    }
    if (lidar_thread_obj.joinable()) {
        lidar_thread_obj.join();
    }
    if (composite_thread_obj.joinable()) {
        composite_thread_obj.join();
    }
    std::cout << "  ✓ 모든 스레드 종료 완료" << std::endl;
    
    // 리소스 정리
    std::cout << "  → 리소스 정리 중..." << std::endl;
    
    // 서버 먼저 중지
    if (streaming_manager) {
        streaming_manager->stop();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 카메라 먼저 해제 (다른 프로세스가 사용할 수 있도록)
    if (camera_manager) {
        delete camera_manager;
        camera_manager = nullptr;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 카메라 해제 대기
    }
    std::cout << "  ✓ 카메라 해제 완료" << std::endl;
    
    // 기타 리소스 해제
    if (lidar_interface) {
        lidar_interface->stop();
        delete lidar_interface;
        lidar_interface = nullptr;
        std::cout << "  ✓ 라이다 해제 완료" << std::endl;
    }
    if (thermal_processor) {
        delete thermal_processor;
        thermal_processor = nullptr;
    }
    if (thermal_overlay) {
        delete thermal_overlay;
        thermal_overlay = nullptr;
    }
    if (targeting_compositor) {
        delete targeting_compositor;
        targeting_compositor = nullptr;
    }
    if (streaming_manager) {
        delete streaming_manager;
        streaming_manager = nullptr;
    }
    std::cout << "  ✓ 스트리밍 서버 리소스 해제 완료" << std::endl;
    
    // 기존 프로세스 정리 (다음 실행을 위해)
    std::cout << "  → 기존 프로세스 정리 중..." << std::endl;
    kill_existing_processes();
    
    std::cout << "\n[완료] 모든 리소스 해제 완료" << std::endl;
    
    return 0;
}
