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
#include "../thermal/src/camera_manager.h"
#include "../thermal/src/thermal_processor.h"
#include "../osd/src/thermal/thermal_overlay.h"
#include "../osd/src/status/status_overlay.h"
#include "../thermal/src/thread_safe_queue.h"
#include "../thermal/src/thermal_data.h"
#include "../thermal/src/utils.h"

// 라이다 인터페이스
#include "../lidar/src/lidar_interface.h"
#include "../lidar/src/lidar_config.h"

// 타겟팅 프레임 합성
#include "../targeting/src/targeting_frame_compositor.h"

// 스트리밍 관리자
#include "../streaming/src/streaming_manager.h"

// ROS2 통합 (선택적)
#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include "../thermal/src/thermal_ros2_publisher.h"
#include "../lidar/src/lidar_ros2_publisher.h"
#include "../ros2/src/status/status_ros2_subscriber.h"
#endif

// 전역 변수
std::atomic<bool> is_running(true);
CameraManager* camera_manager = nullptr;
ThermalProcessor* thermal_processor = nullptr;
ThermalOverlay* thermal_overlay = nullptr;
StatusOverlay* status_overlay = nullptr;
TargetingFrameCompositor* targeting_compositor = nullptr;
StreamingManager* streaming_manager = nullptr;
LidarInterface* lidar_interface = nullptr;

#ifdef ENABLE_ROS2
rclcpp::Node::SharedPtr ros2_node = nullptr;
ThermalROS2Publisher* thermal_ros2_publisher = nullptr;
LidarROS2Publisher* lidar_ros2_publisher = nullptr;
StatusROS2Subscriber* status_ros2_subscriber = nullptr;
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
    int consecutive_failures = 0;
    const int max_failures = 5;  // 5회 연속 실패 시 재연결
    
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
            consecutive_failures = 0;  // 성공 시 카운터 리셋
            if (frame.rows >= 200 && frame.cols >= 200) {
                if (frame.rows != OUTPUT_HEIGHT || frame.cols != OUTPUT_WIDTH) {
                    cv::resize(frame, frame, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
                }
                rgb_frame_queue.push(frame);
            }
        } else {
            consecutive_failures++;
            if (consecutive_failures >= max_failures) {
                std::cout << "  ⚠ RGB 카메라 읽기 실패 " << consecutive_failures 
                          << "회 - 재연결 시도..." << std::endl;
                if (camera_manager->reconnect_rgb_camera()) {
                    consecutive_failures = 0;
                } else {
                    // 재연결 실패 시 잠시 대기 후 재시도
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
}

// 열화상 캡처 스레드
void thermal_capture_thread() {
    int consecutive_failures = 0;
    const int max_failures = 5;  // 5회 연속 실패 시 재연결
    
    while (is_running) {
        if (!camera_manager || !thermal_processor) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        cv::Mat frame;
        if (camera_manager->read_thermal_frame(frame)) {
            consecutive_failures = 0;  // 성공 시 카운터 리셋
            thermal_processor->extract_thermal_data(frame, thermal_data);
            
#ifdef ENABLE_ROS2
            // ROS2 토픽 발행 (선택적)
            if (thermal_ros2_publisher && ros2_node) {
                thermal_ros2_publisher->publishThermalData(thermal_data);
                if (!frame.empty()) {
                    thermal_ros2_publisher->publishThermalImage(frame);
                }
                // 발행자는 spin이 필요 없음 (비동기 발행)
                // spin은 메인 루프에서 status_ros2_subscriber->spin()으로 통합 처리
            }
#endif
        } else {
            consecutive_failures++;
            if (consecutive_failures >= max_failures) {
                std::cout << "  ⚠ 열화상 읽기 실패 " << consecutive_failures 
                          << "회 - 재연결 시도..." << std::endl;
                if (camera_manager->reconnect_thermal_camera()) {
                    consecutive_failures = 0;
                } else {
                    // 재연결 실패 시 잠시 대기 후 재시도
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
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
                    // 발행자는 spin이 필요 없음 (비동기 발행)
                    // spin은 메인 루프에서 status_ros2_subscriber->spin()으로 통합 처리
                }
#endif
            }
        }
        
        // 라이다는 10Hz이므로 100ms 간격으로 업데이트
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 플레이스홀더 그리기 함수
void drawPlaceholder(cv::Mat& frame, const std::string& message) {
    if (frame.empty()) {
        frame = cv::Mat(OUTPUT_HEIGHT, OUTPUT_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    }
    
    // 중앙에 메시지 표시
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    int thickness = 2;
    cv::Scalar text_color(255, 255, 255);  // 흰색
    
    cv::Size text_size = cv::getTextSize(message, font_face, font_scale, thickness, nullptr);
    int text_x = (frame.cols - text_size.width) / 2;
    int text_y = (frame.rows + text_size.height) / 2;
    
    // 배경 사각형 (반투명 검은색)
    int padding = 20;
    cv::Rect bg_rect(text_x - padding, text_y - text_size.height - padding,
                     text_size.width + 2 * padding, text_size.height + 2 * padding);
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay, bg_rect, cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.7, frame, 0.3, 0, frame);
    
    // 텍스트 그리기
    cv::putText(frame, message, cv::Point(text_x, text_y), font_face, font_scale, text_color, thickness, cv::LINE_AA);
}

// 탄의 갯수 시뮬레이션 스레드 (1초에 1개씩 감소)
void ammunition_simulation_thread() {
    int current_ammo = 6;  // 초기값
    const int max_ammo = 6;
    auto last_update = std::chrono::steady_clock::now();
    
    while (is_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_update).count();
        
        if (elapsed >= 1) {  // 1초마다
            if (current_ammo > 0) {
                current_ammo--;
                if (status_overlay) {
                    status_overlay->setAmmunition(current_ammo, max_ammo);
                    std::cout << "  [시뮬레이션] 탄의 갯수: " << current_ammo << "/" << max_ammo << std::endl;
                }
            } else {
                // 0이 되면 다시 6으로 리셋 (테스트용)
                current_ammo = max_ammo;
                if (status_overlay) {
                    status_overlay->setAmmunition(current_ammo, max_ammo);
                    std::cout << "  [시뮬레이션] 탄의 갯수 리셋: " << current_ammo << "/" << max_ammo << std::endl;
                }
            }
            last_update = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 프레임 합성 스레드 (비동기식: 부분 데이터 처리)
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
        
        // RGB 프레임 가져오기 (없어도 계속 진행)
        cv::Mat rgb_frame;
        bool has_rgb = rgb_frame_queue.try_pop(rgb_frame, 0);
        
        // 큐에 더 있으면 모두 버리고 최신 것만 사용
        while (!rgb_frame_queue.empty()) {
            cv::Mat dummy;
            rgb_frame_queue.try_pop(dummy, 0);
        }
        
        // 열화상 데이터 가져오기 (스레드 안전 복사)
        ThermalData data = thermal_data.copy();
        bool has_thermal = data.valid && !data.frame.empty();
        
        // 출력 프레임 준비
        cv::Mat output;
        
        if (has_rgb && !rgb_frame.empty()) {
            // RGB 프레임이 있으면 사용
            if (rgb_frame.rows != OUTPUT_HEIGHT || rgb_frame.cols != OUTPUT_WIDTH) {
                cv::resize(rgb_frame, output, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
            } else {
                output = rgb_frame.clone();
            }
        } else if (has_thermal && !data.frame.empty()) {
            // RGB가 없고 열화상만 있으면 열화상을 배경으로 사용
            cv::Mat thermal_resized;
            cv::resize(data.frame, thermal_resized, cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
            // 열화상은 그레이스케일이므로 BGR로 변환
            if (thermal_resized.channels() == 1) {
                cv::cvtColor(thermal_resized, output, cv::COLOR_GRAY2BGR);
            } else {
                output = thermal_resized.clone();
            }
        } else {
            // 둘 다 없으면 플레이스홀더 생성
            output = cv::Mat(OUTPUT_HEIGHT, OUTPUT_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
            
            // 준비 상태에 따라 메시지 결정
            bool rgb_ready = camera_manager && camera_manager->is_rgb_ready();
            bool thermal_ready = camera_manager && camera_manager->is_thermal_ready();
            
            if (!rgb_ready && !thermal_ready) {
                drawPlaceholder(output, "카메라 초기화 중...");
            } else if (!rgb_ready) {
                drawPlaceholder(output, "RGB 카메라 초기화 중...");
            } else if (!thermal_ready) {
                drawPlaceholder(output, "열화상 카메라 초기화 중...");
            } else {
                drawPlaceholder(output, "프레임 대기 중...");
            }
        }
        
        // 열화상 오버레이 (열화상 데이터가 있고 RGB가 있을 때만)
        if (has_rgb && has_thermal && OVERLAY_THERMAL) {
            thermal_overlay->overlayThermal(output, data.frame);
        }
        
        // 로고 오버레이 제거 (상태 정보가 우선)
        // thermal_overlay->overlayLogo(output);
        
        // 상태 모니터링 OSD (상단 왼쪽)
        if (status_overlay) {
            status_overlay->draw(output);
        }
        
        // 타겟팅 오버레이 (조준, 라이다, hotspot)
        // 열화상 데이터가 없어도 라이다는 표시 가능
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
    
#ifdef ENABLE_ROS2
    // ROS2 초기화
    std::cout << "\n[ROS2 초기화]" << std::endl;
    try {
        rclcpp::init(argc, argv);
        ros2_node = rclcpp::Node::make_shared("humiro_fire_suppression");
        std::cout << "  ✓ ROS2 노드 생성: humiro_fire_suppression" << std::endl;
        std::cout << "  → uXRCE-DDS 토픽 구독 준비 완료" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "  ✗ ROS2 초기화 실패: " << e.what() << std::endl;
        std::cerr << "  → ROS2 없이 계속 진행 (상태 표시는 기본값 사용)" << std::endl;
        ros2_node = nullptr;
    }
#else
    std::cout << "\n[ROS2 비활성화]" << std::endl;
    std::cout << "  ⚠ ROS2가 비활성화되어 있습니다" << std::endl;
    std::cout << "  → 상태 표시는 기본값만 사용됩니다" << std::endl;
    std::cout << "  → ROS2 활성화: cmake -DENABLE_ROS2=ON .." << std::endl;
#endif
    
    std::cout << "\n[초기화]" << std::endl;
    
    // 카메라 프로세스 확인 (종료하지 않고 확인만)
    check_camera_processes();
    
    // 기존 프로세스 종료
    kill_existing_processes();
    
    // USB 카메라 재연결 (설정에 따라)
    #if RESET_USB_CAMERAS_ON_START
    reset_all_usb_cameras();
    #endif
    
    // 카메라 초기화 (비동기식: 개별 초기화)
    std::cout << "\n[카메라 초기화]" << std::endl;
    camera_manager = new CameraManager();
    
    // 개별 카메라 초기화 (하나라도 성공하면 계속 진행)
    bool rgb_ok = camera_manager->initialize_rgb_camera();
    bool thermal_ok = camera_manager->initialize_thermal_camera();
    
    // 둘 다 실패한 경우에만 종료
    if (!rgb_ok && !thermal_ok) {
    // 카메라 없어도 계속 진행 - OSD만 출력
    // 카메라 없어도 계속 진행 - OSD만 출력
    // 카메라 없어도 계속 진행 - OSD만 출력
    // 카메라 없어도 계속 진행 - OSD만 출력
    // 카메라 없어도 계속 진행 - OSD만 출력
    // 카메라 없어도 계속 진행 - OSD만 출력
    // 카메라 없어도 계속 진행 - OSD만 출력
    // 카메라 없어도 계속 진행 - OSD만 출력
    }
    
    // 준비된 카메라 상태 출력
    std::cout << "\n[카메라 준비 상태]" << std::endl;
    if (rgb_ok) {
        std::cout << "  ✓ RGB 카메라 준비 완료" << std::endl;
    } else {
        std::cout << "  ⚠ RGB 카메라 초기화 실패 (나중에 재연결 시도)" << std::endl;
    }
    if (thermal_ok) {
        std::cout << "  ✓ 열화상 카메라 준비 완료" << std::endl;
    } else {
        std::cout << "  ⚠ 열화상 카메라 초기화 실패 (나중에 재연결 시도)" << std::endl;
    }
    
    std::cout << "  ✓ 출력: " << OUTPUT_WIDTH << "x" << OUTPUT_HEIGHT 
              << " @ " << OUTPUT_FPS << "fps" << std::endl;
    
    // 프로세서 초기화
    thermal_processor = new ThermalProcessor();
    
    // 기본 오버레이 (열화상 레이어, 로고)
    thermal_overlay = new ThermalOverlay();
    
    // 상태 모니터링 OSD (상단 표시)
    status_overlay = new StatusOverlay();
    status_overlay->setDroneName("1");  // 기본값: 숫자 1
    status_overlay->setAmmunition(6, 6);  // 기본값: 총 6발
    status_overlay->setFormation(1, 3);  // 기본값: 삼각편대 (3대)
    status_overlay->setBattery(100);  // 기본값 (테스트용)
    //     status_overlay->setGpsSatellites(0);  // 기본값 (ROS2에서 업데이트됨)
    
    // 타겟팅 프레임 합성 (조준, 라이다, hotspot)
    targeting_compositor = new TargetingFrameCompositor();
    
#ifdef ENABLE_ROS2
    // ROS2 발행자 초기화
    if (ros2_node) {
        thermal_ros2_publisher = new ThermalROS2Publisher(ros2_node);
        lidar_ros2_publisher = new LidarROS2Publisher(ros2_node);
        std::cout << "  ✓ ROS2 토픽 발행 활성화" << std::endl;
        
        // ROS2 구독자 초기화 (StatusOverlay 업데이트용)
        if (status_overlay) {
            std::cout << "\n[ROS2 상태 구독자 초기화]" << std::endl;
            status_ros2_subscriber = new StatusROS2Subscriber(ros2_node, status_overlay);
            std::cout << "  ✓ ROS2 상태 구독자 활성화" << std::endl;
            std::cout << "  → 상태바 데이터가 ROS2 토픽과 실시간 연동됩니다" << std::endl;
            std::cout << "  → 토픽 수신 확인: PX4가 연결되면 자동으로 메시지가 표시됩니다" << std::endl;
        } else {
            std::cerr << "  ⚠ status_overlay가 nullptr입니다. ROS2 구독자를 생성할 수 없습니다." << std::endl;
        }
    } else {
        std::cerr << "  ⚠ ros2_node가 nullptr입니다. ROS2 구독자를 생성할 수 없습니다." << std::endl;
        std::cerr << "    → ROS2가 제대로 초기화되었는지 확인하세요." << std::endl;
    }
#endif
    
    // 라이다 오리엔테이션 설정 (각도 오프셋)
    // 라이다의 0도 방향이 실제 전방과 다를 경우 오프셋 설정
    // config.h의 LIDAR_ORIENTATION_OFFSET 값 사용
    targeting_compositor->setLidarOrientation(LIDAR_ORIENTATION_OFFSET);
    if (LIDAR_ORIENTATION_OFFSET != 0.0f) {
        std::cout << "  ✓ 라이다 오리엔테이션 오프셋: " << LIDAR_ORIENTATION_OFFSET << "도" << std::endl;
    }
    
    
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
    std::thread ammo_sim_thread(ammunition_simulation_thread);  // 탄의 갯수 시뮬레이션
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 인코더 확인
    std::cout << "\n[인코더]" << std::endl;
    if (check_encoder("openh264enc")) {
        std::cout << "  ✓ openh264enc" << std::endl;
    } else if (check_encoder("x264enc")) {
        std::cout << "  ✓ x264enc" << std::endl;
    }
    
    // 스트리밍 서버 시작 (하나라도 준비되면 즉시 시작)
    std::cout << "\n[스트리밍 서버 시작]" << std::endl;
    
    // 준비된 카메라 확인
    bool rgb_ready = camera_manager->is_rgb_ready();
    bool thermal_ready = camera_manager->is_thermal_ready();
    
    if (!rgb_ready && !thermal_ready) {
        std::cerr << "  ✗ 스트리밍할 카메라가 없습니다" << std::endl;
        is_running = false;
    } else {
        streaming_manager = new StreamingManager();
        if (!streaming_manager->initialize(&frame_queue, &web_frame_queue)) {
            std::cerr << "  ✗ 스트리밍 서버 초기화 실패" << std::endl;
            is_running = false;
        } else {
            streaming_manager->start();
            
            std::string rtsp_url = streaming_manager->getRTSPUrl();
            std::string http_url = streaming_manager->getHTTPUrl();
            
            std::cout << "\n" << std::string(60, '=') << std::endl;
            std::cout << "  ✓ RTSP 서버 시작됨" << std::endl;
            if (ENABLE_HTTP_SERVER && !http_url.empty()) {
                std::cout << "  ✓ HTTP 웹 서버 시작됨" << std::endl;
            }
            std::cout << "  → 준비된 데이터: ";
            if (rgb_ready) std::cout << "RGB ";
            if (thermal_ready) std::cout << "열화상 ";
            std::cout << std::endl;
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
    
    std::cout << "\n대기 중... (Ctrl+C: 종료)\n" << std::endl;
    
    // 메인 루프
    while (is_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
#ifdef ENABLE_ROS2
        // ROS2 구독자 스핀 (상태 업데이트 - 상태바에 실시간 반영)
        // status_ros2_subscriber->spin()은 내부적으로 rclcpp::spin_some을 호출하므로
        // 다른 곳에서 같은 노드에 대해 spin을 호출하지 않도록 주의
        if (status_ros2_subscriber && ros2_node) {
            status_ros2_subscriber->spin();
        } else {
            // ROS2가 활성화되었지만 구독자가 없는 경우 경고 (한 번만)
            static bool warned_spin = false;
            if (!warned_spin) {
                if (!ros2_node) {
                    std::cerr << "  ⚠ ROS2 노드가 nullptr입니다. 토픽을 수신할 수 없습니다." << std::endl;
                } else if (!status_ros2_subscriber) {
                    std::cerr << "  ⚠ ROS2 구독자가 nullptr입니다. 토픽을 수신할 수 없습니다." << std::endl;
                }
                warned_spin = true;
            }
        }
        
        // ROS2 노드 유지 (rclcpp::ok() 체크)
        if (ros2_node && !rclcpp::ok()) {
            std::cerr << "  ⚠ ROS2 노드가 종료되었습니다" << std::endl;
            break;
        }
#endif
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
    if (ammo_sim_thread.joinable()) {
        ammo_sim_thread.join();
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
    if (status_overlay) {
        delete status_overlay;
        status_overlay = nullptr;
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
    
#ifdef ENABLE_ROS2
    // ROS2 구독자 해제
    if (status_ros2_subscriber) {
        delete status_ros2_subscriber;
        status_ros2_subscriber = nullptr;
        std::cout << "  ✓ ROS2 상태 구독자 해제 완료" << std::endl;
    }
    if (thermal_ros2_publisher) {
        delete thermal_ros2_publisher;
        thermal_ros2_publisher = nullptr;
    }
    if (lidar_ros2_publisher) {
        delete lidar_ros2_publisher;
        lidar_ros2_publisher = nullptr;
    }
#endif
    
    // 기존 프로세스 정리 (다음 실행을 위해)
    std::cout << "  → 기존 프로세스 정리 중..." << std::endl;
    kill_existing_processes();
    
#ifdef ENABLE_ROS2
    // ROS2 종료
    if (rclcpp::ok()) {
        rclcpp::shutdown();
        std::cout << "  ✓ ROS2 종료 완료" << std::endl;
    }
#endif
    
    std::cout << "\n[완료] 모든 리소스 해제 완료" << std::endl;
    
    return 0;
}
