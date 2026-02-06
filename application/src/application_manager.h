#ifndef APPLICATION_MANAGER_H
#define APPLICATION_MANAGER_H

#include <atomic>
#include <thread>
#include <memory>
#include <opencv2/opencv.hpp>
#include "thread_safe_queue.h"
#include "thermal_data.h"

// Forward declarations
class CameraManager;
class ThermalProcessor;
class ThermalOverlay;
class StatusOverlay;
class TargetingFrameCompositor;
class StreamingManager;
class LidarInterface;
class FrameCompositor;

namespace custom_message {
struct FireMissionStart;
class CustomMessage;
}

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
class ThermalROS2Publisher;
class LidarROS2Publisher;
class StatusROS2Subscriber;
class OffboardManager;  // 추가: 자율 비행 관리자
#endif

/**
 * @brief 애플리케이션 전체 관리 클래스
 * 
 * 모든 컴포넌트와 스레드를 관리하며, 초기화, 실행, 종료를 담당합니다.
 */
class ApplicationManager {
public:
    ApplicationManager();
    ~ApplicationManager();
    
    /**
     * @brief 애플리케이션 초기화
     * 
     * @param argc 명령줄 인수 개수
     * @param argv 명령줄 인수 배열
     * @return 초기화 성공 여부
     */
    bool initialize(int argc, char* argv[]);
    
    /**
     * @brief 애플리케이션 실행 (메인 루프)
     */
    void run();
    
    /**
     * @brief 애플리케이션 종료 및 리소스 정리
     */
    void shutdown();
    
    /**
     * @brief 실행 중지 요청
     */
    void stop();
    
    /**
     * @brief 실행 중인지 확인
     */
    bool isRunning() const { return is_running_; }

private:
    // 컴포넌트
    CameraManager* camera_manager_;
    ThermalProcessor* thermal_processor_;
    ThermalOverlay* thermal_overlay_;
    StatusOverlay* status_overlay_;
    TargetingFrameCompositor* targeting_compositor_;
    StreamingManager* streaming_manager_;
    LidarInterface* lidar_interface_;
    FrameCompositor* frame_compositor_;
    custom_message::CustomMessage* custom_message_handler_;  // 외부 메시지용 (15001 - 라우터와 독립)
    custom_message::CustomMessage* test_message_handler_;    // 외부 테스트용 (14553 - EXTERNAL_UDP_PORT)
    
#ifdef ENABLE_ROS2
    rclcpp::Node::SharedPtr ros2_node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    ThermalROS2Publisher* thermal_ros2_publisher_;
    LidarROS2Publisher* lidar_ros2_publisher_;
    StatusROS2Subscriber* status_ros2_subscriber_;
    OffboardManager* offboard_manager_;  // 추가: 자율 비행 관리자
#endif
    
    // 큐
    ThreadSafeQueue<cv::Mat> rgb_frame_queue_;
    ThreadSafeQueue<cv::Mat> frame_queue_;
    ThreadSafeQueue<cv::Mat> web_frame_queue_;
    ThermalData thermal_data_;
    
    // 스레드
    std::thread rgb_capture_thread_;
    std::thread thermal_capture_thread_;
    std::thread lidar_thread_;
    std::thread composite_thread_;
    std::thread camera_init_thread_;
    std::thread ammo_sim_thread_;
    
    // 상태
    std::atomic<bool> is_running_;
    std::atomic<bool> rgb_init_done_;
    std::atomic<bool> thermal_init_done_;
    std::atomic<bool> mission_running_;  // 미션 실행 중 플래그 (중복 실행 방지)
    uint8_t drone_id_ = 1;  // DRONE_ID 환경 변수에서 로드 (멀티 드론 식별용)
    
    // 내부 메서드
    void initializeROS2(int argc, char* argv[]);
    void initializeComponents();
    void initializeStreaming();
    void initializeCustomMessage();
    void startThreads();
    void stopThreads();
    
    // 스레드 함수
    void rgbCaptureLoop();
    void thermalCaptureLoop();
    void lidarLoop();
    void compositeLoop();
    void cameraInitLoop();
    void ammunitionSimulationLoop();
    
    // 정리 메서드
    void cleanupComponents();
    void cleanupROS2();
    
    // 미션 실행
    void executeMission(const custom_message::FireMissionStart& start);

    // offboard 설정 파일에서 target_altitude 읽기
    float readTargetAltitudeFromConfig() const;

    // 미션 종료 통합 정리 (mission_running_ 리셋 + OffboardManager 정리)
    void finishMission(bool reset_offboard = true);

    // target_system 검증 (멀티 드론 환경에서 자신에게 보낸 메시지인지 확인)
    bool isTargetedToMe(uint8_t target_system) const;
    // void testExeMission(const custom_message::FireMissionStart& start);
    // void testExeMission3(const custom_message::FireMissionStart& start);
};

#endif // APPLICATION_MANAGER_H
