#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <gst/gst.h>
#include "rtsp_server.h"
#include "../../thermal/src/thread_safe_queue.h"
#include "../../osd/src/status/status_overlay.h"
#include "../../osd/src/lidar/distance_overlay.h"
#include "../../osd/src/mission/mission_overlay.h"
#include "../../custom_message/include/custom_message/custom_message.h"

// 더미 프레임 생성
cv::Mat createDummyFrame(int width, int height, int frameCount) {
    cv::Mat frame = cv::Mat::zeros(height, width, CV_8UC3);

    // 그리드
    cv::Scalar gridColor(30, 30, 30);
    for (int y = 0; y < height; y += 50) {
        cv::line(frame, cv::Point(0, y), cv::Point(width, y), gridColor, 1);
    }
    for (int x = 0; x < width; x += 50) {
        cv::line(frame, cv::Point(x, 0), cv::Point(x, height), gridColor, 1);
    }

    // 프레임 카운터
    std::string text = "OSD Test - Frame: " + std::to_string(frameCount);
    cv::putText(frame, text, cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 100, 100), 2);

    // 중앙 십자선
    cv::line(frame, cv::Point(width/2 - 20, height/2),
             cv::Point(width/2 + 20, height/2), cv::Scalar(50, 50, 50), 2);
    cv::line(frame, cv::Point(width/2, height/2 - 20),
             cv::Point(width/2, height/2 + 20), cv::Scalar(50, 50, 50), 2);

    return frame;
}

int main(int argc, char* argv[]) {
    // GStreamer 초기화
    gst_init(&argc, &argv);

    std::cout << "=================================" << std::endl;
    std::cout << "  OSD + Custom Message Test" << std::endl;
    std::cout << "=================================" << std::endl;

    const int width = 1280;
    const int height = 720;
    const int fps = 30;

    // 프레임 큐
    ThreadSafeQueue<cv::Mat> frame_queue;

    // RTSP 서버
    RTSPServer rtsp_server;
    if (!rtsp_server.initialize(&frame_queue)) {
        std::cerr << "Failed to initialize RTSP server" << std::endl;
        return 1;
    }

    std::cout << "\n[RTSP URL]" << std::endl;
    std::cout << rtsp_server.get_rtsp_url() << std::endl;
    std::cout << "\nConnect from QGC with this URL\n" << std::endl;

    rtsp_server.start();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // OSD 오버레이
    StatusOverlay status_overlay;
    DistanceOverlay distance_overlay;
    MissionOverlay mission_overlay;

    // 커스텀 메시지 송수신기
    custom_message::CustomMessage custom_msg(15000, 15001, "0.0.0.0", "127.0.0.1", 1, 1);

    // 커스텀 메시지 콜백 등록
    custom_msg.setFireMissionStartCallback([&](const custom_message::FireMissionStart& start) {
        std::cout << "[CustomMsg] Mission Start received: target="
                  << start.target_lat / 1e7 << "," << start.target_lon / 1e7
                  << " auto_fire=" << static_cast<int>(start.auto_fire) << std::endl;
        mission_overlay.updateMissionStart(start);
    });

    custom_msg.setFireMissionStatusCallback([&](const custom_message::FireMissionStatus& status) {
        std::cout << "[CustomMsg] Mission Status: phase=" << static_cast<int>(status.phase)
                  << " progress=" << static_cast<int>(status.progress) << "%" << std::endl;
        mission_overlay.updateMissionStatus(status);
    });

    custom_msg.setFireLaunchControlCallback([&](const custom_message::FireLaunchControl& control) {
        std::cout << "[CustomMsg] Launch Control: command=" << static_cast<int>(control.command) << std::endl;
    });

    custom_msg.setFireSuppressionResultCallback([&](const custom_message::FireSuppressionResult& result) {
        std::cout << "[CustomMsg] Suppression Result: shot=" << static_cast<int>(result.shot_number)
                  << " success=" << static_cast<int>(result.success)
                  << " temp: " << result.temp_before / 10.0 << "C -> " << result.temp_after / 10.0 << "C" << std::endl;
        mission_overlay.updateSuppressionResult(result);
    });

    // 커스텀 메시지 수신 시작
    if (!custom_msg.start()) {
        std::cerr << "Failed to start custom message receiver" << std::endl;
        return 1;
    }
    std::cout << "Custom message receiver started on port 15000" << std::endl;

    // 테스트 데이터
    std::cout << "Setting up test data..." << std::endl;

    status_overlay.updatePx4State("OFFBOARD", true);
    status_overlay.updateOffboardStatus(StatusOverlay::DroneStatus::NAVIGATING);
    status_overlay.setBattery(87);
    status_overlay.setAmmunition(5, 6);
    status_overlay.setFormation(1, 3);

    // LiDAR 테스트 데이터
    std::vector<LidarPoint> test_points;
    for (int angle = 0; angle < 360; angle += 5) {
        float distance = 5.0f + 3.0f * std::sin(angle * M_PI / 180.0);
        test_points.push_back({(float)angle, distance, 255});
    }
    distance_overlay.setLidarData(test_points);

    std::cout << "Starting frame generation..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    int frameCount = 0;
    auto startTime = std::chrono::steady_clock::now();

    // 미션 시뮬레이션을 위한 변수
    int mission_phase = 0;  // 0=IDLE, 1=NAVIGATING, 2=SCANNING, etc.
    int mission_progress = 0;
    int shot_count = 0;

    while (true) {
        // 더미 프레임 생성
        cv::Mat frame = createDummyFrame(width, height, frameCount);

        // LiDAR 오버레이
        distance_overlay.drawOverlay(frame);

        // 상태 오버레이
        status_overlay.draw(frame);

        // 미션 오버레이 (커스텀 메시지)
        mission_overlay.draw(frame);

        // 타임스탬프
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
        std::string timeText = "Running: " + std::to_string(elapsed) + "s";
        cv::putText(frame, timeText, cv::Point(width - 200, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);

        // 프레임 큐에 추가
        frame_queue.push(frame);

        // FPS 제어
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));

        frameCount++;

        // 5초마다 상태 업데이트 및 테스트 메시지 전송
        if (frameCount % (fps * 5) == 0) {
            int battery = 87 - (frameCount / (fps * 10)) * 5;
            if (battery < 0) battery = 0;
            status_overlay.setBattery(battery);

            // LiDAR 데이터 변화
            test_points.clear();
            float offset = frameCount * 0.01f;
            for (int angle = 0; angle < 360; angle += 5) {
                float distance = 5.0f + 3.0f * std::sin((angle + offset) * M_PI / 180.0);
                test_points.push_back({(float)angle, distance, 255});
            }
            distance_overlay.setLidarData(test_points);

            // 테스트 미션 상태 전송 (시뮬레이션)
            custom_message::FireMissionStatus status{};
            status.phase = mission_phase;
            status.progress = mission_progress;
            status.remaining_projectiles = 6 - shot_count;
            status.distance_to_target = 100.0f - mission_progress;
            status.thermal_max_temp = 450 + (mission_phase * 50);
            snprintf(status.status_text, sizeof(status.status_text), "Test mission in progress");

            custom_msg.sendFireMissionStatus(status);

            // 미션 진행 시뮬레이션
            mission_progress += 10;
            if (mission_progress >= 100) {
                mission_progress = 0;
                mission_phase++;
                if (mission_phase > 6) {
                    mission_phase = 0;
                }

                // Phase 4 (SUPPRESSING)에서 발사 결과 전송
                if (mission_phase == 4 && shot_count < 6) {
                    custom_message::FireSuppressionResult result{};
                    result.shot_number = ++shot_count;
                    result.temp_before = 650;
                    result.temp_after = 350;
                    result.success = 1;

                    custom_msg.sendFireSuppressionResult(result);
                }
            }

            std::cout << "Frame: " << frameCount
                      << ", Battery: " << battery << "%"
                      << ", Phase: " << mission_phase
                      << ", Progress: " << mission_progress << "%" << std::endl;

            // 통계 출력
            auto stats = custom_msg.getStatistics();
            std::cout << "  [Stats] Status sent=" << stats.mission_status_sent
                      << ", Result sent=" << stats.suppression_result_sent
                      << ", Status recv=" << stats.mission_status_received
                      << ", Errors=" << stats.send_error_count << std::endl;
        }
    }

    custom_msg.stop();
    rtsp_server.stop();
    return 0;
}
