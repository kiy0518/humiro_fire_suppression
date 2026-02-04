/**
 * @file test_mission2.cpp
 * @brief executeMission2 테스트 프로그램
 *
 * FireMissionStart 메시지를 받으면 자동으로 미션을 실행합니다.
 *
 * 사용법:
 *   ros2 run navigation test_mission2
 *
 * 그 다음 QGC나 다른 프로그램에서 FireMissionStart 메시지를 전송하면
 * 자동으로 미션이 시작됩니다.
 */

#include <rclcpp/rclcpp.hpp>
#include "../offboard/autonomous/offboard_manager.h"
#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_mission2");

    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "╔════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(node->get_logger(), "║           Test Mission 2 시작                             ║");
    RCLCPP_INFO(node->get_logger(), "║   FireMissionStart 메시지 대기 후 자동 미션 실행         ║");
    RCLCPP_INFO(node->get_logger(), "╚════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(node->get_logger(), "");

    // OffboardManager 생성
    OffboardManager manager(node);

    // MissionConfig 설정 (이륙 3m, 호버 10초)
    MissionConfig config;
    config.takeoff_altitude = 3.0f;
    config.hover_duration_sec = 10.0f;

    // executeMission2 실행 (블로킹)
    bool success = manager.executeMission2(config);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "\n[성공] Mission 2 완료!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "\n[실패] Mission 2 실패!");
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}
