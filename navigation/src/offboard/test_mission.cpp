#include <rclcpp/rclcpp.hpp>
#include "autonomous/offboard_manager.h"
#include "autonomous/waypoint_handler.h"
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_mission_node");

    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "  Full Mission Test Program");
    RCLCPP_INFO(node->get_logger(), "========================================\n");

    // OffboardManager 생성
    OffboardManager manager(node);

    // GPS 신호 수신 대기
    RCLCPP_INFO(node->get_logger(), "Waiting for GPS signal...");
    auto waypoint_handler = std::make_unique<WaypointHandler>(node);
    for (int i = 0; i < 50; i++) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    GPSCoordinate home = waypoint_handler->getCurrentPosition();
    RCLCPP_INFO(node->get_logger(), "Home position: Lat=%.7f, Lon=%.7f, Alt=%.2f m\n",
                home.latitude, home.longitude, home.altitude);

    // 미션 설정
    MissionConfig config;
    config.takeoff_altitude = 5.0f;

    // 목표 waypoint (현재 위치에서 북쪽 20m, 동쪽 20m)
    config.target_waypoint.latitude = home.latitude + (20.0 / 111000.0);
    config.target_waypoint.longitude = home.longitude + (20.0 / 88000.0);
    config.target_waypoint.altitude = home.altitude + 5.0f;

    config.target_distance = 10.0f;     // 10m 거리
    config.distance_tolerance = 1.0f;   // ±1m
    config.hover_duration_sec = 10.0f;  // 10초 호버링

    // 미션 실행
    bool success = manager.executeMission(config);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "\n========================================");
        RCLCPP_INFO(node->get_logger(), "  Mission Successful!");
        RCLCPP_INFO(node->get_logger(), "========================================");
    } else {
        RCLCPP_ERROR(node->get_logger(), "\n========================================");
        RCLCPP_ERROR(node->get_logger(), "  Mission Failed!");
        RCLCPP_ERROR(node->get_logger(), "========================================");
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}
