#include <rclcpp/rclcpp.hpp>
#include "autonomous/arm_handler.h"
#include "autonomous/takeoff_handler.h"
#include "autonomous/waypoint_handler.h"
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_waypoint_node");

    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "  Waypoint Navigation Test Program");
    RCLCPP_INFO(node->get_logger(), "========================================");

    // Handlers 생성
    ArmHandler arm_handler(node);
    TakeoffHandler takeoff_handler(node);
    WaypointHandler waypoint_handler(node);

    // 현재 위치 수신 대기
    RCLCPP_INFO(node->get_logger(), "\n[Step 0] Waiting for GPS signal...");
    for (int i = 0; i < 50; i++) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    GPSCoordinate home = waypoint_handler.getCurrentPosition();
    RCLCPP_INFO(node->get_logger(), "Current position: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                home.latitude, home.longitude, home.altitude);

    RCLCPP_INFO(node->get_logger(), "\n[Step 1] Enabling OFFBOARD mode...");
    if (!arm_handler.enableOffboardMode()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to enable OFFBOARD mode");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "OFFBOARD mode enabled");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node->get_logger(), "\n[Step 2] Arming vehicle...");
    if (!arm_handler.arm()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Vehicle armed successfully");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node->get_logger(), "\n[Step 3] Taking off to 5.0 meters...");
    if (!takeoff_handler.takeoff(5.0f, 30000)) {
        RCLCPP_ERROR(node->get_logger(), "Takeoff failed");

        RCLCPP_INFO(node->get_logger(), "\n[Emergency] Disarming...");
        arm_handler.disarm();
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Takeoff successful! Altitude: %.2f m",
                takeoff_handler.getCurrentAltitude());
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 목표 waypoint 설정 (현재 위치에서 북쪽으로 10m, 동쪽으로 10m)
    // 위도 1도 ≈ 111km, 경도 1도 ≈ 88km (위도 35도 기준)
    GPSCoordinate target;
    target.latitude = home.latitude + (10.0 / 111000.0);   // 북쪽 10m
    target.longitude = home.longitude + (10.0 / 88000.0);  // 동쪽 10m
    target.altitude = home.altitude + 5.0;                 // 현재 고도 + 5m

    RCLCPP_INFO(node->get_logger(), "\n[Step 4] Going to waypoint...");
    RCLCPP_INFO(node->get_logger(), "Target: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                target.latitude, target.longitude, target.altitude);

    if (!waypoint_handler.goToWaypoint(target, 60000)) {
        RCLCPP_ERROR(node->get_logger(), "Waypoint navigation failed");

        RCLCPP_INFO(node->get_logger(), "\n[Emergency] Disarming...");
        arm_handler.disarm();
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Waypoint reached!");
    GPSCoordinate current = waypoint_handler.getCurrentPosition();
    RCLCPP_INFO(node->get_logger(), "Final position: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                current.latitude, current.longitude, current.altitude);

    RCLCPP_INFO(node->get_logger(), "\n[Step 5] Hovering for 5 seconds...");
    for (int i = 0; i < 10; i++) {
        takeoff_handler.hover();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(node->get_logger(), "\n[Step 6] Disarming...");
    if (!arm_handler.disarm()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to disarm vehicle");
    } else {
        RCLCPP_INFO(node->get_logger(), "Vehicle disarmed successfully");
    }

    RCLCPP_INFO(node->get_logger(), "\n========================================");
    RCLCPP_INFO(node->get_logger(), "  Test Complete!");
    RCLCPP_INFO(node->get_logger(), "========================================");

    rclcpp::shutdown();
    return 0;
}
