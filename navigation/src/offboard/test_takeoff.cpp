#include <rclcpp/rclcpp.hpp>
#include "autonomous/arm_handler.h"
#include "autonomous/takeoff_handler.h"
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_takeoff_node");

    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "  Takeoff Test Program");
    RCLCPP_INFO(node->get_logger(), "========================================");

    // Handlers 생성
    ArmHandler arm_handler(node);
    TakeoffHandler takeoff_handler(node);

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

    RCLCPP_INFO(node->get_logger(), "Takeoff successful!");
    RCLCPP_INFO(node->get_logger(), "Current altitude: %.2f meters",
                takeoff_handler.getCurrentAltitude());

    RCLCPP_INFO(node->get_logger(), "\n[Step 4] Hovering for 10 seconds...");
    for (int i = 0; i < 20; i++) {  // 10초 동안 호버링
        takeoff_handler.hover();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (i % 4 == 0) {  // 2초마다 로깅
            RCLCPP_INFO(node->get_logger(), "Hovering... Altitude: %.2f m",
                        takeoff_handler.getCurrentAltitude());
        }
    }

    RCLCPP_INFO(node->get_logger(), "\n[Step 5] Disarming...");
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
