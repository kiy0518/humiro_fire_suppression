#include "offboard_manager.h"
#include <thread>
#include <chrono>

OffboardManager::OffboardManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Handlers 초기화
    arm_handler_ = std::make_unique<ArmHandler>(node);
    takeoff_handler_ = std::make_unique<TakeoffHandler>(node);
    waypoint_handler_ = std::make_unique<WaypointHandler>(node);
    distance_adjuster_ = std::make_unique<DistanceAdjuster>(node);
    rtl_handler_ = std::make_unique<RTLHandler>(node);

    RCLCPP_INFO(node_->get_logger(), "OffboardManager initialized");
}

bool OffboardManager::executeMission(const MissionConfig& config)
{
    mission_config_ = config;

    RCLCPP_INFO(node_->get_logger(), "======================================");
    RCLCPP_INFO(node_->get_logger(), "  Starting Autonomous Mission");
    RCLCPP_INFO(node_->get_logger(), "======================================");
    RCLCPP_INFO(node_->get_logger(), "Mission config:");
    RCLCPP_INFO(node_->get_logger(), "  Takeoff altitude: %.2f m", config.takeoff_altitude);
    RCLCPP_INFO(node_->get_logger(), "  Target waypoint: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
                config.target_waypoint.latitude,
                config.target_waypoint.longitude,
                config.target_waypoint.altitude);
    RCLCPP_INFO(node_->get_logger(), "  Target distance: %.2f m (±%.2f m)",
                config.target_distance, config.distance_tolerance);
    RCLCPP_INFO(node_->get_logger(), "======================================\n");

    // === State: ARMING ===
    if (!transitionToState(MissionState::ARMING)) {
        handleError("Failed to transition to ARMING state");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[ARMING] Enabling OFFBOARD mode...");
    if (!arm_handler_->enableOffboardMode()) {
        handleError("Failed to enable OFFBOARD mode");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node_->get_logger(), "[ARMING] Arming vehicle...");
    if (!arm_handler_->arm()) {
        handleError("Failed to arm vehicle");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[ARMING] Vehicle armed successfully\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // === State: TAKEOFF ===
    if (!transitionToState(MissionState::TAKEOFF)) {
        handleError("Failed to transition to TAKEOFF state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[TAKEOFF] Taking off to %.2f m...", config.takeoff_altitude);
    if (!takeoff_handler_->takeoff(config.takeoff_altitude)) {
        handleError("Takeoff failed");
        emergencyRTL();
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[TAKEOFF] Takeoff successful! Altitude: %.2f m\n",
                takeoff_handler_->getCurrentAltitude());
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // === State: NAVIGATE ===
    if (!transitionToState(MissionState::NAVIGATE)) {
        handleError("Failed to transition to NAVIGATE state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[NAVIGATE] Navigating to waypoint...");
    if (!waypoint_handler_->goToWaypoint(config.target_waypoint)) {
        handleError("Waypoint navigation failed");
        emergencyRTL();
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[NAVIGATE] Waypoint reached!\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // === State: ADJUST_DISTANCE ===
    if (!transitionToState(MissionState::ADJUST_DISTANCE)) {
        handleError("Failed to transition to ADJUST_DISTANCE state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[ADJUST_DISTANCE] Adjusting distance to %.2f m...",
                config.target_distance);
    if (!distance_adjuster_->adjustDistance(config.target_distance, config.distance_tolerance)) {
        handleError("Distance adjustment failed");
        emergencyRTL();
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[ADJUST_DISTANCE] Distance adjusted! Front distance: %.2f m\n",
                distance_adjuster_->getFrontDistance());
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // === State: HOVER ===
    if (!transitionToState(MissionState::HOVER)) {
        handleError("Failed to transition to HOVER state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[HOVER] Hovering for %.1f seconds...", config.hover_duration_sec);
    int hover_iterations = static_cast<int>(config.hover_duration_sec * 2);  // 2Hz
    for (int i = 0; i < hover_iterations; i++) {
        distance_adjuster_->hover();
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (i % 4 == 0) {  // 2초마다 로깅
            RCLCPP_INFO(node_->get_logger(), "[HOVER] Distance: %.2f m",
                       distance_adjuster_->getFrontDistance());
        }
    }
    RCLCPP_INFO(node_->get_logger(), "[HOVER] Hover complete\n");

    // === State: RTL ===
    if (!transitionToState(MissionState::RTL)) {
        handleError("Failed to transition to RTL state");
        emergencyRTL();
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[RTL] Returning to launch...");
    if (!rtl_handler_->returnToLaunch()) {
        handleError("RTL failed");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[RTL] RTL complete! Vehicle landed\n");

    // === State: LANDED ===
    transitionToState(MissionState::LANDED);

    RCLCPP_INFO(node_->get_logger(), "======================================");
    RCLCPP_INFO(node_->get_logger(), "  Mission Complete!");
    RCLCPP_INFO(node_->get_logger(), "======================================");

    return true;
}

void OffboardManager::emergencyRTL()
{
    RCLCPP_ERROR(node_->get_logger(), "\n!!! EMERGENCY RTL TRIGGERED !!!\n");

    transitionToState(MissionState::RTL);

    RCLCPP_INFO(node_->get_logger(), "[EMERGENCY] Executing RTL...");
    if (!rtl_handler_->returnToLaunch(120000)) {
        RCLCPP_ERROR(node_->get_logger(), "[EMERGENCY] RTL failed!");
        transitionToState(MissionState::ERROR);
    } else {
        RCLCPP_INFO(node_->get_logger(), "[EMERGENCY] RTL complete");
        transitionToState(MissionState::LANDED);
    }
}

bool OffboardManager::transitionToState(MissionState new_state)
{
    std::string old_state_name = getStateName(current_state_);
    std::string new_state_name = getStateName(new_state);

    RCLCPP_INFO(node_->get_logger(), "State transition: %s -> %s",
                old_state_name.c_str(), new_state_name.c_str());

    current_state_ = new_state;
    return true;
}

void OffboardManager::handleError(const std::string& error_msg)
{
    RCLCPP_ERROR(node_->get_logger(), "ERROR: %s", error_msg.c_str());
    transitionToState(MissionState::ERROR);
}

std::string OffboardManager::getStateName(MissionState state)
{
    switch (state) {
        case MissionState::IDLE:            return "IDLE";
        case MissionState::ARMING:          return "ARMING";
        case MissionState::TAKEOFF:         return "TAKEOFF";
        case MissionState::NAVIGATE:        return "NAVIGATE";
        case MissionState::ADJUST_DISTANCE: return "ADJUST_DISTANCE";
        case MissionState::HOVER:           return "HOVER";
        case MissionState::RTL:             return "RTL";
        case MissionState::LANDED:          return "LANDED";
        case MissionState::ERROR:           return "ERROR";
        default:                            return "UNKNOWN";
    }
}
