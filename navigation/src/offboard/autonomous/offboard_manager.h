#ifndef OFFBOARD_MANAGER_H
#define OFFBOARD_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "arm_handler.h"
#include "takeoff_handler.h"
#include "waypoint_handler.h"
#include "distance_adjuster.h"
#include "rtl_handler.h"
#include <memory>
#include <string>

enum class MissionState {
    IDLE,
    ARMING,
    TAKEOFF,
    NAVIGATE,
    ADJUST_DISTANCE,
    HOVER,
    RTL,
    LANDED,
    ERROR
};

struct MissionConfig {
    float takeoff_altitude = 5.0f;           // 이륙 고도 (미터)
    GPSCoordinate target_waypoint;           // 목표 위치
    float target_distance = 10.0f;           // 목표 거리 (미터)
    float distance_tolerance = 1.0f;         // 거리 허용 오차
    float hover_duration_sec = 5.0f;         // 호버링 시간 (초)
};

class OffboardManager {
public:
    explicit OffboardManager(rclcpp::Node::SharedPtr node);

    /**
     * @brief 미션 실행
     * @param config 미션 설정
     * @return 미션 성공 여부
     */
    bool executeMission(const MissionConfig& config);

    /**
     * @brief 긴급 RTL 실행
     */
    void emergencyRTL();

    /**
     * @brief 현재 미션 상태 반환
     */
    MissionState getCurrentState() const { return current_state_; }

    /**
     * @brief 상태 이름 반환
     */
    static std::string getStateName(MissionState state);

private:
    /**
     * @brief 상태 전환
     */
    bool transitionToState(MissionState new_state);

    /**
     * @brief 에러 처리
     */
    void handleError(const std::string& error_msg);

    rclcpp::Node::SharedPtr node_;

    // Handlers
    std::unique_ptr<ArmHandler> arm_handler_;
    std::unique_ptr<TakeoffHandler> takeoff_handler_;
    std::unique_ptr<WaypointHandler> waypoint_handler_;
    std::unique_ptr<DistanceAdjuster> distance_adjuster_;
    std::unique_ptr<RTLHandler> rtl_handler_;

    // State
    MissionState current_state_{MissionState::IDLE};
    MissionConfig mission_config_;
};

#endif // OFFBOARD_MANAGER_H
