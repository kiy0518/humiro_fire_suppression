#ifndef STATUS_ROS2_SUBSCRIBER_H
#define STATUS_ROS2_SUBSCRIBER_H

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "../../../osd/src/status/status_overlay.h"
#include "../../../navigation/src/offboard/bridge/fc_bridge_protocol.h"
#include <memory>
#include <string>
#include <chrono>
#include <functional>

// Forward declaration
class FCBridgeClient;

/**
 * StatusOverlay를 위한 ROS2 토픽 구독자
 *
 * v2.0: DDS Domain 분리
 * - FC 상태(nav_state, battery, GPS)는 FCBridgeClient에서 10Hz 폴링
 * - px4_msgs 의존성 제거 (더 이상 /fmu/* 토픽 직접 구독 안 함)
 * - 커스텀 VIM4 토픽(/offboard/status, /ammunition, /formation)만 ROS2로 구독
 */
class StatusROS2Subscriber {
public:
    // 모드 변경 콜백 타입 (old_nav_state, new_nav_state)
    using ModeChangeCallback = std::function<void(uint8_t, uint8_t)>;

    /**
     * @param node ROS2 노드 (Domain 1)
     * @param status_overlay OSD 오버레이
     * @param fc_bridge FCBridgeClient 포인터 (OffboardManager에서 공유)
     */
    StatusROS2Subscriber(rclcpp::Node::SharedPtr node,
                         StatusOverlay* status_overlay,
                         FCBridgeClient* fc_bridge);
    ~StatusROS2Subscriber();

    /**
     * ROS2 스핀 (주기적으로 호출)
     */
    void spin();

    /**
     * 모드 변경 콜백 설정 (OFFBOARD → 다른 모드 전환 시 호출됨)
     */
    void setModeChangeCallback(ModeChangeCallback callback);

    // Vehicle status 업데이트 콜백 (nav_state, arming_state)
    using VehicleStatusCallback = std::function<void(uint8_t, uint8_t)>;
    void setVehicleStatusCallback(VehicleStatusCallback callback) { vehicle_status_callback_ = callback; }

    // ========== 미션 사전 조건 체크용 getter ==========
    /** @brief FC 연결 상태 (FCBridgeClient 연결 + fc_connected 플래그) */
    bool isFCConnected() const;
    /** @brief GPS fix 상태 (fix_type >= 3: 3D Fix) */
    bool isGPSFixed() const;
    /** @brief GPS fix type 반환 */
    uint8_t getGPSFixType() const { return gps_fix_type_.load(); }
    /** @brief 배터리 잔량 (0.0~1.0) */
    float getBatteryRemaining() const { return battery_remaining_.load(); }
    /** @brief 현재 nav_state */
    uint8_t getNavState() const { return current_nav_state_.load(); }
    /** @brief 현재 arming_state */
    uint8_t getArmingState() const { return current_arming_state_.load(); }

private:
    // FC 상태 폴링 (FCBridgeClient에서 10Hz)
    void pollFCState();

    // OFFBOARD 모드 상태 콜백 (VIM4 커스텀 토픽: /offboard/status)
    void offboardStatusCallback(const std_msgs::msg::String::SharedPtr msg);

    // 소화탄 갯수 콜백 (커스텀 토픽)
    void ammunitionCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // 편대 정보 콜백 (커스텀 토픽)
    void formationCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // PX4 nav_state를 모드 문자열로 변환
    std::string navStateToModeString(uint8_t nav_state);

    rclcpp::Node::SharedPtr node_;
    StatusOverlay* status_overlay_;
    FCBridgeClient* fc_bridge_;

    // FC 상태 폴링 타이머 (10Hz)
    rclcpp::TimerBase::SharedPtr fc_poll_timer_;

    // ROS2 구독자 (Domain 1 커스텀 토픽만)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr offboard_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ammunition_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr formation_sub_;

    // 마지막 업데이트 시간 (디버깅용)
    std::chrono::steady_clock::time_point last_state_update_;
    std::chrono::steady_clock::time_point last_battery_update_;
    std::chrono::steady_clock::time_point last_gps_update_;

    // 모드 변경 콜백 (OFFBOARD → 다른 모드 전환 감지용)
    ModeChangeCallback mode_change_callback_;
    VehicleStatusCallback vehicle_status_callback_;
    uint8_t last_nav_state_ = 255;  // 이전 nav_state 저장

    // 미션 사전 조건 체크용 상태 변수 (atomic)
    std::atomic<uint8_t> current_nav_state_{0};
    std::atomic<uint8_t> current_arming_state_{0};
    std::atomic<uint8_t> gps_fix_type_{0};
    std::atomic<float> battery_remaining_{0.0f};

    // OSD 확정 상태 (변경 감지용)
    std::string confirmed_mode_;
    bool confirmed_armed_{false};
};

#endif // STATUS_ROS2_SUBSCRIBER_H

#endif // ENABLE_ROS2
