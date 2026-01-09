#ifndef RTL_HANDLER_H
#define RTL_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <atomic>
#include <chrono>
#include <memory>

class RTLHandler {
public:
    explicit RTLHandler(rclcpp::Node::SharedPtr node);

    /**
     * @brief RTL (Return To Launch) 모드 활성화
     * RTL 모드는 이륙 위치로 자동 복귀 후 착륙
     * @param timeout_ms 타임아웃 (밀리초, 기본값 120초)
     * @return RTL 성공 여부
     */
    bool returnToLaunch(int timeout_ms = 120000);

    /**
     * @brief 착륙 명령 실행
     * @param timeout_ms 타임아웃 (밀리초, 기본값 30초)
     * @return 착륙 성공 여부
     */
    bool land(int timeout_ms = 30000);

    /**
     * @brief Home으로부터의 거리 반환
     * @return 거리 (미터)
     */
    double getDistanceFromHome() const;

    /**
     * @brief 착륙 완료 여부 확인
     * @return 착륙 완료 여부
     */
    bool isLanded() const;

    /**
     * @brief RTL 모드 활성화 여부 확인
     * @return RTL 모드 여부
     */
    bool isRTLActive() const;

private:
    /**
     * @brief VehicleStatus 콜백
     */
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    /**
     * @brief VehicleGlobalPosition 콜백
     */
    void vehicleGlobalPositionCallback(
        const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

    /**
     * @brief VehicleLocalPosition 콜백
     */
    void vehicleLocalPositionCallback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    /**
     * @brief Vehicle Command 발행
     */
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f,
                               float param2 = 0.0f, float param3 = 0.0f,
                               float param4 = 0.0f, float param5 = 0.0f,
                               float param6 = 0.0f, float param7 = 0.0f);

    /**
     * @brief Haversine 거리 계산
     */
    double haversineDistance(double lat1, double lon1,
                            double lat2, double lon2) const;

    rclcpp::Node::SharedPtr node_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    // State variables
    std::atomic<uint8_t> nav_state_{0};
    std::atomic<uint8_t> arming_state_{0};
    std::atomic<bool> landed_{false};

    // Position variables
    std::atomic<double> current_latitude_{0.0};
    std::atomic<double> current_longitude_{0.0};
    std::atomic<float> current_altitude_amsl_{0.0f};
    std::atomic<float> current_local_z_{0.0f};

    double home_latitude_{0.0};
    double home_longitude_{0.0};
    float home_altitude_amsl_{0.0f};

    std::atomic<bool> global_position_received_{false};
    std::atomic<bool> home_position_set_{false};

    // FC 시스템 ID (DRONE_ID 환경 변수에서 읽음)
    uint8_t target_system_;

    // Constants
    static constexpr uint16_t VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20;
    static constexpr uint16_t VEHICLE_CMD_NAV_LAND = 21;
    static constexpr uint8_t NAV_STATE_AUTO_RTL = 5;  // PX4 RTL mode
    static constexpr uint8_t ARMING_STATE_STANDBY = 2;  // Disarmed
    static constexpr double EARTH_RADIUS = 6371000.0;
    static constexpr float DEG_TO_RAD = M_PI / 180.0;
    static constexpr float HOME_THRESHOLD = 5.0f;  // 5m 이내면 Home 도달
    static constexpr float LANDED_THRESHOLD = 0.3f;  // 지상 30cm 이내
};

#endif // RTL_HANDLER_H
