/**
 * @file test_offboard_simple.cpp
 * @brief OFFBOARD 모드 페일세이프 테스트용 최소 코드
 *
 * 목적: OffboardControlMode + TrajectorySetpoint를 10Hz로 지속 발행하여
 *       페일세이프가 발생하지 않는지 확인
 *
 * 수정: FireMissionStart 커스텀 메시지를 받으면 미션 시작
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <custom_message/custom_message.h>
#include <chrono>

using namespace std::chrono_literals;

class SimpleOffboardTest : public rclcpp::Node
{
public:
    SimpleOffboardTest() : Node("simple_offboard_test"), counter_(0), mission_started_(false), mission_start_counter_(0)
    {
        // QoS 설정 (PX4 호환)
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // Publishers
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos);

        // Subscribers
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&SimpleOffboardTest::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&SimpleOffboardTest::vehicle_local_position_callback, this, std::placeholders::_1));

        // 10Hz 타이머 (PX4 요구사항: 2Hz 이상)
        timer_ = this->create_wall_timer(100ms, std::bind(&SimpleOffboardTest::timer_callback, this));

        // CustomMessage 초기화 (MAVLink 14550 포트)
        custom_msg_ = std::make_unique<custom_message::CustomMessage>(14550, 14550, "0.0.0.0", "127.0.0.1", 1, 1);

        // FireMissionStart 콜백 등록
        custom_msg_->setFireMissionStartCallback(
            std::bind(&SimpleOffboardTest::fire_mission_start_callback, this, std::placeholders::_1));

        // CustomMessage 시작
        if (custom_msg_->start()) {
            RCLCPP_INFO(this->get_logger(), "[CustomMessage] 수신 대기 중 (포트: 14550)");
        } else {
            RCLCPP_ERROR(this->get_logger(), "[CustomMessage] 시작 실패!");
        }

        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "  Simple Offboard Test (커스텀 메시지 트리거)");
        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "FireMissionStart 메시지를 기다리는 중...");
        RCLCPP_INFO(this->get_logger(), "메시지 수신 시 미션 시퀀스 시작:");
        RCLCPP_INFO(this->get_logger(), "1. 2초간 setpoint 발행 (준비)");
        RCLCPP_INFO(this->get_logger(), "2. OFFBOARD 모드 전환");
        RCLCPP_INFO(this->get_logger(), "3. ARM (시동)");
        RCLCPP_INFO(this->get_logger(), "4. 3m 이륙 후 10초 호버링");
        RCLCPP_INFO(this->get_logger(), "5. 착륙");
        RCLCPP_INFO(this->get_logger(), "==============================================");
    }

private:
    void timer_callback()
    {
        // ★★★ 핵심: 항상 두 메시지를 함께 발행 ★★★
        publish_offboard_control_mode();
        publish_trajectory_setpoint();

        counter_++;

        // 미션이 시작되지 않았으면 아무것도 하지 않음
        if (!mission_started_) {
            return;
        }

        // 미션 시작 후 카운터 증가
        mission_start_counter_++;

        // 타이밍 (10Hz 기준, 미션 시작 후)
        constexpr uint64_t OFFBOARD_SWITCH = 20;  // 2초
        constexpr uint64_t ARM_COUNT = 40;        // 4초
        constexpr uint64_t LAND_COUNT = 140;      // 14초 (이륙 + 10초 호버링)

        if (mission_start_counter_ == OFFBOARD_SWITCH) {
            RCLCPP_INFO(this->get_logger(), "\n[Step 1] OFFBOARD 모드 전환 요청");
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        } else if (mission_start_counter_ == ARM_COUNT) {
            RCLCPP_INFO(this->get_logger(), "\n[Step 2] ARM (시동) 요청");
            arm();

        } else if (mission_start_counter_ == LAND_COUNT) {
            RCLCPP_INFO(this->get_logger(), "\n[Step 3] 착륙 명령");
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        }

        // 상태 출력 (1초마다)
        if (mission_start_counter_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "[%lds] nav_state=%d %s | armed=%s | alt=%.2fm",
                        mission_start_counter_ / 10,
                        nav_state_,
                        (nav_state_ == 14 ? "OFFBOARD" : (nav_state_ == 18 ? "AUTO_LAND" : "")),
                        (arming_state_ == 2 ? "ON" : "OFF"),
                        -current_z_);
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        // 현재 위치 캡처 (미션 시작 시 한 번만)
        if (mission_started_ && mission_start_counter_ == 1 && position_received_) {
            initial_x_ = current_x_;
            initial_y_ = current_y_;
            initial_z_ = current_z_;
            initial_yaw_ = current_yaw_;
            RCLCPP_INFO(this->get_logger(),
                        "[초기 위치] x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                        initial_x_, initial_y_, initial_z_, initial_yaw_);
        }

        // Setpoint 계산
        float target_x = initial_x_;
        float target_y = initial_y_;
        float target_z = initial_z_;
        float target_yaw = initial_yaw_;

        // ARM 이후: 3m 이륙
        if (mission_start_counter_ > 40 && arming_state_ == 2) {
            target_z = initial_z_ - 3.0f;  // NED: 음수가 위로
        }

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {target_x, target_y, target_z};
        msg.yaw = target_yaw;

        trajectory_setpoint_pub_->publish(msg);
    }

    void fire_mission_start_callback(const custom_message::FireMissionStart& start)
    {
        if (mission_started_) {
            RCLCPP_WARN(this->get_logger(), "[FireMissionStart] 이미 미션이 실행 중입니다. 무시합니다.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║   FireMissionStart 메시지 수신!           ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "[목표 위치] lat=%.7f, lon=%.7f, alt=%.2fm",
                    start.target_lat / 1e7, start.target_lon / 1e7, start.target_alt);
        RCLCPP_INFO(this->get_logger(), "[속도] takeoff=%.1fm/s, flight=%.1fm/s",
                    start.takeoff_speed, start.flight_speed);
        RCLCPP_INFO(this->get_logger(), "[설정] auto_fire=%d, max_projectiles=%d",
                    start.auto_fire, start.max_projectiles);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), ">>> 미션 시작! <<<");
        RCLCPP_INFO(this->get_logger(), "");

        mission_started_ = true;
        mission_start_counter_ = 0;
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        uint8_t old_nav = nav_state_;
        uint8_t old_arm = arming_state_;

        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;

        // 모드 변경 감지
        if (old_nav != nav_state_) {
            RCLCPP_WARN(this->get_logger(),
                        "★ 모드 변경: %d -> %d %s",
                        old_nav, nav_state_,
                        (nav_state_ == 14 ? "(OFFBOARD)" :
                         nav_state_ == 18 ? "(AUTO_LAND - 페일세이프?)" : ""));
        }

        if (old_arm != arming_state_) {
            RCLCPP_INFO(this->get_logger(),
                        "★ 시동 상태: %s -> %s",
                        (old_arm == 2 ? "ON" : "OFF"),
                        (arming_state_ == 2 ? "ON" : "OFF"));
        }
    }

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
        current_yaw_ = msg->heading;
        position_received_ = true;
    }

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // CustomMessage
    std::unique_ptr<custom_message::CustomMessage> custom_msg_;

    // State
    uint64_t counter_;
    uint64_t mission_start_counter_;
    bool mission_started_;
    uint8_t nav_state_ = 0;
    uint8_t arming_state_ = 0;
    float current_x_ = 0.0f;
    float current_y_ = 0.0f;
    float current_z_ = 0.0f;
    float current_yaw_ = 0.0f;
    float initial_x_ = 0.0f;
    float initial_y_ = 0.0f;
    float initial_z_ = 0.0f;
    float initial_yaw_ = 0.0f;
    bool position_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleOffboardTest>());
    rclcpp::shutdown();
    return 0;
}
