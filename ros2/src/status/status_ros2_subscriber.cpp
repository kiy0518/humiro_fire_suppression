#ifdef ENABLE_ROS2

#include "status_ros2_subscriber.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <map>

// px4_msgs가 없으면 기본 구조체 사용
#ifndef px4_msgs_FOUND
namespace px4_msgs {
namespace msg {
struct VehicleStatus {
    uint8_t nav_state;
    bool arming_state;
    bool failsafe;
    uint8_t vehicle_type;
    uint8_t system_type;
};
struct BatteryStatus {
    float voltage_v;
    float current_a;
    float remaining;  // 0.0 ~ 1.0
    uint8_t battery_warning;
};
struct VehicleGpsPosition {
    int32_t lat;
    int32_t lon;
    float alt;
    uint8_t satellites_used;
    uint8_t fix_type;
};
}
}
#endif

StatusROS2Subscriber::StatusROS2Subscriber(rclcpp::Node::SharedPtr node, StatusOverlay* status_overlay)
    : node_(node)
    , status_overlay_(status_overlay)
    , last_state_update_(std::chrono::steady_clock::now())
    , last_battery_update_(std::chrono::steady_clock::now())
    , last_gps_update_(std::chrono::steady_clock::now())
{
    if (!node_ || !status_overlay_) {
        std::cerr << "  ✗ StatusROS2Subscriber: node 또는 status_overlay가 nullptr입니다" << std::endl;
        return;
    }
    
    std::cout << "  → ROS2 노드 이름: " << node_->get_name() << std::endl;
    std::cout << "  → ROS2 네임스페이스: " << node_->get_namespace() << std::endl;
    
    // PX4 uXRCE-DDS QoS 설정
    // PX4는 일반적으로 BEST_EFFORT를 사용하므로 구독자도 BEST_EFFORT로 설정
    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    std::cout << "  [DEBUG] QoS 설정: Reliability=BestEffort, Durability=Volatile, Depth=10" << std::endl;
    
    // PX4 상태 구독 (uXRCE-DDS: /fmu/out/vehicle_status_v1)
    // PX4 v1.14+에서는 vehicle_status_v1 토픽 사용
    try {
        vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", px4_qos,
            std::bind(&StatusROS2Subscriber::vehicleStatusCallback, this, std::placeholders::_1));
        std::cout << "  ✓ ROS2 구독: /fmu/out/vehicle_status_v1 (uXRCE-DDS, BestEffort QoS)" << std::endl;
        std::cout << "    → QGC에서 비행 모드 변경 시 즉시 반영됩니다" << std::endl;
        std::cout << "    ⚠ 토픽 수신 대기 중... (PX4가 연결되면 자동으로 수신됩니다)" << std::endl;
        std::cout << "    [DEBUG] 구독자 생성 완료: vehicle_status_sub_=" << (vehicle_status_sub_ ? "OK" : "NULL") << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "  ✗ 구독자 생성 실패: " << e.what() << std::endl;
    }
    
    // 배터리 상태 구독 (uXRCE-DDS: /fmu/out/battery_status)
    battery_sub_ = node_->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status", px4_qos,
        std::bind(&StatusROS2Subscriber::batteryCallback, this, std::placeholders::_1));
    std::cout << "  ✓ ROS2 구독: /fmu/out/battery_status (uXRCE-DDS, BestEffort QoS)" << std::endl;
    
    // GPS 상태 구독 (uXRCE-DDS: /fmu/out/vehicle_gps_position)
    // 실제 메시지 타입은 px4_msgs/msg/SensorGps
    gps_sub_ = node_->create_subscription<px4_msgs::msg::SensorGps>(
        "/fmu/out/vehicle_gps_position", px4_qos,
        std::bind(&StatusROS2Subscriber::gpsCallback, this, std::placeholders::_1));
    std::cout << "  ✓ ROS2 구독: /fmu/out/vehicle_gps_position (uXRCE-DDS, SensorGps, BestEffort QoS)" << std::endl;
    
    // OFFBOARD 모드 상태 구독 (/offboard/status) - VIM4 커스텀 토픽
    offboard_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/offboard/status", 10,
        std::bind(&StatusROS2Subscriber::offboardStatusCallback, this, std::placeholders::_1));
    std::cout << "  ✓ ROS2 구독: /offboard/status (OFFBOARD 모드 상태)" << std::endl;
    
    // 소화탄 갯수 구독 (/ammunition/current) - 커스텀 토픽
    ammunition_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/ammunition/current", 10,
        std::bind(&StatusROS2Subscriber::ammunitionCallback, this, std::placeholders::_1));
    std::cout << "  ✓ ROS2 구독: /ammunition/current" << std::endl;
    
    // 편대 정보 구독 (/formation/current) - 커스텀 토픽
    formation_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/formation/current", 10,
        std::bind(&StatusROS2Subscriber::formationCallback, this, std::placeholders::_1));
    std::cout << "  ✓ ROS2 구독: /formation/current" << std::endl;
}

StatusROS2Subscriber::~StatusROS2Subscriber() {
    // 구독자들은 자동으로 해제됨
}

void StatusROS2Subscriber::spin() {
    if (node_) {
        // rclcpp::spin_some은 같은 노드에 대해 여러 스레드에서 동시 호출 시 문제 발생
        // 단일 스레드에서만 호출하도록 주의
        // timeout을 0으로 설정하여 즉시 반환 (non-blocking)
        rclcpp::spin_some(node_);
        
        // 디버깅: 주기적으로 spin 호출 확인 (10초마다)
        static auto last_spin_log = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_spin_log).count();
        if (elapsed >= 10) {
            static int spin_count = 0;
            spin_count++;
            std::cout << "  [DEBUG] StatusROS2Subscriber::spin() 호출됨 (총 " << spin_count << "회)" << std::endl;
            last_spin_log = now;
        }
    } else {
        static bool warned = false;
        if (!warned) {
            std::cerr << "  ⚠ StatusROS2Subscriber::spin(): node_가 nullptr입니다" << std::endl;
            warned = true;
        }
    }
}

// PX4 nav_state를 모드 문자열로 변환
std::string StatusROS2Subscriber::navStateToModeString(uint8_t nav_state) {
    // PX4 navigation state enum (px4_msgs/VehicleStatus)
    // QGC 표시 이름과 일치하도록 수정
    static std::map<uint8_t, std::string> nav_state_map = {
        {0, "MANUAL"},
        {1, "Altitude"},           // ALTCTL → QGC: "Altitude"
        {2, "Position"},           // POSCTL → QGC: "Position"
        {3, "Mission"},            // AUTO_MISSION → QGC: "Mission"
        {4, "Hold"},               // AUTO_LOITER → QGC: "Hold"
        {5, "AUTO_RTL"},
        {6, "Position Slow"},      // POSITION_SLOW → QGC: "Position Slow"
        {7, "FREE5"},
        {8, "ALTITUDE_CRUISE"},
        {9, "FREE3"},
        {10, "Acro"},              // ACRO → QGC: "Acro"
        {11, "FREE2"},
        {12, "DESCEND"},
        {13, "TERMINATION"},
        {14, "OFFBOARD"},
        {15, "Stabilized"},        // STAB → QGC: "Stabilized"
        {16, "FREE1"},
        {17, "AUTO_TAKEOFF"},
        {18, "AUTO_LAND"},
        {19, "Follow Target"},     // AUTO_FOLLOW_TARGET → QGC: "Follow Target"
        {20, "Precision Landing"}, // AUTO_PRECLAND → QGC: "Precision Landing"
        {21, "ORBIT"},
        {22, "VTOL Takeoff"},      // AUTO_VTOL_TAKEOFF → QGC: "VTOL Takeoff"
        {23, "EXTERNAL1"},
        {24, "EXTERNAL2"},
    };
    
    auto it = nav_state_map.find(nav_state);
    if (it != nav_state_map.end()) {
        return it->second;
    }
    return "UNKNOWN";
}

void StatusROS2Subscriber::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    // 디버깅: 콜백 호출 확인 (항상 출력)
    static int callback_count = 0;
    callback_count++;
    
    if (!status_overlay_) {
        std::cerr << "  ⚠ vehicleStatusCallback: status_overlay_가 nullptr입니다" << std::endl;
        return;
    }
    
    if (!msg) {
        std::cerr << "  ⚠ vehicleStatusCallback: msg가 nullptr입니다" << std::endl;
        return;
    }
    
    // 첫 수신 확인 (토픽 연결 확인)
    static bool first_received = false;
    if (!first_received) {
        std::cout << "  ✓ [토픽 수신 확인] /fmu/out/vehicle_status_v1 첫 메시지 수신!" << std::endl;
        std::cout << "    → PX4 uXRCE-DDS 연결 성공" << std::endl;
        std::cout << "    [DEBUG] 첫 메시지 nav_state=" << (int)msg->nav_state << ", armed=" << (msg->arming_state ? "ON" : "OFF") << std::endl;
        first_received = true;
    }
    
    // PX4 nav_state를 모드 문자열로 변환
    std::string mode = navStateToModeString(msg->nav_state);
    bool is_armed = msg->arming_state;
    
    // 상태 변경 감지 (이전 값과 비교)
    static std::string last_mode = "";
    static bool last_armed = false;
    bool mode_changed = (mode != last_mode);
    bool armed_changed = (is_armed != last_armed);
    
    // 디버깅: 모든 메시지 수신 시 출력 (변경 여부와 관계없이)
    static int call_count = 0;
    call_count++;
    if (call_count % 10 == 0 || mode_changed || armed_changed) {
        std::cout << "  [DEBUG] vehicleStatusCallback 호출됨 (nav_state=" << (int)msg->nav_state 
                  << " → mode=\"" << mode << "\", armed=" << (is_armed ? "ON" : "OFF") << ")" << std::endl;
    }
    
    // StatusOverlay 업데이트 (상태바에 즉시 반영)
    status_overlay_->updatePx4State(mode, is_armed);
    
    // 상태 변경 시 즉시 출력 (QGC에서 변경 시 바로 확인 가능)
    if (mode_changed || armed_changed) {
        std::cout << "  [비행 모드 변경] " << last_mode << " → " << mode 
                  << " (nav_state=" << (int)msg->nav_state << ")"
                  << ", 시동: " << (last_armed ? "ON" : "OFF") 
                  << " → " << (is_armed ? "ON" : "OFF") << std::endl;
        std::cout << "    → StatusOverlay::updatePx4State() 호출 완료" << std::endl;
        last_mode = mode;
        last_armed = is_armed;
        last_state_update_ = std::chrono::steady_clock::now();
    }
    // 주기적으로 출력 (10초마다, 변경이 없을 때)
    else {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_state_update_).count();
        if (elapsed >= 10) {
            std::cout << "  [상태 확인] 모드: " << mode << " (nav_state=" << (int)msg->nav_state 
                      << "), 시동: " << (is_armed ? "ON" : "OFF") << std::endl;
            last_state_update_ = now;
        }
    }
}

void StatusROS2Subscriber::batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
    if (!status_overlay_) {
        return;
    }
    
    // 배터리 퍼센트 계산 (remaining은 0.0 ~ 1.0)
    int percentage = static_cast<int>(msg->remaining * 100.0);
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;
    
    // StatusOverlay 업데이트 (상태바에 즉시 반영)
    status_overlay_->setBattery(percentage);
    
    // 디버깅: 주기적으로 출력 (10초마다)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_battery_update_).count();
    if (elapsed >= 10) {
        std::cout << "  [배터리 업데이트] " << percentage << "% (voltage: " << msg->voltage_v 
                  << "V, current: " << msg->current_a << "A)" << std::endl;
        last_battery_update_ = now;
    }
}

void StatusROS2Subscriber::gpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg) {
    if (!status_overlay_) {
        return;
    }
    
    // GPS 위성 수 가져오기 (SensorGps 메시지 필드 확인 필요)
    // SensorGps는 satellites_used 필드를 사용
    int satellites = static_cast<int>(msg->satellites_used);
    if (satellites < 0) satellites = 0;
    if (satellites > 20) satellites = 20;  // 최대값 제한
    
    // StatusOverlay 업데이트 (상태바에 즉시 반영)
    status_overlay_->setGpsSatellites(satellites);
    
    // 디버깅: 주기적으로 출력 (10초마다)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_gps_update_).count();
    if (elapsed >= 10) {
        // GPS 상태 출력 (SensorGps 필드 확인 필요)
        double lat = static_cast<double>(msg->latitude_deg);
        double lon = static_cast<double>(msg->longitude_deg);
        std::cout << "  [GPS 업데이트] 위도: " << lat << ", 경도: " << lon 
                  << ", 위성: " << satellites << ", fix_type: " << (int)msg->fix_type << std::endl;
        last_gps_update_ = now;
    }
}

void StatusROS2Subscriber::offboardStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!status_overlay_) {
        return;
    }
    
    // 문자열을 DroneStatus로 변환
    std::string status_str = msg->data;
    
    // 문자열을 enum으로 변환
    StatusOverlay::DroneStatus status = StatusOverlay::DroneStatus::IDLE;
    
    if (status_str == "IDLE") {
        status = StatusOverlay::DroneStatus::IDLE;
    } else if (status_str == "ARMING") {
        status = StatusOverlay::DroneStatus::ARMING;
    } else if (status_str == "TAKEOFF") {
        status = StatusOverlay::DroneStatus::TAKEOFF;
    } else if (status_str == "NAVIGATING") {
        status = StatusOverlay::DroneStatus::NAVIGATING;
    } else if (status_str == "DESTINATION_REACHED") {
        status = StatusOverlay::DroneStatus::DESTINATION_REACHED;
    } else if (status_str == "FIRE_READY") {
        status = StatusOverlay::DroneStatus::FIRE_READY;
    } else if (status_str == "FIRING_AUTO_TARGETING") {
        status = StatusOverlay::DroneStatus::FIRING_AUTO_TARGETING;
    } else if (status_str == "AUTO_FIRING") {
        status = StatusOverlay::DroneStatus::AUTO_FIRING;
    } else if (status_str == "MISSION_COMPLETE") {
        status = StatusOverlay::DroneStatus::MISSION_COMPLETE;
    } else if (status_str == "RETURNING") {
        status = StatusOverlay::DroneStatus::RETURNING;
    } else if (status_str == "LANDING") {
        status = StatusOverlay::DroneStatus::LANDING;
    } else if (status_str == "DISARMED") {
        status = StatusOverlay::DroneStatus::DISARMED;
    }
    
    // StatusOverlay 업데이트
    status_overlay_->updateOffboardStatus(status);
    
    // 디버깅: 상태 변경 시 출력
    static std::string last_status = "";
    if (status_str != last_status) {
        std::cout << "  [OFFBOARD 모드] 상태: " << status_str << std::endl;
        last_status = status_str;
    }
}

void StatusROS2Subscriber::ammunitionCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!status_overlay_) {
        return;
    }
    
    // 현재 소화탄 갯수 업데이트 (최대값은 설정에서 유지)
    int current = msg->data;
    if (current < 0) current = 0;
    
    // StatusOverlay 업데이트 (상태바에 즉시 반영)
    // setAmmunition은 current와 max를 모두 받으므로, 현재 max 값을 유지해야 함
    // 하지만 현재 max 값을 가져올 수 없으므로, 일단 기본값 6을 사용
    // TODO: 최대값도 토픽으로 받거나, 별도 메서드 추가 필요
    status_overlay_->setAmmunition(current, 6);  // 기본 최대값 6
    
    // 디버깅: 값 변경 시 출력
    static int last_ammo = -1;
    if (current != last_ammo) {
        std::cout << "  [소화탄 업데이트] " << current << "/6발" << std::endl;
        last_ammo = current;
    }
}

void StatusROS2Subscriber::formationCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!status_overlay_) {
        return;
    }
    
    // 편대 번호 업데이트 (전체 편대 수는 설정에서 유지)
    int current = msg->data;
    if (current < 1) current = 1;
    
    // StatusOverlay 업데이트 (상태바에 즉시 반영)
    // setFormation은 current와 total을 모두 받으므로, 현재 total 값을 유지해야 함
    // 하지만 현재 total 값을 가져올 수 없으므로, 일단 기본값 3을 사용
    // TODO: 전체 편대 수도 토픽으로 받거나, 별도 메서드 추가 필요
    status_overlay_->setFormation(current, 3);  // 기본 전체 편대 수 3
    
    // 디버깅: 값 변경 시 출력
    static int last_formation = -1;
    if (current != last_formation) {
        std::cout << "  [편대 업데이트] " << current << "/3" << std::endl;
        last_formation = current;
    }
}

#endif // ENABLE_ROS2

