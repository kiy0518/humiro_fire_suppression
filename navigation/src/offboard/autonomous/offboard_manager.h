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
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

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
     * @brief 실내 테스트 미션 실행 (GPS 불필요, Optical Flow 사용)
     * @param takeoff_altitude 이륙 고도 (미터, 기본값 1.0m)
     * @param hover_duration 호버링 시간 (초, 기본값 5초)
     * @return 미션 성공 여부
     */
    bool indoorMission(float takeoff_altitude = 1.0f, float hover_duration = 5.0f);

    /**
     * @brief 실내 테스트 미션 2 - LiDAR 기반 전진/후진 테스트
     * @param takeoff_altitude 이륙 고도 (미터, 기본값 1.0m)
     * @param hover_duration 호버링 시간 (초, 기본값 3초)
     * @param target_distance 목표 전방 거리 (미터, 기본값 1.0m)
     * @param movement_speed 이동 속도 (m/s, 기본값 0.1m/s)
     * @return 미션 성공 여부
     */
    bool indoorMission2(float takeoff_altitude = 1.0f,
                        float hover_duration = 3.0f,
                        float target_distance = 1.0f,
                        float movement_speed = 0.1f);


    /**
     * @brief 실내 미션 3 - 삼각 포메이션 자동 조준 및 격발
     * @param vehicle_id 기체 번호 (1: 리더, 2: 좌측 팔로워, 3: 우측 팔로워)
     * @param takeoff_altitude 이륙 고도 (미터, 기본값 10.0m)
     * @param target_distance 목표물까지 거리 (미터, 기본값 10.0m)
     * @return 미션 성공 여부
     */
    bool indoorMission3(uint8_t vehicle_id,
                        float takeoff_altitude = 10.0f,
                        float target_distance = 10.0f);

    /**
     * @brief 긴급 RTL 실행
     */
    void emergencyRTL();

    /**
     * @brief ArmHandler 반환 (테스트용)
     */
    ArmHandler& getArmHandler() { return *arm_handler_; }

    /**
     * @brief TakeoffHandler 반환 (테스트용)
     */
    TakeoffHandler& getTakeoffHandler() { return *takeoff_handler_; }

    /**
     * @brief RTLHandler 반환 (테스트용)
     */
    RTLHandler& getRTLHandler() { return *rtl_handler_; }

    /**
     * @brief 현재 미션 상태 반환
     */
    MissionState getCurrentState() const { return current_state_; }

    /**
     * @brief 현재 FC 비행 모드 확인 (nav_state)
     * @return nav_state 값 (14 = OFFBOARD)
     */
    uint8_t getCurrentNavState() const;

    /**
     * @brief OFFBOARD 모드인지 확인
     * @return true: OFFBOARD 모드, false: 다른 모드
     */
    bool isOffboardMode() const;

    /**
     * @brief 상태를 IDLE로 리셋 (ERROR 상태 복구용)
     */
    void resetToIdle();

    /**
     * @brief OFFBOARD 모드 비활성화 (heartbeat 중단)
     */
    void disableOffboardMode();

    /**
     * @brief 상태 이름 반환
     */
    static std::string getStateName(MissionState state);

private:
    // ========== GPS 계산 헬퍼 함수 ==========
    /**
     * @brief WGS84 좌표계에서 거리/방위각으로 새 좌표 계산
     * @param origin_lat 기준 위도 (도)
     * @param origin_lon 기준 경도 (도)
     * @param distance_m 거리 (미터)
     * @param bearing_deg 방위각 (도, 0=북쪽, 90=동쪽)
     * @return 새 좌표 (위도, 경도)
     */
    std::pair<double, double> calculateOffsetPosition(
        double origin_lat, double origin_lon,
        float distance_m, float bearing_deg);
    
    /**
     * @brief 두 GPS 좌표 간 거리 계산 (Haversine 공식)
     * @param lat1 첫 번째 위도 (도)
     * @param lon1 첫 번째 경도 (도)
     * @param lat2 두 번째 위도 (도)
     * @param lon2 두 번째 경도 (도)
     * @return 거리 (미터)
     */
    float calculateDistance(double lat1, double lon1, double lat2, double lon2);
    
    /**
     * @brief 두 GPS 좌표 간 방위각 계산
     * @param lat1 시작 위도 (도)
     * @param lon1 시작 경도 (도)
     * @param lat2 목표 위도 (도)
     * @param lon2 목표 경도 (도)
     * @return 방위각 (도, 0~360)
     */
    float calculateBearing(double lat1, double lon1, double lat2, double lon2);
    
    /**
     * @brief 목표물 GPS 좌표 계산 (리더 위치 + 거리 + 헤딩)
     * @param leader_lat 리더 위도 (도)
     * @param leader_lon 리더 경도 (도)
     * @param distance 목표물까지 거리 (미터)
     * @param heading 리더 헤딩 (도)
     * @return 목표물 좌표 (위도, 경도)
     */
    std::pair<double, double> calculateTargetPosition(
        double leader_lat, double leader_lon,
        float distance, float heading);

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

    // ========== testMission3 관련 멤버 변수 ==========
    
    // ROS2 Subscribers (센서 데이터)
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    
    // 현재 상태 데이터
    double current_lat_{0.0};
    double current_lon_{0.0};
    float current_alt_{0.0};
    float current_heading_{0.0};  // 도 단위 (0~360)
    bool gps_valid_{false};
    bool attitude_valid_{false};


    // ========== testMission3 팔로워 관련 ==========
    
    // 리더 정보 구조체 (간단한 버전, ROS2 메시지 대신 사용)
    struct LeaderAimInfo {
        double leader_lat;
        double leader_lon;
        float leader_alt;
        float leader_heading;
        double target_lat;
        double target_lon;
        float distance_to_target;
        bool valid;
        
        LeaderAimInfo() : valid(false) {}
    };
    
    LeaderAimInfo leader_info_;
    bool leader_info_received_{false};
    
    // 팔로워 전용 함수
    bool waitForLeaderInfo(int timeout_sec = 10);
    bool calculateFormationPosition(uint8_t vehicle_id, 
                                    double& my_target_lat, 
                                    double& my_target_lon,
                                    float& my_target_heading);
    bool moveToPosition(double target_lat, double target_lon, 
                       float target_alt, float target_heading,
                       float tolerance_m = 1.0f);
    bool followerDistanceControl(float target_distance, float tolerance);

    
    // Callbacks
    void onGpsPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    
    // testMission3 헬퍼 함수
    bool leaderDistanceControl(float target_distance, float tolerance);
    bool leaderThermalAiming();
    void publishLeaderAimPose(double target_lat, double target_lon);

    MissionConfig mission_config_;
};

#endif // OFFBOARD_MANAGER_H
