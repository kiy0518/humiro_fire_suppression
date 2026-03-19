#ifndef STATUS_OVERLAY_H
#define STATUS_OVERLAY_H

#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>

/**
 * 기체 상태 모니터링 OSD 클래스
 * 영상 왼쪽 하단에 기체 상태 및 정보를 실시간으로 표시
 * 모든 비행 모드에서 동작
 */
class StatusOverlay {
public:
    /**
     * 드론 상태 열거형
     */
    enum class DroneStatus {
        IDLE,                    // 대기
        ARMING,                  // 시동
        TAKEOFF,                 // 이륙
        HOVERING,                // 호버링 (이륙 후 안정화)
        ROTATING,                // 회전중 (목표방향)
        NAVIGATING,              // 이동중
        DISTANCE_ADJUST,         // 거리조정중 (LiDAR 벽 감지 + 10m 확보)
        DESTINATION_REACHED,     // 목적지도착
        TRACKING,                // 조준중 (열원 추적)
        FIRE_READY,              // 격발대기 (수동 격발 대기)
        FIRING,                  // 격발
        FIRING_AUTO_TARGETING,   // 격발중(자동조준) - 수동 격발 모드
        AUTO_FIRING,             // 자동조준격발 (자동 격발 모드)
        MISSION_COMPLETE,        // 임무완료
        RETURNING,               // 복귀중
        LANDING,                 // 착륙
        DISARMED                 // 시동끔
    };
    
    StatusOverlay();
    ~StatusOverlay();
    
    /**
     * PX4 상태 업데이트 (모든 모드에서 사용)
     * @param px4_mode PX4 비행 모드 문자열 (예: "AUTO_MISSION", "OFFBOARD", "MANUAL")
     * @param is_armed 시동 상태
     */
    void updatePx4State(const std::string& px4_mode, bool is_armed);
    
    /**
     * VIM4 자동 제어 시스템 상태 업데이트 (OFFBOARD 모드일 때만 사용)
     * @param status 커스텀 상태
     */
    void updateOffboardStatus(DroneStatus status);
    
    /**
     * 소화탄 갯수 설정
     * @param current 현재 갯수
     * @param max 최대 갯수
     */
    void setAmmunition(int current, int max);
    
    /**
     * 기체 이름 설정
     * @param name 기체 이름 (예: "Drone-01")
     */
    void setDroneName(const std::string& name);
    
    /**
     * WiFi 정보 설정
     * @param ssid 연결된 SSID 이름
     * @param rssi 신호 강도 (dBm, 일반적으로 -30 ~ -90)
     */
    void setWifiInfo(const std::string& ssid, int rssi);
    
    /**
     * 배터리 상태 설정
     * @param percentage 배터리 잔량 (0-100)
     */
    void setBattery(int percentage);
    
    /**
     * GPS 정보 설정
     * @param satellites GPS 위성 수
     * @param hdop HDOP 값 (Horizontal Dilution of Precision)
     */
    void setGpsInfo(int satellites, float hdop);

    /**
     * 고도 설정 (이륙 기준 상대 고도)
     * @param altitude 고도 (m), NED local_z의 부호 반전값
     */
    void setAltitude(float altitude);

    /**
     * 하방 거리 설정 (FC 거리센서)
     * @param distance 지면까지 거리 (m), 무효 시 음수
     * @param valid 센서 유효 여부
     */
    void setDistBottom(float distance, bool valid);

    /**
     * 자세 설정 (roll, pitch)
     * @param roll_deg 롤 각도 (도)
     * @param pitch_deg 피치 각도 (도)
     */
    void setAttitude(float roll_deg, float pitch_deg);

    /**
     * 속도 설정 (NED 좌표계)
     * @param vx 북쪽 속도 (m/s)
     * @param vy 동쪽 속도 (m/s)
     * @param vz 하방 속도 (m/s, 양수=하강)
     */
    void setVelocity(float vx, float vy, float vz);

    /**
     * 최대 온도 설정
     * @param temperature 최대 온도 (섭씨)
     */
    void setMaxTemperature(double temperature);

    /**
     * 온도 기준값 설정 (이 값 이하면 회색 표시)
     * @param threshold 기준 온도 (섭씨)
     */
    void setTempThreshold(double threshold);
    
    /**
     * RTL 서브페이즈 설정 (OSD 표시용)
     * @param sub_phase "DESCEND", "SOFT_LAND", "DISARMING", "" (빈 문자열=표시 안함)
     */
    void setRtlSubPhase(const std::string& sub_phase);

    /**
     * RTL 착지 감지 디버그 정보 설정 (OSD 표시용)
     * @param debug "PX4:N RF:5/30 VZ:12/30 d=0.42" 등
     */
    void setRtlLandDebug(const std::string& debug);

    /**
     * QGC 커스텀 메시지 설정
     * @param message 메시지 텍스트
     * @param timeout_seconds 메시지 표시 시간 (초, 0이면 무제한)
     */
    void setCustomMessage(const std::string& message, double timeout_seconds = 5.0);
    
    /**
     * QGC 커스텀 메시지 지우기
     */
    void clearCustomMessage();
    
    /**
     * 상태 오버레이 그리기
     * @param frame 출력 프레임 (수정됨)
     */
    void draw(cv::Mat& frame);
    
private:
    // PX4 모드를 커스텀 상태로 변환
    DroneStatus convertPx4ModeToStatus(const std::string& px4_mode, bool is_armed);
    
    // 상태 색상 가져오기
    cv::Scalar getStatusColor(DroneStatus status);
    
    // 상태 텍스트 가져오기
    std::string getStatusText(DroneStatus status);
    
    // 배경 그리기 (둥근 모서리)
    void drawBackground(cv::Mat& frame, int x, int y, int width, int height);
    
    // 스레드 안전을 위한 뮤텍스
    std::mutex data_mutex_;
    
    // 상태 정보
    DroneStatus current_status_;
    std::string px4_mode_;       // PX4 비행 모드
    bool is_armed_;              // 시동 상태
    bool is_offboard_;           // OFFBOARD 모드 여부
    bool is_offboard_custom_status_;  // VIM4 커스텀 상태 사용 여부 (OFFBOARD 모드일 때)
    
    // 기타 정보
    int ammo_current_;
    int ammo_max_;
    std::string drone_name_;
    std::string wifi_ssid_;
    int wifi_rssi_;
    bool show_wifi_;
    int battery_percentage_;
    int gps_satellites_;
    float gps_hdop_;  // GPS HDOP 값
    double max_temperature_;  // 최대 온도 (섭씨)
    double temp_threshold_;   // 기준 온도 (이하 회색 표시)
    float altitude_;          // 이륙 기준 상대 고도 (m)
    float dist_bottom_;       // 하방 거리 (m)
    bool dist_bottom_valid_;  // 하방 거리 유효 여부
    float roll_deg_;          // 롤 각도 (도)
    float pitch_deg_;         // 피치 각도 (도)
    bool show_attitude_;      // 자세 표시 여부
    float vel_horizontal_;    // 수평 속도 (m/s)
    float vel_vertical_;      // 수직 속도 (m/s, 양수=상승)
    bool show_velocity_;      // 속도 표시 여부

    // 표시 여부 플래그
    bool show_battery_;
    bool show_gps_;
    bool show_temperature_;  // 온도 표시 여부
    bool show_altitude_;     // 고도 표시 여부
    bool show_dist_bottom_;  // 하방 거리 표시 여부
    
    // RTL 서브페이즈
    std::string rtl_sub_phase_;
    std::string rtl_land_debug_;   // 착지 감지 디버그 정보

    // QGC 커스텀 메시지
    std::string custom_message_;
    double custom_message_timeout_;  // 메시지 만료 시간 (초)
    std::chrono::steady_clock::time_point custom_message_time_;  // 메시지 설정 시간
    bool show_custom_message_;  // 커스텀 메시지 표시 여부
};

#endif // STATUS_OVERLAY_H

