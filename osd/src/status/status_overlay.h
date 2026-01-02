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
        NAVIGATING,              // 이동중
        DESTINATION_REACHED,     // 목적지도착
        FIRE_READY,              // 격발대기 (수동 격발 대기)
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
     * 편대 정보 설정
     * @param current 현재 편대 번호
     * @param total 전체 편대 수
     */
    void setFormation(int current, int total);
    
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
     * 최대 온도 설정
     * @param temperature 최대 온도 (섭씨)
     */
    void setMaxTemperature(double temperature);
    
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
    int formation_current_;
    int formation_total_;
    int battery_percentage_;
    int gps_satellites_;
    float gps_hdop_;  // GPS HDOP 값
    double max_temperature_;  // 최대 온도 (섭씨)
    
    // 표시 여부 플래그
    bool show_battery_;
    bool show_gps_;
    bool show_temperature_;  // 온도 표시 여부
};

#endif // STATUS_OVERLAY_H

