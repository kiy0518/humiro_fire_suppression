#ifndef FRAME_COMPOSITOR_H
#define FRAME_COMPOSITOR_H

#include <opencv2/opencv.hpp>
#include "thermal_data.h"
#include "config.h"
#include <vector>

// Forward declaration (lidar_interface.h의 LidarPoint 사용)
struct LidarPoint;

class FrameCompositor {
public:
    FrameCompositor();
    ~FrameCompositor();
    
    cv::Mat composite_frames(const cv::Mat& rgb_frame, const ThermalData& thermal_data);
    
    /**
     * 라이다 데이터 설정 (스레드 안전)
     */
    void setLidarData(const std::vector<LidarPoint>& lidar_points);
    
    /**
     * 라이다 오리엔테이션 설정 (각도 오프셋)
     * @param offset_degrees 라이다 0도 방향의 오프셋 (도 단위)
     *                       예: 90도면 라이다의 0도가 실제 전방에서 90도 회전된 방향
     */
    void setLidarOrientation(float offset_degrees);
    
private:
    void draw_marker(cv::Mat& frame, const ThermalData& data);
    void draw_text_info(cv::Mat& frame, const ThermalData& data);
    void overlay_thermal(cv::Mat& rgb_frame, const cv::Mat& thermal_frame);
    void overlay_logo(cv::Mat& frame);
    void overlay_lidar_radar(cv::Mat& frame);
    cv::Mat create_gradient_mask(int width, int height);
    float apply_curve(float value);
    cv::Scalar getLidarColor(float distance);  // 거리에 따른 색상 반환
    
    cv::Mat logo_image_;  // 로고 이미지 (캐시)
    bool logo_loaded_;    // 로고 로드 여부
    
    // 라이다 데이터 (스레드 안전 접근)
    std::vector<LidarPoint> lidar_points_;
    std::mutex lidar_mutex_;
    
    // 라이다 오리엔테이션 (각도 오프셋)
    float lidar_angle_offset_;  // 라이다 0도 방향의 오프셋 (도 단위)
    std::mutex orientation_mutex_;  // 오프셋 접근 보호
    
    // 라이다 오버레이 설정
    static constexpr int LIDAR_CENTER_X = OUTPUT_WIDTH / 2;
    static constexpr int LIDAR_CENTER_Y = OUTPUT_HEIGHT / 2;
    static constexpr int LIDAR_RADIUS_PIXELS = 200;  // 반경 200픽셀
    static constexpr float LIDAR_MAX_RANGE = 12.0f;  // 12m 최대 거리
    static constexpr float LIDAR_FRONT_ANGLE_RANGE = 60.0f;  // 전방 각도 범위 (±30도)
    static constexpr float LIDAR_CENTER_ANGLE_RANGE = 4.0f;  // 중심 각도 범위 (±2도)
};

#endif // FRAME_COMPOSITOR_H
