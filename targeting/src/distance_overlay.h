#ifndef DISTANCE_OVERLAY_H
#define DISTANCE_OVERLAY_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>

// Forward declaration
struct LidarPoint;

/**
 * 거리 오버레이 클래스
 * 라이다 데이터를 기반으로 거리 정보를 화면에 표시
 */
class DistanceOverlay {
public:
    DistanceOverlay();
    ~DistanceOverlay();
    
    /**
     * 라이다 데이터 설정 (스레드 안전)
     */
    void setLidarData(const std::vector<LidarPoint>& lidar_points);
    
    /**
     * 라이다 오리엔테이션 설정 (각도 오프셋)
     */
    void setLidarOrientation(float offset_degrees);
    
    /**
     * 거리 오버레이 그리기
     * @param frame 출력 프레임
     */
    void drawOverlay(cv::Mat& frame);
    
    /**
     * 설정 업데이트
     */
    void setDisplayMode(const std::string& mode);  // "FULL_360", "FRONT_3DIR", "THREE_POINTS"
    void setShowDirectionLines(bool show);
    void setThreePointTolerance(float tolerance);
    
private:
    cv::Scalar getLidarColor(float distance);
    void drawDistanceRings(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float max_range);
    void drawDirectionLines(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float angle_offset, const std::string& display_mode);
    void drawCenterDistanceText(cv::Mat& frame, int center_x, const std::vector<LidarPoint>& points, float angle_offset);
    void drawThreePoints(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float max_range, const std::vector<LidarPoint>& points, float angle_offset);
    void drawAllPoints(cv::Mat& frame, int center_x, int center_y, int radius_pixels, float max_range, const std::vector<LidarPoint>& points, float angle_offset, const std::string& display_mode);
    
    // 라이다 데이터 (스레드 안전 접근)
    std::vector<LidarPoint> lidar_points_;
    std::mutex lidar_mutex_;
    
    // 라이다 오리엔테이션 (각도 오프셋)
    float lidar_angle_offset_;
    std::mutex orientation_mutex_;
    
    // 설정
    std::string display_mode_;
    bool show_direction_lines_;
    float three_point_tolerance_;
    
    // 상수
    static constexpr int LIDAR_RADIUS_PIXELS = 200;  // 반경 200픽셀
    static constexpr float LIDAR_MAX_RANGE = 12.0f;  // 12m 최대 거리
    static constexpr float LIDAR_CENTER_ANGLE_RANGE = 4.0f;  // 중심 각도 범위 (±2도)
};

#endif // DISTANCE_OVERLAY_H

