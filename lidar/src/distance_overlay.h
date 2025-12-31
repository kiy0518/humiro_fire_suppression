#ifndef DISTANCE_OVERLAY_H
#define DISTANCE_OVERLAY_H

#include <opencv2/opencv.hpp>
#include "lidar_interface.h"
#include <vector>

/**
 * LiDAR 거리 데이터를 영상에 오버레이하는 클래스
 * 
 * 기능:
 * - 카메라 FOV 범위의 LiDAR 데이터를 라인으로 표시
 * - 거리에 따른 색상 코딩
 *   - 10m 부근 (9~11m): 초록색 (발사 최적 거리)
 *   - 가까운 거리 (< 9m): 빨간색
 *   - 먼 거리 (> 11m): 파란색
 */

class DistanceOverlay {
public:
    /**
     * 생성자
     * @param camera_fov 카메라 수평 FOV (도, 기본: 60도)
     * @param image_width 영상 너비 (픽셀)
     * @param image_height 영상 높이 (픽셀)
     */
    DistanceOverlay(float camera_fov = 60.0f, 
                   int image_width = 640, 
                   int image_height = 480);
    
    /**
     * LiDAR 거리 데이터를 영상에 오버레이
     * @param frame 입력/출력 영상
     * @param lidar_data LiDAR 포인트 데이터
     */
    void drawDistanceLine(cv::Mat& frame, const std::vector<LidarPoint>& lidar_data);
    
    /**
     * 거리에 따른 색상 가져오기
     * @param distance 거리 (미터)
     * @return BGR 색상
     */
    cv::Scalar getColorForDistance(float distance);
    
    /**
     * 설정 변경
     */
    void setCameraFOV(float fov) { camera_fov_ = fov; }
    void setImageSize(int width, int height) { 
        image_width_ = width; 
        image_height_ = height; 
    }
    
    /**
     * 거리 정보 텍스트 표시
     */
    void drawDistanceInfo(cv::Mat& frame, const std::vector<LidarPoint>& lidar_data);

    /**
     * 360도 원형 레이더 뷰 표시
     * @param frame 입력/출력 영상
     * @param lidar_data LiDAR 포인트 데이터 (전체 360도)
     */
    void drawRadarView(cv::Mat& frame, const std::vector<LidarPoint>& lidar_data);

private:
    /**
     * 각도를 화면 X 좌표로 변환
     * @param angle 각도 (0도 = 정면)
     * @return X 좌표 (픽셀)
     */
    int angleToScreenX(float angle);
    
    /**
     * 거리를 화면 Y 좌표로 변환 (거리가 가까울수록 아래)
     * @param distance 거리 (미터)
     * @return Y 좌표 (픽셀)
     */
    int distanceToScreenY(float distance);
    
    /**
     * 평균 거리 계산
     */
    float calculateAverageDistance(const std::vector<LidarPoint>& lidar_data);
    
    // 설정
    float camera_fov_;     // 카메라 FOV (도)
    int image_width_;      // 영상 너비
    int image_height_;     // 영상 높이
    
    // 거리 범위 (Y 좌표 매핑용)
    float min_distance_;   // 최소 거리 (m)
    float max_distance_;   // 최대 거리 (m)
    
    // 색상 테마
    cv::Scalar color_optimal_;  // 최적 거리 색상 (초록색)
    cv::Scalar color_close_;    // 가까운 거리 색상 (빨간색)
    cv::Scalar color_far_;      // 먼 거리 색상 (파란색)
};

#endif // DISTANCE_OVERLAY_H
