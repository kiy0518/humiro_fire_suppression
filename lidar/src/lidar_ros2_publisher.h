#ifndef LIDAR_ROS2_PUBLISHER_H
#define LIDAR_ROS2_PUBLISHER_H

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../../lidar/src/lidar_interface.h"
#include <vector>
#include <memory>
#include <string>

/**
 * 라이다 데이터 ROS2 토픽 발행 클래스
 * 외부 모니터링/디버깅용 (내부 통신은 큐 유지)
 */
class LidarROS2Publisher {
public:
    LidarROS2Publisher(rclcpp::Node::SharedPtr node);
    ~LidarROS2Publisher();
    
    /**
     * 라이다 포인트 클라우드 발행
     * @param points 라이다 포인트 벡터
     */
    void publishLidarPoints(const std::vector<LidarPoint>& points);
    
    /**
     * 전방 거리 발행
     * @param distance 전방 거리 (미터)
     */
    void publishFrontDistance(float distance);
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_distance_pub_;
    
    std::string frame_id_;
};

#else
// ROS2 비활성화 시 빈 클래스
class LidarROS2Publisher {
public:
    LidarROS2Publisher(void* node) {}
    ~LidarROS2Publisher() {}
    void publishLidarPoints(const std::vector<LidarPoint>&) {}
    void publishFrontDistance(float) {}
};

#endif // ENABLE_ROS2

#endif // LIDAR_ROS2_PUBLISHER_H


