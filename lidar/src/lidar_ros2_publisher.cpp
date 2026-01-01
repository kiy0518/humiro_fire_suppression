#ifdef ENABLE_ROS2

#include "lidar_ros2_publisher.h"

LidarROS2Publisher::LidarROS2Publisher(rclcpp::Node::SharedPtr node)
    : node_(node)
    , frame_id_("lidar") {
    
    // 토픽 발행자 생성
    points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", 10);
    front_distance_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/lidar/front_distance", 10);
    
    RCLCPP_INFO(node_->get_logger(), "Lidar ROS2 publisher initialized");
}

LidarROS2Publisher::~LidarROS2Publisher() {
}

void LidarROS2Publisher::publishLidarPoints(const std::vector<LidarPoint>& points) {
    if (points.empty()) {
        return;
    }
    
    // PCL PointCloud 생성
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(points.size());
    
    // 라이다 포인트를 PCL 포인트로 변환
    for (size_t i = 0; i < points.size(); i++) {
        const auto& pt = points[i];
        // 각도와 거리로부터 x, y 좌표 계산
        float angle_rad = pt.angle * M_PI / 180.0f;
        cloud.points[i].x = pt.distance * cos(angle_rad);
        cloud.points[i].y = pt.distance * sin(angle_rad);
        cloud.points[i].z = 0.0f;
    }
    
    // PCL PointCloud를 ROS2 PointCloud2 메시지로 변환
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id_;
    
    points_pub_->publish(msg);
}

void LidarROS2Publisher::publishFrontDistance(float distance) {
    std_msgs::msg::Float32 msg;
    msg.data = distance;
    front_distance_pub_->publish(msg);
}

#endif // ENABLE_ROS2


