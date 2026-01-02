#ifdef ENABLE_ROS2

#include "thermal_ros2_publisher.h"
#include <sensor_msgs/image_encodings.hpp>

ThermalROS2Publisher::ThermalROS2Publisher(rclcpp::Node::SharedPtr node)
    : node_(node)
    , frame_id_("thermal_camera") {
    
    // 토픽 발행자 생성
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/thermal/image", 10);
    max_temp_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/thermal/max_temperature", 10);
    min_temp_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/thermal/min_temperature", 10);
    center_pub_ = node_->create_publisher<geometry_msgs::msg::Point>("/thermal/center", 10);
    hotspot_pub_ = node_->create_publisher<geometry_msgs::msg::Point>("/thermal/hotspot", 10);
    
    RCLCPP_INFO(node_->get_logger(), "Thermal ROS2 publisher initialized");
}

ThermalROS2Publisher::~ThermalROS2Publisher() {
}

void ThermalROS2Publisher::publishThermalData(const ThermalData& thermal_data) {
    if (!thermal_data.valid) {
        return;
    }
    
    // 최대 온도 발행 (섭씨 온도)
    std_msgs::msg::Float32 max_temp_msg;
    max_temp_msg.data = thermal_data.max_temp_celsius;
    max_temp_pub_->publish(max_temp_msg);
    
    // 최소 온도 발행 (섭씨 온도)
    std_msgs::msg::Float32 min_temp_msg;
    min_temp_msg.data = thermal_data.min_temp_celsius;
    min_temp_pub_->publish(min_temp_msg);
    
    // 중심점 발행
    geometry_msgs::msg::Point center_msg;
    center_msg.x = static_cast<double>(thermal_data.center_x);
    center_msg.y = static_cast<double>(thermal_data.center_y);
    center_msg.z = 0.0;
    center_pub_->publish(center_msg);
    
    // Hotspot 발행
    geometry_msgs::msg::Point hotspot_msg;
    hotspot_msg.x = static_cast<double>(thermal_data.hotspot_x);
    hotspot_msg.y = static_cast<double>(thermal_data.hotspot_y);
    hotspot_msg.z = 0.0;
    hotspot_pub_->publish(hotspot_msg);
}

void ThermalROS2Publisher::publishThermalImage(const cv::Mat& frame) {
    if (frame.empty()) {
        return;
    }
    
    try {
        // OpenCV Mat을 ROS2 Image 메시지로 변환
        std_msgs::msg::Header header;
        header.stamp = node_->now();
        header.frame_id = frame_id_;
        
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
            header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        
        image_pub_->publish(*img_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

#endif // ENABLE_ROS2


