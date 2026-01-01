#ifndef THERMAL_ROS2_PUBLISHER_H
#define THERMAL_ROS2_PUBLISHER_H

#ifdef ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include "thermal_data.h"
#include <memory>
#include <string>

/**
 * 열화상 데이터 ROS2 토픽 발행 클래스
 * 외부 모니터링/디버깅용 (내부 통신은 큐 유지)
 */
class ThermalROS2Publisher {
public:
    ThermalROS2Publisher(rclcpp::Node::SharedPtr node);
    ~ThermalROS2Publisher();
    
    /**
     * 열화상 데이터 발행
     * @param thermal_data 열화상 데이터
     */
    void publishThermalData(const ThermalData& thermal_data);
    
    /**
     * 열화상 이미지 발행
     * @param frame 열화상 프레임 (OpenCV Mat)
     */
    void publishThermalImage(const cv::Mat& frame);
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr max_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_temp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr center_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr hotspot_pub_;
    
    std::string frame_id_;
};

#else
// ROS2 비활성화 시 빈 클래스
class ThermalROS2Publisher {
public:
    ThermalROS2Publisher(void* node) {}
    ~ThermalROS2Publisher() {}
    void publishThermalData(const ThermalData&) {}
    void publishThermalImage(const cv::Mat&) {}
};

#endif // ENABLE_ROS2

#endif // THERMAL_ROS2_PUBLISHER_H


