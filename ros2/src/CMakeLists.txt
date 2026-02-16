cmake_minimum_required(VERSION 3.10)

# ROS2 통신 라이브러리
# Status, Collision Avoidance, Command 등의 ROS2 구독자/발행자 포함
# v2.0: StatusROS2Subscriber는 FCBridgeClient(UDP IPC)를 사용, px4_msgs 불필요

set(ROS2_SOURCES
    status/status_ros2_subscriber.cpp
)

# ROS2 옵션 (선택적)
option(ENABLE_ROS2 "Enable ROS2 topic publishing" OFF)

if(ENABLE_ROS2)
    # ROS2 찾기
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    # ROS2 정의 추가
    add_definitions(-DENABLE_ROS2)

    message(STATUS "ROS2 communication library enabled (px4_msgs-free)")
else()
    message(STATUS "ROS2 communication library disabled (use -DENABLE_ROS2=ON to enable)")
endif()

# 정적 라이브러리 생성 (선택적)
if(ENABLE_ROS2)
    add_library(ros2_comm_lib STATIC ${ROS2_SOURCES})

    # Include 경로: navigation 디렉토리 (fc_bridge_protocol.h, fc_bridge_client.h)
    get_filename_component(NAVIGATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../navigation/src/offboard" ABSOLUTE)

    target_include_directories(ros2_comm_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/status
        ${NAVIGATION_DIR}
    )

    ament_target_dependencies(ros2_comm_lib
        rclcpp
        std_msgs
    )
endif()
