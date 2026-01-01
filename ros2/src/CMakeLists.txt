cmake_minimum_required(VERSION 3.10)

# ROS2 통신 라이브러리
# Status, Collision Avoidance, Command 등의 ROS2 구독자/발행자 포함

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
    # px4_msgs 필수 패키지 (REQUIRED로 변경)
    find_package(px4_msgs REQUIRED)
    
    if(NOT px4_msgs_FOUND)
        message(FATAL_ERROR "px4_msgs is required but not found. Please install px4_msgs.")
    endif()
    
    message(STATUS "px4_msgs found: ${px4_msgs_DIR}")
    add_definitions(-Dpx4_msgs_FOUND)
    
    # ROS2 정의 추가
    add_definitions(-DENABLE_ROS2)
    
    message(STATUS "ROS2 communication library enabled")
else()
    message(STATUS "ROS2 communication library disabled (use -DENABLE_ROS2=ON to enable)")
endif()

# 정적 라이브러리 생성 (선택적)
if(ENABLE_ROS2)
    add_library(ros2_comm_lib STATIC ${ROS2_SOURCES})
    
    target_include_directories(ros2_comm_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/status
    )
    
    # px4_msgs 필수이므로 항상 링크
    ament_target_dependencies(ros2_comm_lib
        rclcpp
        std_msgs
        px4_msgs
    )
    
    # px4_msgs include 경로 직접 추가 (경로 중복 문제 해결)
    # px4_msgs_DIR은 share/px4_msgs/cmake이므로, ../../..를 하면 install/px4_msgs로 감
    get_filename_component(PX4_MSGS_INSTALL_DIR "${px4_msgs_DIR}/../../.." ABSOLUTE)
    set(PX4_MSGS_INCLUDE_DIR "${PX4_MSGS_INSTALL_DIR}/include")
    message(STATUS "px4_msgs_DIR: ${px4_msgs_DIR}")
    message(STATUS "px4_msgs install directory: ${PX4_MSGS_INSTALL_DIR}")
    message(STATUS "px4_msgs include directory: ${PX4_MSGS_INCLUDE_DIR}")
    
    # include 경로 추가
    target_include_directories(ros2_comm_lib PUBLIC
        "${PX4_MSGS_INCLUDE_DIR}"
    )
    
    target_compile_definitions(ros2_comm_lib PRIVATE px4_msgs_FOUND)
endif()

