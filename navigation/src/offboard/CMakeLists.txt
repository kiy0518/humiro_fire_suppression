cmake_minimum_required(VERSION 3.8)
project(offboard_control)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 환경 확인
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS2 environment not sourced. Please source setup.bash")
endif()

# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
)

# ArmHandler library
add_library(arm_handler STATIC
  autonomous/arm_handler.cpp
)

target_link_libraries(arm_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# TakeoffHandler library
add_library(takeoff_handler STATIC
  autonomous/takeoff_handler.cpp
)

target_link_libraries(takeoff_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# WaypointHandler library
add_library(waypoint_handler STATIC
  autonomous/waypoint_handler.cpp
)

target_link_libraries(waypoint_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# RTLHandler library
add_library(rtl_handler STATIC
  autonomous/rtl_handler.cpp
)

target_link_libraries(rtl_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# DistanceAdjuster library
add_library(distance_adjuster STATIC
  autonomous/distance_adjuster.cpp
)

target_link_libraries(distance_adjuster
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# OffboardManager library
add_library(offboard_manager STATIC
  autonomous/offboard_manager.cpp
)

target_link_libraries(offboard_manager
  arm_handler
  takeoff_handler
  waypoint_handler
  distance_adjuster
  rtl_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# Test executables
add_executable(test_arm test_arm.cpp)
target_link_libraries(test_arm
  arm_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

add_executable(test_takeoff test_takeoff.cpp)
target_link_libraries(test_takeoff
  arm_handler
  takeoff_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

add_executable(test_waypoint test_waypoint.cpp)
target_link_libraries(test_waypoint
  arm_handler
  takeoff_handler
  waypoint_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

add_executable(test_rtl test_rtl.cpp)
target_link_libraries(test_rtl
  arm_handler
  takeoff_handler
  waypoint_handler
  rtl_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

add_executable(test_mission test_mission.cpp)
target_link_libraries(test_mission
  offboard_manager
  waypoint_handler
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# Install
install(TARGETS
  arm_handler
  takeoff_handler
  waypoint_handler
  rtl_handler
  distance_adjuster
  offboard_manager
  test_arm
  test_takeoff
  test_waypoint
  test_rtl
  test_mission
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
