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
find_package(humiro_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
  ${humiro_msgs_INCLUDE_DIRS}
)

# ========== OffboardManager library ==========
# px4_msgs는 handler 인터페이스(state_handler.h)에서 TrajectorySetpoint 사용
# FC 통신은 FCBridgeClient (UDP IPC)로 대체됨
add_library(offboard_manager STATIC
  offboard_manager.cpp
  bridge/fc_bridge_client.cpp
)

target_link_libraries(offboard_manager
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${humiro_msgs_LIBRARIES}
)

# ========== fc_bridge 실행파일 (Domain 0, loopback) ==========
# /fmu/* ROS2 토픽 ↔ UDP IPC 변환 프로세스
add_executable(fc_bridge
  bridge/fc_bridge_node.cpp
  bridge/fc_bridge_server.cpp
)

target_link_libraries(fc_bridge
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)

# Install
install(TARGETS
  offboard_manager
  fc_bridge
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
