cmake_minimum_required(VERSION 3.10)
project(ThermalRGBStreaming)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 찾기
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# GStreamer 찾기
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

include_directories(${GSTREAMER_INCLUDE_DIRS})

# GStreamer RTSP Server 찾기 (pkg-config가 없을 수 있으므로 직접 찾기)
find_path(GSTREAMER_RTSP_INCLUDE_DIR
    NAMES gst/rtsp-server/rtsp-server.h
    PATHS /usr/include/gstreamer-1.0 /usr/local/include/gstreamer-1.0
    NO_DEFAULT_PATH
)

find_library(GSTREAMER_RTSP_LIBRARY
    NAMES gstrtspserver-1.0
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
    NO_DEFAULT_PATH
)

if(GSTREAMER_RTSP_INCLUDE_DIR AND GSTREAMER_RTSP_LIBRARY)
    include_directories(${GSTREAMER_RTSP_INCLUDE_DIR}/..)
    message(STATUS "Found GStreamer RTSP Server: ${GSTREAMER_RTSP_LIBRARY}")
    message(STATUS "  Include dir: ${GSTREAMER_RTSP_INCLUDE_DIR}")
else()
    # 직접 경로 지정 (fallback)
    set(GSTREAMER_RTSP_INCLUDE_DIR "/usr/include/gstreamer-1.0")
    set(GSTREAMER_RTSP_LIBRARY "/usr/lib/aarch64-linux-gnu/libgstrtspserver-1.0.so")
    include_directories(${GSTREAMER_RTSP_INCLUDE_DIR})
    message(STATUS "Using hardcoded GStreamer RTSP Server path: ${GSTREAMER_RTSP_LIBRARY}")
endif()

# libmicrohttpd 찾기
find_library(MHD_LIBRARY microhttpd)
if(NOT MHD_LIBRARY)
    message(FATAL_ERROR "libmicrohttpd not found")
endif()

# 프로젝트 루트 찾기
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

# 라이다 디렉토리
set(LIDAR_DIR "${PROJECT_ROOT}/lidar/src")
include_directories(${LIDAR_DIR})

# 타겟팅 라이브러리 (빌드된 라이브러리 링크)
set(TARGETING_DIR "${PROJECT_ROOT}/targeting/src")
set(TARGETING_BUILD_DIR "${TARGETING_DIR}/build")
include_directories(${TARGETING_DIR})

# 스트리밍 라이브러리 (빌드된 라이브러리 링크)
set(STREAMING_DIR "${PROJECT_ROOT}/streaming/src")
set(STREAMING_BUILD_DIR "${STREAMING_DIR}/build")
include_directories(${STREAMING_DIR})

# ROS2 옵션 (선택적)
option(ENABLE_ROS2 "Enable ROS2 topic publishing" OFF)

# 소스 파일
set(SOURCES
    main.cpp
    utils.cpp
    camera_manager.cpp
    thermal_processor.cpp
    thermal_basic_overlay.cpp
)

# ROS2 지원 추가 (선택적)
if(ENABLE_ROS2)
    # ROS2 찾기
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(pcl_conversions REQUIRED)
    find_package(PCL REQUIRED)
    
    # ROS2 정의 추가
    add_definitions(-DENABLE_ROS2)
    
    # ROS2 소스 추가
    list(APPEND SOURCES
        thermal_ros2_publisher.cpp
        ../../lidar/src/lidar_ros2_publisher.cpp
    )
    
    message(STATUS "ROS2 support enabled")
endif()

# 라이브러리 링크 디렉토리
link_directories(${TARGETING_BUILD_DIR})
link_directories(${STREAMING_BUILD_DIR})

# 실행 파일 생성
add_executable(thermal_rgb_streaming ${SOURCES})

# GStreamer App 라이브러리 찾기
find_library(GSTREAMER_APP_LIBRARY
    NAMES gstapp-1.0
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
    NO_DEFAULT_PATH
)

if(NOT GSTREAMER_APP_LIBRARY)
    set(GSTREAMER_APP_LIBRARY "/usr/lib/aarch64-linux-gnu/libgstapp-1.0.so")
endif()

# 타겟팅 및 스트리밍 라이브러리 먼저 빌드
add_custom_command(TARGET thermal_rgb_streaming PRE_BUILD
    COMMAND ${CMAKE_COMMAND} --build ${TARGETING_BUILD_DIR} --target targeting_lib
    COMMAND ${CMAKE_COMMAND} --build ${STREAMING_BUILD_DIR} --target streaming_lib
    COMMENT "Building targeting and streaming libraries..."
)

# 링크 라이브러리
# streaming_lib이 정적 라이브러리이므로, streaming_lib이 의존하는 모든 라이브러리를 링크해야 함
target_link_libraries(thermal_rgb_streaming
    ${OpenCV_LIBS}
    targeting_lib
    streaming_lib
    ${GSTREAMER_LIBRARIES}  # streaming_lib과 utils.cpp용
    ${GSTREAMER_RTSP_LIBRARY}  # streaming_lib용
    ${GSTREAMER_APP_LIBRARY}  # streaming_lib용
    ${MHD_LIBRARY}  # streaming_lib용
    pthread
)

# ROS2 라이브러리 (선택적)
if(ENABLE_ROS2)
    ament_target_dependencies(thermal_rgb_streaming
        rclcpp
        sensor_msgs
        std_msgs
        geometry_msgs
        cv_bridge
        pcl_conversions
        PCL
    )
endif()

# 컴파일 플래그
target_compile_options(thermal_rgb_streaming PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
)
