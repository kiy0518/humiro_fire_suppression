cmake_minimum_required(VERSION 3.10)

# CMake 정책 설정 (경고 제거)
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

project(HumiroFireSuppression)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 찾기
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# GStreamer 찾기
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
include_directories(${GSTREAMER_INCLUDE_DIRS})

# GStreamer RTSP Server 찾기
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
else()
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
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)

# 각 계층 디렉토리
set(THERMAL_DIR "${PROJECT_ROOT}/thermal/src")
set(THERMAL_BUILD_DIR "${THERMAL_DIR}/build")
set(LIDAR_DIR "${PROJECT_ROOT}/lidar/src")
set(TARGETING_DIR "${PROJECT_ROOT}/targeting/src")
set(TARGETING_BUILD_DIR "${TARGETING_DIR}/build")
set(OSD_DIR "${PROJECT_ROOT}/osd/src")
set(OSD_BUILD_DIR "${OSD_DIR}/build")
set(STREAMING_DIR "${PROJECT_ROOT}/streaming/src")
set(STREAMING_BUILD_DIR "${STREAMING_DIR}/build")
set(ROS2_DIR "${PROJECT_ROOT}/ros2/src")
set(ROS2_BUILD_DIR "${ROS2_DIR}/build")
set(CUSTOM_MESSAGE_DIR "${PROJECT_ROOT}/custom_message")
set(CUSTOM_MESSAGE_BUILD_DIR "${CUSTOM_MESSAGE_DIR}/build")

# Include 디렉토리
include_directories(${THERMAL_DIR})
include_directories(${LIDAR_DIR})
include_directories(${TARGETING_DIR})
include_directories(${OSD_DIR}/thermal)
include_directories(${OSD_DIR}/lidar)
include_directories(${OSD_DIR}/targeting)
include_directories(${OSD_DIR}/status)
include_directories(${STREAMING_DIR})
include_directories(${ROS2_DIR})
include_directories(${ROS2_DIR}/status)
include_directories(${CUSTOM_MESSAGE_DIR}/include)

# 소스 파일
set(SOURCES
    main.cpp
    src/application_manager.cpp
    src/frame_compositor.cpp
)

# ROS2 옵션 (선택적) - add_executable 전에 확인 필요
option(ENABLE_ROS2 "Enable ROS2 topic publishing" OFF)

# ROS2 지원 추가 (선택적) - add_executable 전에 SOURCES에 추가
if(ENABLE_ROS2)
    # ROS2 소스 추가 (ROS2 통신 라이브러리 사용)
    # status_ros2_subscriber는 ros2_comm_lib에 포함됨
    list(APPEND SOURCES
        ${THERMAL_DIR}/thermal_ros2_publisher.cpp
        ${LIDAR_DIR}/lidar_ros2_publisher.cpp
    )
endif()

# 라이브러리 링크 디렉토리
link_directories(${THERMAL_BUILD_DIR})
link_directories(${OSD_BUILD_DIR})
link_directories(${TARGETING_BUILD_DIR})
link_directories(${STREAMING_BUILD_DIR})
link_directories(${ROS2_BUILD_DIR})
link_directories(${CUSTOM_MESSAGE_BUILD_DIR})

# 실행 파일 생성
add_executable(humiro_fire_suppression ${SOURCES})

# GStreamer App 라이브러리 찾기
find_library(GSTREAMER_APP_LIBRARY
    NAMES gstapp-1.0
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
    NO_DEFAULT_PATH
)

if(NOT GSTREAMER_APP_LIBRARY)
    set(GSTREAMER_APP_LIBRARY "/usr/lib/aarch64-linux-gnu/libgstapp-1.0.so")
endif()

# thermal, targeting, streaming 라이브러리는 먼저 빌드해야 함
# 각 라이브러리를 먼저 빌드:
#   cd thermal/src/build && cmake .. && make thermal_lib
#   cd targeting/src/build && cmake .. && make targeting_lib  
#   cd streaming/src/build && cmake .. && make streaming_lib

# ROS2 지원 추가 (선택적) - add_executable 후에 설정
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
    # PX4 uXRCE-DDS 메시지 (px4_msgs) - 필수
    find_package(px4_msgs REQUIRED)
    
    if(NOT px4_msgs_FOUND)
        message(FATAL_ERROR "px4_msgs is required but not found. Please install px4_msgs.")
    endif()
    
    # ROS2 정의 추가
    add_definitions(-DENABLE_ROS2)
    add_definitions(-Dpx4_msgs_FOUND)
    
    message(STATUS "ROS2 support enabled")
    message(STATUS "px4_msgs found: ${px4_msgs_DIR}")
else()
    message(STATUS "ROS2 support disabled (use -DENABLE_ROS2=ON to enable)")
endif()

# 커스텀 메시지 라이브러리 먼저 빌드
add_subdirectory(${CUSTOM_MESSAGE_DIR} ${CUSTOM_MESSAGE_BUILD_DIR})

# 링크 라이브러리
# streaming_lib이 정적 라이브러리이므로, streaming_lib이 의존하는 모든 라이브러리를 링크해야 함
# targeting_lib이 osd_lib을 참조하므로, 순환 의존성을 해결하기 위해 --start-group/--end-group 사용
target_link_libraries(humiro_fire_suppression
    ${OpenCV_LIBS}
    thermal_lib
    -Wl,--start-group
    osd_lib
    targeting_lib
    -Wl,--end-group
    streaming_lib
    custom_message  # 커스텀 메시지 라이브러리
    ${GSTREAMER_LIBRARIES}  # streaming_lib과 utils.cpp용
    ${GSTREAMER_RTSP_LIBRARY}  # streaming_lib용
    ${GSTREAMER_APP_LIBRARY}  # streaming_lib용
    ${MHD_LIBRARY}  # streaming_lib용
    pthread
)

# ROS2 통신 라이브러리 (선택적)
if(ENABLE_ROS2)
    # ros2_comm_lib 먼저 빌드
    add_subdirectory(${ROS2_DIR} ${ROS2_BUILD_DIR})
    target_link_libraries(humiro_fire_suppression ros2_comm_lib)
endif()

# ROS2 라이브러리 (선택적)
if(ENABLE_ROS2)
    ament_target_dependencies(humiro_fire_suppression
        rclcpp
        sensor_msgs
        std_msgs
        geometry_msgs
        cv_bridge
        pcl_conversions
        PCL
    )
    
    # px4_msgs 링크 (있는 경우)
    if(px4_msgs_FOUND)
        ament_target_dependencies(humiro_fire_suppression px4_msgs)
    endif()
endif()

# 컴파일 플래그
target_compile_options(humiro_fire_suppression PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
)

