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

# 프로젝트 루트 찾기
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

# 소스 파일 (main.cpp는 application/로 이동, thermal_basic_overlay는 overlays/로 이동)
set(SOURCES
    utils.cpp
    camera_manager.cpp
    thermal_processor.cpp
)

# thermal 라이브러리 생성 (메인 애플리케이션은 application/로 이동)
add_library(thermal_lib STATIC ${SOURCES})

# 링크 라이브러리
target_link_libraries(thermal_lib
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    pthread
)

# 컴파일 플래그
target_compile_options(thermal_lib PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
)
