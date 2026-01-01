cmake_minimum_required(VERSION 3.10)
project(Streaming)

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

# GStreamer App 라이브러리 찾기
find_library(GSTREAMER_APP_LIBRARY
    NAMES gstapp-1.0
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
    NO_DEFAULT_PATH
)

if(NOT GSTREAMER_APP_LIBRARY)
    set(GSTREAMER_APP_LIBRARY "/usr/lib/aarch64-linux-gnu/libgstapp-1.0.so")
endif()

# 프로젝트 루트 찾기
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

# 열화상 소스 경로 (thread_safe_queue, config.h)
set(THERMAL_DIR "${PROJECT_ROOT}/thermal/src")
include_directories(${THERMAL_DIR})

# 소스 파일
set(SOURCES
    rtsp_server.cpp
    http_server.cpp
    streaming_manager.cpp
)

# 라이브러리 생성
add_library(streaming_lib STATIC ${SOURCES})

# 링크 라이브러리
target_link_libraries(streaming_lib
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_RTSP_LIBRARY}
    ${GSTREAMER_APP_LIBRARY}
    ${MHD_LIBRARY}
    pthread
)

# 컴파일 플래그
target_compile_options(streaming_lib PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
)

