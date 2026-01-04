cmake_minimum_required(VERSION 3.10)
project(osd_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 찾기
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 프로젝트 루트 찾기
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

# Include 디렉토리
set(THERMAL_DIR "${PROJECT_ROOT}/thermal/src")
set(LIDAR_DIR "${PROJECT_ROOT}/lidar/src")
include_directories(${THERMAL_DIR})
include_directories(${LIDAR_DIR})

# FreeType 찾기 (선택사항)
find_package(Freetype QUIET)
if(FREETYPE_FOUND)
    add_definitions(-DHAVE_FREETYPE)
    message(STATUS "FreeType found: ${FREETYPE_LIBRARIES}")
else()
    message(STATUS "FreeType not found, using PIL fallback for Korean text")
endif()

# 소스 파일
set(SOURCES
    thermal/thermal_overlay.cpp
    lidar/distance_overlay.cpp
    targeting/aim_indicator.cpp
    targeting/hotspot_tracker.cpp
    status/status_overlay.cpp
    utils/korean_text_renderer.cpp
)

# OSD 라이브러리 생성
add_library(osd_lib STATIC ${SOURCES})

# 링크 라이브러리
target_link_libraries(osd_lib
    ${OpenCV_LIBS}
    pthread
)

# FreeType 링크 (있는 경우)
if(FREETYPE_FOUND)
    target_link_libraries(osd_lib ${FREETYPE_LIBRARIES})
    target_include_directories(osd_lib PRIVATE ${FREETYPE_INCLUDE_DIRS})
endif()

