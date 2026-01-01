cmake_minimum_required(VERSION 3.10)
project(Targeting)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 찾기
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 프로젝트 루트 찾기
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

# 라이다 인터페이스 경로
set(LIDAR_DIR "${PROJECT_ROOT}/lidar/src")
include_directories(${LIDAR_DIR})

# 열화상 데이터 경로
set(THERMAL_DIR "${PROJECT_ROOT}/thermal/src")
include_directories(${THERMAL_DIR})

# OSD 라이브러리 경로
set(OSD_DIR "${PROJECT_ROOT}/osd/src")
set(OSD_BUILD_DIR "${OSD_DIR}/build")
include_directories(${OSD_DIR}/thermal)
include_directories(${OSD_DIR}/lidar)
include_directories(${OSD_DIR}/targeting)

# 소스 파일
set(SOURCES
    targeting_frame_compositor.cpp
)

# 라이다 소스 파일 (필요한 경우)
set(LIDAR_SOURCES
    ${LIDAR_DIR}/lidar_interface.cpp
)

# 라이브러리 생성
add_library(targeting_lib STATIC ${SOURCES} ${LIDAR_SOURCES})

# 링크 라이브러리
link_directories(${OSD_BUILD_DIR})
target_link_libraries(targeting_lib
    osd_lib
    ${OpenCV_LIBS}
    pthread
)

# 빌드 순서 강제 (osd_lib이 먼저 빌드되도록)
add_custom_command(TARGET targeting_lib PRE_BUILD
    COMMAND ${CMAKE_COMMAND} --build ${OSD_BUILD_DIR} --target osd_lib
    COMMENT "Building osd_lib..."
)

# 헤더 파일 설치 (선택사항)
install(TARGETS targeting_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

