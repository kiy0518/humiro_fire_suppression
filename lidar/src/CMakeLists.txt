cmake_minimum_required(VERSION 3.10)
project(lidar_integration)

# C++17 표준 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# OpenCV 찾기
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 스레드 라이브러리
find_package(Threads REQUIRED)

# 소스 파일
set(LIDAR_SOURCES
    lidar_interface.cpp
)

# 헤더 파일
set(LIDAR_HEADERS
    lidar_interface.h
)

# 라이브러리 생성
add_library(lidar_integration STATIC ${LIDAR_SOURCES})
target_link_libraries(lidar_integration 
    ${OpenCV_LIBS}
    Threads::Threads
)

# 프로젝트 루트 찾기
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

# OSD 라이브러리 경로
set(OSD_DIR "${PROJECT_ROOT}/osd/src")
set(OSD_BUILD_DIR "${OSD_DIR}/build")
include_directories(${OSD_DIR}/lidar)

# 테스트 프로그램
add_executable(lidar_test main_test.cpp)
target_link_libraries(lidar_test 
    lidar_integration
    ${OpenCV_LIBS}
    Threads::Threads
)
link_directories(${OSD_BUILD_DIR})
target_link_libraries(lidar_test osd_lib)

# 각도별 통계 테스트 프로그램
add_executable(angle_statistics_test angle_statistics_test.cpp)
target_link_libraries(angle_statistics_test 
    lidar_integration
    Threads::Threads
)

# 빌드 순서 강제 (osd_lib이 먼저 빌드되도록)
add_custom_command(TARGET lidar_test PRE_BUILD
    COMMAND ${CMAKE_COMMAND} --build ${OSD_BUILD_DIR} --target osd_lib
    COMMENT "Building osd_lib..."
)

# 설치
install(TARGETS lidar_integration lidar_test
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${LIDAR_HEADERS}
    DESTINATION include/lidar_integration
)

# 출력 메시지
message(STATUS "LiDAR Integration Configuration:")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
