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
    distance_overlay.cpp
)

# 헤더 파일
set(LIDAR_HEADERS
    lidar_interface.h
    distance_overlay.h
)

# 라이브러리 생성
add_library(lidar_integration STATIC ${LIDAR_SOURCES})
target_link_libraries(lidar_integration 
    ${OpenCV_LIBS}
    Threads::Threads
)

# 테스트 프로그램
add_executable(lidar_test main_test.cpp)
target_link_libraries(lidar_test 
    lidar_integration
    ${OpenCV_LIBS}
    Threads::Threads
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
