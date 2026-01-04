cmake_minimum_required(VERSION 3.10)
project(custom_message)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 헤더 파일 경로
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# 소스 파일
set(SOURCES
    src/custom_message.cpp
)

# 헤더 파일
set(HEADERS
    include/custom_message/custom_message.h
    include/custom_message/custom_message_type.h
)

# 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})

# 포함 디렉토리 설정
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
)

# 컴파일 옵션
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

# 스레드 라이브러리 링크
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

# 설치 설정
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${HEADERS}
    DESTINATION include/custom_message
)

# Examples
add_executable(test_message_sender examples/test_message_sender.cpp)
target_link_libraries(test_message_sender 
    custom_message
    Threads::Threads
)
