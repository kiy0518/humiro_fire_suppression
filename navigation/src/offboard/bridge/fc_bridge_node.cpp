/**
 * @file fc_bridge_node.cpp
 * @brief FC Bridge 프로세스 진입점
 *
 * ROS_DOMAIN_ID=0 환경에서 실행.
 * /droneN/fmu/* 토픽을 구독/발행하고, 로컬 UDP IPC로 메인앱과 통신.
 *
 * 실행:
 *   ROS_DOMAIN_ID=0 FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_loopback_only.xml \
 *     ./fc_bridge --ros-args -r __ns:=/drone1
 */

#include "fc_bridge_server.h"
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <iostream>
#include <signal.h>

static std::atomic<bool> g_shutdown{false};

static void signal_handler(int) {
    g_shutdown.store(true);
}

int main(int argc, char* argv[])
{
    // 시그널 핸들러
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // ROS2 초기화
    rclcpp::init(argc, argv);

    // PX4 토픽 네임스페이스 결정
    // FC_BRIDGE_PX4_NS: PX4 토픽 접두사 (예: "/drone1" → "/drone1/fmu/out/*")
    // 비어있으면 네임스페이스 없음 (FC 모드 기본값: "/fmu/out/*")
    std::string px4_ns;
    const char* px4_ns_env = std::getenv("FC_BRIDGE_PX4_NS");
    if (px4_ns_env && px4_ns_env[0] != '\0') {
        px4_ns = px4_ns_env;
    }
    // 기본값: 빈 문자열 (micro-ros-agent가 네임스페이스 없이 실행될 때)

    // IPC 포트 (환경 변수 오버라이드 가능)
    uint16_t state_port = FC_BRIDGE_STATE_PORT;
    uint16_t command_port = FC_BRIDGE_COMMAND_PORT;
    const char* sp = std::getenv("FC_BRIDGE_STATE_PORT");
    const char* cp = std::getenv("FC_BRIDGE_COMMAND_PORT");
    if (sp) state_port = static_cast<uint16_t>(std::atoi(sp));
    if (cp) command_port = static_cast<uint16_t>(std::atoi(cp));

    std::cout << "========================================" << std::endl;
    std::cout << "  FC Bridge Process" << std::endl;
    std::cout << "  Domain: " << (std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "0") << std::endl;
    std::cout << "  PX4 NS: " << px4_ns << std::endl;
    std::cout << "  State Port: " << state_port << " (fc_bridge → app)" << std::endl;
    std::cout << "  Command Port: " << command_port << " (app → fc_bridge)" << std::endl;
    std::cout << "========================================" << std::endl;

    // ROS2 노드 생성
    auto node = rclcpp::Node::make_shared("fc_bridge");

    // FC Bridge 서버 생성 및 시작
    FCBridgeServer server(node, px4_ns, state_port, command_port);
    if (!server.start()) {
        std::cerr << "[FATAL] FCBridgeServer failed to start!" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    std::cout << "[FC Bridge] Running... (Ctrl+C to stop)" << std::endl;

    // ROS2 스핀 (메인 스레드)
    while (rclcpp::ok() && !g_shutdown.load()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\n[FC Bridge] Shutting down..." << std::endl;
    server.stop();
    rclcpp::shutdown();
    return 0;
}
