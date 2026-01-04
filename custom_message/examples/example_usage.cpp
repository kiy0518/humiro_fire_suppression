/**
 * @file example_usage.cpp
 * @brief 커스텀 메시지 송수신 라이브러리 사용 예제
 * 
 * 컴파일:
 *   g++ -std=c++17 -I../include example_usage.cpp -L../build -lcustom_message -pthread -o example_usage
 * 
 * 실행:
 *   ./example_usage
 */

#include "custom_message/custom_message.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

using namespace custom_message;

// 전역 변수 (시그널 핸들러용)
std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    std::cout << "\n[예제] 종료 신호 수신 (signal: " << signal << ")" << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    // 시그널 핸들러 등록
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== 커스텀 메시지 송수신 예제 ===" << std::endl;
    std::cout << "수신 포트: 14550" << std::endl;
    std::cout << "송신 포트: 14550" << std::endl;
    std::cout << "대상 주소: 127.0.0.1" << std::endl;
    std::cout << "종료: Ctrl+C" << std::endl;
    std::cout << std::endl;

    // 메시지 송수신기 생성
    CustomMessage msg_handler(14550, 14550, "0.0.0.0", "127.0.0.1", 1, 1);

    // 수신 콜백 등록
    msg_handler.setFireMissionStartCallback([](const FireMissionStart& start) {
        std::cout << "[수신] 미션 시작 - "
                  << "위치: (" << (start.target_lat / 1e7) << ", " << (start.target_lon / 1e7) << ")"
                  << ", 고도: " << start.target_alt << "m"
                  << ", 자동발사: " << (start.auto_fire ? "예" : "아니오")
                  << ", 최대발사: " << static_cast<int>(start.max_projectiles) << std::endl;
    });

    msg_handler.setFireMissionStatusCallback([](const FireMissionStatus& status) {
        std::cout << "[수신] 미션 상태 - "
                  << "단계: " << phase_names[status.phase] << " (" << static_cast<int>(status.phase) << ")"
                  << ", 진행률: " << static_cast<int>(status.progress) << "%"
                  << ", 남은발사: " << static_cast<int>(status.remaining_projectiles)
                  << ", 거리: " << status.distance_to_target << "m"
                  << ", 최대온도: " << (status.thermal_max_temp / 10.0f) << "°C"
                  << ", 상태: " << status.status_text << std::endl;
    });

    msg_handler.setFireLaunchControlCallback([](const FireLaunchControl& control) {
        const char* cmd_names[] = {"확인", "중단", "상태요청"};
        std::cout << "[수신] 발사 제어 - "
                  << "명령: " << cmd_names[control.command] << " (" << static_cast<int>(control.command) << ")" << std::endl;
    });

    msg_handler.setFireSuppressionResultCallback([](const FireSuppressionResult& result) {
        std::cout << "[수신] 진압 결과 - "
                  << "발사번호: " << static_cast<int>(result.shot_number)
                  << ", 성공: " << (result.success ? "예" : "아니오") << std::endl;
    });

    // 메시지 송수신 시작
    if (!msg_handler.start()) {
        std::cerr << "메시지 송수신 시작 실패" << std::endl;
        return 1;
    }

    std::cout << "메시지 송수신 대기 중..." << std::endl;
    std::cout << "5초 후 테스트 메시지 전송 시작" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 테스트 메시지 전송 (주기적으로)
    int test_count = 0;
    while (g_running && msg_handler.isRunning()) {
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 미션 상태 전송 테스트
        FireMissionStatus status;
        status.phase = test_count % 7;  // 0-6 단계
        status.progress = (test_count * 10) % 101;  // 0-100%
        status.remaining_projectiles = 6 - (test_count % 7);
        status.distance_to_target = 100.0f - (test_count * 5.0f);
        status.thermal_max_temp = 500 + (test_count * 10);  // °C * 10
        snprintf(status.status_text, sizeof(status.status_text), "테스트 상태 %d", test_count);

        if (msg_handler.sendFireMissionStatus(status)) {
            std::cout << "[송신] 미션 상태 전송 성공 (단계: " << static_cast<int>(status.phase) << ")" << std::endl;
        } else {
            std::cerr << "[송신] 미션 상태 전송 실패" << std::endl;
        }

        // 진압 결과 전송 테스트 (3초마다)
        if (test_count % 2 == 0) {
            FireSuppressionResult result;
            result.shot_number = test_count / 2;
            result.success = (test_count % 4 != 0) ? 1 : 0;
            result.reserved[0] = 0;
            result.reserved[1] = 0;
            result.reserved[2] = 0;
            result.reserved[3] = 0;
            result.reserved[4] = 0;
            result.reserved[5] = 0;

            if (msg_handler.sendFireSuppressionResult(result)) {
                std::cout << "[송신] 진압 결과 전송 성공" << std::endl;
            }
        }

        test_count++;

        // 통계 정보 출력 (10초마다)
        if (test_count % 3 == 0) {
            auto stats = msg_handler.getStatistics();
            std::cout << "\n[통계] "
                      << "수신 - 미션시작: " << stats.mission_start_received
                      << ", 미션상태: " << stats.mission_status_received
                      << ", 발사제어: " << stats.launch_control_received
                      << ", 진압결과: " << stats.suppression_result_received
                      << " | "
                      << "송신 - 미션시작: " << stats.mission_start_sent
                      << ", 미션상태: " << stats.mission_status_sent
                      << ", 발사제어: " << stats.launch_control_sent
                      << ", 진압결과: " << stats.suppression_result_sent
                      << " | "
                      << "오류 - 알수없음: " << stats.unknown_message_count
                      << ", 파싱오류: " << stats.parse_error_count
                      << ", 송신오류: " << stats.send_error_count << std::endl;
        }
    }

    // 메시지 송수신 중지
    msg_handler.stop();

    std::cout << "\n[예제] 종료" << std::endl;
    return 0;
}
