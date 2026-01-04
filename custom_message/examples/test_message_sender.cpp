/**
 * @file test_message_sender.cpp
 * @brief 테스트용 MAVLink 메시지 송신 프로그램
 * 
 * OSD 커스텀 메시지 출력을 테스트하기 위한 프로그램입니다.
 * 15000 포트로 메시지를 전송합니다.
 * 
 * 사용법:
 *   ./test_message_sender
 * 
 * @author Humiro Fire Suppression Team
 * @date 2026-01-04
 */

#include "custom_message/custom_message.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

using namespace custom_message;

void printMenu() {
    std::cout << "\n===========================================" << std::endl;
    std::cout << "  테스트 메시지 송신 프로그램 (15000 포트)" << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << "1. FIRE_MISSION_START 전송" << std::endl;
    std::cout << "2. FIRE_LAUNCH_CONTROL (CONFIRM) 전송" << std::endl;
    std::cout << "3. FIRE_LAUNCH_CONTROL (ABORT) 전송" << std::endl;
    std::cout << "4. FIRE_LAUNCH_CONTROL (REQUEST_STATUS) 전송" << std::endl;
    std::cout << "5. FIRE_MISSION_STATUS 전송" << std::endl;
    std::cout << "6. FIRE_SUPPRESSION_RESULT 전송" << std::endl;
    std::cout << "0. 종료" << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << "선택: ";
}

int main() {
    std::cout << "\n[테스트 메시지 송신 프로그램 시작]" << std::endl;
    std::cout << "대상 포트: 15000 (테스트용)" << std::endl;
    std::cout << "대상 주소: 127.0.0.1 (로컬)" << std::endl;
    
    // CustomMessage 생성 (15000 포트로 전송)
    CustomMessage msg_handler(
        15001,  // 수신 포트 (사용 안 함, 충돌 방지용)
        15000,  // 송신 포트 (테스트 대상)
        "0.0.0.0",
        "127.0.0.1",  // 로컬호스트
        1,  // 시스템 ID
        1   // 컴포넌트 ID
    );
    
    if (!msg_handler.start()) {
        std::cerr << "메시지 핸들러 시작 실패" << std::endl;
        return 1;
    }
    
    std::cout << "✓ 메시지 핸들러 시작 완료" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    while (true) {
        printMenu();
        
        int choice;
        std::cin >> choice;
        
        if (choice == 0) {
            std::cout << "프로그램 종료" << std::endl;
            break;
        }
        
        switch (choice) {
            case 1: {
                // FIRE_MISSION_START 전송
                FireMissionStart start;
                start.target_system = 1;
                start.target_component = 1;
                start.target_lat = 371234567;   // 37.1234567° * 1e7
                start.target_lon = 1271234567;  // 127.1234567° * 1e7
                start.target_alt = 10.0f;       // 10m
                start.auto_fire = 1;            // 자동 발사
                start.max_projectiles = 5;
                
                std::cout << "\n[전송] FIRE_MISSION_START" << std::endl;
                std::cout << "  위도: 37.1234567°" << std::endl;
                std::cout << "  경도: 127.1234567°" << std::endl;
                std::cout << "  고도: 10.0m" << std::endl;
                std::cout << "  Auto Fire: 예" << std::endl;
                
                if (msg_handler.sendFireMissionStart(start)) {
                    std::cout << "✓ 전송 성공" << std::endl;
                } else {
                    std::cout << "✗ 전송 실패" << std::endl;
                }
                break;
            }
            
            case 2: {
                // FIRE_LAUNCH_CONTROL (CONFIRM)
                FireLaunchControl control;
                control.target_system = 1;
                control.target_component = 1;
                control.command = 0;  // CONFIRM
                
                std::cout << "\n[전송] FIRE_LAUNCH_CONTROL (CONFIRM)" << std::endl;
                
                if (msg_handler.sendFireLaunchControl(control)) {
                    std::cout << "✓ 전송 성공" << std::endl;
                } else {
                    std::cout << "✗ 전송 실패" << std::endl;
                }
                break;
            }
            
            case 3: {
                // FIRE_LAUNCH_CONTROL (ABORT)
                FireLaunchControl control;
                control.target_system = 1;
                control.target_component = 1;
                control.command = 1;  // ABORT
                
                std::cout << "\n[전송] FIRE_LAUNCH_CONTROL (ABORT)" << std::endl;
                
                if (msg_handler.sendFireLaunchControl(control)) {
                    std::cout << "✓ 전송 성공" << std::endl;
                } else {
                    std::cout << "✗ 전송 실패" << std::endl;
                }
                break;
            }
            
            case 4: {
                // FIRE_LAUNCH_CONTROL (REQUEST_STATUS)
                FireLaunchControl control;
                control.target_system = 1;
                control.target_component = 1;
                control.command = 2;  // REQUEST_STATUS
                
                std::cout << "\n[전송] FIRE_LAUNCH_CONTROL (REQUEST_STATUS)" << std::endl;
                
                if (msg_handler.sendFireLaunchControl(control)) {
                    std::cout << "✓ 전송 성공" << std::endl;
                } else {
                    std::cout << "✗ 전송 실패" << std::endl;
                }
                break;
            }
            
            case 5: {
                // FIRE_MISSION_STATUS
                FireMissionStatus status;
                status.phase = static_cast<uint8_t>(FireMissionPhase::FIRE_PHASE_NAVIGATING);
                status.progress = 50;
                status.remaining_projectiles = 8;
                status.distance_to_target = 25.5f;
                status.thermal_max_temp = 850;  // 85.0°C
                std::strncpy(status.status_text, "Flying to target", sizeof(status.status_text) - 1);
                
                std::cout << "\n[전송] FIRE_MISSION_STATUS" << std::endl;
                std::cout << "  상태: NAVIGATING (50%)" << std::endl;
                std::cout << "  거리: 25.5m" << std::endl;
                
                if (msg_handler.sendFireMissionStatus(status)) {
                    std::cout << "✓ 전송 성공" << std::endl;
                } else {
                    std::cout << "✗ 전송 실패" << std::endl;
                }
                break;
            }
            
            case 6: {
                // FIRE_SUPPRESSION_RESULT
                FireSuppressionResult result;
                result.shot_number = 1;
                result.temp_before = 900;  // 90.0°C
                result.temp_after = 300;   // 30.0°C
                result.success = 1;
                
                std::cout << "\n[전송] FIRE_SUPPRESSION_RESULT" << std::endl;
                std::cout << "  발사 번호: 1" << std::endl;
                std::cout << "  온도: 90.0°C → 30.0°C" << std::endl;
                std::cout << "  결과: 성공" << std::endl;
                
                if (msg_handler.sendFireSuppressionResult(result)) {
                    std::cout << "✓ 전송 성공" << std::endl;
                } else {
                    std::cout << "✗ 전송 실패" << std::endl;
                }
                break;
            }
            
            default:
                std::cout << "잘못된 선택입니다." << std::endl;
                break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    msg_handler.stop();
    return 0;
}
