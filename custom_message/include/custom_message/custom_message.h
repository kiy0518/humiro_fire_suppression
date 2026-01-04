/**
 * @file custom_message.h
 * @brief 커스텀 MAVLink 메시지 송수신 라이브러리
 * 
 * QGC와 VIM4 간 화재 진압 미션 전용 커스텀 MAVLink 메시지를 송수신하는 라이브러리
 * 
 * @author Humiro Fire Suppression Team
 * @date 2026-01-03
 */

#ifndef CUSTOM_MESSAGE_H
#define CUSTOM_MESSAGE_H

#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include "custom_message_type.h"

namespace custom_message {

// 전방 선언
class CustomMessageImpl;

/**
 * @brief 미션 시작 콜백 함수 타입
 * 
 * @param start 미션 시작 메시지
 */
using FireMissionStartCallback = std::function<void(const FireMissionStart& start)>;

/**
 * @brief 미션 상태 콜백 함수 타입
 * 
 * @param status 미션 상태 메시지
 */
using FireMissionStatusCallback = std::function<void(const FireMissionStatus& status)>;

/**
 * @brief 발사 제어 콜백 함수 타입
 * 
 * @param control 발사 제어 메시지
 */
using FireLaunchControlCallback = std::function<void(const FireLaunchControl& control)>;

/**
 * @brief 진압 결과 콜백 함수 타입
 * 
 * @param result 진압 결과 메시지
 */
using FireSuppressionResultCallback = std::function<void(const FireSuppressionResult& result)>;

/**
 * @brief 커스텀 MAVLink 메시지 송수신기
 * 
 * QGC와 VIM4 간 화재 진압 미션 전용 커스텀 MAVLink 메시지를 송수신하고 파싱하여
 * 등록된 콜백 함수를 호출합니다.
 */
class CustomMessage {
public:
    /**
     * @brief 생성자
     * 
     * @param receive_port UDP 수신 포트 (기본값: 14550)
     * @param send_port UDP 송신 포트 (기본값: 14550)
     * @param bind_address 바인드 주소 (기본값: "0.0.0.0")
     * @param target_address 송신 대상 주소 (기본값: "127.0.0.1")
     * @param system_id MAVLink 시스템 ID (기본값: 1)
     * @param component_id MAVLink 컴포넌트 ID (기본값: 1)
     */
    CustomMessage(
        uint16_t receive_port = 14550,
        uint16_t send_port = 14550,
        const std::string& bind_address = "0.0.0.0",
        const std::string& target_address = "127.0.0.1",
        uint8_t system_id = 1,
        uint8_t component_id = 1
    );
    
    /**
     * @brief 소멸자
     */
    ~CustomMessage();

    // 복사 및 이동 생성자/연산자 삭제
    CustomMessage(const CustomMessage&) = delete;
    CustomMessage& operator=(const CustomMessage&) = delete;
    CustomMessage(CustomMessage&&) = delete;
    CustomMessage& operator=(CustomMessage&&) = delete;

    /**
     * @brief 메시지 수신 시작
     * 
     * @return true 성공, false 실패
     */
    bool start();

    /**
     * @brief 메시지 수신 중지
     */
    void stop();

    /**
     * @brief 수신 중인지 확인
     * 
     * @return true 수신 중, false 중지됨
     */
    bool isRunning() const;

    // ========== 수신 콜백 등록 ==========

    /**
     * @brief 미션 시작 콜백 등록
     * 
     * @param callback 콜백 함수
     */
    void setFireMissionStartCallback(FireMissionStartCallback callback);

    /**
     * @brief 미션 상태 콜백 등록
     * 
     * @param callback 콜백 함수
     */
    void setFireMissionStatusCallback(FireMissionStatusCallback callback);

    /**
     * @brief 발사 제어 콜백 등록
     * 
     * @param callback 콜백 함수
     */
    void setFireLaunchControlCallback(FireLaunchControlCallback callback);

    /**
     * @brief 진압 결과 콜백 등록
     * 
     * @param callback 콜백 함수
     */
    void setFireSuppressionResultCallback(FireSuppressionResultCallback callback);

    // ========== 송신 함수 ==========

    /**
     * @brief 미션 시작 전송
     * 
     * @param start 미션 시작 메시지
     * @return true 성공, false 실패
     */
    bool sendFireMissionStart(const FireMissionStart& start);

    /**
     * @brief 미션 상태 전송
     * 
     * @param status 미션 상태 메시지
     * @return true 성공, false 실패
     */
    bool sendFireMissionStatus(const FireMissionStatus& status);

    /**
     * @brief 발사 제어 전송
     * 
     * @param control 발사 제어 메시지
     * @return true 성공, false 실패
     */
    bool sendFireLaunchControl(const FireLaunchControl& control);

    /**
     * @brief 진압 결과 전송
     * 
     * @param result 진압 결과 메시지
     * @return true 성공, false 실패
     */
    bool sendFireSuppressionResult(const FireSuppressionResult& result);

    /**
     * @brief 송신 대상 주소 설정
     * 
     * @param address 대상 IP 주소
     * @param port 대상 포트
     */
    void setTargetAddress(const std::string& address, uint16_t port);

    // ========== 통계 정보 ==========

    /**
     * @brief 수신된 메시지 통계 정보
     */
    struct Statistics {
        uint64_t mission_start_received = 0;
        uint64_t mission_status_received = 0;
        uint64_t launch_control_received = 0;
        uint64_t suppression_result_received = 0;
        uint64_t mission_start_sent = 0;
        uint64_t mission_status_sent = 0;
        uint64_t launch_control_sent = 0;
        uint64_t suppression_result_sent = 0;
        uint64_t unknown_message_count = 0;
        uint64_t parse_error_count = 0;
        uint64_t send_error_count = 0;
    };

    /**
     * @brief 통계 정보 가져오기
     * 
     * @return Statistics 통계 정보
     */
    Statistics getStatistics() const;

    /**
     * @brief 통계 정보 리셋
     */
    void resetStatistics();

private:
    std::unique_ptr<CustomMessageImpl> impl_;
};

} // namespace custom_message

#endif // CUSTOM_MESSAGE_H
