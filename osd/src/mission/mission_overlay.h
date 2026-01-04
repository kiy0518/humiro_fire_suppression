/**
 * @file mission_overlay.h
 * @brief 화재 진압 미션 정보 OSD 오버레이
 *
 * 커스텀 메시지로 수신된 미션 상태, 발사 결과 등을 화면에 표시
 *
 * @author Humiro Fire Suppression Team
 * @date 2026-01-04
 */

#ifndef MISSION_OVERLAY_H
#define MISSION_OVERLAY_H

#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include "../../custom_message/include/custom_message/custom_message_type.h"

/**
 * @brief 미션 상태 오버레이 클래스
 *
 * 화재 진압 미션의 현재 상태, 진행률, 발사 결과 등을 화면에 표시합니다.
 */
class MissionOverlay {
public:
    MissionOverlay();
    ~MissionOverlay();

    /**
     * @brief OSD 오버레이 그리기
     *
     * @param frame OpenCV 프레임 (수정됨)
     */
    void draw(cv::Mat& frame);

    /**
     * @brief 미션 상태 업데이트
     *
     * @param status 미션 상태 메시지
     */
    void updateMissionStatus(const custom_message::FireMissionStatus& status);

    /**
     * @brief 발사 결과 업데이트
     *
     * @param result 발사 결과 메시지
     */
    void updateSuppressionResult(const custom_message::FireSuppressionResult& result);

    /**
     * @brief 미션 시작 정보 업데이트
     *
     * @param start 미션 시작 메시지
     */
    void updateMissionStart(const custom_message::FireMissionStart& start);

    /**
     * @brief 통신 상태 업데이트
     *
     * @param connected 연결 여부
     * @param last_message_time 마지막 메시지 수신 시간 (초)
     */
    void updateConnectionStatus(bool connected, double last_message_time);

private:
    // Phase를 문자열로 변환
    std::string phaseToString(uint8_t phase);

    // Phase 색상 반환
    cv::Scalar phaseToColor(uint8_t phase);

    // 미션 상태 그리기
    void drawMissionStatus(cv::Mat& frame);

    // 발사 결과 그리기 (최근 결과)
    void drawSuppressionResult(cv::Mat& frame);

    // 통신 상태 그리기
    void drawConnectionStatus(cv::Mat& frame);

    // 뮤텍스
    std::mutex status_mutex_;
    std::mutex result_mutex_;
    std::mutex start_mutex_;
    std::mutex connection_mutex_;

    // 미션 상태
    custom_message::FireMissionStatus current_status_;
    bool has_status_;

    // 발사 결과 (최근 3개)
    struct ResultInfo {
        custom_message::FireSuppressionResult result;
        double timestamp;
    };
    std::vector<ResultInfo> recent_results_;
    static constexpr size_t MAX_RESULTS = 3;

    // 미션 시작 정보
    custom_message::FireMissionStart mission_start_;
    bool has_mission_start_;

    // 통신 상태
    bool is_connected_;
    double last_message_time_;

    // OSD 위치
    static constexpr int STATUS_X = 20;
    static constexpr int STATUS_Y = 100;
    static constexpr int RESULT_X = 20;
    static constexpr int RESULT_Y = 300;
    static constexpr int CONNECTION_X = 20;
    static constexpr int CONNECTION_Y = 60;
};

#endif // MISSION_OVERLAY_H
