/**
 * @file mission_overlay.cpp
 * @brief 화재 진압 미션 정보 OSD 오버레이 구현
 */

#include "mission_overlay.h"
#include <chrono>
#include <iomanip>
#include <sstream>

MissionOverlay::MissionOverlay()
    : has_status_(false)
    , has_mission_start_(false)
    , is_connected_(false)
    , last_message_time_(0.0)
{
    memset(&current_status_, 0, sizeof(current_status_));
    memset(&mission_start_, 0, sizeof(mission_start_));
}

MissionOverlay::~MissionOverlay() {
}

void MissionOverlay::draw(cv::Mat& frame) {
    drawConnectionStatus(frame);
    drawMissionStatus(frame);
    drawSuppressionResult(frame);
}

void MissionOverlay::updateMissionStatus(const custom_message::FireMissionStatus& status) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_ = status;
    has_status_ = true;

    // 마지막 메시지 시간 업데이트
    std::lock_guard<std::mutex> conn_lock(connection_mutex_);
    auto now = std::chrono::steady_clock::now();
    auto epoch = now.time_since_epoch();
    last_message_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count() / 1000.0;
    is_connected_ = true;
}

void MissionOverlay::updateSuppressionResult(const custom_message::FireSuppressionResult& result) {
    std::lock_guard<std::mutex> lock(result_mutex_);

    // 현재 시간
    auto now = std::chrono::steady_clock::now();
    auto epoch = now.time_since_epoch();
    double timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count() / 1000.0;

    ResultInfo info;
    info.result = result;
    info.timestamp = timestamp;

    recent_results_.push_back(info);

    // 최대 개수 유지
    if (recent_results_.size() > MAX_RESULTS) {
        recent_results_.erase(recent_results_.begin());
    }
}

void MissionOverlay::updateMissionStart(const custom_message::FireMissionStart& start) {
    std::lock_guard<std::mutex> lock(start_mutex_);
    mission_start_ = start;
    has_mission_start_ = true;
}

void MissionOverlay::updateConnectionStatus(bool connected, double last_message_time) {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    is_connected_ = connected;
    last_message_time_ = last_message_time;
}

std::string MissionOverlay::phaseToString(uint8_t phase) {
    using custom_message::FireMissionPhase;

    switch (static_cast<FireMissionPhase>(phase)) {
        case FireMissionPhase::FIRE_PHASE_IDLE:
            return "IDLE";
        case FireMissionPhase::FIRE_PHASE_NAVIGATING:
            return "NAVIGATING";
        case FireMissionPhase::FIRE_PHASE_SCANNING:
            return "SCANNING";
        case FireMissionPhase::FIRE_PHASE_READY_TO_FIRE:
            return "READY TO FIRE";
        case FireMissionPhase::FIRE_PHASE_SUPPRESSING:
            return "SUPPRESSING";
        case FireMissionPhase::FIRE_PHASE_VERIFYING:
            return "VERIFYING";
        case FireMissionPhase::FIRE_PHASE_COMPLETE:
            return "COMPLETE";
        default:
            return "UNKNOWN";
    }
}

cv::Scalar MissionOverlay::phaseToColor(uint8_t phase) {
    using custom_message::FireMissionPhase;

    switch (static_cast<FireMissionPhase>(phase)) {
        case FireMissionPhase::FIRE_PHASE_IDLE:
            return cv::Scalar(128, 128, 128);  // 회색
        case FireMissionPhase::FIRE_PHASE_NAVIGATING:
            return cv::Scalar(255, 255, 0);    // 청록
        case FireMissionPhase::FIRE_PHASE_SCANNING:
            return cv::Scalar(255, 200, 0);    // 파랑
        case FireMissionPhase::FIRE_PHASE_READY_TO_FIRE:
            return cv::Scalar(0, 255, 255);    // 노랑
        case FireMissionPhase::FIRE_PHASE_SUPPRESSING:
            return cv::Scalar(0, 165, 255);    // 주황
        case FireMissionPhase::FIRE_PHASE_VERIFYING:
            return cv::Scalar(0, 255, 0);      // 초록
        case FireMissionPhase::FIRE_PHASE_COMPLETE:
            return cv::Scalar(0, 200, 0);      // 진한 초록
        default:
            return cv::Scalar(255, 255, 255);  // 흰색
    }
}

void MissionOverlay::drawMissionStatus(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(status_mutex_);

    if (!has_status_) {
        // 상태 정보가 없을 때
        cv::putText(frame, "Mission: No Data", cv::Point(STATUS_X, STATUS_Y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100, 100, 100), 2);
        return;
    }

    int y = STATUS_Y;
    const int line_height = 25;

    // Phase
    std::string phase_str = phaseToString(current_status_.phase);
    cv::Scalar phase_color = phaseToColor(current_status_.phase);
    cv::putText(frame, "Phase: " + phase_str, cv::Point(STATUS_X, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, phase_color, 2);
    y += line_height;

    // Progress
    std::ostringstream progress_oss;
    progress_oss << "Progress: " << static_cast<int>(current_status_.progress) << "%";
    cv::putText(frame, progress_oss.str(), cv::Point(STATUS_X, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    y += line_height;

    // Remaining Projectiles
    std::ostringstream proj_oss;
    proj_oss << "Projectiles: " << static_cast<int>(current_status_.remaining_projectiles);
    cv::putText(frame, proj_oss.str(), cv::Point(STATUS_X, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    y += line_height;

    // Distance to Target
    std::ostringstream dist_oss;
    dist_oss << "Distance: " << std::fixed << std::setprecision(1) << current_status_.distance_to_target << "m";
    cv::putText(frame, dist_oss.str(), cv::Point(STATUS_X, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    y += line_height;

    // Thermal Max Temp
    if (current_status_.thermal_max_temp > 0) {
        std::ostringstream temp_oss;
        temp_oss << "Max Temp: " << std::fixed << std::setprecision(1)
                 << (current_status_.thermal_max_temp / 10.0) << "C";
        cv::Scalar temp_color = current_status_.thermal_max_temp > 500 ?
                               cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255);
        cv::putText(frame, temp_oss.str(), cv::Point(STATUS_X, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, temp_color, 2);
        y += line_height;
    }

    // Status Text (한글 제거 - 영어만 표시)
    // if (strlen(current_status_.status_text) > 0) {
    //     std::string status_text(current_status_.status_text);
    //     cv::putText(frame, status_text, cv::Point(STATUS_X, y),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    // }
}

void MissionOverlay::drawSuppressionResult(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(result_mutex_);

    if (recent_results_.empty()) {
        return;
    }

    int y = RESULT_Y;
    const int line_height = 25;

    // 제목
    cv::putText(frame, "Recent Shots:", cv::Point(RESULT_X, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    y += line_height;

    // 최근 결과들 (최신순)
    for (auto it = recent_results_.rbegin(); it != recent_results_.rend(); ++it) {
        const auto& result = it->result;

        std::ostringstream oss;
        oss << "Shot #" << static_cast<int>(result.shot_number) << ": ";
        oss << (result.success ? "SUCCESS" : "FAILED") << " | ";
        oss << (result.temp_before / 10.0) << "C -> " << (result.temp_after / 10.0) << "C";

        cv::Scalar color = result.success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(frame, oss.str(), cv::Point(RESULT_X, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        y += line_height;
    }
}

void MissionOverlay::drawConnectionStatus(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(connection_mutex_);

    std::string conn_text;
    cv::Scalar conn_color;

    if (is_connected_) {
        // 현재 시간
        auto now = std::chrono::steady_clock::now();
        auto epoch = now.time_since_epoch();
        double current_time = std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count() / 1000.0;

        double elapsed = current_time - last_message_time_;

        if (elapsed < 2.0) {
            conn_text = "MAVLink: CONNECTED";
            conn_color = cv::Scalar(0, 255, 0);  // 초록
        } else if (elapsed < 5.0) {
            conn_text = "MAVLink: WEAK";
            conn_color = cv::Scalar(0, 255, 255);  // 노랑
        } else {
            conn_text = "MAVLink: TIMEOUT";
            conn_color = cv::Scalar(0, 0, 255);  // 빨강
            is_connected_ = false;
        }
    } else {
        conn_text = "MAVLink: DISCONNECTED";
        conn_color = cv::Scalar(128, 128, 128);  // 회색
    }

    cv::putText(frame, conn_text, cv::Point(CONNECTION_X, CONNECTION_Y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, conn_color, 2);
}
