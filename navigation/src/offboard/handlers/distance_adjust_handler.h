/**
 * @file distance_adjust_handler.h
 * @brief LiDAR 기반 타겟 거리 조정 핸들러
 *
 * NAVIGATE 도착 후, LiDAR로 벽을 감지하고 수직 정렬 + 10m 거리 확보.
 *
 * 내부 페이즈:
 *   INITIAL_HOVER → APPROACH → WALL_DETECT → WALL_ALIGN → RETREAT → FINAL_STABILIZE → DONE
 *
 * 활성화 조건: lidar_ptr != nullptr && continuous_update_mode == false (리더만)
 *
 * 전환 조건:
 *   - DONE → COMPLETE (다음 핸들러로 전환)
 *   - 전체 타임아웃 (90초) → COMPLETE (폴백)
 */

#ifndef DISTANCE_ADJUST_HANDLER_H
#define DISTANCE_ADJUST_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"
#include "lidar_interface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

class DistanceAdjustHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();

        // 도착 위치 기록
        arrival_x_ = ctx.current_local_x.load();
        arrival_y_ = ctx.current_local_y.load();
        hold_z_ = ctx.start_local_z - ctx.takeoff_altitude;
        approach_heading_ = ctx.target_yaw;

        // ctx에서 거리조정 파라미터 로드
        approach_speed_ = ctx.adjust_approach_speed;
        retreat_speed_ = ctx.adjust_approach_speed;  // 후진 속도 = 접근 속도
        target_wall_dist_ = ctx.adjust_target_wall_dist;
        retreat_distance_ = ctx.adjust_retreat_dist;

        // 상태 초기화
        phase_ = AdjustPhase::INITIAL_HOVER;
        phase_enter_time_ = std::chrono::steady_clock::now();
        detect_x_ = arrival_x_;
        detect_y_ = arrival_y_;
        final_x_ = arrival_x_;
        final_y_ = arrival_y_;
        target_perp_yaw_ = approach_heading_;
        wall_detected_ = false;
        approach_timeout_ = false;
        lidar_lost_ticks_ = 0;
        last_log_tick_ = -1;
        wall_confirm_count_ = 0;
        front_dist_history_.clear();
        prev_align_yawspeed_ = 0.0f;

        // 벽 버킷 초기화
        for (auto& b : wall_buckets_) {
            b.distance = 0.0f;
            b.confidence = 0.0f;
            b.count = 0;
        }

        RCLCPP_INFO(ctx.logger,
            "[DIST_ADJUST] 시작: 도착점 NED(%.1f, %.1f), heading=%.1f°, lidar=%s",
            arrival_x_, arrival_y_,
            approach_heading_ * 180.0f / M_PI,
            ctx.lidar_ptr ? "연결됨" : "없음");
    }

    void onExit(MissionContext& ctx) override {
        // 거리 조정 완료 시 target 좌표/yaw 업데이트
        if (ctx.distance_adjust_result.completed) {
            ctx.target_ned_x = final_x_;
            ctx.target_ned_y = final_y_;
            ctx.target_yaw = target_perp_yaw_;
            RCLCPP_INFO(ctx.logger,
                "[DIST_ADJUST] 완료: 최종 NED(%.1f, %.1f), yaw=%.1f°, 벽거리=%.1fm",
                final_x_, final_y_,
                target_perp_yaw_ * 180.0f / M_PI,
                ctx.distance_adjust_result.wall_distance);
        } else {
            RCLCPP_WARN(ctx.logger,
                "[DIST_ADJUST] 미완료 종료 (%.1fs)", ctx.elapsedSec());
        }
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        // 전체 타임아웃
        if (elapsed >= TOTAL_TIMEOUT_SEC) {
            RCLCPP_WARN(ctx.logger,
                "[DIST_ADJUST] 전체 타임아웃 (%.0f초) → 현재 위치로 완료", TOTAL_TIMEOUT_SEC);
            recordFinalPosition(ctx);
            return TransitionResult::COMPLETE;
        }

        // LiDAR 연결 체크
        if (ctx.lidar_ptr && !ctx.lidar_ptr->isConnected()) {
            lidar_lost_ticks_++;
            if (lidar_lost_ticks_ > LIDAR_LOST_TIMEOUT_TICKS) {
                RCLCPP_WARN(ctx.logger, "[DIST_ADJUST] LiDAR 연결 끊김 5초 → 위치 고정 대기");
                if (phase_ == AdjustPhase::APPROACH) {
                    approach_timeout_ = true;  // 호버링 대기 모드
                }
            }
        } else {
            lidar_lost_ticks_ = 0;
        }

        double phase_elapsed = phaseSec();

        switch (phase_) {
            case AdjustPhase::INITIAL_HOVER:
                if (phase_elapsed >= INITIAL_HOVER_SEC) {
                    setPhase(AdjustPhase::APPROACH, ctx, "안정화 완료 → 접근 시작");
                }
                break;

            case AdjustPhase::APPROACH:
                tickApproach(ctx, phase_elapsed);
                break;

            case AdjustPhase::WALL_DETECT:
                tickWallDetect(ctx, phase_elapsed);
                break;

            case AdjustPhase::WALL_ALIGN:
                tickWallAlign(ctx, phase_elapsed);
                break;

            case AdjustPhase::RETREAT:
                tickRetreat(ctx, phase_elapsed);
                break;

            case AdjustPhase::FINAL_STABILIZE:
                if (phase_elapsed >= FINAL_STABILIZE_SEC) {
                    recordFinalPosition(ctx);
                    phase_ = AdjustPhase::DONE;
                }
                break;

            case AdjustPhase::DONE:
                return TransitionResult::COMPLETE;
        }

        // 진행 로깅 (3초마다)
        int tick_3s = static_cast<int>(elapsed / 3.0);
        if (tick_3s != last_log_tick_) {
            last_log_tick_ = tick_3s;
            logProgress(ctx, elapsed);
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        switch (phase_) {
            case AdjustPhase::INITIAL_HOVER:
                // 도착점에서 위치 고정
                sp.position = {arrival_x_, arrival_y_, hold_z_};
                sp.velocity = {NAN, NAN, NAN};
                sp.yaw = approach_heading_;
                sp.yawspeed = 0.0f;
                break;

            case AdjustPhase::APPROACH:
                if (approach_timeout_) {
                    // 타임아웃: 현재 위치 고정 (GCS 대기)
                    float cx = ctx.current_local_x.load();
                    float cy = ctx.current_local_y.load();
                    sp.position = {cx, cy, hold_z_};
                    sp.velocity = {NAN, NAN, NAN};
                    sp.yaw = approach_heading_;
                    sp.yawspeed = 0.0f;
                } else {
                    // 전진: velocity 제어 + 고도 위치 제어
                    float vx = approach_speed_ * std::cos(approach_heading_);
                    float vy = approach_speed_ * std::sin(approach_heading_);
                    sp.position = {NAN, NAN, hold_z_};
                    sp.velocity = {vx, vy, NAN};
                    sp.yaw = approach_heading_;
                    sp.yawspeed = 0.0f;
                }
                break;

            case AdjustPhase::WALL_DETECT:
                // 감지점에서 위치 고정
                sp.position = {detect_x_, detect_y_, hold_z_};
                sp.velocity = {NAN, NAN, NAN};
                sp.yaw = approach_heading_;
                sp.yawspeed = 0.0f;
                break;

            case AdjustPhase::WALL_ALIGN: {
                // 위치 고정 + yaw 회전 (가속도 제한)
                sp.position = {detect_x_, detect_y_, hold_z_};
                sp.velocity = {NAN, NAN, NAN};

                float current_yaw = ctx.current_yaw.load();
                float yaw_diff = target_perp_yaw_ - current_yaw;
                while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
                while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

                float abs_diff = std::fabs(yaw_diff);
                float target_yawspeed = 0.0f;
                if (abs_diff < 0.02f) {
                    sp.yaw = target_perp_yaw_;
                    sp.yawspeed = 0.0f;
                    prev_align_yawspeed_ = 0.0f;
                    break;
                } else if (abs_diff < YAW_DECEL_ANGLE) {
                    float s = MAX_ALIGN_YAW_RATE * (abs_diff / YAW_DECEL_ANGLE);
                    s = std::max(0.05f, s);
                    target_yawspeed = (yaw_diff > 0) ? s : -s;
                } else {
                    target_yawspeed = (yaw_diff > 0) ? MAX_ALIGN_YAW_RATE : -MAX_ALIGN_YAW_RATE;
                }

                // 가속도 제한
                float max_delta = ALIGN_YAW_ACCEL * 0.1f;
                float delta = target_yawspeed - prev_align_yawspeed_;
                if (delta > max_delta) delta = max_delta;
                if (delta < -max_delta) delta = -max_delta;
                float yawspeed = prev_align_yawspeed_ + delta;
                prev_align_yawspeed_ = yawspeed;

                sp.yaw = NAN;
                sp.yawspeed = yawspeed;
                break;
            }

            case AdjustPhase::RETREAT: {
                // 벽 반대 방향으로 후진 + 고도/헤딩 유지
                float retreat_heading = target_perp_yaw_ + M_PI;
                float vx = retreat_speed_ * std::cos(retreat_heading);
                float vy = retreat_speed_ * std::sin(retreat_heading);
                sp.position = {NAN, NAN, hold_z_};
                sp.velocity = {vx, vy, NAN};
                sp.yaw = target_perp_yaw_;
                sp.yawspeed = 0.0f;
                break;
            }

            case AdjustPhase::FINAL_STABILIZE:
                sp.position = {final_x_, final_y_, hold_z_};
                sp.velocity = {NAN, NAN, NAN};
                sp.yaw = target_perp_yaw_;
                sp.yawspeed = 0.0f;
                break;

            case AdjustPhase::DONE:
                sp.position = {final_x_, final_y_, hold_z_};
                sp.velocity = {NAN, NAN, NAN};
                sp.yaw = target_perp_yaw_;
                sp.yawspeed = 0.0f;
                break;
        }

        return true;
    }

    const char* name() const override { return "DISTANCE_ADJUST"; }

private:
    // ========== 내부 페이즈 ==========
    enum class AdjustPhase {
        INITIAL_HOVER,
        APPROACH,
        WALL_DETECT,
        WALL_ALIGN,
        RETREAT,
        FINAL_STABILIZE,
        DONE
    };

    // ========== 설정 가능 파라미터 (onEnter에서 ctx로부터 로드) ==========
    float approach_speed_{0.3f};          // m/s
    float retreat_speed_{0.3f};            // m/s (= approach_speed)
    float target_wall_dist_{10.0f};       // m (목표 벽 거리)
    float retreat_distance_{4.0f};         // m (감지점에서 후진 거리)

    // ========== 상수 ==========
    static constexpr float LIDAR_DETECT_DIST = 6.0f;       // m (벽 감지 거리)
    static constexpr float EMERGENCY_STOP_DIST = 2.0f;     // m (비상 정지)
    static constexpr float MAX_ALIGN_YAW_RATE = 0.3f;      // rad/s
    static constexpr float ALIGN_YAW_ACCEL = 0.2f;         // rad/s² (yaw 가속도 제한)
    static constexpr float YAW_DECEL_ANGLE = 0.5f;         // rad (감속 시작 각도)
    static constexpr float YAW_CONVERGE_THRESH = 0.05f;    // rad (~3°)

    // SLAM 파라미터
    static constexpr float SMOOTHING_ALPHA = 0.3f;
    static constexpr float CONFIDENCE_DECAY = 0.9f;
    static constexpr float MIN_CONFIDENCE = 0.4f;
    static constexpr float CHANGE_THRESHOLD = 0.8f;
    static constexpr float FRONT_ANGLE_RANGE = 60.0f;      // ±60°
    static constexpr int RANSAC_ITERATIONS = 50;
    static constexpr float RANSAC_INLIER_THRESH = 0.3f;    // m
    static constexpr int MIN_WALL_INLIERS = 15;
    static constexpr int WALL_CONFIRM_TICKS = 3;

    // 타이밍
    static constexpr float INITIAL_HOVER_SEC = 2.0f;
    static constexpr float APPROACH_TIMEOUT_SEC = 30.0f;
    static constexpr float DETECT_TIMEOUT_SEC = 10.0f;
    static constexpr float ALIGN_TIMEOUT_SEC = 15.0f;
    static constexpr float RETREAT_TIMEOUT_SEC = 20.0f;
    static constexpr float FINAL_STABILIZE_SEC = 3.0f;
    static constexpr float TOTAL_TIMEOUT_SEC = 90.0f;
    static constexpr int LIDAR_LOST_TIMEOUT_TICKS = 50;    // 5초
    static constexpr int FRONT_DIST_STABLE_COUNT = 10;     // 1초
    static constexpr size_t MEDIAN_FILTER_SIZE = 5;

    // ========== 벽 버킷 (SLAM 방식) ==========
    struct WallBucket {
        float distance{0.0f};
        float confidence{0.0f};
        int count{0};
    };

    // ========== 벽 라인 피팅 결과 ==========
    struct WallLine {
        bool valid{false};
        float normal_local{0.0f};   // 벽 법선 (LiDAR 로컬 각도, rad)
        float distance{0.0f};       // 수직 거리 (m)
        int inlier_count{0};
    };

    // ========== 상태 변수 ==========
    AdjustPhase phase_{AdjustPhase::INITIAL_HOVER};
    std::chrono::steady_clock::time_point phase_enter_time_;

    float arrival_x_{0.0f}, arrival_y_{0.0f}, hold_z_{0.0f};
    float detect_x_{0.0f}, detect_y_{0.0f};
    float final_x_{0.0f}, final_y_{0.0f};
    float approach_heading_{0.0f};
    float target_perp_yaw_{0.0f};
    float prev_align_yawspeed_{0.0f};

    bool wall_detected_{false};
    bool approach_timeout_{false};
    int lidar_lost_ticks_{0};
    int last_log_tick_{-1};
    int wall_confirm_count_{0};
    int front_dist_stable_count_{0};

    std::array<WallBucket, 360> wall_buckets_{};
    std::vector<float> front_dist_history_;
    WallLine best_wall_{};

    // ========== 페이즈 전환 헬퍼 ==========
    void setPhase(AdjustPhase next, MissionContext& ctx, const char* reason) {
        RCLCPP_INFO(ctx.logger, "[DIST_ADJUST] %s → %s: %s",
                    phaseName(phase_), phaseName(next), reason);
        phase_ = next;
        phase_enter_time_ = std::chrono::steady_clock::now();
    }

    double phaseSec() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - phase_enter_time_).count();
    }

    static const char* phaseName(AdjustPhase p) {
        switch (p) {
            case AdjustPhase::INITIAL_HOVER: return "INITIAL_HOVER";
            case AdjustPhase::APPROACH: return "APPROACH";
            case AdjustPhase::WALL_DETECT: return "WALL_DETECT";
            case AdjustPhase::WALL_ALIGN: return "WALL_ALIGN";
            case AdjustPhase::RETREAT: return "RETREAT";
            case AdjustPhase::FINAL_STABILIZE: return "FINAL_STABILIZE";
            case AdjustPhase::DONE: return "DONE";
            default: return "UNKNOWN";
        }
    }

    // ========== APPROACH 페이즈 ==========
    void tickApproach(MissionContext& ctx, double phase_elapsed) {
        // 타임아웃 체크 (이미 타임아웃이면 GCS 대기)
        if (approach_timeout_) {
            return;  // STAY 유지, fillSetpoint에서 위치 고정
        }

        if (phase_elapsed >= APPROACH_TIMEOUT_SEC) {
            approach_timeout_ = true;
            RCLCPP_WARN(ctx.logger,
                "[DIST_ADJUST] 접근 타임아웃 (%.0f초) → 위치 고정, GCS 명령 대기",
                APPROACH_TIMEOUT_SEC);
            return;
        }

        // LiDAR 전방 거리 읽기
        if (!ctx.lidar_ptr) return;

        float front_dist = ctx.lidar_ptr->getFrontDistance();
        if (front_dist <= 0.0f) return;

        // 비상 정지
        if (front_dist <= EMERGENCY_STOP_DIST) {
            detect_x_ = ctx.current_local_x.load();
            detect_y_ = ctx.current_local_y.load();
            setPhase(AdjustPhase::WALL_DETECT, ctx,
                     "비상 정지! 전방 거리 2m 이내");
            return;
        }

        // 메디안 필터
        float filtered = getMedianFiltered(front_dist);

        // 6m 이내 안정적 감지 확인
        if (filtered <= LIDAR_DETECT_DIST && filtered > 0.0f) {
            front_dist_stable_count_++;
            if (front_dist_stable_count_ >= FRONT_DIST_STABLE_COUNT) {
                detect_x_ = ctx.current_local_x.load();
                detect_y_ = ctx.current_local_y.load();
                char msg[128];
                snprintf(msg, sizeof(msg), "벽 감지! 전방 %.2fm (안정 %d회)",
                         filtered, front_dist_stable_count_);
                setPhase(AdjustPhase::WALL_DETECT, ctx, msg);
            }
        } else {
            front_dist_stable_count_ = 0;
        }
    }

    // ========== WALL_DETECT 페이즈 ==========
    void tickWallDetect(MissionContext& ctx, double phase_elapsed) {
        // 타임아웃
        if (phase_elapsed >= DETECT_TIMEOUT_SEC) {
            if (best_wall_.valid) {
                applyWallResult(ctx);
                setPhase(AdjustPhase::WALL_ALIGN, ctx,
                         "감지 타임아웃, 최선 추정값 사용");
            } else {
                // 벽 감지 실패: 현재 헤딩으로 진행
                target_perp_yaw_ = approach_heading_;
                setPhase(AdjustPhase::RETREAT, ctx,
                         "감지 타임아웃, 벽 미검출 → 현재 헤딩으로 후진");
            }
            return;
        }

        // LiDAR 스캔 누적
        if (!ctx.lidar_ptr) return;

        LidarScan scan;
        if (!ctx.lidar_ptr->getLatestScan(scan)) return;

        accumulateScan(scan);

        // RANSAC 라인 피팅 시도 (매 틱)
        WallLine wall = fitWallRANSAC(ctx.current_yaw.load());

        if (wall.valid && wall.inlier_count >= MIN_WALL_INLIERS) {
            if (wall.inlier_count > best_wall_.inlier_count) {
                best_wall_ = wall;
            }
            wall_confirm_count_++;
            if (wall_confirm_count_ >= WALL_CONFIRM_TICKS) {
                applyWallResult(ctx);
                setPhase(AdjustPhase::WALL_ALIGN, ctx,
                         "벽 검출 확인 완료");
            }
        } else {
            wall_confirm_count_ = 0;
        }
    }

    // ========== WALL_ALIGN 페이즈 ==========
    void tickWallAlign(MissionContext& ctx, double phase_elapsed) {
        if (phase_elapsed >= ALIGN_TIMEOUT_SEC) {
            setPhase(AdjustPhase::RETREAT, ctx,
                     "정렬 타임아웃 → 현재 헤딩으로 후진");
            return;
        }

        float current_yaw = ctx.current_yaw.load();
        float yaw_diff = target_perp_yaw_ - current_yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

        if (std::fabs(yaw_diff) < YAW_CONVERGE_THRESH) {
            setPhase(AdjustPhase::RETREAT, ctx,
                     "수직 정렬 완료 → 후진 시작");
        }
    }

    // ========== RETREAT 페이즈 ==========
    void tickRetreat(MissionContext& ctx, double phase_elapsed) {
        if (phase_elapsed >= RETREAT_TIMEOUT_SEC) {
            final_x_ = ctx.current_local_x.load();
            final_y_ = ctx.current_local_y.load();
            setPhase(AdjustPhase::FINAL_STABILIZE, ctx,
                     "후진 타임아웃 → 현재 위치에서 안정화");
            return;
        }

        // GPS 변위로 거리 추적
        float cx = ctx.current_local_x.load();
        float cy = ctx.current_local_y.load();
        float dx = cx - detect_x_;
        float dy = cy - detect_y_;
        float retreat_dist = std::sqrt(dx * dx + dy * dy);

        if (retreat_dist >= retreat_distance_) {
            final_x_ = cx;
            final_y_ = cy;
            char msg[128];
            snprintf(msg, sizeof(msg), "후진 완료: %.2fm (목표 %.1fm)",
                     retreat_dist, retreat_distance_);
            setPhase(AdjustPhase::FINAL_STABILIZE, ctx, msg);
        }
    }

    // ========== 벽 결과 적용 ==========
    void applyWallResult(MissionContext& ctx) {
        // 벽 법선 (로컬) → NED 월드 좌표
        float current_yaw = ctx.current_yaw.load();
        float wall_normal_world = current_yaw + best_wall_.normal_local;

        // 드론이 벽을 바라보는 방향 = 법선 반대 방향
        target_perp_yaw_ = wall_normal_world + M_PI;
        while (target_perp_yaw_ > M_PI) target_perp_yaw_ -= 2.0f * M_PI;
        while (target_perp_yaw_ < -M_PI) target_perp_yaw_ += 2.0f * M_PI;

        wall_detected_ = true;

        RCLCPP_INFO(ctx.logger,
            "[DIST_ADJUST] 벽 분석: 법선=%.1f°(로컬), 수직yaw=%.1f°, 거리=%.2fm, 인라이어=%d",
            best_wall_.normal_local * 180.0f / M_PI,
            target_perp_yaw_ * 180.0f / M_PI,
            best_wall_.distance,
            best_wall_.inlier_count);
    }

    // ========== 최종 위치 기록 ==========
    void recordFinalPosition(MissionContext& ctx) {
        ctx.distance_adjust_result.completed = true;
        ctx.distance_adjust_result.final_lat = ctx.current_lat.load();
        ctx.distance_adjust_result.final_lon = ctx.current_lon.load();
        ctx.distance_adjust_result.final_alt_amsl = ctx.current_alt_amsl.load();
        ctx.distance_adjust_result.final_yaw = target_perp_yaw_;
        ctx.distance_adjust_result.wall_distance = wall_detected_ ?
            (best_wall_.distance + retreat_distance_) : 0.0f;

        final_x_ = ctx.current_local_x.load();
        final_y_ = ctx.current_local_y.load();

        RCLCPP_INFO(ctx.logger,
            "[DIST_ADJUST] 최종 위치 기록: GPS(%.7f, %.7f), yaw=%.1f°, 벽거리=%.1fm",
            ctx.distance_adjust_result.final_lat,
            ctx.distance_adjust_result.final_lon,
            target_perp_yaw_ * 180.0f / M_PI,
            ctx.distance_adjust_result.wall_distance);
    }

    // ========== 메디안 필터 ==========
    float getMedianFiltered(float new_value) {
        front_dist_history_.push_back(new_value);
        if (front_dist_history_.size() > MEDIAN_FILTER_SIZE) {
            front_dist_history_.erase(front_dist_history_.begin());
        }

        if (front_dist_history_.size() < 3) {
            return new_value;
        }

        std::vector<float> sorted = front_dist_history_;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }

    // ========== SLAM 방식 LiDAR 스캔 누적 ==========
    void accumulateScan(const LidarScan& scan) {
        // 신뢰도 감소 (매 호출마다)
        for (auto& b : wall_buckets_) {
            b.confidence *= CONFIDENCE_DECAY;
        }

        for (const auto& pt : scan.points) {
            if (pt.distance < 0.1f || pt.distance > 8.0f || pt.quality < 30) {
                continue;
            }

            int bucket = static_cast<int>(std::round(pt.angle)) % 360;
            if (bucket < 0) bucket += 360;

            auto& b = wall_buckets_[bucket];
            if (b.count > 0) {
                float diff = std::fabs(pt.distance - b.distance);
                float alpha = (diff > CHANGE_THRESHOLD) ?
                    SMOOTHING_ALPHA * 0.5f : SMOOTHING_ALPHA;
                b.distance = alpha * pt.distance + (1.0f - alpha) * b.distance;
                b.confidence = std::min(1.0f, b.confidence + 0.3f);
            } else {
                b.distance = pt.distance;
                b.confidence = 1.0f;
            }
            b.count++;
        }
    }

    // ========== RANSAC 벽 라인 피팅 ==========
    WallLine fitWallRANSAC(float drone_heading) {
        // 전방 호에서 고신뢰 포인트 추출
        // LiDAR 0° = 드론 전방, 각도는 반시계 방향
        struct Point2D { float x, y; };
        std::vector<Point2D> points;

        for (int i = -static_cast<int>(FRONT_ANGLE_RANGE);
             i <= static_cast<int>(FRONT_ANGLE_RANGE); i++) {
            int bucket = ((i % 360) + 360) % 360;
            const auto& b = wall_buckets_[bucket];

            if (b.confidence >= MIN_CONFIDENCE && b.distance > 0.5f && b.distance < 8.0f) {
                float angle_rad = static_cast<float>(i) * M_PI / 180.0f;
                // LiDAR 로컬 좌표: x=lateral(오른쪽+), y=forward(전방+)
                float x = b.distance * std::sin(angle_rad);
                float y = b.distance * std::cos(angle_rad);
                points.push_back({x, y});
            }
        }

        WallLine result{};
        if (static_cast<int>(points.size()) < MIN_WALL_INLIERS) {
            return result;
        }

        // RANSAC
        std::mt19937 rng(static_cast<unsigned>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
        std::uniform_int_distribution<int> dist(0, static_cast<int>(points.size()) - 1);

        int best_inliers = 0;
        float best_a = 0.0f, best_b_coeff = 0.0f;

        for (int iter = 0; iter < RANSAC_ITERATIONS; iter++) {
            int i1 = dist(rng);
            int i2 = dist(rng);
            if (i1 == i2) continue;

            const auto& p1 = points[i1];
            const auto& p2 = points[i2];

            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float len = std::sqrt(dx * dx + dy * dy);
            if (len < 0.1f) continue;

            // 라인 방정식: ax + by + c = 0 (정규화)
            float a = -dy / len;
            float b = dx / len;
            float c = -(a * p1.x + b * p1.y);

            // 인라이어 카운트
            int inliers = 0;
            for (const auto& p : points) {
                float d = std::fabs(a * p.x + b * p.y + c);
                if (d < RANSAC_INLIER_THRESH) {
                    inliers++;
                }
            }

            if (inliers > best_inliers) {
                best_inliers = inliers;
                best_a = a;
                best_b_coeff = b;
            }
        }

        if (best_inliers < MIN_WALL_INLIERS) {
            return result;
        }

        // 인라이어로 최소제곱 재피팅 (y = mx + b 형태)
        // 대신 법선 벡터 (best_a, best_b_coeff)에서 직접 각도 추출
        // 벽 법선 = (best_a, best_b_coeff) 또는 (-best_a, -best_b_coeff)
        // 드론 쪽을 향하는 법선 선택 (y 성분이 음수 = 드론 방향)
        float normal_x = best_a;
        float normal_y = best_b_coeff;

        // 법선이 드론을 향하도록 (y < 0 = 드론 쪽)
        if (normal_y > 0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }

        // LiDAR 로컬 프레임에서 법선 각도
        // x=lateral, y=forward → atan2(x, y) = 0이면 전방
        result.normal_local = std::atan2(normal_x, normal_y);
        result.valid = true;
        result.inlier_count = best_inliers;

        // 평균 수직 거리 계산
        float c = -(best_a * points[0].x + best_b_coeff * points[0].y);
        // 원점(드론)에서 라인까지 거리
        result.distance = std::fabs(c);

        return result;
    }

    // ========== 로깅 ==========
    void logProgress(MissionContext& ctx, double elapsed) {
        float front_dist = -1.0f;
        if (ctx.lidar_ptr) {
            front_dist = ctx.lidar_ptr->getFrontDistance();
        }

        RCLCPP_INFO(ctx.logger,
            "[DIST_ADJUST] %.0fs %s | NED(%.1f,%.1f) front=%.2fm yaw=%.1f°",
            elapsed, phaseName(phase_),
            ctx.current_local_x.load(), ctx.current_local_y.load(),
            front_dist,
            ctx.current_yaw.load() * 180.0f / M_PI);
    }
};

#endif // DISTANCE_ADJUST_HANDLER_H
