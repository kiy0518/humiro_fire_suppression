/**
 * @file prepare_handler.h
 * @brief PREPARING 상태 핸들러
 *
 * PX4 OFFBOARD 모드 진입 전 준비 단계.
 * 2초간 OffboardControlMode 하트비트를 발행하여 PX4가 수신 상태가 되도록 함.
 * (PX4는 최소 0.5초 이상 하트비트를 수신해야 OFFBOARD 모드 진입 허용)
 *
 * 전환 조건: 2초 경과 후 OFFBOARD 모드 전환 명령 전송 → COMPLETE
 */

#ifndef PREPARE_HANDLER_H
#define PREPARE_HANDLER_H

#include "state_handler.h"
#include "../mission_context.h"

class PrepareHandler : public StateHandler {
public:
    void onEnter(MissionContext& ctx) override {
        ctx.state_enter_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(ctx.logger, "[PREPARE] Heartbeat 발행 시작 (2초 대기)...");
    }

    void onExit(MissionContext& ctx) override {
        (void)ctx;
    }

    TransitionResult tick(MissionContext& ctx) override {
        double elapsed = ctx.elapsedSec();

        // 진행 로그 (0.5초마다)
        if (static_cast<int>(elapsed * 2) % 1 == 0 &&
            elapsed - std::floor(elapsed * 2) / 2.0 < 0.15) {
            // 간단한 주기 로그
        }

        // 조건: 2초 경과
        if (elapsed >= 2.0) {
            RCLCPP_INFO(ctx.logger, "[PREPARE] 2초 경과 → OFFBOARD 모드 전환 요청");
            ctx.publishCommand(176 /*VEHICLE_CMD_DO_SET_MODE*/, 1.0f, 6.0f, 0.0f);
            return TransitionResult::COMPLETE;
        }

        return TransitionResult::STAY;
    }

    bool fillSetpoint(MissionContext& ctx,
                      px4_msgs::msg::TrajectorySetpoint& sp) override {
        // PREPARE 중: 현재 위치 유지
        sp.position = {
            ctx.current_local_x.load(),
            ctx.current_local_y.load(),
            ctx.current_local_z.load()
        };
        sp.velocity = {NAN, NAN, NAN};
        sp.yaw = ctx.current_yaw.load();
        sp.yawspeed = 0.0f;
        return true;
    }

    const char* name() const override { return "PREPARING"; }
};

#endif // PREPARE_HANDLER_H
