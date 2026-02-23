/**
 * @file motion_profile.h
 * @brief 속도/가속도 제한 setpoint 생성기 (사다리꼴 속도 프로파일)
 *
 * step-function setpoint으로 인한 급격한 틸트/추력 반응을 방지.
 * PX4 position controller에 부드러운 position + velocity 참조를 제공.
 *
 * 사용법:
 *   MotionProfile1D profile;
 *   profile.reset(current_position, max_vel, max_accel, dt);
 *   // 매 10Hz tick:
 *   profile.update(target_position);
 *   float pos = profile.getPosition();
 *   float vel = profile.getVelocity();
 */

#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <cmath>
#include <algorithm>
#include <array>

class MotionProfile1D {
public:
    /**
     * 시작 위치에서 프로파일 초기화.
     * @param position  현재 위치 (시작점)
     * @param max_vel   최대 속도 (m/s, 양수)
     * @param max_accel 최대 가속도 (m/s², 양수)
     * @param dt        시간 간격 (초), 10Hz → 0.1
     */
    void reset(float position, float max_vel, float max_accel, float dt) {
        pos_ = position;
        vel_ = 0.0f;
        max_vel_ = max_vel;
        max_accel_ = max_accel;
        dt_ = dt;
        initialized_ = true;
    }

    /**
     * 목표를 향해 한 tick 진행. 10Hz마다 호출.
     * 사다리꼴 속도 프로파일: 가속 → 순항 → 감속 → 정지.
     * @param target 목표 위치
     */
    void update(float target) {
        if (!initialized_) return;

        float error = target - pos_;
        float dir = (error > 0.0f) ? 1.0f : -1.0f;
        float abs_error = std::fabs(error);

        // 감속 거리: v² / (2a)
        float abs_vel = std::fabs(vel_);
        float decel_dist = (abs_vel * abs_vel) / (2.0f * max_accel_);

        float desired_vel;
        if (abs_error < 0.001f) {
            // 목표 도달: 정지
            desired_vel = 0.0f;
        } else if (abs_error <= decel_dist + abs_vel * dt_) {
            // 감속 구간: 목표에서 정지하도록 속도 감소
            // v_desired = sqrt(2 * a * 남은거리)
            desired_vel = dir * std::sqrt(2.0f * max_accel_ * abs_error);
            // 반대 방향으로 움직이고 있으면 정지
            if (dir * vel_ < 0.0f) {
                desired_vel = 0.0f;
            }
        } else {
            // 가속/순항 구간
            desired_vel = dir * max_vel_;
        }

        // 가속도 제한 적용
        float vel_error = desired_vel - vel_;
        float max_dv = max_accel_ * dt_;
        if (std::fabs(vel_error) > max_dv) {
            vel_ += (vel_error > 0.0f ? max_dv : -max_dv);
        } else {
            vel_ = desired_vel;
        }

        // 위치 적분
        pos_ += vel_ * dt_;

        // 오버슈트 방지: 목표를 지나쳤으면 클램프
        float new_error = target - pos_;
        if (error * new_error < 0.0f) {
            pos_ = target;
            vel_ = 0.0f;
        }
    }

    float getPosition() const { return pos_; }
    float getVelocity() const { return vel_; }
    bool isInitialized() const { return initialized_; }

    /** 목표에 도달했는지 확인 (허용 오차 내) */
    bool isSettled(float target, float pos_tol = 0.05f, float vel_tol = 0.05f) const {
        return std::fabs(target - pos_) < pos_tol && std::fabs(vel_) < vel_tol;
    }

private:
    float pos_{0.0f};
    float vel_{0.0f};
    float max_vel_{1.0f};
    float max_accel_{1.0f};
    float dt_{0.1f};
    bool initialized_{false};
};


/**
 * 3D 모션 프로파일 (XYZ 독립 축).
 * MotionProfile1D 3개의 래퍼.
 * XY와 Z축 제한값을 별도 설정 가능.
 */
class MotionProfile3D {
public:
    /**
     * 모든 축 시작 위치에서 초기화.
     * @param x, y, z         시작 위치
     * @param max_vel_xy      수평 최대 속도 (m/s)
     * @param max_accel_xy    수평 최대 가속도 (m/s²)
     * @param max_vel_z       수직 최대 속도 (m/s)
     * @param max_accel_z     수직 최대 가속도 (m/s²)
     * @param dt              시간 간격 (초)
     */
    void reset(float x, float y, float z,
               float max_vel_xy, float max_accel_xy,
               float max_vel_z, float max_accel_z,
               float dt) {
        x_.reset(x, max_vel_xy, max_accel_xy, dt);
        y_.reset(y, max_vel_xy, max_accel_xy, dt);
        z_.reset(z, max_vel_z, max_accel_z, dt);
    }

    /** 모든 축 목표를 향해 업데이트 */
    void update(float tx, float ty, float tz) {
        x_.update(tx);
        y_.update(ty);
        z_.update(tz);
    }

    std::array<float, 3> getPosition() const {
        return {x_.getPosition(), y_.getPosition(), z_.getPosition()};
    }

    std::array<float, 3> getVelocity() const {
        return {x_.getVelocity(), y_.getVelocity(), z_.getVelocity()};
    }

    bool isInitialized() const {
        return x_.isInitialized();
    }

    bool isSettled(float tx, float ty, float tz,
                   float pos_tol = 0.1f, float vel_tol = 0.05f) const {
        return x_.isSettled(tx, pos_tol, vel_tol) &&
               y_.isSettled(ty, pos_tol, vel_tol) &&
               z_.isSettled(tz, pos_tol, vel_tol);
    }

private:
    MotionProfile1D x_, y_, z_;
};

#endif // MOTION_PROFILE_H
