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
 * 3D 모션 프로파일 (XY 결합 + Z 독립).
 *
 * XY: 현재→목표 직선 방향으로 단일 사다리꼴 속도 프로파일 적용.
 *     속도 벡터가 항상 목표를 가리키므로 직선 경로 유지.
 * Z:  독립 MotionProfile1D (고도 제어).
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
        px_ = x; py_ = y;
        vx_ = 0.0f; vy_ = 0.0f;
        max_vel_xy_ = max_vel_xy;
        max_accel_xy_ = max_accel_xy;
        dt_ = dt;
        initialized_ = true;
        z_.reset(z, max_vel_z, max_accel_z, dt);
    }

    /**
     * 목표를 향해 한 tick 진행 (10Hz).
     * XY: 목표 방향으로 사다리꼴 가감속, 직선 경로 유지.
     * Z: 독립 1D 프로파일.
     */
    void update(float tx, float ty, float tz) {
        if (!initialized_) return;
        z_.update(tz);

        float dx = tx - px_;
        float dy = ty - py_;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 0.001f) {
            // 목표 도달: 정지
            px_ = tx; py_ = ty;
            vx_ = 0.0f; vy_ = 0.0f;
            return;
        }

        // 방향 단위 벡터 (현재 프로파일 위치 → 목표)
        float nx = dx / dist;
        float ny = dy / dist;

        // 현재 속도의 목표 방향 성분 (양수=목표 접근, 음수=이탈)
        float speed_along = vx_ * nx + vy_ * ny;

        // 사다리꼴 속도 프로파일: 감속 거리 계산
        float abs_speed = std::fabs(speed_along);
        float decel_dist = (abs_speed * abs_speed) / (2.0f * max_accel_xy_);

        float desired_speed;
        if (speed_along < 0.0f) {
            // 목표에서 멀어지는 중: 빨리 감속 → 정지
            desired_speed = 0.0f;
        } else if (dist <= decel_dist + abs_speed * dt_) {
            // 감속 구간: 목표에서 정지하도록 속도 감소
            desired_speed = std::sqrt(2.0f * max_accel_xy_ * dist);
        } else {
            // 가속/순항 구간
            desired_speed = max_vel_xy_;
        }

        // 가속도 제한 적용 (along-track 방향)
        float speed_error = desired_speed - speed_along;
        float max_dv = max_accel_xy_ * dt_;
        float new_speed;
        if (std::fabs(speed_error) > max_dv) {
            new_speed = speed_along + (speed_error > 0.0f ? max_dv : -max_dv);
        } else {
            new_speed = desired_speed;
        }

        // 속도 = 스칼라 속도 × 방향 벡터 (항상 목표를 가리킴)
        vx_ = new_speed * nx;
        vy_ = new_speed * ny;

        // 위치 적분
        px_ += vx_ * dt_;
        py_ += vy_ * dt_;

        // 오버슈트 방지: 목표를 지나쳤으면 클램프
        float new_dx = tx - px_;
        float new_dy = ty - py_;
        if (dx * new_dx + dy * new_dy < 0.0f) {
            px_ = tx; py_ = ty;
            vx_ = 0.0f; vy_ = 0.0f;
        }
    }

    std::array<float, 3> getPosition() const {
        return {px_, py_, z_.getPosition()};
    }

    std::array<float, 3> getVelocity() const {
        return {vx_, vy_, z_.getVelocity()};
    }

    bool isInitialized() const {
        return initialized_;
    }

    bool isSettled(float tx, float ty, float tz,
                   float pos_tol = 0.1f, float vel_tol = 0.05f) const {
        float dx = tx - px_;
        float dy = ty - py_;
        float dist = std::sqrt(dx * dx + dy * dy);
        float speed = std::sqrt(vx_ * vx_ + vy_ * vy_);
        return dist < pos_tol && speed < vel_tol &&
               z_.isSettled(tz, pos_tol, vel_tol);
    }

private:
    // XY 결합 (직선 경로)
    float px_{0.0f}, py_{0.0f};       // 프로파일 위치
    float vx_{0.0f}, vy_{0.0f};       // 프로파일 속도
    float max_vel_xy_{1.0f};
    float max_accel_xy_{1.0f};
    float dt_{0.1f};
    bool initialized_{false};

    // Z 독립
    MotionProfile1D z_;
};

#endif // MOTION_PROFILE_H
