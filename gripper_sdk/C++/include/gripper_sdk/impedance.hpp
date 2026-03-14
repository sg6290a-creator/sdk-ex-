#pragma once
#include <array>
#include <chrono>
#include <thread>
#include "gripper_sdk/motor_group.hpp"
#include "gripper_sdk/motor_map.hpp"
#include "gripper_sdk/params.hpp"

namespace gripper {

/**
 * 전류 제어 모드 기반 임피던스 제어기
 * τ_cmd = Kp*(q_d - q) + Kd*(dq_d - dq)
 */
class ImpedanceController {
public:
    ImpedanceController(MotorGroup& group,
                        double kp = IMP_KP,
                        double kd = IMP_KD,
                        int16_t current_limit = CURRENT_LIMIT,
                        double dt = CONTROL_DT);

    void reset();

    // 한 스텝 실행, 전류 명령 반환
    std::array<int16_t, NUM_MOTORS> step(
        const std::array<int32_t, NUM_MOTORS>& goal_positions,
        const std::array<int32_t, NUM_MOTORS>& goal_velocities = {});

    // duration 초 동안 제어 루프 실행
    void run(const std::array<int32_t, NUM_MOTORS>& goal_positions,
             double duration);

private:
    MotorGroup& group_;
    double kp_, kd_;
    int16_t current_limit_;
    double dt_;

    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point prev_time_;
    bool initialized_ = false;
};

}  // namespace gripper
