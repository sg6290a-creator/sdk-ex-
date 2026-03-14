#include "gripper_sdk/impedance.hpp"
#include <algorithm>
#include <cstdio>

namespace gripper {

ImpedanceController::ImpedanceController(
    MotorGroup& group, double kp, double kd,
    int16_t current_limit, double dt)
    : group_(group), kp_(kp), kd_(kd),
      current_limit_(current_limit), dt_(dt)
{}

void ImpedanceController::reset()
{
    Gripper::MotorState state;
    group_.readAll(state);
    prev_time_ = Clock::now();
    initialized_ = true;
}

std::array<int16_t, NUM_MOTORS> ImpedanceController::step(
    const std::array<int32_t, NUM_MOTORS>& goal_positions,
    const std::array<int32_t, NUM_MOTORS>& goal_velocities)
{
    // 한 번의 FastSyncRead로 전체 상태 획득
    Gripper::MotorState state;
    group_.readAll(state);

    std::array<int16_t, NUM_MOTORS> cmds;
    for (int i = 0; i < NUM_MOTORS; i++) {
        double pos_err = goal_positions[i] - state.position[i];
        double vel_err = goal_velocities[i] - state.velocity[i];
        double cmd = kp_ * pos_err + kd_ * vel_err;
        cmds[i] = static_cast<int16_t>(
            std::clamp(static_cast<int>(cmd),
                       static_cast<int>(-current_limit_),
                       static_cast<int>(current_limit_)));
    }

    // Write → 다음 루프의 Read가 TX 완료를 보장
    group_.writeCurrents(cmds);
    return cmds;
}

void ImpedanceController::run(
    const std::array<int32_t, NUM_MOTORS>& goal_positions,
    double duration)
{
    reset();
    auto end_time = Clock::now() +
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(duration));

    std::array<int32_t, NUM_MOTORS> zero_vel{};

    while (Clock::now() < end_time) {
        auto loop_start = Clock::now();
        step(goal_positions, zero_vel);

        // dt 맞추기
        auto elapsed = Clock::now() - loop_start;
        auto target = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(dt_));
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        }
    }

    // 종료 시 전류 0
    std::array<int16_t, NUM_MOTORS> zero{};
    group_.writeCurrents(zero);
}

}  // namespace gripper
