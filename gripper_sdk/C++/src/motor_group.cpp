#include "gripper_sdk/motor_group.hpp"

namespace gripper {

MotorGroup::MotorGroup(Gripper& gripper)
    : gripper_(gripper), cached_state_{}
{}

bool MotorGroup::readAll(Gripper::MotorState& state)
{
    return gripper_.readState(state);
}

std::array<int32_t, NUM_MOTORS> MotorGroup::readPositions()
{
    gripper_.readState(cached_state_);
    std::array<int32_t, NUM_MOTORS> out;
    for (int i = 0; i < NUM_MOTORS; i++)
        out[i] = cached_state_.position[i];
    return out;
}

std::array<int32_t, NUM_MOTORS> MotorGroup::readVelocities()
{
    gripper_.readState(cached_state_);
    std::array<int32_t, NUM_MOTORS> out;
    for (int i = 0; i < NUM_MOTORS; i++)
        out[i] = cached_state_.velocity[i];
    return out;
}

std::array<int16_t, NUM_MOTORS> MotorGroup::readCurrents()
{
    gripper_.readState(cached_state_);
    std::array<int16_t, NUM_MOTORS> out;
    for (int i = 0; i < NUM_MOTORS; i++)
        out[i] = cached_state_.current[i];
    return out;
}

bool MotorGroup::writeCurrents(const std::array<int16_t, NUM_MOTORS>& currents)
{
    return gripper_.writeCurrents(currents.data());
}

bool MotorGroup::writePositions(const std::array<int32_t, NUM_MOTORS>& positions)
{
    return gripper_.writePositions(positions.data());
}

}  // namespace gripper
