#pragma once
#include <array>
#include "gripper_sdk/gripper.hpp"

namespace gripper {

/**
 * MotorGroup: Gripper 위에 편의 래퍼
 * Python의 MotorGroup과 동일한 인터페이스
 */
class MotorGroup {
public:
    explicit MotorGroup(Gripper& gripper);

    // 한 번에 전체 상태 읽기 (FastSyncRead 1회)
    bool readAll(Gripper::MotorState& state);

    // 개별 데이터 읽기 (내부적으로 readAll 호출)
    std::array<int32_t, NUM_MOTORS> readPositions();
    std::array<int32_t, NUM_MOTORS> readVelocities();
    std::array<int16_t, NUM_MOTORS> readCurrents();

    // 쓰기
    bool writeCurrents(const std::array<int16_t, NUM_MOTORS>& currents);
    bool writePositions(const std::array<int32_t, NUM_MOTORS>& positions);

private:
    Gripper& gripper_;
    Gripper::MotorState cached_state_;
};

}  // namespace gripper
