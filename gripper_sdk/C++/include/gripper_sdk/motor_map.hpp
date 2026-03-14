#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>
#include "gripper_sdk/params.hpp"

namespace gripper {

// XL330 변환 상수
constexpr int32_t TICK_CENTER    = 2048;
constexpr double  TICK_PER_RAD   = 2048.0 / M_PI;
constexpr double  RAD_PER_TICK   = M_PI / 2048.0;
constexpr double  KT             = 0.52 / 1750.0;     // N·m / mA
constexpr double  KT_INV         = 1750.0 / 0.52;     // mA / N·m
constexpr double  VEL_LSB_TO_RADS = 0.229 * 2.0 * M_PI / 60.0;
constexpr double  VEL_RADS_TO_LSB = 1.0 / VEL_LSB_TO_RADS;

inline double tick_to_rad(int32_t tick) {
    return (tick - TICK_CENTER) * RAD_PER_TICK;
}

inline int32_t rad_to_tick(double rad) {
    return static_cast<int32_t>(rad * TICK_PER_RAD + TICK_CENTER);
}

inline int16_t torque_to_current(double torque_nm) {
    int val = static_cast<int>(torque_nm * KT_INV);
    return static_cast<int16_t>(std::clamp(val, -1750, 1750));
}

inline double current_to_torque(int16_t current_ma) {
    return current_ma * KT;
}

inline double vel_lsb_to_rads(int32_t lsb) {
    return lsb * VEL_LSB_TO_RADS;
}

inline int32_t vel_rads_to_lsb(double rads) {
    return static_cast<int32_t>(rads * VEL_RADS_TO_LSB);
}

}  // namespace gripper
