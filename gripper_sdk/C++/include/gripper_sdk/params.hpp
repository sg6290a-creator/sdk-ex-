#pragma once
#include <cstdint>
#include <string>

namespace gripper {

// 포트 설정
inline const std::string PORT     = "/dev/ttyUSB0";
constexpr int            BAUDRATE = 4000000;

// 모터 ID
constexpr int     NUM_MOTORS = 11;
constexpr uint8_t MOTOR_IDS[NUM_MOTORS] = {1,2,3,4,5,6,7,8,9,10,11};

// XL330-M288 Control Table 주소
constexpr uint16_t ADDR_OPERATING_MODE = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE  = 64;
constexpr uint16_t ADDR_GOAL_CURRENT   = 102;  // 2 byte
constexpr uint16_t ADDR_GOAL_POSITION  = 116;  // 4 byte
constexpr uint16_t ADDR_PRESENT_CUR    = 126;  // 2 byte
constexpr uint16_t ADDR_PRESENT_VEL    = 128;  // 4 byte
constexpr uint16_t ADDR_PRESENT_POS    = 132;  // 4 byte

// Indirect Address (Read 전용 영역)
// current(126,2) + velocity(128,4) + position(132,4) = 10 bytes
constexpr uint16_t ADDR_INDIRECT_ADDR_READ = 180;
constexpr uint16_t ADDR_INDIRECT_DATA_READ = 230;
constexpr uint16_t INDIRECT_READ_LEN       = 10;

// Operating modes
constexpr uint8_t MODE_CURRENT          = 0;
constexpr uint8_t MODE_VELOCITY         = 1;
constexpr uint8_t MODE_POSITION         = 3;
constexpr uint8_t MODE_EXT_POSITION     = 4;
constexpr uint8_t MODE_CURRENT_POSITION = 5;

// 전류 제한 (mA)
constexpr int16_t CURRENT_LIMIT = 500;

// 임피던스 제어 기본 게인
constexpr double IMP_KP = 5.0;   // mA/tick
constexpr double IMP_KD = 0.3;   // mA/(tick/s)

// 제어 루프 주기
constexpr double CONTROL_DT = 0.01;  // 100 Hz

}  // namespace gripper
