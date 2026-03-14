#pragma once
#include <array>
#include <memory>
#include <stdexcept>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include "gripper_sdk/params.hpp"

namespace gripper {

/**
 * Indirect Address + FastSyncRead + SyncWrite 기반 고속 그리퍼 통신
 *
 * 제어 루프 1회 = FastSyncRead 1회 + SyncWrite 1회 = USB 왕복 2회
 */
class Gripper {
public:
    Gripper(const std::string& port = PORT, int baudrate = BAUDRATE);
    ~Gripper();

    // 복사 금지
    Gripper(const Gripper&) = delete;
    Gripper& operator=(const Gripper&) = delete;

    // ── 초기화 / 종료 ──
    void enable();
    void disable();
    void setMode(uint8_t mode);
    void close();

    // ── 고속 읽기 (FastSyncRead 1회, 10byte × 11모터) ──
    struct MotorState {
        int16_t current[NUM_MOTORS];    // mA
        int32_t velocity[NUM_MOTORS];   // LSB
        int32_t position[NUM_MOTORS];   // tick
    };
    bool readState(MotorState& state);

    // ── 고속 쓰기 (SyncWrite 1회) ──
    bool writeCurrents(const int16_t currents[NUM_MOTORS]);
    bool writePositions(const int32_t positions[NUM_MOTORS]);

    // ── 로우레벨 접근 ──
    dynamixel::PortHandler*   portHandler()   { return port_handler_; }
    dynamixel::PacketHandler* packetHandler() { return packet_handler_; }

private:
    void setupIndirectAddress();

    dynamixel::PortHandler*   port_handler_;
    dynamixel::PacketHandler* packet_handler_;

    // FastSyncRead (재사용, 매 루프 txRxPacket만 호출)
    std::unique_ptr<dynamixel::GroupFastSyncRead> fast_reader_;
    // SyncWrite는 매번 clearParam 후 재구성
};

}  // namespace gripper
