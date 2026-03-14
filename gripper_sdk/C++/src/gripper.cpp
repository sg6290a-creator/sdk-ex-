#include "gripper_sdk/gripper.hpp"
#include <dynamixel_sdk/group_fast_sync_read.h>
#include <dynamixel_sdk/group_sync_write.h>
#include <cstdio>
#include <unistd.h>  // usleep

namespace gripper {

Gripper::Gripper(const std::string& port, int baudrate)
{
    port_handler_ = dynamixel::PortHandler::getPortHandler(port.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0f);

    if (!port_handler_->openPort())
        throw std::runtime_error("Failed to open port: " + port);
    if (!port_handler_->setBaudRate(baudrate))
        throw std::runtime_error("Failed to set baudrate");

    printf("[Gripper] Connected: %s @ %d bps\n", port.c_str(), baudrate);

    // Indirect Address 설정 (토크 OFF 상태에서)
    setupIndirectAddress();

    // FastSyncRead 초기화 (Indirect Data Read 영역)
    fast_reader_ = std::make_unique<dynamixel::GroupFastSyncRead>(
        port_handler_, packet_handler_,
        ADDR_INDIRECT_DATA_READ, INDIRECT_READ_LEN);

    for (int i = 0; i < NUM_MOTORS; i++) {
        fast_reader_->addParam(MOTOR_IDS[i]);
    }
}

Gripper::~Gripper()
{
    close();
}

void Gripper::setupIndirectAddress()
{
    // Indirect Address Read 영역에 매핑:
    // [0..1]  → Present Current  (126, 127)
    // [2..5]  → Present Velocity (128, 129, 130, 131)
    // [6..9]  → Present Position (132, 133, 134, 135)
    const uint16_t src_addrs[INDIRECT_READ_LEN] = {
        126, 127,               // current  (2 byte)
        128, 129, 130, 131,     // velocity (4 byte)
        132, 133, 134, 135      // position (4 byte)
    };

    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t id = MOTOR_IDS[i];
        for (int j = 0; j < INDIRECT_READ_LEN; j++) {
            uint8_t dxl_error = 0;
            int result = packet_handler_->write2ByteTxRx(
                port_handler_, id,
                ADDR_INDIRECT_ADDR_READ + 2 * j,
                src_addrs[j], &dxl_error);
            if (result != COMM_SUCCESS) {
                printf("[Gripper] Indirect addr setup failed: ID=%d idx=%d err=%d\n",
                       id, j, result);
            }
        }
    }
    printf("[Gripper] Indirect Address configured for %d motors\n", NUM_MOTORS);
}

void Gripper::enable()
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t dxl_error = 0;
        packet_handler_->write1ByteTxRx(
            port_handler_, MOTOR_IDS[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
    }
}

void Gripper::disable()
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t dxl_error = 0;
        packet_handler_->write1ByteTxRx(
            port_handler_, MOTOR_IDS[i], ADDR_TORQUE_ENABLE, 0, &dxl_error);
    }
}

void Gripper::setMode(uint8_t mode)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t dxl_error = 0;
        packet_handler_->write1ByteTxRx(
            port_handler_, MOTOR_IDS[i], ADDR_OPERATING_MODE, mode, &dxl_error);
    }
}

void Gripper::close()
{
    disable();
    port_handler_->closePort();
}

bool Gripper::readState(MotorState& state)
{
    // 이전 write 패킷 잔여 데이터 제거
    port_handler_->clearPort();

    int result = fast_reader_->txRxPacket();
    if (result != COMM_SUCCESS) {
        printf("[Gripper] FastSyncRead failed: %s\n",
               packet_handler_->getTxRxResult(result));
        return false;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t id = MOTOR_IDS[i];
        uint16_t base = ADDR_INDIRECT_DATA_READ;

        if (!fast_reader_->isAvailable(id, base, INDIRECT_READ_LEN)) {
            printf("[Gripper] Data not available for ID=%d\n", id);
            return false;
        }

        // Indirect Data 순서: current(2) + velocity(4) + position(4)
        state.current[i]  = static_cast<int16_t>(
            fast_reader_->getData(id, base, 2));
        state.velocity[i] = static_cast<int32_t>(
            fast_reader_->getData(id, base + 2, 4));
        state.position[i] = static_cast<int32_t>(
            fast_reader_->getData(id, base + 6, 4));
    }
    return true;
}

bool Gripper::writeCurrents(const int16_t currents[NUM_MOTORS])
{
    dynamixel::GroupSyncWrite writer(
        port_handler_, packet_handler_, ADDR_GOAL_CURRENT, 2);

    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t data[2] = {
            static_cast<uint8_t>(currents[i] & 0xFF),
            static_cast<uint8_t>((currents[i] >> 8) & 0xFF)
        };
        writer.addParam(MOTOR_IDS[i], data);
    }

    int result = writer.txPacket();
    if (result == COMM_SUCCESS) {
        // TX-only 패킷의 물리적 전송 완료 대기
        // 45byte @ 4Mbps ≈ 0.11ms, 여유 포함
        usleep(150);
    }
    return result == COMM_SUCCESS;
}

bool Gripper::writePositions(const int32_t positions[NUM_MOTORS])
{
    dynamixel::GroupSyncWrite writer(
        port_handler_, packet_handler_, ADDR_GOAL_POSITION, 4);

    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t data[4] = {
            static_cast<uint8_t>( positions[i]        & 0xFF),
            static_cast<uint8_t>((positions[i] >>  8) & 0xFF),
            static_cast<uint8_t>((positions[i] >> 16) & 0xFF),
            static_cast<uint8_t>((positions[i] >> 24) & 0xFF)
        };
        writer.addParam(MOTOR_IDS[i], data);
    }

    int result = writer.txPacket();
    if (result == COMM_SUCCESS) {
        usleep(200);  // 67byte @ 4Mbps ≈ 0.17ms
    }
    return result == COMM_SUCCESS;
}

}  // namespace gripper
