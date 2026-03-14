/**
 * 500Hz 사인파 모션 테스트 + PlotJuggler 실시간 시각화
 *
 * PlotJuggler 설정:
 *   Start → UDP Server → port 9870 → JSON format
 *
 * 빌드 후 실행:
 *   ./test_sine
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <csignal>
#include <atomic>

#include "gripper_sdk/gripper.hpp"
#include "gripper_sdk/motor_group.hpp"
#include "gripper_sdk/motor_map.hpp"
#include "gripper_sdk/params.hpp"
#include "gripper_sdk/udp_logger.hpp"

using namespace gripper;
using Clock = std::chrono::high_resolution_clock;
using us    = std::chrono::microseconds;

constexpr double TARGET_HZ    = 500.0;
constexpr double TARGET_DT_US = 1e6 / TARGET_HZ;
constexpr double SINE_AMP_RAD = 0.3;
constexpr double SINE_FREQ_HZ = 0.5;
constexpr double DURATION_S   = 30.0;

std::atomic<bool> running{true};
void sigint_handler(int) { running = false; }

int main()
{
    std::signal(SIGINT, sigint_handler);

    UdpLogger logger;  // 127.0.0.1:9870

    Gripper grip(PORT, BAUDRATE);
    grip.disable();
    grip.setMode(MODE_CURRENT_POSITION);
    grip.enable();

    // 초기 위치를 중심으로 사인파
    Gripper::MotorState init_state;
    grip.readState(init_state);
    int32_t center[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++)
        center[i] = init_state.position[i];

    printf("[Sine] amp=%.2f rad  freq=%.1f Hz  duration=%.0f s\n",
           SINE_AMP_RAD, SINE_FREQ_HZ, DURATION_S);

    auto test_start = Clock::now();
    auto next_wake  = test_start;

    while (running) {
        auto now = Clock::now();
        double t_s = std::chrono::duration<double>(now - test_start).count();
        if (t_s >= DURATION_S) break;

        // 사인파 목표 위치
        int32_t goal[NUM_MOTORS];
        int32_t offset = rad_to_tick(SINE_AMP_RAD * std::sin(2.0 * M_PI * SINE_FREQ_HZ * t_s))
                         - TICK_CENTER;
        for (int i = 0; i < NUM_MOTORS; i++)
            goal[i] = center[i] + offset;

        grip.writePositions(goal);

        Gripper::MotorState state;
        grip.readState(state);

        // UDP → PlotJuggler (부하 거의 0)
        logger.send(t_s, state.position, goal, state.current, state.velocity);

        next_wake += us(static_cast<long>(TARGET_DT_US));
        std::this_thread::sleep_until(next_wake);
    }

    grip.disable();
    grip.close();
    printf("\n[Done]\n");
    return 0;
}
