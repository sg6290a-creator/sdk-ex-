/**
 * 400Hz 통신 안정성 테스트 (1분)
 * - 루프 드랍(deadline miss) 횟수 측정
 * - 지터 통계 출력
 */
#include <chrono>
#include <cstdio>
#include <numeric>
#include <vector>
#include <cmath>
#include <algorithm>
#include <thread>

#include "gripper_sdk/gripper.hpp"
#include "gripper_sdk/params.hpp"

using Clock = std::chrono::high_resolution_clock;
using us    = std::chrono::microseconds;

constexpr double TARGET_HZ      = 500.0;
constexpr double TARGET_DT_US   = 1e6 / TARGET_HZ;   // 2500 us
constexpr double DEADLINE_US    = TARGET_DT_US * 1.5; // 3750 us — 이 이상이면 드랍
constexpr double TEST_DURATION_S = 60.0;

int main()
{
    using namespace gripper;

    printf("=== %.0fHz 안정성 테스트 (1분) ===\n", TARGET_HZ);
    printf("포트: %s  보드레이트: %d\n", PORT.c_str(), BAUDRATE);
    printf("목표 주기: %.0f us (%.0f Hz)\n", TARGET_DT_US, TARGET_HZ);
    printf("드랍 기준: %.0f us 초과\n\n", DEADLINE_US);

    Gripper grip(PORT, BAUDRATE);
    grip.disable();
    grip.setMode(MODE_CURRENT);
    grip.enable();

    Gripper::MotorState state;
    int16_t zero_currents[NUM_MOTORS] = {};

    int    total_loops  = 0;
    int    drops        = 0;
    int    read_fails   = 0;
    double max_dt_us    = 0.0;
    double sum_dt_us    = 0.0;
    double sum_sq_dt_us = 0.0;

    // 10초마다 중간 보고
    auto last_report = Clock::now();

    auto test_start = Clock::now();
    auto next_wake  = test_start;

    while (true) {
        auto now = Clock::now();
        double elapsed_s = std::chrono::duration<double>(now - test_start).count();
        if (elapsed_s >= TEST_DURATION_S) break;

        // 루프 시작
        auto loop_start = Clock::now();

        grip.writeCurrents(zero_currents);
        bool ok = grip.readState(state);
        if (!ok) read_fails++;

        auto loop_end = Clock::now();
        double dt_us = std::chrono::duration<double, std::micro>(loop_end - loop_start).count();

        total_loops++;
        sum_dt_us    += dt_us;
        sum_sq_dt_us += dt_us * dt_us;
        if (dt_us > max_dt_us) max_dt_us = dt_us;
        if (dt_us > DEADLINE_US) drops++;

        // 10초마다 중간 보고
        if (std::chrono::duration<double>(loop_end - last_report).count() >= 10.0) {
            double mean = sum_dt_us / total_loops;
            double drop_rate = 100.0 * drops / total_loops;
            printf("[%.0fs] loops=%d  mean=%.1fus  max=%.1fus  drops=%d (%.2f%%)  read_fails=%d\n",
                   elapsed_s, total_loops, mean, max_dt_us, drops, drop_rate, read_fails);
            last_report = loop_end;
        }

        // 다음 루프 시작 시각까지 대기
        next_wake += us(static_cast<long>(TARGET_DT_US));
        std::this_thread::sleep_until(next_wake);
    }

    grip.disable();
    grip.close();

    // 최종 통계
    double mean   = sum_dt_us / total_loops;
    double var    = (sum_sq_dt_us / total_loops) - (mean * mean);
    double stddev = std::sqrt(var);
    double drop_rate = 100.0 * drops / total_loops;
    double actual_hz = total_loops / TEST_DURATION_S;

    printf("\n=== 최종 결과 ===\n");
    printf("  총 루프:       %d\n",      total_loops);
    printf("  실제 Hz:       %.1f Hz\n", actual_hz);
    printf("  평균 루프:     %.1f us\n", mean);
    printf("  표준편차:      %.1f us\n", stddev);
    printf("  최대 루프:     %.1f us\n", max_dt_us);
    printf("  드랍 횟수:     %d / %d (%.3f%%)\n", drops, total_loops, drop_rate);
    printf("  Read 실패:     %d\n",      read_fails);

    return 0;
}
