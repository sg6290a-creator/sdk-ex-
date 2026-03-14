/**
 * 통신 루프 타이밍 측정
 *
 * 측정 항목:
 *   - FastSyncRead (pos+vel+cur 동시) 소요 시간
 *   - SyncWrite (current) 소요 시간
 *   - 풀 루프 (read + write) 소요 시간
 *   - 달성 가능한 최대 Hz 추정
 */
#include <chrono>
#include <cstdio>
#include <numeric>
#include <vector>
#include <algorithm>
#include <cmath>

#include "gripper_sdk/gripper.hpp"
#include "gripper_sdk/motor_group.hpp"
#include "gripper_sdk/params.hpp"

using Clock = std::chrono::high_resolution_clock;

struct Stats {
    double mean, std_dev, min_val, max_val;
};

Stats measure(const char* label, std::function<void()> fn, int n = 200)
{
    std::vector<double> times;
    times.reserve(n);

    for (int i = 0; i < n; i++) {
        auto t0 = Clock::now();
        fn();
        auto t1 = Clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        times.push_back(ms);
    }

    double sum = std::accumulate(times.begin(), times.end(), 0.0);
    double mean = sum / n;
    double sq_sum = 0;
    for (auto t : times) sq_sum += (t - mean) * (t - mean);
    double std_dev = std::sqrt(sq_sum / n);
    double min_val = *std::min_element(times.begin(), times.end());
    double max_val = *std::max_element(times.begin(), times.end());

    printf("  %-30s mean=%.2fms  std=%.2fms  min=%.2fms  max=%.2fms\n",
           label, mean, std_dev, min_val, max_val);

    return {mean, std_dev, min_val, max_val};
}

int main()
{
    using namespace gripper;

    printf("=== C++ 루프 타이밍 측정 ===\n");
    printf("포트: %s  보드레이트: %d  모터 수: %d\n\n",
           PORT.c_str(), BAUDRATE, NUM_MOTORS);

    Gripper grip(PORT, BAUDRATE);
    grip.disable();
    grip.setMode(MODE_CURRENT);
    grip.enable();

    MotorGroup group(grip);
    Gripper::MotorState state;
    int16_t zero_currents[NUM_MOTORS] = {};

    printf("[개별 측정] (n=200)\n");

    auto t_read = measure("FastSyncRead (pos+vel+cur)", [&]() {
        grip.readState(state);
    });

    auto t_write = measure("SyncWrite (current)", [&]() {
        grip.writeCurrents(zero_currents);
    });

    printf("\n[풀 루프 측정] FastSyncRead + SyncWrite\n");
    printf("  (순서: Write → Read — Read의 clearPort가 Write 잔여 TX를 정리)\n");

    auto t_full = measure("Full loop", [&]() {
        grip.writeCurrents(zero_currents);
        grip.readState(state);
    });

    printf("\n[결과 요약]\n");
    printf("  이론 최대 Hz (풀루프):  %.1f Hz\n", 1000.0 / t_full.mean);
    printf("  권장 제어 주기:         %.1fms (%.1f Hz)\n",
           t_full.mean * 1.5, 1000.0 / (t_full.mean * 1.5));
    printf("  Read-only 최대 Hz:      %.1f Hz\n", 1000.0 / t_read.mean);

    grip.disable();
    grip.close();

    return 0;
}
