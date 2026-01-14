#pragma once
#include <algorithm>

// forward declarations
struct Inputs;
struct Outputs;

struct Plant {
    // states
    double velocity = 0.0;   // m/s
    double lift_pos = 0.0;   // 0.0 ~ 1.0
    double dump_pos = 0.0;   // 0.0 ~ 1.0

    // params (튜닝값)
    double v_target = 1.0;   // m/s (옵션 A)
    double alpha    = 0.15;  // 속도 1차 지연 계수
    double lift_rate = 0.6;  // pos/sec
    double dump_rate = 0.6;  // pos/sec

    // step
    void step(const Outputs& out, Inputs& in, double dt_sec);
};
