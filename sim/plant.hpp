#pragma once
#include "../src/main_inputs_outputs.hpp"

struct Plant {
    // simple plant states (0~1)
    double lift_pos = 0.0;
    double dump_pos = 0.0;

    // velocity simulation
    double vel = 0.0;

    void step(const Outputs& out, Inputs& in, double dt);
};
