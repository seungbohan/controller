#include "plant.hpp"
#include <algorithm>
#include <cmath>

void Plant::step(const Outputs& out, Inputs& in, double dt) {
    const double u = out.motor_cmd ? out.motor_cmd : 0.0;

    const double max_accel = 2.0;
    const double drag = 1.2;

    double accel = u * max_accel - drag * vel;

    vel += accel * dt;

    if (std::abs(vel) < 1e-4) vel = 0.0;

    in.velocity = vel;
}
