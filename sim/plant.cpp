#include "plant.hpp"
#include <algorithm>
#include <cmath>

void Plant::step(const Outputs& out, Inputs& in, double dt) {
    if(out.drive_cmd) {
        vel += (out.motor_cmd - vel) * std::clamp(dt * 5.0, 0.0, 1.0);
    } else {
        vel += (0.0 - vel) * std::clamp(dt * 5.0, 0.0, 1.0);
    }

    if(std::abs(vel) < 1e-4) vel = 0.0;
    in.velocity = vel;
}
