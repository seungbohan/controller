#include "plant.hpp"
#include <algorithm>
#include <cmath>

void Plant::step(const Outputs& out, Inputs& in, double dt) {
    // -------------------------
    // drive velocity (1st order)
    // -------------------------
    const double target_v = out.drive_cmd ? 1.0 : 0.0;
    // simple response
    vel += (target_v - vel) * std::clamp(dt * 2.0, 0.0, 1.0);
    if (std::abs(vel) < 1e-4) vel = 0.0;

    in.velocity = vel;

    // -------------------------
    // lift/dump positions (hold-to-run)
    // -------------------------
    if (out.lift_cmd) lift_pos += dt * 0.6;
    if (out.dump_cmd) dump_pos += dt * 1.2;

    lift_pos = std::clamp(lift_pos, 0.0, 1.0);
    dump_pos = std::clamp(dump_pos, 0.0, 1.0);

    // plant generates completes when reaching 1.0 (optional)
    // (너 demo에서는 main에서 complete를 직접 토글하니까, 여기서는 자동 complete는 꺼둬도 됨)
}
