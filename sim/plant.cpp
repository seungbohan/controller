#include "plant.hpp"
#include "../src/main_inputs_outputs.hpp" // 아래 3번 참고

void Plant::step(const Outputs& out, Inputs& in, double dt) {
    // ---- velocity (1st order lag) ----
    if (out.drive_cmd) {
        velocity += (v_target - velocity) * alpha;
    } else {
        velocity += (0.0 - velocity) * alpha;
    }

    // 작은 값 정리
    if (std::abs(velocity) < 1e-3) velocity = 0.0;

    // ---- lift position ----
    if (out.lift_cmd) {
        lift_pos += lift_rate * dt;
    }
    lift_pos = std::clamp(lift_pos, 0.0, 1.0);
    in.lift_complete = (lift_pos >= 1.0);

    // ---- dump position ----
    if (out.dump_cmd) {
        dump_pos += dump_rate * dt;
    }
    dump_pos = std::clamp(dump_pos, 0.0, 1.0);
    in.dump_complete = (dump_pos >= 1.0);

    // publish measured back to controller inputs
    in.velocity = velocity;
}
