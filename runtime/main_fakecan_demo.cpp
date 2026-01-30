#include <iostream>
#include <thread>
#include <chrono>

#include "controller_core.hpp"
#include "plant.hpp"
#include "drivers/fakecan_bus.hpp"
#include "drivers/fakecan_codec.hpp"


static constexpr double DT_S = 0.01;

int main() {
  ControllerCore core;
  Plant plant;
  FakeCanBus bus;

  Inputs in{};
  Outputs out{};

  // 초기 command 주입
  in.drive_enable = true;
  in.comms_ok = true;
  in.battery_ok = true;
  in.target_velocity = 1.0;

  bus.push_rx(encode_cmd(in));

  while (true) {
    // ---- RX ----
    auto rx = bus.pop_rx();
    if (rx) {
      decode_cmd(*rx, in);
    }

    // ---- Control ----
    out = core.step(in, DT_S);
    plant.step(out, in, DT_S);

    // ---- TX ----
    auto tx = encode_act(out);
    bus.push_tx(tx);

    // ---- Monitor ----
    std::cout
      << "vel=" << in.velocity
      << " target=" << in.target_velocity
      << " cmd=" << out.motor_cmd
      << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
