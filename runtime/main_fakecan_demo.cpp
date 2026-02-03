#include <iostream>
#include <thread>
#include <chrono>

#include "controller_core.hpp"
#include "plant.hpp"
#include "drivers/fakecan_bus.hpp"
#include "drivers/fakecan_codec.hpp"

static uint64_t now_us() {
  using namespace std::chrono;
  return duration_cast<microseconds>(
    steady_clock::now().time_since_epoch()
  ).count();
}


static constexpr double DT_S = 0.01;

int main() {
  ControllerCore core;
  Plant plant;
  FakeCanBus bus(FakeCanBus::Config{
    .delay_us  = 2000,
    .jitter_us = 3000,
    .drop_rate = 0.01
  });


  Inputs in{};
  Outputs out{};

  // 초기 command 주입
  in.drive_enable = true;
  in.comms_ok = true;
  in.battery_ok = true;
  in.target_velocity = 1.0;

  bus.push_rx(encode_cmd(in));

  static uint64_t last_cmd_us = 0;

  while (true) {
    bus.poll(now_us());

    // ---- RX ----
    auto rx = bus.pop_rx();
    if (rx) {
      decode_cmd(*rx, in);
      last_cmd_us = now_us();
    }

    if (auto rx = bus.pop_rx()) {
        decode_cmd(*rx, in);
        last_cmd_us = now_us();
        }

    in.comms_ok = (now_us() - last_cmd_us) <= 100000; // 100ms    

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
