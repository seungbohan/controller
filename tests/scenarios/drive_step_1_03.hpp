#pragma once
#include "../../include/main_inputs_outputs.hpp"

struct DriveStep_1_03 {
  const char* name() const { return "STEP 1.0 -> 0.3"; }

  int step_tick() const { return 100; } // 1.0s
  int end_tick()  const { return 400; } // 4.0s

  double v0() const { return 1.0; }
  double v1() const { return 0.3; }

  void init(Inputs& in) const {
    in = Inputs{};
    in.battery_ok = true;
    in.comms_ok = true;
    in.drive_enable = true;

    in.estop_button = false;
    in.operator_ack = false;

    in.can_timeout = false;
    in.critical_dtc = false;
    in.lift_timeout = false;
    in.lift_sensor_error = false;
    in.dump_timeout = false;
    in.dump_sensor_error = false;

    // pre-step target을 1.0으로 유지하다가 0.3으로 내림
    in.target_velocity = v0();
  }

  void apply(int tick, Inputs& in) const {
    in.operator_ack = false;
    in.target_velocity = (tick < step_tick()) ? v0() : v1();
  }
};
