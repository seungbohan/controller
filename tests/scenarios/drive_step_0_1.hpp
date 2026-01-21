#pragma once
#include "main_inputs_outputs.hpp"

inline Inputs scenario_drive_step_0_to_1(double t, const Inputs& base = {}) {
  Inputs in = base;
  in.drive_enable = true;
  in.estop_button = false;
  in.battery_ok = true;
  in.comms_ok = true;

  // step at t = 1.0s
  if (t < 1.0) in.target_velocity = 0.0;
  else        in.target_velocity = 1.0;

  return in;
}
