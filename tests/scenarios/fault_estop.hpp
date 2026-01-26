#pragma once
#include "../../include/main_inputs_outputs.hpp"

struct FaultEstop {
  const char* name() const { return "FAULT: E-STOP cutoff"; }

  int end_tick() const { return 300; } // 3.0s

  // estop 펄스 구간
  int estop_on_tick()  const { return 100; } // 1.0s
  int estop_off_tick() const { return 120; } // 1.2s

  // ACK는 estop 해제 후 주기
  int ack_tick() const { return 150; } // 1.5s (1 tick 펄스)

  void init(Inputs& in) const {
    in = Inputs{};
    in.battery_ok = true;
    in.comms_ok = true;
    in.drive_enable = true;
    in.target_velocity = 1.0;

    in.estop_button = false;
    in.operator_ack = false;

    in.can_timeout = false;
    in.critical_dtc = false;
    in.lift_timeout = false;
    in.lift_sensor_error = false;
    in.dump_timeout = false;
    in.dump_sensor_error = false;
  }

  void apply(int tick, Inputs& in) const {
    in.operator_ack = false;

    // 유지 입력
    in.drive_enable = true;
    in.target_velocity = 1.0;

    // estop 구간
    if (tick >= estop_on_tick() && tick < estop_off_tick()) in.estop_button = true;
    else in.estop_button = false;

    // ack 펄스
    if (tick == ack_tick()) in.operator_ack = true;
  }
};
