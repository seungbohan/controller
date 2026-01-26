#pragma once
#include "../../include/main_inputs_outputs.hpp"

struct CommsLostLatch {
  const char* name() const { return "FAULT: COMMS lost latch + ACK clear"; }

  int end_tick() const { return 400; } // 4.0s

  // comms 끊김 구간(예: 300ms)
  int comms_off_start() const { return 100; } // 1.0s
  int comms_off_end()   const { return 130; } // 1.3s (30 ticks = 300ms)

  // 복구 후 ACK (필터 복구시간 고려해서 여유)
  int ack_tick() const { return 200; } // 2.0s

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
    in.drive_enable = true;
    in.target_velocity = 1.0;

    // comms 끊김
    if (tick >= comms_off_start() && tick < comms_off_end()) in.comms_ok = false;
    else in.comms_ok = true;

    // ack 펄스
    if (tick == ack_tick()) in.operator_ack = true;
  }
};
