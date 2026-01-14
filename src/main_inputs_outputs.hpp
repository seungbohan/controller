#pragma once
#include <cstdint>

// =====================
// Inputs from "vehicle" / sensors / operator
// =====================
struct Inputs {
    // operator
    bool drive_enable = false;
    bool lift_request = false;
    bool dump_request = false;
    bool operator_ack = false;
    bool estop_button = false;

    // system status
    bool battery_ok = true;
    bool comms_ok   = true;

    // faults (raw)
    bool can_timeout        = false;
    bool critical_dtc       = false;
    bool lift_timeout       = false;
    bool lift_sensor_error  = false;
    bool dump_timeout       = false;
    bool dump_sensor_error  = false;

    // derived / diagnostic
    bool no_active_fault = true;

    // feedback (e.g. velocity)
    double velocity = 0.0;

    // operation complete flags (from plant/sensors)
    bool lift_complete = false;
    bool dump_complete = false;
};

// =====================
// Outputs to actuators / CAN commands
// =====================
struct Outputs {
    bool drive_cmd = false;
    bool lift_cmd  = false;
    bool dump_cmd  = false;

    // ✅ 나중 CAN Diag/DTC 프레임으로 내보낼 "FAULT 코드"
    // 0이면 정상 / 0이 아니면 fault reason 코드
    std::uint16_t fault_code = 0;
};
