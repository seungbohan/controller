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

    // feedback
    double velocity = 0.0;

    // operation complete
    bool lift_complete = false;
    bool dump_complete = false;

    // control / test inputs
    double target_velocity = 1.0;  // 제어 목표
    int scenario_id = 0;            // 시험 시나리오 ID
    int step_id = 0;                // 스텝 번호
    bool step_active = false;       // 스텝 시작 신호
};


// =====================
// Outputs to actuators / CAN commands
// =====================
struct Outputs {
    bool drive_cmd = false;
    bool lift_cmd  = false;
    bool dump_cmd  = false;

    double motor_cmd = 0.0;

    // ✅ 나중 CAN Diag/DTC 프레임으로 내보낼 "FAULT 코드"
    // 0이면 정상 / 0이 아니면 fault reason 코드
    std::uint16_t fault_code = 0;
};
