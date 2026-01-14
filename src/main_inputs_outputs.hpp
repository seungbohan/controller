// =====================
// Inputs
// =====================
struct Inputs {
    // commands / requests (HOLD 방식: 누르고 있는 동안만 true)
    bool drive_enable = false;   // hold-to-drive enable (데드맨/enable 스위치 느낌)
    bool lift_request = false;   // hold-to-lift
    bool dump_request = false;   // hold-to-dump

    // safety & health
    bool estop_button = false;
    bool operator_ack = false;   // FAULT 해제용 ACK
    bool battery_ok   = true;
    bool comms_ok     = true;

    // derived (diagnostics)
    bool no_active_fault = true;

    // measured
    double velocity = 0.0;

    // operation results
    bool lift_complete = false;
    bool dump_complete = false;

    // fault causes
    bool can_timeout       = false;
    bool critical_dtc      = false;
    bool lift_timeout      = false;
    bool lift_sensor_error = false;
    bool dump_timeout      = false;
    bool dump_sensor_error = false;
};

// =====================
// Outputs (예시)
// =====================
struct Outputs {
    bool drive_cmd = false;  // 실제 구동 enable/torque allow 같은 신호라고 생각
    bool lift_cmd  = false;  // 밸브/모터 구동 명령 (예시)
    bool dump_cmd  = false;
};
