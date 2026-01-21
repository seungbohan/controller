#pragma once
#include <string>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include "pid.hpp"
#include "main_inputs_outputs.hpp"

// =====================
// State
// =====================
enum class State {
    IDLE,
    DRIVE,
    LIFT_OP,
    DUMP_OP,
    FAULT,
    E_STOP
};

inline std::string state_to_string(State s) {
    switch (s) {
        case State::IDLE:    return "IDLE";
        case State::DRIVE:   return "DRIVE";
        case State::LIFT_OP: return "LIFT_OP";
        case State::DUMP_OP: return "DUMP_OP";
        case State::FAULT:   return "FAULT";
        case State::E_STOP:  return "E_STOP";
        default:             return "UNKNOWN";
    }
}

// =====================
// Fault Reason + Priority
// =====================
enum class FaultReason : std::uint16_t {
    NONE = 0,

    ESTOP            = 10,
    CRITICAL_DTC     = 20,
    CAN_TIMEOUT      = 30,
    COMMS_LOST       = 40,

    LIFT_TIMEOUT     = 50,
    LIFT_SENSOR_ERR  = 60,
    DUMP_TIMEOUT     = 70,
    DUMP_SENSOR_ERR  = 80
};

inline const char* fault_to_string(FaultReason r) {
    switch (r) {
        case FaultReason::NONE:            return "NONE";
        case FaultReason::ESTOP:           return "E_STOP";
        case FaultReason::CRITICAL_DTC:    return "CRITICAL_DTC";
        case FaultReason::CAN_TIMEOUT:     return "CAN_TIMEOUT";
        case FaultReason::COMMS_LOST:      return "COMMS_LOST";
        case FaultReason::LIFT_TIMEOUT:    return "LIFT_TIMEOUT";
        case FaultReason::LIFT_SENSOR_ERR: return "LIFT_SENSOR_ERR";
        case FaultReason::DUMP_TIMEOUT:    return "DUMP_TIMEOUT";
        case FaultReason::DUMP_SENSOR_ERR: return "DUMP_SENSOR_ERR";
        default:                           return "UNKNOWN_FAULT";
    }
}

inline FaultReason pick_fault_reason(const Inputs& in, bool comms_ok_filtered) {
    if (in.estop_button)            return FaultReason::ESTOP;
    if (in.critical_dtc)            return FaultReason::CRITICAL_DTC;
    if (in.can_timeout)             return FaultReason::CAN_TIMEOUT;
    if (!comms_ok_filtered)         return FaultReason::COMMS_LOST;

    if (in.lift_timeout)            return FaultReason::LIFT_TIMEOUT;
    if (in.lift_sensor_error)       return FaultReason::LIFT_SENSOR_ERR;
    if (in.dump_timeout)            return FaultReason::DUMP_TIMEOUT;
    if (in.dump_sensor_error)       return FaultReason::DUMP_SENSOR_ERR;

    return FaultReason::NONE;
}

// =====================
// Controller
// =====================
class Controller {
public:
    State state = State::IDLE;

    bool fault_latched = false;
    FaultReason latched_reason = FaultReason::NONE;

    Outputs out{};

    PID drive_pid{5.0, 3.0, 1.5};

    bool lift_inhibit_until_release = false;
    bool dump_inhibit_until_release = false;

    bool get_comms_ok_filtered() const { return comms_ok_filtered; }

    // Expose for testing
    int get_comms_fail_ms() const { return comms_fail_ms; }
    int get_comms_ok_ms() const { return comms_ok_ms; }

    void step(const Inputs& in) {
        const bool stopped = (std::abs(in.velocity) < 0.01);

        out = Outputs{};

        if (!in.lift_request) lift_inhibit_until_release = false;
        if (!in.dump_request) dump_inhibit_until_release = false;

        constexpr int DT_MS = 10;

        if (!in.comms_ok) {
            comms_fail_ms += DT_MS;
            comms_ok_ms = 0;

            if (comms_fail_ms >= COMMS_FAIL_TIMEOUT_MS) {
                comms_ok_filtered = false;
            }
        } else {
            comms_ok_ms += DT_MS;
            comms_fail_ms = 0;

            if (comms_ok_ms >= COMMS_RECOVER_STABLE_MS) {
                comms_ok_filtered = true;
            }
        }

        if (in.estop_button) {
            state = State::E_STOP;
            out.fault_code = static_cast<std::uint16_t>(FaultReason::ESTOP);
            return;
        }

        const FaultReason current = pick_fault_reason(in, comms_ok_filtered);

        if (current != FaultReason::NONE && !fault_latched) {
            fault_latched = true;
            latched_reason = current;
        }

        if (fault_latched) {
            state = State::FAULT;
        }

        switch (state) {
            case State::IDLE:    handle_idle(in, stopped);  break;
            case State::DRIVE:   handle_drive(in);          break;
            case State::LIFT_OP: handle_lift(in, stopped);  break;
            case State::DUMP_OP: handle_dump(in, stopped);  break;
            case State::FAULT:   handle_fault(in);          break;
            case State::E_STOP:  handle_estop(in);          break;
        }
    }

    // Constants exposed for testing
    static constexpr int COMMS_FAIL_TIMEOUT_MS   = 50;
    static constexpr int COMMS_RECOVER_STABLE_MS = 100;

private:
    int  comms_fail_ms = 0;
    int  comms_ok_ms   = 0;
    bool comms_ok_filtered = true;

    void handle_idle(const Inputs& in, bool stopped) {
        if (in.drive_enable && in.battery_ok && comms_ok_filtered && stopped) {
            state = State::DRIVE;
            out.drive_cmd = true;
            return;
        }

        if (!lift_inhibit_until_release &&
            in.lift_request && !in.drive_enable && stopped) {
            state = State::LIFT_OP;
            out.lift_cmd = true;
            return;
        }

        if (!dump_inhibit_until_release &&
            in.dump_request && !in.drive_enable && stopped) {
            state = State::DUMP_OP;
            out.dump_cmd = true;
            return;
        }
    }

    void handle_drive(const Inputs& in) {
        if (!in.drive_enable || !in.battery_ok || !comms_ok_filtered) {
            out.drive_cmd = false;
            out.motor_cmd = 0.0;
            drive_pid.reset();
            state = State::IDLE;
            return;
        }

        constexpr double dt = 0.01;
        out.drive_cmd = true;
        out.motor_cmd = drive_pid.compute(in.target_velocity, in.velocity, dt);
    }

    void handle_lift(const Inputs& in, bool stopped) {
        if (in.drive_enable) { state = State::IDLE; return; }

        out.lift_cmd = in.lift_request && stopped;

        if (in.lift_complete) {
            lift_inhibit_until_release = true;
            state = State::IDLE;
            return;
        }
        if (!in.lift_request) { state = State::IDLE; return; }
    }

    void handle_dump(const Inputs& in, bool stopped) {
        if (in.drive_enable) { state = State::IDLE; return; }

        out.dump_cmd = in.dump_request && stopped;

        if (in.dump_complete) {
            dump_inhibit_until_release = true;
            state = State::IDLE;
            return;
        }
        if (!in.dump_request) { state = State::IDLE; return; }
    }

    void handle_fault(const Inputs& in) {
        out = Outputs{};
        out.fault_code = static_cast<std::uint16_t>(latched_reason);

        if (in.no_active_fault && in.operator_ack) {
            fault_latched = false;
            latched_reason = FaultReason::NONE;
            state = State::IDLE;
        }
    }

    void handle_estop(const Inputs& in) {
        out = Outputs{};
        out.fault_code = static_cast<std::uint16_t>(FaultReason::ESTOP);

        if (!in.estop_button && in.operator_ack) {
            state = State::IDLE;
        }
    }
};
