#include "controller_core.hpp"
#include <cmath>

ControllerCore::ControllerCore() {}

void ControllerCore::reset() {
    state_ = State::IDLE;
    fault_latched_ = false;
    latched_reason_ = FaultReason::NONE;

    out_ = Outputs{};
    drive_pid_.reset();

    lift_inhibit_until_release_ = false;
    dump_inhibit_until_release_ = false;

    comms_fail_ms_ = 0;
    comms_ok_ms_ = 0;
    comms_ok_filtered_ = true;

    dbg_ = ControllerDebug{};
}

FaultReason ControllerCore::pick_fault_reason(const Inputs& in, bool comms_ok_filtered) {
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

Outputs ControllerCore::step(const Inputs& in, double dt) {
    const bool stopped = (std::abs(in.velocity) < 0.01);

    // 기본 출력 안전(중립)
    out_ = Outputs{};

    // inhibit 해제: 버튼이 false로 돌아오면 해제
    if (!in.lift_request) lift_inhibit_until_release_ = false;
    if (!in.dump_request) dump_inhibit_until_release_ = false;

    // =========================
    // COMMS_LOST 판정 (시간 기반 + 복구 히스테리시스)
    // =========================
    const int dt_ms = (dt > 0) ? (int)std::lround(dt * 1000.0) : 10;

    if (!in.comms_ok) {
        comms_fail_ms_ += dt_ms;
        comms_ok_ms_ = 0;

        if (comms_fail_ms_ >= COMMS_FAIL_TIMEOUT_MS) {
            comms_ok_filtered_ = false;
        }
    } else {
        comms_ok_ms_ += dt_ms;
        comms_fail_ms_ = 0;

        if (comms_ok_ms_ >= COMMS_RECOVER_STABLE_MS) {
            comms_ok_filtered_ = true;
        }
    }

    // 0) E-STOP 최우선 상태
    if (in.estop_button) {
        state_ = State::E_STOP;
        out_.fault_code = static_cast<std::uint16_t>(FaultReason::ESTOP);
        // debug 갱신
        dbg_.state = (int)state_;
        dbg_.fault_latched = fault_latched_;
        dbg_.comms_ok_filtered = comms_ok_filtered_;
        dbg_.fault_code = out_.fault_code;
        return out_;
    }

    // 1) fault 감지 (우선순위 1개 선택)
    const FaultReason current = pick_fault_reason(in, comms_ok_filtered_);

    // 2) 래치 세트: 처음 fault만 저장
    if (current != FaultReason::NONE && !fault_latched_) {
        fault_latched_ = true;
        latched_reason_ = current;
    }

    // 3) fault_latched면 FAULT로 강제
    if (fault_latched_) {
        state_ = State::FAULT;
    }

    // 4) 상태 처리
    switch (state_) {
        case State::IDLE:    handle_idle(in, stopped);        break;
        case State::DRIVE:   handle_drive(in, dt);            break;
        case State::LIFT_OP: handle_lift(in, stopped);        break;
        case State::DUMP_OP: handle_dump(in, stopped);        break;
        case State::FAULT:   handle_fault(in);                break;
        case State::E_STOP:  handle_estop(in);                break;
    }

    // debug 갱신
    dbg_.state = (int)state_;
    dbg_.fault_latched = fault_latched_;
    dbg_.comms_ok_filtered = comms_ok_filtered_;
    dbg_.fault_code = out_.fault_code;

    return out_;
}

void ControllerCore::handle_idle(const Inputs& in, bool stopped) {
    // DRIVE
    if (in.drive_enable && in.battery_ok && comms_ok_filtered_) {
        state_ = State::DRIVE;
        return;
    }

    // LIFT (hold-to-lift) + 완료 후 release 전 재진입 금지
    if (!lift_inhibit_until_release_ &&
        in.lift_request && !in.drive_enable && stopped) {
        state_ = State::LIFT_OP;
        out_.lift_cmd = true;
        return;
    }

    // DUMP (hold-to-dump) + 완료 후 release 전 재진입 금지
    if (!dump_inhibit_until_release_ &&
        in.dump_request && !in.drive_enable && stopped) {
        state_ = State::DUMP_OP;
        out_.dump_cmd = true;
        return;
    }
}

void ControllerCore::handle_drive(const Inputs& in, double dt) {
    if (!in.drive_enable || !in.battery_ok || !comms_ok_filtered_) {
        out_.drive_cmd = false;
        out_.motor_cmd = 0.0;
        drive_pid_.reset();
        state_ = State::IDLE;
        return;
    }

    out_.drive_cmd = true;
    out_.motor_cmd = drive_pid_.compute(in.target_velocity, in.velocity, dt);
}

void ControllerCore::handle_lift(const Inputs& in, bool stopped) {
    if (in.drive_enable) { state_ = State::IDLE; return; }

    out_.lift_cmd = in.lift_request && stopped;

    if (in.lift_complete) {
        lift_inhibit_until_release_ = true;
        state_ = State::IDLE;
        return;
    }
    if (!in.lift_request) { state_ = State::IDLE; return; }
}

void ControllerCore::handle_dump(const Inputs& in, bool stopped) {
    if (in.drive_enable) { state_ = State::IDLE; return; }

    out_.dump_cmd = in.dump_request && stopped;

    if (in.dump_complete) {
        dump_inhibit_until_release_ = true;
        state_ = State::IDLE;
        return;
    }
    if (!in.dump_request) { state_ = State::IDLE; return; }
}

void ControllerCore::handle_fault(const Inputs& in) {
    // FAULT에서는 출력 중립 + fault_code 출력
    out_ = Outputs{};
    out_.fault_code = static_cast<std::uint16_t>(latched_reason_);

    // 원인 사라짐 + ACK로만 해제
    if (in.no_active_fault && in.operator_ack) {
        fault_latched_ = false;
        latched_reason_ = FaultReason::NONE;
        state_ = State::IDLE;
    }
}

void ControllerCore::handle_estop(const Inputs& in) {
    out_ = Outputs{};
    out_.fault_code = static_cast<std::uint16_t>(FaultReason::ESTOP);

    // E-STOP 해제 + ACK로만 복귀
    if (!in.estop_button && in.operator_ack) {
        state_ = State::IDLE;
    }
}
