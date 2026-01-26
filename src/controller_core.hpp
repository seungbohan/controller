#pragma once
#include <cstdint>
#include "../include/main_inputs_outputs.hpp"
#include "../include/pid.hpp"

// main.cpp에 있던 enum들을 코어로 옮김
enum class State {
    IDLE,
    DRIVE,
    LIFT_OP,
    DUMP_OP,
    FAULT,
    E_STOP
};

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

struct ControllerDebug {
    int state = 0;
    bool fault_latched = false;
    bool comms_ok_filtered = true;
    std::uint16_t fault_code = 0;

    PID::PIDDebug pid_dbg;
};

class ControllerCore {
public:
    ControllerCore();

    // 10ms 주기에서 호출 (dt는 초 단위: 0.01)
    Outputs step(const Inputs& in, double dt);

    ControllerDebug debug() const { return dbg_; }

    void reset();

private:
    // ----- fault 우선순위 선택 -----
    static FaultReason pick_fault_reason(const Inputs& in, bool comms_ok_filtered);

    // ----- 상태 핸들러 -----
    void handle_idle(const Inputs& in, bool stopped);
    void handle_drive(const Inputs& in, double dt);
    void handle_lift(const Inputs& in, bool stopped);
    void handle_dump(const Inputs& in, bool stopped);
    void handle_fault(const Inputs& in);
    void handle_estop(const Inputs& in);

private:
    // core state
    State state_ = State::IDLE;

    bool fault_latched_ = false;
    FaultReason latched_reason_ = FaultReason::NONE;

    // outputs
    Outputs out_{};

    // PID (main.cpp 기본값 그대로)
    PID drive_pid_{1.9, 2.5, 0.0};

    // complete 후 버튼 release 전 재진입 금지
    bool lift_inhibit_until_release_ = false;
    bool dump_inhibit_until_release_ = false;

    // comms filter 내부 상태
    int  comms_fail_ms_ = 0;
    int  comms_ok_ms_   = 0;
    bool comms_ok_filtered_ = true;

    static constexpr int COMMS_FAIL_TIMEOUT_MS   = 50;   // 50ms 이상 끊김이면 확정
    static constexpr int COMMS_RECOVER_STABLE_MS = 100;  // 100ms 이상 안정이면 복구 인정

    // debug snapshot
    ControllerDebug dbg_{};
};
