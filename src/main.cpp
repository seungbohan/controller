#include <iostream>
#include <string>
#include <chrono>
#include <thread>

// =====================
// State Definition
// =====================
enum class State {
    IDLE,
    DRIVE,
    LIFT_OP,
    DUMP_OP,
    FAULT,
    E_STOP
};

static std::string state_to_string(State s) {
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
// Inputs (mock for now)
// =====================
struct Inputs {
    // commands / requests
    bool drive_enable = false;
    bool lift_request = false;
    bool dump_request = false;

    // safety & health
    bool estop_button = false;   // ANY -> E_STOP
    bool operator_ack = false;   // recovery
    bool battery_ok = true;
    bool comms_ok = true;
    bool no_active_fault = true;

    // measured
    double velocity = 0.0;

    // operation results
    bool lift_complete = false;
    bool dump_complete = false;

    // fault causes (example)
    bool can_timeout = false;
    bool critical_dtc = false;

    bool lift_timeout = false;
    bool lift_sensor_error = false;
    bool dump_timeout = false;
    bool dump_sensor_error = false;
};

// =====================
// Controller
// =====================
class Controller {
public:
    State state = State::IDLE;

    // 내부 요청(래치)
    bool lift_req_latched = false;
    bool dump_req_latched = false;

    void step(const Inputs& in) {
        // 1) Global priority transitions
        if (in.estop_button) {
	    lift_req_latched = false;
	    dump_req_latched = false;
            state = State::E_STOP;
            return;
        }

	// 1) 입력을 "트리거"로 내부 요청 세트
	if (in.lift_request) lift_req_latched = true;
	if (in.dump_request) dump_req_latched = true;

        if (is_fault_condition(in)) {
            state = State::FAULT;
            return;
        }

        // 2) State-specific transitions
        switch (state) {
            case State::IDLE:
                handle_idle(in);
                break;

            case State::DRIVE:
                handle_drive(in);
                break;

            case State::LIFT_OP:
                handle_lift(in);
                break;

            case State::DUMP_OP:
                handle_dump(in);
                break;

            case State::FAULT:
                handle_fault(in);
                break;

            case State::E_STOP:
                handle_estop(in);
                break;
        }
    }

private:
    static bool is_fault_condition(const Inputs& in) {
        // Global fault: CAN timeout or critical DTC
        if (in.can_timeout || in.critical_dtc) return true;

        // Operation faults
        if (in.lift_timeout || in.lift_sensor_error) return true;
        if (in.dump_timeout || in.dump_sensor_error) return true;

        // If system says "fault exists"
        if (!in.no_active_fault) return true;

        return false;
    }

    void handle_idle(const Inputs& in) {
        // IDLE -> DRIVE
        if (in.drive_enable &&
            !in.estop_button &&
            in.battery_ok &&
            in.comms_ok &&
            in.no_active_fault &&
            in.velocity == 0.0) {
            state = State::DRIVE;
            return;
        }

        // IDLE -> LIFT_OP (interlock: v=0, drive_enable=0)
        if (in.lift_request &&
            !in.drive_enable &&
            in.velocity == 0.0 &&
            !in.estop_button &&
            in.no_active_fault) {
            state = State::LIFT_OP;
            return;
        }

        // IDLE -> DUMP_OP (interlock: v=0, drive_enable=0)
        if (in.dump_request &&
            !in.drive_enable &&
            in.velocity == 0.0 &&
            !in.estop_button &&
            in.no_active_fault) {
            state = State::DUMP_OP;
            return;
        }
    }

    void handle_drive(const Inputs& in) {
        // DRIVE -> IDLE
        if (!in.drive_enable && in.velocity == 0.0) {
            state = State::IDLE;
            return;
        }

        // DRIVE -> LIFT_OP (only when stopped)
        if (in.lift_request && in.velocity == 0.0) {
            state = State::LIFT_OP;
            return;
        }

        // DRIVE -> DUMP_OP (only when stopped)
        if (in.dump_request && in.velocity == 0.0) {
            state = State::DUMP_OP;
            return;
        }
    }

    void handle_lift(const Inputs& in) {
        // LIFT_OP -> IDLE
        if (in.lift_complete && !in.lift_request) {
	    lift_req_latched = false;
            state = State::IDLE;
            return;
        }
        // faults are handled by global fault logic
    }

    void handle_dump(const Inputs& in) {
        // DUMP_OP -> IDLE
        if (in.dump_complete && !in.dump_request) {
	    dump_req_latched = false;
            state = State::IDLE;
            return;
        }
        // faults are handled by global fault logic
    }

    void handle_fault(const Inputs& in) {
        // FAULT -> IDLE (manual ack required)
        if (in.no_active_fault && in.operator_ack) {
            state = State::IDLE;
            return;
        }
    }

    void handle_estop(const Inputs& in) {
        // E_STOP -> IDLE (manual ack + estop released)
        if (!in.estop_button && in.operator_ack) {
            state = State::IDLE;
            return;
        }
    }
};

// =====================
// Simple test driver
// =====================
int main() {
    Controller ctrl;
    Inputs in;

    using clock = std::chrono::steady_clock;

    auto last_control = clock::now();
    auto last_log     = last_control;
    auto last_diag    = last_control;

    int tick10ms = 0;

    std::cout << "Controller started (10ms control / 50ms log / 100ms diag)\n";

    while (true) {
        auto now = clock::now();

        // -------------------------
        // 10ms: Control loop
        // -------------------------
        if (now - last_control >= std::chrono::milliseconds(10)) {
            // (선택) 여기서 입력 갱신: read_inputs(in);
            // 지금은 mock 입력 그대로 사용

// ---- demo input sequence ----
if (tick10ms == 200) {
    in.drive_enable = true;
}
if (tick10ms == 600) {
    in.drive_enable = false;
}
if (tick10ms == 800) {
    in.lift_request = true;
}
if (tick10ms == 1000) {
    in.lift_request = false;
    in.lift_complete = true;
}
if (tick10ms == 1010) {
    in.lift_complete = false;
}
if (tick10ms == 1200) {
    in.dump_request = true;
}
// ---- E_STOP demo ----
if (tick10ms == 1300) { 
    in.estop_button = true;     // ✅ 즉시 E_STOP
}
if (tick10ms == 1350) {
    in.estop_button = false;    // 비상정지 버튼 해제
    in.operator_ack = true;     // ✅ 복귀 승인(ACK)
}
if (tick10ms == 1360) {
    in.operator_ack = false;    // ACK 리셋
}

if (tick10ms == 1400) {
    in.dump_request = false;
    in.dump_complete = true;
}
if (tick10ms == 1410) {
    in.dump_complete = false;
}


            ctrl.step(in);        // 상태머신 + 제어 로직
            last_control = now;
            tick10ms++;
        }

        // -------------------------
        // 50ms: Logging
        // -------------------------
        if (now - last_log >= std::chrono::milliseconds(50)) {
            std::cout << "[tick=" << tick10ms
                      << "] state=" << state_to_string(ctrl.state)
                      << " v=" << in.velocity
                      << " drive=" << (in.drive_enable ? 1 : 0)
                      << " lift=" << (in.lift_request ? 1 : 0)
                      << " dump=" << (in.dump_request ? 1 : 0)
                      << " estop=" << (in.estop_button ? 1 : 0)
                      << "\n";
            last_log = now;
        }

        // -------------------------
        // 100ms: Diagnostics
        // -------------------------
        if (now - last_diag >= std::chrono::milliseconds(100)) {
            // TODO: run diagnostics (예: comms_ok 갱신, timeout 검사, DTC 판정)
            // 예시: 통신 타임아웃이면 in.can_timeout = true; 같은 식
            last_diag = now;
        }

        // CPU 100% 방지 (RTOS에서 yield 같은 역할)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
