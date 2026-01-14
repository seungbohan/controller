#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include "../sim/plant.hpp"
#include "main_inputs_outputs.hpp"


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
// Controller (HOLD version)
// - DRIVE/LIFT/DUMP: latch 없음(hold)
// - FAULT: latch 유지(ACK + no_active_fault로만 해제)
// - E_STOP 최우선
// =====================
class Controller {
public:
    State state = State::IDLE;

    // FAULT는 래치 유지(ACK + no_active_fault로만 해제)
    bool fault_latched = false;

    Outputs out{};

    // ✅ complete 후 버튼 release까지 재진입 금지
    bool lift_inhibit_until_release = false;
    bool dump_inhibit_until_release = false;

    void step(const Inputs& in) {
        const bool stopped = (std::abs(in.velocity) < 0.01);

        // 기본 출력은 항상 안전(중립)부터
        out = Outputs{};

        // ✅ inhibit 해제: 버튼이 false로 돌아오면 해제
        if (!in.lift_request) lift_inhibit_until_release = false;
        if (!in.dump_request) dump_inhibit_until_release = false;

        // 0) E-STOP 최우선
        if (in.estop_button) {
            state = State::E_STOP;
            return;
        }

        // 1) FAULT 조건 발생 시 래치 세트
        if (is_fault_condition(in)) {
            fault_latched = true;
        }

        // 2) fault_latched면 강제 FAULT (하지만 handle_fault()는 실행되게 함)
        if (fault_latched) {
            state = State::FAULT;
        }

        // 3) 상태 처리
        switch (state) {
            case State::IDLE:    handle_idle(in, stopped);  break;
            case State::DRIVE:   handle_drive(in);          break;
            case State::LIFT_OP: handle_lift(in, stopped);  break;
            case State::DUMP_OP: handle_dump(in, stopped);  break;
            case State::FAULT:   handle_fault(in);          break;
            case State::E_STOP:  handle_estop(in);          break;
        }
    }

private:
    static bool is_fault_condition(const Inputs& in) {
        if (in.can_timeout || in.critical_dtc) return true;
        if (in.lift_timeout || in.lift_sensor_error) return true;
        if (in.dump_timeout || in.dump_sensor_error) return true;
        if (!in.comms_ok) return true;
        return false;
    }

    void handle_idle(const Inputs& in, bool stopped) {
        // DRIVE (hold-to-drive)
        if (in.drive_enable && in.battery_ok && in.comms_ok && stopped) {
            state = State::DRIVE;
            out.drive_cmd = true; // 즉시 출력 켬
            return;
        }

        // LIFT (hold-to-lift) + ✅ 완료 후 release 전 재진입 금지
        if (!lift_inhibit_until_release &&
            in.lift_request && !in.drive_enable && stopped) {
            state = State::LIFT_OP;
            out.lift_cmd = true;
            return;
        }

        // DUMP (hold-to-dump) + ✅ 완료 후 release 전 재진입 금지
        if (!dump_inhibit_until_release &&
            in.dump_request && !in.drive_enable && stopped) {
            state = State::DUMP_OP;
            out.dump_cmd = true;
            return;
        }
    }

    void handle_drive(const Inputs& in) {
        // DRIVE 출력은 enable 유지 동안만
        out.drive_cmd = in.drive_enable && in.battery_ok && in.comms_ok;

        // 버튼 놓으면 즉시 IDLE
        if (!in.drive_enable) {
            state = State::IDLE;
            return;
        }
    }

    void handle_lift(const Inputs& in, bool stopped) {
        // 주행 enable 들어오면 lift 중단
        if (in.drive_enable) {
            state = State::IDLE;
            return;
        }

        // hold-to-run
        out.lift_cmd = in.lift_request && stopped;

        // ✅ 완료면: IDLE로 나가되, 버튼 release 전까진 재진입 금지
        if (in.lift_complete) {
            lift_inhibit_until_release = true;
            state = State::IDLE;
            return;
        }

        // 버튼 떼면 즉시 종료
        if (!in.lift_request) {
            state = State::IDLE;
            return;
        }
    }

    void handle_dump(const Inputs& in, bool stopped) {
        if (in.drive_enable) {
            state = State::IDLE;
            return;
        }

        out.dump_cmd = in.dump_request && stopped;

        // ✅ 완료면: IDLE로 나가되, 버튼 release 전까진 재진입 금지
        if (in.dump_complete) {
            dump_inhibit_until_release = true;
            state = State::IDLE;
            return;
        }

        if (!in.dump_request) {
            state = State::IDLE;
            return;
        }
    }

    void handle_fault(const Inputs& in) {
        // FAULT에서는 출력 중립(안전)
        out = Outputs{};

        // 원인 사라짐 + ACK로만 해제
        if (in.no_active_fault && in.operator_ack) {
            fault_latched = false;
            state = State::IDLE;
        }
    }

    void handle_estop(const Inputs& in) {
        // E-STOP에서는 출력 중립
        out = Outputs{};

        // 버튼 해제 + ACK로만 복귀
        if (!in.estop_button && in.operator_ack) {
            state = State::IDLE;
        }
    }
};


// =====================
// Main (demo)
// =====================
int main() {
    Controller ctrl;
    Inputs in;
    Plant plant;

    using clock = std::chrono::steady_clock;
    auto last_control = clock::now();
    auto last_log     = last_control;
    auto last_diag    = last_control;

    int tick10ms = 0;

    std::cout << "Controller started (HOLD: drive/lift/dump, FAULT latched)\n";

    while (true) {
        auto now = clock::now();

        // -------------------------
        // 10ms: Control loop
        // -------------------------
        if (now - last_control >= std::chrono::milliseconds(10)) {

            // ---- demo sequence ----
            // DRIVE enable hold (on for a while, then off)
            if (tick10ms == 200)  in.drive_enable = true;
            if (tick10ms == 600)  in.drive_enable = false;

            // LIFT hold (press and hold, then release; complete happens while holding)
            if (tick10ms == 800)  in.lift_request = true;
            if (tick10ms == 980)  in.lift_complete = true;  // 완료 발생
            if (tick10ms == 990)  in.lift_complete = false;
            if (tick10ms == 1000) in.lift_request = false;  // 버튼 뗌(hold 종료)

            // DUMP hold
            if (tick10ms == 1200) in.dump_request = true;
            if (tick10ms == 1380) in.dump_complete = true;
            if (tick10ms == 1390) in.dump_complete = false;
            if (tick10ms == 1400) in.dump_request = false;

            // E-STOP demo
            if (tick10ms == 1450) in.estop_button = true;
            if (tick10ms == 1500) { in.estop_button = false; in.operator_ack = true; }
            if (tick10ms == 1510) in.operator_ack = false;

            // FAULT demo (comms lost)
            if (tick10ms == 1600) in.comms_ok = false;
            if (tick10ms == 1800) in.comms_ok = true;        // 복구돼도 FAULT latch 유지
            if (tick10ms == 1900) in.operator_ack = true;    // no_active_fault && ACK로 해제
            if (tick10ms == 1910) in.operator_ack = false;

            ctrl.step(in);
            plant.step(ctrl.out, in, 0.01); // 10ms = 0.01s

            last_control = now;
            tick10ms++;
        }

        // -------------------------
        // 50ms: Logging
        // -------------------------
        if (now - last_log >= std::chrono::milliseconds(50)) {
            std::cout << "[tick=" << tick10ms
                      << "] state=" << state_to_string(ctrl.state)
                      << " drive_en=" << (in.drive_enable ? 1 : 0)
                      << " lift_btn=" << (in.lift_request ? 1 : 0)
                      << " dump_btn=" << (in.dump_request ? 1 : 0)
                      << " estop=" << (in.estop_button ? 1 : 0)
                      << " comms=" << (in.comms_ok ? 1 : 0)
                      << " fault_latch=" << (ctrl.fault_latched ? 1 : 0)
                      << " | OUT drive=" << (ctrl.out.drive_cmd ? 1 : 0)
                      << " lift=" << (ctrl.out.lift_cmd ? 1 : 0)
                      << " dump=" << (ctrl.out.dump_cmd ? 1 : 0)
                      << " vel=" << in.velocity
                      << " lift_p=" << plant.lift_pos
                      << " dump_p=" << plant.dump_pos
                      << "\n";
            last_log = now;
        }

        // -------------------------
        // 100ms: Diagnostics
        // -------------------------
        if (now - last_diag >= std::chrono::milliseconds(100)) {
            in.no_active_fault = in.comms_ok
                              && !in.can_timeout
                              && !in.critical_dtc
                              && !in.lift_timeout
                              && !in.lift_sensor_error
                              && !in.dump_timeout
                              && !in.dump_sensor_error;

            last_diag = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
