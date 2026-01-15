#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

#include "../sim/plant.hpp"
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
// Fault Reason + Priority
// =====================
enum class FaultReason : std::uint16_t {
    NONE = 0,

    // priority 높은 순서로 위에서부터(원하면 여기 순서 바꾸면 됨)
    ESTOP            = 10,
    CRITICAL_DTC     = 20,
    CAN_TIMEOUT      = 30,
    COMMS_LOST       = 40,

    LIFT_TIMEOUT     = 50,
    LIFT_SENSOR_ERR  = 60,
    DUMP_TIMEOUT     = 70,
    DUMP_SENSOR_ERR  = 80
};

static const char* fault_to_string(FaultReason r) {
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

// ✅ 우선순위 테이블(원하는 정책대로 여기만 바꾸면 끝)
// ✅ 실무식: "comms_ok_filtered"를 넣어서 1틱 노이즈 같은 걸 타임아웃 기반으로 걸러냄
static FaultReason pick_fault_reason(const Inputs& in, bool comms_ok_filtered) {
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
// Controller (HOLD version + FAULT latch)
// =====================
class Controller {
public:
    State state = State::IDLE;

    // FAULT latch
    bool fault_latched = false;

    // ✅ "처음 잡힌 fault reason"을 latched로 저장 (원인 안 흔들리게)
    FaultReason latched_reason = FaultReason::NONE;

    Outputs out{};

    // ✅ complete 후 버튼 release까지 재진입 금지
    bool lift_inhibit_until_release = false;
    bool dump_inhibit_until_release = false;

    // ✅ 외부(main)에서 no_active_fault 계산할 때 쓰기 위한 getter
    bool get_comms_ok_filtered() const { return comms_ok_filtered; }

    void step(const Inputs& in) {
        const bool stopped = (std::abs(in.velocity) < 0.01);

        // 기본 출력 안전(중립)
        out = Outputs{};

        // ✅ inhibit 해제: 버튼이 false로 돌아오면 해제
        if (!in.lift_request) lift_inhibit_until_release = false;
        if (!in.dump_request) dump_inhibit_until_release = false;

        // =========================================================
        // ✅ 실무식 COMMS_LOST 판정 (시간 기반 타임아웃 + 복구 히스테리시스)
        // - FAIL_TIMEOUT_MS 동안 연속으로 comms_ok=false면 "확정 끊김"
        // - RECOVER_STABLE_MS 동안 연속으로 comms_ok=true면 "복구 인정"
        // - 이후 fault_latched 철학은 그대로: 래치 + ACK
        // =========================================================
        constexpr int DT_MS = 10; // 이 step은 10ms 주기로 불린다는 가정

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

        // 0) E-STOP 최우선 상태
        if (in.estop_button) {
            state = State::E_STOP;
            // fault_code도 같이 올려주면(선택) CAN에서도 동일 처리 가능
            out.fault_code = static_cast<std::uint16_t>(FaultReason::ESTOP);
            return;
        }

        // 1) fault 감지 (우선순위로 1개 선택)  ✅ comms_ok_filtered 사용
        const FaultReason current = pick_fault_reason(in, comms_ok_filtered);

        // 2) 래치 세트: 처음 fault만 저장
        if (current != FaultReason::NONE && !fault_latched) {
            fault_latched = true;
            latched_reason = current;
        }

        // 3) fault_latched면 FAULT로 강제
        if (fault_latched) {
            state = State::FAULT;
        }

        // 4) 상태 처리
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
    // ---- COMMS 필터 내부 상태 ----
    int  comms_fail_ms = 0;
    int  comms_ok_ms   = 0;
    bool comms_ok_filtered = true;

    // ---- 튜닝 파라미터(실무 기본값) ----
    static constexpr int COMMS_FAIL_TIMEOUT_MS   = 50;   // 50ms 이상 끊김이면 확정 FAULT
    static constexpr int COMMS_RECOVER_STABLE_MS = 100;  // 100ms 이상 안정이면 복구 인정

    void handle_idle(const Inputs& in, bool stopped) {
        // DRIVE (hold-to-drive)
        // ✅ comms_ok_filtered를 쓸지, 원본 in.comms_ok를 쓸지는 정책 선택
        // 실무에서는 "동작 허가"도 필터된 값을 쓰는 편이 많음(깜빡임 방지)
        if (in.drive_enable && in.battery_ok && comms_ok_filtered && stopped) {
            state = State::DRIVE;
            out.drive_cmd = true;
            return;
        }

        // LIFT (hold-to-lift) + 완료 후 release 전 재진입 금지
        if (!lift_inhibit_until_release &&
            in.lift_request && !in.drive_enable && stopped) {
            state = State::LIFT_OP;
            out.lift_cmd = true;
            return;
        }

        // DUMP (hold-to-dump) + 완료 후 release 전 재진입 금지
        if (!dump_inhibit_until_release &&
            in.dump_request && !in.drive_enable && stopped) {
            state = State::DUMP_OP;
            out.dump_cmd = true;
            return;
        }
    }

    void handle_drive(const Inputs& in) {
        // ✅ 여기서도 필터된 comms 값 사용(노이즈로 drive_cmd 깜빡임 방지)
        out.drive_cmd = in.drive_enable && in.battery_ok && comms_ok_filtered;
        if (!in.drive_enable) state = State::IDLE;
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
        // FAULT에서는 출력 중립 + fault_code 출력
        out = Outputs{};
        out.fault_code = static_cast<std::uint16_t>(latched_reason);

        // 원인 사라짐 + ACK로만 해제
        // (no_active_fault는 main loop에서 주기적으로 갱신해줌)
        if (in.no_active_fault && in.operator_ack) {
            fault_latched = false;
            latched_reason = FaultReason::NONE;
            state = State::IDLE;
        }
    }

    void handle_estop(const Inputs& in) {
        out = Outputs{};
        out.fault_code = static_cast<std::uint16_t>(FaultReason::ESTOP);

        // E-STOP 해제 + ACK로만 복귀
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
    FaultReason last_printed_reason = FaultReason::NONE;

    std::cout << "Controller started (HOLD: drive/lift/dump, FAULT latched)\n";

    while (true) {
        auto now = clock::now();

        // -------------------------
        // 10ms: Control loop
        // -------------------------
        if (now - last_control >= std::chrono::milliseconds(10)) {

            // ---- demo sequence ----
            if (tick10ms == 200)  in.drive_enable = true;
            if (tick10ms == 600)  in.drive_enable = false;

            if (tick10ms == 800)  in.lift_request = true;
            if (tick10ms == 980)  in.lift_complete = true;
            if (tick10ms == 990)  in.lift_complete = false;
            if (tick10ms == 1000) in.lift_request = false;

            if (tick10ms == 1200) in.dump_request = true;
            if (tick10ms == 1380) in.dump_complete = true;
            if (tick10ms == 1390) in.dump_complete = false;
            if (tick10ms == 1400) in.dump_request = false;

            if (tick10ms == 1450) in.estop_button = true;
            if (tick10ms == 1500) { in.estop_button = false; in.operator_ack = true; }
            if (tick10ms == 1510) in.operator_ack = false;

            // FAULT demo (comms lost)
            if (tick10ms == 1600) in.comms_ok = false;
            if (tick10ms == 1800) in.comms_ok = true;        // 복구돼도 latch 유지
            if (tick10ms == 1900) in.operator_ack = true;    // no_active_fault && ACK면 해제
            if (tick10ms == 1910) in.operator_ack = false;

            ctrl.step(in);
            plant.step(ctrl.out, in, 0.01);

            last_control = now;
            tick10ms++;
        }

        // -------------------------
        // 100ms: Diagnostics (no_active_fault 갱신)
        // -------------------------
        if (now - last_diag >= std::chrono::milliseconds(100)) {
            // ✅ 여기서도 "필터된 comms"를 사용해야 논리가 일관됨
            in.no_active_fault =
                   ctrl.get_comms_ok_filtered()
                && !in.can_timeout
                && !in.critical_dtc
                && !in.lift_timeout
                && !in.lift_sensor_error
                && !in.dump_timeout
                && !in.dump_sensor_error
                && !in.estop_button;

            last_diag = now;
        }

        // -------------------------
        // 50ms: Logging
        // -------------------------
        if (now - last_log >= std::chrono::milliseconds(50)) {
            // ✅ fault reason 변화 있을 때 한 번 더 강조 로그(보기 편하게)
            if (ctrl.fault_latched && ctrl.latched_reason != last_printed_reason) {
                std::cout << "!!! FAULT LATCHED: "
                          << fault_to_string(ctrl.latched_reason)
                          << " (code=" << static_cast<std::uint16_t>(ctrl.latched_reason) << ")\n";
                last_printed_reason = ctrl.latched_reason;
            }
            if (!ctrl.fault_latched) last_printed_reason = FaultReason::NONE;

            std::cout << "[tick=" << tick10ms
                      << "] state=" << state_to_string(ctrl.state)
                      << " drive_en=" << (in.drive_enable ? 1 : 0)
                      << " lift_btn=" << (in.lift_request ? 1 : 0)
                      << " dump_btn=" << (in.dump_request ? 1 : 0)
                      << " estop=" << (in.estop_button ? 1 : 0)
                      << " comms_raw=" << (in.comms_ok ? 1 : 0)
                      << " comms_filt=" << (ctrl.get_comms_ok_filtered() ? 1 : 0)
                      << " fault_latch=" << (ctrl.fault_latched ? 1 : 0)
                      << " fault_reason=" << (ctrl.fault_latched ? fault_to_string(ctrl.latched_reason) : "NONE")
                      << " fault_code=" << ctrl.out.fault_code
                      << " | OUT drive=" << (ctrl.out.drive_cmd ? 1 : 0)
                      << " lift=" << (ctrl.out.lift_cmd ? 1 : 0)
                      << " dump=" << (ctrl.out.dump_cmd ? 1 : 0)
                      << " vel=" << in.velocity
                      << " lift_p=" << plant.lift_pos
                      << " dump_p=" << plant.dump_pos
                      << "\n";

            last_log = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
