#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

#include "../sim/plant.hpp"
#include "controller_core.hpp"
#include "logger.hpp"

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

// =====================
// Main (demo)
// =====================
int main() {
    constexpr bool ENABLE_CONSOLE_LOG = true;
    constexpr bool ENABLE_CSV_LOG     = true;

    constexpr double DT_S = 0.01;
    
    ControllerCore core;
    ControllerDebug dbg = core.debug();
    State st = static_cast<State>(dbg.state);
    FaultReason fr = static_cast<FaultReason>(dbg.fault_code);
    Inputs in;
    Plant plant;
    Outputs out{}; 
    CSVLogger csv("drive_pid_log.csv", ENABLE_CSV_LOG);

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
        // ============================
        // 목적: 5개 시나리오 PASS/FAIL을 로그로 확인
        // 각 시나리오 사이에 1~2초 "휴지(Idle)"를 넣어서 보기 쉽게 함.
        //
        // 시간 환산: tick10ms 100 = 1초
        // ============================

        // ---- defaults that persist unless changed ----
        if (now - last_control >= std::chrono::milliseconds(10)) {

        // ============================
        // (1) HOLD 입력은 매 tick 기본값을 0으로 리셋
        // ============================
        in.drive_enable = false;
        in.lift_request = false;
        in.dump_request = false;
        in.operator_ack = false;

        // comms_ok도 "기본 true"로 두고, 시나리오에서만 false로 만들면 깔끔함
        in.comms_ok = true;

        // ============================
        // (2) 1회성 초기화 (최초 1번만)
        // ============================
        if (tick10ms == 0) {
            in.velocity = 0.0;
            in.battery_ok = true;
            in.estop_button = false;

            in.can_timeout = false;
            in.critical_dtc = false;
            in.lift_timeout = false;
            in.lift_sensor_error = false;
            in.dump_timeout = false;
            in.dump_sensor_error = false;

            in.lift_complete = false;
            in.dump_complete = false;
        }

        // ============================
        // (3) 5개 시나리오 스케줄
        // ============================

        // Scenario 1: DRIVE hold
        if (tick10ms >= 100 && tick10ms < 2050) {
            in.drive_enable = true;
        }

        // // Scenario 2: LIFT hold + complete + inhibit
        // if (tick10ms >= 450 && tick10ms < 600) {
        //     in.lift_request = true;
        // }
        // if (tick10ms == 550) in.lift_complete = true;
        // if (tick10ms == 560) in.lift_complete = false;

        // // Scenario 3: lift + dump 동시에
        // if (tick10ms >= 800 && tick10ms < 850) {
        //     in.lift_request = true;
        //     in.dump_request = true;
        // }

        // // Scenario 4: E-STOP
        // if (tick10ms >= 950 && tick10ms < 1050) {
        //     in.drive_enable = true;
        // }
        // if (tick10ms >= 1000 && tick10ms < 1020) {
        //     in.estop_button = true;
        // }
        // if (tick10ms == 1020) in.estop_button = false;
        // if (tick10ms == 1030) in.operator_ack = true; // ACK 펄스

        // // Scenario 5: COMMS_LOST (필터 타임아웃 넘기도록 300ms 끊기)
        // if (tick10ms >= 1150 && tick10ms < 1180) {
        //     in.comms_ok = false;
        // }
        // // 복구 후 200ms 정도 기다렸다 ACK (필터 복구 안정시간 100ms + diag 갱신 고려)
        // if (tick10ms == 1220) in.operator_ack = true;

        // ============================
        // (4) 컨트롤러/플랜트 실행 
        // ============================
        out = core.step(in, DT_S);
        plant.step(out, in, DT_S);

        // ============================
        // (5) tick 증가 
        // ============================
        tick10ms++;
        last_control = now;
    }

        // ============================
        // End of test suite
        // ============================


        // -------------------------
        // 100ms: Diagnostics (no_active_fault 갱신)
        // -------------------------
        if (now - last_diag >= std::chrono::milliseconds(100)) {
                

            in.no_active_fault =
                   dbg.comms_ok_filtered
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
            if (dbg.fault_latched && fr != last_printed_reason) {
                std::cout << "!!! FAULT LATCHED: "
                          << fault_to_string(fr)
                          << " (code=" << static_cast<std::uint16_t>(fr) << ")\n";
                last_printed_reason = fr;
            }
            if (!dbg.fault_latched) last_printed_reason = FaultReason::NONE;

            if (ENABLE_CONSOLE_LOG) {
            std::cout << "[tick=" << tick10ms
                      << "] state=" << state_to_string(st)
                      << " drive_en=" << (in.drive_enable ? 1 : 0)
                      << " lift_btn=" << (in.lift_request ? 1 : 0)
                      << " dump_btn=" << (in.dump_request ? 1 : 0)
                      << " estop=" << (in.estop_button ? 1 : 0)
                      << " comms_raw=" << (in.comms_ok ? 1 : 0)
                      << " comms_filt=" << (dbg.comms_ok_filtered ? 1 : 0)
                      << " fault_latch=" << (dbg.fault_latched ? 1 : 0)
                      << " fault_reason=" << (dbg.fault_latched ? fault_to_string(fr) : "NONE")
                      << " fault_code=" << out.fault_code
                      << " | OUT drive=" << (out.drive_cmd ? 1 : 0)
                      << " lift=" << (out.lift_cmd ? 1 : 0)
                      << " dump=" << (out.dump_cmd ? 1 : 0)
                      << " vel=" << in.velocity
                      << " motor_cmd=" << out.motor_cmd
                      << " lift_p=" << plant.lift_pos
                      << " dump_p=" << plant.dump_pos
                      << " integ=" << dbg.pid_dbg.integ
                      << " u_unsat=" << dbg.pid_dbg.u_unsat
                      << " u_sat=" << dbg.pid_dbg.u_sat
                      << " windup_block=" << (dbg.pid_dbg.would_worsen ? 1 : 0)
                      << "\n";     
            }
                // csv log    
                csv.log(
                tick10ms,
                DT_S,
                state_to_string(st),
                (in.drive_enable ? 1 : 0),
                (in.lift_request ? 1 : 0),
                (in.dump_request ? 1 : 0),
                (in.estop_button ? 1 : 0),
                (in.comms_ok ? 1 : 0),
                (dbg.comms_ok_filtered ? 1 : 0),
                (dbg.fault_latched ? 1 : 0),
                (dbg.fault_latched ? fault_to_string(fr) : "NONE"),
                static_cast<unsigned>(dbg.fault_code),
                (out.drive_cmd ? 1 : 0),
                (out.lift_cmd ? 1 : 0),
                (out.dump_cmd ? 1 : 0),
                in.target_velocity,
                in.velocity,
                out.motor_cmd,
                dbg.pid_dbg.integ,
                dbg.pid_dbg.u_unsat,
                dbg.pid_dbg.u_sat,
                (dbg.pid_dbg.would_worsen ? 1 : 0),
                plant.lift_pos,
                plant.dump_pos
            );

            last_log = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
