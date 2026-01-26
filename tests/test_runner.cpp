#include <vector>
#include <iostream>
#include <cmath>

#include "../src/controller_core.hpp"
#include "../sim/plant.hpp"

#include "metrics/drive_metrics.hpp"
#include "scenarios/drive_step_0_1.hpp"
#include "scenarios/drive_step_1_03.hpp"
#include "scenarios/drive_step_03_08.hpp"
#include "scenarios/fault_estop.hpp"
#include "scenarios/comms_lost_latch.hpp"

static constexpr double DT_S = 0.01;

// =======================
// Drive scenario runner
// =======================
template <typename DriveScenario>
static StepResult run_drive_case(const DriveScenario& sc,
                                 ControllerCore& core,
                                 Plant& plant) {
  Inputs in{};
  Outputs out{};

  sc.init(in);

  std::vector<Sample> log;
  log.reserve(sc.end_tick() + 10);

  for (int tick = 0; tick < sc.end_tick(); ++tick) {
    sc.apply(tick, in);          // command 생성
    out = core.step(in, DT_S);   // 제어기
    plant.step(out, in, DT_S);   // plant + sensor

    const double t = tick * DT_S;
    log.push_back(Sample{
      t,
      in.target_velocity,
      in.velocity,
      out.motor_cmd
    });
  }

  const double step_time = sc.step_tick() * DT_S;
  const double end_time  = sc.end_tick()  * DT_S;

  Metrics m = compute_metrics_step(
    log, step_time, end_time, sc.v0(), sc.v1()
  );

  DriveCriteria crit;
  PassFail pf = judge(m, crit);

  return StepResult{sc.name(), m, pf};
}

// =======================
// Fault: E-STOP
// =======================
static bool run_fault_estop_case() {
  ControllerCore core; core.reset();
  Plant plant;
  FaultEstop sc;

  Inputs in{};
  Outputs out{};

  sc.init(in);

  bool cutoff_ok = false;
  bool cleared_ok = false;

  const int estop_on = sc.estop_on_tick();

  for (int tick = 0; tick < sc.end_tick(); ++tick) {
    sc.apply(tick, in);

    out = core.step(in, DT_S);
    plant.step(out, in, DT_S);

    auto dbg = core.debug();

    if (tick == estop_on + 1) {
      cutoff_ok = (std::abs(out.motor_cmd) < 1e-9);
    }

    if (tick > sc.ack_tick() + 20) {
      if (!dbg.fault_latched) cleared_ok = true;
    }
  }

  std::cout << "\n[" << sc.name() << "]\n";
  std::cout << "Cutoff <= 1 cycle : " << (cutoff_ok ? "PASS" : "FAIL") << "\n";
  std::cout << "Cleared after ACK : " << (cleared_ok ? "PASS" : "FAIL") << "\n";
  std::cout << "RESULT: "
            << ((cutoff_ok && cleared_ok) ? "✅ PASS" : "❌ FAIL")
            << "\n\n";

  return cutoff_ok && cleared_ok;
}

// =======================
// Fault: COMMS lost
// =======================
static bool run_comms_lost_case() {
  ControllerCore core; core.reset();
  Plant plant;
  CommsLostLatch sc;

  Inputs in{};
  Outputs out{};

  sc.init(in);

  bool fault_latched_seen = false;
  bool cleared_ok = false;

  for (int tick = 0; tick < sc.end_tick(); ++tick) {
    sc.apply(tick, in);

    out = core.step(in, DT_S);
    plant.step(out, in, DT_S);

    auto dbg = core.debug();

    if (dbg.fault_latched)
      fault_latched_seen = true;

    if (tick > sc.ack_tick() + 20) {
      if (!dbg.fault_latched)
        cleared_ok = true;
    }
  }

  std::cout << "\n[" << sc.name() << "]\n";
  std::cout << "Fault latched seen : "
            << (fault_latched_seen ? "PASS" : "FAIL") << "\n";
  std::cout << "Cleared after ACK  : "
            << (cleared_ok ? "PASS" : "FAIL") << "\n";
  std::cout << "RESULT: "
            << ((fault_latched_seen && cleared_ok) ? "✅ PASS" : "❌ FAIL")
            << "\n\n";

  return fault_latched_seen && cleared_ok;
}

// =======================
// main
// =======================
int main() {
  // ---- Drive suite ----
  {
    ControllerCore core; core.reset();
    Plant plant;

    DriveStep_0_1  s1;
    DriveStep_1_03 s2;
    DriveStep_03_08 s3;

    std::vector<StepResult> results;
    results.reserve(3);

    auto r1 = run_drive_case(s1, core, plant);
    print_step_report(r1);
    results.push_back(r1);

    core.reset(); plant = Plant{};
    auto r2 = run_drive_case(s2, core, plant);
    print_step_report(r2);
    results.push_back(r2);

    core.reset(); plant = Plant{};
    auto r3 = run_drive_case(s3, core, plant);
    print_step_report(r3);
    results.push_back(r3);

    print_suite_summary(results);
  }

  // ---- Fault tests ----
  bool ok_estop = run_fault_estop_case();
  bool ok_comms = run_comms_lost_case();

  return (ok_estop && ok_comms) ? 0 : 1;
}
