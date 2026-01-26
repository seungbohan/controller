#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include "passfail_criteria.hpp"

struct Sample {
    double t;       // seconds
    double target;  // in.target_velocity
    double vel;     // in.velocity
    double u;       // out.motor_cmd
};

struct Metrics {
  double rise_time = std::numeric_limits<double>::quiet_NaN();
  double overshoot_pct = 0.0;
  double settling_time = std::numeric_limits<double>::quiet_NaN();
  double ss_error = std::numeric_limits<double>::quiet_NaN();
  double max_sat_duration = 0.0; // optional
};

struct PassFail {
  bool pr01_rise = false;
  bool pr02_over = false;
  bool pr03_settle = false;
  bool pr04_ss = false;
  bool pr05_sat = true; // optional default true
};

// ===== Multi-step support =====
struct StepCase {
  const char* name;
  double step_time;  // seconds
  double end_time;
  double v0;         // pre-step target
  double v1;         // post-step target
};

struct StepResult {
  const char* name;
  Metrics m;
  PassFail pf;
};

inline Metrics compute_metrics_step(const std::vector<Sample>& log,
                                    double step_time,
                                    double t_end,
                                    double v0,
                                    double v1) {
  Metrics m{};
  const double delta = v1 - v0;

  // 1) 90% transition time
  const double thr = v0 + 0.9 * delta;
  for (size_t i = 0; i < log.size(); ++i) {
    const auto& s = log[i];
    if (s.t < step_time || s.t >= t_end) continue;

    if (delta >= 0.0) {
      if (s.vel >= thr) { m.rise_time = s.t - step_time; break; }
    } else {
      if (s.vel <= thr) { m.rise_time = s.t - step_time; break; }
    }
  }

  // 2) Overshoot/undershoot (% of |delta|)
  double peak = (delta >= 0.0) ? -1e9 : 1e9;
  for (const auto& s : log) {
    if (s.t < step_time || s.t >= t_end) continue;
    if (delta >= 0.0) peak = std::max(peak, s.vel);
    else              peak = std::min(peak, s.vel);
  }
  const double mag = (delta >= 0.0) ? (peak - v1) : (v1 - peak);
  if (std::abs(delta) > 1e-9) m.overshoot_pct = std::max(0.0, mag / std::abs(delta) * 100.0);
  else m.overshoot_pct = 0.0;

  // 3) Settling time: must stay within band until t_end
  const double band = 0.05 * std::max(1e-9, std::abs(v1));
  const double lo = v1 - band;
  const double hi = v1 + band;

  for (size_t i = 0; i < log.size(); ++i) {
    const auto& si = log[i];
    if (si.t < step_time || si.t >= t_end) continue;

    bool stays = true;
    for (size_t j = i; j < log.size(); ++j) {
      const auto& sj = log[j];
      if (sj.t < step_time) continue;
      if (sj.t >= t_end) break;
      if (sj.vel < lo || sj.vel > hi) { stays = false; break; }
    }
    if (stays) { m.settling_time = si.t - step_time; break; }
  }

  // 4) Steady-state error: mean of last 0.5s inside [step_time, t_end)
  const double window = 0.5;
  const double t0 = std::max(step_time, t_end - window);

  double sum = 0.0; int cnt = 0;
  for (const auto& s : log) {
    if (s.t < t0 || s.t >= t_end) continue;
    sum += s.vel; cnt++;
  }
  if (cnt > 0) {
    const double mean = sum / cnt;
    m.ss_error = mean - v1;
  }

  // 5) Max contiguous saturation duration inside [step_time, t_end)
  double current = 0.0, best = 0.0;
  double last_t = std::numeric_limits<double>::quiet_NaN();

  for (const auto& s : log) {
    if (s.t < step_time || s.t >= t_end) continue;
    const bool sat = (std::abs(s.u) >= 0.999);

    if (sat) {
      if (std::isfinite(last_t)) current += (s.t - last_t);
    } else {
      best = std::max(best, current);
      current = 0.0;
    }
    last_t = s.t;
  }
  best = std::max(best, current);
  m.max_sat_duration = best;

  return m;
}


inline void print_step_report(const StepResult& r) {
  std::cout << "\n[" << r.name << "]\n";
  std::cout << "Rise/Fall(90%)   : " << r.m.rise_time << " s   (" << (r.pf.pr01_rise ? "PASS" : "FAIL") << ")\n";
  std::cout << "Overshoot/Unders : " << r.m.overshoot_pct << " %   (" << (r.pf.pr02_over ? "PASS" : "FAIL") << ")\n";
  std::cout << "Settling Time    : " << r.m.settling_time << " s   (" << (r.pf.pr03_settle ? "PASS" : "FAIL") << ")\n";
  std::cout << "Steady-State Err : " << r.m.ss_error << "     (" << (r.pf.pr04_ss ? "PASS" : "FAIL") << ")\n";
  std::cout << "Sat Duration     : " << r.m.max_sat_duration << " s (" << (r.pf.pr05_sat ? "PASS" : "FAIL") << ")\n";

  const bool all = r.pf.pr01_rise && r.pf.pr02_over && r.pf.pr03_settle && r.pf.pr04_ss && r.pf.pr05_sat;
  std::cout << "RESULT: " << (all ? "✅ PASS" : "❌ FAIL") << "\n";
}

inline void print_suite_summary(const std::vector<StepResult>& results) {
  std::cout << "\n==============================\n";
  std::cout << "[SUITE SUMMARY]\n";
  std::cout << "------------------------------\n";
  bool all_ok = true;
  for (const auto& r : results) {
    const bool ok = r.pf.pr01_rise && r.pf.pr02_over && r.pf.pr03_settle && r.pf.pr04_ss && r.pf.pr05_sat;
    all_ok = all_ok && ok;
    std::cout << r.name << " : " << (ok ? "PASS" : "FAIL") << "\n";
  }
  std::cout << "------------------------------\n";
  std::cout << "SUITE RESULT: " << (all_ok ? "✅ PASS" : "❌ FAIL") << "\n";
  std::cout << "==============================\n\n";
}

inline PassFail judge(const Metrics& m, const DriveCriteria& c) {
  PassFail pf;
  pf.pr01_rise   = std::isfinite(m.rise_time)     && (m.rise_time <= c.rise_time_max_s);
  pf.pr02_over   = std::isfinite(m.overshoot_pct) && (m.overshoot_pct <= c.overshoot_max_pct);
  pf.pr03_settle = std::isfinite(m.settling_time) && (m.settling_time <= c.settling_time_max_s);
  pf.pr04_ss     = std::isfinite(m.ss_error)      && (std::abs(m.ss_error) <= c.ss_error_max);
  pf.pr05_sat    = std::isfinite(m.max_sat_duration) && (m.max_sat_duration <= c.sat_max_s);
  return pf;
}