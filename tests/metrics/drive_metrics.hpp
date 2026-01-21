#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

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

static Metrics compute_metrics_step01(const std::vector<Sample>& log,
                                     double step_time,
                                     double target_final) {
  Metrics m{};

  // 1) rise time (0->90%)
  // 기준값: 0.9*target_final (초기값이 0이라고 가정)
  const double v90 = 0.9 * target_final;
  double t_rise = std::numeric_limits<double>::quiet_NaN();

  for (size_t i = 0; i < log.size(); ++i) {
    if (log[i].t < step_time) continue;
    if (log[i].vel >= v90) { t_rise = log[i].t - step_time; break; }
  }
  m.rise_time = t_rise;

  // 2) overshoot
  double vmax = -1e9;
  for (auto& s : log) {
    if (s.t < step_time) continue;
    vmax = std::max(vmax, s.vel);
  }
  if (target_final > 1e-9) {
    m.overshoot_pct = std::max(0.0, (vmax - target_final) / target_final * 100.0);
  }

  // 3) settling time (±5%)
  const double band = 0.05 * std::abs(target_final);
  const double lo = target_final - band;
  const double hi = target_final + band;

  // "어느 시점 이후로 끝까지 밴드 안"을 만족하는 최초 시점 찾기
  double t_settle = std::numeric_limits<double>::quiet_NaN();
  for (size_t i = 0; i < log.size(); ++i) {
    if (log[i].t < step_time) continue;

    bool stays = true;
    for (size_t j = i; j < log.size(); ++j) {
      if (log[j].vel < lo || log[j].vel > hi) { stays = false; break; }
    }
    if (stays) { t_settle = log[i].t - step_time; break; }
  }
  m.settling_time = t_settle;

  // 4) steady-state error (마지막 0.5초 평균)
  const double window = 0.5;
  const double t_end = log.empty() ? 0.0 : log.back().t;
  const double t0 = std::max(step_time, t_end - window);

  double sum = 0.0; int cnt = 0;
  for (auto& s : log) {
    if (s.t < t0) continue;
    sum += s.vel; cnt++;
  }
  if (cnt > 0) {
    double mean = sum / cnt;
    m.ss_error = mean - target_final;
  }

  // 5) saturation duration (optional): |u| >= 0.999 연속 최대 길이
  double current = 0.0, best = 0.0;
  for (size_t i = 0; i < log.size(); ++i) {
    if (log[i].t < step_time) continue;
    bool sat = (std::abs(log[i].u) >= 0.999);
    if (sat) current += (i>0 ? (log[i].t - log[i-1].t) : 0.0);
    else { best = std::max(best, current); current = 0.0; }
  }
  best = std::max(best, current);
  m.max_sat_duration = best;

  return m;
}   

static PassFail judge(const Metrics& m, const Criteria& c) {
  PassFail pf;
  pf.pr01_rise   = std::isfinite(m.rise_time)     && (m.rise_time <= c.rise_max);
  pf.pr02_over   = std::isfinite(m.overshoot_pct) && (m.overshoot_pct <= c.overshoot_max_pct);
  pf.pr03_settle = std::isfinite(m.settling_time) && (m.settling_time <= c.settle_max);
  pf.pr04_ss     = std::isfinite(m.ss_error)      && (std::abs(m.ss_error) <= c.ss_err_max);
  pf.pr05_sat    = std::isfinite(m.max_sat_duration) && (m.max_sat_duration <= c.sat_max);
  return pf;
}
