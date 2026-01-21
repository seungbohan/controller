#pragma once
struct DriveCriteria {
  double rise_time_max_s = 1.0;      // PR-01
  double overshoot_max_pct = 5.0;    // PR-02
  double settling_time_max_s = 2.0;  // PR-03 (±5%)
  double ss_error_max = 0.02;        // PR-04
  double sat_max_s = 0.300;          // PR-05
  double fault_cutoff_max_s = 0.010; // PR-06 (1 cycle = 10ms)
};
