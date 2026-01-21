#pragma once
#include <algorithm>
#include <cmath>

class PID {
public:
    double kp, ki, kd;

    double integ;
    double integ_min;
    double integ_max;

    double prev_error = 0.0;
    bool first = true;

    double output_min;
    double output_max;

    struct PIDDebug {
        double error = 0.0;
        double integ = 0.0;
        double u_unsat = 0.0;
        double u_sat = 0.0;
        bool   would_worsen = false;
    };
    PIDDebug dbg = {};  // 명시적 초기화

    PID(double p, double i = 0.0, double d = 0.0)
        : kp(p), ki(i), kd(d),
          integ(0.0),
          integ_min(-5.0),
          integ_max(5.0),
          output_min(-1.0),
          output_max(1.0)
          // dbg는 멤버 초기화로 이미 0 초기화됨
    {}

    void reset() {
        integ = 0.0;
        prev_error = 0.0;
        first = true;
        dbg = PIDDebug{};
    }

    double compute(double target, double current, double dt) {
        const double error = target - current;

        double derr = 0.0;
        if (!first) derr = (error - prev_error) / dt;
        prev_error = error;
        first = false;

        const double u_unsat = kp * error + ki * integ + kd * derr;

        const bool saturating_high = (u_unsat > output_max);
        const bool saturating_low  = (u_unsat < output_min);

        const bool would_worsen =
            (saturating_high && error > 0) ||
            (saturating_low  && error < 0);

        if (ki != 0.0 && !would_worsen) {
            integ += error * dt;
            integ = std::clamp(integ, integ_min, integ_max);
        }

        const double output = kp * error + ki * integ + kd * derr;
        const double u_sat = std::clamp(output, output_min, output_max);

        dbg.error = error;
        dbg.integ = integ;
        dbg.u_unsat = u_unsat;
        dbg.u_sat = u_sat;
        dbg.would_worsen = would_worsen;

        return u_sat;
    }
};