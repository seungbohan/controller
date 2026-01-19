#pragma once
#include <algorithm>

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

    PID(double p, double i = 0.0, double d = 0.0)
        : kp(p), ki(i), kd(d),
          integ(0.0),
          integ_min(-5.0),
          integ_max(5.0),
          output_min(-1.0),
          output_max(1.0)
    {}

    void reset() {
        integ = 0.0;
        prev_error = 0.0;
        first = true;
    }

    double compute(double target, double current, double dt) {
        double error = target - current;

        integ += error * dt;
        integ = std::clamp(integ, integ_min, integ_max);

        double derr = 0.0;
        if(!first) {
            derr = (error - prev_error) / dt;
        }
        prev_error = error;
        first = false;

        double output = kp * error + ki * integ + kd * derr;

        return std::clamp(output, output_min, output_max);
    }
};