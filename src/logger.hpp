#pragma once
#include <fstream>
#include <string>
#include <iomanip>

struct CSVLogger {
    std::ofstream file;
    bool enabled = true;

    CSVLogger(const std::string& path, bool enable = true)
        : enabled(enable)
    {
        if (!enabled) return;
        file.open(path);
        // 헤더(원하는 컬럼 더 추가해도 됨)
        file << "tick,time_s,state,drive_en,lift_btn,dump_btn,estop,"
                "comms_raw,comms_filt,fault_latch,fault_reason,fault_code,"
                "drive_cmd,lift_cmd,dump_cmd,target_vel,vel,motor_cmd,"
                "integ,u_unsat,u_sat,windup_block,lift_p,dump_p\n";
        file.flush();
    }

    ~CSVLogger() {
        if (enabled && file.is_open()) file.close();
    }

    void log(
        int tick,
        double dt_s,
        const std::string& state,
        int drive_en, int lift_btn, int dump_btn, int estop,
        int comms_raw, int comms_filt,
        int fault_latch, const std::string& fault_reason, unsigned fault_code,
        int drive_cmd, int lift_cmd, int dump_cmd,
        double target_vel, double vel, double motor_cmd,
        double integ, double u_unsat, double u_sat, int windup_block,
        double lift_p, double dump_p
    ) {
        if (!enabled || !file.is_open()) return;

        const double time_s = tick * dt_s;

        file << tick << ","
             << std::fixed << std::setprecision(3) << time_s << ","
             << state << ","
             << drive_en << "," << lift_btn << "," << dump_btn << "," << estop << ","
             << comms_raw << "," << comms_filt << ","
             << fault_latch << "," << fault_reason << "," << fault_code << ","
             << drive_cmd << "," << lift_cmd << "," << dump_cmd << ","
             << std::setprecision(3) << target_vel << ","
             << std::setprecision(6) << vel << ","
             << std::setprecision(6) << motor_cmd << ","
             << std::setprecision(6) << integ << ","
             << std::setprecision(6) << u_unsat << ","
             << std::setprecision(6) << u_sat << ","
             << windup_block << ","
             << std::setprecision(6) << lift_p << ","
             << std::setprecision(6) << dump_p
             << "\n";
    }
};
