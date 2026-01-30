#pragma once
#include "can_frame.hpp"
#include "../include/main_inputs_outputs.hpp"
#include <cstring>

// ---------- Encode ------------
inline CanFrame encode_cmd(const Inputs& in) {
    CanFrame f;
    f.id = 0x100;
    f.dlc = 4;

    int16_t vel = static_cast<int16_t>(in.target_velocity * 1000);

    f.data[0] = vel & 0xFF;
    f.data[1] = (vel >> 8) & 0xFF;
    f.data[2] = 
        (in.drive_enable ? 1 : 0) |
        (in.estop_button ? 2 : 0) |
        (in.operator_ack ? 4 : 0);
    f.data[3] =
        (in.comms_ok ? 1 : 0) |
        (in.battery_ok ? 2 : 0);

    return f;
}

inline CanFrame encode_act(const Outputs& out) {
    CanFrame f;
    f.id = 0x200;
    f.dlc = 2;

    int16_t cmd = static_cast<int16_t>(out.motor_cmd * 1000);
    f.data[0] = cmd & 0xFF;
    f.data[1] = (cmd >> 8) & 0xFF;

    return f;
}

// ---------- Decode ------------
inline void decode_cmd(const CanFrame f, Inputs& in) {
    if (f.id != 0x100) return;

    int16_t vel = f.data[0] | (f.data[1] << 8);
    in.target_velocity = vel / 1000.0;

    in.drive_enable = f.data[2] & 1;
    in.estop_button = f.data[2] & 2;
    in.operator_ack = f.data[2] & 4;

    in.comms_ok = f.data[3] & 1;
    in.battery_ok = f.data[3] & 2;
}