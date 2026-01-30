#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "controller_core.hpp"        
#include "plant.hpp"                  
#include "drivers/fakecan_bus.hpp"    
#include "drivers/fakecan_codec.hpp"

static constexpr double DT_S = 0.01;

static void set_stdin_nonblocking_raw(bool enable) {
    static termios oldt{};
    static bool saved = false;

    if (enable) {
        if (!saved) {
            tcgetattr(STDIN_FILENO, &oldt);
            saved = true;
        }
        termios newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    } else {
        if (saved) tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
  }
}

static int read_key_nonblock() {
    unsigned char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n==1) return c;
    return -1;
} 

static double clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int main() {
    ControllerCore core;
    Plant plant;
    FakeCanBus bus;

    Inputs in{};
    Outputs out{};

    in.drive_enable = true;
    in.battery_ok = true;
    in.comms_ok = true;
    in.target_velocity = 1.0;

    bus.push_rx(encode_cmd(in));


    std::cout
        << "==== FakeCAN Keyboard Demo ====\n"
        << "[W] target +0.1\n"
        << "[S] target -0.1\n"
        << "[D] drive_enable toggle\n"
        << "[SPACE] E-STOP toggle\n"
        << "[A] ACK pulse (1 tick)\n"
        << "[Q] quit\n\n";

    set_stdin_nonblocking_raw(true);

    bool running = true;
    bool ack_pulse = false;
    
    while(running) {
        int k = read_key_nonblock();
        if (k != -1) {
            if (k == 'q' || k == 'Q') {
                running = false;
            } else if (k == 'w' || k == 'W') {
                in.target_velocity = clamp(in.target_velocity + 0.1, -1.0, 1.0);
            } else if (k == 's' || k == 'S') {
                in.target_velocity = clamp(in.target_velocity - 0.1, -1.0, 1.0);
            } else if (k == 'd' || k == 'D') {
                in.drive_enable = !in.drive_enable;
            } else if (k == ' ') {
                in.estop_button = !in.estop_button;
            } else if (k == 'a' || k == 'A') {
                in.operator_ack = true;   // 1 tick만 true로 쏘기
                ack_pulse= true;
            }

            bus.push_rx(encode_cmd(in));
        }

        // RX: CAN -> Inputs 반영
        if (auto rx = bus.pop_rx()) {
            decode_cmd(*rx, in);
        }

        // Controle
        out = core.step(in, DT_S);
        plant.step(out, in, DT_S);

        // TX: Ouputs -> CAN
        bus.push_tx(encode_act(out));

        if (ack_pulse) {
            in.operator_ack = false;
            ack_pulse = false;
        }

        // Monitor
        static int div = 0;
        if (++div >= 10) {
            div = 0;
            auto dbg = core.debug();

            std::cout
            << "state=" << int(dbg.state)
            << " fault_latched=" << dbg.fault_latched
            << " drive=" << in.drive_enable
            << " estop=" << in.estop_button
            << " target=" << in.target_velocity
            << " vel=" << in.velocity
            << " cmd=" << out.motor_cmd
            << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    set_stdin_nonblocking_raw(false);
    std::cout << "bye\n";
    return 0;
}