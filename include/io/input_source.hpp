#pragma once
#include <cstdint>
#include "../main_inputs_outputs.hpp"

// Inputs + 메타(시간/유효성) 묶음
struct InputFrame {
    Inputs in{};
    std::uint64_t t_us = 0;
    bool valid = true;
};

class IInputSource {
public:
    virtual ~IInputSource() = default;
    // tick마다 poll해서 Inputs를 갱신
    virtual bool read(InputFrame& frame) = 0;
};
