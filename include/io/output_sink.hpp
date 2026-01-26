#pragma once
#include <cstdint>
#include "../main_inputs_outputs.hpp"

// Outputs + 메타 묶음
struct OutputFrame {
    Outputs out{};
    std::uint64_t t_us = 0;
};

class IOutputSink {
public:
    virtual ~IOutputSink() = default;
    virtual void write(const OutputFrame& frame) = 0;
};
