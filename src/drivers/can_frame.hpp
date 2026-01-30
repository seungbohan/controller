#pragma once
#include <cstdint>

struct CanFrame 
{
    uint32_t id = 0;
    uint8_t dlc = 0;
    uint8_t data[8]{};
    uint64_t t_us = 0;
};
