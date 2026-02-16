#pragma once
#include <cstdint>

constexpr int STATE_SIZE = 15;

struct TelemetryPacket
{
    double time;
    double state[STATE_SIZE];
    uint32_t flags;
    uint32_t counter;
};