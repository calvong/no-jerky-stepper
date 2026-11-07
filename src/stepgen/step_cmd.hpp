// Step command: the unit of work between the planner task (producer) and the RMT encoder (ISR).
//
// A command is `steps` step periods of `ticks` each. Each period is low first and ends with
// the step pulse, so the pulse of the k-th step ends exactly k * ticks after the command
// starts. steps == 0 is a pause of `ticks` (all low).
// The direction bit applies to the pulses of the command; when it differs from the current
// DIR level, the encoder performs the DIR-change procedure inside the leading low time of the
// first period (see step_encoder.hpp).
#pragma once

#include <cstdint>

namespace nj {

struct StepCmd {
    uint32_t ticks = 0;
    uint16_t steps = 0;
    uint8_t flags = 0;
    uint8_t reserved = 0;

    static constexpr uint8_t kDirPositive = 0x01;

    bool dir_positive() const { return (flags & kDirPositive) != 0; }

    /// Total duration of the command in ticks (modulo 2^32).
    uint32_t total_ticks() const { return steps == 0 ? ticks : ticks * static_cast<uint32_t>(steps); }

    static StepCmd pause(uint32_t ticks)
    {
        StepCmd c;
        c.ticks = ticks;
        return c;
    }

    static StepCmd step(uint32_t ticks, uint16_t steps, bool dir_positive)
    {
        StepCmd c;
        c.ticks = ticks;
        c.steps = steps;
        c.flags = dir_positive ? kDirPositive : 0;
        return c;
    }
};

}  // namespace nj
