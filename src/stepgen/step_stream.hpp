// StepStream: everything shared between the producer (planner task / StepGenerator) and the
// consumer (StepEncoder in the RMT ISR) of one channel.
#pragma once

#include <atomic>
#include <cstdint>

#include "config.hpp"
#include "stepgen/spsc_ring.hpp"
#include "stepgen/step_cmd.hpp"

namespace nj {

struct StepStream {
    SpscRing<StepCmd, cfg::kStepQueueLen> ring;

    /// Sum of total_ticks() of pushed / consumed commands (modulo 2^32).
    std::atomic<uint32_t> pushed_ticks{0};
    std::atomic<uint32_t> consumed_ticks{0};

    /// Set by the producer when no more commands will come (motion finished). The encoder ends
    /// the RMT transaction once the ring is empty and this is set.
    std::atomic<bool> producer_idle{true};

    /// The encoder notifies the producer once when the lead drops below `low_water_ticks`.
    std::atomic<bool> notify_armed{false};
    uint32_t low_water_ticks = cfg::kLookaheadTicks / 2;

    // Statistics written by the encoder.
    std::atomic<uint32_t> underruns{0};
    std::atomic<uint32_t> stretched_ticks{0};

    bool push(const StepCmd& c)
    {
        if (!ring.push(c)) {
            return false;
        }
        pushed_ticks.fetch_add(c.total_ticks(), std::memory_order_relaxed);
        return true;
    }

    /// Time queued ahead of the encoder [ticks].
    uint32_t lead_ticks() const
    {
        return pushed_ticks.load(std::memory_order_relaxed) - consumed_ticks.load(std::memory_order_relaxed);
    }

    /// Reset after a hard stop (consumer stopped).
    void flush()
    {
        ring.clear();
        consumed_ticks.store(pushed_ticks.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
};

}  // namespace nj
