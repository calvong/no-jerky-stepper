// Motion strategy extension point.
//
// A strategy turns "go from this state to that target" into PolySegments. The axis, the
// step generator and the driver only consume PolySegments, so a new strategy (trapezoid,
// jerk-limited S-curve, time-optimal, ...) is just another implementation of this interface.
// Strategies run in task context, never in the ISR.
#pragma once

#include "motion/poly.hpp"
#include "nj/types.hpp"

namespace nj {

class MotionStrategy {
public:
    virtual ~MotionStrategy() = default;

    /// Plan a move from `from` to rest at `target`. duration <= 0 lets the strategy choose
    /// one from `lim`. Writes up to `max_out` segments to `out`, count to `*n_out`.
    virtual Status plan_move(const State& from, double target, double duration, const Limits& lim,
                             PolySegment* out, int max_out, int* n_out) const = 0;

    /// Plan a smooth stop (to v = a = 0, end position free) from `from`.
    virtual Status plan_stop(const State& from, double duration, const Limits& lim,
                             PolySegment* out, int max_out, int* n_out) const = 0;
};

/// The core strategy: one quintic per move, one quartic per stop.
class MinJerkStrategy final : public MotionStrategy {
public:
    Status plan_move(const State& from, double target, double duration, const Limits& lim,
                     PolySegment* out, int max_out, int* n_out) const override;
    Status plan_stop(const State& from, double duration, const Limits& lim,
                     PolySegment* out, int max_out, int* n_out) const override;
};

const MotionStrategy& min_jerk_strategy();

}  // namespace nj
