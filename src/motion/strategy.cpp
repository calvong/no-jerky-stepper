#include "motion/strategy.hpp"

#include <cmath>

#include "motion/min_jerk.hpp"

namespace nj {

Status MinJerkStrategy::plan_move(const State& from, double target, double duration, const Limits& lim,
                                  PolySegment* out, int max_out, int* n_out) const
{
    *n_out = 0;
    if (max_out < 1 || !std::isfinite(target)) {
        return Status::InvalidArg;
    }
    const State to{target, 0.0, 0.0};
    double T = duration;
    if (!(T > 0.0)) {
        if (!lim.any()) {
            return Status::InvalidArg;
        }
        // Best effort when the start state makes the limits unreachable (e.g. limits lowered
        // while moving): the duration with the smallest limit excess is used.
        T = min_jerk_duration(from, to, lim);
    }
    T = std::fmax(T, kMinSegmentDuration);
    out[0] = min_jerk(from, to, T);
    *n_out = 1;
    return Status::Ok;
}

Status MinJerkStrategy::plan_stop(const State& from, double duration, const Limits& lim,
                                  PolySegment* out, int max_out, int* n_out) const
{
    *n_out = 0;
    if (max_out < 1) {
        return Status::InvalidArg;
    }
    if (from.v == 0.0 && from.a == 0.0) {
        return Status::Ok;  // already at rest
    }
    double T = duration;
    if (!(T > 0.0)) {
        if (lim.any()) {
            T = stop_duration(from, lim);  // best effort even if not strictly feasible
        } else {
            T = 0.1;
        }
    }
    T = std::fmax(T, kMinSegmentDuration);
    out[0] = min_jerk_free_end(from, 0.0, T);
    *n_out = 1;
    return Status::Ok;
}

const MotionStrategy& min_jerk_strategy()
{
    static const MinJerkStrategy s;
    return s;
}

}  // namespace nj
