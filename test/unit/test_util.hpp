// Helpers shared by the unit tests (host and on-target/QEMU).
// Step streams are checked on the fly (StepChecker) so the tests fit in ESP32 RAM.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "core/axis.hpp"
#include "stepgen/step_stream.hpp"
#include "unity.h"

namespace njt {

using namespace nj;

/// Deterministic pseudo-random numbers.
struct Rng {
    uint32_t s = 12345;
    uint32_t next()
    {
        s = s * 1664525u + 1013904223u;
        return s;
    }
    double uniform(double lo, double hi) { return lo + (hi - lo) * (next() >> 8) / 16777216.0; }
};

/// A copy of a trajectory's segments (Trajectory discards consumed segments as it runs).
struct TrajCopy {
    std::vector<Trajectory::Entry> e;

    double p(double t) const
    {
        for (const auto& x : e) {
            if (t < x.t0 + x.seg.T) {
                return x.seg.p(t < x.t0 ? 0.0 : t - x.t0);
            }
        }
        return e.empty() ? 0.0 : e.back().seg.end().p;
    }

    /// Merge the live segments of `tr` (call after every planning call).
    void capture(const Trajectory& tr)
    {
        if (tr.size() == 0) {
            return;
        }
        const double t_first = tr.entry(0).t0;
        while (!e.empty() && e.back().t0 >= t_first) {
            e.pop_back();
        }
        if (!e.empty() && e.back().t0 + e.back().seg.T > t_first) {
            e.back().seg.T = t_first - e.back().t0;
        }
        for (int i = 0; i < tr.size(); ++i) {
            e.push_back(tr.entry(i));
        }
    }
};

/// Replays commands as they are produced and accumulates statistics. Assumes the stream is
/// continuous from tick `t` (no idle gaps), which holds for every test that uses it.
struct StepChecker {
    const TrajCopy* tc = nullptr;  ///< optional: tracking error against this trajectory
    int64_t t = 0;
    int64_t pos = 0;
    int64_t steps = 0;
    size_t cmds = 0;
    size_t pauses = 0;
    int64_t last_step_t = INT64_MIN;
    int last_dir = 0;
    int dir_changes = 0;
    bool dir_change_first_of_cmd = true;
    uint32_t min_dir_change_ticks = UINT32_MAX;
    int64_t min_interval = INT64_MAX;
    double max_err = 0.0;        ///< max |p(t_k) - pos_k|
    double max_cross_err = 0.0;  ///< max |p(t_k) - (pos_k - dir/2)|: exactness of crossings

    void feed(const StepCmd& c)
    {
        ++cmds;
        if (c.steps == 0) {
            ++pauses;
            t += c.ticks;
            return;
        }
        const int d = c.dir_positive() ? 1 : -1;
        for (uint32_t i = 0; i < c.steps; ++i) {
            t += c.ticks;
            pos += d;
            ++steps;
            if (last_dir != 0 && d != last_dir) {
                ++dir_changes;
                dir_change_first_of_cmd = dir_change_first_of_cmd && i == 0;
                if (c.ticks < min_dir_change_ticks) {
                    min_dir_change_ticks = c.ticks;
                }
            }
            if (last_step_t != INT64_MIN && t - last_step_t < min_interval) {
                min_interval = t - last_step_t;
            }
            last_step_t = t;
            last_dir = d;
            if (tc) {
                const double p = tc->p(static_cast<double>(t) / cfg::kTickHz);
                max_err = std::fmax(max_err, std::fabs(p - static_cast<double>(pos)));
                max_cross_err = std::fmax(max_cross_err, std::fabs(p - (static_cast<double>(pos) - 0.5 * d)));
            }
        }
    }
};

inline void drain(StepStream& s, StepChecker& chk)
{
    StepCmd c;
    while (s.ring.pop(c)) {
        s.consumed_ticks.fetch_add(c.total_ticks());
        chk.feed(c);
    }
}

/// Run an axis until its motion is finished.
inline void run_to_end(Axis& ax, StepStream& s, StepChecker& chk, double max_time = 600.0)
{
    const int64_t max_tick = static_cast<int64_t>(max_time * cfg::kTickHz);
    while (ax.has_motion() && ax.commit_tick() < max_tick) {
        ax.generate(s, ax.commit_tick() + cfg::kLookaheadTicks, false);
        drain(s, chk);
    }
    drain(s, chk);
}

/// Generate until the commit horizon reaches `t` [s].
inline void run_until(Axis& ax, StepStream& s, StepChecker& chk, double t, bool keep_alive = false)
{
    const int64_t until = static_cast<int64_t>(t * cfg::kTickHz);
    while (ax.commit_tick() < until) {
        const auto r = ax.generate(s, until, keep_alive);
        drain(s, chk);
        if (r == StepGenerator::Result::Idle) {
            break;
        }
    }
}

inline AxisConfig default_axis_config(Limits lim = {})
{
    AxisConfig c;
    c.gen = make_gen_config(3 * cfg::kTicksPerUs, 5 * cfg::kTicksPerUs);
    c.limits = lim;
    return c;
}

/// Tracking bound: 0.5 step at an exact crossing, + hysteresis around reversals, + a little
/// for the uniform spacing inside a slice.
constexpr double kTrackBound = 0.5 + cfg::kReversalHysteresis + 0.25;

}  // namespace njt
