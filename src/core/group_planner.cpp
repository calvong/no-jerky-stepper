#include "core/group_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "motion/min_jerk.hpp"
#include "motion/path_planner.hpp"

namespace nj {

int64_t group_commit_tick(Axis* const* axes, int n)
{
    int64_t t = 0;
    for (int i = 0; i < n; ++i) {
        t = std::max(t, axes[i]->commit_tick());
    }
    return t;
}

void group_align(Axis* const* axes, int n)
{
    const int64_t t = group_commit_tick(axes, n);
    for (int i = 0; i < n; ++i) {
        axes[i]->rebase(t);
    }
}

Status group_move_to(Axis* const* axes, int n, const double* targets, double duration)
{
    if (n <= 0 || n > kMaxGroupAxes) {
        return Status::InvalidArg;
    }
    const int64_t tick = group_commit_tick(axes, n);
    double T = duration;
    if (!(T > 0.0)) {
        T = 0.0;
        for (int i = 0; i < n; ++i) {
            const State s0 = axes[i]->state_at(tick);
            // Best effort if a moving start state makes the limits unreachable.
            T = std::max(T, min_jerk_duration(s0, State{targets[i], 0.0, 0.0}, axes[i]->limits()));
        }
        if (!(T > 0.0)) {
            return Status::InvalidArg;  // no limits on any axis
        }
    }
    for (int i = 0; i < n; ++i) {
        if (!axes[i]->can_append(1, tick)) {
            return Status::NoMem;
        }
    }
    for (int i = 0; i < n; ++i) {
        const Status st = axes[i]->move_to(targets[i], T, tick);
        if (st != Status::Ok) {
            return st;
        }
    }
    return Status::Ok;
}

Status group_stop(Axis* const* axes, int n, double duration)
{
    if (n <= 0 || n > kMaxGroupAxes) {
        return Status::InvalidArg;
    }
    const int64_t tick = group_commit_tick(axes, n);
    double T = duration;
    if (!(T > 0.0)) {
        T = 0.0;
        for (int i = 0; i < n; ++i) {
            T = std::max(T, stop_duration(axes[i]->state_at(tick), axes[i]->limits()));
        }
        if (!(T > 0.0)) {
            T = 0.1;
        }
    }
    for (int i = 0; i < n; ++i) {
        const Status st = axes[i]->stop(T, tick);
        if (st != Status::Ok) {
            return st;
        }
    }
    return Status::Ok;
}

Status group_follow_path(Axis* const* axes, int n, const GroupWaypoint* wps, int m, const PathOptions& opts)
{
    if (n <= 0 || n > kMaxGroupAxes || m <= 0 || wps == nullptr) {
        return Status::InvalidArg;
    }
    if (m > cfg::kPathCapacity) {
        return Status::NoMem;
    }
    const int64_t tick = group_commit_tick(axes, n);

    // Per-axis problems in each axis' scratch; durations are filled per trial below.
    bool any_auto = false;
    double T0[cfg::kPathCapacity];
    for (int k = 0; k < m; ++k) {
        T0[k] = wps[k].duration > 0.0 ? wps[k].duration : 0.0;
        any_auto = any_auto || !(wps[k].duration > 0.0);
    }
    for (int i = 0; i < n; ++i) {
        PathProblem& P = axes[i]->problem();
        P.start = axes[i]->state_at(tick);
        P.n = m;
        P.policy = opts.policy;
        P.limits = axes[i]->limits();
        for (int k = 0; k < m; ++k) {
            P.pos[k] = wps[k].pos[i];
            P.vel[k] = std::numeric_limits<double>::quiet_NaN();
            P.acc[k] = std::numeric_limits<double>::quiet_NaN();
            // 0 marks an auto segment: path_limit_ratio() only checks those.
            P.duration[k] = wps[k].duration > 0.0 ? wps[k].duration : 0.0;
        }
        if (any_auto) {
            if (!P.limits.any()) {
                continue;  // this axis does not constrain the timing
            }
            double Ti[cfg::kPathCapacity];
            base_durations(P, Ti);
            for (int k = 0; k < m; ++k) {
                if (!(wps[k].duration > 0.0)) {
                    T0[k] = std::max(T0[k], Ti[k]);
                }
            }
        }
    }

    double T[cfg::kPathCapacity];
    auto apply = [&](double s) {
        for (int k = 0; k < m; ++k) {
            T[k] = wps[k].duration > 0.0 ? wps[k].duration : s * T0[k];
        }
        double r = 0.0;
        for (int i = 0; i < n; ++i) {
            PathProblem& P = axes[i]->problem();
            build_path(P, T, axes[i]->solution(), axes[i]->work());
            if (P.limits.any()) {
                r = std::max(r, path_limit_ratio(P, axes[i]->solution()));
            }
        }
        return r;
    };

    double s = 1.0;
    if (any_auto) {
        bool have_limits = false;
        for (int i = 0; i < n; ++i) {
            have_limits = have_limits || axes[i]->limits().any();
        }
        if (!have_limits) {
            return Status::InvalidArg;
        }
        // path_limit_ratio() skips explicit segments via P.duration; auto ones are 0 there.
        bool ok = true;
        s = search_min_scale(apply, 1.0, 1e-3, 1e3, &ok);
        if (!ok) {
            return Status::Infeasible;
        }
    }
    apply(s);

    for (int i = 0; i < n; ++i) {
        if (!axes[i]->can_append(m, tick)) {
            return Status::NoMem;
        }
    }
    Waypoint w[cfg::kPathCapacity];
    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < m; ++k) {
            w[k] = Waypoint{};
            w[k].pos = wps[k].pos[i];
            w[k].duration = T[k];
        }
        const Status st = axes[i]->follow_path(w, m, opts, tick);
        if (st != Status::Ok) {
            return st;
        }
    }
    return Status::Ok;
}

}  // namespace nj
