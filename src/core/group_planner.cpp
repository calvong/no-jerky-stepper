#include "core/group_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "motion/cruise.hpp"
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

namespace {

// Common rest-based ramp time of all members for cruise share f (slowest member decides).
// Returns 0 if no member has limits.
double group_ramp_duration(Axis* const* axes, int n, const double* L, double f)
{
    double T = 0.0;
    for (int i = 0; i < n; ++i) {
        if (axes[i]->limits().any()) {
            T = std::max(T, cruise_ramp_duration(L[i], f, axes[i]->limits()));
        }
    }
    return T;
}

double group_total(double f, double T_ramp)
{
    return 2.0 * T_ramp + cruise_time(f, T_ramp);
}

// Cruise share for the whole group, 0 = no cruise. Distance and Percent are measured on the
// member with the longest travel. Auto tries the share at which each member just reaches
// its own v_max and keeps the one with the shortest group duration.
double group_cruise_fraction(Axis* const* axes, int n, const double* L, const Cruise& c)
{
    double L_lead = 0.0;
    for (int i = 0; i < n; ++i) {
        L_lead = std::max(L_lead, L[i]);
    }
    if (c.mode != CruiseMode::Auto) {
        return cruise_fraction(c, L_lead, Limits{});
    }
    if (!cruise_valid(c) || !(L_lead > 0.0) || L_lead < c.min_distance) {
        return 0.0;
    }
    Cruise any_length = c;
    any_length.min_distance = 0.0;
    double best_f = 0.0;
    double best_T = INFINITY;
    for (int i = 0; i < n; ++i) {
        const double f = cruise_fraction(any_length, L[i], axes[i]->limits());
        if (f > 0.0) {
            const double T = group_total(f, group_ramp_duration(axes, n, L, f));
            if (T < best_T) {
                best_T = T;
                best_f = f;
            }
        }
    }
    // Keep the plain quintic (f = 0) if it is not slower.
    if (best_f > 0.0 && group_total(0.0, group_ramp_duration(axes, n, L, 0.0)) <= best_T) {
        return 0.0;
    }
    return best_f;
}

bool at_rest(const State& s)
{
    return s.v == 0.0 && s.a == 0.0;
}

Status group_cruise_move(Axis* const* axes, int n, const double* targets, const State* s0, const double* L,
                         double f, double duration, int64_t tick)
{
    CruiseTiming tm;
    tm.f = f;
    if (duration > 0.0) {
        tm.T_out = cruise_ramp_for_total(duration, f);
        tm.T_in = tm.T_out;
    } else {
        tm.T_out = group_ramp_duration(axes, n, L, f);
        if (!(tm.T_out > 0.0)) {
            return Status::InvalidArg;  // no limits on any axis
        }
        tm.T_in = tm.T_out;
        bool all_rest = true;
        for (int i = 0; i < n; ++i) {
            all_rest = all_rest && at_rest(s0[i]);
        }
        if (!all_rest) {
            // One ramp-in time for all members, the shortest within every member's limits
            // (best effort if a moving start makes them unreachable).
            State end[kMaxGroupAxes];
            Limits eff[kMaxGroupAxes];
            for (int i = 0; i < n; ++i) {
                end[i] = cruise_ramp_in_end(s0[i], targets[i], tm);
                eff[i] = limits_for_start(axes[i]->limits(), s0[i]);
            }
            auto ratio = [&](double T) {
                double r = 0.0;
                for (int i = 0; i < n; ++i) {
                    if (eff[i].any()) {
                        r = std::max(r, limit_ratio(peaks(min_jerk(s0[i], end[i], T)), eff[i]));
                    }
                }
                return r;
            };
            tm.T_in = search_min_scale(ratio, tm.T_out, kMinSegmentDuration, kMaxSegmentDuration, nullptr);
        }
    }
    for (int i = 0; i < n; ++i) {
        if (!axes[i]->can_append(3, tick)) {
            return Status::NoMem;
        }
    }
    for (int i = 0; i < n; ++i) {
        const Status st = axes[i]->move_cruise(targets[i], tm, tick);
        if (st != Status::Ok) {
            return st;
        }
    }
    return Status::Ok;
}

}  // namespace

Status group_move_to(Axis* const* axes, int n, const double* targets, double duration, const Cruise& cruise)
{
    if (n <= 0 || n > kMaxGroupAxes) {
        return Status::InvalidArg;
    }
    const int64_t tick = group_commit_tick(axes, n);
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(targets[i])) {
            return Status::InvalidArg;
        }
    }
    if (cruise.mode != CruiseMode::Off) {
        State s0[kMaxGroupAxes];
        double L[kMaxGroupAxes];
        for (int i = 0; i < n; ++i) {
            s0[i] = axes[i]->state_at(tick);
            L[i] = std::fabs(targets[i] - s0[i].p);
        }
        const double f = group_cruise_fraction(axes, n, L, cruise);
        if (f > 0.0) {
            return group_cruise_move(axes, n, targets, s0, L, f, duration, tick);
        }
    }
    const Cruise off{};  // members' own cruise settings must not break the shared timing
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
        const Status st = axes[i]->move_to(targets[i], T, tick, &off);
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
