// Minimum-jerk building blocks.
//
//  * min_jerk():          quintic meeting position/velocity/acceleration at both ends
//                         (the Euler-Lagrange solution of min integral(jerk^2), x^(6) = 0).
//  * min_jerk_free_end(): same cost with the end position left free. The natural boundary
//                         condition x^(5)(T) = 0 makes it a quartic (c5 = 0). Used for smooth
//                         stops (v1 = 0) and velocity transitions.
//  * duration helpers:    smallest T that keeps |v|, |a|, |j| within the limits.
#pragma once

#include <cmath>

#include "motion/poly.hpp"
#include "nj/types.hpp"

namespace nj {

PolySegment min_jerk(const State& s0, const State& s1, double T);

/// Quartic from s0 to velocity v1 with a(T) = 0, minimising integral(jerk^2) with free end
/// position. Distance covered = (v0 + v1) T / 2 + a0 T^2 / 12.
PolySegment min_jerk_free_end(const State& s0, double v1, double T);

/// Rest-to-rest duration: max(1.875 D / v, sqrt(5.7735 D / a), cbrt(60 D / j)).
/// Returns 0 when distance is 0 or no limit is set.
double rest_to_rest_duration(double distance, const Limits& lim);

/// Smallest duration of min_jerk(s0, s1, T) within the limits (searched when not at rest).
/// Returns 0 when no limit is set. `feasible` (optional) reports whether the limits can be met.
double min_jerk_duration(const State& s0, const State& s1, const Limits& lim, bool* feasible = nullptr);

/// Smallest duration of a smooth stop (min_jerk_free_end(s0, 0, T)) within the limits.
double stop_duration(const State& s0, const Limits& lim, bool* feasible = nullptr);

constexpr double kMinSegmentDuration = 1e-3;  ///< [s]
constexpr double kMaxSegmentDuration = 1e4;   ///< [s]

/// Smallest x in [lo_bound, hi_bound] with ratio(x) <= 1, where ratio(x) is a limit ratio
/// that mostly decreases with x (but from a moving start state the feasible set can be a
/// narrow window, so the upward scan is fine-grained). Starts below `guess`.
/// If nothing is feasible, returns the x with the smallest ratio and sets *feasible = false.
template <class RatioFn>
double search_min_scale(RatioFn&& ratio, double guess, double lo_bound, double hi_bound, bool* feasible)
{
    if (!(guess > lo_bound)) {
        guess = lo_bound * 4.0;
    }
    double x = std::fmax(guess * 0.25, lo_bound);
    double best_x = x;
    double best_r = INFINITY;
    double lo = 0.0;   // largest known infeasible
    double hi = -1.0;  // smallest known feasible
    for (int i = 0; i < 160 && x <= hi_bound; ++i) {
        const double r = ratio(x);
        if (r < best_r) {
            best_r = r;
            best_x = x;
        }
        if (r <= 1.0) {
            hi = x;
            break;
        }
        lo = x;
        x *= 1.12;
    }
    if (hi < 0.0) {
        if (feasible) {
            *feasible = false;
        }
        return best_x;
    }
    if (lo == 0.0) {
        // Feasible on the first try: walk down to find an infeasible bracket.
        while (hi / 1.12 >= lo_bound) {
            const double y = hi / 1.12;
            if (ratio(y) <= 1.0) {
                hi = y;
            } else {
                lo = y;
                break;
            }
        }
        if (lo == 0.0) {
            if (feasible) {
                *feasible = true;
            }
            return hi;
        }
    }
    for (int i = 0; i < 40 && (hi - lo) > 1e-6 * hi; ++i) {
        const double m = 0.5 * (lo + hi);
        if (ratio(m) <= 1.0) {
            hi = m;
        } else {
            lo = m;
        }
    }
    if (feasible) {
        *feasible = true;
    }
    return hi;
}

}  // namespace nj
