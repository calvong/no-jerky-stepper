// PolySegment: the single trajectory representation used by the whole library.
//
// Every motion strategy (min-jerk quintic, quartic stop, and later trapezoid / S-curve /
// time-optimal pieces) is a polynomial of degree <= 5 in local time. The step generator
// and the driver only ever see PolySegments, so new strategies never touch them.
#pragma once

#include "nj/types.hpp"

namespace nj {

struct PolySegment {
    double T = 0.0;                         ///< duration [s]
    double c[6] = {0, 0, 0, 0, 0, 0};       ///< p(t) = sum c[i] t^i, local t in [0, T]

    double p(double t) const { return c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5])))); }
    double v(double t) const { return c[1] + t * (2 * c[2] + t * (3 * c[3] + t * (4 * c[4] + t * 5 * c[5]))); }
    double a(double t) const { return 2 * c[2] + t * (6 * c[3] + t * (12 * c[4] + t * 20 * c[5])); }
    double j(double t) const { return 6 * c[3] + t * (24 * c[4] + t * 60 * c[5]); }
    double snap(double t) const { return 24 * c[4] + t * 120 * c[5]; }
    State state(double t) const { return {p(t), v(t), a(t)}; }
    State end() const { return state(T); }
};

/// Constant-position segment.
PolySegment make_hold(double p, double T);

/// Real roots of k[0] + k[1] x + ... + k[deg] x^deg inside [lo, hi], ascending.
/// deg <= 5; `out` must hold `deg` values. Returns the number of roots.
int roots_in(const double* k, int deg, double lo, double hi, double* out);

/// Maximum absolute velocity / acceleration / jerk over the segment.
struct Peaks {
    double v = 0.0;
    double a = 0.0;
    double j = 0.0;
};
Peaks peaks(const PolySegment& s);

/// max(peak / limit) over the limits that are set; 0 when no limit is set. <= 1 means feasible.
double limit_ratio(const Peaks& pk, const Limits& lim);

/// Limits widened so that a start state already beyond them does not make every plan
/// infeasible (e.g. limits lowered while moving).
Limits limits_for_start(const Limits& lim, const State& start);

}  // namespace nj
