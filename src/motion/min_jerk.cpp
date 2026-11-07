#include "motion/min_jerk.hpp"

#include <algorithm>
#include <cmath>

namespace nj {

PolySegment min_jerk(const State& s0, const State& s1, double T)
{
    // Coefficients from the sympy solution in python/mjt_calculations.py
    // (previously src/motion/mjt.c::compute_mjt_coeff), now in double with real-valued T.
    PolySegment s;
    s.T = T;
    const double x0 = s0.p, v0 = s0.v, a0 = s0.a;
    const double xT = s1.p, vT = s1.v, aT = s1.a;
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    const double dx = xT - x0;
    s.c[0] = x0;
    s.c[1] = v0;
    s.c[2] = 0.5 * a0;
    s.c[3] = (-3.0 * T2 * a0 + T2 * aT - 12.0 * T * v0 - 8.0 * T * vT + 20.0 * dx) / (2.0 * T3);
    s.c[4] = (3.0 * T2 * a0 - 2.0 * T2 * aT + 16.0 * T * v0 + 14.0 * T * vT - 30.0 * dx) / (2.0 * T4);
    s.c[5] = (-T2 * a0 + T2 * aT - 6.0 * T * v0 - 6.0 * T * vT + 12.0 * dx) / (2.0 * T5);
    return s;
}

PolySegment min_jerk_free_end(const State& s0, double v1, double T)
{
    // v(t) = v0 + a0 t + k2 t^2 + k3 t^3 with v(T) = v1, a(T) = 0.
    const double dv = s0.v - v1;
    const double k3 = (2.0 * dv + s0.a * T) / (T * T * T);
    const double k2 = -(3.0 * dv + 2.0 * s0.a * T) / (T * T);
    PolySegment s;
    s.T = T;
    s.c[0] = s0.p;
    s.c[1] = s0.v;
    s.c[2] = 0.5 * s0.a;
    s.c[3] = k2 / 3.0;
    s.c[4] = k3 / 4.0;
    s.c[5] = 0.0;
    return s;
}

double rest_to_rest_duration(double distance, const Limits& lim)
{
    const double D = std::fabs(distance);
    if (D == 0.0) {
        return 0.0;
    }
    double T = 0.0;
    if (lim.v_max > 0.0) {
        T = std::max(T, 1.875 * D / lim.v_max);
    }
    if (lim.a_max > 0.0) {
        T = std::max(T, std::sqrt(5.773502691896258 * D / lim.a_max));
    }
    if (lim.j_max > 0.0) {
        T = std::max(T, std::cbrt(60.0 * D / lim.j_max));
    }
    return T;
}

namespace {

bool at_rest(const State& s)
{
    return s.v == 0.0 && s.a == 0.0;
}

}  // namespace

double min_jerk_duration(const State& s0, const State& s1, const Limits& lim, bool* feasible)
{
    if (feasible) {
        *feasible = true;
    }
    if (!lim.any()) {
        return 0.0;
    }
    if (at_rest(s0) && at_rest(s1)) {
        return std::max(rest_to_rest_duration(s1.p - s0.p, lim), kMinSegmentDuration);
    }
    const Limits eff = limits_for_start(lim, s0);
    const double guess = std::max(rest_to_rest_duration(s1.p - s0.p, lim), 0.05);
    auto ratio = [&](double T) { return limit_ratio(peaks(min_jerk(s0, s1, T)), eff); };
    return search_min_scale(ratio, guess, kMinSegmentDuration, kMaxSegmentDuration, feasible);
}

double stop_duration(const State& s0, const Limits& lim, bool* feasible)
{
    if (feasible) {
        *feasible = true;
    }
    if (!lim.any()) {
        return 0.0;
    }
    const Limits eff = limits_for_start(lim, s0);
    double guess = 0.05;
    if (lim.a_max > 0.0) {
        guess = std::max(guess, 1.5 * std::fabs(s0.v) / lim.a_max);
    }
    auto ratio = [&](double T) { return limit_ratio(peaks(min_jerk_free_end(s0, 0.0, T)), eff); };
    return search_min_scale(ratio, guess, kMinSegmentDuration, kMaxSegmentDuration, feasible);
}

}  // namespace nj
