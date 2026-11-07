#include "motion/cruise.hpp"

#include <algorithm>
#include <cmath>

#include "motion/min_jerk.hpp"

namespace nj {

namespace {

constexpr double kPeakV = 1.875;  // peak / average velocity of the rest-to-rest quintic
// Half profile reaching v: d >= kRampA v^2 / a and d^2 >= kRampJ v^3 / j.
constexpr double kRampA = 0.5 * 5.773502691896258 / (kPeakV * kPeakV);
constexpr double kRampJ = 15.0 / (kPeakV * kPeakV * kPeakV);
constexpr double kMinShare = 1e-9;

bool at_rest(const State& s)
{
    return s.v == 0.0 && s.a == 0.0;
}

}  // namespace

bool cruise_valid(const Cruise& c)
{
    if (!std::isfinite(c.value) || !std::isfinite(c.min_distance) || c.min_distance < 0.0) {
        return false;
    }
    switch (c.mode) {
    case CruiseMode::Off:
    case CruiseMode::Auto:
        return true;
    case CruiseMode::Distance:
        return c.value >= 0.0;
    case CruiseMode::Percent:
        return c.value >= 0.0 && c.value < 100.0;
    }
    return false;
}

double auto_ramp_distance(const Limits& lim)
{
    const double v = lim.v_max;
    if (!(v > 0.0) || !(lim.a_max > 0.0 || lim.j_max > 0.0)) {
        return 0.0;
    }
    double d = 0.0;
    if (lim.a_max > 0.0) {
        d = std::max(d, kRampA * v * v / lim.a_max);
    }
    if (lim.j_max > 0.0) {
        d = std::max(d, std::sqrt(kRampJ * v * v * v / lim.j_max));
    }
    return d;
}

double cruise_fraction(const Cruise& c, double L, const Limits& lim)
{
    if (c.mode == CruiseMode::Off || !(L > 0.0) || !std::isfinite(L) || !cruise_valid(c) || L < c.min_distance) {
        return 0.0;
    }
    double d_c = 0.0;
    switch (c.mode) {
    case CruiseMode::Off:
        return 0.0;
    case CruiseMode::Distance:
        d_c = c.value;
        break;
    case CruiseMode::Percent:
        d_c = c.value / 100.0 * L;
        break;
    case CruiseMode::Auto: {
        const double d_r = auto_ramp_distance(lim);
        if (!(d_r > 0.0)) {
            return 0.0;
        }
        d_c = L - 2.0 * d_r;
        break;
    }
    }
    if (!(d_c > 0.0) || !(d_c < L)) {
        return 0.0;
    }
    const double f = d_c / L;
    return (f > kMinShare && f < 1.0 - kMinShare) ? f : 0.0;
}

double cruise_ramp_duration(double L, double f, const Limits& lim)
{
    return std::max(0.5 * rest_to_rest_duration((1.0 - f) * std::fabs(L), lim), kMinSegmentDuration);
}

double cruise_ramp_for_total(double T, double f)
{
    return std::max(T * 0.5 * kPeakV * (1.0 - f) / (kPeakV * (1.0 - f) + f), kMinSegmentDuration);
}

double cruise_time(double f, double T_out)
{
    return 2.0 * f * T_out / (kPeakV * (1.0 - f));
}

State cruise_ramp_in_end(const State& from, double target, const CruiseTiming& tm)
{
    const double D = target - from.p;
    const double s = D >= 0.0 ? 1.0 : -1.0;
    const double d_r = 0.5 * (1.0 - tm.f) * std::fabs(D);
    return State{from.p + s * d_r, s * kPeakV * d_r / tm.T_out, 0.0};
}

CruiseTiming cruise_timing(const State& from, double target, double f, double duration, const Limits& lim)
{
    CruiseTiming tm;
    tm.f = f;
    if (duration > 0.0) {
        tm.T_out = cruise_ramp_for_total(duration, f);
        tm.T_in = tm.T_out;
        return tm;
    }
    tm.T_out = cruise_ramp_duration(target - from.p, f, lim);
    tm.T_in = tm.T_out;
    if (!at_rest(from) && lim.any()) {
        // Best effort if the moving start makes the limits unreachable.
        tm.T_in = std::max(min_jerk_duration(from, cruise_ramp_in_end(from, target, tm), lim), kMinSegmentDuration);
    }
    return tm;
}

int build_cruise_move(const State& from, double target, const CruiseTiming& tm, PolySegment* out)
{
    const State ramp_end = cruise_ramp_in_end(from, target, tm);
    out[0] = min_jerk(from, ramp_end, tm.T_in);
    PolySegment c;
    c.T = cruise_time(tm.f, tm.T_out);
    c.c[0] = ramp_end.p;
    c.c[1] = ramp_end.v;
    out[1] = c;
    // Start the ramp-out at the cruise segment's own end, so the joint is exact.
    out[2] = min_jerk(c.end(), State{target, 0.0, 0.0}, tm.T_out);
    return 3;
}

}  // namespace nj
