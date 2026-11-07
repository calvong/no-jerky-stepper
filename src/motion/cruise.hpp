// Cruise phase for long point-to-point moves: ramp up -> constant velocity -> ramp down.
//
// A single min-jerk quintic only touches its peak velocity (1.875x the average) for an
// instant, so a long move limited by v_max takes T = 1.875 D / v_max. Splitting it into
//
//   ramp-in  : min_jerk(start, {p0 + s d_r, s v_c, 0}, T_in)
//   cruise   : p(t) = p1 + s v_c t                     for T_c
//   ramp-out : min_jerk({p2, s v_c, 0}, {target, 0, 0}, T_out)
//
// keeps the min-jerk character at both ends and spends the middle at constant speed.
//
// Each ramp is exactly half of a rest-to-rest min-jerk profile over distance 2 d_r and time
// 2 T_r: that half ends at v = 1.875 d_r / T_r with a = 0, and the quintic through those
// boundary values is unique, so min_jerk() reproduces it. Hence (with cruise share f of the
// move length L, d_r = (1 - f) L / 2, d_c = f L):
//
//   T_r from the limits  = rest_to_rest_duration(2 d_r, lim) / 2      (peaks of the half
//                          profile: v = 1.875 d/T, a = 2.887 d/T^2, j = 15 d/T^3)
//   v_c                  = 1.875 d_r / T_r                         (<= v_max automatically)
//   T_c                  = d_c / v_c = 2 f T_r / (1.875 (1 - f))   (independent of L)
//   T_r for a total T    = T * 0.9375 (1 - f) / (1.875 (1 - f) + f)
//
// T_c only depends on f and T_r, so axes sharing f and T_r share their timing (group moves
// stay straight lines). p, v, a are continuous at the joints; jerk jumps, as at a retarget.
#pragma once

#include "motion/poly.hpp"
#include "nj/types.hpp"

namespace nj {

/// Values finite, Distance >= 0, Percent in [0, 100), min_distance >= 0.
bool cruise_valid(const Cruise& c);

/// Auto mode: the shortest ramp that reaches v_max within a_max / j_max,
/// max(0.82112 v^2 / a, sqrt(2.27556 v^3 / j)). 0 when v_max or both a_max and j_max are unset
/// (then a cruise cannot be faster than the plain quintic).
double auto_ramp_distance(const Limits& lim);

/// Cruise share f in (0, 1) of a move of length L, or 0 for "no cruise" (mode Off, L below
/// min_distance, cruise distance <= 0 or >= L, Auto without the limits it needs).
double cruise_fraction(const Cruise& c, double L, const Limits& lim);

/// Shortest ramp time (from rest) for a move of length L with cruise share f.
double cruise_ramp_duration(double L, double f, const Limits& lim);

/// Ramp time such that the whole move (ramp + cruise + ramp) takes T.
double cruise_ramp_for_total(double T, double f);

/// Cruise time for share f and ramp-out time T_out.
double cruise_time(double f, double T_out);

struct CruiseTiming {
    double f = 0.0;      ///< cruise share of the move length, (0, 1)
    double T_in = 0.0;   ///< ramp-in duration [s]
    double T_out = 0.0;  ///< ramp-out duration [s]; sets the cruise speed and time
};

/// Timing of one axis' cruise move. duration > 0: total time of a move from rest (a moving
/// start uses T_in = T_out, so the total is kept). duration <= 0: from the limits; a moving
/// start gets the shortest ramp-in within the limits (best effort if unreachable).
CruiseTiming cruise_timing(const State& from, double target, double f, double duration, const Limits& lim);

/// The ramp-in end state for a timing (used to search T_in from a moving start).
State cruise_ramp_in_end(const State& from, double target, const CruiseTiming& tm);

/// Writes the three segments of a cruise move to out[0..2]. Returns 3.
int build_cruise_move(const State& from, double target, const CruiseTiming& tm, PolySegment* out);

}  // namespace nj
