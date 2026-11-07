// Multi-waypoint minimum-jerk path planning for one axis.
//
// A path is a start state plus n waypoints. Segment k is a min-jerk quintic from waypoint k-1
// (or the start state) to waypoint k. The policy decides velocity/acceleration at the
// intermediate waypoints; the last waypoint always ends at rest.
//
//  * Stop:      v = a = 0 everywhere.
//  * Heuristic: v = time-weighted average of the neighbouring slopes (0 if they differ in
//               sign), a = 0.
//  * Optimal:   minimise the total integral of jerk^2 with the waypoint times fixed. The
//               optimality conditions are continuity of jerk and snap at every intermediate
//               waypoint (C4 spline), giving a block-tridiagonal 2x2 system solved in O(n).
//
// Durations that are 0 ("auto") start from the rest-to-rest duration of each segment and are
// scaled by one common factor, searched so that every segment stays within the limits.
#pragma once

#include "config.hpp"
#include "motion/poly.hpp"
#include "nj/types.hpp"

namespace nj {

struct PathProblem {
    State start;
    int n = 0;
    double pos[cfg::kPathCapacity];
    double duration[cfg::kPathCapacity];  ///< > 0 explicit, 0 auto
    double vel[cfg::kPathCapacity];       ///< NaN = free
    double acc[cfg::kPathCapacity];       ///< NaN = free
    ViaPolicy policy = ViaPolicy::Optimal;
    Limits limits;
};

struct PathSolution {
    int n = 0;
    PolySegment seg[cfg::kPathCapacity];
    double T[cfg::kPathCapacity];
};

/// Scratch memory for the solver (kept out of the stack: it is a few KB).
struct PathWork {
    double T0[cfg::kPathCapacity];
    double v[cfg::kPathCapacity];
    double a[cfg::kPathCapacity];
    double A[cfg::kPathCapacity][4];
    double B[cfg::kPathCapacity][4];
    double C[cfg::kPathCapacity][4];
    double d[cfg::kPathCapacity][2];
};

/// Solve the path. Status::InvalidArg if an auto duration is requested without limits,
/// Status::Infeasible if no scaling meets the limits.
Status solve_path(const PathProblem& prob, PathSolution& sol, PathWork& work);

/// Build the segments for fully specified durations T[0..n-1] (no limit search).
void build_path(const PathProblem& prob, const double* T, PathSolution& sol, PathWork& work);

/// Worst limit ratio over all segments of a solution (<= 1 means within limits).
double path_limit_ratio(const PathProblem& prob, const PathSolution& sol);

/// Default (unscaled) duration of each segment: explicit, or rest-to-rest from the limits.
void base_durations(const PathProblem& prob, double* T0);

}  // namespace nj
