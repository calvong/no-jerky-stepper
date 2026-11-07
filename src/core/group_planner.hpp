// Coordinated planning for several axes.
//
// All members are re-planned at one common commit tick (the latest commit horizon among
// them) with one common set of segment durations. For rest-to-rest moves every axis then
// follows the same normalised min-jerk profile s(tau), so the motion is a straight line in
// joint space. Durations from limits are decided by the slowest axis.
#pragma once

#include <cstdint>

#include "core/axis.hpp"
#include "nj/types.hpp"

namespace nj {

int64_t group_commit_tick(Axis* const* axes, int n);

/// Bring idle axes to a common time base (the latest commit tick). Call before starting a
/// group from standstill.
void group_align(Axis* const* axes, int n);

Status group_move_to(Axis* const* axes, int n, const double* targets, double duration);
Status group_follow_path(Axis* const* axes, int n, const GroupWaypoint* wps, int m, const PathOptions& opts);
Status group_stop(Axis* const* axes, int n, double duration);

}  // namespace nj
