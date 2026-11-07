// Platform-independent public types of no-jerky-stepper.
// Units throughout the library: position [steps], time [s].
#pragma once

#include <cstdint>
#include <limits>

namespace nj {

/// Kinematic state of one axis: position [steps], velocity [steps/s], acceleration [steps/s^2].
struct State {
    double p = 0.0;
    double v = 0.0;
    double a = 0.0;
};

/// Kinematic limits used to choose durations automatically. A value of 0 means "unlimited".
/// Limits are not enforced on moves whose duration you give explicitly.
struct Limits {
    double v_max = 0.0;  ///< [steps/s]
    double a_max = 0.0;  ///< [steps/s^2]
    double j_max = 0.0;  ///< [steps/s^3]

    bool any() const { return v_max > 0.0 || a_max > 0.0 || j_max > 0.0; }
};

/// Result of a planning call in the platform-independent core.
enum class Status : uint8_t {
    Ok = 0,
    InvalidArg,  ///< bad argument (e.g. auto duration requested but no limits set)
    NoMem,       ///< segment pool or waypoint buffer full
    Infeasible,  ///< no duration satisfies the limits
    Busy,        ///< operation needs the axis to be idle
};

enum class StopMode : uint8_t {
    Smooth,     ///< decelerate with a minimum-jerk profile to v = a = 0 (position free)
    Immediate,  ///< stop emitting pulses now (position may be lost if the motor is fast)
};

/// How the velocity/acceleration at intermediate waypoints is chosen.
enum class ViaPolicy : uint8_t {
    Stop,       ///< come to rest at every waypoint
    Heuristic,  ///< v = weighted average of neighbouring slopes (0 on direction change), a = 0
    Optimal,    ///< minimise total integral of jerk^2 over the whole path (C4 quintic spline)
};

/// One waypoint of a path.
struct Waypoint {
    double pos = 0.0;       ///< target position [steps]
    double duration = 0.0;  ///< duration of the segment that arrives here [s]; 0 = from limits
    /// Optional velocity/acceleration at this waypoint. NaN = chosen by the via policy.
    /// Ignored on the last waypoint of a path, which always ends at rest.
    double vel = std::numeric_limits<double>::quiet_NaN();
    double acc = std::numeric_limits<double>::quiet_NaN();
};

struct PathOptions {
    ViaPolicy policy = ViaPolicy::Optimal;
};

inline constexpr int kMaxGroupAxes = 4;

/// One waypoint of a multi-axis path: one position per group member.
struct GroupWaypoint {
    double pos[kMaxGroupAxes] = {0.0, 0.0, 0.0, 0.0};
    double duration = 0.0;  ///< [s]; 0 = from limits (slowest axis decides)
};

/// Runtime statistics of one axis.
struct Stats {
    uint32_t underruns = 0;        ///< step queue ran empty while moving
    uint32_t stretched_ticks = 0;  ///< extra time inserted by the encoder (DIR drain/setup)
    uint32_t rate_clamps = 0;      ///< steps delayed to respect the max step rate
    uint32_t dir_stretches = 0;    ///< reversal steps delayed to fit the DIR drain
};

}  // namespace nj
