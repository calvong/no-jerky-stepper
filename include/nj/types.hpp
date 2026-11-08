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

/// How a point-to-point move uses a constant-velocity (cruise) phase.
enum class CruiseMode : uint8_t {
    Off,       ///< one min-jerk quintic for the whole move
    Distance,  ///< cruise for a fixed distance (`Cruise::value`, steps)
    Percent,   ///< cruise for a share of the move (`Cruise::value`, 0 <= value < 100)
    Auto,      ///< ramp up to v_max and cruise the rest (needs Limits::v_max)
};

/// Cruise phase of point-to-point moves: ramp up -> constant velocity -> ramp down.
/// Each ramp is half of a rest-to-rest min-jerk profile, so p, v and a stay continuous.
/// A move cruises only if its distance is >= `min_distance` and the resolved cruise
/// distance is strictly between 0 and the move distance; otherwise it is a plain min-jerk
/// quintic. Waypoint paths ignore this setting.
struct Cruise {
    CruiseMode mode = CruiseMode::Off;
    double value = 0.0;         ///< Distance: cruise length [steps]; Percent: [0, 100); Auto: unused
    double min_distance = 0.0;  ///< moves shorter than this never cruise [steps]
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

// ---------------------------------------------------------------------------------------------
// Closed-loop position feedback (see nj/sensor.hpp and Stepper::attachSensor)

/// Absolute sensors report a position in a fixed frame (SoftPot, rangefinder, absolute
/// encoder). Incremental sensors count from an arbitrary start (quadrature encoder) and need a
/// reference (calibrateSensor() or setPosition()) before feedback becomes active.
enum class SensorKind : uint8_t { Absolute, Incremental };

/// Where the library calls PositionSensor::update().
enum class SampleContext : uint8_t {
    FeedbackTask,  ///< may block (I2C, slow ADC): runs in the low-priority nj_feedback task
    Planner,       ///< fast, non-blocking (PCNT register, cached ADC/DMA value): planner task
};

enum class FeedbackMode : uint8_t {
    Off,
    Settle,               ///< correct the residual error once a move has ended
    Continuous,           ///< correct slip continuously, during motion and at rest
    ContinuousAndSettle,  ///< continuous during motion, plus a settle phase at the end
};

/// Feedback tuning. Positions in steps, times in seconds unless the name says otherwise.
struct FeedbackConfig {
    FeedbackMode mode = FeedbackMode::Settle;
    /// measured [steps] = sensor reading [sensor units] * steps_per_unit + offset_steps
    double steps_per_unit = 1.0;
    double offset_steps = 0.0;
    uint32_t sample_period_ms = 10;  ///< FeedbackTask sensors (Planner ones: every planner service)
    double sensor_latency_ms = 0.0;  ///< age of a reading when update() returns (filter delay, ToF integration)
    double filter_tau_s = 0.05;      ///< low-pass time constant of the slip estimate
    double deadband_steps = 2.0;     ///< continuous mode: ignore changes of the slip smaller than this
    double correction_time_s = 0.15; ///< minimum duration of one correction (longer if the limits require)
    double max_correction_steps = 200.0;  ///< |slip| above this is a fault (stall, sensor glitch); 0 = off
    double max_jump_steps = 0.0;     ///< slip change between two samples above this is a fault; 0 = off
    uint32_t sensor_timeout_ms = 200;  ///< no valid sample for this long is a fault; 0 = off
    bool stop_on_fault = true;       ///< smooth stop when a fault is raised
    // Settle phase
    double settle_tolerance_steps = 1.0;
    uint32_t settle_delay_ms = 50;  ///< wait after the motion ends before measuring
    uint8_t settle_samples = 8;     ///< samples averaged per settle measurement
    uint8_t settle_max_attempts = 3;
};

enum class FeedbackState : uint8_t {
    Off,
    Unreferenced,  ///< incremental sensor without a reference: no corrections yet
    Tracking,      ///< estimating (and in continuous modes correcting) slip
    Settling,      ///< settle phase in progress
    Settled,       ///< settle phase finished within tolerance
    Fault,         ///< corrections frozen; cleared by setFeedbackMode() / calibrateSensor()
};

enum class FeedbackFault : uint8_t { None, Timeout, SlipTooLarge, Jump, SettleFailed };

struct FeedbackStatus {
    FeedbackState state = FeedbackState::Off;
    FeedbackFault fault = FeedbackFault::None;
    double measured = 0.0;  ///< last measured position [steps]
    double slip = 0.0;      ///< filtered commanded - measured [steps] (lost steps, compliance)
    double offset = 0.0;    ///< correction currently applied on top of the plan [steps]
    uint32_t samples = 0;   ///< valid samples used
    uint32_t rejected = 0;  ///< samples dropped (too old for the history, or unreferenced)
    uint32_t faults = 0;
    uint8_t settle_attempts = 0;
};

/// Runtime statistics of one axis.
struct Stats {
    uint32_t underruns = 0;        ///< step queue ran empty while moving
    uint32_t stretched_ticks = 0;  ///< extra time inserted by the encoder (DIR drain/setup)
    uint32_t rate_clamps = 0;      ///< steps delayed to respect the max step rate
    uint32_t dir_stretches = 0;    ///< reversal steps delayed to fit the DIR drain
};

}  // namespace nj
