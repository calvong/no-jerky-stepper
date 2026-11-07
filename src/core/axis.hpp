// Axis: the platform-independent motion state of one motor.
//
// Owns the trajectory, the step generator and the pending waypoints. All planning calls
// splice the new plan in at the *commit horizon* (the time up to which steps have already
// been generated): the trajectory is truncated there, the exact analytic state (p, v, a) at
// that time becomes the start of the new plan, so position, velocity and acceleration stay
// continuous through every retarget.
//
// Not thread-safe: the platform glue serialises access (planner mutex).
#pragma once

#include <cstdint>

#include "config.hpp"
#include "motion/cruise.hpp"
#include "motion/path_planner.hpp"
#include "motion/strategy.hpp"
#include "motion/trajectory.hpp"
#include "nj/types.hpp"
#include "stepgen/step_generator.hpp"
#include "stepgen/step_stream.hpp"

namespace nj {

struct AxisConfig {
    GenConfig gen;
    Limits limits;
    Cruise cruise;                             ///< cruise phase of point-to-point moves
    const MotionStrategy* strategy = nullptr;  ///< nullptr = min-jerk
};

class Axis {
public:
    /// A pending waypoint arriving within this time of the commit horizon is not re-planned
    /// when the path changes (keeps late appends from forcing violent corrections).
    static constexpr double kFreezeTime = 0.1;

    Axis() { configure(AxisConfig{}); }
    void configure(const AxisConfig& c);
    void set_limits(const Limits& l) { cfg_.limits = l; }
    const Limits& limits() const { return cfg_.limits; }
    void set_cruise(const Cruise& c) { cfg_.cruise = c; }
    const Cruise& cruise() const { return cfg_.cruise; }

    // ---- planning (at_tick < 0 = own commit horizon; otherwise must be >= it) ----
    /// Point-to-point move ending at rest. `cruise` overrides the configured cruise setting.
    Status move_to(double target, double duration, int64_t at_tick = -1, const Cruise* cruise = nullptr);
    /// Ramp + cruise + ramp move with a given timing (group moves share one timing).
    Status move_cruise(double target, const CruiseTiming& timing, int64_t at_tick = -1);
    Status follow_path(const Waypoint* wps, int n, const PathOptions& opts, int64_t at_tick = -1);
    Status append_waypoints(const Waypoint* wps, int n);
    Status stop(double duration = 0.0, int64_t at_tick = -1);
    /// Hard stop: forget all motion, hold at `position` (the platform has flushed the queue).
    void halt(int64_t position, bool dir_positive);
    Status set_position(int64_t position);

    // ---- generation ----
    StepGenerator::Result generate(StepStream& out, int64_t until_tick, bool keep_alive);
    /// Move the time base of an idle axis forward (group alignment).
    void rebase(int64_t tick);

    // ---- queries ----
    bool has_motion() const;
    State commanded() const { return traj_.eval(gen_.scan_time()); }
    State state_at(int64_t tick) const { return traj_.eval(gen_.to_time(tick)); }
    int64_t commit_tick() const { return gen_.scan_tick(); }
    double target() const { return traj_.end_state().p; }
    int pending_waypoints() const { return path_n_; }
    bool can_append(int segments, int64_t at_tick) const;

    const Trajectory& trajectory() const { return traj_; }
    const StepGenerator& generator() const { return gen_; }
    StepGenerator& generator() { return gen_; }

    // Solver scratch (used by the group planner).
    PathProblem& problem() { return prob_; }
    PathSolution& solution() { return sol_; }
    PathWork& work() { return work_; }

private:
    struct Pending {
        Waypoint wp;
        int32_t tag;
        double arrival;  ///< planned absolute arrival time, NaN if not planned yet
    };

    int64_t resolve_tick(int64_t at_tick) const;
    Status replan(int64_t at_tick);
    Status splice(int64_t at_tick, const PolySegment* segs, int n);
    void pop_passed(int32_t tag);

    AxisConfig cfg_;
    const MotionStrategy* strategy_ = nullptr;
    Trajectory traj_;
    StepGenerator gen_;
    Pending path_[cfg::kPathCapacity];
    Pending backup_[cfg::kPathCapacity];
    int path_n_ = 0;
    PathOptions opts_;
    int32_t next_tag_ = 0;
    PathProblem prob_;
    PathSolution sol_;
    PathWork work_;
};

/// Low time a reversed step needs: RMT drain (K filler symbols) + DIR setup + pulse + margin.
uint32_t dir_lead_ticks(uint32_t pulse_ticks, uint32_t dir_setup_ticks);

/// Generator config for a motor with the given pulse/DIR timing.
GenConfig make_gen_config(uint32_t pulse_ticks, uint32_t dir_setup_ticks);

}  // namespace nj
