// StepGenerator: samples a Trajectory and produces StepCmds.
//
// The trajectory is scanned in slices of NJ_SLICE_US (1 ms). Steps happen where the position
// crosses the half-step boundaries pos +/- 0.5.
//  * >= 2 steps in a slice: the exact times of the first and last crossing are solved and
//    the steps in between are spaced uniformly (integer ticks, remainder spread so the total
//    is exact). Within 1 ms the velocity is effectively constant, so this tracks the curve
//    to a small fraction of a step without per-step root finding.
//  * 1 step in a slice (low speed): the exact crossing time is used.
//  * Reversal: only stepped once the position has moved back by 0.5 + hysteresis, which
//    keeps the gap around a turnaround long enough for the DIR drain (see StepEncoder).
//  * End of motion: lands exactly on round(final position).
// All times are int64 ticks of the RMT clock, accumulated in absolute time (no drift).
#pragma once

#include <cstdint>

#include "config.hpp"
#include "motion/trajectory.hpp"
#include "stepgen/step_stream.hpp"

namespace nj {

struct GenConfig {
    double tick_hz = cfg::kTickHz;
    uint32_t slice_ticks = cfg::kSliceTicks;
    uint32_t min_period_ticks = cfg::kMinStepPeriodTicks;  ///< >= 2 * pulse
    uint32_t dir_lead_ticks = 0;  ///< minimum low time before a reversed step (drain + setup + pulse)
    double hysteresis = cfg::kReversalHysteresis;
};

class StepGenerator {
public:
    enum class Result : uint8_t {
        Reached,   ///< scanned up to the requested tick
        SinkFull,  ///< step queue full
        Idle,      ///< motion finished, nothing more to emit (and no keep-alive)
    };

    static constexpr uint32_t kMaxCmdsPerSlice = 16;
    static constexpr int64_t kMaxCmdTicks = int64_t(1) << 30;

    void configure(const GenConfig& c) { cfg_ = c; }
    const GenConfig& config() const { return cfg_; }

    void reset(int64_t tick, int64_t pos, bool dir_positive);

    /// Generate commands until the scan reaches `until_tick`, the stream is full, or the motion
    /// is finished. With `keep_alive`, a finished axis keeps emitting pauses (group members).
    Result run(const TrajView& traj, int64_t until_tick, StepStream& out, bool keep_alive);

    /// Move the time base of an idle generator forward (group alignment).
    void rebase(int64_t tick);

    int64_t scan_tick() const { return scan_; }        ///< commit horizon
    double scan_time() const { return to_time(scan_); }
    int64_t emitted_tick() const { return last_emit_; }  ///< end of the last pushed command
    int64_t position() const { return pos_; }
    bool dir_positive() const { return dir_ > 0; }
    bool idle() const { return idle_; }

    uint32_t rate_clamps() const { return rate_clamps_; }
    uint32_t dir_stretches() const { return dir_stretches_; }

    double to_time(int64_t tick) const { return static_cast<double>(tick) / cfg_.tick_hz; }
    int64_t to_tick(double t) const;

private:
    void process_slice(const TrajView& traj, int64_t s0, int64_t s1, StepStream& out);
    double solve_crossing(const TrajView& traj, double b, double ta, double tb) const;
    int64_t spacing(int d) const { return d != dir_ ? cfg_.dir_lead_ticks : cfg_.min_period_ticks; }
    void emit_step(int64_t t_req, int d, StepStream& out);
    void emit_uniform(int64_t to_tick, int64_t m, StepStream& out);
    void push_steps(int64_t period, int64_t count, StepStream& out);
    void emit_pause_until(int64_t tick, StepStream& out);

    GenConfig cfg_;
    int64_t scan_ = 0;
    int64_t last_emit_ = 0;
    int64_t pos_ = 0;
    int dir_ = 1;
    bool idle_ = true;
    uint32_t rate_clamps_ = 0;
    uint32_t dir_stretches_ = 0;
};

}  // namespace nj
