// Feedback: closed-loop position correction from an absolute or incremental sensor.
//
// Principle
//   slip = commanded(t_sample) - measured(t_sample)
// counts the steps that were commanded but did not show up (lost steps, belt stretch,
// backlash). It does not depend on the correction already applied, because the commanded
// position includes that correction, so delayed samples never double-count. The correction
// is an *offset track* on the Axis (added to the plan by the step generator), driven towards
// the filtered slip with min-jerk segments, so corrections are C2-smooth like every retarget.
//
// The commanded position at a sample's time is looked up in a short history: each planner
// cycle records (time at which the commit horizon will be played, commanded position there).
//
// Modes
//   Continuous: offset target follows the filtered slip (with a deadband), always.
//   Settle:     once the plan has ended and the stream has drained, wait, average N samples,
//               correct the mean, repeat until within tolerance or out of attempts.
//   ContinuousAndSettle: continuous while the plan moves, then one settle phase.
// When idle and not settling, the offset is folded into the step counter (absorb), so the
// counter matches the plan again.
//
// Platform-independent (host-testable); times are microseconds of any monotonic clock.
// Not thread-safe: the platform glue serialises access (planner mutex).
#pragma once

#include <cstdint>

#include "core/axis.hpp"
#include "nj/types.hpp"

namespace nj {

/// Pluggable slip estimator. v1: first-order low-pass. Planned: a 1-D Kalman filter with the
/// commanded trajectory as its process model (better for slow, noisy rangefinders).
class SlipEstimator {
public:
    virtual ~SlipEstimator() = default;
    virtual void reset() = 0;
    virtual void update(int64_t t_us, double slip) = 0;
    virtual bool valid() const = 0;
    virtual double value() const = 0;
    /// The step counter was shifted: the slip changes by `d`.
    virtual void shift(double d) = 0;
};

class LowPassSlip final : public SlipEstimator {
public:
    void set_tau(double tau_s) { tau_s_ = tau_s; }
    void reset() override { valid_ = false; }
    void update(int64_t t_us, double slip) override;
    bool valid() const override { return valid_; }
    double value() const override { return v_; }
    void shift(double d) override { v_ += d; }

private:
    double tau_s_ = 0.05;
    double v_ = 0.0;
    int64_t t_us_ = 0;
    bool valid_ = false;
};

struct FeedbackIo {
    int64_t now_us = 0;
    int64_t scan_play_us = 0;  ///< when the commit horizon will be played (now + queued lead)
    bool stream_busy = false;  ///< steps still queued or being transmitted
};

struct FeedbackOutcome {
    bool stop = false;     ///< a fault was raised and stop_on_fault is set
    int64_t absorbed = 0;  ///< counter shift done by Axis::absorb_offset(); resync the hw counter
};

class Feedback {
public:
    static constexpr int kHistory = 64;
    static constexpr int kSampleQueue = 8;

    Feedback() { configure(FeedbackConfig{}, SensorKind::Absolute); }

    /// Reset everything. Incremental sensors start unreferenced.
    void configure(const FeedbackConfig& c, SensorKind kind);
    const FeedbackConfig& config() const { return cfg_; }
    /// Change the mode; clears a fault and any settle phase.
    void set_mode(FeedbackMode m);
    bool enabled() const { return cfg_.mode != FeedbackMode::Off; }

    /// Queue a sensor reading taken at t_us (the configured latency is subtracted later).
    void add_sample(int64_t t_us, double raw);

    /// Use the next sample to reference the sensor: measured := commanded. Clears a fault.
    void request_reference();
    bool referenced() const { return referenced_; }
    bool reference_pending() const { return ref_pending_; }
    /// The step frame was shifted by `delta` (setPosition on an absolute sensor).
    void reframe(double delta);

    /// One planner cycle: record the commanded position, process the queued samples, run the
    /// controller and apply corrections to `ax`.
    FeedbackOutcome service(Axis& ax, const FeedbackIo& io);

    /// Settle phase pending or running (the stepper counts as moving).
    bool settling() const;
    /// Abandon a settle phase (immediate stop).
    void cancel_settle();
    FeedbackStatus status() const;

    // Exposed for tests.
    bool commanded_at(int64_t t_us, double* p) const;

private:
    enum class Phase : uint8_t { None, Pending, Waiting, Averaging, Correcting, Done };

    struct HistEntry {
        int64_t t_us;
        double p;
    };
    struct Sample {
        int64_t t_us;
        double raw;
    };

    void record(int64_t t_us, double p);
    void process_sample(const Sample& s, FeedbackOutcome& out);
    void raise(FeedbackFault f, FeedbackOutcome& out);
    double correction_time(const Axis& ax, double delta) const;
    void shift_counter(double d);
    bool timeout_watched(const Axis& ax) const;

    FeedbackConfig cfg_;
    SensorKind kind_ = SensorKind::Absolute;
    LowPassSlip lowpass_;
    SlipEstimator* est_ = &lowpass_;

    HistEntry hist_[kHistory];
    int hist_head_ = 0;  ///< oldest entry
    int hist_n_ = 0;

    Sample queue_[kSampleQueue];
    int queue_n_ = 0;

    bool referenced_ = true;
    bool ref_pending_ = false;
    FeedbackState state_ = FeedbackState::Off;
    FeedbackFault fault_ = FeedbackFault::None;
    Phase phase_ = Phase::None;
    uint8_t attempts_ = 0;
    int64_t phase_t0_us_ = 0;
    double avg_sum_ = 0.0;
    uint32_t avg_n_ = 0;

    bool have_prev_slip_ = false;
    double prev_slip_ = 0.0;
    double measured_ = 0.0;
    uint32_t samples_ = 0;
    uint32_t rejected_ = 0;
    uint32_t faults_ = 0;
    int64_t last_sample_us_ = 0;
    bool watching_ = false;
    int64_t watch_since_us_ = 0;
    double offset_ = 0.0;
};

}  // namespace nj
