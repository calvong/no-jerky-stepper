#include "core/feedback.hpp"

#include <algorithm>
#include <cmath>

#include "motion/min_jerk.hpp"

namespace nj {

void LowPassSlip::update(int64_t t_us, double slip)
{
    if (!valid_ || !(tau_s_ > 0.0)) {
        v_ = slip;
        t_us_ = t_us;
        valid_ = true;
        return;
    }
    const double dt = std::max(0.0, static_cast<double>(t_us - t_us_) * 1e-6);
    const double alpha = 1.0 - std::exp(-dt / tau_s_);
    v_ += alpha * (slip - v_);
    if (t_us > t_us_) {
        t_us_ = t_us;
    }
}

void Feedback::configure(const FeedbackConfig& c, SensorKind kind)
{
    cfg_ = c;
    kind_ = kind;
    lowpass_.set_tau(c.filter_tau_s);
    hist_head_ = 0;
    hist_n_ = 0;
    queue_n_ = 0;
    referenced_ = kind == SensorKind::Absolute;
    ref_pending_ = false;
    samples_ = 0;
    rejected_ = 0;
    faults_ = 0;
    measured_ = 0.0;
    offset_ = 0.0;
    set_mode(c.mode);
}

void Feedback::set_mode(FeedbackMode m)
{
    cfg_.mode = m;
    fault_ = FeedbackFault::None;
    phase_ = Phase::None;
    attempts_ = 0;
    est_->reset();
    have_prev_slip_ = false;
    watching_ = false;
    if (m == FeedbackMode::Off) {
        state_ = FeedbackState::Off;
    } else {
        state_ = referenced_ ? FeedbackState::Tracking : FeedbackState::Unreferenced;
    }
}

void Feedback::add_sample(int64_t t_us, double raw)
{
    if (!std::isfinite(raw)) {
        return;
    }
    if (queue_n_ == kSampleQueue) {
        // Keep the newest: drop the oldest queued sample.
        for (int i = 1; i < kSampleQueue; ++i) {
            queue_[i - 1] = queue_[i];
        }
        --queue_n_;
        ++rejected_;
    }
    queue_[queue_n_++] = Sample{t_us, raw};
}

void Feedback::request_reference()
{
    ref_pending_ = true;
}

void Feedback::reframe(double delta)
{
    if (!referenced_) {
        return;
    }
    cfg_.offset_steps += delta;
    measured_ += delta;
    for (int i = 0; i < hist_n_; ++i) {
        hist_[(hist_head_ + i) % kHistory].p += delta;
    }
}

void Feedback::shift_counter(double d)
{
    for (int i = 0; i < hist_n_; ++i) {
        hist_[(hist_head_ + i) % kHistory].p += d;
    }
    est_->shift(d);
    prev_slip_ += d;
}

void Feedback::record(int64_t t_us, double p)
{
    if (hist_n_ > 0) {
        HistEntry& last = hist_[(hist_head_ + hist_n_ - 1) % kHistory];
        if (t_us <= last.t_us) {
            last.p = p;  // same (or clamped) time: keep the newest value
            return;
        }
    }
    if (hist_n_ == kHistory) {
        hist_head_ = (hist_head_ + 1) % kHistory;
        --hist_n_;
    }
    hist_[(hist_head_ + hist_n_) % kHistory] = HistEntry{t_us, p};
    ++hist_n_;
}

bool Feedback::commanded_at(int64_t t_us, double* p) const
{
    if (hist_n_ == 0) {
        return false;
    }
    const HistEntry& first = hist_[hist_head_];
    if (t_us < first.t_us) {
        return false;  // older than the history
    }
    const HistEntry& last = hist_[(hist_head_ + hist_n_ - 1) % kHistory];
    if (t_us >= last.t_us) {
        *p = last.p;
        return true;
    }
    for (int i = hist_n_ - 2; i >= 0; --i) {
        const HistEntry& a = hist_[(hist_head_ + i) % kHistory];
        if (a.t_us <= t_us) {
            const HistEntry& b = hist_[(hist_head_ + i + 1) % kHistory];
            const double f = static_cast<double>(t_us - a.t_us) / static_cast<double>(b.t_us - a.t_us);
            *p = a.p + f * (b.p - a.p);
            return true;
        }
    }
    *p = first.p;
    return true;
}

void Feedback::raise(FeedbackFault f, FeedbackOutcome& out)
{
    if (state_ == FeedbackState::Fault) {
        return;
    }
    state_ = FeedbackState::Fault;
    fault_ = f;
    ++faults_;
    phase_ = Phase::None;
    out.stop = out.stop || cfg_.stop_on_fault;
}

void Feedback::process_sample(const Sample& s, FeedbackOutcome& out)
{
    last_sample_us_ = s.t_us;
    const int64_t t = s.t_us - static_cast<int64_t>(std::llround(cfg_.sensor_latency_ms * 1000.0));
    double cmd = 0.0;
    if (!commanded_at(t, &cmd)) {
        ++rejected_;
        return;
    }
    if (ref_pending_) {
        cfg_.offset_steps = cmd - s.raw * cfg_.steps_per_unit;
        ref_pending_ = false;
        referenced_ = true;
        est_->reset();
        have_prev_slip_ = false;
        if (state_ == FeedbackState::Fault || state_ == FeedbackState::Unreferenced) {
            fault_ = FeedbackFault::None;
            state_ = enabled() ? FeedbackState::Tracking : FeedbackState::Off;
            phase_ = Phase::None;
        }
    }
    if (!referenced_) {
        ++rejected_;
        return;
    }
    measured_ = s.raw * cfg_.steps_per_unit + cfg_.offset_steps;
    const double slip = cmd - measured_;
    if (state_ == FeedbackState::Fault) {
        return;
    }
    if (cfg_.max_jump_steps > 0.0 && have_prev_slip_ && std::fabs(slip - prev_slip_) > cfg_.max_jump_steps) {
        prev_slip_ = slip;
        raise(FeedbackFault::Jump, out);
        return;
    }
    have_prev_slip_ = true;
    prev_slip_ = slip;
    est_->update(t, slip);
    ++samples_;
    if (phase_ == Phase::Averaging && t >= phase_t0_us_) {
        avg_sum_ += slip;
        ++avg_n_;
    }
}

double Feedback::correction_time(const Axis& ax, double delta) const
{
    return std::max(cfg_.correction_time_s, rest_to_rest_duration(std::fabs(delta), ax.limits()));
}

bool Feedback::settling() const
{
    return phase_ == Phase::Pending || phase_ == Phase::Waiting || phase_ == Phase::Averaging ||
           phase_ == Phase::Correcting;
}

void Feedback::cancel_settle()
{
    if (settling()) {
        phase_ = Phase::None;
        if (state_ == FeedbackState::Settling) {
            state_ = FeedbackState::Tracking;
        }
    }
}

bool Feedback::timeout_watched(const Axis& ax) const
{
    const FeedbackMode m = cfg_.mode;
    const bool cont = m == FeedbackMode::Continuous || m == FeedbackMode::ContinuousAndSettle;
    return (cont && (m == FeedbackMode::Continuous || ax.has_planned_motion())) || settling();
}

FeedbackOutcome Feedback::service(Axis& ax, const FeedbackIo& io)
{
    FeedbackOutcome out;
    record(io.scan_play_us, ax.commanded().p);
    for (int i = 0; i < queue_n_; ++i) {
        process_sample(queue_[i], out);
    }
    queue_n_ = 0;

    const bool idle = !ax.has_motion() && !io.stream_busy;
    // Fold a finished correction into the step counter (not in the middle of a settle phase).
    if (idle && !settling() && ax.offset_target() != 0.0) {
        const int64_t sh = ax.absorb_offset();
        if (sh != 0) {
            shift_counter(-static_cast<double>(sh));
            out.absorbed = sh;
        }
    }
    offset_ = ax.offset_target();

    if (!enabled() || state_ == FeedbackState::Fault) {
        return out;
    }
    if (!referenced_) {
        state_ = FeedbackState::Unreferenced;
        return out;
    }

    // Faults: sensor timeout and implausible slip.
    const bool watch = timeout_watched(ax);
    if (watch && !watching_) {
        watch_since_us_ = io.now_us;
    }
    watching_ = watch;
    if (watch && cfg_.sensor_timeout_ms > 0) {
        const int64_t since = std::max(last_sample_us_, watch_since_us_);
        if (io.now_us - since > static_cast<int64_t>(cfg_.sensor_timeout_ms) * 1000) {
            raise(FeedbackFault::Timeout, out);
            return out;
        }
    }
    if (est_->valid() && cfg_.max_correction_steps > 0.0 && std::fabs(est_->value()) > cfg_.max_correction_steps) {
        raise(FeedbackFault::SlipTooLarge, out);
        return out;
    }

    const FeedbackMode m = cfg_.mode;
    const bool cont = m == FeedbackMode::Continuous || m == FeedbackMode::ContinuousAndSettle;
    const bool settle = m == FeedbackMode::Settle || m == FeedbackMode::ContinuousAndSettle;
    const bool plan_moving = ax.has_planned_motion();

    if (plan_moving) {
        if (settle) {
            phase_ = Phase::Pending;
            attempts_ = 0;
        }
        state_ = FeedbackState::Tracking;
    }

    // Continuous correction: the offset follows the filtered slip.
    if (cont && est_->valid() && (plan_moving || m == FeedbackMode::Continuous)) {
        const double target = est_->value();
        if (std::fabs(target - ax.offset_target()) > cfg_.deadband_steps) {
            ax.set_offset(target, correction_time(ax, target - ax.offset_state().p));
            offset_ = ax.offset_target();
        }
    }

    // Settle phase.
    switch (phase_) {
    case Phase::None:
        break;
    case Phase::Pending:
        if (!plan_moving && idle) {
            phase_ = Phase::Waiting;
            phase_t0_us_ = io.now_us;
            state_ = FeedbackState::Settling;
        }
        break;
    case Phase::Waiting:
        state_ = FeedbackState::Settling;
        if (io.now_us - phase_t0_us_ >= static_cast<int64_t>(cfg_.settle_delay_ms) * 1000) {
            phase_ = Phase::Averaging;
            phase_t0_us_ = io.now_us;
            avg_sum_ = 0.0;
            avg_n_ = 0;
        }
        break;
    case Phase::Averaging: {
        const uint32_t need = std::max<uint32_t>(1, cfg_.settle_samples);
        if (avg_n_ >= need) {
            const double mean = avg_sum_ / static_cast<double>(avg_n_);
            if (std::fabs(mean - ax.offset_target()) <= cfg_.settle_tolerance_steps) {
                phase_ = Phase::Done;
                state_ = FeedbackState::Settled;
            } else if (attempts_ >= cfg_.settle_max_attempts) {
                raise(FeedbackFault::SettleFailed, out);
            } else {
                ++attempts_;
                ax.set_offset(mean, correction_time(ax, mean - ax.offset_state().p));
                offset_ = ax.offset_target();
                phase_ = Phase::Correcting;
            }
        }
        break;
    }
    case Phase::Correcting:
        if (idle) {
            phase_ = Phase::Waiting;
            phase_t0_us_ = io.now_us;
        }
        break;
    case Phase::Done:
        state_ = FeedbackState::Settled;
        break;
    }
    return out;
}

FeedbackStatus Feedback::status() const
{
    FeedbackStatus s;
    s.state = state_;
    s.fault = fault_;
    s.measured = measured_;
    s.slip = est_->valid() ? est_->value() : 0.0;
    s.offset = offset_;
    s.samples = samples_;
    s.rejected = rejected_;
    s.faults = faults_;
    s.settle_attempts = attempts_;
    return s;
}

}  // namespace nj
