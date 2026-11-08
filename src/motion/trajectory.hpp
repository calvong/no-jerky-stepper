// Trajectory: a fixed-capacity queue of PolySegments on an absolute time axis.
//
// Time is "trajectory time" in seconds. When no segment covers a time, the trajectory holds
// its end position. No heap allocation; capacity is NJ_SEGMENT_POOL.
#pragma once

#include <cstdint>

#include "config.hpp"
#include "motion/poly.hpp"

namespace nj {

class Trajectory {
public:
    static constexpr int kCapacity = cfg::kSegmentPool;

    Trajectory() { reset(0.0, 0.0); }

    /// Drop everything and hold position p from time t.
    void reset(double t, double p);

    /// Append a segment starting at t_end(). `tag` is reported back by discard_before().
    bool append(const PolySegment& seg, int32_t tag = -1);

    State eval(double t) const;
    double p(double t) const { return eval(t).p; }

    double t_end() const { return t_end_; }
    State end_state() const { return end_; }
    bool holding_at(double t) const { return t >= t_end_; }
    int size() const { return n_; }
    int free_slots() const { return kCapacity - n_; }

    /// Number of segments that would survive truncate(t).
    int count_before(double t) const;

    /// Cut the motion at time t: the segment containing t ends at t, later ones are dropped,
    /// and the end state becomes the analytic state at t. If t is beyond the end, the hold is
    /// simply extended to t.
    void truncate(double t);

    /// Drop segments that end at or before t. Returns the largest tag among them (-1 if none).
    int32_t discard_before(double t);

    struct Entry {
        double t0;
        PolySegment seg;
        int32_t tag;
    };
    /// i-th live segment (0 = oldest).
    const Entry& entry(int i) const { return e_[(head_ + i) % kCapacity]; }

private:
    Entry& at(int i) { return e_[(head_ + i) % kCapacity]; }

    Entry e_[kCapacity];
    int head_ = 0;
    int n_ = 0;
    double t_end_ = 0.0;
    State end_;
};

/// Read-only sum of the planned trajectory and an optional offset track (feedback
/// corrections). This is what the step generator samples. Implicitly constructible from a
/// Trajectory, so code without an offset track is unchanged.
class TrajView {
public:
    TrajView(const Trajectory& main, const Trajectory* offset = nullptr) : m_(main), o_(offset) {}  // NOLINT

    State eval(double t) const
    {
        State s = m_.eval(t);
        if (o_) {
            const State d = o_->eval(t);
            s.p += d.p;
            s.v += d.v;
            s.a += d.a;
        }
        return s;
    }
    double p(double t) const { return o_ ? m_.p(t) + o_->p(t) : m_.p(t); }
    bool holding_at(double t) const { return m_.holding_at(t) && (!o_ || o_->holding_at(t)); }
    State end_state() const
    {
        State s = m_.end_state();
        if (o_) {
            const State d = o_->end_state();
            s.p += d.p;
            s.v += d.v;
            s.a += d.a;
        }
        return s;
    }

private:
    const Trajectory& m_;
    const Trajectory* o_;
};

}  // namespace nj
