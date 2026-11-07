#include "motion/trajectory.hpp"

namespace nj {

void Trajectory::reset(double t, double p)
{
    head_ = 0;
    n_ = 0;
    t_end_ = t;
    end_ = State{p, 0.0, 0.0};
}

bool Trajectory::append(const PolySegment& seg, int32_t tag)
{
    if (n_ >= kCapacity || !(seg.T > 0.0)) {
        return false;
    }
    Entry& e = at(n_);
    e.t0 = t_end_;
    e.seg = seg;
    e.tag = tag;
    ++n_;
    t_end_ = e.t0 + seg.T;
    end_ = seg.end();
    return true;
}

State Trajectory::eval(double t) const
{
    for (int i = 0; i < n_; ++i) {
        const Entry& e = entry(i);
        if (t < e.t0 + e.seg.T) {
            const double lt = t - e.t0;
            return e.seg.state(lt < 0.0 ? 0.0 : lt);
        }
    }
    if (t == t_end_) {
        return end_;
    }
    return State{end_.p, 0.0, 0.0};
}

int Trajectory::count_before(double t) const
{
    int k = 0;
    for (int i = 0; i < n_; ++i) {
        if (entry(i).t0 < t) {
            ++k;
        }
    }
    return k;
}

void Trajectory::truncate(double t)
{
    if (t >= t_end_) {
        if (t > t_end_) {
            end_ = State{end_.p, 0.0, 0.0};
            t_end_ = t;
        }
        return;
    }
    const State s = eval(t);
    int keep = 0;
    for (int i = 0; i < n_; ++i) {
        Entry& e = at(i);
        if (e.t0 < t) {
            ++keep;
            if (t < e.t0 + e.seg.T) {
                e.seg.T = t - e.t0;
            }
        }
    }
    n_ = keep;
    t_end_ = t;
    end_ = s;
}

int32_t Trajectory::discard_before(double t)
{
    int32_t tag = -1;
    while (n_ > 0) {
        const Entry& e = entry(0);
        if (e.t0 + e.seg.T > t) {
            break;
        }
        if (e.tag > tag) {
            tag = e.tag;
        }
        head_ = (head_ + 1) % kCapacity;
        --n_;
    }
    return tag;
}

}  // namespace nj
