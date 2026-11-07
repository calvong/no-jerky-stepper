#include "core/axis.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "motion/min_jerk.hpp"

namespace nj {

uint32_t dir_lead_ticks(uint32_t pulse_ticks, uint32_t dir_setup_ticks)
{
    return cfg::kRmtMemSymbols * cfg::kFillerSymbolTicks + dir_setup_ticks + pulse_ticks + 2 * cfg::kTicksPerUs;
}

GenConfig make_gen_config(uint32_t pulse_ticks, uint32_t dir_setup_ticks)
{
    GenConfig g;
    g.min_period_ticks = std::max<uint32_t>(cfg::kMinStepPeriodTicks, 2 * pulse_ticks);
    g.dir_lead_ticks = dir_lead_ticks(pulse_ticks, dir_setup_ticks);
    return g;
}

void Axis::configure(const AxisConfig& c)
{
    cfg_ = c;
    if (cfg_.gen.dir_lead_ticks == 0) {
        cfg_.gen.dir_lead_ticks = dir_lead_ticks(3 * cfg::kTicksPerUs, 5 * cfg::kTicksPerUs);
    }
    strategy_ = cfg_.strategy ? cfg_.strategy : &min_jerk_strategy();
    gen_.configure(cfg_.gen);
    gen_.reset(0, 0, true);
    traj_.reset(0.0, 0.0);
    path_n_ = 0;
}

int64_t Axis::resolve_tick(int64_t at_tick) const
{
    return std::max(at_tick, gen_.scan_tick());
}

bool Axis::can_append(int segments, int64_t at_tick) const
{
    const double t = gen_.to_time(resolve_tick(at_tick));
    return traj_.count_before(t) + segments <= Trajectory::kCapacity;
}

bool Axis::has_motion() const
{
    if (!traj_.holding_at(gen_.scan_time())) {
        return true;
    }
    return gen_.position() != std::llround(traj_.end_state().p);
}

Status Axis::splice(int64_t at_tick, const PolySegment* segs, int n)
{
    const double t = gen_.to_time(resolve_tick(at_tick));
    if (traj_.count_before(t) + n > Trajectory::kCapacity) {
        return Status::NoMem;
    }
    traj_.truncate(t);
    for (int i = 0; i < n; ++i) {
        traj_.append(segs[i], -1);
    }
    return Status::Ok;
}

Status Axis::move_to(double target, double duration, int64_t at_tick)
{
    const int64_t tick = resolve_tick(at_tick);
    const State s0 = traj_.eval(gen_.to_time(tick));
    PolySegment segs[4];
    int n = 0;
    const Status st = strategy_->plan_move(s0, target, duration, cfg_.limits, segs, 4, &n);
    if (st != Status::Ok) {
        return st;
    }
    const Status sp = splice(tick, segs, n);
    if (sp == Status::Ok) {
        path_n_ = 0;
    }
    return sp;
}

Status Axis::stop(double duration, int64_t at_tick)
{
    const int64_t tick = resolve_tick(at_tick);
    const State s0 = traj_.eval(gen_.to_time(tick));
    PolySegment segs[4];
    int n = 0;
    const Status st = strategy_->plan_stop(s0, duration, cfg_.limits, segs, 4, &n);
    if (st != Status::Ok) {
        return st;
    }
    const Status sp = splice(tick, segs, n);
    if (sp == Status::Ok) {
        path_n_ = 0;
    }
    return sp;
}

void Axis::halt(int64_t position, bool dir_positive)
{
    path_n_ = 0;
    traj_.reset(gen_.scan_time(), static_cast<double>(position));
    gen_.reset(gen_.scan_tick(), position, dir_positive);
}

Status Axis::set_position(int64_t position)
{
    if (has_motion()) {
        return Status::Busy;
    }
    halt(position, gen_.dir_positive());
    return Status::Ok;
}

void Axis::rebase(int64_t tick)
{
    gen_.rebase(tick);
    traj_.truncate(gen_.scan_time());
}

Status Axis::follow_path(const Waypoint* wps, int n, const PathOptions& opts, int64_t at_tick)
{
    if (n <= 0 || wps == nullptr) {
        return Status::InvalidArg;
    }
    if (n > cfg::kPathCapacity) {
        return Status::NoMem;
    }
    const int saved_n = path_n_;
    const PathOptions saved_opts = opts_;
    std::memcpy(backup_, path_, sizeof(Pending) * static_cast<size_t>(saved_n));

    for (int i = 0; i < n; ++i) {
        path_[i] = Pending{wps[i], next_tag_++, std::numeric_limits<double>::quiet_NaN()};
    }
    path_n_ = n;
    opts_ = opts;
    const Status st = replan(at_tick);
    if (st != Status::Ok) {
        std::memcpy(path_, backup_, sizeof(Pending) * static_cast<size_t>(saved_n));
        path_n_ = saved_n;
        opts_ = saved_opts;
    }
    return st;
}

Status Axis::append_waypoints(const Waypoint* wps, int n)
{
    if (n <= 0 || wps == nullptr) {
        return Status::InvalidArg;
    }
    // A plain move in progress becomes the first (already planned) waypoint, so the new
    // waypoints are appended after it instead of replacing it.
    const bool implicit = path_n_ == 0 && !traj_.holding_at(gen_.scan_time());
    if (path_n_ + n + (implicit ? 1 : 0) > cfg::kPathCapacity) {
        return Status::NoMem;
    }
    const int saved_n = path_n_;
    std::memcpy(backup_, path_, sizeof(Pending) * static_cast<size_t>(saved_n));

    if (implicit) {
        Waypoint w;
        w.pos = traj_.end_state().p;
        path_[0] = Pending{w, next_tag_++, traj_.t_end()};
        path_n_ = 1;
    }
    for (int i = 0; i < n; ++i) {
        path_[path_n_++] = Pending{wps[i], next_tag_++, std::numeric_limits<double>::quiet_NaN()};
    }
    const Status st = replan(-1);
    if (st != Status::Ok) {
        std::memcpy(path_, backup_, sizeof(Pending) * static_cast<size_t>(saved_n));
        path_n_ = saved_n;
    }
    return st;
}

Status Axis::replan(int64_t at_tick)
{
    const double t_c = gen_.to_time(resolve_tick(at_tick));

    // Keep a waypoint that is about to be reached exactly as planned.
    int first = 0;
    double t_start = t_c;
    if (path_n_ > 0 && std::isfinite(path_[0].arrival) && path_[0].arrival > t_c &&
        path_[0].arrival - t_c < kFreezeTime) {
        first = 1;
        t_start = path_[0].arrival;
    }
    const int m = path_n_ - first;
    if (traj_.count_before(t_start) + m > Trajectory::kCapacity) {
        return Status::NoMem;
    }
    if (m == 0) {
        traj_.truncate(t_start);
        return Status::Ok;
    }

    PathProblem& P = prob_;
    P.start = traj_.eval(t_start);
    P.n = m;
    P.policy = opts_.policy;
    P.limits = cfg_.limits;
    double prev_arrival = t_start;
    for (int k = 0; k < m; ++k) {
        const Pending& w = path_[first + k];
        P.pos[k] = w.wp.pos;
        P.vel[k] = w.wp.vel;
        P.acc[k] = w.wp.acc;
        double T = w.wp.duration > 0.0 ? w.wp.duration : 0.0;
        if (std::isfinite(w.arrival)) {
            // Already planned: keep its schedule.
            T = std::max(w.arrival - prev_arrival, kMinSegmentDuration);
            prev_arrival = w.arrival;
        }
        P.duration[k] = T;
    }

    const Status st = solve_path(P, sol_, work_);
    if (st != Status::Ok) {
        return st;
    }
    traj_.truncate(t_start);
    double t = t_start;
    for (int k = 0; k < m; ++k) {
        Pending& w = path_[first + k];
        traj_.append(sol_.seg[k], w.tag);
        t += sol_.T[k];
        w.arrival = t;
    }
    return Status::Ok;
}

void Axis::pop_passed(int32_t tag)
{
    int k = 0;
    while (k < path_n_ && path_[k].tag <= tag) {
        ++k;
    }
    if (k > 0) {
        std::memmove(path_, path_ + k, sizeof(Pending) * static_cast<size_t>(path_n_ - k));
        path_n_ -= k;
    }
}

StepGenerator::Result Axis::generate(StepStream& out, int64_t until_tick, bool keep_alive)
{
    const StepGenerator::Result r = gen_.run(traj_, until_tick, out, keep_alive);
    const int32_t tag = traj_.discard_before(gen_.scan_time());
    if (tag >= 0) {
        pop_passed(tag);
    }
    return r;
}

}  // namespace nj
