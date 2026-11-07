#include "stepgen/step_generator.hpp"

#include <algorithm>
#include <cmath>

namespace nj {

int64_t StepGenerator::to_tick(double t) const
{
    return static_cast<int64_t>(std::llround(t * cfg_.tick_hz));
}

void StepGenerator::reset(int64_t tick, int64_t pos, bool dir_positive)
{
    scan_ = tick;
    last_emit_ = tick;
    pos_ = pos;
    dir_ = dir_positive ? 1 : -1;
    idle_ = true;
}

void StepGenerator::rebase(int64_t tick)
{
    if (tick > scan_) {
        scan_ = tick;
    }
    if (last_emit_ < scan_) {
        last_emit_ = scan_;
    }
}

StepGenerator::Result StepGenerator::run(const Trajectory& traj, int64_t until_tick, StepStream& out,
                                         bool keep_alive)
{
    for (;;) {
        if (out.ring.free_slots() < kMaxCmdsPerSlice) {
            return Result::SinkFull;
        }

        if (traj.holding_at(scan_time())) {
            // Land exactly on the final position (hysteresis can leave us one step short).
            const int64_t target = std::llround(traj.end_state().p);
            if (pos_ != target) {
                const int d = target > pos_ ? 1 : -1;
                emit_step(std::max(scan_, last_emit_ + spacing(d)), d, out);
                continue;
            }
            if (!keep_alive) {
                idle_ = true;
                if (last_emit_ < scan_) {
                    last_emit_ = scan_;  // dead time is not transmitted
                }
                return Result::Idle;
            }
            idle_ = false;
            if (scan_ >= until_tick) {
                return Result::Reached;
            }
            scan_ += cfg_.slice_ticks;
        } else {
            idle_ = false;
            if (scan_ >= until_tick) {
                return Result::Reached;
            }
            const int64_t s0 = scan_;
            const int64_t s1 = s0 + cfg_.slice_ticks;
            process_slice(traj, s0, s1, out);
            scan_ = s1;
        }

        // Keep the stream fed during slow motion / holds, but always leave dir_lead ticks
        // unemitted so that a reversed step can carry its full DIR drain + setup time.
        const int64_t hold = scan_ - static_cast<int64_t>(cfg_.dir_lead_ticks);
        if (hold - last_emit_ >= static_cast<int64_t>(cfg_.slice_ticks)) {
            emit_pause_until(hold, out);
        }
    }
}

void StepGenerator::process_slice(const Trajectory& traj, int64_t s0, int64_t s1, StepStream& out)
{
    double tc = to_time(s0);
    const double t1 = to_time(s1);
    const double hyst = 0.5 + cfg_.hysteresis;

    for (int it = 0; it < 4; ++it) {
        const double p1 = traj.p(t1);
        const double rel = dir_ * (p1 - static_cast<double>(pos_));
        if (rel > 0.5) {
            // Boundaries pos + dir*(k - 0.5), k = 1..n, are crossed.
            const int64_t n = static_cast<int64_t>(std::ceil(rel + 0.5)) - 1;
            const double b1 = static_cast<double>(pos_) + dir_ * 0.5;
            const double tf = solve_crossing(traj, b1, tc, t1);
            if (n == 1) {
                emit_step(to_tick(tf), dir_, out);
            } else {
                const double bn = static_cast<double>(pos_) + dir_ * (static_cast<double>(n) - 0.5);
                const double tl = solve_crossing(traj, bn, tf, t1);
                emit_step(to_tick(tf), dir_, out);
                emit_uniform(std::max(to_tick(tl), last_emit_), n - 1, out);
            }
            return;
        }
        if (rel < -hyst) {
            const double br = static_cast<double>(pos_) - dir_ * hyst;
            const double tr = solve_crossing(traj, br, tc, t1);
            emit_step(to_tick(tr), -dir_, out);
            tc = tr;
            continue;
        }
        return;
    }
}

double StepGenerator::solve_crossing(const Trajectory& traj, double b, double ta, double tb) const
{
    double fa = traj.p(ta) - b;
    double fb = traj.p(tb) - b;
    if (fa == 0.0) {
        return ta;
    }
    if ((fa < 0.0) == (fb < 0.0)) {
        return std::fabs(fa) < std::fabs(fb) ? ta : tb;  // no bracket (numerical edge case)
    }
    const double tol = 0.01 / cfg_.tick_hz;
    double t = ta + (tb - ta) * fa / (fa - fb);
    for (int i = 0; i < 40; ++i) {
        const State s = traj.eval(t);
        const double f = s.p - b;
        if (f == 0.0) {
            return t;
        }
        if ((f < 0.0) == (fa < 0.0)) {
            ta = t;
            fa = f;
        } else {
            tb = t;
        }
        double tn = (s.v != 0.0) ? t - f / s.v : 0.5 * (ta + tb);
        if (!(tn > ta && tn < tb)) {
            tn = 0.5 * (ta + tb);  // Newton left the bracket: bisect
        }
        if (std::fabs(tn - t) < tol || (tb - ta) < tol) {
            return tn;
        }
        t = tn;
    }
    return t;
}

void StepGenerator::emit_step(int64_t t_req, int d, StepStream& out)
{
    const bool change = d != dir_;
    const int64_t earliest = last_emit_ + spacing(d);
    int64_t t = t_req;
    if (t < earliest) {
        t = earliest;
        if (change) {
            ++dir_stretches_;
        } else {
            ++rate_clamps_;
        }
    }
    int64_t gap = t - last_emit_;
    while (gap > kMaxCmdTicks) {
        const int64_t chunk = kMaxCmdTicks / 2;
        out.push(StepCmd::pause(static_cast<uint32_t>(chunk)));
        gap -= chunk;
    }
    out.push(StepCmd::step(static_cast<uint32_t>(gap), 1, d > 0));
    last_emit_ = t;
    pos_ += d;
    dir_ = d;
}

void StepGenerator::push_steps(int64_t period, int64_t count, StepStream& out)
{
    while (count > 0) {
        const int64_t c = std::min<int64_t>(count, 0xFFFF);
        out.push(StepCmd::step(static_cast<uint32_t>(period), static_cast<uint16_t>(c), dir_ > 0));
        count -= c;
    }
}

void StepGenerator::emit_uniform(int64_t to_tick, int64_t m, StepStream& out)
{
    const int64_t span = to_tick - last_emit_;
    int64_t q = span / m;
    int64_t r = span % m;
    if (q < static_cast<int64_t>(cfg_.min_period_ticks)) {
        q = cfg_.min_period_ticks;
        r = 0;
        ++rate_clamps_;
    }
    push_steps(q, m - r, out);
    push_steps(q + 1, r, out);
    last_emit_ += (m - r) * q + r * (q + 1);
    pos_ += dir_ * m;
}

void StepGenerator::emit_pause_until(int64_t tick, StepStream& out)
{
    while (tick > last_emit_) {
        const int64_t chunk = std::min<int64_t>(tick - last_emit_, kMaxCmdTicks);
        out.push(StepCmd::pause(static_cast<uint32_t>(chunk)));
        last_emit_ += chunk;
    }
}

}  // namespace nj
