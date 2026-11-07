// Dump the step stream of a scenario as CSV, for python/plot_steps.py.
//
//   dump_steps <move|retarget|path|reverse> > steps.csv
//
// Rows:
//   ideal,t,p,v,a   planned trajectory sampled every 1 ms
//   step,t,pos,dir  every emitted step (t = pulse time)
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "core/axis.hpp"

using namespace nj;

namespace {

struct Seg {
    double t0;
    PolySegment s;
};
std::vector<Seg> g_plan;  // every segment ever planned, merged like the trajectory

void capture(const Trajectory& tr)
{
    if (tr.size() == 0) {
        return;
    }
    const double t_first = tr.entry(0).t0;
    while (!g_plan.empty() && g_plan.back().t0 >= t_first) {
        g_plan.pop_back();
    }
    if (!g_plan.empty() && g_plan.back().t0 + g_plan.back().s.T > t_first) {
        g_plan.back().s.T = t_first - g_plan.back().t0;
    }
    for (int i = 0; i < tr.size(); ++i) {
        g_plan.push_back(Seg{tr.entry(i).t0, tr.entry(i).seg});
    }
}

State plan_at(double t)
{
    for (const Seg& s : g_plan) {
        if (t < s.t0 + s.s.T) {
            return s.s.state(t < s.t0 ? 0.0 : t - s.t0);
        }
    }
    return g_plan.empty() ? State{} : State{g_plan.back().s.end().p, 0.0, 0.0};
}

int64_t g_t = 0;
int64_t g_pos = 0;

void drain(StepStream& st)
{
    StepCmd c;
    while (st.ring.pop(c)) {
        st.consumed_ticks.fetch_add(c.total_ticks());
        if (c.steps == 0) {
            g_t += c.ticks;
            continue;
        }
        const int d = c.dir_positive() ? 1 : -1;
        for (uint32_t i = 0; i < c.steps; ++i) {
            g_t += c.ticks;
            g_pos += d;
            std::printf("step,%.9f,%lld,%d\n", static_cast<double>(g_t) / cfg::kTickHz,
                        static_cast<long long>(g_pos), d);
        }
    }
}

void run_until(Axis& ax, StepStream& st, double t)
{
    const int64_t until = static_cast<int64_t>(t * cfg::kTickHz);
    while (ax.commit_tick() < until && ax.has_motion()) {
        ax.generate(st, until, false);
        drain(st);
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const char* scenario = argc > 1 ? argv[1] : "move";
    static Axis ax;
    static StepStream st;
    AxisConfig ac;
    ac.gen = make_gen_config(3 * cfg::kTicksPerUs, 5 * cfg::kTicksPerUs);
    ac.limits = Limits{20000.0, 1e5, 5e6};
    ax.configure(ac);

    if (std::strcmp(scenario, "move") == 0) {
        ax.move_to(3200, 1.0);
        capture(ax.trajectory());
    } else if (std::strcmp(scenario, "retarget") == 0) {
        ax.move_to(20000, 2.0);
        capture(ax.trajectory());
        run_until(ax, st, 0.8);
        ax.move_to(-5000, 0.0);
        capture(ax.trajectory());
    } else if (std::strcmp(scenario, "path") == 0) {
        Waypoint w[4];
        const double p[4] = {3000, 8000, 5000, 0};
        for (int k = 0; k < 4; ++k) {
            w[k].pos = p[k];
        }
        ax.follow_path(w, 4, PathOptions{ViaPolicy::Optimal});
        capture(ax.trajectory());
    } else if (std::strcmp(scenario, "reverse") == 0) {
        Waypoint w[2];
        w[0].pos = 40;
        w[0].duration = 0.5;
        w[1].pos = -40;
        w[1].duration = 0.5;
        ax.follow_path(w, 2, PathOptions{ViaPolicy::Heuristic});
        capture(ax.trajectory());
    } else {
        std::fprintf(stderr, "unknown scenario '%s' (move|retarget|path|reverse)\n", scenario);
        return 2;
    }
    run_until(ax, st, 1e4);
    drain(st);

    const double t_end = g_plan.empty() ? 0.0 : g_plan.back().t0 + g_plan.back().s.T;
    for (double t = 0.0; t <= t_end + 1e-9; t += 1e-3) {
        const State s = plan_at(t);
        std::printf("ideal,%.6f,%.6f,%.6f,%.6f\n", t, s.p, s.v, s.a);
    }
    return 0;
}
