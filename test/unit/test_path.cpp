// Waypoint paths: interpolation, continuity, jerk-optimality, limits, streaming append.
#include <cmath>
#include <limits>
#include <vector>

#include "motion/min_jerk.hpp"
#include "motion/path_planner.hpp"
#include "test_util.hpp"

using namespace nj;
using namespace njt;

namespace {

PathProblem* g_prob = nullptr;
PathSolution* g_sol = nullptr;
PathWork* g_work = nullptr;

void setup_problem(ViaPolicy pol, bool auto_durations)
{
    if (!g_prob) {
        g_prob = new PathProblem;
        g_sol = new PathSolution;
        g_work = new PathWork;
    }
    PathProblem& P = *g_prob;
    P.start = State{0.0, 0.0, 0.0};
    const double pos[5] = {800.0, 2000.0, 1500.0, -300.0, 400.0};
    const double dur[5] = {0.5, 0.7, 0.4, 0.9, 0.6};
    P.n = 5;
    for (int k = 0; k < 5; ++k) {
        P.pos[k] = pos[k];
        P.duration[k] = auto_durations ? 0.0 : dur[k];
        P.vel[k] = std::numeric_limits<double>::quiet_NaN();
        P.acc[k] = std::numeric_limits<double>::quiet_NaN();
    }
    P.policy = pol;
    P.limits = Limits{8000.0, 60000.0, 2e6};
}

double seg_cost(const PolySegment& s)
{
    // integral of j(t)^2: j^2 is a quartic, 3-point Gauss-Legendre is exact.
    const double x[3] = {-0.7745966692414834, 0.0, 0.7745966692414834};
    const double w[3] = {5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0};
    double c = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double t = 0.5 * s.T * (x[i] + 1.0);
        const double j = s.j(t);
        c += w[i] * j * j;
    }
    return c * 0.5 * s.T;
}

double path_cost(const PathProblem& P, const PathSolution& S, int knot, int comp, double delta)
{
    // Rebuild with knot `knot` velocity (comp 0) or acceleration (comp 1) perturbed.
    double cost = 0.0;
    State prev = P.start;
    for (int k = 0; k < S.n; ++k) {
        State s = S.seg[k].end();
        if (k == knot) {
            (comp == 0 ? s.v : s.a) += delta;
        }
        cost += seg_cost(min_jerk(prev, s, S.T[k]));
        prev = s;
    }
    return cost;
}

}  // namespace

static void test_paths_interpolate_and_are_c2()
{
    for (ViaPolicy pol : {ViaPolicy::Stop, ViaPolicy::Heuristic, ViaPolicy::Optimal}) {
        setup_problem(pol, false);
        TEST_ASSERT_EQUAL(Status::Ok, solve_path(*g_prob, *g_sol, *g_work));
        const PathSolution& S = *g_sol;
        TEST_ASSERT_EQUAL_INT(5, S.n);
        for (int k = 0; k < S.n; ++k) {
            TEST_ASSERT_DOUBLE_WITHIN(1e-6, g_prob->pos[k], S.seg[k].end().p);
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, g_prob->duration[k], S.T[k]);
            if (k + 1 < S.n) {
                const State a = S.seg[k].end();
                const State b = S.seg[k + 1].state(0.0);
                TEST_ASSERT_DOUBLE_WITHIN(1e-6, a.p, b.p);
                TEST_ASSERT_DOUBLE_WITHIN(1e-5, a.v, b.v);
                TEST_ASSERT_DOUBLE_WITHIN(1e-3, a.a, b.a);
            }
        }
        TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, S.seg[4].end().v);
        TEST_ASSERT_DOUBLE_WITHIN(1e-3, 0.0, S.seg[4].end().a);
    }
}

static void test_optimal_is_c4_and_minimal()
{
    setup_problem(ViaPolicy::Optimal, false);
    TEST_ASSERT_EQUAL(Status::Ok, solve_path(*g_prob, *g_sol, *g_work));
    const PathSolution& S = *g_sol;
    for (int k = 0; k + 1 < S.n; ++k) {
        const double j0 = S.seg[k].j(S.T[k]);
        const double j1 = S.seg[k + 1].j(0.0);
        const double s0 = S.seg[k].snap(S.T[k]);
        const double s1 = S.seg[k + 1].snap(0.0);
        TEST_ASSERT_DOUBLE_WITHIN(1e-6 * (1 + std::fabs(j0)), j0, j1);
        TEST_ASSERT_DOUBLE_WITHIN(1e-6 * (1 + std::fabs(s0)), s0, s1);
    }
    const double c0 = path_cost(*g_prob, S, -1, 0, 0.0);
    for (int k = 0; k + 1 < S.n; ++k) {
        for (int comp = 0; comp < 2; ++comp) {
            const double h = comp == 0 ? 5.0 : 50.0;
            TEST_ASSERT_TRUE(path_cost(*g_prob, S, k, comp, h) > c0);
            TEST_ASSERT_TRUE(path_cost(*g_prob, S, k, comp, -h) > c0);
        }
    }
    // And it beats the heuristic.
    PathSolution* H = new PathSolution;
    g_prob->policy = ViaPolicy::Heuristic;
    TEST_ASSERT_EQUAL(Status::Ok, solve_path(*g_prob, *H, *g_work));
    TEST_ASSERT_TRUE(path_cost(*g_prob, *H, -1, 0, 0.0) > c0);
    delete H;
}

static void test_auto_durations_respect_limits()
{
    for (ViaPolicy pol : {ViaPolicy::Stop, ViaPolicy::Heuristic, ViaPolicy::Optimal}) {
        setup_problem(pol, true);
        TEST_ASSERT_EQUAL(Status::Ok, solve_path(*g_prob, *g_sol, *g_work));
        const double r = path_limit_ratio(*g_prob, *g_sol);
        TEST_ASSERT_TRUE(r <= 1.0 + 1e-9);
        TEST_ASSERT_TRUE(r > 0.98);  // and not needlessly slow
        for (int k = 0; k < g_sol->n; ++k) {
            TEST_ASSERT_DOUBLE_WITHIN(1e-6, g_prob->pos[k], g_sol->seg[k].end().p);
        }
    }
}

static void test_explicit_via_velocity()
{
    setup_problem(ViaPolicy::Optimal, false);
    g_prob->vel[1] = 0.0;
    g_prob->acc[1] = 0.0;
    TEST_ASSERT_EQUAL(Status::Ok, solve_path(*g_prob, *g_sol, *g_work));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, g_sol->seg[1].end().v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, g_sol->seg[2].state(0.0).v);
}

static void test_axis_path_append_equivalence()
{
    Waypoint w[4];
    const double pos[4] = {500, 1500, 900, 2000};
    for (int k = 0; k < 4; ++k) {
        w[k].pos = pos[k];
        w[k].duration = 0.5;
    }
    PathOptions o;
    o.policy = ViaPolicy::Optimal;
    Axis* a = new Axis;
    Axis* b = new Axis;
    a->configure(default_axis_config());
    b->configure(default_axis_config());
    TEST_ASSERT_EQUAL(Status::Ok, a->follow_path(w, 4, o));
    TEST_ASSERT_EQUAL(Status::Ok, b->follow_path(w, 2, o));
    TEST_ASSERT_EQUAL(Status::Ok, b->append_waypoints(w + 2, 2));
    TEST_ASSERT_EQUAL_INT(a->trajectory().size(), b->trajectory().size());
    for (int i = 0; i < a->trajectory().size(); ++i) {
        const auto& ea = a->trajectory().entry(i);
        const auto& eb = b->trajectory().entry(i);
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, ea.t0, eb.t0);
        for (int c = 0; c < 6; ++c) {
            TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(ea.seg.c[c])), ea.seg.c[c], eb.seg.c[c]);
        }
    }
    delete a;
    delete b;
}

static void test_axis_append_while_moving()
{
    const Limits lim{8000.0, 60000.0, 2e6};
    Axis* ax = new Axis;
    ax->configure(default_axis_config(lim));
    StepStream* st = new StepStream;
    StepChecker cmds;
    Waypoint w[3];
    w[0].pos = 1000;
    w[1].pos = 3000;
    w[2].pos = -1000;
    PathOptions o;
    o.policy = ViaPolicy::Optimal;
    TEST_ASSERT_EQUAL(Status::Ok, ax->follow_path(w, 2, o));
    TrajCopy tc;
    tc.capture(ax->trajectory());
    cmds.tc = &tc;
    TEST_ASSERT_EQUAL_INT(2, ax->pending_waypoints());
    run_until(*ax, *st, cmds, 0.15);
    const double t = ax->generator().scan_time();
    const State before = ax->commanded();
    TEST_ASSERT_EQUAL(Status::Ok, ax->append_waypoints(w + 2, 1));
    tc.capture(ax->trajectory());
    const State at = ax->trajectory().eval(t);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(before.p)), before.p, at.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(before.v)), before.v, at.v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(before.a)), before.a, at.a);
    // Passes through all waypoints; via 3000 is no longer a stop.
    const Trajectory& tr = ax->trajectory();
    bool saw3000 = false;
    for (int i = 0; i < tr.size(); ++i) {
        const State e = tr.entry(i).seg.end();
        if (std::fabs(e.p - 3000.0) < 1e-6) {
            saw3000 = true;
            TEST_ASSERT_TRUE(std::fabs(e.v) > 1.0 || std::fabs(e.a) > 1.0);
        }
    }
    TEST_ASSERT_TRUE(saw3000);
    run_to_end(*ax, *st, cmds);
    TEST_ASSERT_EQUAL_INT64(-1000, ax->generator().position());
    TEST_ASSERT_EQUAL_INT(0, ax->pending_waypoints());
    TEST_ASSERT_EQUAL_INT64(-1000, cmds.pos);
    TEST_ASSERT_TRUE(cmds.max_err <= kTrackBound);
    delete ax;
    delete st;
}

void nj_test_path()
{
    RUN_TEST(test_paths_interpolate_and_are_c2);
    RUN_TEST(test_optimal_is_c4_and_minimal);
    RUN_TEST(test_auto_durations_respect_limits);
    RUN_TEST(test_explicit_via_velocity);
    RUN_TEST(test_axis_path_append_equivalence);
    RUN_TEST(test_axis_append_while_moving);
}
