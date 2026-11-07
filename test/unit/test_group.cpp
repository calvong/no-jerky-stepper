// Group planning: shared commit tick and durations, straight-line rest-to-rest moves.
#include <cmath>
#include <vector>

#include "core/group_planner.hpp"
#include "motion/poly.hpp"
#include "test_util.hpp"

using namespace nj;
using namespace njt;

namespace {

struct Rig {
    Axis ax[3];
    StepStream st[3];
    Axis* p[3];
    Rig(const Limits* lims)
    {
        for (int i = 0; i < 3; ++i) {
            ax[i].configure(default_axis_config(lims[i]));
            p[i] = &ax[i];
        }
    }
};

}  // namespace

static void test_group_move_is_straight_line()
{
    const Limits lims[3] = {{20000, 1e5, 5e6}, {5000, 1e5, 5e6}, {20000, 2e4, 5e6}};
    Rig* r = new Rig(lims);
    const double tgt[3] = {1000.0, -500.0, 2500.0};
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(r->p, 3, tgt, 0.0));
    double T = -1.0;
    for (int i = 0; i < 3; ++i) {
        const Trajectory& tr = r->ax[i].trajectory();
        TEST_ASSERT_EQUAL_INT(1, tr.size());
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, tr.entry(0).t0);
        if (T < 0) {
            T = tr.entry(0).seg.T;
        }
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, T, tr.entry(0).seg.T);
        // Each axis respects its own limits.
        TEST_ASSERT_TRUE(limit_ratio(peaks(tr.entry(0).seg), lims[i]) <= 1.0 + 1e-9);
    }
    for (int k = 1; k < 100; ++k) {
        const double t = T * k / 100.0;
        const double s0 = r->ax[0].trajectory().p(t) / tgt[0];
        for (int i = 1; i < 3; ++i) {
            TEST_ASSERT_DOUBLE_WITHIN(1e-9, s0, r->ax[i].trajectory().p(t) / tgt[i]);
        }
    }
    delete r;
}

static void test_group_retarget_shares_commit_tick()
{
    const Limits lims[3] = {{20000, 1e5, 5e6}, {20000, 1e5, 5e6}, {20000, 1e5, 5e6}};
    Rig* r = new Rig(lims);
    const double tgt[3] = {4000.0, 3000.0, -2000.0};
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(r->p, 3, tgt, 2.0));
    StepChecker sink;
    // Members generated to slightly different horizons (as when one ring filled first).
    run_until(r->ax[0], r->st[0], sink, 0.30, true);
    run_until(r->ax[1], r->st[1], sink, 0.31, true);
    run_until(r->ax[2], r->st[2], sink, 0.29, true);
    const int64_t tick = group_commit_tick(r->p, 3);
    TEST_ASSERT_EQUAL_INT64(r->ax[1].commit_tick(), tick);
    State before[3];
    for (int i = 0; i < 3; ++i) {
        before[i] = r->ax[i].state_at(tick);
    }
    const double tgt2[3] = {0.0, 0.0, 0.0};
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(r->p, 3, tgt2, 1.0));
    const double t = static_cast<double>(tick) / cfg::kTickHz;
    double T = -1.0;
    for (int i = 0; i < 3; ++i) {
        const Trajectory& tr = r->ax[i].trajectory();
        const Trajectory::Entry& e = tr.entry(tr.size() - 1);
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, t, e.t0);  // all spliced at the same time
        if (T < 0) {
            T = e.seg.T;
        }
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, T, e.seg.T);
        const State s = tr.eval(t);
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, before[i].p, s.p);
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, before[i].v, s.v);
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, before[i].a, s.a);
    }
    delete r;
}

static void test_group_path_shares_durations()
{
    const Limits lims[3] = {{8000, 6e4, 2e6}, {3000, 6e4, 2e6}, {8000, 1e4, 2e6}};
    Rig* r = new Rig(lims);
    GroupWaypoint w[3];
    const double pts[3][3] = {{1000, 200, -300}, {2500, 900, 400}, {0, 0, 0}};
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < 3; ++i) {
            w[k].pos[i] = pts[k][i];
        }
    }
    PathOptions o;
    o.policy = ViaPolicy::Optimal;
    TEST_ASSERT_EQUAL(Status::Ok, group_follow_path(r->p, 3, w, 3, o));
    for (int k = 0; k < 3; ++k) {
        const double T = r->ax[0].trajectory().entry(k).seg.T;
        for (int i = 0; i < 3; ++i) {
            const Trajectory::Entry& e = r->ax[i].trajectory().entry(k);
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, T, e.seg.T);
            TEST_ASSERT_DOUBLE_WITHIN(1e-6, pts[k][i], e.seg.end().p);
            TEST_ASSERT_TRUE(limit_ratio(peaks(e.seg), lims[i]) <= 1.0 + 1e-6);
        }
    }
    // Run all members to the end together (keep-alive) and check landing.
    StepChecker sink;
    for (int i = 0; i < 3; ++i) {
        run_to_end(r->ax[i], r->st[i], sink);
        TEST_ASSERT_EQUAL_INT64(0, r->ax[i].generator().position());
    }
    delete r;
}

void nj_test_group()
{
    RUN_TEST(test_group_move_is_straight_line);
    RUN_TEST(test_group_retarget_shares_commit_tick);
    RUN_TEST(test_group_path_shares_durations);
}
