// Cruise phase of point-to-point moves: ramp + constant velocity + ramp.
#include <cmath>

#include "core/group_planner.hpp"
#include "motion/cruise.hpp"
#include "motion/min_jerk.hpp"
#include "motion/strategy.hpp"
#include "test_util.hpp"

using namespace nj;
using namespace njt;

namespace {

const Limits kLim{20000.0, 1e5, 5e6};

Cruise cruise(CruiseMode m, double value = 0.0, double min_distance = 0.0)
{
    Cruise c;
    c.mode = m;
    c.value = value;
    c.min_distance = min_distance;
    return c;
}

void check_joint(const PolySegment& a, const PolySegment& b)
{
    const State e = a.end();
    const State s = b.state(0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(e.p)), e.p, s.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-7 * (1 + std::fabs(e.v)), e.v, s.v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4 * (1 + std::fabs(e.a)), e.a, s.a);
}

// The cruise segment is a straight line: no acceleration, jerk, ...
void check_constant_velocity(const PolySegment& c)
{
    for (int i = 2; i < 6; ++i) {
        TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, c.c[i]);
    }
}

double total_time(const PolySegment* s, int n)
{
    double T = 0.0;
    for (int i = 0; i < n; ++i) {
        T += s[i].T;
    }
    return T;
}

}  // namespace

static void test_cruise_fraction_modes()
{
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(Cruise{}, 4000.0, kLim));
    // Distance: absolute cruise length, plain quintic when the move is not longer.
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.25, cruise_fraction(cruise(CruiseMode::Distance, 1000), 4000.0, kLim));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Distance, 1000), 1000.0, kLim));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Distance, 1000), 500.0, kLim));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Distance, 0), 500.0, kLim));
    // Percent of the move.
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.4, cruise_fraction(cruise(CruiseMode::Percent, 40), 4000.0, kLim));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Percent, 0), 4000.0, kLim));
    // min_distance gate.
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Percent, 40, 5000), 4000.0, kLim));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.4, cruise_fraction(cruise(CruiseMode::Percent, 40, 4000), 4000.0, kLim));
    // Auto: ramps just long enough to reach v_max.
    const double d_r = auto_ramp_distance(kLim);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.5 * 5.773502691896258 / (1.875 * 1.875) * 20000.0 * 20000.0 / 1e5, d_r);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, (20000.0 - 2 * d_r) / 20000.0,
                              cruise_fraction(cruise(CruiseMode::Auto), 20000.0, kLim));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Auto), 1.9 * d_r, kLim));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Auto), 20000.0, Limits{0, 1e5, 5e6}));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Auto), 20000.0, Limits{20000, 0, 0}));
    // Jerk-bound auto ramp.
    const Limits jl{20000.0, 0.0, 1e6};
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, std::sqrt(15.0 / (1.875 * 1.875 * 1.875) * 8e12 / 1e6), auto_ramp_distance(jl));
    // Validation.
    TEST_ASSERT_TRUE(cruise_valid(cruise(CruiseMode::Percent, 99.9)));
    TEST_ASSERT_FALSE(cruise_valid(cruise(CruiseMode::Percent, 100)));
    TEST_ASSERT_FALSE(cruise_valid(cruise(CruiseMode::Percent, -1)));
    TEST_ASSERT_FALSE(cruise_valid(cruise(CruiseMode::Distance, -1)));
    TEST_ASSERT_FALSE(cruise_valid(cruise(CruiseMode::Distance, NAN)));
    TEST_ASSERT_FALSE(cruise_valid(cruise(CruiseMode::Auto, 0, -5)));
    TEST_ASSERT_DOUBLE_WITHIN(0.0, 0.0, cruise_fraction(cruise(CruiseMode::Percent, 150), 4000.0, kLim));
}

static void test_cruise_distance_from_limits()
{
    PolySegment s[4];
    int n = 0;
    const State s0{0.0, 0.0, 0.0};
    TEST_ASSERT_EQUAL(Status::Ok,
                      min_jerk_strategy().plan_move(s0, 50000.0, 0.0, kLim, cruise(CruiseMode::Distance, 20000), s, 4, &n));
    TEST_ASSERT_EQUAL_INT(3, n);
    check_joint(s[0], s[1]);
    check_joint(s[1], s[2]);
    check_constant_velocity(s[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 20000.0, s[1].end().p - s[1].p(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 15000.0, s[0].end().p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 50000.0, s[2].end().p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, s[2].end().v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, s[2].end().a);
    // Within the limits, and the ramps are as fast as they allow.
    const double r0 = limit_ratio(peaks(s[0]), kLim);
    const double r2 = limit_ratio(peaks(s[2]), kLim);
    TEST_ASSERT_TRUE(r0 <= 1.0 + 1e-9 && r2 <= 1.0 + 1e-9);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, r0);
    TEST_ASSERT_TRUE(limit_ratio(peaks(s[1]), kLim) <= 1.0 + 1e-9);
    // Ramp-out mirrors ramp-in.
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, s[0].T, s[2].T);
    // v-bound: T = (1.875 * 2 d_r + d_c) / v_max = 3.8125 s, vs 1.875 D / v_max = 4.6875 s.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, (1.875 * 30000.0 + 20000.0) / kLim.v_max, total_time(s, n));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.6875, rest_to_rest_duration(50000.0, kLim));
}

static void test_cruise_percent_explicit_duration()
{
    PolySegment s[4];
    int n = 0;
    const State s0{1000.0, 0.0, 0.0};
    TEST_ASSERT_EQUAL(Status::Ok,
                      min_jerk_strategy().plan_move(s0, -30000.0, 4.0, kLim, cruise(CruiseMode::Percent, 50), s, 4, &n));
    TEST_ASSERT_EQUAL_INT(3, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0, total_time(s, n));
    check_joint(s[0], s[1]);
    check_joint(s[1], s[2]);
    check_constant_velocity(s[1]);
    TEST_ASSERT_TRUE(s[1].c[1] < 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -15500.0, s[1].end().p - s[1].p(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -30000.0, s[2].end().p);
    // Explicit durations work without limits too.
    TEST_ASSERT_EQUAL(Status::Ok, min_jerk_strategy().plan_move(s0, -30000.0, 4.0, Limits{},
                                                                cruise(CruiseMode::Percent, 50), s, 4, &n));
    TEST_ASSERT_EQUAL_INT(3, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0, total_time(s, n));
}

static void test_cruise_auto_reaches_vmax()
{
    PolySegment s[4];
    int n = 0;
    const State s0{0.0, 0.0, 0.0};
    TEST_ASSERT_EQUAL(Status::Ok,
                      min_jerk_strategy().plan_move(s0, 100000.0, 0.0, kLim, cruise(CruiseMode::Auto), s, 4, &n));
    TEST_ASSERT_EQUAL_INT(3, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, kLim.v_max, s[1].c[1]);
    TEST_ASSERT_TRUE(limit_ratio(peaks(s[0]), kLim) <= 1.0 + 1e-9);
    TEST_ASSERT_TRUE(limit_ratio(peaks(s[2]), kLim) <= 1.0 + 1e-9);
    // Both v and a are tight at the shortest ramp that reaches v_max.
    const Peaks pk = peaks(s[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6 * kLim.a_max, kLim.a_max, pk.a);
    // Short move: plain quintic.
    TEST_ASSERT_EQUAL(Status::Ok,
                      min_jerk_strategy().plan_move(s0, 5000.0, 0.0, kLim, cruise(CruiseMode::Auto), s, 4, &n));
    TEST_ASSERT_EQUAL_INT(1, n);
    // A distance cruise that does not fit: plain quintic.
    TEST_ASSERT_EQUAL(Status::Ok, min_jerk_strategy().plan_move(s0, 5000.0, 0.0, kLim,
                                                                cruise(CruiseMode::Distance, 8000), s, 4, &n));
    TEST_ASSERT_EQUAL_INT(1, n);
}

static void retarget_from_cruise(double new_target)
{
    AxisConfig ac = default_axis_config(kLim);
    ac.cruise = cruise(CruiseMode::Auto);
    Axis* ax = new Axis;
    ax->configure(ac);
    StepStream* st = new StepStream;
    TrajCopy tc;
    StepChecker chk;
    chk.tc = &tc;
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(80000, 0.0));
    TEST_ASSERT_EQUAL_INT(3, ax->trajectory().size());
    tc.capture(ax->trajectory());
    run_until(*ax, *st, chk, 1.5);
    const double t = ax->generator().scan_time();
    const State before = ax->commanded();
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, kLim.v_max, before.v);  // cruising
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(new_target, 0.0));
    tc.capture(ax->trajectory());
    const State at = ax->trajectory().eval(t);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, before.p, at.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, before.v, at.v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, before.a, at.a);
    for (int i = 0; i < ax->trajectory().size(); ++i) {
        const Trajectory::Entry& e = ax->trajectory().entry(i);
        TEST_ASSERT_TRUE(limit_ratio(peaks(e.seg), kLim) <= 1.0 + 1e-6);
    }
    run_to_end(*ax, *st, chk);
    TEST_ASSERT_EQUAL_INT64(static_cast<int64_t>(new_target), ax->generator().position());
    TEST_ASSERT_EQUAL_INT64(static_cast<int64_t>(new_target), chk.pos);
    TEST_ASSERT_TRUE(chk.max_err <= kTrackBound);
    delete ax;
    delete st;
}

static void test_cruise_retarget_forward()
{
    retarget_from_cruise(70000.0);  // still far ahead: cruises again
}

static void test_cruise_retarget_backward()
{
    retarget_from_cruise(-10000.0);  // behind: reverses through a ramp
}

static void test_cruise_per_call_override()
{
    Axis* ax = new Axis;
    ax->configure(default_axis_config(kLim));
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(60000, 0.0));
    TEST_ASSERT_EQUAL_INT(1, ax->trajectory().size());  // configured: off
    const Cruise c = cruise(CruiseMode::Percent, 30);
    ax->halt(0, true);
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(60000, 0.0, -1, &c));
    TEST_ASSERT_EQUAL_INT(3, ax->trajectory().size());
    delete ax;
}

namespace {

struct Rig {
    Axis ax[3];
    Axis* p[3];
    explicit Rig(const Limits* lims)
    {
        for (int i = 0; i < 3; ++i) {
            ax[i].configure(default_axis_config(lims[i]));
            p[i] = &ax[i];
        }
    }
};

void check_group_cruise(const Cruise& c, double duration)
{
    const Limits lims[3] = {{20000, 1e5, 5e6}, {8000, 1e5, 5e6}, {20000, 2e4, 5e6}};
    Rig* r = new Rig(lims);
    // Members' own settings must not change the shared timing.
    r->ax[1].set_cruise(cruise(CruiseMode::Distance, 100));
    const double tgt[3] = {60000.0, -25000.0, 0.0};
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(r->p, 3, tgt, duration, c));
    double T = 0.0;
    for (int k = 0; k < 3; ++k) {
        const double Tk = r->ax[0].trajectory().entry(k).seg.T;
        T += Tk;
        for (int i = 0; i < 3; ++i) {
            const Trajectory& tr = r->ax[i].trajectory();
            TEST_ASSERT_EQUAL_INT(3, tr.size());
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, Tk, tr.entry(k).seg.T);
            if (duration <= 0.0) {
                TEST_ASSERT_TRUE(limit_ratio(peaks(tr.entry(k).seg), lims[i]) <= 1.0 + 1e-9);
            }
        }
    }
    if (duration > 0.0) {
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, duration, T);
    }
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, tgt[i], r->ax[i].trajectory().end_state().p);
    }
    // Straight line in joint space; the idle member holds.
    for (int k = 1; k < 200; ++k) {
        const double t = T * k / 200.0;
        const double s0 = r->ax[0].trajectory().p(t) / tgt[0];
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, s0, r->ax[1].trajectory().p(t) / tgt[1]);
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, r->ax[2].trajectory().p(t));
    }
    delete r;
}

}  // namespace

static void test_group_cruise_percent()
{
    check_group_cruise(cruise(CruiseMode::Percent, 50), 0.0);
    check_group_cruise(cruise(CruiseMode::Percent, 50), 6.0);
}

static void test_group_cruise_distance()
{
    // 30000 of the 60000-step lead travel: f = 0.5 on every member.
    check_group_cruise(cruise(CruiseMode::Distance, 30000), 0.0);
}

static void test_group_cruise_auto_is_faster()
{
    check_group_cruise(cruise(CruiseMode::Auto), 0.0);
    const Limits lims[3] = {{20000, 1e5, 5e6}, {8000, 1e5, 5e6}, {20000, 2e4, 5e6}};
    const double tgt[3] = {60000.0, -25000.0, 0.0};
    Rig* a = new Rig(lims);
    Rig* b = new Rig(lims);
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(a->p, 3, tgt, 0.0, cruise(CruiseMode::Auto)));
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(b->p, 3, tgt, 0.0));
    TEST_ASSERT_TRUE(a->ax[0].trajectory().t_end() < 0.8 * b->ax[0].trajectory().t_end());
    delete a;
    delete b;
}

void nj_test_cruise()
{
    RUN_TEST(test_cruise_fraction_modes);
    RUN_TEST(test_cruise_distance_from_limits);
    RUN_TEST(test_cruise_percent_explicit_duration);
    RUN_TEST(test_cruise_auto_reaches_vmax);
    RUN_TEST(test_cruise_retarget_forward);
    RUN_TEST(test_cruise_retarget_backward);
    RUN_TEST(test_cruise_per_call_override);
    RUN_TEST(test_group_cruise_percent);
    RUN_TEST(test_group_cruise_distance);
    RUN_TEST(test_group_cruise_auto_is_faster);
}
