// StepGenerator through Axis: step accuracy, rates, reversals, landing.
#include <cmath>

#include "test_util.hpp"

using namespace nj;
using namespace njt;

static Axis* g_ax = nullptr;
static StepStream* g_st = nullptr;

static void fresh(Limits lim = {})
{
    delete g_ax;
    delete g_st;
    g_ax = new Axis;
    g_ax->configure(default_axis_config(lim));
    g_st = new StepStream;
}

static void release()
{
    delete g_ax;
    delete g_st;
    g_ax = nullptr;
    g_st = nullptr;
}

static void test_rest_to_rest_steps()
{
    fresh();
    TEST_ASSERT_EQUAL(Status::Ok, g_ax->move_to(3200, 1.0));
    TrajCopy tc;
    tc.capture(g_ax->trajectory());
    StepChecker chk;
    chk.tc = &tc;
    run_to_end(*g_ax, *g_st, chk);

    TEST_ASSERT_EQUAL_INT64(3200, g_ax->generator().position());
    TEST_ASSERT_EQUAL_INT64(3200, chk.steps);
    TEST_ASSERT_EQUAL_INT64(3200, chk.pos);
    TEST_ASSERT_EQUAL_INT(0, chk.dir_changes);
    TEST_ASSERT_TRUE_MESSAGE(chk.max_err <= 0.75, "tracking error above 0.75 step");
    TEST_ASSERT_TRUE(chk.min_interval >= g_ax->generator().config().min_period_ticks);
    TEST_ASSERT_EQUAL_UINT32(0, g_ax->generator().rate_clamps());
    TEST_ASSERT_FALSE(g_ax->has_motion());
}

static void test_fast_move_is_compact_and_exact()
{
    fresh();
    TEST_ASSERT_EQUAL(Status::Ok, g_ax->move_to(-40000, 1.0));  // peak 75 kHz
    TrajCopy tc;
    tc.capture(g_ax->trajectory());
    StepChecker chk;
    chk.tc = &tc;
    run_to_end(*g_ax, *g_st, chk);
    TEST_ASSERT_EQUAL_INT64(-40000, chk.pos);
    TEST_ASSERT_EQUAL_INT64(40000, chk.steps);
    TEST_ASSERT_TRUE(chk.max_err <= 0.75);
    // At most a few commands per 1 ms slice: far fewer than one per step.
    TEST_ASSERT_TRUE(chk.cmds < 4000);
    TEST_ASSERT_EQUAL_UINT32(0, g_ax->generator().rate_clamps());
}

static void test_slow_steps_are_exact()
{
    fresh();
    TEST_ASSERT_EQUAL(Status::Ok, g_ax->move_to(10, 5.0));
    TrajCopy tc;
    tc.capture(g_ax->trajectory());
    StepChecker chk;
    chk.tc = &tc;
    run_to_end(*g_ax, *g_st, chk);
    TEST_ASSERT_EQUAL_INT64(10, chk.steps);
    // Each step lands on its half-step crossing (to well under a tick of motion).
    TEST_ASSERT_TRUE(chk.max_cross_err < 1e-3);
    // Slow motion is fed with pauses so the encoder never starves.
    TEST_ASSERT_TRUE(chk.pauses > 100);
}

static void test_reversal_has_dir_lead()
{
    fresh();
    Waypoint w[2];
    w[0].pos = 1000;
    w[0].duration = 0.6;
    w[1].pos = -500;
    w[1].duration = 0.8;
    PathOptions o;
    o.policy = ViaPolicy::Stop;
    TEST_ASSERT_EQUAL(Status::Ok, g_ax->follow_path(w, 2, o));
    TrajCopy tc;
    tc.capture(g_ax->trajectory());
    StepChecker chk;
    chk.tc = &tc;
    run_to_end(*g_ax, *g_st, chk);
    TEST_ASSERT_EQUAL_INT64(-500, chk.pos);
    TEST_ASSERT_TRUE(chk.max_err <= kTrackBound);
    TEST_ASSERT_EQUAL_INT(1, chk.dir_changes);
    TEST_ASSERT_TRUE(chk.dir_change_first_of_cmd);
    TEST_ASSERT_TRUE(chk.min_dir_change_ticks >= g_ax->generator().config().dir_lead_ticks);
    TEST_ASSERT_EQUAL_UINT32(0, g_ax->generator().dir_stretches());
}

static void test_overspeed_is_clamped_not_lost()
{
    fresh();
    TEST_ASSERT_EQUAL(Status::Ok, g_ax->move_to(100000, 0.5));  // peak 375 kHz > max rate
    StepChecker chk;
    run_to_end(*g_ax, *g_st, chk);
    TEST_ASSERT_EQUAL_INT64(100000, chk.pos);
    TEST_ASSERT_TRUE(g_ax->generator().rate_clamps() > 0);
    TEST_ASSERT_TRUE(chk.min_interval >= g_ax->generator().config().min_period_ticks);
}

static void test_keep_alive_streams_pauses()
{
    fresh();
    StepChecker chk;
    run_until(*g_ax, *g_st, chk, 0.1, true);
    TEST_ASSERT_TRUE(chk.cmds > 10);
    TEST_ASSERT_EQUAL_INT64(0, chk.steps);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(chk.cmds), static_cast<int>(chk.pauses));
    TEST_ASSERT_TRUE(chk.t > static_cast<int64_t>(0.09 * cfg::kTickHz));
    release();
}

void nj_test_stepgen()
{
    RUN_TEST(test_rest_to_rest_steps);
    RUN_TEST(test_fast_move_is_compact_and_exact);
    RUN_TEST(test_slow_steps_are_exact);
    RUN_TEST(test_reversal_has_dir_lead);
    RUN_TEST(test_overspeed_is_clamped_not_lost);
    RUN_TEST(test_keep_alive_streams_pauses);
}
