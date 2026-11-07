// Runtime retargeting and stops: C2 continuity at the splice, no underruns, exact landing.
#include <cmath>
#include <vector>

#include "motion/min_jerk.hpp"
#include "test_util.hpp"

using namespace nj;
using namespace njt;

static void check_splice(const Trajectory& tr, double t, const State& before)
{
    // The new plan starts exactly at the old analytic state.
    const State at = tr.eval(t);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(before.p)), before.p, at.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(before.v)), before.v, at.v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(before.a)), before.a, at.a);
    // And continues smoothly: one-sided values just after the splice match a Taylor step.
    const double e = 1e-6;
    const State after = tr.eval(t + e);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, before.p + before.v * e + 0.5 * before.a * e * e, after.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-2, before.v + before.a * e, after.v);
}

static void test_retarget_is_c2_continuous()
{
    Axis* ax = new Axis;
    ax->configure(default_axis_config());
    StepStream* st = new StepStream;
    StepChecker cmds;
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(5000, 2.0));
    run_until(*ax, *st, cmds, 0.6);
    const double tc = ax->generator().scan_time();
    const State before = ax->commanded();
    TEST_ASSERT_TRUE(std::fabs(before.v) > 100.0 && std::fabs(before.a) > 10.0);
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(-2000, 1.5));
    check_splice(ax->trajectory(), tc, before);
    run_to_end(*ax, *st, cmds);
    TEST_ASSERT_EQUAL_INT64(-2000, ax->generator().position());
    delete ax;
    delete st;
}

static void test_random_retargets_never_underrun()
{
    const Limits lim{20000.0, 2e5, 2e7};
    Axis* ax = new Axis;
    ax->configure(default_axis_config(lim));
    StepStream* st = new StepStream;
    Rng rng;
    TrajCopy tc;
    StepChecker all;
    all.tc = &tc;

    // Virtual-time playback: the consumer eats commands in real time, the producer fills to
    // playback + look-ahead every 2 ms (like the planner task), retargets every ~150 ms.
    // Times below are in the generator's tick domain. The motion never idles between the first
    // and the last retarget, so the sum of command durations equals generator time.
    bool running = false;
    int64_t play = 0;         // playback position
    int64_t avail_until = 0;  // end of the commands handed to the "encoder"
    int underruns = 0;
    double target = 0;
    const int64_t step = static_cast<int64_t>(0.002 * cfg::kTickHz);
    for (int it = 0; it < 6000; ++it) {
        if (it % 75 == 10 && it < 1500) {
            // Alternate far targets so the axis never comes to rest between retargets.
            const double sign = (it / 75) % 2 ? 1.0 : -1.0;
            target = sign * std::round(rng.uniform(10000, 30000));
            TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(target, 0.0));
            tc.capture(ax->trajectory());
        }
        const int64_t playback = ax->generator().emitted_tick() - static_cast<int64_t>(st->lead_ticks());
        ax->generate(*st, playback + cfg::kLookaheadTicks, false);
        if (!running && !st->ring.empty()) {
            running = true;  // transaction starts
            play = avail_until;
        }
        if (!running) {
            continue;
        }
        play += step;
        StepCmd c;
        while (avail_until < play && st->ring.pop(c)) {
            avail_until += c.total_ticks();
            st->consumed_ticks.fetch_add(c.total_ticks());
            all.feed(c);
        }
        if (avail_until < play) {
            if (ax->has_motion()) {
                ++underruns;
            } else {
                running = false;  // transaction ends
            }
            play = avail_until;
        }
    }
    TEST_ASSERT_EQUAL_INT(0, underruns);
    TEST_ASSERT_FALSE(ax->has_motion());
    TEST_ASSERT_EQUAL_INT64(static_cast<int64_t>(target), ax->generator().position());
    TEST_ASSERT_EQUAL_INT64(static_cast<int64_t>(target), all.pos);
    TEST_ASSERT_TRUE(all.max_err <= kTrackBound);
    delete ax;
    delete st;
}

static void test_smooth_stop()
{
    const Limits lim{10000.0, 50000.0, 2e6};
    Axis* ax = new Axis;
    ax->configure(default_axis_config(lim));
    StepStream* st = new StepStream;
    StepChecker cmds;
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(50000, 0.0));
    run_until(*ax, *st, cmds, 1.0);
    const double tc = ax->generator().scan_time();
    const State before = ax->commanded();
    TEST_ASSERT_TRUE(before.v > 1000.0);
    TEST_ASSERT_EQUAL(Status::Ok, ax->stop());
    check_splice(ax->trajectory(), tc, before);
    // The stop segment ends at rest and respects the limits.
    const Trajectory& tr = ax->trajectory();
    const Trajectory::Entry& last = tr.entry(tr.size() - 1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, last.seg.end().v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, last.seg.end().a);
    TEST_ASSERT_TRUE(limit_ratio(peaks(last.seg), limits_for_start(lim, before)) <= 1.0 + 1e-9);
    const double end = tr.end_state().p;
    run_to_end(*ax, *st, cmds);
    TEST_ASSERT_EQUAL_INT64(std::llround(end), ax->generator().position());
    TEST_ASSERT_TRUE(end < 50000.0);
    delete ax;
    delete st;
}

static void test_halt_and_set_position()
{
    Axis* ax = new Axis;
    ax->configure(default_axis_config());
    StepStream* st = new StepStream;
    StepChecker cmds;
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(1000, 1.0));
    run_until(*ax, *st, cmds, 0.3);
    TEST_ASSERT_EQUAL(Status::Busy, ax->set_position(0));
    ax->halt(123, true);
    TEST_ASSERT_FALSE(ax->has_motion());
    TEST_ASSERT_EQUAL(Status::Ok, ax->set_position(-7));
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(3, 0.2));
    run_to_end(*ax, *st, cmds);
    TEST_ASSERT_EQUAL_INT64(3, ax->generator().position());
    delete ax;
    delete st;
}

void nj_test_retarget()
{
    RUN_TEST(test_retarget_is_c2_continuous);
    RUN_TEST(test_random_retargets_never_underrun);
    RUN_TEST(test_smooth_stop);
    RUN_TEST(test_halt_and_set_position);
}
