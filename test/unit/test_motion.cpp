// Polynomial helpers, min-jerk segments, durations, trajectory container.
#include <cmath>

#include "motion/min_jerk.hpp"
#include "motion/poly.hpp"
#include "motion/strategy.hpp"
#include "motion/trajectory.hpp"
#include "test_util.hpp"

using namespace nj;
using namespace njt;

static void test_roots_cubic()
{
    // (x - 0.2)(x - 0.5)(x - 0.9) = x^3 - 1.6x^2 + 0.73x - 0.09
    const double k[4] = {-0.09, 0.73, -1.6, 1.0};
    double r[3];
    const int n = roots_in(k, 3, 0.0, 1.0, r);
    TEST_ASSERT_EQUAL_INT(3, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.2, r[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.5, r[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.9, r[2]);
    TEST_ASSERT_EQUAL_INT(1, roots_in(k, 3, 0.3, 0.6, r));
    TEST_ASSERT_EQUAL_INT(0, roots_in(k, 3, 0.95, 2.0, r));
}

static void test_min_jerk_boundary_conditions()
{
    Rng rng;
    for (int i = 0; i < 200; ++i) {
        const State s0{rng.uniform(-1e5, 1e5), rng.uniform(-2e4, 2e4), rng.uniform(-1e5, 1e5)};
        const State s1{rng.uniform(-1e5, 1e5), rng.uniform(-2e4, 2e4), rng.uniform(-1e5, 1e5)};
        const double T = rng.uniform(0.01, 10.0);
        const PolySegment s = min_jerk(s0, s1, T);
        const State a = s.state(0.0);
        const State b = s.end();
        TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(s0.p)), s0.p, a.p);
        TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(s0.v)), s0.v, a.v);
        TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + std::fabs(s0.a)), s0.a, a.a);
        // End values involve large cancellations for short T: scale tolerance by magnitudes.
        const double sc = std::fabs(s0.p) + std::fabs(s1.p) + (std::fabs(s0.v) + std::fabs(s1.v)) * T +
                          (std::fabs(s0.a) + std::fabs(s1.a)) * T * T;
        TEST_ASSERT_DOUBLE_WITHIN(1e-9 * (1 + sc), s1.p, b.p);
        TEST_ASSERT_DOUBLE_WITHIN(1e-8 * (1 + sc / T), s1.v, b.v);
        TEST_ASSERT_DOUBLE_WITHIN(1e-7 * (1 + sc / (T * T)), s1.a, b.a);
    }
}

static void test_rest_to_rest_duration_is_tight()
{
    const Limits lim{20000.0, 1e5, 5e6};
    for (double D : {1.0, 100.0, 3200.0, 1e5, 1e6}) {
        const double T = rest_to_rest_duration(D, lim);
        const PolySegment s = min_jerk(State{0, 0, 0}, State{D, 0, 0}, T);
        const double r = limit_ratio(peaks(s), lim);
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, r);  // exactly at the binding limit
    }
}

static void test_peaks_match_dense_sampling()
{
    Rng rng;
    for (int i = 0; i < 50; ++i) {
        const State s0{0.0, rng.uniform(-500, 500), rng.uniform(-3000, 3000)};
        const State s1{rng.uniform(-2000, 2000), rng.uniform(-500, 500), rng.uniform(-3000, 3000)};
        const double T = rng.uniform(0.2, 3.0);
        const PolySegment s = min_jerk(s0, s1, T);
        const Peaks pk = peaks(s);
        double v = 0, a = 0, j = 0;
        const int N = 20000;
        for (int k = 0; k <= N; ++k) {
            const double t = T * k / N;
            v = std::fmax(v, std::fabs(s.v(t)));
            a = std::fmax(a, std::fabs(s.a(t)));
            j = std::fmax(j, std::fabs(s.j(t)));
        }
        TEST_ASSERT_TRUE(pk.v >= v * (1 - 1e-12));
        TEST_ASSERT_TRUE(pk.a >= a * (1 - 1e-12));
        TEST_ASSERT_TRUE(pk.j >= j * (1 - 1e-12));
        TEST_ASSERT_DOUBLE_WITHIN(1e-4 * pk.v + 1e-9, v, pk.v);
        TEST_ASSERT_DOUBLE_WITHIN(1e-4 * pk.a + 1e-9, a, pk.a);
        TEST_ASSERT_DOUBLE_WITHIN(1e-4 * pk.j + 1e-9, j, pk.j);
    }
}

static void test_min_jerk_duration_from_moving_state()
{
    const Limits lim{10000.0, 50000.0, 2e6};
    Rng rng;
    for (int i = 0; i < 30; ++i) {
        // Start states from which the limits are reachable.
        const State s0{0.0, rng.uniform(-6000, 6000), rng.uniform(-10000, 10000)};
        const State s1{rng.uniform(-20000, 20000), 0.0, 0.0};
        bool ok = false;
        const double T = min_jerk_duration(s0, s1, lim, &ok);
        TEST_ASSERT_TRUE(ok);
        const Limits eff = limits_for_start(lim, s0);
        TEST_ASSERT_TRUE(limit_ratio(peaks(min_jerk(s0, s1, T)), eff) <= 1.0 + 1e-9);
        // Minimal (up to the search tolerance): slightly faster violates a limit.
        if (T > 2 * kMinSegmentDuration) {
            TEST_ASSERT_TRUE(limit_ratio(peaks(min_jerk(s0, s1, T * 0.99)), eff) > 1.0);
        }
    }
}

static void test_unreachable_limits_are_best_effort()
{
    // Nearly at v_max and still accelerating hard with a low jerk limit: overshooting v_max
    // cannot be avoided. moveTo must still produce a plan rather than refuse.
    const Limits lim{10000.0, 50000.0, 2e5};
    const State s0{0.0, 9900.0, 49000.0};
    bool ok = true;
    const double T = min_jerk_duration(s0, State{-5000.0, 0.0, 0.0}, lim, &ok);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_TRUE(T >= kMinSegmentDuration && T <= kMaxSegmentDuration);
    PolySegment seg[2];
    int n = 0;
    TEST_ASSERT_EQUAL(Status::Ok, min_jerk_strategy().plan_move(s0, -5000.0, 0.0, lim, Cruise{}, seg, 2, &n));
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -5000.0, seg[0].end().p);
}

static void test_free_end_stop()
{
    const State s0{100.0, 5000.0, -20000.0};
    const double T = 0.8;
    const PolySegment s = min_jerk_free_end(s0, 0.0, T);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, s0.p, s.p(0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, s0.v, s.v(0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, s0.a, s.a(0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 0.0, s.v(T));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, s.a(T));
    const double dist = s0.v * T / 2 + s0.a * T * T / 12;
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, s0.p + dist, s.p(T));

    const Limits lim{10000.0, 40000.0, 1e6};
    bool ok = false;
    const double Ts = stop_duration(s0, lim, &ok);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(limit_ratio(peaks(min_jerk_free_end(s0, 0.0, Ts)), lim) <= 1.0 + 1e-9);
}

static void test_trajectory_truncate_and_discard()
{
    Trajectory tr;
    tr.reset(1.0, 0.0);
    TEST_ASSERT_TRUE(tr.append(min_jerk(State{0, 0, 0}, State{100, 0, 0}, 1.0), 7));
    TEST_ASSERT_TRUE(tr.append(min_jerk(State{100, 0, 0}, State{-50, 0, 0}, 2.0), 8));
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 4.0, tr.t_end());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 50.0, tr.p(1.5));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -50.0, tr.p(10.0));

    const State mid = tr.eval(2.7);
    tr.truncate(2.7);
    TEST_ASSERT_EQUAL_INT(2, tr.size());
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 2.7, tr.t_end());
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, mid.p, tr.end_state().p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, mid.v, tr.end_state().v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, mid.a, tr.end_state().a);
    TEST_ASSERT_EQUAL_INT(2, tr.count_before(2.7));
    TEST_ASSERT_EQUAL_INT(1, tr.count_before(2.0));

    TEST_ASSERT_EQUAL_INT(-1, tr.discard_before(1.9));
    TEST_ASSERT_EQUAL_INT(7, tr.discard_before(2.0));
    TEST_ASSERT_EQUAL_INT(1, tr.size());
}

void nj_test_motion()
{
    RUN_TEST(test_roots_cubic);
    RUN_TEST(test_min_jerk_boundary_conditions);
    RUN_TEST(test_rest_to_rest_duration_is_tight);
    RUN_TEST(test_peaks_match_dense_sampling);
    RUN_TEST(test_min_jerk_duration_from_moving_state);
    RUN_TEST(test_unreachable_limits_are_best_effort);
    RUN_TEST(test_free_end_stop);
    RUN_TEST(test_trajectory_truncate_and_discard);
}
