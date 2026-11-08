// Closed-loop feedback: offset track, slip estimation, continuous / settle correction, faults,
// incremental sensors. A simulated plant plays the step stream in real time, can lose pulses,
// and feeds (optionally delayed / noisy) sensor readings back like the planner task does.
#include <cmath>

#include "core/feedback.hpp"
#include "core/group_planner.hpp"
#include "nj/sensor.hpp"
#include "test_util.hpp"

using namespace nj;
using namespace njt;

namespace {

/// Plays commands from the stream in real time; `skip` pulses are lost (motor does not move).
struct Motor {
    StepStream* st = nullptr;
    StepCmd cur;
    bool have = false;
    uint32_t done = 0;
    int64_t budget = 0;  ///< ticks of playback time available
    int64_t pos = 0;
    int64_t skip = 0;
    bool running = false;

    void advance(int64_t ticks)
    {
        if (!running) {
            return;
        }
        budget += ticks;
        for (;;) {
            if (!have) {
                if (!st->ring.pop(cur)) {
                    running = false;  // transaction ends when the queue runs dry
                    budget = 0;
                    return;
                }
                have = true;
                done = 0;
            }
            if (cur.steps == 0) {
                if (cur.ticks > budget) {
                    return;
                }
                budget -= cur.ticks;
                st->consumed_ticks.fetch_add(cur.ticks);
                have = false;
                continue;
            }
            while (done < cur.steps) {
                if (cur.ticks > budget) {
                    return;
                }
                budget -= cur.ticks;
                st->consumed_ticks.fetch_add(cur.ticks);
                ++done;
                if (skip > 0) {
                    --skip;
                } else {
                    pos += cur.dir_positive() ? 1 : -1;
                }
            }
            have = false;
        }
    }
};

struct Sim {
    Axis ax;
    StepStream st;
    Feedback fb;
    Motor motor;
    Rng rng;
    int64_t now_us = 0;
    static constexpr int64_t kCycleUs = 2000;  // planner service period
    int64_t sample_period_us = 2000;
    int64_t next_sample_us = 0;
    double noise = 0.0;           ///< uniform +- [steps]
    int64_t sensor_delay_us = 0;  ///< true age of a reading
    bool sensor_alive = true;
    double glitch = 0.0;          ///< added to the next reading only
    double spu = 1.0;             ///< steps per sensor unit
    double sensor_zero = 0.0;     ///< sensor reading at step 0 [sensor units]
    int64_t pos_hist[64] = {};    ///< motor position per cycle (for delayed readings)
    int hist_i = 0;
    uint32_t stops = 0;
    double max_abs_offset = 0.0;
    Axis* const* group_axes = nullptr;  ///< set: a fault stops the whole group (like service_group)
    int group_n = 0;

    Sim(const Limits& lim, const FeedbackConfig& fc, SensorKind kind = SensorKind::Absolute)
    {
        ax.configure(default_axis_config(lim));
        fb.configure(fc, kind);
        motor.st = &st;
    }

    double reading() const
    {
        const int64_t lag = sensor_delay_us / kCycleUs;
        const int64_t p = pos_hist[(hist_i - static_cast<int>(lag) + 64 * 4) % 64];
        return sensor_zero + static_cast<double>(p) / spu;
    }

    void cycle()
    {
        now_us += kCycleUs;
        motor.advance(kCycleUs * cfg::kTicksPerUs);
        hist_i = (hist_i + 1) % 64;
        pos_hist[hist_i] = motor.pos;

        if (sensor_alive && now_us >= next_sample_us) {
            double r = reading() + rng.uniform(-noise, noise) / spu + glitch / spu;
            glitch = 0.0;
            fb.add_sample(now_us, r);
            next_sample_us = now_us + sample_period_us;
        }

        const StepGenerator& gen = ax.generator();
        const int64_t playback = gen.emitted_tick() - static_cast<int64_t>(st.lead_ticks());
        FeedbackIo io;
        io.now_us = now_us;
        io.scan_play_us = now_us + std::max<int64_t>(0, gen.scan_tick() - playback) / cfg::kTicksPerUs;
        io.stream_busy = motor.running || !st.ring.empty();
        const FeedbackOutcome out = fb.service(ax, io);
        if (out.stop) {
            ++stops;
            if (group_axes) {
                group_stop(group_axes, group_n, 0.0);
            } else if (ax.has_motion() && ax.stop(0.0) != Status::Ok) {
                ax.stop(0.25);
            }
        }
        max_abs_offset = std::fmax(max_abs_offset, std::fabs(ax.offset_target()));

        if (ax.has_motion()) {
            ax.generate(st, playback + cfg::kLookaheadTicks, false);
        }
        if (!motor.running && !st.ring.empty()) {
            motor.running = true;
        }
    }

    bool busy() const { return ax.has_motion() || motor.running || !st.ring.empty() || fb.settling(); }

    void run_for(double s)
    {
        const int64_t end = now_us + static_cast<int64_t>(s * 1e6);
        while (now_us < end) {
            cycle();
        }
    }

    /// Run until idle (motion, stream and settle phase done) or `max_s`.
    void run_idle(double max_s = 20.0)
    {
        const int64_t end = now_us + static_cast<int64_t>(max_s * 1e6);
        cycle();
        while (busy() && now_us < end) {
            cycle();
        }
        run_for(0.05);  // let the absorb happen
    }

    /// Run until the motor has moved past `p` (in the direction of travel), then lose `n` pulses.
    void lose_after(int64_t p, int64_t n, double max_s = 20.0)
    {
        const int64_t end = now_us + static_cast<int64_t>(max_s * 1e6);
        while (motor.pos < p && now_us < end) {
            cycle();
        }
        motor.skip = n;
    }
};

FeedbackConfig base_config(FeedbackMode m)
{
    FeedbackConfig c;
    c.mode = m;
    c.filter_tau_s = 0.02;
    c.deadband_steps = 1.0;
    c.correction_time_s = 0.1;
    c.max_correction_steps = 200;
    c.sensor_timeout_ms = 100;
    c.settle_delay_ms = 20;
    c.settle_samples = 5;
    c.settle_tolerance_steps = 1.0;
    return c;
}

const Limits kLim{20000.0, 2e5, 2e7};

}  // namespace

static void test_offset_track_is_smooth_and_absorbed()
{
    Axis* ax = new Axis;
    ax->configure(default_axis_config(kLim));
    StepStream* st = new StepStream;
    StepChecker chk;
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(4000, 1.0));
    run_until(*ax, *st, chk, 0.4);
    const double t = ax->generator().scan_time();
    const State before = ax->commanded();
    TEST_ASSERT_EQUAL(Status::Ok, ax->set_offset(25.0, 0.2));
    // The sum stays C2-continuous at the splice and the plan itself is untouched.
    const State at = ax->view().eval(t);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9 * 4000, before.p, at.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6 * 4000, before.v, at.v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6 * 4000, before.a, at.a);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4000.0, ax->target());
    run_to_end(*ax, *st, chk);
    TEST_ASSERT_EQUAL_INT64(4025, ax->generator().position());
    TEST_ASSERT_EQUAL_INT64(4025, chk.pos);
    TEST_ASSERT_TRUE(chk.min_interval >= ax->generator().config().min_period_ticks);
    TEST_ASSERT_FALSE(ax->has_motion());
    // Fold the offset into the counter: the counter follows the plan again.
    TEST_ASSERT_EQUAL_INT64(25, ax->absorb_offset());
    TEST_ASSERT_EQUAL_INT64(4000, ax->generator().position());
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, ax->offset_target());
    TEST_ASSERT_FALSE(ax->has_motion());
    // A retarget while an offset is active keeps both.
    TEST_ASSERT_EQUAL(Status::Ok, ax->set_offset(-10.0, 0.3));
    TEST_ASSERT_EQUAL(Status::Ok, ax->move_to(1000, 0.5));
    run_to_end(*ax, *st, chk);
    TEST_ASSERT_EQUAL_INT64(990, ax->generator().position());
    delete ax;
    delete st;
}

static void test_continuous_corrects_lost_steps()
{
    Sim* s = new Sim(kLim, base_config(FeedbackMode::Continuous));
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(6000, 2.0));
    s->lose_after(2000, 20);
    s->run_idle();
    TEST_ASSERT_EQUAL(FeedbackState::Tracking, s->fb.status().state);
    TEST_ASSERT_INT64_WITHIN(1, 6000, s->motor.pos);
    TEST_ASSERT_TRUE(std::fabs(s->max_abs_offset - 20.0) < 3.0);  // no large overshoot of the correction
    // Idle: the correction was folded into the counter, which equals the target again.
    TEST_ASSERT_EQUAL_INT64(6000, s->ax.generator().position());
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, s->ax.offset_target());
    TEST_ASSERT_DOUBLE_WITHIN(1.5, 0.0, s->fb.status().slip);
    TEST_ASSERT_EQUAL_UINT32(0, s->stops);
    delete s;
}

static void test_settle_corrects_final_error()
{
    Sim* s = new Sim(kLim, base_config(FeedbackMode::Settle));
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(3000, 1.0));
    s->lose_after(2500, 5);
    // During the move nothing is corrected in Settle mode.
    while (s->ax.has_planned_motion()) {
        s->cycle();
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, s->ax.offset_target());
    }
    TEST_ASSERT_TRUE(s->fb.settling());
    s->run_idle();
    const FeedbackStatus fs = s->fb.status();
    TEST_ASSERT_EQUAL(FeedbackState::Settled, fs.state);
    TEST_ASSERT_TRUE(fs.settle_attempts >= 1 && fs.settle_attempts <= 3);
    TEST_ASSERT_INT64_WITHIN(1, 3000, s->motor.pos);
    TEST_ASSERT_EQUAL_INT64(3000, s->ax.generator().position());
    delete s;
}

static void test_continuous_and_settle_with_noise()
{
    FeedbackConfig c = base_config(FeedbackMode::ContinuousAndSettle);
    c.deadband_steps = 2.0;
    c.settle_samples = 16;
    Sim* s = new Sim(kLim, c);
    s->noise = 1.0;
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(-5000, 1.5));
    s->run_for(0.6);
    s->motor.skip = 30;
    s->run_idle();
    TEST_ASSERT_EQUAL(FeedbackState::Settled, s->fb.status().state);
    TEST_ASSERT_INT64_WITHIN(1, -5000, s->motor.pos);
    delete s;
}

static void test_noise_below_deadband_does_nothing()
{
    FeedbackConfig c = base_config(FeedbackMode::Continuous);
    c.deadband_steps = 2.0;
    Sim* s = new Sim(kLim, c);
    s->noise = 0.8;
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(5000, 1.5));
    s->run_idle();
    s->run_for(0.5);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, s->max_abs_offset);
    TEST_ASSERT_EQUAL_INT64(5000, s->motor.pos);
    delete s;
}

static void test_latency_is_compensated()
{
    // Readings are 30 ms old. With the latency configured, the history lookup compares them
    // with where the motor was commanded to be 30 ms ago, so a fast move causes no correction.
    FeedbackConfig c = base_config(FeedbackMode::Continuous);
    c.deadband_steps = 3.0;
    c.sensor_latency_ms = 30.0;
    Sim* s = new Sim(kLim, c);
    s->sensor_delay_us = 30000;
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(8000, 1.0));  // peak 15000 steps/s
    s->run_idle();
    TEST_ASSERT_TRUE(s->max_abs_offset < 1e-9);
    TEST_ASSERT_EQUAL_INT64(8000, s->motor.pos);
    TEST_ASSERT_EQUAL(FeedbackState::Tracking, s->fb.status().state);
    delete s;
}

static void test_slip_too_large_faults_and_stops()
{
    Sim* s = new Sim(kLim, base_config(FeedbackMode::Continuous));
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(20000, 3.0));
    s->lose_after(3000, 400);
    s->run_idle();
    const FeedbackStatus fs = s->fb.status();
    TEST_ASSERT_EQUAL(FeedbackState::Fault, fs.state);
    TEST_ASSERT_EQUAL(FeedbackFault::SlipTooLarge, fs.fault);
    TEST_ASSERT_EQUAL_UINT32(1, s->stops);
    TEST_ASSERT_TRUE(s->ax.generator().position() < 20000);  // stopped early
    // Clearing the fault re-arms the loop.
    s->fb.set_mode(FeedbackMode::Continuous);
    TEST_ASSERT_EQUAL(FeedbackState::Tracking, s->fb.status().state);
    delete s;
}

static void test_sensor_timeout_faults()
{
    Sim* s = new Sim(kLim, base_config(FeedbackMode::Continuous));
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(6000, 2.0));
    s->run_for(0.3);
    s->sensor_alive = false;
    s->run_for(0.3);
    TEST_ASSERT_EQUAL(FeedbackState::Fault, s->fb.status().state);
    TEST_ASSERT_EQUAL(FeedbackFault::Timeout, s->fb.status().fault);
    TEST_ASSERT_EQUAL_UINT32(1, s->stops);
    delete s;
}

static void test_settle_mode_ignores_missing_sensor_at_rest()
{
    // E.g. a SoftPot nobody touches: no samples while idle is not a fault in Settle mode.
    Sim* s = new Sim(kLim, base_config(FeedbackMode::Settle));
    s->sensor_alive = false;
    s->run_for(1.0);
    TEST_ASSERT_EQUAL(FeedbackState::Tracking, s->fb.status().state);
    delete s;
}

static void test_jump_faults()
{
    FeedbackConfig c = base_config(FeedbackMode::Continuous);
    c.max_jump_steps = 10.0;
    Sim* s = new Sim(kLim, c);
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(6000, 2.0));
    s->run_for(0.5);
    TEST_ASSERT_EQUAL(FeedbackState::Tracking, s->fb.status().state);  // fast motion is not a jump
    s->glitch = 50.0;
    s->run_for(0.02);
    TEST_ASSERT_EQUAL(FeedbackFault::Jump, s->fb.status().fault);
    delete s;
}

static void test_incremental_needs_reference()
{
    FeedbackConfig c = base_config(FeedbackMode::Continuous);
    c.steps_per_unit = 3200.0 / 4000.0;  // e.g. 3200 steps/rev motor, 4000 counts/rev encoder
    Sim* s = new Sim(kLim, c, SensorKind::Incremental);
    s->spu = c.steps_per_unit;
    s->sensor_zero = 12345;  // arbitrary power-up count
    s->run_for(0.05);
    TEST_ASSERT_EQUAL(FeedbackState::Unreferenced, s->fb.status().state);
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(2000, 1.0));
    s->lose_after(500, 10);
    s->run_idle();
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, s->max_abs_offset);  // no corrections without a reference
    TEST_ASSERT_EQUAL_INT64(1990, s->motor.pos);
    TEST_ASSERT_TRUE(s->fb.status().rejected > 0);

    // Reference: the motor's current position becomes the counter value.
    s->fb.request_reference();
    s->run_for(0.02);
    TEST_ASSERT_FALSE(s->fb.reference_pending());
    TEST_ASSERT_EQUAL(FeedbackState::Tracking, s->fb.status().state);
    TEST_ASSERT_DOUBLE_WITHIN(0.6, 2000.0, s->fb.status().measured);  // measured := commanded
    // Now losses are corrected.
    TEST_ASSERT_EQUAL(Status::Ok, s->ax.move_to(4000, 1.0));
    s->lose_after(2500, 15);
    s->run_idle();
    TEST_ASSERT_INT64_WITHIN(1, 4000 - 10, s->motor.pos);
    delete s;
}

namespace {

struct WrapCounter : IncrementalSensor {
    int32_t raw = 0;
    int32_t range = 65536;
    bool ok = true;
    bool readRaw(int32_t* r) override
    {
        *r = raw;
        return ok;
    }
    int32_t rawRange() const override { return range; }
};

}  // namespace

static void test_incremental_unwrap()
{
    // Signed 16-bit counter (PCNT style, [-32768, 32767]).
    WrapCounter c;
    int64_t truth = 0;
    TEST_ASSERT_TRUE(c.update());
    for (int i = 0; i < 400; ++i) {
        truth += 1000;
        c.raw = static_cast<int16_t>(static_cast<uint16_t>(truth & 0xFFFF));
        TEST_ASSERT_TRUE(c.update());
    }
    TEST_ASSERT_EQUAL_INT64(400000, c.count());
    for (int i = 0; i < 900; ++i) {
        truth -= 777;
        c.raw = static_cast<int16_t>(static_cast<uint16_t>(truth & 0xFFFF));
        c.update();
    }
    TEST_ASSERT_EQUAL_INT64(truth, c.count());
    TEST_ASSERT_DOUBLE_WITHIN(0.0, static_cast<double>(truth), c.getPosition());

    // Unsigned 12-bit (magnetic, [0, 4096)), both directions, plus a failed read.
    WrapCounter m;
    m.range = 4096;
    m.raw = 4000;
    TEST_ASSERT_TRUE(m.update());
    int64_t t2 = 0;
    const int deltas[] = {50, 200, 1500, -1900, -2000, -300, 1000, 2047, -2048};
    for (int d : deltas) {
        t2 += d;
        m.raw = static_cast<int32_t>(((4000 + t2) % 4096 + 4096) % 4096);
        TEST_ASSERT_TRUE(m.update());
        TEST_ASSERT_EQUAL_INT64(t2, m.count());
    }
    m.ok = false;
    TEST_ASSERT_FALSE(m.update());
    TEST_ASSERT_EQUAL_INT64(t2, m.count());
    m.zero(-5);
    TEST_ASSERT_EQUAL_INT64(-5, m.count());
}

static void test_history_interpolation_and_reframe()
{
    Sim* s = new Sim(kLim, base_config(FeedbackMode::Continuous));
    s->run_for(0.02);
    double p = 0;
    TEST_ASSERT_TRUE(s->fb.commanded_at(s->now_us, &p));
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, p);
    TEST_ASSERT_FALSE(s->fb.commanded_at(s->now_us - 10000000, &p));  // older than the history
    // setPosition on an absolute sensor shifts the mapping with the frame: still no slip.
    s->fb.reframe(100.0);
    TEST_ASSERT_TRUE(s->fb.commanded_at(s->now_us, &p));
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 100.0, p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 100.0, s->fb.config().offset_steps);
    delete s;
}

// ---- groups: a closed-loop member next to an open-loop member ----

namespace {

/// X with feedback, Y open loop (feedback Off behaves like a member without a sensor), planned
/// as a group. A fault on X stops both through group_stop(), as service_group does.
struct GroupSim {
    Sim* x;
    Sim* y;
    Axis* axes[2];

    explicit GroupSim(const FeedbackConfig& fc)
    {
        x = new Sim(kLim, fc);
        FeedbackConfig off = fc;
        off.mode = FeedbackMode::Off;
        y = new Sim(kLim, off);
        axes[0] = &x->ax;
        axes[1] = &y->ax;
        x->group_axes = axes;
        x->group_n = 2;
        y->group_axes = axes;
        y->group_n = 2;
        group_align(axes, 2);
    }
    ~GroupSim()
    {
        delete x;
        delete y;
    }
    void cycle()
    {
        x->cycle();
        y->cycle();
    }
    bool busy() const { return x->busy() || y->busy(); }
};

}  // namespace

static void test_group_mixed_closed_and_open_loop()
{
    GroupSim* g = new GroupSim(base_config(FeedbackMode::ContinuousAndSettle));
    const double targets[2] = {6000.0, 3000.0};
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(g->axes, 2, targets, 2.0));
    bool lost = false;
    int64_t t_loss = 0;
    double dev_max = 0.0;    // |x - 2y| over the whole move
    double dev_after = 0.0;  // same, once the correction had time to act
    const int64_t end = g->x->now_us + 20000000;
    while (g->busy() && g->x->now_us < end) {
        if (!lost && g->x->motor.pos >= 2000) {
            g->x->motor.skip = 25;
            lost = true;
            t_loss = g->x->now_us;
        }
        g->cycle();
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, g->y->ax.offset_target());  // Y is never touched
        if (g->y->ax.has_planned_motion()) {
            const double dev = std::fabs(static_cast<double>(g->x->motor.pos - 2 * g->y->motor.pos));
            dev_max = std::fmax(dev_max, dev);
            if (lost && g->x->now_us - t_loss > 400000) {
                dev_after = std::fmax(dev_after, dev);
            }
        }
    }
    for (int i = 0; i < 25; ++i) {
        g->cycle();  // let the absorb happen
    }
    TEST_ASSERT_TRUE(lost);
    TEST_ASSERT_TRUE(dev_max >= 20.0);   // the loss really pushed X off the line ...
    TEST_ASSERT_TRUE(dev_after <= 4.0);  // ... and the correction put it back on
    TEST_ASSERT_INT64_WITHIN(1, 6000, g->x->motor.pos);
    TEST_ASSERT_EQUAL_INT64(3000, g->y->motor.pos);
    TEST_ASSERT_EQUAL_INT64(6000, g->x->ax.generator().position());  // correction absorbed
    TEST_ASSERT_EQUAL_UINT32(0, g->x->stops);
    delete g;
}

static void test_group_fault_stops_all_members()
{
    GroupSim* g = new GroupSim(base_config(FeedbackMode::Continuous));
    const double targets[2] = {20000.0, 10000.0};
    TEST_ASSERT_EQUAL(Status::Ok, group_move_to(g->axes, 2, targets, 3.0));
    bool lost = false;
    const int64_t end = g->x->now_us + 20000000;
    while (g->busy() && g->x->now_us < end) {
        if (!lost && g->x->motor.pos >= 3000) {
            g->x->motor.skip = 400;  // stall
            lost = true;
        }
        g->cycle();
    }
    TEST_ASSERT_EQUAL(FeedbackFault::SlipTooLarge, g->x->fb.status().fault);
    TEST_ASSERT_EQUAL_UINT32(1, g->x->stops);
    // Y has no sensor but stopped with X, short of its target ...
    TEST_ASSERT_TRUE(g->y->ax.target() < 10000.0);
    TEST_ASSERT_TRUE(g->x->ax.target() < 20000.0);
    // ... at the same moment with shared timing, so the plans stay on the 2:1 line.
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 2.0 * g->y->ax.target(), g->x->ax.target());
    delete g;
}

void nj_test_feedback()
{
    RUN_TEST(test_offset_track_is_smooth_and_absorbed);
    RUN_TEST(test_continuous_corrects_lost_steps);
    RUN_TEST(test_settle_corrects_final_error);
    RUN_TEST(test_continuous_and_settle_with_noise);
    RUN_TEST(test_noise_below_deadband_does_nothing);
    RUN_TEST(test_latency_is_compensated);
    RUN_TEST(test_slip_too_large_faults_and_stops);
    RUN_TEST(test_sensor_timeout_faults);
    RUN_TEST(test_settle_mode_ignores_missing_sensor_at_rest);
    RUN_TEST(test_jump_faults);
    RUN_TEST(test_incremental_needs_reference);
    RUN_TEST(test_incremental_unwrap);
    RUN_TEST(test_history_interpolation_and_reframe);
    RUN_TEST(test_group_mixed_closed_and_open_loop);
    RUN_TEST(test_group_fault_stops_all_members);
}
