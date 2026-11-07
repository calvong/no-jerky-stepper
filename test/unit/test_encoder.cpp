// StepEncoder: symbol validity, chunking invariance, DIR drain, end of stream, underrun.
#include <vector>

#include "stepgen/step_encoder.hpp"
#include "test_util.hpp"

using namespace nj;
using namespace njt;

namespace {

struct Capture {
    std::vector<uint32_t> sym;
    std::vector<size_t> dir_at;  // symbol index at which DIR toggled
    std::vector<int> dir_val;
    int notifies = 0;
};
Capture* g_cap = nullptr;

void on_dir(void* ctx, bool positive)
{
    // Index of the next symbol to be written = number of symbols written so far.
    g_cap->dir_at.push_back(ctx ? static_cast<StepEncoder*>(ctx)->symbols_total() : 0);
    g_cap->dir_val.push_back(positive);
}
void on_notify(void*)
{
    ++g_cap->notifies;
}

uint32_t d0(uint32_t w) { return w & 0x7FFF; }
uint32_t d1(uint32_t w) { return (w >> 16) & 0x7FFF; }
bool l0(uint32_t w) { return (w & 0x8000) != 0; }
bool l1(uint32_t w) { return (w & 0x80000000u) != 0; }

std::vector<StepCmd> sample_cmds()
{
    // (Built from an array: GCC 14 -O2 reports a false stringop-overflow on push_back here.)
    static const StepCmd list[] = {
        StepCmd::step(20000, 1, true),
        StepCmd::step(400, 25, true),
        StepCmd::step(401, 3, true),
        StepCmd::pause(70001),           // > 65534: split pause
        StepCmd::pause(3),
        StepCmd::step(250000, 1, true),  // long low phase
        StepCmd::step(6000, 1, false),   // reversal with enough lead
        StepCmd::step(300, 10, false),
        StepCmd::step(200, 1, true),     // reversal with too little lead: stretched
        StepCmd::step(1000, 4, true),
        StepCmd::pause(1),
        StepCmd::step(50, 2, true),
    };
    return std::vector<StepCmd>(list, list + sizeof(list) / sizeof(list[0]));
}

// Encode `cmds` with chunks drawn from `rng` (or one big chunk when rng == nullptr).
void encode_all(const std::vector<StepCmd>& cmds, Rng* rng, Capture& cap, StepEncoder& enc, StepStream& st)
{
    g_cap = &cap;
    for (const StepCmd& c : cmds) {
        TEST_ASSERT_TRUE(st.push(c));
    }
    st.producer_idle.store(true);
    StepEncoder::Config ec;
    enc.configure(ec, &st, &on_dir, &on_notify, &enc);
    enc.begin(true);
    bool done = false;
    uint32_t buf[64];
    for (int guard = 0; guard < 100000 && !done; ++guard) {
        const size_t room = rng ? 1 + rng->next() % 48 : 64;
        const size_t n = enc.encode(buf, room, &done);
        TEST_ASSERT_TRUE(n <= room);
        TEST_ASSERT_TRUE(n > 0 || done);
        for (size_t i = 0; i < n; ++i) {
            cap.sym.push_back(buf[i]);
        }
    }
    TEST_ASSERT_TRUE(done);
}

}  // namespace

static void test_encoder_symbols_valid_and_time_exact()
{
    const std::vector<StepCmd> cmds = sample_cmds();
    Capture cap;
    StepEncoder* enc = new StepEncoder;
    StepStream* st = new StepStream;
    encode_all(cmds, nullptr, cap, *enc, *st);

    uint64_t sym_ticks = 0;
    int pulses = 0;
    for (uint32_t w : cap.sym) {
        TEST_ASSERT_TRUE(d0(w) >= 1 && d0(w) <= 0x7FFF);
        TEST_ASSERT_TRUE(d1(w) >= 1 && d1(w) <= 0x7FFF);
        TEST_ASSERT_FALSE(l0(w));  // low always comes first
        sym_ticks += d0(w) + d1(w);
        if (l1(w)) {
            ++pulses;
            TEST_ASSERT_EQUAL_UINT32(enc->config().pulse_ticks, d1(w));
        }
    }
    uint64_t cmd_ticks = 0;
    int steps = 0;
    for (const StepCmd& c : cmds) {
        cmd_ticks += c.total_ticks();
        steps += c.steps;
    }
    TEST_ASSERT_EQUAL_INT(steps, pulses);
    const uint64_t tail = enc->config().filler_ticks;
    TEST_ASSERT_EQUAL_UINT64(cmd_ticks + st->stretched_ticks.load() + tail, sym_ticks);
    TEST_ASSERT_TRUE(st->stretched_ticks.load() > 0);  // the short reversal was stretched
    TEST_ASSERT_EQUAL_INT32(1 + 25 + 3 + 1 - 1 - 10 + 1 + 4 + 2, enc->position());
    delete enc;
    delete st;
}

static void test_encoder_dir_toggles_after_drain()
{
    const std::vector<StepCmd> cmds = sample_cmds();
    Capture cap;
    StepEncoder* enc = new StepEncoder;
    StepStream* st = new StepStream;
    encode_all(cmds, nullptr, cap, *enc, *st);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(cap.dir_at.size()));
    const uint32_t K = enc->config().drain_symbols;
    for (size_t k = 0; k < cap.dir_at.size(); ++k) {
        const size_t at = cap.dir_at[k];
        // Index of the last pulse symbol before the toggle.
        size_t last = 0;
        bool found = false;
        for (size_t i = 0; i < at; ++i) {
            if (l1(cap.sym[i])) {
                last = i;
                found = true;
            }
        }
        TEST_ASSERT_TRUE(found);
        TEST_ASSERT_TRUE_MESSAGE(at - (last + 1) >= K, "DIR toggled before the RMT buffer drained");
        // DIR setup: low time between the toggle and the next pulse edge.
        uint64_t low = 0;
        size_t i = at;
        for (; i < cap.sym.size() && !l1(cap.sym[i]); ++i) {
            low += d0(cap.sym[i]) + d1(cap.sym[i]);
        }
        low += d0(cap.sym[i]);
        TEST_ASSERT_TRUE(low >= enc->config().dir_setup_ticks);
    }
    TEST_ASSERT_FALSE(cap.dir_val[0]);
    TEST_ASSERT_TRUE(cap.dir_val[1]);
    delete enc;
    delete st;
}

static void test_encoder_chunking_invariant()
{
    const std::vector<StepCmd> cmds = sample_cmds();
    Capture one;
    StepEncoder* e1 = new StepEncoder;
    StepStream* s1 = new StepStream;
    encode_all(cmds, nullptr, one, *e1, *s1);
    for (uint32_t seed = 1; seed < 6; ++seed) {
        Capture many;
        StepEncoder* e2 = new StepEncoder;
        StepStream* s2 = new StepStream;
        Rng rng;
        rng.s = seed;
        encode_all(cmds, &rng, many, *e2, *s2);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(one.sym.size()), static_cast<int>(many.sym.size()));
        for (size_t i = 0; i < one.sym.size(); ++i) {
            TEST_ASSERT_EQUAL_HEX32(one.sym[i], many.sym[i]);
        }
        TEST_ASSERT_EQUAL_INT(static_cast<int>(one.dir_at.size()), static_cast<int>(many.dir_at.size()));
        for (size_t i = 0; i < one.dir_at.size(); ++i) {
            TEST_ASSERT_EQUAL_INT(static_cast<int>(one.dir_at[i]), static_cast<int>(many.dir_at[i]));
        }
        delete e2;
        delete s2;
    }
    delete e1;
    delete s1;
}

static void test_encoder_underrun_and_notify()
{
    Capture cap;
    g_cap = &cap;
    StepEncoder* enc = new StepEncoder;
    StepStream* st = new StepStream;
    StepEncoder::Config ec;
    enc->configure(ec, st, &on_dir, &on_notify, nullptr);
    enc->begin(true);
    st->producer_idle.store(false);
    st->notify_armed.store(true);
    bool done = false;
    uint32_t buf[8];
    const size_t n = enc->encode(buf, 8, &done);
    TEST_ASSERT_FALSE(done);
    TEST_ASSERT_EQUAL_UINT32(8, static_cast<uint32_t>(n));
    TEST_ASSERT_EQUAL_UINT32(8, st->underruns.load());
    TEST_ASSERT_EQUAL_INT(1, cap.notifies);  // armed once, notified once
    // Producer catches up, then finishes.
    st->push(StepCmd::step(1000, 2, true));
    st->producer_idle.store(true);
    size_t total = 0;
    for (int i = 0; i < 10 && !done; ++i) {
        total += enc->encode(buf, 8, &done);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_EQUAL_INT32(2, enc->position());
    TEST_ASSERT_EQUAL_UINT32(3, static_cast<uint32_t>(total));  // 2 pulses + tail
    delete enc;
    delete st;
}

void nj_test_encoder()
{
    RUN_TEST(test_encoder_symbols_valid_and_time_exact);
    RUN_TEST(test_encoder_dir_toggles_after_drain);
    RUN_TEST(test_encoder_chunking_invariant);
    RUN_TEST(test_encoder_underrun_and_notify);
}
