// StepEncoder: turns StepCmds into RMT symbols. Runs in the RMT ISR (through the IDF
// "simple encoder" callback) but is plain C++ so it is tested on the host.
//
// Symbol word layout (same as rmt_symbol_word_t):
//   bits 0..14 duration0, bit 15 level0, bits 16..30 duration1, bit 31 level1.
// Durations are 1..32767 ticks; 0 would be an end marker and is never produced.
//
// One step period of P ticks = (P - pulse) low, then `pulse` high:
//   low-only symbols while more than 32767 low ticks remain, then {0: rest, 1: pulse}.
//
// DIR change. The callback runs ahead of playback by at most the RMT memory size (K
// symbols), and the DIR pin is toggled at encode time. So before toggling, the encoder makes
// sure at least K symbols have been written since the last pulse ("drain"): every pulse of
// the old direction has then been played. It splits the leading low time of the reversed
// step into the missing drain symbols, toggles DIR, and keeps >= dir_setup ticks of low
// before the new pulse. If the low time is too short it is stretched and counted in
// StepStream::stretched_ticks (StepGenerator spaces reversals so this normally never
// happens).
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "config.hpp"
#include "stepgen/step_stream.hpp"

namespace nj {

class StepEncoder {
public:
    struct Config {
        uint32_t pulse_ticks = 3 * cfg::kTicksPerUs;
        uint32_t dir_setup_ticks = 5 * cfg::kTicksPerUs;
        uint32_t drain_symbols = cfg::kRmtMemSymbols;
        uint32_t filler_ticks = cfg::kFillerSymbolTicks;   ///< >= 2
        uint32_t underrun_ticks = cfg::kUnderrunFillTicks;  ///< >= 2
    };
    using DirFn = void (*)(void* ctx, bool positive);
    using NotifyFn = void (*)(void* ctx);

    void configure(const Config& c, StepStream* stream, DirFn dir_fn, NotifyFn notify_fn, void* ctx);

    /// Prepare a new RMT transaction. The channel must be idle (nothing in flight).
    void begin(bool dir_level_positive);

    /// Write up to `free` symbols. Sets *done when the transaction is finished (queue empty
    /// and producer idle). Always writes >= 1 symbol when free >= 1 unless *done is set.
    size_t encode(uint32_t* out, size_t free, bool* done);

    /// Signed count of pulses written (runs ahead of the motor by up to K symbols).
    int32_t position() const { return position_.load(std::memory_order_relaxed); }
    void set_position(int32_t p) { position_.store(p, std::memory_order_relaxed); }
    bool dir_positive() const { return dir_; }
    const Config& config() const { return cfg_; }
    /// Symbols written since configure() (diagnostics/tests).
    uint32_t symbols_total() const { return total_; }

    static constexpr uint32_t sym(uint32_t d0, bool l0, uint32_t d1, bool l1)
    {
        return d0 | (l0 ? 0x8000u : 0u) | (d1 << 16) | (l1 ? 0x80000000u : 0u);
    }
    static constexpr uint32_t low_sym(uint32_t ticks)  // ticks in [2, 0xFFFE]
    {
        return sym(ticks >> 1, false, ticks - (ticks >> 1), false);
    }

private:
    enum class Phase : uint8_t { Fetch, Drain, Low, Pause };

    bool load_period();
    void set_dir(bool positive);
    void maybe_notify();
    void note_symbol() { if (since_pulse_ < 0xFFFF) ++since_pulse_; }
    void put(uint32_t* out, size_t& n, uint32_t w)
    {
        out[n++] = w;
        ++total_;
    }

    Config cfg_;
    StepStream* s_ = nullptr;
    DirFn dir_fn_ = nullptr;
    NotifyFn notify_fn_ = nullptr;
    void* ctx_ = nullptr;

    StepCmd cmd_;
    uint32_t steps_left_ = 0;
    uint32_t low_left_ = 0;
    uint32_t drain_left_ = 0;
    uint32_t drain_ticks_ = 0;
    uint32_t carry_ = 0;
    uint32_t since_pulse_ = 0xFFFF;
    Phase phase_ = Phase::Fetch;
    bool dir_ = true;
    bool pending_dir_ = true;
    bool tail_done_ = true;
    uint32_t total_ = 0;
    std::atomic<int32_t> position_{0};
};

}  // namespace nj
