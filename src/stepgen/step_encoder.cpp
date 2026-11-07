#include "stepgen/step_encoder.hpp"

#include "platform/iram.hpp"

namespace nj {

void StepEncoder::configure(const Config& c, StepStream* stream, DirFn dir_fn, NotifyFn notify_fn, void* ctx)
{
    cfg_ = c;
    if (cfg_.filler_ticks < 2) {
        cfg_.filler_ticks = 2;
    }
    if (cfg_.underrun_ticks < 2) {
        cfg_.underrun_ticks = 2;
    }
    if (cfg_.dir_setup_ticks < 1) {
        cfg_.dir_setup_ticks = 1;
    }
    if (cfg_.pulse_ticks < 1) {
        cfg_.pulse_ticks = 1;
    }
    s_ = stream;
    dir_fn_ = dir_fn;
    notify_fn_ = notify_fn;
    ctx_ = ctx;
}

void StepEncoder::begin(bool dir_level_positive)
{
    phase_ = Phase::Fetch;
    steps_left_ = 0;
    low_left_ = 0;
    drain_left_ = 0;
    carry_ = 0;
    since_pulse_ = 0xFFFF;  // nothing in flight: no drain needed
    tail_done_ = true;
    dir_ = dir_level_positive;
}

NJ_IRAM void StepEncoder::set_dir(bool positive)
{
    if (dir_fn_) {
        dir_fn_(ctx_, positive);
    }
    dir_ = positive;
}

NJ_IRAM void StepEncoder::maybe_notify()
{
    if (s_->lead_ticks() < s_->low_water_ticks && s_->notify_armed.exchange(false) && notify_fn_) {
        notify_fn_(ctx_);
    }
}

NJ_IRAM bool StepEncoder::load_period()
{
    if (steps_left_ == 0) {
        StepCmd c;
        if (!s_->ring.pop(c)) {
            return false;
        }
        cmd_ = c;
        s_->consumed_ticks.fetch_add(c.total_ticks(), std::memory_order_relaxed);
        maybe_notify();
        if (c.steps == 0) {
            low_left_ = c.ticks + carry_;
            carry_ = 0;
            phase_ = Phase::Pause;
            return true;
        }
        steps_left_ = c.steps;
    }
    --steps_left_;

    uint32_t lw;
    if (cmd_.ticks > cfg_.pulse_ticks) {
        lw = cmd_.ticks - cfg_.pulse_ticks;
    } else {
        lw = 1;
        s_->stretched_ticks.fetch_add(cfg_.pulse_ticks + 1 - cmd_.ticks, std::memory_order_relaxed);
    }
    lw += carry_;
    carry_ = 0;

    const bool want = cmd_.dir_positive();
    if (want != dir_) {
        const uint32_t K = cfg_.drain_symbols;
        const uint32_t need = since_pulse_ >= K ? 0 : K - since_pulse_;
        const uint32_t setup = cfg_.dir_setup_ticks;
        if (need == 0) {
            if (lw < setup) {
                s_->stretched_ticks.fetch_add(setup - lw, std::memory_order_relaxed);
                lw = setup;
            }
            set_dir(want);
            phase_ = Phase::Low;
        } else {
            const uint32_t budget = lw > setup ? lw - setup : 0;
            uint32_t per = budget / need;
            if (per < cfg_.filler_ticks) {
                per = cfg_.filler_ticks;
                const uint32_t new_lw = per * need + setup;
                s_->stretched_ticks.fetch_add(new_lw - lw, std::memory_order_relaxed);
                lw = new_lw;
            } else if (per > 0xFFFE) {
                per = 0xFFFE;  // the rest of the low time follows after the toggle
            }
            drain_ticks_ = per;
            drain_left_ = need;
            pending_dir_ = want;
            phase_ = Phase::Drain;
        }
    } else {
        phase_ = Phase::Low;
    }
    low_left_ = lw;
    return true;
}

NJ_IRAM size_t StepEncoder::encode(uint32_t* out, size_t free, bool* done)
{
    size_t n = 0;
    while (n < free) {
        switch (phase_) {
        case Phase::Fetch:
            if (!load_period()) {
                if (s_->producer_idle.load(std::memory_order_acquire) && s_->ring.empty()) {
                    if (!tail_done_) {
                        // Last symbol ended high (the pulse): add a short low tail.
                        put(out, n, low_sym(cfg_.filler_ticks));
                        note_symbol();
                        tail_done_ = true;
                        continue;
                    }
                    carry_ = 0;
                    *done = true;
                    return n;
                }
                // Underrun: keep the line low and ask the producer for more.
                put(out, n, low_sym(cfg_.underrun_ticks));
                note_symbol();
                s_->underruns.fetch_add(1, std::memory_order_relaxed);
                if (s_->notify_armed.exchange(false) && notify_fn_) {
                    notify_fn_(ctx_);
                }
            }
            break;

        case Phase::Drain:
            put(out, n, low_sym(drain_ticks_));
            note_symbol();
            low_left_ -= drain_ticks_;
            if (--drain_left_ == 0) {
                set_dir(pending_dir_);
                phase_ = Phase::Low;
            }
            break;

        case Phase::Low:
            if (low_left_ > 0x7FFF) {
                const uint32_t chunk = (low_left_ - 1 < 0xFFFE) ? low_left_ - 1 : 0xFFFE;
                put(out, n, low_sym(chunk));
                note_symbol();
                low_left_ -= chunk;
            } else {
                put(out, n, sym(low_left_, false, cfg_.pulse_ticks, true));
                since_pulse_ = 0;
                tail_done_ = false;
                position_.fetch_add(dir_ ? 1 : -1, std::memory_order_relaxed);
                low_left_ = 0;
                phase_ = Phase::Fetch;
            }
            break;

        case Phase::Pause:
            if (low_left_ >= 2) {
                uint32_t chunk = low_left_ < 0xFFFE ? low_left_ : 0xFFFE;
                if (low_left_ - chunk == 1) {
                    --chunk;  // never leave a single tick that cannot form a symbol
                }
                put(out, n, low_sym(chunk));
                note_symbol();
                low_left_ -= chunk;
            }
            if (low_left_ < 2) {
                carry_ += low_left_;
                low_left_ = 0;
                phase_ = Phase::Fetch;
            }
            break;
        }
    }
    return n;
}

}  // namespace nj
