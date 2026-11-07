// RmtStepChannel: one RMT TX channel streaming StepCmds from a StepStream.
//
// Uses the IDF "simple encoder" (rmt_new_simple_encoder, IDF >= 5.3): the driver calls our
// callback from its ISR whenever half of the channel memory is free, and the callback pulls
// commands from the lock-free queue (the same pattern as FastAccelStepper's RMT v2 driver).
// One RMT transaction lasts as long as the motion: it ends when the queue is empty and the
// producer is idle, and the planner task starts a new one when motion resumes.
#pragma once

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stepgen/step_encoder.hpp"
#include "stepgen/step_stream.hpp"

namespace nj {

struct ChannelConfig {
    gpio_num_t step_pin = GPIO_NUM_NC;
    gpio_num_t dir_pin = GPIO_NUM_NC;
    bool invert_dir = false;
    uint32_t pulse_ticks = 3 * cfg::kTicksPerUs;
    uint32_t dir_setup_ticks = 5 * cfg::kTicksPerUs;
};

class RmtStepChannel {
public:
    RmtStepChannel() = default;
    RmtStepChannel(const RmtStepChannel&) = delete;
    RmtStepChannel& operator=(const RmtStepChannel&) = delete;
    ~RmtStepChannel();

    esp_err_t init(const ChannelConfig& c, TaskHandle_t notify_task);

    /// Start a transaction (task context; channel must not be running).
    esp_err_t start();
    /// Abort transmission immediately and flush the queue.
    esp_err_t halt();

    bool running() const { return running_.load(std::memory_order_acquire); }
    StepStream& stream() { return stream_; }
    const StepStream& stream() const { return stream_; }
    StepEncoder& encoder() { return encoder_; }
    rmt_channel_handle_t handle() const { return chan_; }
    int32_t position() const { return encoder_.position(); }

private:
    static size_t encode_cb(const void* data, size_t data_size, size_t symbols_written, size_t symbols_free,
                            rmt_symbol_word_t* symbols, bool* done, void* arg);
    static bool done_cb(rmt_channel_handle_t chan, const rmt_tx_done_event_data_t* edata, void* arg);
    static void set_dir(void* ctx, bool positive);
    static void notify(void* ctx);

    ChannelConfig cfg_;
    rmt_channel_handle_t chan_ = nullptr;
    rmt_encoder_handle_t enc_ = nullptr;
    TaskHandle_t task_ = nullptr;
    StepStream stream_;
    StepEncoder encoder_;
    std::atomic<bool> running_{false};
    uint8_t payload_ = 0;
};

}  // namespace nj
