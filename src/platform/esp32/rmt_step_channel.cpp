#include "platform/esp32/rmt_step_channel.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "hal/gpio_ll.h"
#include "platform/iram.hpp"

namespace nj {

static const char* TAG = "nj_rmt";

static_assert(sizeof(rmt_symbol_word_t) == sizeof(uint32_t), "unexpected RMT symbol size");

RmtStepChannel::~RmtStepChannel()
{
    if (chan_) {
        rmt_disable(chan_);
        rmt_del_channel(chan_);
    }
    if (enc_) {
        rmt_del_encoder(enc_);
    }
}

esp_err_t RmtStepChannel::init(const ChannelConfig& c, TaskHandle_t notify_task)
{
    cfg_ = c;
    task_ = notify_task;
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(c.step_pin), ESP_ERR_INVALID_ARG, TAG, "invalid step pin");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(c.dir_pin), ESP_ERR_INVALID_ARG, TAG, "invalid dir pin");

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << c.dir_pin;
    io.mode = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "dir pin config");
    ESP_RETURN_ON_ERROR(gpio_set_level(c.dir_pin, c.invert_dir ? 0 : 1), TAG, "dir pin level");

    rmt_tx_channel_config_t tx = {};
    tx.gpio_num = c.step_pin;
    tx.clk_src = RMT_CLK_SRC_DEFAULT;
    tx.resolution_hz = CONFIG_NJ_RMT_RESOLUTION_HZ;
    tx.mem_block_symbols = cfg::kRmtMemSymbols;
    tx.trans_queue_depth = 2;
    tx.intr_priority = 0;
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx, &chan_), TAG, "rmt_new_tx_channel");

    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = &RmtStepChannel::done_cb;
    ESP_RETURN_ON_ERROR(rmt_tx_register_event_callbacks(chan_, &cbs, this), TAG, "register callbacks");

    rmt_simple_encoder_config_t ec = {};
    ec.callback = &RmtStepChannel::encode_cb;
    ec.arg = this;
    ec.min_chunk_size = 1;  // the encoder always writes >= 1 symbol when there is room
    ESP_RETURN_ON_ERROR(rmt_new_simple_encoder(&ec, &enc_), TAG, "rmt_new_simple_encoder");

    StepEncoder::Config ecfg;
    ecfg.pulse_ticks = c.pulse_ticks;
    ecfg.dir_setup_ticks = c.dir_setup_ticks;
    encoder_.configure(ecfg, &stream_, &RmtStepChannel::set_dir, &RmtStepChannel::notify, this);
    encoder_.begin(true);

    ESP_RETURN_ON_ERROR(rmt_enable(chan_), TAG, "rmt_enable");
    return ESP_OK;
}

esp_err_t RmtStepChannel::start()
{
    if (running()) {
        return ESP_ERR_INVALID_STATE;
    }
    encoder_.begin(encoder_.dir_positive());
    running_.store(true, std::memory_order_release);
    rmt_transmit_config_t tc = {};
    tc.loop_count = 0;
    tc.flags.eot_level = 0;
    tc.flags.queue_nonblocking = 1;
    const esp_err_t err = rmt_transmit(chan_, enc_, &payload_, 1, &tc);
    if (err != ESP_OK) {
        running_.store(false, std::memory_order_release);
        ESP_LOGE(TAG, "rmt_transmit failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t RmtStepChannel::halt()
{
    ESP_RETURN_ON_ERROR(rmt_disable(chan_), TAG, "rmt_disable");
    running_.store(false, std::memory_order_release);
    stream_.flush();
    return rmt_enable(chan_);
}

NJ_IRAM size_t RmtStepChannel::encode_cb(const void*, size_t, size_t, size_t symbols_free,
                                         rmt_symbol_word_t* symbols, bool* done, void* arg)
{
    auto* self = static_cast<RmtStepChannel*>(arg);
    return self->encoder_.encode(reinterpret_cast<uint32_t*>(symbols), symbols_free, done);
}

NJ_IRAM bool RmtStepChannel::done_cb(rmt_channel_handle_t, const rmt_tx_done_event_data_t*, void* arg)
{
    auto* self = static_cast<RmtStepChannel*>(arg);
    self->running_.store(false, std::memory_order_release);
    BaseType_t woken = pdFALSE;
    if (self->task_) {
        vTaskNotifyGiveFromISR(self->task_, &woken);
    }
    return woken == pdTRUE;
}

NJ_IRAM void RmtStepChannel::set_dir(void* ctx, bool positive)
{
    auto* self = static_cast<RmtStepChannel*>(ctx);
    const uint32_t level = (positive != self->cfg_.invert_dir) ? 1 : 0;
    gpio_ll_set_level(GPIO_LL_GET_HW(GPIO_PORT_0), self->cfg_.dir_pin, level);
}

NJ_IRAM void RmtStepChannel::notify(void* ctx)
{
    auto* self = static_cast<RmtStepChannel*>(ctx);
    if (!self->task_) {
        return;
    }
    if (xPortInIsrContext()) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(self->task_, &woken);
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    } else {
        xTaskNotifyGive(self->task_);
    }
}

}  // namespace nj
