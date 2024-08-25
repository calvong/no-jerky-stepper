#ifndef ESP32S3_RMT_H
#define ESP32S3_RMT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/rmt_tx.h>


typedef struct esp32s3_rmt_encoder_data {
    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t rmt_encoder;
    uint32_t* data;         
    uint32_t data_size;     
} esp32s3_rmt_encoder_data_t;


// public functions
rmt_channel_handle_t esp32s3_rmt_init(uint8_t step_pin);
esp_err_t esp32s3_rmt_new_stepper_curve_encoder(esp32s3_rmt_encoder_data_t *encoder_data);


// static functions
static size_t esp32s3_rmt_encode_stepper_curve(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
static esp_err_t esp32s3_rmt_del_stepper_curve_encoder(rmt_encoder_t *encoder);
static esp_err_t esp32s3_rmt_reset_stepper_curve_encoder(rmt_encoder_t *encoder);
static void esp32s3_rmt_tx_done_callback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_data);
static void increase_allocated_curve_memory_check(uint32_t current_size, uint32_t* current_max_size, rmt_symbol_word_t *curve);

#ifdef __cplusplus
}
#endif

#endif // esp32s3_rmt_H