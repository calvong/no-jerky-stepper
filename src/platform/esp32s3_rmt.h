#ifndef ESP32S3_RMT_H
#define ESP32S3_RMT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/rmt_tx.h>


typedef struct esp32s3_rmt {
    rmt_channel_handle_t* rmt_channels;
    rmt_sync_manager_handle_t rmt_sync_manager;
} esp32s3_rmt_t;


typedef struct esp32s3_rmt_encoder_data {
    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t rmt_encoder;
    uint32_t* data;         
    uint32_t data_size;     
} esp32s3_rmt_encoder_data_t;


// public functions
esp32s3_rmt_t esp32s3_rmt_init(uint8_t n_motors, uint8_t step_pins[]);
esp_err_t esp32s3_rmt_new_stepper_curve_encoder(esp32s3_rmt_encoder_data_t *encoder_data);
void esp32s3_rmt_channels_sync_reset(esp32s3_rmt_t esp32s3_rmt);

// static functions
static size_t esp32s3_rmt_encode_stepper_curve(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
static esp_err_t esp32s3_rmt_del_stepper_curve_encoder(rmt_encoder_t *encoder);
static esp_err_t esp32s3_rmt_reset_stepper_curve_encoder(rmt_encoder_t *encoder);
static void esp32s3_rmt_tx_done_callback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_data);

#ifdef __cplusplus
}
#endif

#endif // esp32s3_rmt_H