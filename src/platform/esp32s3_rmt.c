#include <stdint.h>
#include <esp_log.h>

#include "esp32s3_rmt.h"


typedef struct esp32s3_rmt_curve_encoder {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder;
    uint32_t data_size;
    rmt_symbol_word_t curve[];
} esp32s3_rmt_curve_encoder_t;


esp32s3_rmt_t esp32s3_rmt_init(uint8_t n_motors, uint8_t step_pins[])
{
    esp32s3_rmt_t rmt_data;

    // configure ESP32-S3 RMT channels
    rmt_data.rmt_channels = (rmt_channel_handle_t*) malloc(n_motors * sizeof(rmt_channel_handle_t)); 
    // ^ Not neccessary to free this memory as it is only allocated once and used throughout the program

    for (uint8_t i=0; i < n_motors; i++)
    {
        rmt_tx_channel_config_t rmt_tx_config = {
            .gpio_num = (gpio_num_t) step_pins[i],
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .mem_block_symbols = 128,    // memory block size, n * 4 = 4n Bytes 
            .trans_queue_depth = 10,    // number of transactions that can be queued in the background
            .resolution_hz = 1000000,   // 1 MHz resolution
            .flags.invert_out = false,  // output signal is not inverted
            // .flags.with_dma = true,     // use DMA - limited to only 1 channel if using DMA!!!
        };

        ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_tx_config, &rmt_data.rmt_channels[i]));   

        // enable RMT channels
        ESP_ERROR_CHECK(rmt_enable(rmt_data.rmt_channels[i]));
    }

    // install RMT sync manager
    // > RMT transmission does not start until rmt_transmit() is called for ALL channels
    // > Different channels may finish transmission at different times, call rmt_sync_reset() before starting new transmissions
    // reference:
    // https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/api-reference/peripherals/rmt.html#rmt-multiple-channels-simultaneous-transmission
    rmt_sync_manager_config_t sync_config = {
        .tx_channel_array = rmt_data.rmt_channels,
        .array_size = (size_t) n_motors,
    };

    ESP_ERROR_CHECK(rmt_new_sync_manager(&sync_config, &rmt_data.rmt_sync_manager));

    // register RMT callback
    for (uint8_t i = 0; i < n_motors; i++)
    {
        // ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(rmt_data.rmt_channels[i], esp32s3_rmt_tx_done_callback, NULL));
    }

    return rmt_data;
}


void esp32s3_rmt_channels_sync_reset(esp32s3_rmt_t esp32s3_rmt)
{
    ESP_ERROR_CHECK(rmt_sync_reset(esp32s3_rmt.rmt_sync_manager));
}


esp_err_t esp32s3_rmt_new_stepper_curve_encoder(esp32s3_rmt_encoder_data_t *encoder_data)
{
    esp_err_t ret = ESP_OK;

    esp32s3_rmt_curve_encoder_t* step_encoder = NULL;

    // allocate memory for the encoder
    step_encoder = rmt_alloc_encoder_mem(sizeof(esp32s3_rmt_curve_encoder_t) + encoder_data->data_size * sizeof(rmt_symbol_word_t));
    if (step_encoder == NULL)
    {
        printf("Failed to allocate memory for the RMT encoder\n");
    }

    // create new copy encoder
    rmt_copy_encoder_config_t copy_encoder_config = {};
    if (rmt_new_copy_encoder(&copy_encoder_config, &step_encoder->copy_encoder) != ESP_OK)
    {
        printf("Failed to create new RMT copy encoder\n");
        return ESP_FAIL;
    }

    // convert user data into RMT symbol format
    for (uint32_t i = 0; i < encoder_data->data_size; i++)
    {
        if (i==0)
        {
            printf("Data: %ld, half: %ld\n", encoder_data->data[i], encoder_data->data[i] /2);
        }
        uint16_t symbol_duration = (uint16_t) encoder_data->data[i] / 2; // divide timestep by 2 to create one step pulse
        // FIXME: for duration that exceeds 0xFFFF, split the duration into multiple symbols
        step_encoder->curve[i].level0 = 0;
        step_encoder->curve[i].duration0 = symbol_duration;
        step_encoder->curve[i].level1 = 1;
        step_encoder->curve[i].duration1 = symbol_duration;
    }

    // set the RMT encoder data
    step_encoder->base.del = esp32s3_rmt_del_stepper_curve_encoder;
    step_encoder->base.reset = esp32s3_rmt_reset_stepper_curve_encoder;
    step_encoder->base.encode = esp32s3_rmt_encode_stepper_curve;

    encoder_data->rmt_encoder = &(step_encoder->base);
    step_encoder->data_size = encoder_data->data_size;

    return ret;
}


/**
 * @brief Encode the user data into RMT symbols
 * 
 * @param primary_data not using this parameter to pass the user data from rmt_transmit().
 *                     Instead, using esp32s3_rmt_curve_encoder_t *stepper_encoder = __containerof(encoder, esp32s3_rmt_curve_encoder_t, base);
 *                     to access the user data stored in esp32s3_rmt_curve_encoder_t
 */
static size_t esp32s3_rmt_encode_stepper_curve(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{   
    esp32s3_rmt_curve_encoder_t *stepper_encoder = __containerof(encoder, esp32s3_rmt_curve_encoder_t, base);
    rmt_encoder_handle_t copy_encoder = stepper_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    printf("curve %d, data size %ld\n", stepper_encoder->curve[0].duration0, stepper_encoder->data_size);
    encoded_symbols = copy_encoder->encode(copy_encoder,
                                           channel,
                                           stepper_encoder->curve,
                                           stepper_encoder->data_size * sizeof(rmt_symbol_word_t),
                                           &session_state);

    *ret_state = session_state;
    printf("Encoded symbols: %d\n", encoded_symbols);
    return encoded_symbols;
}


static esp_err_t esp32s3_rmt_del_stepper_curve_encoder(rmt_encoder_t *encoder)
{
    esp32s3_rmt_curve_encoder_t *stepper_encoder = __containerof(encoder, esp32s3_rmt_curve_encoder_t, base);
    rmt_del_encoder(stepper_encoder->copy_encoder);
    free(stepper_encoder);
    return ESP_OK;
}


static esp_err_t esp32s3_rmt_reset_stepper_curve_encoder(rmt_encoder_t *encoder)
{
    esp32s3_rmt_curve_encoder_t *stepper_encoder = __containerof(encoder, esp32s3_rmt_curve_encoder_t, base);
    rmt_encoder_reset(stepper_encoder->copy_encoder);
    return ESP_OK;
}


static void esp32s3_rmt_tx_done_callback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_data)
{
    // TODO

    printf("RMT channel transmission done naniiii\n");
}