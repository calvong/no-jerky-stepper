#include <stdint.h>
#include <esp_log.h>

#include "esp32s3_rmt.h"


typedef struct esp32s3_rmt_curve_encoder {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder;
    uint32_t data_size;
    rmt_symbol_word_t* curve;
} esp32s3_rmt_curve_encoder_t;


rmt_channel_handle_t esp32s3_rmt_init(uint8_t step_pin)
{
    rmt_channel_handle_t rmt_channel;

    // configure ESP32-S3 RMT channels
    rmt_tx_channel_config_t rmt_tx_config = {
        .gpio_num = (gpio_num_t) step_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 140,    // memory block size, n * 4 = 4n Bytes 
        .trans_queue_depth = 1,    // number of transactions that can be queued in the background
        .resolution_hz = 1000000,   // 1 MHz resolution
        .flags.invert_out = false,  // output signal is not inverted
        // .flags.with_dma = true,     // use DMA - limited to only 1 channel if using DMA!!!
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_tx_config, &rmt_channel));   

    // enable RMT channels
    ESP_ERROR_CHECK(rmt_enable(rmt_channel));
    
    // TODO: register RMT callback - this should be in IRAM. See API reference footer 2

    return rmt_channel;
}



esp_err_t esp32s3_rmt_new_stepper_curve_encoder(esp32s3_rmt_encoder_data_t *encoder_data)
{
    esp_err_t ret = ESP_OK;

    esp32s3_rmt_curve_encoder_t* step_encoder = NULL;

    // --- convert user data into RMT symbol format ---
    // allocate memory for the curve, start with the input data size
    uint32_t allocated_curve_memory = encoder_data->data_size;
    rmt_symbol_word_t* curve_symbol_word = (rmt_symbol_word_t*) malloc(allocated_curve_memory * sizeof(rmt_symbol_word_t));

    uint32_t curve_size = 0;
    for (uint32_t i = 0; i < encoder_data->data_size; i++)
    {
        // check if need to reallocate memory for the curve
        increase_allocated_curve_memory_check(curve_size, &allocated_curve_memory, curve_symbol_word);

        // check if the duration exceeds 0x8000 (2^15 not uint16_t!!)
        uint8_t n_shifts = 1;
        if (encoder_data->data[i] > (uint32_t) 0x8000)
        {

            uint32_t big_symbol = 0;
            // maximum divide by 2^10 = 1024
            for (uint8_t k = 1; k < 10; k++)
            {
                big_symbol = encoder_data->data[i] >> (k + 1);

                if (big_symbol <= (uint32_t) 0x8000)
                {
                    n_shifts = k;
                    break;
                }
            }
            // split the duration into multiple symbols
            uint16_t symbol_duration = (uint16_t) big_symbol;
            printf("big symbol: %ld, duration=%d, OG=%ld, nshifts=%d, i=%ld, curve_idx=%ld\n", big_symbol,symbol_duration, encoder_data->data[i], n_shifts, i, curve_size);

            for (uint8_t j = 0; j < 1 << (n_shifts - 1); j++)
            {   
                increase_allocated_curve_memory_check(curve_size, &allocated_curve_memory, curve_symbol_word);

                curve_symbol_word[curve_size].level0 = 1;
                curve_symbol_word[curve_size].duration0 = symbol_duration;
                curve_symbol_word[curve_size].level1 = 1;
                curve_symbol_word[curve_size].duration1 = symbol_duration;
         
                curve_size++;
            }
            for (uint8_t j = 0; j < 1 << (n_shifts - 1); j++)
            {
                increase_allocated_curve_memory_check(curve_size, &allocated_curve_memory, curve_symbol_word);

                curve_symbol_word[curve_size].level0 = 0;
                curve_symbol_word[curve_size].duration0 = symbol_duration;
                curve_symbol_word[curve_size].level1 = 0;
                curve_symbol_word[curve_size].duration1 = symbol_duration;

                curve_size++;
            }
        }
        else
        {
            increase_allocated_curve_memory_check(curve_size, &allocated_curve_memory, curve_symbol_word);

            uint16_t symbol_duration = (uint16_t) encoder_data->data[i] >> 1; // divide timestep by 2 to create one step pulse
            // FIXME: for duration that exceeds 0xFFFF, split the duration into multiple symbols
            curve_symbol_word[curve_size].level0 = 1;
            curve_symbol_word[curve_size].duration0 = symbol_duration;
            curve_symbol_word[curve_size].level1 = 0;
            curve_symbol_word[curve_size].duration1 = symbol_duration;

            curve_size++;
        }
    }

    // allocate memory for the encoder
    step_encoder = rmt_alloc_encoder_mem(sizeof(esp32s3_rmt_curve_encoder_t) + curve_size * sizeof(rmt_symbol_word_t));
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

    // set the RMT encoder data
    step_encoder->base.del = esp32s3_rmt_del_stepper_curve_encoder;
    step_encoder->base.reset = esp32s3_rmt_reset_stepper_curve_encoder;
    step_encoder->base.encode = esp32s3_rmt_encode_stepper_curve;

    encoder_data->rmt_encoder = &(step_encoder->base);
    step_encoder->data_size = curve_size;
    
    // shrink the curve memory to the actual size
    curve_symbol_word = (rmt_symbol_word_t*) realloc(curve_symbol_word, curve_size * sizeof(rmt_symbol_word_t));
    
    step_encoder->curve = curve_symbol_word;

    printf("Curve size: %ld\n", curve_size);

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
    free(stepper_encoder->curve);
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


static void increase_allocated_curve_memory_check(uint32_t current_size, uint32_t* current_max_size, rmt_symbol_word_t *curve)
{
    if (current_size >= *current_max_size)
    {
        curve = (rmt_symbol_word_t*) realloc(curve, 2 * (*current_max_size) * sizeof(rmt_symbol_word_t));

        *current_max_size = 2 * (*current_max_size);
    }
}