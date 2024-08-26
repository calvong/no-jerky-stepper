/**
 * @file no_jerky_platform.c
 * @brief ESP32-S3 platform specific functions for no jerky stepper. Modify these to port the library to other platforms.
 */
#include <freertos/FreeRTOS.h>  // IMPORTANT: make sure CONFIG_FREERTOS_HZ=1000 for correct timing
#include <driver/gpio.h>
#include <esp_check.h>
#include <string.h>

#include "no_jerky_platform.h"


no_jerky_output_t no_jerky_init(no_jerky_motor_pins_t motor_pins)
{
    no_jerky_output_t output_ch;

    // configure motor pins
    gpio_set_direction((gpio_num_t) motor_pins.dir, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t) motor_pins.step, GPIO_MODE_OUTPUT);

    gpio_set_level((gpio_num_t) motor_pins.dir, 1);

    // configure ESP32-S3 RMT channels
    output_ch.rmt_channel = esp32s3_rmt_init(motor_pins.step);

    return output_ch;
}


void output_not_jerky_motion_curve(no_jerky_output_t output_ch, uint32_t *curve, uint32_t curve_size)
{
    // create RMT curve encoder
    esp32s3_rmt_encoder_data_t encoder_data = {
        .rmt_channel = output_ch.rmt_channel,
        .rmt_encoder = NULL,
        .symbols = NULL,
        .n_symbols = 0
    };

    rmt_transmit_config_t rmt_tx_config = {.loop_count=0};

    rmt_symbol_word_t* whole_curve_symbols = NULL;
    uint32_t whole_curve_symbol_size = 0;
    // convert curve data into RMT symbol format
    esp32s3_stepper_curve_to_rmt_symbol(curve, curve_size, &whole_curve_symbols, &whole_curve_symbol_size);

    printf("whole_curve_symbol_size: %ld\n", whole_curve_symbol_size);
    printf("whole_curve_symbols[0].duration0: %d\n", whole_curve_symbols[0].duration0);
    if (whole_curve_symbol_size > 48)   // 48 = max memory block size (per channel) for RMT encoder
    {
        // divde the curve into 48 symbol chunks - should not be more than uint8_t!?
        uint8_t n_chunks = whole_curve_symbol_size / 48;
        uint8_t remainder_chunk = whole_curve_symbol_size % 48;

        printf("n_chunks: %d, reminder: %d\n", n_chunks, remainder_chunk);

        // allocate memory for the symbols
        encoder_data.symbols = (rmt_symbol_word_t*) malloc(48 * sizeof(rmt_symbol_word_t));

        // encode and transmit the first n_chunks
        for (uint8_t i = 0; i < n_chunks; i++)
        {
            // set the RMT encoder data
            encoder_data.n_symbols = 48;

            // copy the curve symbols
            memcpy(encoder_data.symbols, &whole_curve_symbols[i * 48], 48 * sizeof(rmt_symbol_word_t));

            printf("copied memory for chunk id: %d\n", i);

            // create new RMT curve encoder
            ESP_ERROR_CHECK(esp32s3_rmt_new_stepper_curve_encoder(&encoder_data));
            printf("chunk id: %d encoded\n", i);

            // transmit RMT curve
            ESP_ERROR_CHECK(rmt_transmit(output_ch.rmt_channel,
                                        encoder_data.rmt_encoder,
                                        curve,                      // dummy data
                                        curve_size,                 // dummy size
                                        &rmt_tx_config));

            printf("chunk id: %d transmitted\n", i);

            // delete RMT curve encoder - free the allocated memory!!
            encoder_data.rmt_encoder->del(encoder_data.rmt_encoder);
            // free(encoder_data.symbols);

            // encoder_data.symbols = NULL;
            // encoder_data.rmt_encoder = NULL;
        }

        // free the allocated memory
        free(encoder_data.symbols);

        // encode and transmit the remainder_chunk
        if (remainder_chunk > 0)
        {
            // allocate memory for the symbols
            encoder_data.symbols = (rmt_symbol_word_t*) malloc(remainder_chunk * sizeof(rmt_symbol_word_t));

            // set the RMT encoder data
            encoder_data.n_symbols = remainder_chunk;
            encoder_data.symbols = &whole_curve_symbols[n_chunks * 48];

            // create new RMT curve encoder
            ESP_ERROR_CHECK(esp32s3_rmt_new_stepper_curve_encoder(&encoder_data));

            // transmit RMT curve
            ESP_ERROR_CHECK(rmt_transmit(output_ch.rmt_channel,
                                        encoder_data.rmt_encoder,
                                        curve,                      // dummy data
                                        curve_size,                 // dummy size
                                        &rmt_tx_config));

            // delete RMT curve encoder - free the allocated memory!!
            encoder_data.rmt_encoder->del(encoder_data.rmt_encoder);
        }

        // free the allocated memory
        free(encoder_data.symbols);
    }
    else
    {
        // allocate memory for the symbols
        encoder_data.symbols = (rmt_symbol_word_t*) malloc(whole_curve_symbol_size * sizeof(rmt_symbol_word_t));

        // set the RMT encoder data
        encoder_data.n_symbols = whole_curve_symbol_size;
        encoder_data.symbols = whole_curve_symbols;

        // create new RMT curve encoder
        ESP_ERROR_CHECK(esp32s3_rmt_new_stepper_curve_encoder(&encoder_data));

        // transmit RMT curve
        ESP_ERROR_CHECK(rmt_transmit(output_ch.rmt_channel,
                                    encoder_data.rmt_encoder,
                                    curve,                      // dummy data
                                    curve_size,                 // dummy size
                                    &rmt_tx_config));

        // delete RMT curve encoder - free the allocated memory!!
        encoder_data.rmt_encoder->del(encoder_data.rmt_encoder);

        // free the allocated memory
        free(encoder_data.symbols);
    }

    // free the allocated memory
    free(whole_curve_symbols);
}


void wait_for_motor_motion_done(no_jerky_output_t output_ch)
{
    rmt_tx_wait_all_done(output_ch.rmt_channel, -1);
}

// TODO
// void resync_no_jerky_group_output(no_jerky_output_t output_ch)
// {
//     ESP_ERROR_CHECK(rmt_sync_reset(output_ch.esp32s3_rmt.rmt_sync_manager));
// }


void no_jerky_delay_ms(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

