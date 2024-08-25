/**
 * @file no_jerky_platform.c
 * @brief ESP32-S3 platform specific functions for no jerky stepper. Modify these to port the library to other platforms.
 */
#include <freertos/FreeRTOS.h>  // IMPORTANT: make sure CONFIG_FREERTOS_HZ=1000 for correct timing
#include <driver/gpio.h>
#include "no_jerky_platform.h"
#include "esp_check.h"


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
    printf("here\n");

    // create RMT curve encoder
    esp32s3_rmt_encoder_data_t encoder_data = {
        .rmt_channel = output_ch.rmt_channel,
        .rmt_encoder = NULL,
        .data = curve,
        .data_size = curve_size
    };

    esp32s3_rmt_new_stepper_curve_encoder(&encoder_data);
    printf("here2\n");

    rmt_transmit_config_t rmt_tx_config = {.loop_count=0};

    // transmit RMT curve
    ESP_ERROR_CHECK(rmt_transmit(output_ch.rmt_channel,
                                 encoder_data.rmt_encoder,
                                 &encoder_data.data,
                                 2,
                                 &rmt_tx_config));
    printf("here3\n");

    // delete RMT curve encoder - free the allocated memory!!
    encoder_data.rmt_encoder->del(encoder_data.rmt_encoder);
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
