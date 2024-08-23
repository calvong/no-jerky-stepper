/**
 * @file no_jerky_platform.c
 * @brief ESP32-S3 platform specific functions for no jerky stepper. Modify these to port the library to other platforms.
 */
#include <freertos/FreeRTOS.h>  // IMPORTANT: make sure CONFIG_FREERTOS_HZ=1000 for correct timing
#include <driver/gpio.h>
#include "no_jerky_platform.h"
#include "esp_check.h"


no_jerky_platform_data_t no_jerky_platform_init(no_jerky_motor_pins_t motor_pins[])
{
    no_jerky_platform_data_t platform_data;

    // check number of motors/channels
    uint8_t n_motors = sizeof(motor_pins) / sizeof(motor_pins[0]);
    platform_data.n_motors = n_motors;

    // configure motor pins
    for (uint8_t i = 0; i < n_motors; i++)
    {
        // Cannot image number of motors being more than 255!?
        gpio_set_direction((gpio_num_t) motor_pins[i].dir_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t) motor_pins[i].step_pin, GPIO_MODE_OUTPUT);

        gpio_set_level((gpio_num_t) motor_pins[i].dir_pin, 1);
    }

    platform_data.motor_pins = &motor_pins[0];

    // configure ESP32-S3 RMT channels
    uint8_t step_pins[n_motors];
    for (uint8_t i = 0; i < n_motors; i++)
    {
        step_pins[i] = motor_pins[i].step_pin;
    }
    
    platform_data.esp32s3_rmt = esp32s3_rmt_init(n_motors, step_pins);

    return platform_data;
}


void send_not_jerky_stepper_motor_curve(no_jerky_platform_data_t platform_data, uint32_t *curve, uint32_t curve_size, uint8_t motor_id)
{
    // create RMT curve encoder
    esp32s3_rmt_encoder_data_t encoder_data = {
        .rmt_channel = platform_data.esp32s3_rmt.rmt_channels[motor_id],
        .rmt_encoder = NULL,
        .data = curve,
        .data_size = curve_size
    };

    esp32s3_rmt_new_stepper_curve_encoder(&encoder_data);
    printf("ID: %d, encoded\n", motor_id);

    rmt_transmit_config_t rmt_tx_config = {.loop_count=0};

    // transmit RMT curve
    ESP_ERROR_CHECK(rmt_transmit(platform_data.esp32s3_rmt.rmt_channels[motor_id],
                                 encoder_data.rmt_encoder,
                                 &encoder_data.data,
                                 encoder_data.data_size,
                                 &rmt_tx_config));
    printf("ID: %d, transmitted\n", motor_id);

    // delete RMT curve encoder - free the allocated memory!!
    encoder_data.rmt_encoder->del(encoder_data.rmt_encoder);
}


void no_jerky_delay_ms(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
