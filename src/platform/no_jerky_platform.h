/**
 * @file no_jerky_platform.h
 * @brief ESP32-S3 platform specific functions for no jerky stepper. Modify these to port the library to other platforms.
 */
#ifndef NO_JERKY_PLATFORM_H
#define NO_JERKY_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp32s3_rmt.h"
#include <esp_async_memcpy.h>


typedef struct no_jerky_motor_pins
{
    uint8_t dir;
    uint8_t step;
    uint8_t enable;
} no_jerky_motor_pins_t;


typedef struct no_jerky_output
{
    // platform specific PWM/motor output peripheral
    rmt_channel_handle_t rmt_channel;
} no_jerky_output_t;


no_jerky_output_t no_jerky_init(no_jerky_motor_pins_t motor_pins);
void output_not_jerky_motion_curve(no_jerky_output_t output_ch, uint32_t *curve, uint32_t curve_size);
void wait_for_motor_motion_done(no_jerky_output_t output_ch);

void no_jerky_delay_ms(uint16_t ms);


#ifdef __cplusplus
}
#endif

#endif  // NO_JERKY_PLATFORM_H 