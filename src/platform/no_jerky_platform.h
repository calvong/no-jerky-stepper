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


typedef struct no_jerky_motor_pins
{
    uint8_t dir_pin;
    uint8_t step_pin;
    // enable motors at the higher level
} no_jerky_motor_pins_t;


typedef struct no_jerky_platform_data
{
    // allows multi-channel/multi-motor control
    // generic platform data
    no_jerky_motor_pins_t* motor_pins;
    uint8_t n_motors;

    // platform specific data
    esp32s3_rmt_t esp32s3_rmt;

} no_jerky_platform_data_t;


no_jerky_platform_data_t no_jerky_platform_init(no_jerky_motor_pins_t motor_pins[]);

void send_not_jerky_stepper_motor_curve(no_jerky_platform_data_t platform_data, uint32_t *curve, uint32_t curve_size, uint8_t motor_id);
void wait_for_motor_motion_done(no_jerky_platform_data_t platform_data);
void no_jerky_delay_ms(uint16_t ms);


#ifdef __cplusplus
}
#endif

#endif  // NO_JERKY_PLATFORM_H 