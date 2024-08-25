#ifndef NO_JERKY_STEPPER_H
#define NO_JERKY_STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "no_jerky_platform.h"

typedef struct no_jerky_stepper
{
    no_jerky_output_t output_ch;    // motor output channel
    no_jerky_motor_pins_t pins;     // motor pins
    uint8_t motor_id;               // motor ID
    const char* motor_group;        // motor group name, if any. Otherwise, NULL

    // functions
    void (*output_not_jerky_motion_curve)(no_jerky_output_t, uint32_t*, uint32_t);

} no_jerky_stepper_t;


no_jerky_stepper_t create_a_not_jerky_stepper(no_jerky_motor_pins_t motor_pins, uint8_t motor_id, const char* motor_group);


#ifdef __cplusplus
}
#endif

#endif
