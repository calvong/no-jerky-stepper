#include "no_jerky_stepper.h"


no_jerky_stepper_t create_a_not_jerky_stepper(no_jerky_motor_pins_t motor_pins, uint8_t motor_id, const char* motor_group)
{
    no_jerky_stepper_t stepper;
    stepper.output_ch = no_jerky_init(motor_pins);
    stepper.pins = motor_pins;
    stepper.motor_id = motor_id;
    stepper.motor_group = motor_group;
    
    stepper.output_not_jerky_motion_curve = &output_not_jerky_motion_curve;
    return stepper;
}