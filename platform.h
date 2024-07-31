#ifndef NO_JERKY_PLATFORM_H
#define NO_JERKY_PLATFORM_H

#include <stdint.h>

typedef struct no_jerky_motor_pins
{
    uint8_t dir_pin;
    uint8_t step_pin;
}no_jerky_motor_pins_t;

void no_jerky_step(no_jerky_motor_pins_t motor_pins, uint8_t dir);
void no_jerky_delay_ms(uint16_t ms);
void no_jerky_isr();

#endif 