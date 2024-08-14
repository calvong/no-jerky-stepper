#include "platform.h"

// platform specific includes
// this platform: ESP32
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>  // IMPORTANT: make sure CONFIG_FREERTOS_HZ=1000 for correct timing

void no_jerky_step(no_jerky_motor_pins_t motor_pins, uint8_t dir)
{
    gpio_set_level(motor_pins.dir_pin, dir);
    gpio_set_level(motor_pins.step_pin, 1);
    gpio_set_level(motor_pins.step_pin, 0);

    // handle delay/sleep elsewhere
}

