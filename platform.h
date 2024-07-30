#ifndef NO_JERKY_PLATFORM_H
#define NO_JERKY_PLATFORM_H

#include <cstdint>

void no_jerky_step();
void no_jerky_dir(uint8_t dir);
void no_jerky_delay_ms(uint16_t ms);
void no_jerky_isr();

#endif 