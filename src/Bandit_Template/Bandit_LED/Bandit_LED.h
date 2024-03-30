#ifndef BANDIT_LED_h
#define BANDIT_LED_h

#include "Bandit_Pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

void init_LED_pins();
void set_RGB_levels(uint8_t R_, uint8_t G_, uint8_t B_);
void set_ULED_level(uint8_t _L);

#endif