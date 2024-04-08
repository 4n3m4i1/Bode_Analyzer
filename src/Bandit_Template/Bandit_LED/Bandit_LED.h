#ifndef BANDIT_LED_h
#define BANDIT_LED_h

#include "Bandit_Pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define BANDIT_PINK_SUPER_ARGUMENT      Bandit_RGBU.R = 236, Bandit_RGBU.G = 10, Bandit_RGBU.B = 50
#define USER_LED_BANDIT_RDY             127
#define USER_LED_BANDIT_DC_CAL_DONE     32

void init_LED_pins();
void set_RGB_levels(uint8_t R_, uint8_t G_, uint8_t B_);
void set_ULED_level(uint8_t _L);

#endif