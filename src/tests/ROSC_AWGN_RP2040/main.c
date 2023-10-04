#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/structs/rosc.h"


#define NUM_SAMPLES     1024

uint16_t arrrrrr[NUM_SAMPLES];



void main(){
    stdio_init_all();
    printf("Start\n");
    gpio_init(0);
    gpio_set_dir(0, GPIO_OUT);
    gpio_put(0, 0);

    busy_wait_ms(1000);

    while(1){
        for(int n = 0; n < NUM_SAMPLES; ++n){
            arrrrrr[n] = n;
        }

        for(int n = 0; n < NUM_SAMPLES; ++n){
            printf("%u\n",arrrrrr[n]);
        }

        uint16_t val = 0;
        for(int m = 0; m < NUM_SAMPLES; ++m){
            for(int n = 0; n < 16; ++n){
                uint16_t ral = rosc_hw->randombit;
                val |= (ral << n);
                busy_wait_us(1);
            }
            gpio_put(0,(val & (1 << 15)));
            arrrrrr[m] = val;
        }
        
        for(int n = 0; n < NUM_SAMPLES; ++n){
            printf("%u\n",arrrrrr[n]);
        }
        
        printf("DONE\n\n\n\n");
        
    }
}