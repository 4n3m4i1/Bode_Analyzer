#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/structs/rosc.h"


#define NUM_SAMPLES     1024

uint16_t arrrrrr[NUM_SAMPLES];

#define SEED 12345678


#define TAP0        14u
#define TAP1        9u
#define TAP2        5u
#define READ_TAP    11u


bool run_lfsr(uint32_t *elef){
    *elef <<= 1;

    // tap 14 9 5
    *elef |= (((*elef & (1lu << TAP0)) > 0) ^ ((*elef & (1lu << TAP1)) > 0) ^ ((*elef & (1lu << TAP2)) > 0)) ? 1lu : 0;

    //printf("0b");
    //for(int n = 0; n < 32; ++n){
    //    bool bit = *elef & (1lu << (31 - n));
    //    printf("%c", bit + '0');
    //}
    //printf("\n");

    return (*elef & (1lu << READ_TAP)) ? 1 : 0;
}


void main(){
    uint32_t cool_bits = 0xABCDEF12;

    stdio_init_all();
    printf("Start\n");
    gpio_init(0);
    gpio_init(2);
    gpio_set_dir(0, GPIO_OUT);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(0, 0);
    gpio_put(2, 0);

    busy_wait_ms(1000);

/*
  *rosc_ctl =  ROSC_CTRL_FREQ_RANGE_VALUE_HIGH ;// | ROSC_CTRL_ENABLE_VALUE_ENABLE;
  *rosc_freqA = (ROSC_FREQA_PASSWD_VALUE_PASS<<16) | 0xffff ;
  *rosc_freqB = (ROSC_FREQB_PASSWD_VALUE_PASS<<16) | 0xffff ;
*/
    rosc_hw->ctrl   = ROSC_CTRL_FREQ_RANGE_VALUE_HIGH;
    rosc_hw->freqa  = (ROSC_FREQA_PASSWD_VALUE_PASS << 16) | 0xFFFF;
    rosc_hw->freqb  = (ROSC_FREQB_PASSWD_VALUE_PASS << 16) | 0xFFFF;
    
    //rosc_hw->freqa |= (0x09696 << 16) | (7u << 12) | (7u << 8) | (7u << 4) | (7u);

    while(1){
        //for(int n = 0; n < NUM_SAMPLES; ++n){
        //    arrrrrr[n] = n;
        //}
//
        //for(int n = 0; n < NUM_SAMPLES; ++n){
        //    printf("%u\n",arrrrrr[n]);
        //}

        //uint16_t val = rosc_hw->randombit;
        //for(int m = 0; m < NUM_SAMPLES; ++m){
            //for(int n = 0; n < 16; ++n){
            //    uint16_t ral = rosc_hw->randombit;
            //    val |= (ral << n);
            //    //busy_wait_us(1);
            //}
            //gpio_put(0,(val & (1 << 15)));
        //    arrrrrr[m] = val;
        //}
        asm("nop");
 
        gpio_put(0,rosc_hw->randombit);

        //for(int n = 0; n < NUM_SAMPLES; ++n){
        //    printf("%u\n",arrrrrr[n]);
        //}
        
        //printf("DONE\n\n\n\n");

        //uint8_t val2 = run_lfsr(&cool_bits);

        //gpio_put(2,val2);
        //printf("%c\n",val2 + '0');

        //busy_wait_us(10);
        
    }
}