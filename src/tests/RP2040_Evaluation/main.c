#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"


#include "core_0_exec.h"
#include "core_1_exec.h"

#include "./trig_tables/Q15_sin_table.h"
#include "./trig_tables/Q15_cos_table.h"

///////////////////// MAIN //////////////////////////////
int main(){
    stdio_init_all();

    // Setup for peripherals done on core 0



    multicore_launch_core1(core_1_main);
    core_0_main();

    // We should NEVER be here!!!
    while(1){
        tight_loop_contents();
    }

}




////////////////////// FUNctions //////////////////////////
