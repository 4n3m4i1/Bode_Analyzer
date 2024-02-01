#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "Memory_Management.h"


#include "core_0_exec.h"
#include "core_1_exec.h"

CORE_0_MEM int abc0[2048] = {0};
CORE_1_MEM int abc1[2048] = {0};

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
