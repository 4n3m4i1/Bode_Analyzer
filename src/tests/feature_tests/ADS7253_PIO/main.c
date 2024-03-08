#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/structs/rosc.h"
#include "hardware/pio.h"
#include "hardware/structs/pio.h"

#include "ads7253_pio.pio.h"

enum PINZ {
    NC,
    CS_PAD,
    SCK_PAD,
    SDO_PAD
};

void main(){
    stdio_init_all();

    busy_wait_ms(1000);

    ADS7253_MainSM_Setup(pio1, 0, CS_PAD, SCK_PAD, SDO_PAD);

    uint16_t pioinstr = 0;
    uint16_t pioaddr = 0;

    while(1){
        if(pio1->sm[0].instr != pioinstr){
            pioinstr = pio1->sm[0].instr;
            pioaddr = pio1->sm[0].addr;
            printf("%2d 0x%04X\n", pioaddr, pioinstr);
    }
        char a = getchar_timeout_us(0);
        switch(a){
            case 'a':{
                pio_sm_put(pio1, 0, 0xAF81);
            }
            break;
        }
        tight_loop_contents();
    }
}
