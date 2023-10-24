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

#include "pio_val_2_pin.pio.h"


#define DMA_ROSC_CHANNEL    0
#define DMA_ROSC_PIN        0

int dma_awgn_data_chan, dma_awgn_ctrl_chan;

uint32_t awgn_txfer_ct = 0xFFFFFFFF;


void main(){
    stdio_init_all();

//    gpio_init(0);
//    gpio_init(2);
//    gpio_set_dir(0, GPIO_OUT);
//    gpio_set_dir(2, GPIO_OUT);
//    gpio_put(0, 0);
//    gpio_put(2, 0);
//
    busy_wait_ms(1000);

    

/*
  *rosc_ctl =  ROSC_CTRL_FREQ_RANGE_VALUE_HIGH ;// | ROSC_CTRL_ENABLE_VALUE_ENABLE;
  *rosc_freqA = (ROSC_FREQA_PASSWD_VALUE_PASS<<16) | 0xffff ;
  *rosc_freqB = (ROSC_FREQB_PASSWD_VALUE_PASS<<16) | 0xffff ;
*/
    rosc_hw->ctrl   = ROSC_CTRL_FREQ_RANGE_VALUE_HIGH;
    rosc_hw->freqa  = (ROSC_FREQA_PASSWD_VALUE_PASS << 16) | 0xFFFF;
    rosc_hw->freqb  = (ROSC_FREQB_PASSWD_VALUE_PASS << 16) | 0xFFFF;

    PIO awgn_pio = pio0;
    int awgn_sm = 0;
    uint offset = pio_add_program(awgn_pio, &val2pin_program);
    awgn_pio->sm[awgn_sm].clkdiv = (uint32_t) ((3 << 16));
    val2pin_program_init(awgn_pio, awgn_sm, offset, DMA_ROSC_PIN);


    dma_awgn_ctrl_chan = dma_claim_unused_channel(true);
    dma_awgn_data_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(dma_awgn_ctrl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_chain_to(&c, dma_awgn_data_chan);

    dma_channel_configure(
        dma_awgn_ctrl_chan,
        &c,
        &dma_hw->ch[dma_awgn_data_chan].al3_transfer_count, // Write address (only need to set this once)
        &awgn_txfer_ct,   // Number of DMA transfers the data channel should make
        1,                // Write the same value many times, then halt and interrupt
        false             // Don't start yet
    );


    
    c = dma_channel_get_default_config(dma_awgn_data_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);
    channel_config_set_chain_to(&c, dma_awgn_ctrl_chan);    // trigger ctrl on finish

    dma_channel_configure(
        dma_awgn_data_chan,
        &c,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        NULL,             // Don't provide a read address yet
        0xFFFFFFFF,       // Write the same value many times, then halt and interrupt
        false             // Don't start yet
    );
    
    dma_channel_set_read_addr(dma_awgn_data_chan, &rosc_hw->randombit, true);

    while(1){
        //awgn_pio->txf[0] = rosc_hw->randombit;

        tight_loop_contents();
    }
}
