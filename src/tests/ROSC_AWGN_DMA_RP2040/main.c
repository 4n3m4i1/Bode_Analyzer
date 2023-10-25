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

const uint32_t awgn_txfer_ct = 0x0FFFFFFF;


void main(){
    stdio_init_all();

    gpio_init(0);
//    gpio_init(2);
    gpio_set_dir(0, GPIO_OUT);
//    gpio_set_dir(2, GPIO_OUT);
//    gpio_put(0, 0);
//    gpio_put(2, 0);
//
    busy_wait_ms(1000);

    rosc_hw->ctrl   = ROSC_CTRL_FREQ_RANGE_VALUE_HIGH;
    rosc_hw->freqa  = (ROSC_FREQA_PASSWD_VALUE_PASS << 16) | 0xFFFF;
    rosc_hw->freqb  = (ROSC_FREQB_PASSWD_VALUE_PASS << 16) | 0xFFFF;

    PIO awgn_pio = pio0;
    int awgn_sm = 0;
    uint offset = pio_add_program(awgn_pio, &val2pin_program);
    awgn_pio->sm[awgn_sm].clkdiv = (uint32_t) ((15 << 16));
    val2pin_program_init(awgn_pio, awgn_sm, offset, DMA_ROSC_PIN);


    //dma_awgn_ctrl_chan = dma_claim_unused_channel(true);
    //dma_awgn_data_chan = dma_claim_unused_channel(true);

    dma_awgn_ctrl_chan = 1;
    dma_awgn_data_chan = 0;

    dma_channel_config c = dma_channel_get_default_config(dma_awgn_ctrl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_chain_to(&c, dma_awgn_data_chan);

    dma_channel_configure(
        dma_awgn_ctrl_chan,
        &c,
        // Write to data channel transfer count to refill it
        ((volatile uint32_t *)(0x50000000 + 0x01C)),
        //&dma_hw->ch[dma_awgn_data_chan].al2_transfer_count,
        &awgn_txfer_ct,   // Number of DMA transfers the data channel should make
        1,                // Write the transfer count once to the data channel
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
        &pio0_hw->txf[0],       // Write to PIO0 TX FIFO to put randombit on pin
        &rosc_hw->randombit,    // Read random bit from ROSC
        0,                      // Wait for ctrl channel to fill tx count
        false                   // Don't start yet
    );
    
    dma_start_channel_mask(1 << dma_awgn_ctrl_chan);

    while(1){
        //if(dma_hw->ch[dma_awgn_data_chan].al2_transfer_count == 0) {
        //    printf("Refill! @ 0x%08X\n", &(dma_hw->ch[dma_awgn_data_chan].al2_transfer_count));
        //    *((volatile uint32_t *)(0x50000000 + 0x01C)) = awgn_txfer_ct;
           // dma_hw->ch[dma_awgn_data_chan].al2_transfer_count = awgn_txfer_ct;
        //}
        busy_wait_ms(10);
        tight_loop_contents();
    }
}
