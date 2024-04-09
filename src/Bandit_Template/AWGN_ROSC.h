#ifndef AWGN_ROSC_h
#define AWGN_ROSC_h

#include "hardware/dma.h"
#include "hardware/structs/rosc.h"
#include "hardware/pio.h"
#include "hardware/structs/pio.h"

#include "Bandit_Pins.h"

#include "pio_val_2_pin.pio.h"


#define DMA_ROSC_CHANNEL    0
#define DMA_ROSC_PIN        0

#define WGN_PROP_TIME_US    2500

int dma_awgn_data_chan, dma_awgn_ctrl_chan;

const uint32_t awgn_txfer_ct = 0x0FFFFFFF;

void setup_rosc_full_tilt(){
    rosc_hw->ctrl   = ROSC_CTRL_FREQ_RANGE_VALUE_HIGH;
    rosc_hw->freqa  = (ROSC_FREQA_PASSWD_VALUE_PASS << 16) | 0xFFFF;
    rosc_hw->freqb  = (ROSC_FREQB_PASSWD_VALUE_PASS << 16) | 0xFFFF;
}

void setup_PIO_for_switching(){
    PIO awgn_pio = pio0;
    int awgn_sm = 0;
    uint offset = pio_add_program(awgn_pio, &val2pin_program);
    awgn_pio->sm[awgn_sm].clkdiv = (uint32_t) ((15 << 16));
    val2pin_program_init(awgn_pio, awgn_sm, offset, PDM_WGN_PAD);
}

void setup_chained_dma_channels(){
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
        // Write to data channel 0 transfer count to refill it
        ((volatile uint32_t *)(DMA_BASE + 0x1C)),
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
}

void start_randombit_dma_chain(int dma_channel){
    dma_start_channel_mask(1 << dma_channel);
}

void start_wgn_dma_channels(){
    dma_hw->ch[dma_awgn_ctrl_chan].ctrl_trig |= (1u);
    dma_hw->ch[dma_awgn_data_chan].ctrl_trig |= (1u);
}

void stop_randombit_dma_chain(int dma_channel){
    dma_channel_abort(dma_channel);
}

void stop_wgn_dma_channels(){
    dma_hw->ch[dma_awgn_ctrl_chan].ctrl_trig &= ~(1u);
    dma_hw->ch[dma_awgn_data_chan].ctrl_trig &= ~(1u);
    pio0->txf[0] = 0;
}

#endif