// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ---------------------- //
// ads7253_write_data_pio //
// ---------------------- //

#define ads7253_write_data_pio_wrap_target 0
#define ads7253_write_data_pio_wrap 5

#define ads7253_write_data_pio_offset_entry_point 3u

static const uint16_t ads7253_write_data_pio_program_instructions[] = {
            //     .wrap_target
    0x7101, //  0: out    pins, 1         side 2 [1] 
    0xa042, //  1: nop                    side 0     
    0x0040, //  2: jmp    x--, 0          side 0     
    0x88e0, //  3: pull   ifempty block   side 1     
    0xa022, //  4: mov    x, y            side 0     
    0xc007, //  5: irq    nowait 7        side 0     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program ads7253_write_data_pio_program = {
    .instructions = ads7253_write_data_pio_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config ads7253_write_data_pio_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ads7253_write_data_pio_wrap_target, offset + ads7253_write_data_pio_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}
#endif

// --------------------- //
// ads7253_read_data_pio //
// --------------------- //

#define ads7253_read_data_pio_wrap_target 0
#define ads7253_read_data_pio_wrap 4

static const uint16_t ads7253_read_data_pio_program_instructions[] = {
            //     .wrap_target
    0x21c7, //  0: wait   1 irq, 7               [1] 
    0x4201, //  1: in     pins, 1                [2] 
    0x0041, //  2: jmp    x--, 1                     
    0xa022, //  3: mov    x, y                       
    0xc047, //  4: irq    clear 7                    
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program ads7253_read_data_pio_program = {
    .instructions = ads7253_read_data_pio_program_instructions,
    .length = 5,
    .origin = -1,
};

static inline pio_sm_config ads7253_read_data_pio_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ads7253_read_data_pio_wrap_target, offset + ads7253_read_data_pio_wrap);
    return c;
}

static inline void ADS7253_PIO_Setup(PIO pio, bool DUALSDO, uint CSPIN, uint SCKPIN, uint SDOPIN, uint SDIPIN_A, uint SDIPIN_B){
    const unsigned int MAINSPI_SM = 0;
    const unsigned int DIN_A_SM = 1;
    const unsigned int DIN_B_SM = 2;
    const unsigned int mainsmoffset = 0;
    const unsigned int datasmoffset = 16;
    unsigned int transfer_size;
    if(DUALSDO){        // setup for dual sdo operation
        transfer_size = 16;
    } else {            // setup for single sdo operation
        transfer_size = 32;
    }   
    pio_sm_config mainsm = ads7253_write_data_pio_program_get_default_config(mainsmoffset);
    sm_config_set_sideset_pins(&mainsm, CSPIN);  // CSN and SCK pins as sideset
    sm_config_set_out_pins(&mainsm, SDOPIN, 1);
    hw_set_bits(&pio->input_sync_bypass, 1u << SDIPIN_A);
    if(DUALSDO) hw_set_bits(&pio->input_sync_bypass, 1u << SDIPIN_B);
    //pio_sm_init(pio, MAINSPI_SM, ads7253_write_data_pio_offset_entry_point, &mainsm);
    // Load main pacing state machine with number of bits to transfer
    pio_sm_exec(pio, MAINSPI_SM, pio_encode_set(pio_x, transfer_size - 2));
    pio_sm_exec(pio, MAINSPI_SM, pio_encode_set(pio_y, transfer_size - 2));
    
    pio_sm_set_pins_with_mask(pio, MAINSPI_SM, (1u << CSPIN), (3u << CSPIN) | (1u << SDOPIN));
    pio_sm_set_pindirs_with_mask(pio, MAINSPI_SM, (1u << CSPIN) | (1u << SCKPIN) | (1 << SDOPIN),
                                                    (1u << CSPIN) | (1u << SCKPIN) | (1 << SDOPIN));
    pio_sm_set_pindirs_with_mask(pio, DIN_A_SM, 0, (1u << SDIPIN_A));
    pio_sm_set_pindirs_with_mask(pio, DIN_B_SM, 0, (1u << SDIPIN_B));

    pio_sm_config datasm_A = ads7253_read_data_pio_program_get_default_config(datasmoffset);
    sm_config_set_in_pins(&datasm_A, SDIPIN_A);
    pio_sm_config datasm_B = ads7253_read_data_pio_program_get_default_config(datasmoffset);
    sm_config_set_in_pins(&datasm_B, SDIPIN_B);
    sm_config_set_out_shift(&mainsm, false, true, transfer_size);
    sm_config_set_in_shift(&datasm_A, false, true, transfer_size);
    sm_config_set_in_shift(&datasm_B, false, true, transfer_size);
    pio_sm_exec(pio, DIN_A_SM, pio_encode_set(pio_x, transfer_size - 2));
    pio_sm_exec(pio, DIN_A_SM, pio_encode_set(pio_y, transfer_size - 2));
    pio_sm_exec(pio, DIN_B_SM, pio_encode_set(pio_x, transfer_size - 2));
    pio_sm_exec(pio, DIN_B_SM, pio_encode_set(pio_y, transfer_size - 2));

    pio_gpio_init(pio, CSPIN);
    pio_gpio_init(pio, SCKPIN);
    pio_gpio_init(pio, SDOPIN);
    pio_gpio_init(pio, SDIPIN_A);
    if(DUALSDO) pio_gpio_init(pio, SDIPIN_B);
    pio_sm_set_clkdiv_int_frac(pio, MAINSPI_SM, 1, 0);
    pio_sm_set_clkdiv_int_frac(pio, DIN_A_SM, 1, 0);
    pio_sm_set_clkdiv_int_frac(pio, DIN_B_SM, 1, 0);
    pio_sm_init(pio, MAINSPI_SM, ads7253_write_data_pio_offset_entry_point, &mainsm);
    pio_sm_init(pio, DIN_A_SM, datasmoffset, &datasm_A);
    pio_sm_init(pio, DIN_B_SM, datasmoffset, &datasm_B);
    pio_add_program_at_offset(pio, &ads7253_write_data_pio_program, mainsmoffset);
    pio_add_program_at_offset(pio, &ads7253_read_data_pio_program, datasmoffset);
    
    pio->sm[MAINSPI_SM].shiftctrl |= (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS);
    pio->sm[DIN_A_SM].shiftctrl |= (PIO_SM1_SHIFTCTRL_FJOIN_RX_BITS);
    pio->sm[DIN_B_SM].shiftctrl |= (PIO_SM2_SHIFTCTRL_FJOIN_RX_BITS);
    
    pio_sm_set_enabled(pio, MAINSPI_SM, true);
    pio_sm_set_enabled(pio, DIN_A_SM, true);
    if(DUALSDO) pio_sm_set_enabled(pio, DIN_B_SM, true);
}

#endif

