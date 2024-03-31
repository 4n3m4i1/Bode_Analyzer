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
#define ads7253_write_data_pio_wrap 6

#define ads7253_write_data_pio_offset_entry_point 6u

static const uint16_t ads7253_write_data_pio_program_instructions[] = {
            //     .wrap_target
    0x7101, //  0: out    pins, 1         side 2 [1] 
    0xa042, //  1: nop                    side 0     
    0x0040, //  2: jmp    x--, 0          side 0     
    0x7101, //  3: out    pins, 1         side 2 [1] 
    0xa022, //  4: mov    x, y            side 0     
    0x00e0, //  5: jmp    !osre, 0        side 0     
    0x89e0, //  6: pull   ifempty block   side 1 [1] 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program ads7253_write_data_pio_program = {
    .instructions = ads7253_write_data_pio_program_instructions,
    .length = 7,
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
#define ads7253_read_data_pio_wrap 3

static const uint16_t ads7253_read_data_pio_program_instructions[] = {
            //     .wrap_target
    0x2082, //  0: wait   1 gpio, 2                  
    0x4001, //  1: in     pins, 1                    
    0x2002, //  2: wait   0 gpio, 2                  
    0x0000, //  3: jmp    0                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program ads7253_read_data_pio_program = {
    .instructions = ads7253_read_data_pio_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config ads7253_read_data_pio_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ads7253_read_data_pio_wrap_target, offset + ads7253_read_data_pio_wrap);
    return c;
}

static inline void ADS7253_MainSM_Setup(PIO pio, uint mainsm, uint CSPIN, uint SCKPIN, uint SDOPIN){
    const uint mainsmoffset = 0;
    const uint shiftct = 16;
    pio_sm_config c = ads7253_write_data_pio_program_get_default_config(mainsmoffset);
    /*
    pio_sm_config c = cpha ? spi_cpha1_cs_program_get_default_config(prog_offs) : spi_cpha0_cs_program_get_default_config(prog_offs);
    sm_config_set_out_pins(&c, pin_mosi, 1);
    sm_config_set_in_pins(&c, pin_miso);
    sm_config_set_sideset_pins(&c, pin_sck);
    sm_config_set_out_shift(&c, false, true, n_bits);
    sm_config_set_in_shift(&c, false, true, n_bits);
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_set_pins_with_mask(pio, sm, (2u << pin_sck), (3u << pin_sck) | (1u << pin_mosi));
    pio_sm_set_pindirs_with_mask(pio, sm, (3u << pin_sck) | (1u << pin_mosi), (3u << pin_sck) | (1u << pin_mosi) | (1u << pin_miso));
    pio_gpio_init(pio, pin_mosi);
    pio_gpio_init(pio, pin_miso);
    pio_gpio_init(pio, pin_sck);
    pio_gpio_init(pio, pin_sck + 1);
    gpio_set_outover(pin_sck, cpol ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    hw_set_bits(&pio->input_sync_bypass, 1u << pin_miso);
    uint entry_point = prog_offs + (cpha ? spi_cpha1_cs_offset_entry_point : spi_cpha0_cs_offset_entry_point);
    pio_sm_init(pio, sm, entry_point, &c);
    pio_sm_exec(pio, sm, pio_encode_set(pio_x, n_bits - 2));
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, n_bits - 2));
    pio_sm_set_enabled(pio, sm, true);
    */
    sm_config_set_out_pins(&c, SDOPIN, 1);
    sm_config_set_sideset_pins(&c, CSPIN);
    sm_config_set_out_shift(&c, false, true, shiftct);
    c.clkdiv = 16 << 16;
    pio_sm_set_pins_with_mask(pio, mainsm, (1u << CSPIN), (1u << CSPIN) | (1u << SCKPIN) | (1u << SDOPIN));
    pio_sm_set_pindirs_with_mask(pio, mainsm, (1u << CSPIN) | (1u << SCKPIN) | (1u << SDOPIN), (1u << CSPIN) | (1u << SCKPIN) | (1u << SDOPIN));
    pio_gpio_init(pio, CSPIN);
    pio_gpio_init(pio, SCKPIN);
    pio_gpio_init(pio, SDOPIN);
    pio_sm_init(pio, mainsm, ads7253_write_data_pio_offset_entry_point, &c);
    pio_sm_exec(pio, mainsm, pio_encode_set(pio_x, shiftct - 2));
    pio_sm_exec(pio, mainsm, pio_encode_set(pio_y, shiftct - 2));
    pio_sm_set_enabled(pio, mainsm, true);
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
    pio_sm_set_pins_with_mask(pio, MAINSPI_SM, (1u << CSPIN), (1u << CSPIN) | (1u << SCKPIN) | (1u << SDOPIN));
    pio_sm_set_pindirs_with_mask(pio, MAINSPI_SM, (1u << CSPIN) | (1u << SCKPIN) | (1 << SDOPIN),
                                                    (1u << CSPIN) | (1u << SCKPIN) | (1 << SDOPIN));
    //pio_sm_set_consecutive_pindirs(pio, MAINSPI_SM, CSPIN, 3, true);
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
    pio_sm_set_clkdiv_int_frac(pio, MAINSPI_SM, 16, 0);
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
