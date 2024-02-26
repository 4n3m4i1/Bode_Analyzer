#ifndef ADS7253_RP2040_h
#define ADS7253_RP2040_h

#include "pico/stdlib.h"
#include <inttypes.h>
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"       // Going to use PWM Wrap IRQ to set sampling ISR interval
#include "hardware/regs/resets.h"
#include "hardware/resets.h"

enum ADS7253_REGISTER_ADDRESSES {
    ADS7253_NOP,
    ADS7253_REFDAC_A_READ,
    ADS7253_REFDAC_B_READ,
    ADS7253_CFR_READ,
    ADS7253_CFR_WRITE = (1 << 3),
    ADS7253_REFDAC_A_WRITE,
    ADS7253_REFDAC_B_WRITE
};


enum ADS7253_CFR_BITS {
    ADS7253_CFR_ZERO_0,
    ADS7253_CFR_ZERO_1,
    ADS7253_CFR_ZERO_2,
    ADS7253_CFR_RD_DATA_FMT,
    ADS7253_CFR_STANDBY,
    ADS7253_CFR_REF_SEL,
    ADS7253_CFR_INM_SEL,
    ADS7253_CFR_ZERO_3,
    ADS7253_CFR_INPUT_RANGE,
    ADS7253_CFR_RD_DATA_LINES,
    ADS7253_CFR_RD_CLK_MODE,
    ADS7253_CFR_ADDR0,
    ADS7253_CFR_ADDR1,
    ADS7253_CFR_ZERO_4,
    ADS7253_CFR_WRITE_READ
};

enum ADS7253_REFDAC_BITS {
    ADS7253_REFDAC_ZERO_0,
    ADS7253_REFDAC_ZERO_1,
    ADS7253_REFDAC_ZERO_2,
    ADS7253_REFDAC_D_0,
    ADS7253_REFDAC_D_1,
    ADS7253_REFDAC_D_2,
    ADS7253_REFDAC_D_3,
    ADS7253_REFDAC_D_4,
    ADS7253_REFDAC_D_5,
    ADS7253_REFDAC_D_6,
    ADS7253_REFDAC_D_7,
    ADS7253_REFDAC_ADDR0,
    ADS7253_REFDAC_ADDR1,
    ADS7253_REFDAC_ZERO_3,
    ADS7253_REFDAC_WRITE_READ
};

/*
    Modes:
        Single 32   -> 0 << 1 | 0
        Single 16   -> 1 << 1 | 0
        Dual 32     -> 0 << 1 | 1
        Dual 16     -> 1 << 1 | 1
*/
enum ADS7253_SAMPLING_MODES {
    ADS7253_32_CLK_MODE,
    ADS7253_16_CLK_MODE
};  

enum ADS7253_SPI_MODES {
    ADS7253_SINGLE_SDO,
    ADS7253_DUAL_SDO
};

#define ADS7253_SINGLE_32_MODE_         ((ADS7253_32_CLK_MODE << 1) | (ADS7253_SINGLE_SDO))
#define ADS7253_SINGLE_16_MODE_         ((ADS7253_16_CLK_MODE << 1) | (ADS7253_SINGLE_SDO))
#define ADS7253_DUAL_32_MODE_           ((ADS7253_32_CLK_MODE << 1) | (ADS7253_DUAL_SDO))
#define ADS7253_DUAL_16_MODE_           ((ADS7253_16_CLK_MODE << 1) | (ADS7253_DUAL_SDO))

#define ADS7253_MIN_SINGLE_SDO_16_CLKS  32
#define ADS7253_MIN_SINGLE_SDO_32_CLKS  48
#define ADS7253_MIN_DUAL_SDO_16_CLKS    16
#define ADS7253_MIN_DUAL_SDO_32_CLKS    32

struct ADS7253_Inst_t {
    spi_inst_t      *ads_spi;
    uint16_t        CFR_A;
    uint16_t        CFR_B;
    uint16_t        REFDAC_A;
    uint16_t        REFDAC_B;
    uint32_t        Fs;
    uint32_t        F_sck;

    void            *adc_isr_func;
};

/*
    Initializes ADS7253 struct and SPI link
*/
void ADS7253_Setup(struct ADS7253_Inst_t *ads, uint32_t sampling_fm, uint32_t clk_mode, uint16_t sdo_mode){
    hw_set_bits(&resets_hw->reset, (ads->ads_spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS));
    hw_clear_bits(&resets_hw->reset, (ads->ads_spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS));
    while (~resets_hw->reset_done & (ads->ads_spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS))
        tight_loop_contents();

    uint32_t clk_per_txfr = 0;
    switch((clk_mode << 1) | sdo_mode){
        case ADS7253_SINGLE_32_MODE_:
            clk_per_txfr = ADS7253_MIN_SINGLE_SDO_32_CLKS;
        break;
        case ADS7253_DUAL_32_MODE_:
            clk_per_txfr = ADS7253_MIN_DUAL_SDO_32_CLKS;
        break;
        case ADS7253_DUAL_16_MODE_:
            clk_per_txfr = ADS7253_MIN_DUAL_SDO_16_CLKS;
        break;
        case ADS7253_SINGLE_16_MODE_:
        default:
            clk_per_txfr = ADS7253_MIN_SINGLE_SDO_16_CLKS;
        break;
    }

    

    // Total single transfer time target ~1us, may need faster
    //      to run at full 1MSPS
    ads->F_sck = spi_set_baudrate(ads->ads_spi, (1000 * 1000 * clk_per_txfr));

    // Shift out on RISING, shift in on FALLING
    spi_set_format(ads->ads_spi, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // If TI fmt works better idk
    //  Set TI Synchronous Serial Format
    //hw_set_bits(&spi_get_hw(ads->ads_spi)->SSPCR0, 1u << 4);

    // Enable SPI and enable DREQ output
    hw_set_bits(&spi_get_hw(ads->ads_spi)->dmacr, SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS);
    hw_set_bits(&spi_get_hw(ads->ads_spi)->cr1, SPI_SSPCR1_SSE_BITS);
}

void ADS7253_Setup_ISR(struct ADS7253_Inst_t *ads, void *adc_isr_func){
   
}

void ADS7253_Enable_ISR(struct ADS7253_Inst_t *ads){

}

void ADS7253_Disable_ISR(struct ADS7253_Inst_t *ads){

}

void ADS7253_Free_Running(struct ADS7253_Inst_t *ads){

}

void ADS7253_Stop_Sampling(struct ADS7253_Inst_t *ads){

}

#endif