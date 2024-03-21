#ifndef ADS7253_RP2040_h
#define ADS7253_RP2040_h

#include "pico/stdlib.h"
#include <inttypes.h>
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"       // Going to use PWM Wrap IRQ to set sampling ISR interval
#include "hardware/regs/resets.h"
#include "hardware/resets.h"
#include "hardware/pio.h"

#include "ads7253_Data_Interface.pio.h"
#include "ADS7253_SPI.pio.h"
/*
    ADS7253 Data Transfer Protocol

    Unlike what one would hope, or expect, TI uses a slightly modified SPI
    interface that doesn't fit any of the common Motorola Modes (0,1,2,3) nor
    the already somewhat unique TI SPI mode.

    Note: FALLING and RISING refer to SCK edges
    Note: Latched data is what an IC will read
    Note: Update is shifting new data out of an IC

    Data transfers occur as follows from the ADS7253 perspective:
        - SDI data latched on FALLING
        - SDO Updates on FALLING (max 20ns to next valid data)

    As such the following transfer pattern is required from a controller:
        - SDO shifted from controller on RISING
        - SDI latched on RISING

    Note the perspective shift in the functional descriptions.

    The official TI recommendation is to use a SN74LVC2G74 posedge DFF on the
    SDO line of the ADS7253, allowing CPOL = 0 CPHA = 1 standard mode 1 to
    be used.

    (https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/496896/spi-of-ads8353-ads7853-ads7253/1797162#1797162)

    In this library a PIO driver is supplied negating the need for additional
    circuitry externally.
*/

#define ADS_PIO_MAIN_SM 0
#define ADS_PIO_SDOA_SM 1
#define ADS_PIO_SDOB_SM 2

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
    ADS7253_CFR_ZERO_3,
    ADS7253_CFR_RD_DATA_FMT,
    ADS7253_CFR_STANDBY,
    ADS7253_CFR_REF_SEL,
    ADS7253_CFR_INM_SEL,
    ADS7253_CFR_ZERO_4,
    ADS7253_CFR_INPUT_RANGE,
    ADS7253_CFR_RD_DATA_LINES,
    ADS7253_CFR_RD_CLK_MODE,
    ADS7253_CFR_ADDR0,
    ADS7253_CFR_ADDR1,
    ADS7253_CFR_ZERO_5,
    ADS7253_CFR_WRITE_READ
};

#define ADS7253_SET_SINGLE_SDO  (1u << ADS7253_CFR_RD_DATA_LINES)
#define ADS7253_SET_16_CLK_MODE (1u << ADS7253_CFR_RD_CLK_MODE)
#define ADS7253_USE_INTERNAL_REFERENCE  (1u << ADS7253_CFR_REF_SEL)
#define ADS7253_2X_REFDAC       (1u << ADS7253_CFR_INPUT_RANGE)

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

#define ADS7253_CMD(_operation, _data)  ((uint16_t)(_operation << 12) | _data)

struct ADS7253_Inst_t {
    spi_inst_t      *ads_spi;
    uint16_t        CFR_A;
    uint16_t        CFR_B;
    uint16_t        REFDAC_A;
    uint16_t        REFDAC_B;
    uint32_t        Fs;
    uint32_t        F_sck;
    uint16_t        csn_pin;

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

    spi_init(ads->ads_spi, 1000 * 1000 * clk_per_txfr);

    // Total single transfer time target ~1us, may need faster
    //      to run at full 1MSPS
    ads->F_sck = spi_set_baudrate(ads->ads_spi, (1000 * 1000 * clk_per_txfr));

    // Shift out on RISING, shift in on FALLING
    spi_set_format(ads->ads_spi, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // If TI fmt works better idk
    //  Set TI Synchronous Serial Format
    //hw_set_bits(&spi_get_hw(ads->ads_spi)->cr0, 1u << 4);

    // Enable SPI and enable DREQ output
    hw_set_bits(&spi_get_hw(ads->ads_spi)->dmacr, SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS);
    hw_set_bits(&spi_get_hw(ads->ads_spi)->cr1, SPI_SSPCR1_SSE_BITS);
}

void ADS7253_Write_Command(struct ADS7253_Inst_t *ads, uint16_t command){
    uint16_t nullval = 0;
    gpio_put(ads->csn_pin, 0);
    spi_write16_blocking(ads->ads_spi, &command, 1);
    spi_write16_blocking(ads->ads_spi, &nullval, 1);
    //spi_write16_blocking(ads->ads_spi, &nullval, 1);
    gpio_put(ads->csn_pin, 1);
}

uint32_t ADS7253_RW_CMD(struct ADS7253_Inst_t *ads, uint16_t command){
    uint16_t retval[2];
    gpio_put(ads->csn_pin, 0);
    spi_write16_blocking(ads->ads_spi, &command, 1);
    spi_read16_blocking(ads->ads_spi, 0, retval, 2);
    gpio_put(ads->csn_pin, 1);
    return *((uint32_t *)retval);
}

//void ADS7253_Setup_ISR(struct ADS7253_Inst_t *ads, void *adc_isr_func){
//   
//}
//
//void ADS7253_Enable_ISR(struct ADS7253_Inst_t *ads){
//
//}
//
//void ADS7253_Disable_ISR(struct ADS7253_Inst_t *ads){
//
//}
//
//void ADS7253_Free_Running(struct ADS7253_Inst_t *ads){
//
//}
//
//void ADS7253_Stop_Sampling(struct ADS7253_Inst_t *ads){
//
//}

void pio_ADS7253_Setup(const PIO pio, uint div_i, uint div_f, uint CSN_PIN, uint SCK_PIN, uint DOUT_PIN, uint DIN_A, uint DIN_B){
    ADS7253_PIO_Setup(pio, 1, CSN_PIN, SCK_PIN, DOUT_PIN, DIN_A, DIN_B);
}

void  __time_critical_func(pio_spi_write_16_blocking)(const PIO pio, uint16_t data){
    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
    *txfifo = data;
}

void  __time_critical_func(pio_ADS7253_Write_CMD)(const PIO pio, uint16_t command_){
    io_rw_16 *txfifo = (io_rw_16 *) &(pio->txf[ADS_PIO_MAIN_SM]);
    *txfifo = command_;
    *txfifo = 0x0000;
}

void  __time_critical_func(pio_ADS7253_Read_Reg)(const PIO pio, uint16_t adsreg, uint16_t *rxdat){
    io_rw_16 *txfifo = (io_rw_16 *) &(pio->txf[ADS_PIO_MAIN_SM]);
    io_rw_16 *rxfifo = (io_rw_16 *) &(pio->rxf[ADS_PIO_SDOA_SM]);

    uint16_t lenoftxfr;

    while(!pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)) lenoftxfr = *rxfifo;

    lenoftxfr = 3;

    *txfifo = adsreg << 12;
    *txfifo = 0x0000;
    *txfifo = 0x0000;

    while(lenoftxfr){
        if(lenoftxfr && !pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)){
            *rxdat++ = *rxfifo;
            --lenoftxfr;
        }
    }
}


//void  pio_spi_write_16_blocking(const PIO pio, uint16_t data){
//    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
//    *txfifo = data;
//}
//
//void  pio_ADS7253_Write_CMD(const PIO pio, uint16_t command_){
//    io_rw_16 *txfifo = (io_rw_16 *) &(pio->txf[ADS_PIO_MAIN_SM]);
//    *txfifo = command_;
//    *txfifo = 0x0000;
//    *txfifo = 0x0000;
//}
//
//void  pio_ADS7253_Read_Reg(const PIO pio, uint16_t adsreg){
//    io_rw_16 *txfifo = (io_rw_16 *) &(pio->txf[ADS_PIO_MAIN_SM]);
//    io_rw_16 *rxfifo = (io_rw_16 *) &(pio->rxf[ADS_PIO_SDOA_SM]);
//
//    *txfifo = adsreg;
//    *txfifo = 0x0000;
//    *txfifo = 0x0000;
//}


#endif