/*
    Joseph A. De Vico
    21/03/2024

    PIO ADS7253 Library
        Supports single and dual 16 clock modes natively
        With double TX can support 32 bit modes.

    There are some known timing and desync issues, but
        largely functionality is there.
*/

#ifndef PIO_ADS7253_h
#define PIO_ADS7253_h

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pio_ads7253_spi.pio.h"

#define ADS_PIO_MAIN_SM 0
#define ADS_PIO_SDOA_SM 1
#define ADS_PIO_SDOB_SM 2

#define ADS_READ_REG_COUNT  3
#define ADS_WRITE_DAC_COUNT 3

#define ADS_MAX_CODE_BINARY 4095
#define ADS_MID_CODE_BINARY 2047
#define ADS_MIN_CODE_BINARY 0

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
#define ADS7253_SET_DUAL_SDO    (0u << ADS7253_CFR_RD_DATA_LINES)
#define ADS7253_SET_16_CLK_MODE (1u << ADS7253_CFR_RD_CLK_MODE)
#define ADS7253_SET_32_CLK_MODE (0u << ADS7253_CFR_RD_CLK_MODE)
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


#define ADS7253_MIN_SINGLE_SDO_16_CLKS  32
#define ADS7253_MIN_SINGLE_SDO_32_CLKS  48
#define ADS7253_MIN_DUAL_SDO_16_CLKS    16
#define ADS7253_MIN_DUAL_SDO_32_CLKS    32

#define ADS7253_CMD(_operation, _data)  ((uint16_t)(_operation << 12) | _data)


void ADS7253_write16_blocking(const PIO pio, const uint16_t *src, size_t len);

void ADS7253_write16_blocking_clr_RX(const PIO pio, const uint16_t *src, size_t len);

void ADS7253_read16_blocking(const PIO pio, uint16_t *dst, size_t len);

void ADS7253_write16_read16_blocking(const PIO pio, uint16_t *src, uint16_t *dst, size_t len);
void ADS7253_write16_read16_blocking_dual(const PIO pio, uint16_t *src, uint16_t *dstA, uint16_t *dstB, size_t len);

void ADS7253_Dual_Sampling(const PIO pio, uint16_t *src, uint16_t *dstA, uint16_t *dstB, size_t len);

// No checks, raw pointer assigns via io_rw_16 accesses
void ADS7253_Read_Dual_Data(const PIO pio, uint16_t *dst_A, uint16_t *dst_B);

// Has checks, returns B << 16 | A
uint32_t ADS7253_read_RX_buffer(const PIO pio);

// Slow, but apparently legit.. Reads CFR after application, returns CFR readback
uint16_t ADS7253_TI_Approved_Configuration(const PIO pio, uint16_t init_args);

#endif