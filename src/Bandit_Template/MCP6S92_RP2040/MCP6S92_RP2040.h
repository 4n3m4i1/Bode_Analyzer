#ifndef MCP6S92_RP2040_h
#define MCP6S92_RP2040_h

#include <pico/stdlib.h>
#include <inttypes.h>
#include "hardware/spi.h"
#include "hardware/resets.h"

// Channel/Gain switch settling time (us)
// t_ch + t_G = 1.5 + 1 = 2.5 us, round up
#define MCP6S92_SETTLING_TIME   3

enum MCP6S92_INSTRUCTIONS {
    MCP6S92_NOP,
    MCP6S92_SHUTDOWN,
    MCP6S92_REG_WRITE,
    MCP6S92_NOP_RES
};

enum MCP6S92_REGISTER_ADDRESSES {
    MCP6S92_GAIN_REGISTER,
    MCP6S92_CHANNEL_REGISTER
};

enum MCP6S92_GAIN_LEVELS {
    MCP6S92_x1_GAIN,
    MCP6S92_x2_GAIN,
    MCP6S92_x4_GAIN,
    MCP6S92_x5_GAIN,
    MCP6S92_x8_GAIN,
    MCP6S92_x10_GAIN,
    MCP6S92_x16_GAIN,
    MCP6S92_x32_GAIN
};

enum MCP6S92_CHANNELS {
    MCP6S92_CHAN_0,
    MCP6S92_CHAN_1
};

enum MCP6S92_CMD_INDEX {
    MCP6S92_INSTRUCTION_BYTE,
    MCP6S92_DATA_BYTE
};

#define MCP6S92_SPIBAUD     (5 * 1000 * 1000)           // 5MHzg
#define MCP6S92_INSTR(instruction_mcp, register_mcp)    (((instruction_mcp << 5) | (register_mcp)))

union MCP6S92_SPI_CMD {
    uint8_t     cmd_bytes[2];
    uint16_t    mcp_cmd_word;
};


void MCP6S92_Init(spi_inst_t  *mcp_spi, uint16_t CSN_PIN, uint16_t SCK_PIN, uint16_t SI_PIN){
    hw_set_bits(&resets_hw->reset, (mcp_spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS));
    hw_clear_bits(&resets_hw->reset, (mcp_spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS));
    while (~resets_hw->reset_done & (mcp_spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS))
        tight_loop_contents();
    
    spi_set_baudrate(mcp_spi, MCP6S92_SPIBAUD);
    spi_set_format(mcp_spi, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init_mask(CSN_PIN | SCK_PIN | SI_PIN);

    gpio_init(CSN_PIN);
    gpio_pull_up(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);
    gpio_put(CSN_PIN, 0);

    gpio_init(SCK_PIN);
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
    gpio_init(SI_PIN);
    gpio_set_function(SI_PIN, GPIO_FUNC_SPI);

    hw_set_bits(&spi_get_hw(mcp_spi)->dmacr, SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS);
    hw_set_bits(&spi_get_hw(mcp_spi)->cr1, SPI_SSPCR1_SSE_BITS);

    sleep_ms(1);
    gpio_put(CSN_PIN, 1);   // Finish initialization pulse of CSN
}

void MCP6S92_Send_Command(spi_inst_t *mcp_spi, union MCP6S92_SPI_CMD *cmd, uint16_t CSN){
    gpio_put(CSN, 0);
    spi_write16_blocking(mcp_spi, &(cmd->mcp_cmd_word), 1);
    gpio_put(CSN, 1);
}

void MCP6S92_Send_Command_Raw(spi_inst_t *mcp_spi, uint16_t instr_, uint16_t data_, uint16_t CSN){
    gpio_put(CSN, 0);
    uint16_t tmp = (instr_ << 8) | (data_);
    spi_write16_blocking(mcp_spi, &tmp, 1);
    gpio_put(CSN, 1);
}


#endif