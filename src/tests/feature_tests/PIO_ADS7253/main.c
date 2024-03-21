#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pio_spi.h"

#include "ADS_CONSTANTS.h"

// Builtin Led GPIO 25
#define LED         25
#define GPIO_ON     1
#define GPIO_OFF    0

#define SPICSN      1
#define SPISCK      2
#define SPIMOSI     3
#define SPIMISO     4

#define VDDA_EN_PAD 21

int main(){
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, GPIO_OFF);

    gpio_init(VDDA_EN_PAD);
    gpio_set_dir(VDDA_EN_PAD, GPIO_OUT);
    gpio_put(VDDA_EN_PAD, true);

    stdio_init_all();

    sleep_ms(200);

    pio_spi_inst_t SPIPIO;
    SPIPIO.pio = pio1;
    SPIPIO.sm = 0;
    SPIPIO.cs_pin = SPICSN;

    //gpio_init(SPICSN);
    //gpio_set_dir(SPICSN, GPIO_OUT);
    //gpio_put(SPICSN, GPIO_ON);

    //static inline void pio_spi_init(PIO pio, uint sm, uint prog_offs, uint n_bits,
    //    float clkdiv, bool cpha, bool cpol, uint pin_sck, uint pin_mosi, uint pin_miso)

    //pio_spi_init(pio1, 0, 0, 8, 16, 0, 0, SPISCK, SPIMOSI, SPIMISO);

    /*
    static inline void pio_ads7253_spi_init(PIO pio, uint sm, uint num_bits, uint clkdiv_i, uint clkdiv_f,
        uint pin_cs, uint pin_copi, uint pin_miso) {
    */

    //unsigned int progoffset = pio_add_program(SPIPIO.pio, &ADS7253_SPI_CTRL_program);
    pio_ads7253_spi_init(SPIPIO.pio, 16, 1, 0, SPICSN, SPIMOSI, SPIMISO, 0);

    uint16_t refval = 0x0108;

    uint8_t autolooper = false;

    while(1){
        char read_val = getchar_timeout_us(0);

        if(autolooper){
            uint16_t sendem[1] = {0};
            pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));
            
            while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                printf("A: %6u\tB: %6u\n", pio1->rxf[1] >> 2, pio1->rxf[2] >> 2);
            }
            busy_wait_us(5);
        }

        switch(read_val){
            case 'l':{
                autolooper = (autolooper) ? 0 : 1;
            };
            break;

            case 's':{
                printf("STANBY!\r\n");
                uint16_t sendemcmd[3];
                sendemcmd[0] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (0 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_STANDBY)));
                sendemcmd[1] = 0x0000;
                sendemcmd[2] = 0x0000;
                pio_spi_write16_blocking(&SPIPIO, sendemcmd, count_of(sendemcmd));
                printf("STANBY ENTRD\n");
                
            }
            
            case 'c': {
                printf("Configuring!\r\n");
                uint16_t sendemcmd[3] = {0,0,0};
                //sendemcmd[0] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (0 << ADS7253_CFR_RD_DATA_LINES) | (0 << ADS7253_CFR_REF_SEL)));
                sendemcmd[0] = ADS7253_CMD(ADS7253_CFR_WRITE,
                                            ADS7253_SET_SINGLE_SDO  |
                                            ADS7253_SET_16_CLK_MODE |
                                            ADS7253_USE_INTERNAL_REFERENCE |
                                            ADS7253_2X_REFDAC
                                            );
                
                sendemcmd[1] = 0x0000;
                sendemcmd[2] = 0x0000;
                pio_spi_write16_blocking(&SPIPIO, sendemcmd, count_of(sendemcmd));

                busy_wait_us(2);

                sendemcmd[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, refval << 3);
                pio_spi_write16_blocking(&SPIPIO, sendemcmd, count_of(sendemcmd));

                busy_wait_us(2);

                sendemcmd[0] = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, refval << 3);
                pio_spi_write16_blocking(&SPIPIO, sendemcmd, count_of(sendemcmd));

                printf("Configured with: 0x%4X\n", sendemcmd[0]);
            }
            

            case 'r': {
                printf("Reading!\n");
            
                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("RX: %6u\t%6u\n", pio1->rxf[1], pio1->rxf[2]);
                }
            }
            break;

            case 'z': {
                uint16_t sendem[3] = {0,0,0};
                sendem[0] = ADS7253_CMD(ADS7253_CFR_READ, 0);
                //sendem[1] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                //sendem[2] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));

                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));
               

                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("RX: %6u\t%6u\n", pio1->rxf[1], pio1->rxf[2]);
                }

                printf("Read CFR with: 0x%4X\n", sendem[0]);
            }
            break;

            case 'R':{
                printf("Real Deal Read from TI\r\n");
                uint16_t sendem[3] = {0,0,0};

                sendem[0] = ADS7253_CMD(ADS7253_CFR_READ, 0);
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));
                
                volatile uint16_t A = 0;
                
                A += 10; A += 10; A += 10;

                sendem[0] = ADS7253_CMD(ADS7253_CFR_WRITE,
                                            ADS7253_SET_16_CLK_MODE |
                                            ADS7253_USE_INTERNAL_REFERENCE |
                                            ADS7253_2X_REFDAC
                                            );

                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));

                A += 10; A += 10; A = sendem[0];

                sendem[0] = 0;
                pio_spi_write16_blocking(&SPIPIO, sendem, 2);

                printf("Configured: 0x%04X\n", A);
            };
            break;

            case 'w': {
                uint16_t sendem[3] = {0,0,0};
                printf("sending\r\n");
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));
                
                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("RX: 0x%4X\t0x%4X\n", pio1->rxf[1], pio1->rxf[2]);
                }
                printf("RTS\r\n");
            }
            break;

            case 'p':
                printf("Current PTR: %d\r\n", SPIPIO.pio->sm[0].addr);
                printf("Current INSTR: 0x%04X\r\n", SPIPIO.pio->sm[0].instr);
            break;

            case '+':{
                refval += 16;
                uint16_t sendem[3] = {0,0,0};
                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, (refval << 3));
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));

                busy_wait_us(4);

                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, (refval << 3));
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));

                printf("Plus\t0x%04X\n", refval);
                printf("Set DAC B with: 0x%04X\n", sendem[0]);
            }
            break;

            case '-':{
                refval -= 16;
                uint16_t sendem[3] = {0,0,0};
                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, refval << 3);
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));

                busy_wait_us(4);

                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, refval << 3);
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));
                printf("Minus\t0x%04X\n", refval);
                printf("Set DAC B with: 0x%04X\n", sendem[0]);
            }
            break;

            case 'm': {
                refval -= 16;
                uint16_t sendem[3] = {0x9FF8,0,0};
                //sendem[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, refval << 3);
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));

                busy_wait_us(4);

                //sendem[0] = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, refval << 3);
                pio_spi_write16_blocking(&SPIPIO, sendem, count_of(sendem));
                printf("Minus\t0x%04X\n", refval);
                printf("Set DAC B with: 0x%04X\n", sendem[0]);
            };
        }

    }
}