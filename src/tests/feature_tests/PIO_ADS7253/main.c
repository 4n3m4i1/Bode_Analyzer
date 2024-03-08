#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pio_spi.h"

// Builtin Led GPIO 25
#define LED         25
#define GPIO_ON     1
#define GPIO_OFF    0

#define SPICSN      1
#define SPISCK      2
#define SPIMOSI     3
#define SPIMISO     4

int main(){
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, GPIO_OFF);

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
    pio_ads7253_spi_init(SPIPIO.pio, 16, 1, 0, SPICSN, SPIMOSI, SPIMISO);

    while(1){
        char read_val = getchar_timeout_us(0);

        switch(read_val){
            case 'w': {
                uint16_t sendem[2] = {0x8181, 0xA5A5};
                printf("sending\r\n");
               // gpio_put(SPICSN, GPIO_OFF);
                pio_spi_write16_blocking(&SPIPIO, sendem, 2);
                //SPIPIO.pio->txf[0] = sendem[0];
                //SPIPIO.pio->txf[0] = sendem[1];
               // gpio_put(SPICSN, GPIO_ON);
               printf("RTS\r\n");
            }
            break;

            case 'p':
                printf("Current PTR: %d\r\n", SPIPIO.pio->sm[0].addr);
                printf("Current INSTR: 0x%04X\r\n", SPIPIO.pio->sm[0].instr);
            break;
        }

    }
}