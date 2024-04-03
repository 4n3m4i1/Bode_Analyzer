#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "PIO_ADS7253/PIO_ADS7253.h"

// From Bandit_Pins.h
#define SPICSN      1
#define SPISCK      2
#define SPIMOSI     3
#define SPIMISO     4

#define VDDA_EN_PAD 21

enum AUTO_MODES {
    AUTO_NOP,
    AUTO_SPAM_SAMPLES,
    AUTO_COLLECT_BLOCK,
    AUTO_PRINT_BLOCK
};

#define BLOCK_SIZE      1024

uint16_t    block_a[BLOCK_SIZE];
uint16_t    block_b[BLOCK_SIZE];
uint16_t    block_wr_ptr;

int main(){

    gpio_init(VDDA_EN_PAD);
    gpio_set_dir(VDDA_EN_PAD, GPIO_OUT);
    gpio_put(VDDA_EN_PAD, true);

    stdio_init_all();

    sleep_ms(200);

    //PIO pio1 = pio1;

    pio_ads7253_spi_init(pio1, 16, 1, 0, SPICSN, SPIMOSI, SPIMISO, 0);

    uint8_t auto_mode = 0;

    uint16_t refval = 0x108;

    while('!'){
        char read_val = getchar_timeout_us(0);

        switch(auto_mode){
            case AUTO_NOP:{

            }
            break;

            case AUTO_SPAM_SAMPLES:{
                uint16_t sendem[1] = {0};
                ADS7253_write16_blocking(pio1, sendem, count_of(sendem));

                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("A: %6u\tB: %6u\n", pio1->rxf[1] >> 2, pio1->rxf[2] >> 2);
                }
                busy_wait_us(1);
            }
            break;

            case AUTO_COLLECT_BLOCK:{
                uint16_t sendem[1] = {0};
                //ADS7253_write16_blocking(pio1, sendem, count_of(sendem));
                //busy_wait_us(1);
                //ADS7253_Read_Dual_Data(pio1, &block_a[block_wr_ptr], &block_b[block_wr_ptr++]);
                ADS7253_Dual_Sampling(pio1, sendem, &block_a[block_wr_ptr], &block_b[block_wr_ptr++], 1);
                
                if(block_wr_ptr == BLOCK_SIZE) auto_mode = AUTO_PRINT_BLOCK;
            }
            break;

            case AUTO_PRINT_BLOCK:{
                block_wr_ptr = 0;
                printf("\nBLOCK START\nLen = %u\n\n", BLOCK_SIZE);
                for(uint16_t n = 0; n < BLOCK_SIZE; ++n){
                    printf("%u\t%u\n", block_a[n], block_b[n]);
                }
                printf("\nBLOCK DONE\n\n");
                auto_mode += 1;
            }
            break;

            default:
                auto_mode = AUTO_NOP;
            break;
        }


        switch(read_val){
            case 'a': {
                auto_mode += 1;
                switch(auto_mode){
                    case AUTO_SPAM_SAMPLES: printf("Sample -> Print Cycle!\n"); break;
                    case AUTO_COLLECT_BLOCK: printf("Collecting %u Samples!\n", BLOCK_SIZE); break;
                    default: printf("Idle Mode.\n"); auto_mode = AUTO_NOP; break;
                }
            }
            break;

            case 'S': {
                uint16_t cfrval = ADS7253_TI_Approved_Configuration(pio1, 
                                                                    ADS7253_SET_DUAL_SDO |
                                                                    ADS7253_SET_16_CLK_MODE |
                                                                    ADS7253_USE_INTERNAL_REFERENCE |
                                                                    ADS7253_2X_REFDAC
                                                                    );

                printf("CFR Read As: 0x%4X\n", cfrval);
            };

            case 'W': {
                printf("Sending 3 words\n");
                uint16_t blank[3] = {0,0,0};
                ADS7253_write16_blocking(pio1, blank, count_of(blank));


            };

            case 'w': {
                printf("Sending 1 word\n");
                uint16_t blank[3] = {0,0,0};
                ADS7253_write16_blocking(pio1, blank, 1);
            };

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

                ADS7253_write16_blocking(pio1, sendem, count_of(sendem));
               

                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("RX: %6u\t%6u\n", pio1->rxf[1], pio1->rxf[2]);
                }

                printf("Read CFR with: 0x%4X\n", sendem[0]);
            }
            break;

            case '+': {
                uint16_t sendem[3] = {0,0,0};
                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, refval << 3);
                //sendem[1] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                //sendem[2] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));

                ADS7253_write16_blocking(pio1, sendem, count_of(sendem));
               
                busy_wait_us_32(2);

                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, refval << 3);

                ADS7253_write16_blocking(pio1, sendem, count_of(sendem));

                busy_wait_us_32(2);

                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("RX: %6u\t%6u\n", pio1->rxf[1], pio1->rxf[2]);
                }

                printf("Refval Written: 0x%04X\n", refval++);
            }
            break;

            case '-': {
                uint16_t sendem[3] = {0,0,0};
                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, refval << 3);
                //sendem[1] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                //sendem[2] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));

                ADS7253_write16_blocking(pio1, sendem, count_of(sendem));
               
                busy_wait_us_32(2);

                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, refval << 3);

                ADS7253_write16_blocking(pio1, sendem, count_of(sendem));

                busy_wait_us_32(2);

                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("RX: %6u\t%6u\n", pio1->rxf[1], pio1->rxf[2]);
                }

                printf("Refval Written: 0x%04X\n", refval--);
            }
            break;
        }

    }

}