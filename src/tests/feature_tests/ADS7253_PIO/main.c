#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/structs/rosc.h"
#include "hardware/pio.h"
#include "hardware/structs/pio.h"

#include "ads7253_pio.pio.h"

/*
    This is terrible and just for testing, too lazy to break out a real debugger :)
*/

enum PINZ {
    NC,
    CS_PAD,
    SCK_PAD,
    SDO_PAD,
    SDI_A_PAD,
    SDI_B_PAD
};

enum ESSEMM {
    SPI_CTRL_SM,
    SPI_DINA_SM,
    SPI_DINB_SM
};

void __time_critical_func(pio_spi_write16_read16_blocking)(PIO pio, uint16_t *src, uint16_t *dst_A, uint16_t *dst_B, size_t len) {
    size_t tx_remain = len;
    size_t rx_remain = len;
    io_rw_16 *txfifo = (io_rw_16 *) pio->txf[0];
    io_rw_16 *rxfifo_A = (io_rw_16 *) pio->rxf[1];
    io_rw_16 *rxfifo_B = (io_rw_16 *) pio->rxf[2];

    uint32_t DBG_CT = 15000;

    //while (tx_remain || rx_remain) {
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, 0)) {
        //if(tx_remain){
            *txfifo = *src++;
            //pio->txf[0] = *src++;
            //pio->txf[0] = *src++;
            printf("TEE EXX\n");
            --tx_remain;
        }
        printf("AA\n");
        if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, 1)) {
            printf("ZZ\n");
            //*dst_A++ = *rxfifo_A;
            *dst_A++ = pio->rxf[1];
            printf("YY\n");
            //*dst_B++ = *rxfifo_B;
            *dst_B++ = pio->rxf[2];
            printf("ARR EXX\n");
            --rx_remain;
        }

        DBG_CT--;
        if(!DBG_CT){
            printf("Broke on Stuck!\n");
            break;
        }
        printf("WOOP\n");
    }
}

void printbin(size_t len, void *data, bool spaceatfours){
    uint32_t dat = *((uint32_t *)data);
    for(size_t n = 0; n < len; ++n){
        printf("%c", (dat & (1u << ((len - 1) - n))) ? '1' : '0');
        if(n && ((n & 3) == 3)) printf(" ");
    } 
}

void pinctrl_reg_decode(uint32_t pinctrl, size_t smnum){
    printf("SM %d Pinctrl:\r\n", smnum);
    printf("\tSideset Count: %d\r\n", pinctrl >> 29);
    printf("\tSet Count:\t%d\r\n", (pinctrl >> 26) & 7u);
    printf("\tOut Count:\t%d\r\n", (pinctrl >> 20) & 0b11111);
    printf("\tIn Base:\t%d\r\n", (pinctrl >> 15) & 0b11111);
    printf("\tSide Base:\t%d\r\n", (pinctrl >> 10) & 0b11111);
    printf("\tSet Base:\t%d\r\n", (pinctrl >> 5) & 0b11111);
    printf("\tOut Base:\t%d\r\n\n", (pinctrl >> 0) & 0b11111);
}

void main(){
    stdio_init_all();

    busy_wait_ms(100);

    //ADS7253_MainSM_Setup(pio1, 0, CS_PAD, SCK_PAD, SDO_PAD);
    ADS7253_PIO_Setup(pio1, 1, CS_PAD, SCK_PAD, SDO_PAD, SDI_A_PAD, SDI_B_PAD);

    uint16_t pioinstr = 0;
    uint16_t pioaddr = 0;

    uint16_t cmd[3] = {0};
    uint16_t data_A[3];
    uint16_t data_B[3];

    for(int n = 0; n < 3; ++n){
        data_A[n] = 0xFEA4;
        data_B[n] = 0x4AEF;
    }

    cmd[0] = (1 << 15); // CFR write
    cmd[0] |= (1 << 11);    // 16 clk mode
    cmd[0] |= (1 << 9);     // in = 2x vref
    cmd[0] |= (1 << 6);     // use internal reference

   // pio_spi_write16_read16_blocking(pio1, cmd, data_A, data_B, 3);
    pio_sm_put(pio1, 0, cmd[0]);
    pio_sm_put(pio1, 0, cmd[1]);
    pio_sm_put(pio1, 0, cmd[2]);

    while(1){
        //if(pio1->sm[0].instr != pioinstr){
        //    pioinstr = pio1->sm[0].instr;
        //    pioaddr = pio1->sm[0].addr;
        //    printf("%2d 0x%04X\n", pioaddr, pioinstr);
        //}
        
        char a = getchar_timeout_us(0);
        switch(a){
            case 'a':{
                pio_sm_put(pio1, 0, 0xAF81);
                break;
            }
            break;

            case 's':{
                printf("SM\t\t0\t1\t2\t3\r\n");
                printf("Enabled:\t%d\t%d\t%d\t%d\r\n", (pio1->ctrl & (1 << 0)) ? 1 : 0,
                                            (pio1->ctrl & (1 << 1)) ? 1 : 0,
                                            (pio1->ctrl & (1 << 2)) ? 1 : 0,
                                            (pio1->ctrl & (1 << 3)) ? 1 : 0);
                printf("FiFoStat:\t0x%8X\r\n", pio1->fstat);
                printf("FiFo RX:\t%d\t%d\t%d\t%d\r\n", ((pio1->flevel >> 4) & 0xF),
                                                        ((pio1->flevel >> 12) & 0xF),
                                                        ((pio1->flevel >> 20) & 0xF),
                                                        ((pio1->flevel >> 28) & 0xF));
                printf("FiFo TX:\t%d\t%d\t%d\t%d\r\n", ((pio1->flevel >> 0) & 0xF),
                                                        ((pio1->flevel >> 8) & 0xF),
                                                        ((pio1->flevel >> 16) & 0xF),
                                                        ((pio1->flevel >> 24) & 0xF));
                printf("PadDir: \t"); printbin(32, &pio1->dbg_padoe, true); printf("\r\n");
                printf("PadSet:\t"); printbin(32, &pio1->dbg_padout, true); printf("\r\n");
                printf("FiFo Dbgg:\t0x%8X\r\n", pio1->fdebug);
                printf("SM0 Exec Ctrl:\t"); printbin(32, &pio1->sm[0].execctrl, 1); printf("\r\n");
                printf("SM1 Exec Ctrl:\t"); printbin(32, &pio1->sm[1].execctrl, 1); printf("\r\n");
                printf("SM2 Exec Ctrl:\t"); printbin(32, &pio1->sm[2].execctrl, 1); printf("\r\n");
                printf("SM3 Exec Ctrl:\t"); printbin(32, &pio1->sm[3].execctrl, 1); printf("\r\n");


                printf("SM0 Shif Ctrl:\t"); printbin(32, &pio1->sm[0].shiftctrl, 1); printf("\r\n");
                printf("SM1 Shif Ctrl:\t"); printbin(32, &pio1->sm[1].shiftctrl, 1); printf("\r\n");
                printf("SM2 Shif Ctrl:\t"); printbin(32, &pio1->sm[2].shiftctrl, 1); printf("\r\n");
                printf("SM3 Shif Ctrl:\t"); printbin(32, &pio1->sm[3].shiftctrl, 1); printf("\r\n");

                printf("PC VAL:\t%d\t%d\t%d\t%d\r\n", pio1->sm[0].addr, pio1->sm[1].addr, pio1->sm[2].addr, pio1->sm[3].addr);
                printf("Instr:\t\t0x%04X\t0x%04X\t0x%04X\t0x%04X\r\n", pio1->sm[0].instr, pio1->sm[1].instr, pio1->sm[2].instr, pio1->sm[3].instr);

                pinctrl_reg_decode(pio1->sm[0].pinctrl, 0);
                pinctrl_reg_decode(pio1->sm[1].pinctrl, 1);
                pinctrl_reg_decode(pio1->sm[2].pinctrl, 2);
                pinctrl_reg_decode(pio1->sm[3].pinctrl, 3);

                break;
            }

            case 'r':{
                printf("Sending\r\n");
                cmd[0] = 0;
                cmd[1] = 0;
                cmd[2] = 0;

                pio_spi_write16_read16_blocking(pio1, cmd, data_A, data_B, 3);

                printf("Data: %d\t%d\r\n", data_A[0], data_B[0]);
                printf("Data: %d\t%d\r\n", data_A[1], data_B[1]);
                printf("Data: %d\t%d\r\n", data_A[2], data_B[2]);
                break;
            }
            break;

            case 'c':{
                printf("Values Buffered: %s\n",
                        (pio_sm_is_rx_fifo_empty(pio1, 1)) ? "False" : "True" );
                while(!pio_sm_is_rx_fifo_empty(pio1, 1)){
                    printf("PData:\t%d\t%d\r\n", 
                            pio1->rxf[1], 
                            pio1->rxf[2]
                            //*((io_rw_16 *)pio1->rxf[1]),
                            //*((io_rw_16 *)pio1->rxf[2])
                            );
                }
            }
            break;
        }

        tight_loop_contents();
    }
}
