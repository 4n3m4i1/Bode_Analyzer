// Generic Include
#include <stdlib.h>
#include <stdio.h>
#include <string.h>   // memcpy
#include <ctype.h>
#include <inttypes.h>
#include <time.h>
// TinyUSB Stack
#include "bsp/board.h"
#include "tusb.h"

// RP2040 Project Specific
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/structs/rosc.h"
#include "generic_include.h"


#define CDC_DATA_CHAN 0
#define CDC_CTRL_CHAN 1
#define CDC_PACKET_LEN  64
#define BUF_LEN 512
#define NUM_PACKET_PER_BUF  ((BUF_LEN / CDC_PACKET_LEN) * sizeof(Q15))

#define HEADER_STATE 0x00
#define HH_STATE 0x01
#define FFR_STATE 0x02
#define FFI_STATE 0x03
#define IDLE 0x04
// #define WAIT 0x05

static uint16_t STATE = IDLE;
static uint16_t NEXT_STATE = IDLE;
static char WANTED_CHAR = '~';
// Q15 header_data[CDC_PACKET_LEN];
// float h_hat_fake[BUF_LEN];
Q15 fft_r_fake[BUF_LEN];
Q15 fft_i_fake[BUF_LEN];
static void idle_work(void);
static void send_header_packet(uint16_t *h_data);
static void send_f_packets(Q15 *f_data, uint16_t ns);
static void USB_Handler(Q15 *f_data);
int main(){

    board_init();
    tud_init(0);
    tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
    STATE = 5;
    NEXT_STATE = 5;
    for(int16_t n = 0; n < BUF_LEN; ++n){
        // h_hat_fake[n] = n;
        fft_r_fake[n] = n << 1;
        fft_i_fake[n] = n << 2;
    }
    // uint32_t i = 0;
    // header_data[0] = 0x4242;
    // header_data[1] = 128;
    // header_data[CDC_PACKET_LEN - 1] = 0x1F;
    // Q15 test_data = 0x01;
    while(1){
        tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
        STATE = NEXT_STATE;
        tud_task(); //tinyusb task
        // if (tud_cdc_n_available(CDC_CTRL_CHAN) >= 1) {
            switch (STATE)
            {
            case HEADER_STATE: {
                USB_Handler(fft_r_fake);
                NEXT_STATE = IDLE;
                tud_task();
            }
                break;
            case IDLE: {

                idle_work();
                NEXT_STATE = HEADER_STATE;
            }
                break;
            default:
                break;
            }
        // }
        // i += 1;
    }

}
static void send_header_packet(uint16_t *h_data){
    tud_cdc_n_write(CDC_DATA_CHAN, (uint8_t *)h_data, CDC_PACKET_LEN);
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
}
static void send_f_packets(Q15 *f_data, uint16_t ns){
    uint16_t PACKET_PER_BUF = (ns * (uint16_t)sizeof(Q15)) / CDC_PACKET_LEN;
    
    for(uint16_t n = 0; n < PACKET_PER_BUF; ++n){
        while((uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN) < CDC_PACKET_LEN){
            tud_task(); // tinyusb device task
        }
        tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)f_data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
}
static void USB_Handler(Q15 *f_data){
    // tusb here!
    // No stall states, this should whizz thru and be fine with the
    //  potential to not be handled for some time.
    //  Either use ISRs or hardware buffers


    uint16_t header_data[32];

    //setup header data
    header_data[0] = 0x4242;
    header_data[1] = 0x200;
    header_data[2] = 0x018;
    header_data[3] = 0x66;
    for(uint16_t n = 4; n < 31; ++n){
        header_data[n] = 0;
    }
    header_data[31] = 0xAA40;
    tud_cdc_n_read_flush(CDC_CTRL_CHAN);
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
    send_header_packet(header_data);
    while (tud_cdc_n_read_char(CDC_CTRL_CHAN) != 'a') {
        tud_task(); // wait for ACK from GUI
    }
    tud_cdc_n_read_flush(CDC_CTRL_CHAN);
    send_f_packets(f_data, BUF_LEN);
}
static void idle_work(void){
    // Q15 tmp[CDC_PACKET_LEN] = {0x00};
    tud_cdc_n_read_flush(CDC_CTRL_CHAN);
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
    busy_wait_us_32(100);
}
// static int32_t recieve_ack(void){
//     int32_t byte = tud_cdc_n_read_char(CDC_CTRL_CHAN);
//     if (byte != 1) {
//         return 0;
//     } else {
//         return 1;
//     }
// }
dfguyasgkdhaghsdukhadkhvgsyjhdgiuyasdkash
void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char) {
    if (wanted_char == '~' && itf == CDC_CTRL_CHAN) {
        NEXT_STATE = IDLE; 
        tud_cdc_n_read_flush(CDC_CTRL_CHAN);
        }
    
}

