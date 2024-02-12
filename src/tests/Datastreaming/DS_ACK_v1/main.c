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
#define BUF_LEN 128
#define NUM_PACKET_PER_BUF  ((BUF_LEN / CDC_PACKET_LEN) * sizeof(Q15))

#define HEADER_STATE 0x00
#define HH_STATE 0x01
#define FFR_STATE 0x02
#define FFI_STATE 0x03
#define IDLE 0x04
// #define WAIT 0x05

static uint16_t STATE = IDLE;
static uint16_t NEXT_STATE = IDLE;
static char WANTED_CHAR = 'a';
Q15 header_data[CDC_PACKET_LEN];
Q15 h_hat_fake[BUF_LEN];
Q15 fft_r_fake[BUF_LEN];
Q15 fft_i_fake[BUF_LEN];
static void idle_work(void);
static void send_header_packet(Q15 *h_data);
static void send_hh_packets(Q15 *hh_data);
static void send_ffr_packets(Q15 *ffr_data);
static void send_ffi_packets(Q15 *ffi_data);
// static int32_t recieve_ack(void);

// static bool usb_port_write_available(void);
int main(){

    board_init();
    tud_init(BOARD_TUD_RHPORT);
    tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
    STATE = IDLE;
    for(int16_t n = 0; n < BUF_LEN; ++n){
        h_hat_fake[n] = n;
        fft_r_fake[n] = n << 1;
        fft_i_fake[n] = n << 2;
    }
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    // uint32_t i = 0;
    header_data[0] = 0x4242;
    header_data[1] = 128;
    header_data[CDC_PACKET_LEN - 1] = 0x1F;
    // Q15 test_data = 0x01;
    while(1){

        tud_task(); //tinyusb task
        // if (tud_cdc_n_available(CDC_CTRL_CHAN) >= 1) {
            switch (STATE)
            {
            case HEADER_STATE:
                if (STATE == NEXT_STATE) {
                    send_header_packet(header_data);
                }
                tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
                NEXT_STATE = HH_STATE;
                tud_task();
                break;
            case HH_STATE:
                if (STATE == NEXT_STATE) {
                    send_hh_packets(h_hat_fake);
                }
                tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
                NEXT_STATE = FFR_STATE;
                tud_task();
                break;
            case FFR_STATE:
                if (STATE == NEXT_STATE) {
                    send_ffr_packets(fft_r_fake);
                }
                tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
                NEXT_STATE = FFI_STATE;
                tud_task();
                break;
            case FFI_STATE:
                if (STATE == NEXT_STATE) {
                    send_ffi_packets(fft_i_fake);
                }
                tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
                NEXT_STATE = IDLE;
                tud_task();
                break;
            case IDLE:

                if (STATE == NEXT_STATE) {
                    idle_work();
                }
                tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR); 
                NEXT_STATE = HEADER_STATE;
                break;
            default:
                break;
            }
        // }
        // i += 1;
    }

}
static void send_header_packet(Q15 *h_data){
    tud_cdc_n_write(CDC_DATA_CHAN, (uint8_t *)h_data, CDC_PACKET_LEN);
}
static void send_hh_packets(Q15 *hh_data){
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while((uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN) < CDC_PACKET_LEN){
            tud_task(); // tinyusb device task
        }
        tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)hh_data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
}
static void send_ffr_packets(Q15 *ffr_data){
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while((uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN) < CDC_PACKET_LEN){
            tud_task(); // tinyusb device task
        }
        tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)ffr_data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
}
static void send_ffi_packets(Q15 *ffi_data){
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while((uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN) < CDC_PACKET_LEN){
            tud_task(); // tinyusb device task
        }
        tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)ffi_data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
}
static void idle_work(void){
    Q15 tmp[CDC_PACKET_LEN] = {0x00};
    tud_cdc_n_write(CDC_DATA_CHAN, (uint8_t *)tmp, CDC_PACKET_LEN);
}
// static int32_t recieve_ack(void){
//     int32_t byte = tud_cdc_n_read_char(CDC_CTRL_CHAN);
//     if (byte != 1) {
//         return 0;
//     } else {
//         return 1;
//     }
// }

void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char) {
    tud_cdc_n_write_char(itf, wanted_char);
    tud_cdc_n_read_flush(CDC_CTRL_CHAN);
    gpio_put(25, !gpio_get_out_level(25));
    STATE = NEXT_STATE;
}