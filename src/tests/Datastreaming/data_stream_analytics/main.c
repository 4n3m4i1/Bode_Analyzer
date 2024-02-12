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


// static uint16_t ACK_FLAG = 0;

static uint16_t STATE = IDLE;
static uint32_t START_TIME = 0;
static uint32_t END_TIME = 0;
static uint32_t TOTAL_TIME = 0;
// static uint32_t NUM_PACKETS = 0;

Q15 header_data[CDC_PACKET_LEN];
Q15 h_hat_fake[BUF_LEN];
Q15 fft_r_fake[BUF_LEN];
Q15 fft_i_fake[BUF_LEN];
static void idle_work(void);
static void send_header_packet(Q15 *h_data);
static void send_hh_packets(Q15 *hh_data);
static void send_ffr_packets(Q15 *ffr_data);
static void send_ffi_packets(Q15 *ffi_data);
// static bool usb_port_write_available(void);
int main(){

    board_init();
    tud_init(BOARD_TUD_RHPORT);
    STATE = IDLE;
    for(int16_t n = 0; n < BUF_LEN; ++n){
        h_hat_fake[n] = n;
        fft_r_fake[n] = n << 1;
        fft_i_fake[n] = n << 2;
    }
    // uint32_t i = 0;
    header_data[0] = 0x4242;
    header_data[1] = 128;
    header_data[CDC_PACKET_LEN - 1] = 0x1F;
    // Q15 test_data = 0x01;
    while(1){

        tud_task(); //tinyusb task
        // if (i == 100) {
        //     char tmp[64] = {0x00};
        //     sprintf(tmp,"20 Data Blocks sent in: %lu us | AVG time: %lu us\r\n", TOTAL_TIME, TOTAL_TIME/20);
        //     tud_cdc_n_write(CDC_CTRL_CHAN, tmp, CDC_PACKET_LEN);
        //     tud_cdc_n_write_flush(CDC_CTRL_CHAN);
        //     NUM_PACKETS = 0;
        //     TOTAL_TIME = 0;
        //     i = 0;
        // }   
        // busy_wait_ms(10);
        // //send packets
        // tud_cdc_write((uint8_t *)&test_data, CDC_PACKET_LEN);
        if (tud_cdc_n_available(CDC_CTRL_CHAN)) {
            switch (STATE)
            {
            case HEADER_STATE:
                send_header_packet(header_data);
                STATE = HH_STATE;
                tud_task();
                // NUM_PACKETS += 1;
                break;
            case HH_STATE:
                send_hh_packets(h_hat_fake);
                STATE = FFR_STATE;
                // NUM_PACKETS += 2;
                tud_task();
                break;
            case FFR_STATE:
                send_ffr_packets(fft_r_fake);
                STATE = FFI_STATE;
                // NUM_PACKETS += 2;
                tud_task();
                break;
            case FFI_STATE:
                send_ffi_packets(fft_i_fake);
                STATE = IDLE;
                // NUM_PACKETS += 2;
                char tmp[64] = {0x00};
                END_TIME = time_us_32();
                uint32_t duration = END_TIME - START_TIME;
                sprintf(tmp,"Sent in: %lu us\r\n", duration);
                TOTAL_TIME += duration;
                tud_cdc_n_write(CDC_CTRL_CHAN, tmp, CDC_PACKET_LEN);
                tud_cdc_n_write_flush(CDC_CTRL_CHAN);
                tud_task();
                break;
            case IDLE:
                START_TIME = time_us_32();
                // NUM_PACKETS += 1;
                idle_work();
                STATE = HEADER_STATE;
                break;
            default:
                break;
            }
        }
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