// Generic Include
#include <stdlib.h>
#include <stdio.h>
#include <string.h>   // memcpy
#include <ctype.h>
#include <inttypes.h>

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
#define CDC_PACKET_LEN  64
#define BUF_LEN 128
#define NUM_PACKET_PER_BUF  ((BUF_LEN / CDC_PACKET_LEN) * sizeof(Q15))

#define HEADER_STATE 0x00
#define HH_STATE 0x01
#define FFR_STATE 0x02
#define FFI_STATE 0x03
#define IDLE 0x04

uint16_t STATE = IDLE;

Q15 header_data[CDC_PACKET_LEN];
Q15 h_hat_fake[BUF_LEN];
Q15 fft_r_fake[BUF_LEN];
Q15 fft_i_fake[BUF_LEN];
static void idle_work(void);
static void send_header_packet(Q15 *h_data);
static void send_hh_packets(Q15 *hh_data);
static void send_ffr_packets(Q15 *ffr_data);
static void send_ffi_packets(Q15 *ffi_data);
static bool usb_port_write_available(void);
int main(){

    board_init();
    tud_init(BOARD_TUD_RHPORT);
    STATE = HEADER_STATE;

    for(int16_t n = 0; n < BUF_LEN; ++n){
        h_hat_fake[n] = n;
        fft_r_fake[n] = n << 1;
        fft_i_fake[n] = n << 2;
    }

    header_data[0] = 0x01;
    header_data[1] = 0x1A;
    header_data[CDC_PACKET_LEN - 1] = 0x1F;

    while(1){

        tud_task(); //tinyusb task

        //send packets

        if (usb_port_write_available()) {
            switch (STATE)
            {
            case HEADER_STATE:
                send_header_packet(header_data);
                STATE = HH_STATE;
                tud_task();
                break;
            case HH_STATE:
                send_hh_packets(h_hat_fake);
                STATE = FFR_STATE;
                tud_task();
                break;
            case FFR_STATE:
                send_ffr_packets(fft_r_fake);
                STATE = FFI_STATE;
                tud_task();
                break;
            case FFI_STATE:
                send_ffi_packets(fft_i_fake);
                STATE = IDLE;
                tud_task();
                break;
            case IDLE:
                idle_work();
                tud_task();
                STATE = HEADER_STATE;
                break;
            default:
                break;
            }
        }
    }
    return 0;
}

static void send_header_packet(Q15 *h_data){
    tud_cdc_write((uint8_t *)h_data, CDC_PACKET_LEN);
}
static void send_hh_packets(Q15 *hh_data){
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while(!usb_port_write_available()){
            tud_task(); // tinyusb device task
        }
        tud_cdc_write(((uint8_t *)hh_data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
}
static void send_ffr_packets(Q15 *ffr_data){
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while(!usb_port_write_available()){
            tud_task(); // tinyusb device task
        }
        tud_cdc_write(((uint8_t *)ffr_data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
}
static void send_ffi_packets(Q15 *ffi_data){
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while(!usb_port_write_available()){
            tud_task(); // tinyusb device task
        }
        tud_cdc_write(((uint8_t *)ffi_data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
    tud_cdc_write_flush();
}
static bool usb_port_write_available(void){
    return tud_cdc_write_available() == CDC_PACKET_LEN;
}
static void idle_work(void){
    Q15 tmp[CDC_PACKET_LEN] = {0x00};
    tud_cdc_write((uint8_t*)tmp, CDC_PACKET_LEN);
    tud_cdc_write_flush();
}