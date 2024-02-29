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
// float h_hat_fake[BUF_LEN];
Q15 fft_r_fake[BUF_LEN];
Q15 fft_i_fake[BUF_LEN];
static void idle_work(void);
static void send_header_packet(Q15 *h_data);
static void send_hh_packets(double *hh_data);
static void send_ffr_packets(Q15 *ffr_data);
static void send_ffi_packets(Q15 *ffi_data);

int main(){

    board_init();
    tud_init(BOARD_TUD_RHPORT);
    tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, WANTED_CHAR);
    STATE = IDLE;
    for(int16_t n = 0; n < BUF_LEN; ++n){
        // h_hat_fake[n] = n;
        fft_r_fake[n] = n << 1;
        fft_i_fake[n] = n << 2;
    }
    double h_hat_fake[] = {5.27838506e-02,  2.63917090e-01,  4.94396016e-01,  3.60652428e-01,
 -5.21983226e-02, -1.90376257e-01,  5.51147280e-03,  1.00490426e-01,
 -5.85379879e-04, -5.30473666e-02,  6.27126923e-05,  2.80000200e-02,
 -9.22543397e-06, -1.47828326e-02, -1.31357753e-06,  7.79960767e-03,
 -4.52248187e-06, -4.12558187e-03, -6.85572836e-06,  2.17241340e-03,
  2.90588853e-06, -1.14608446e-03, -1.84458351e-06,  6.06766272e-04,
  5.90173396e-06, -3.17399625e-04, -5.86862735e-06,  1.62697421e-04,
  3.59464565e-06, -7.68986908e-05,  9.41936325e-06,  4.42754187e-05,
 -9.87864217e-06, -2.98732900e-05,  3.14432259e-06,  1.70002650e-05,
 -1.98921975e-07, -1.08707306e-05, -7.54607743e-07,  1.04762942e-05,
  6.08718198e-06, -3.35238857e-06, -3.41054835e-06,  3.40531464e-06,
  2.54247536e-06, -3.88346304e-06, -4.56478908e-06, -2.15806069e-06,
 -1.56836764e-06, -6.92550710e-07,  5.84521820e-07, -2.75237157e-06,
 -2.82605851e-06,  6.17697161e-06,  1.59339945e-05,  1.26891162e-05,
 -7.86930245e-07, -7.91582649e-06, -4.86094588e-06, -9.26906760e-07,
 -1.49259237e-06, -3.87900318e-06, -2.56016509e-06,  2.34408391e-06,
  3.89571147e-06,  2.02062940e-06, -5.13654523e-07, -8.20577921e-07,
  2.67793845e-06,  6.30273837e-06,  2.73570835e-06, -5.04931481e-06,
 -7.35107387e-06, -3.55745257e-06, -7.23796029e-07,  1.71969362e-08,
  1.28004155e-06,  1.80401326e-06,  2.08152764e-06,  3.12401852e-06,
  1.85369820e-06, -3.02300765e-06, -7.69988722e-06, -4.02752075e-06,
  2.04516817e-06,  3.02334022e-06,  1.37573270e-06,  1.57405427e-06,
  4.50914576e-06,  4.90275914e-06,  9.90606469e-07,  1.66922413e-06,
  6.20043200e-06,  5.53216977e-06,  1.88142301e-06,  1.97011011e-06,
  4.82599140e-06,  6.69228853e-06,  9.05414242e-06,  9.62221819e-06,
  4.89638772e-06, -1.42020727e-06, -3.24717486e-06, -2.10566302e-07,
  3.43880881e-06,  3.24161936e-06, -5.75430912e-07, -2.26902772e-06,
 -1.96843347e-07,  4.29775005e-06,  1.08408719e-05,  1.29232029e-05,
  6.61429532e-06, -2.54815625e-06, -4.88579237e-06, -2.59008434e-06,
 -3.12051320e-06, -4.46043985e-06, -7.06420036e-07,  9.52299940e-07,
 -1.84621413e-06, -8.50408002e-07,  1.92925725e-06,  9.41709808e-07,
 -1.45922797e-06,  5.32387738e-07,  2.84134149e-06, -1.01673651e-07};
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
static void send_hh_packets(double *hh_data){
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