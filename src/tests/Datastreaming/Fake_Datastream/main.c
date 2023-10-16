/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

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


#define CDC_DATA_CHAN   0
#define CDC_CTRL_CHAN   1

// Fake Data Transfer Stuff
/*
  A fixed large length of buffer
  This can hold less data, but will always
  send at full length to make life easy.
*/
#define CDC_PACKET_LEN  64
#define BUF_LEN 1024
#define NUM_PACKET_PER_BUF  ((BUF_LEN / CDC_PACKET_LEN) * sizeof(Q15))





uint8_t header_data[CDC_PACKET_LEN];
Q15     h_hat_fake[BUF_LEN];
Q15     fft_r_fake[BUF_LEN];
Q15     fft_i_fake[BUF_LEN];


//static void cdc_task(void);
static void send_data_packet(Q15 *h_hats, Q15 *fftrr, Q15 *fftri, uint16_t num_h_hats);




int main(){
    board_init();

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    for(uint n = 0; n < BUF_LEN; ++n){
        h_hat_fake[n] = n;
        fft_r_fake[n] = n << 1;
        fft_i_fake[n] = n << 2;
    }


    while (1)
    {
        tud_task(); // tinyusb device task
        //cdc_task();
        if(tud_cdc_n_available(CDC_CTRL_CHAN)){
            send_data_packet(h_hat_fake,fft_r_fake,fft_i_fake,BUF_LEN);
        }
    }

    return 0;
}

static void send_data_packet(Q15 *h_hats, Q15 *fftrr, Q15 *fftri, uint16_t num_h_hats){
    /*
        Data: All LSB first / little endian
        Header sample:
        - 1 byte Start of Field
        - 2 bytes Message Type
        - 60 bytes idk yet
        - 1 byte End of Field
    */
    static char usb_2_send_buf[CDC_PACKET_LEN] = {0x00};

    //static const char waitmsg[CDC_PACKET_LEN] = "Waiting for TX buffer..\r\n";
    
    char tmp[64] = {0x00};

    uint16_t sentbytes = 0;

    sprintf(tmp,"Can Write: %u bytes!\r\n", (uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN));
    tud_cdc_n_write(CDC_CTRL_CHAN, tmp, CDC_PACKET_LEN);
    tud_cdc_n_write_flush(CDC_CTRL_CHAN);

    tud_task(); // tinyusb device task

    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while((uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN) < CDC_PACKET_LEN){
            tud_task(); // tinyusb device task
            busy_wait_us(10);
        }
        sentbytes += (uint16_t)tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)h_hats + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }

    tud_task(); // tinyusb device task
    
    if((fftri && fftrr) || num_h_hats){
        num_h_hats = 0;
        usb_2_send_buf[0] = '\0';
    }
    //tud_cdc_n_write(CDC_DATA_CHAN, h_hats, BUF_LEN * sizeof(Q15));
    tud_cdc_n_write_flush(CDC_DATA_CHAN);

    tud_task(); // tinyusb device task
/*
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF >> 1; ++n){
        tud_cdc_n_write(CDC_DATA_CHAN, (fftrr + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
        tud_cdc_n_write_flush(CDC_DATA_CHAN);
    }
    for(uint32_t n = 0; n < NUM_PACKET_PER_BUF >> 1; ++n){
        tud_cdc_n_write(CDC_DATA_CHAN, (fftri + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
        tud_cdc_n_write_flush(CDC_DATA_CHAN);
    }
*/      
        
        

        uint16_t count = (uint16_t)tud_cdc_n_read(CDC_CTRL_CHAN, usb_2_send_buf, sizeof(usb_2_send_buf));

        sprintf(usb_2_send_buf,"BA: %u\tRead: %u\tSent: %u\r\n", (uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN),count, sentbytes);
        tud_cdc_n_write(CDC_CTRL_CHAN,usb_2_send_buf,CDC_PACKET_LEN);
    //}
}

/*
// echo to either Serial0 or Serial1
// with Serial0 as all lower case, Serial1 as all upper case
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count)
{
    uint8_t const case_diff = 'a' - 'A';

    for(uint32_t i=0; i<count; i++)
    {
        if (itf == 0)
        {
        // echo back 1st port as lower case
        if (isupper(buf[i])) buf[i] += case_diff;
        }
        else
        {
        // echo back 2nd port as upper case
        if (islower(buf[i])) buf[i] -= case_diff;
        }

        tud_cdc_n_write_char(itf, buf[i]);
    }
    tud_cdc_n_write_flush(itf);
}
*/
//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
/*
static void cdc_task(void)
{
    uint8_t itf;

    for (itf = 0; itf < CFG_TUD_CDC; itf++)
    {
        // connected() check for DTR bit
        // Most but not all terminal client set this when making connection
        // if ( tud_cdc_n_connected(itf) )
        {
        if ( tud_cdc_n_available(itf) )
        {
            uint8_t buf[64];

            uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

            // echo back to both serial ports
            echo_serial_port(0, buf, count);
            echo_serial_port(1, buf, count);
        }
        }
    }
}

*/
