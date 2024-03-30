#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"

#include "tusb.h"
#include "tusb_config.h"

#include "Bode_Bandit.h"

  //////////////////////////////////////////////////////////////////////
 //////////////////   GLOBAL PREALLOC BUFFERS   ///////////////////////
//////////////////////////////////////////////////////////////////////
// Preserve the abilitiy to transfer to additional buffers
//  in currently General allocated memory to interleave processing/sampling
//  more!

/*
    Current Memory Access Scheme:
    Time:       0           1           2           3
    Core 0:     Settings    N/A         N/A         FR FI
    Core 1:     XN DN       XN DN       Shared      XN DN
    Reason:     Sampling    LMS         FFT Copy    FFT / Sampling

    In theory zero contention, minor waits incurred.
    Adding a tertiary exclusive access bank would allow
    Core 0 processing during time slots 1 and 2, probably working
    on parallelized downsampling or precalculating errors for h_hat
    updates for LMS. (That's like... a good idea)
*/

// Sampling Banks
CORE_1_MEM Q15 X_N_0[TOTAL_BUFF_LEN];
CORE_1_MEM Q15 D_N_0[TOTAL_BUFF_LEN];

// LMS FIR For Producing Y[n]
CORE_1_MEM Q15 LMS_FIR_BANK[TOTAL_ADAPTIVE_FIR_LEN];
CORE_1_MEM Q15 LMS_H_HATS[TOTAL_ADAPTIVE_FIR_LEN];

// Downsampling FIR and Tap Sets
CORE_1_MEM Q15 DDSAMP_FIR_BANK[DDSAMP_FIR_LEN];
CORE_1_MEM Q15 DDSAMP_TAP_BANK[DDSAMP_FIR_LEN][DDSAMP_TAP_OPTIONS];

// FFT Tap Buffers
CORE_0_MEM Q15 FR_BUFF[TOTAL_ADAPTIVE_FIR_LEN];
CORE_0_MEM Q15 FI_BUFF[TOTAL_ADAPTIVE_FIR_LEN];

// Reserve Buffer for Stored Results
CORE_0_MEM Q15 RESULTS_BUFFER[TOTAL_ADAPTIVE_FIR_LEN];

// Core 0 FFT Buffers
struct CORE_0_MEM FFT_PARAMS cool_fft;


// Global Settings Buffer, Mutual Exclusive Access via hardware semaphores
//  to save on access stalls
struct CORE_0_MEM BANDIT_SETTINGS  Global_Bandit_Settings;
CORE_0_MEM uint16_t USB_STATE;
CORE_0_MEM uint16_t USB_NEXT_STATE;

// Core 1 Defines
#define ADCRES_BITS     12u
#define ADC_RES_CODES   (1u << ADCRES_BITS)
#define ADC_VREF_CODE   0x108                   // ~2.15 V from ADS7253 DAC

  //////////////////////////////////////////////////////////////////////
 ///////////////////   FUNCTION DEFINITIONS   /////////////////////////
//////////////////////////////////////////////////////////////////////
// Generic Function Definitions

// Core 0 Function Definitions
static void core_0_main();
static void USB_Handler(struct FFT_PARAMS *fft);
static void FFT_Handler();
static void update_bandit_config();

static inline void fft_setup(struct FFT_PARAMS *cool_fft, uint16_t len);
static inline void fft_clear_fi(struct FFT_PARAMS *cool_fft);
//void setup_tiny_usb();
//void execute_fft(struct FFT_PARAMS *fft);
//void post_process_fft_data(struct FFT_PARAMS *fft);
//void transmit_final_results(struct FFT_PARAMS *fft);
//

static void send_header_packet(Q15 *h_data);
static void send_f_packets(Q15 *data, uint16_t num_samples);

// Core 1 Function Definitions
static void    core_1_main();
//void    setup_ADC();
//void    setup_sampling_ISR();
//void    sampling_ISR_func();
//int     run_adaptive_taps();
//void    handle_intercore_h_hat_transfer();  // DMA tap data to other core, or error on overrun, start fft core 0


  //////////////////////////////////////////////////////////////////////
 ////////////////////////////    Code!    /////////////////////////////
//////////////////////////////////////////////////////////////////////
// Main
// Start up in Core 0
int main(){
    stdio_init_all();
    init_LED_pins();

    gpio_init(VDDA_EN_PAD);
    gpio_set_dir(VDDA_EN_PAD, GPIO_OUT);
    gpio_put(VDDA_EN_PAD, true);
    
    //start_randombit_dma_chain();

    // Setup for peripherals done on core 0
    multicore_launch_core1(core_1_main);
    core_0_main();
    // We should NEVER be here!!!
    while(1){
        // need something here to indicate we messed up.
        tight_loop_contents();
    }

}

  //////////////////////////////////////////////////////////////////////
 /////////////////    Core 0 Main Loop    /////////////////////////////
//////////////////////////////////////////////////////////////////////
static void core_0_main(){
    // Do tiny USB and control stuff here,
    
    Global_Bandit_Settings.settings_bf = BANDIT_DFL_SETTINGS;
    Global_Bandit_Settings.manual_tap_len_setting = 0;
    Global_Bandit_Settings.manual_error_limit = 0;
    Global_Bandit_Settings.manual_freq_range = 0;


    // Setup AWGN Generation from overdriven ROSC -> DMA -> PIO
    setup_rosc_full_tilt();
    setup_PIO_for_switching();
    setup_chained_dma_channels();

    if(CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_WGN_ON)){
        start_randombit_dma_chain(dma_awgn_ctrl_chan);
    }

    // Setup Structs and Default Parameters for FFT
    fft_setup(&cool_fft, DEFAULT_LMS_TAP_LEN);

    // Initialize tinyUSB with board and start listening for start char from GUI
    tud_init(BOARD_TUD_RHPORT);
    tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, START_CHAR);

    // USB USB_STATE Machine, Settings Application, FFT
    USB_NEXT_STATE = USB_INIT;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t l = 0;
    
    while(1){
        
        tud_task();
        USB_STATE = USB_NEXT_STATE;
        switch(USB_STATE) {
            case USB_INIT:
                //need idle work until GUI is ready
                break;
            case USB_FFT_DATA_COLLECT:
                FFT_Handler(); //fft data from shared memory & run fft calc
                USB_NEXT_STATE = USB_SEND_TUSB;
                break;
            case USB_SEND_TUSB:
                USB_Handler(&cool_fft); //send data to GUI through tusb
                USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
                break;
            default:
                break; //need debug for USB_STATE error

        }


        // set_RGB_levels(r-- - g,g--,b++);
        // set_ULED_level(l--);

        // sleep_ms(15);
        tight_loop_contents();
    }
}

  //////////////////////////////////////////////////////////////////////
 /////////////////    Core 1 Main Loop    /////////////////////////////
//////////////////////////////////////////////////////////////////////
static void core_1_main(){
    // Setup LMS controller and buffering
    struct LMS_Fixed_Inst LMS_Inst;
    LMS_Struct_Init(&LMS_Inst, 
                        0x1111, 
                        0x1111, 
                        0, 
                        STD_MAX_SAMPLES, 
                        0
                        ); // Errors are just straight guesses rn, offset as well
    LMS_Inst.tap_len = DEFAULT_LMS_TAP_LEN;
    LMS_Inst.d_n = D_N_0;
    LMS_Inst.x_n = X_N_0;

    // Setup FIR component of the LMS
    struct Q15_FIR_PARAMS LMS_FIR;
    setup_Q15_FIR(&LMS_FIR, DEFAULT_LMS_TAP_LEN);
    LMS_FIR.size_true = TOTAL_ADAPTIVE_FIR_LEN;
    LMS_FIR.data = LMS_FIR_BANK;
    LMS_FIR.taps = LMS_H_HATS;

    /*
        Initiate frontend PGA/MUX
            Set channel 0 (D_N), 1x gain
    */
    spi_inst_t *mcp_spi = spi1;
    MCP6S92_Init(mcp_spi, PGA_CSN_PAD, PGA_SCK_PAD, PGA_SI_PAD);
    // Internal loopback for testing!
    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_0, PGA_CSN_PAD);
    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);

    /*
        Initialize ADC SPI w/ pio1
    */



    while(1){
        tight_loop_contents();
        for(int n = 0; n < TOTAL_BUFF_LEN; ++n){    // for testing 
            X_N_0[n] = 2;
            D_N_0[n] = 3;
        }
    }
}


  //////////////////////////////////////////////////////////////////////
 ///////////////////////   FUNctions!   ///////////////////////////////
//////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////
 ////////////////////// CORE 0: FFT, USB, CTRL /////////////
/////////////////////////////////////////////////////////
static void USB_Handler(struct FFT_PARAMS *fft){
    // tusb here!
    // No stall states, this should whizz thru and be fine with the
    //  potential to not be handled for some time.
    //  Either use ISRs or hardware buffers

    uint16_t num_samples = fft->num_samples;
    Q15 header_data[CDC_PACKET_LEN];

    //setup header data
    header_data[0] = 0x4242;
    header_data[1] = fft->num_samples;
    header_data[2] = fft->log2_num_samples;
    header_data[3] = fft->shift_amount;
    header_data[CDC_PACKET_LEN - 1] = 0x40;

    send_header_packet(header_data);
    while (tud_cdc_n_read_char(CDC_CTRL_CHAN) != 'a') {
        tud_task(); // wait for ACK from GUI
    }
    tud_cdc_n_read_flush(CDC_CTRL_CHAN);
    send_f_packets(fft->fr, num_samples);
}

//sends header data through data chan to GUI interface
static void send_header_packet(Q15 *h_data){
    tud_cdc_n_write(CDC_DATA_CHAN, (uint8_t *)h_data, CDC_PACKET_LEN);
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
}

//sends fft data through data chan to GUI interface
static void send_f_packets(Q15 *data, uint16_t num_samples){
    uint16_t NUM_PACKET_PER_BUF = num_samples / CDC_PACKET_LEN;

    for(uint16_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
        while((uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN) < CDC_PACKET_LEN){
            tud_task(); // tinyusb device task
        }
        tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
    }
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
}

// just for pseudo code utility
static void FFT_Handler(){

}

// recieve config data from GUI and update BANDIT_SETTINGS accordingly
static void update_bandit_config(){

}

//triggered when wanted_char is recieved thru ctrl channel
void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char) { 
    switch (wanted_char) {
        case START_CHAR:
            // signal core 1 to begin processing
            USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
            tud_cdc_n_read_flush(itf);
            tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, SETTINGS_CHAR);
            break;
        case SETTINGS_CHAR:
            //update_fft_config();      // idk bandit config?
            update_bandit_config();
            tud_cdc_n_read_flush(itf); 
            USB_NEXT_STATE = USB_FFT_DATA_COLLECT; 
            break;
        default:
            tud_cdc_n_read_flush(itf);
            break;
    }
}

static inline void fft_setup(struct FFT_PARAMS *cool_fft, uint16_t len){
    cool_fft->num_samples = len;
    cool_fft->log2_num_samples = get_log_2(len);
    cool_fft->shift_amount = get_shift_amt(cool_fft->log2_num_samples);
    get_table_offset_shift(cool_fft, len);
    cool_fft->fr = FR_BUFF;
    cool_fft->fi = FI_BUFF;
}

static inline void fft_clear_fi(struct FFT_PARAMS *cool_fft){
    for(uint16_t n = 0; n < cool_fft->num_samples; ++n){
        cool_fft->fi[n] = 0;
    }
}






  /////////////////////////////////////////////////////////
 ////////////////////// CORE 1: ADC, LMS, Downsampling ///
/////////////////////////////////////////////////////////



  /////////////////////////////////////////////////////////
 ////////////////////// Generic Functions :) /////////////
/////////////////////////////////////////////////////////
