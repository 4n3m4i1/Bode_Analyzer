#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "Bode_Bandit.h"

  //////////////////////////////////////////////////////////////////////
 //////////////////   GLOBAL PREALLOC BUFFERS   ///////////////////////
//////////////////////////////////////////////////////////////////////

// Sampling Banks
CORE_1_MEM Q15 X_N[TOTAL_BUFF_LEN];
CORE_1_MEM Q15 D_N[TOTAL_BUFF_LEN];

// LMS FIR For Producing Y[n]
CORE_1_MEM Q15 LMS_FIR_BANK[TOTAL_ADAPTIVE_FIR_LEN];
CORE_1_MEM Q15 LMS_H_HATS[TOTAL_ADAPTIVE_FIR_LEN];

// FFT Tap Buffers
CORE_0_MEM Q15 FR_BUFF[TOTAL_ADAPTIVE_FIR_LEN];
CORE_0_MEM Q15 FI_BUFF[TOTAL_ADAPTIVE_FIR_LEN];


  //////////////////////////////////////////////////////////////////////
 ///////////////////   FUNCTION DEFINITIONS   /////////////////////////
//////////////////////////////////////////////////////////////////////
// Core 0 Function Definitions
static void core_0_main();
static inline void fft_start_setup(struct FFT_PARAMS *cool_fft);
//void setup_tiny_usb();
//void execute_fft(struct FFT_PARAMS *fft);
//void post_process_fft_data(struct FFT_PARAMS *fft);
//void transmit_final_results(struct FFT_PARAMS *fft);
//

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
    //  to get data between the cores do a DMARQ
    //  that'll hot copy the struct over into CORE0MEM or into general idk
    
    struct FFT_PARAMS cool_fft;
    fft_start_setup(&cool_fft);


    while(1){
        tight_loop_contents();
    }
}

  //////////////////////////////////////////////////////////////////////
 /////////////////    Core 1 Main Loop    /////////////////////////////
//////////////////////////////////////////////////////////////////////
static void core_1_main(){
    // Setup LMS controller and buffering
    struct LMS_Fixed_Inst LMS_Inst;
    LMS_Struct_Init(&LMS_Inst, 0x1111, 0x1111, 0, STD_MAX_SAMPLES, 0); // Errors are just straight guesses rn, offset as well
    LMS_Inst.tap_len = DEFAULT_LMS_TAP_LEN;
    LMS_Inst.d_n = D_N;
    LMS_Inst.x_n = X_N;

    // Setup FIR component of the LMS
    struct Q15_FIR_PARAMS LMS_FIR;
    setup_Q15_FIR(&LMS_FIR, DEFAULT_LMS_TAP_LEN);
    LMS_FIR.size_true = TOTAL_ADAPTIVE_FIR_LEN;
    LMS_FIR.data = LMS_FIR_BANK;
    LMS_FIR.taps = LMS_H_HATS;



    while(1){
        tight_loop_contents();
        for(int n = 0; n < TOTAL_BUFF_LEN; ++n){    // for testing 
            X_N[n] = 2;
            D_N[n] = 3;
        }
    }
}


  //////////////////////////////////////////////////////////////////////
 ///////////////////////   FUNctions!   ///////////////////////////////
//////////////////////////////////////////////////////////////////////

////////////////////// CORE 0: FFT, USB, CTRL /////////////
static inline void fft_start_setup(struct FFT_PARAMS *cool_fft){
    cool_fft->num_samples = DEFAULT_LMS_TAP_LEN;
    cool_fft->log2_num_samples = get_log_2(DEFAULT_LMS_TAP_LEN);
    cool_fft->shift_amount = get_shift_amt(cool_fft->log2_num_samples);
    get_table_offset_shift(&cool_fft, DEFAULT_LMS_TAP_LEN);
    cool_fft->fr = FR_BUFF;
    cool_fft->fi = FI_BUFF;
}
