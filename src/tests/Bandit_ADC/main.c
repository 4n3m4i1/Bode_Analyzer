#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

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
CORE_1_MEM uint16_t Q15_Sampling_Bank_ptr;
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


// Global Settings Buffer, Mutual Exclusive Access via hardware semaphores
//  to save on access stalls
struct CORE_0_MEM BANDIT_SETTINGS  Global_Bandit_Settings;

  //////////////////////////////////////////////////////////////////////
 ///////////////////   FUNCTION DEFINITIONS   /////////////////////////
//////////////////////////////////////////////////////////////////////
// Generic Function Definitions
static void init_LED_pins();
static void set_RGB_levels(uint8_t R_, uint8_t G_, uint8_t B_);
static void set_ULED_level(uint8_t _L);

// Core 0 Function Definitions
static void core_0_main();
static void USB_Handler();
static inline void fft_setup(struct FFT_PARAMS *cool_fft, uint16_t len);
static inline void fft_clear_fi(struct FFT_PARAMS *cool_fft);
//void setup_tiny_usb();
//void execute_fft(struct FFT_PARAMS *fft);
//void post_process_fft_data(struct FFT_PARAMS *fft);
//void transmit_final_results(struct FFT_PARAMS *fft);
//

// Core 1 Function Definitions
static void     core_1_main();
static void     setup_ADC(struct ADS7253_Inst_t *ADC, irq_handler_t adc_read_handler);
static void     set_ADC_free_running();
static void     stop_ADC_free_running();
//void    setup_sampling_ISR();
void            sampling_ISR_func();
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
    gpio_put(VDDA_EN_PAD, false);
    
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
    struct FFT_PARAMS cool_fft;
    fft_setup(&cool_fft, DEFAULT_LMS_TAP_LEN);



    // USB State Machine, Settings Application, FFT
    
    
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


    struct ADS7253_Inst_t ADC_Inst;
    setup_ADC(&ADC_Inst, sampling_ISR_func);
    Q15_Sampling_Bank_ptr = 0;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t l = 0;
    
    set_ADC_free_running();

    while(1){
        if(Q15_Sampling_Bank_ptr >= TOTAL_BUFF_LEN){
            set_RGB_levels(r++,g,b++);
            Q15_Sampling_Bank_ptr = 0;
            set_ADC_free_running();
        } else {
            tight_loop_contents();
            sleep_us(1);
        }
        //tight_loop_contents();

        //set_ULED_level((uint8_t)pwm_hw->slice[PWMSLICE_ISR].ctr);
        //sleep_ms(15);
    }
}


  //////////////////////////////////////////////////////////////////////
 ///////////////////////   FUNctions!   ///////////////////////////////
//////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////
 ////////////////////// CORE 0: FFT, USB, CTRL /////////////
/////////////////////////////////////////////////////////
static void USB_Handler(){
    // tusb here!
    // No stall states, this should whizz thru and be fine with the
    //  potential to not be handled for some time.
    //  Either use ISRs or hardware buffers
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
void sampling_ISR_func(){
    pwm_clear_irq(PWMSLICE_ISR);
    static uint16_t rdsamp[2];
    gpio_put(ADC_CSN_PAD, false);
    spi_read16_blocking(spi0, (uint16_t)ADS7253_NOP, rdsamp, 2);
    gpio_put(ADC_CSN_PAD, true);
    // DDSamp Here, have ~100 cycles so be efficient!

    //  ADC Channel A = D_N
    //  ADC Channel B = X_N
    //  Should equal output from downsampling but here we are...
    D_N_0[Q15_Sampling_Bank_ptr]    = (Q15)(((int32_t)rdsamp[0]) - 2048);
    X_N_0[Q15_Sampling_Bank_ptr++]  = (Q15)(((int32_t)rdsamp[1]) - 2048);
    if(Q15_Sampling_Bank_ptr >= TOTAL_BUFF_LEN) stop_ADC_free_running();
};

// Setup ADC and PGA
static void setup_ADC(struct ADS7253_Inst_t *ADC, irq_handler_t adc_read_handler){
    gpio_init_mask(
                PINSH(ADC_CSN_PAD) |
                PINSH(ADC_SCK_PAD) |
                PINSH(ADC_SDI_PAD) |
                PINSH(ADC_SDO_A_PAD)
                );

    gpio_set_dir(ADC_CSN_PAD, GPIO_OUT);
    gpio_pull_up(ADC_CSN_PAD);               // Pullup just in case
    gpio_put(ADC_CSN_PAD, true);                // CS is active low

    gpio_set_function(ADC_SCK_PAD, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SDI_PAD, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SDO_A_PAD, GPIO_FUNC_SPI);

    
    ADC->ads_spi = spi0;

    ADS7253_Setup(ADC, 500 * 1000, ADS7253_16_CLK_MODE, ADS7253_SINGLE_SDO);

    // Setup a PWM timer for high resolution trimming of sampling rate
    //  Use slice 1 since it cant be brought out to pins anyway
    // Want: Fs = 500k
    //  125,000,000 / 500,000 = 250
    //  Divider == 1
    //  Top = 250
    pwm_hw->slice[PWMSLICE_ISR].top =   249u;
    pwm_hw->slice[PWMSLICE_ISR].div =   1u << PWM_CH1_DIV_INT_LSB;
    pwm_hw->slice[PWMSLICE_ISR].ctr =   0u;
    //pwm_hw->slice[1].csr =  PWM_CH1_CSR_EN_BITS;
    pwm_clear_irq(PWMSLICE_ISR);
    //pwm_hw->inte |=         (1 << 1);   // Enable PWM block IRQ output from slice 1
    pwm_set_irq_enabled(PWMSLICE_ISR, true);

    //irq_set_exclusive_handler(PWM_IRQ_WRAP, adc_read_handler);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, sampling_ISR_func);
    irq_set_enabled(PWM_IRQ_WRAP, true);
}

static void set_ADC_free_running(){
    pwm_hw->slice[PWMSLICE_ISR].ctr    = 0;
    pwm_hw->slice[PWMSLICE_ISR].csr    |=  PWM_CH1_CSR_EN_BITS;
}

static void stop_ADC_free_running(){
    pwm_hw->slice[PWMSLICE_ISR].csr    &= ~(PWM_CH1_CSR_EN_BITS);
}
  /////////////////////////////////////////////////////////
 ////////////////////// Generic Functions :) /////////////
/////////////////////////////////////////////////////////
static void init_LED_pins(){
    // Cleanup for direct register access, minimal slice readdress
    //  since they are shared to a high degree
    gpio_init_mask(                         // Enable GPIO for LEDs
                PINSH(RGB_R_PAD) | 
                PINSH(RGB_G_PAD) | 
                PINSH(RGB_B_PAD) | 
                PINSH(USER_LED_PAD)
                );

    // Set GPIO Muxes to PWM sources
    gpio_set_function(RGB_R_PAD, GPIO_FUNC_PWM);
    gpio_set_function(RGB_G_PAD, GPIO_FUNC_PWM);
    gpio_set_function(RGB_B_PAD, GPIO_FUNC_PWM);
    gpio_set_function(USER_LED_PAD, GPIO_FUNC_PWM);

    // Zero out all PWM levels just in case
    set_RGB_levels(0,0,0);  
    set_ULED_level(0);  

    /*
        Set PWM top, set PWM slice clk div, 
        invert both channel outputs, enable pwm slice

        125MHz / 256 (divider) == 488281 Hz counter clock
            488281 / 256 max -> 1.907kHz output f, good enough!
    */

    // RED and GREEN RGB Channels
    pwm_hw->slice[PWMSLICE_RG].top =    0xFF;
    pwm_hw->slice[PWMSLICE_RG].div =    (0xFF << PWM_CH6_DIV_INT_LSB);
    pwm_hw->slice[PWMSLICE_RG].csr =    PWM_CH6_CSR_A_INV_BITS |
                                        PWM_CH6_CSR_B_INV_BITS |
                                        PWM_CH6_CSR_EN_BITS;

    // BLUE RGB Channel, USER LED
    pwm_hw->slice[PWMSLICE_BU].top =    0xFF;
    pwm_hw->slice[PWMSLICE_BU].div =    (0xFF << PWM_CH7_DIV_INT_LSB);
    pwm_hw->slice[PWMSLICE_BU].csr =    PWM_CH7_CSR_A_INV_BITS |
                                        PWM_CH7_CSR_B_INV_BITS |
                                        PWM_CH7_CSR_EN_BITS;

}
static void set_RGB_levels(uint8_t R_, uint8_t G_, uint8_t B_){
    // Channel B is upper 16 bits, Channel A is lower 16 bits
    //  Red     -> Channel A, slice 6
    //  Green   -> Channel B, slice 6
    //  Blue    -> Channel A, slice 7
    pwm_hw->slice[PWMSLICE_RG].cc = (((uint16_t)G_ << 16) | (uint16_t)R_);
    pwm_hw->slice[PWMSLICE_BU].cc = (pwm_hw->slice[PWMSLICE_BU].cc & 0xFFFF0000 | (uint16_t)B_); 
}
static void set_ULED_level(uint8_t _L){
    // Channel B is upper 16 bits, CHannel A is lower 16 bits
    //  UserLed -> Channel B, slice 7
    pwm_hw->slice[PWMSLICE_BU].cc = ((uint16_t)_L << 16) | (pwm_hw->slice[PWMSLICE_BU].cc & 0x0000FFFF); 
}


