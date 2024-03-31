#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"

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


// Global Settings Buffer, Mutual Exclusive Access via hardware semaphores
//  to save on access stalls
struct CORE_0_MEM BANDIT_SETTINGS  Global_Bandit_Settings;

// Core 1 Defines
#define ADCRES_BITS     12u
#define ADC_RES_CODES   (1u << ADCRES_BITS)
#define ADC_VREF_CODE   0x108                   // ~2.15 V from ADS7253 DAC

#define ADC_PIO         pio1
CORE_1_MEM uint16_t tmp_arr[ADS_READ_REG_COUNT] = {0,0,0};
CORE_1_MEM uint8_t  CORE_1_STATE = CORE_1_IDLE;

struct BANDIT_LED_COLORS {
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t U;
};

struct BANDIT_LED_COLORS Bandit_RGBU;

  //////////////////////////////////////////////////////////////////////
 ///////////////////   FUNCTION DEFINITIONS   /////////////////////////
//////////////////////////////////////////////////////////////////////
// Generic Function Definitions




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

    Bandit_RGBU.R = 0;
    Bandit_RGBU.G = 0;
    Bandit_RGBU.B = 120;
    Bandit_RGBU.U = 0;
    set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B);
    set_ULED_level(Bandit_RGBU.U);
    
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
        static volatile uint16_t a = 0;
        ++a;
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
    pio_ads7253_spi_init(pio1, 16, 1, 0, ADC_CSN_PAD, ADC_SDI_PAD, ADC_SDO_A_PAD, 0);

    Bandit_RGBU.B = 0;
start_adc_setup:

    set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B++);
    ADS7253_TI_Approved_Configuration(ADC_PIO,
                                ADS7253_SET_DUAL_SDO |
                                ADS7253_SET_16_CLK_MODE |
                                ADS7253_USE_INTERNAL_REFERENCE |
                                ADS7253_2X_REFDAC
                                );


    while(!pio_sm_is_rx_fifo_empty(ADC_PIO, ADS_PIO_SDOA_SM) || 
            !pio_sm_is_rx_fifo_empty(ADC_PIO, ADS_PIO_SDOB_SM)){
            
            ADS7253_Read_Dual_Data(ADC_PIO, &tmp_arr[0], &tmp_arr[1]);
    }

    busy_wait_us_32(10);
    
    // Read the CFR and ensure it's lookin good before proceeding
    tmp_arr[0] = ADS7253_CMD(ADS7253_CFR_READ, 0);
    tmp_arr[1] = 0;
    tmp_arr[2] = 0;
    
    // Ask for CFR readback twice just to be sure the data is latched
    //  and CFR is updated.
    ADS7253_write16_blocking(ADC_PIO, tmp_arr, ADS_READ_REG_COUNT);
    busy_wait_us_32(50);
    ADS7253_write16_blocking(ADC_PIO, tmp_arr, ADS_READ_REG_COUNT);
    busy_wait_us_32(50);
    
    while(!pio_sm_is_rx_fifo_empty(ADC_PIO, ADS_PIO_SDOA_SM)){
        tmp_arr[0] = ADC_PIO->rxf[ADS_PIO_SDOA_SM];
        tmp_arr[1] = ADC_PIO->rxf[ADS_PIO_SDOB_SM];
        
        // okay like 2624 is hacky, but it is the expected CFR value
        //  EVEN THOUGH IT SHOULDNT BE THAT
        //  Through all testing that's been the correct return.
        //  This DOES NOT match the datasheet, but through testing is
        //  legit... so idk dude, sendit - Joseph 03/31/2024
        if(tmp_arr[0] == 2624){
            Bandit_RGBU.G = 127;
            set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B);
            break;
        }
    }


    if(Bandit_RGBU.G != 127){
        set_RGB_levels(Bandit_RGBU.R++, Bandit_RGBU.G, Bandit_RGBU.B);
        busy_wait_us_32(50);
        goto start_adc_setup;
    }
    

    // Setup Sampling Pace Flags
    Sampling_Setup(500000);

    // Main Loop for Core 1
    //  All peripherals should be configured by now :)
    while(1){
        switch(CORE_1_STATE){
            case CORE_1_IDLE: {
                set_ULED_level(Bandit_RGBU.U++);
                busy_wait_ms(20);
                
                CORE_1_STATE = CORE_1_APPLY_SETTINGS;
            }
            break;
            case CORE_1_APPLY_SETTINGS: {
                
                CORE_1_STATE = CORE_1_SAMPLE;
            }
            break;
            case CORE_1_SAMPLE: {
                tmp_arr[0] = 0;
                Start_Sampling();   // You have 500 clocks between each flag, careful!
                
                for(uint_fast16_t n = 0; n < STD_MAX_SAMPLES; ++n){
                    while(!Sample_Now());
                    //Sample_Flag_Clear();
                    hw_clear_bits(&pwm_hw->intr, PWM_INTR_CH3_BITS);
                    ADS7253_Dual_Sampling(ADC_PIO, tmp_arr, &D_N_0[n], &X_N_0[n], 1);
                }

                Stop_Sampling();

                CORE_1_STATE = CORE_1_DOWNSAMPLE;
            }
            break;
            case CORE_1_DOWNSAMPLE: {
                for(int n = 0; n < STD_MAX_SAMPLES; ++n){
                    printf("%u\t%u\n", D_N_0[n], X_N_0[n]);
                }
                CORE_1_STATE = CORE_1_LMS;
            }
            break;
            case CORE_1_LMS: {

                CORE_1_STATE = CORE_1_POST_PROC;
            }
            break;
            case CORE_1_POST_PROC: {

                CORE_1_STATE = CORE_1_SHIP_RESULTS;
            }
            break;
            case CORE_1_SHIP_RESULTS: {

                CORE_1_STATE = CORE_1_IDLE;
            }
            break;
            default:
                // Idk should have some out of bounds safety, but whatever
                CORE_1_STATE = CORE_1_IDLE;
            break;
        }
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



  /////////////////////////////////////////////////////////
 ////////////////////// Generic Functions :) /////////////
/////////////////////////////////////////////////////////
