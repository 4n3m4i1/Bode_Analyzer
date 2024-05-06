/*
    Testing Defines
    DO NOT UNCOMMENT UNLESS YOU KNOW EXACTLY WHAT YOU'RE DOING!!!!!!
*/
//#define FORCE_SAMPLING_4_TESTING
#define NO_DEBUG_LED
//#define FORCESEND_TESTING
//#define FORCESEND_FAKETAPS
//#define FORCESEND_NOFFT
//#define FORCESEND_USE_REALDATA
//#define SEND_RAW_2_OUTPUT

#ifdef FORCESEND_USE_REALDATA
#include "realdata.h"
// For debug only
#include "dummy_taps.h"
#endif


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


  //////////////////////////////////////////////////////////////////////
 /////////////////////    GLOBAL ALLOCATIONS    ///////////////////////
//////////////////////////////////////////////////////////////////////
// Core 1 -> 0 transfer buffers
struct Transfer_Data {
    volatile bool       updated;
    volatile bool       calibration_data;
    volatile uint16_t   len;
    volatile Q15        data[TOTAL_ADAPTIVE_FIR_LEN];

    volatile uint32_t   lms_samples_processed;
    volatile Q15        lms_error;
    volatile uint16_t   lms_error_attempts;
};

struct Transfer_Data ICTXFR_A;


struct BANDIT_SETTINGS  Global_Bandit_Settings;



  //////////////////////////////////////////////////////////////////////
 /////////////////////    CORE 0 ALLOCATIONS    ///////////////////////
//////////////////////////////////////////////////////////////////////
// FFT Tap Buffers
CORE_0_MEM Q15 FR_BUFF[TOTAL_ADAPTIVE_FIR_LEN];
CORE_0_MEM Q15 FI_BUFF[TOTAL_ADAPTIVE_FIR_LEN];

// FFT Averaging buffer
CORE_0_MEM Q15 FFT_AVG_BUFF[TOTAL_ADAPTIVE_FIR_LEN];
CORE_0_MEM Q15 FFT_CAL_BUFF[TOTAL_ADAPTIVE_FIR_LEN];

// Reserve Buffer for Stored Results
CORE_0_MEM Q15 RESULTS_BUFFER[TOTAL_ADAPTIVE_FIR_LEN];


volatile CORE_0_MEM uint16_t USB_STATE;
volatile CORE_0_MEM uint16_t USB_NEXT_STATE;

volatile CORE_0_MEM bool SKIP_FFT = false;
volatile CORE_0_MEM bool SKIP_LMS = false;

CORE_0_MEM uint8_t  BS_RX_BF[BS_BF_LEN];
volatile CORE_0_MEM bool CORE_0_SKIP_FFT = false;
  //////////////////////////////////////////////////////////////////////
 /////////////////////    CORE 1 ALLOCATIONS    ///////////////////////
//////////////////////////////////////////////////////////////////////
struct BANDIT_LED_COLORS {
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t U;
};

struct CORE_1_MEM BANDIT_LED_COLORS Bandit_RGBU;

// Sampling Banks
CORE_1_MEM Q15 X_N_0[TOTAL_BUFF_LEN];
CORE_1_MEM Q15 D_N_0[TOTAL_BUFF_LEN];

// LMS FIR For Producing Y[n]
CORE_1_MEM Q15 LMS_FIR_BANK[TOTAL_ADAPTIVE_FIR_LEN + DEFAULT_LMS_TAP_LEN];
CORE_1_MEM Q15 LMS_H_HATS[TOTAL_ADAPTIVE_FIR_LEN + DEFAULT_LMS_TAP_LEN];

CORE_1_MEM Q15 LMS_H_HATS_AVG[TOTAL_ADAPTIVE_FIR_LEN + DEFAULT_LMS_TAP_LEN];

CORE_1_MEM Q15 LMS_H_HATS_CORRECTION[TOTAL_ADAPTIVE_FIR_LEN + DEFAULT_LMS_TAP_LEN];

struct CORE_1_MEM BANDIT_CORE_1_DEBUG_REPORT Bandit_Debug_Report;

// Core 1 Defines
#define ADCRES_BITS     12u
#define ADC_RES_CODES   (1u << ADCRES_BITS)
#define ADC_VREF_CODE   0x108                   // ~2.19 V from ADS7253 DAC

#define ADC_PIO         pio1
CORE_1_MEM uint16_t     tmp_arr[ADS_READ_REG_COUNT] = {0,0,0};

struct CORE_1_MEM LMS_Fixed_Inst DFL_LMS_Inst;


  //////////////////////////////////////////////////////////////////////
 ///////////////////  CORE 0 FUNCTION DEFINITIONS   ///////////////////
//////////////////////////////////////////////////////////////////////
static void core_0_main();

static void USB_Handler(struct FFT_PARAMS *fft);

static void send_header_packet(uint16_t *h_data);
static void send_f_packets(Q15 *data, uint16_t num_samples);

static inline void fft_setup(struct FFT_PARAMS *cool_fft, uint16_t len, uint16_t maxtablesize);
static inline void fft_clear_fi(struct FFT_PARAMS *cool_fft);


  //////////////////////////////////////////////////////////////////////
 ///////////////////  CORE 1 FUNCTION DEFINITIONS   ///////////////////
//////////////////////////////////////////////////////////////////////
static void     core_1_main();
static void     transfer_results_to_safe_mem(Q15 *src, struct Transfer_Data *dst, uint16_t len);
static void     clear_adc_read_buffers();


  //////////////////////////////////////////////////////////////////////
 ////////////////////////////    Code!    /////////////////////////////
//////////////////////////////////////////////////////////////////////
// Main
// Start up in Core 0
int main(){
    board_init();
    tud_init(BOARD_TUD_RHPORT);

    init_LED_pins();

    Bandit_RGBU.R = 0;
    Bandit_RGBU.G = 0;
    Bandit_RGBU.B = 0;
    Bandit_RGBU.U = 0;
    set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B);
    set_ULED_level(Bandit_RGBU.U);

    // Keep VDDA on, until sleep/low power modes are developed :)
    gpio_init(VDDA_EN_PAD);
    gpio_set_dir(VDDA_EN_PAD, GPIO_OUT);
    gpio_put(VDDA_EN_PAD, true);

    Setup_Semaphores();

    // Init h_hat core transfer register lengths to any known value.
    ICTXFR_A.len = 0;
    
    /*
        The AHB Lite cross bar is a 2:3 priority MUX that connects:
            - PIO0, PIO1, USB
        with the main AHB bus.

        USB falls in the lowest real time priority thus accesses to it
            (only Core 0) are regarded as lower priority.
        The access paths indicate wait-free access can be ensured for
            both white noise generation (DMA -> PIO0) and sampling
            (Core 1 <-> PIO1) if they have higher bus priority.
        
        Access allowance is done by gated high priority access, such that
            low priority access can occur. Sampling is not continuous,
            WGN generation is not continuous. But during their runtimes
            BOTH need 0 collisions.
    */

    // Allow Core 1 to have priority access to AHB Lite Bus
    //  This ensures the timing of sampling is deterministic
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);

    // Tenative allowance for High Priority DMA R/W access to AHB and APB
    //  buses for uninterrupted white noise generation.
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_DMA_W_BITS);
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_DMA_R_BITS);

    // Wait for bus priority changes to be applied.
    //  "In normal circumstances this occurs almost immediately"
    while(!bus_ctrl_hw->priority_ack);

    // Setup to count AHB-Lite MUX collisions
    bus_ctrl_hw->counter[0].value = 0;
    bus_ctrl_hw->counter[0].sel = arbiter_fastperi_perf_event_access_contested;


    // Initialize Status LEDs,
    //  Sequence Seen on Proper Init
    set_RGB_levels(BANDIT_PINK_SUPER_ARGUMENT);
    set_ULED_level(Bandit_RGBU.U = 10);

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
    // Claim lock on settings until Core 0 can deal with initialization
    //  prevents Core 1 from starting any processing without settings
    //  in place.
    
    Acquire_Lock_Blocking(INTERCORE_SETTINGS_LOCK);
    
    // Do not start updated to prevent useless DSP cycling
    Global_Bandit_Settings.updated = false;
    Global_Bandit_Settings.settings_bf = BANDIT_DFL_SETTINGS;
    Global_Bandit_Settings.manual_tap_len_setting = 0;
    Global_Bandit_Settings.manual_error_limit = 0;
    Global_Bandit_Settings.manual_freq_range = 0;
    Global_Bandit_Settings.manual_lms_offset = 0;
    Global_Bandit_Settings.skip_lms = false;
    CLR_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_AUTO_RUN);
    SET_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_AUTO_SEND);

    Release_Lock(INTERCORE_SETTINGS_LOCK);

    // Setup Structs and Default Parameters for FFT
    struct FFT_PARAMS cool_fft;
    fft_setup(&cool_fft, DEFAULT_LMS_TAP_LEN, (uint16_t)SINTABLESIZE);

    // Initialize tinyUSB with board and start listening for start char from GUI
    tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, START_CHAR);
    USB_NEXT_STATE = USB_INIT;
    tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, START_CHAR);
    // USB State Machine, Settings Application, FFT

    uint32_t pace_timer = timer_hw->timerawl;
    //uint32_t pace_time_limit = 31250;
    uint32_t pace_time_limit = 100000;

    bool is_calibration_data = false;
    Q15 calibration_max = 0;

    while(1){
        tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, START_CHAR);
        tud_task();
        USB_STATE = USB_NEXT_STATE;

        //volatile uint32_t spinlock_irq_status_A;

        switch(USB_STATE) {
            case USB_INIT: {
                //need idle work until GUI is ready
                tud_task();
                USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
            }
            break;
            
            case USB_APPLY_SETTINGS: {
                USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
            }
            break;

            case USB_FFT_DATA_COLLECT: {
                // Acquire safe access to FFT mem, can use 2 locks for ping pong access
                tud_task();

                struct Transfer_Data *data_src;

                Acquire_Lock_Blocking(INTERCORE_FFTMEM_LOCK_A);
                pace_timer = timer_hw->timerawl;
                data_src = &ICTXFR_A;
                tud_task();
                
                if(!data_src->updated){
                    // FFT Data wasn't updated, shouldn't happen given semaphore
                    //  pacing structure
                    Release_Lock(INTERCORE_FFTMEM_LOCK_A);
                    USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
                } else {
                    // If there is new FFT data to be had
                    data_src->updated = false;
                    // If data is all zeros (Core 1 probably gave up on convergence)
                    //  or the length is zero (P R O B L E M)
                    //  then don't run the FFT
                    bool allzeros = true;

                    is_calibration_data = data_src->calibration_data;

                    if(data_src->len && !SKIP_FFT){
                        // Apply windowing here if needed!!!
                        
                        fft_setup(&cool_fft, data_src->len, (uint16_t)SINTABLESIZE);

                        for(uint16_t n = 0; n < data_src->len; ++n){
                            if((cool_fft.fr[n] = data_src->data[n])) allzeros = false;
                            cool_fft.fi[n] = 0;
                        }
                    }

#ifdef FORCESEND_TESTING 
                    USB_NEXT_STATE = USB_RUN_DMC_JK_RUN_FFT;
#else
                    if(allzeros && !SKIP_FFT){
                        Release_Lock(INTERCORE_FFTMEM_LOCK_A);
                        USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
                    } else {
                        USB_NEXT_STATE = USB_RUN_DMC_JK_RUN_FFT;
                    } 
#endif     
                }
            }
            break;
            
            case USB_RUN_DMC_JK_RUN_FFT: {
                tud_task();
                if(!SKIP_FFT){
                    FFT_fixdpt(&cool_fft);      // Run FFT
                    FFT_mag(&cool_fft);         // Extract magnitude, output in fft.fr
                    cool_fft.num_samples >>= 1; // Print only half of fft result, dodge reflected stuff
                    
                    if(is_calibration_data){
                        Q15 maxval = 0;
                        for(uint_fast16_t n = 0; n < cool_fft.num_samples; ++n){
                            if(cool_fft.fr[n] > maxval) maxval = cool_fft.fr[n];
                            //FFT_CAL_BUFF[n] = cool_fft.fr[n];
                            FFT_AVG_BUFF[n] = 0;
                        }

                        for(uint_fast16_t n = 0; n < cool_fft.num_samples; ++n){
                            FFT_CAL_BUFF[n] = maxval - cool_fft.fr[n];
                        }
                        
                        calibration_max = maxval;
                        
                    } else {
                        if(SKIP_LMS){
                            Q15 maxval = 0;
                            for(uint_fast16_t n = 0; n < cool_fft.num_samples; ++n) if(cool_fft.fr[n] > maxval) maxval = cool_fft.fr[n];

                            Q15 normalize_calibration = div_Q15(calibration_max, maxval);

                            for(uint_fast16_t n = 0; n < cool_fft.num_samples; ++n){
                                FFT_AVG_BUFF[n] += cool_fft.fr[n];
                                FFT_AVG_BUFF[n] >>= 1;
                                cool_fft.fr[n] = FFT_AVG_BUFF[n];
                                //cool_fft.fr[n] -= mul_Q15(FFT_CAL_BUFF[n], normalize_calibration);
                                cool_fft.fr[n] -= mul_Q15(normalize_calibration, FFT_CAL_BUFF[n]);
                                cool_fft.fr[n] <<= 3;   

                            }
                        } else {
                            
                        }
                        
                    }
                } else {
                    for(uint16_t n = 0; n < ICTXFR_A.len; ++n){
                        cool_fft.fr[n] = ICTXFR_A.data[n];
                        // Debug
                        //cool_fft.fr[n] = (Q15)n;
                    }    
                    cool_fft.num_samples = ICTXFR_A.len;
                }
                

#ifdef FORCESEND_NOFFT
                for(uint16_t n = 0; n < ICTXFR_A.len; ++n){
                        cool_fft.fr[n] = ICTXFR_A.data[n];
                }
#else
                
                
                // Raw Core 1 data bypass
                //if(CORE_0_SKIP_FFT){
                //    for(uint16_t n = 0; n < ICTXFR_A.len; ++n){
                //        cool_fft.fr[n] = ICTXFR_A.data[n];
                //    }
                //}
#endif

                // Free memory constraints
                //  stall for limiting packets per second
                if(!SKIP_FFT) while(timer_hw->timerawl - pace_timer < pace_time_limit) tud_task();
                // LMS Sends 2x as many bytes as usual FFT packets, so slow down A LOT
                else while(timer_hw->timerawl - pace_timer < (pace_time_limit << 1)) tud_task();
                Release_Lock(INTERCORE_FFTMEM_LOCK_A);
                tud_task();

                if(is_calibration_data) USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
                else USB_NEXT_STATE = USB_SEND_TUSB;
            }
            break;

            case USB_SEND_TUSB: {
                // Transmit FFT Data to computer
                tud_task();
#ifdef FORCESEND_TESTING  
#ifndef FORCESEND_NOFFT
                // knock down giant DC peak...
                //for(uint16_t n = 0; n < cool_fft.log2_num_samples - 3; ++n){
                //    cool_fft.fr[n] = cool_fft.fr[cool_fft.log2_num_samples - 1];
                //}
                cool_fft.fr[0] = cool_fft.fr[1];
#endif                
                send_f_packets(cool_fft.fr, cool_fft.num_samples);
                tud_task();
                busy_wait_ms(100);
#else
                //if(!SKIP_FFT) cool_fft.fr[0] = cool_fft.fr[1];
                USB_Handler(&cool_fft);     //send data to GUI through tusb
#endif
                USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
            }
            break;

            case USB_SEND_CORE_DEBUG: {
                // indicate to CORE 1 debug report is wanted
                Acquire_Lock_Blocking(INTERCORE_CORE_1_DBG_LOCK);

                // idk yet

                Release_Lock(INTERCORE_CORE_1_DBG_LOCK);
                USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
            }
            break;
            
            default: {
                USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
            }
            break; //need debug for USB_STATE error

        }

        
    }
}

  //////////////////////////////////////////////////////////////////////
 /////////////////    Core 1 Main Loop    /////////////////////////////
//////////////////////////////////////////////////////////////////////
static void core_1_main(){
    busy_wait_us_32(1000);
    Acquire_Lock_Blocking(INTERCORE_CORE_1_DBG_LOCK);
    Bandit_Debug_Report.epoch_core_1_boot_us = timer_hw->timelr;
    uint8_t Bandit_Calibration_State = BANDIT_UNCALIBRATED;
    Q15 Bandit_DC_Offset_Cal = 0;
    
    
    DFL_LMS_Inst.tap_len = DEFAULT_LMS_TAP_LEN;
    DFL_LMS_Inst.max_convergence_attempts = 4;

    // Setup LMS controller and buffering
    struct LMS_Fixed_Inst LMS_Inst;
    
    LMS_Struct_Init(&DFL_LMS_Inst, 
                        0x0666, 
                        0x2000, 
                        0, 
                        STD_MAX_SAMPLES, 
                        0
                        ); // Errors are just straight guesses rn, offset as well
    DFL_LMS_Inst.tap_len = DEFAULT_LMS_TAP_LEN;
    DFL_LMS_Inst.d_n = D_N_0;
    DFL_LMS_Inst.x_n = X_N_0;
    DFL_LMS_Inst.iteration_ct = STD_MAX_SAMPLES;
    DFL_LMS_Inst.d_n_offset = 0;
    DFL_LMS_Inst.learning_rate = 0x0020;    // 0.001
    DFL_LMS_Inst.averaging_counts_limit = 2;
    //DFL_LMS_Inst.learning_rate = 0x0003;    // 0.0001
    //DFL_LMS_Inst.learning_rate = 0x0008;    // 0.00025

    // Set default settings to local struct copy
    LMS_Struct_Equate(&DFL_LMS_Inst, &LMS_Inst);

    // Setup FIR component of the LMS
    struct Q15_FIR_PARAMS LMS_FIR;
    setup_Q15_FIR(&LMS_FIR, DEFAULT_LMS_TAP_LEN);
    LMS_FIR.size_true = TOTAL_ADAPTIVE_FIR_LEN;
    LMS_FIR.data = LMS_FIR_BANK;
    LMS_FIR.taps = LMS_H_HATS;
    
    LMS_Inst.max_convergence_attempts = 4;
    LMS_Inst.averaging_counts_limit = 2;
    LMS_Inst.averaging_counts = 0;

    /*
        For Tap Length Update:
            - Set LMS_Inst.tap_len = len
            - setup_Q15_FIR(&LMS_FIR, len)
            - len must be power of 2
    */

    Q15 DOWNSAMPLE_SAMP_BUFFR[DOWNSAMPLE_LEN];

    struct Q15_FIR_PARAMS CUT_125KHZ;
    setup_Q15_FIR(&CUT_125KHZ, DOWNSAMPLE_LEN);
    CUT_125KHZ.size_true = DOWNSAMPLE_LEN;
    CUT_125KHZ.taps = h125_taps;
    CUT_125KHZ.data = DOWNSAMPLE_SAMP_BUFFR;
   
    struct Q15_FIR_PARAMS CUT_62KHZ;
    setup_Q15_FIR(&CUT_62KHZ, DOWNSAMPLE_LEN);
    CUT_62KHZ.size_true = DOWNSAMPLE_LEN;
    CUT_62KHZ.taps = h62_taps;
    CUT_62KHZ.data = DOWNSAMPLE_SAMP_BUFFR;

    struct Q15_FIR_PARAMS CUT_31KHZ;
    setup_Q15_FIR(&CUT_31KHZ, DOWNSAMPLE_LEN);
    CUT_31KHZ.size_true = DOWNSAMPLE_LEN;
    CUT_31KHZ.taps = h31_taps;
    CUT_31KHZ.data = DOWNSAMPLE_SAMP_BUFFR;


    /*
        Initiate frontend PGA/MUX
            Set channel 0 (D_N), 1x gain
    */
    spi_inst_t *mcp_spi = spi1;
    MCP6S92_Init(mcp_spi, PGA_CSN_PAD, PGA_SCK_PAD, PGA_SI_PAD);
    // Don't init frontend PGA until calibration
    //MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_0, PGA_CSN_PAD);
    //MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);
    

    /*
        Initialize ADC SPI w/ pio1
    */
    pio_ads7253_spi_init(pio1, 16, 1, 0, ADC_CSN_PAD, ADC_SDI_PAD, ADC_SDO_A_PAD, 0);

    Bandit_RGBU.B = 0;
    Bandit_RGBU.R = 0;

#ifdef FORCE_SAMPLING_4_TESTING
    goto debug_no_adc_setup_label;
#endif

    uint32_t banditsetup_adc_attempts = 0;
start_adc_setup:
    banditsetup_adc_attempts++;
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

    busy_wait_us_32(150);
    
    // Read the CFR and ensure it's lookin good before proceeding
    tmp_arr[0] = ADS7253_CMD(ADS7253_CFR_READ, 0);
    tmp_arr[1] = 0;
    tmp_arr[2] = 0;
    
    // Ask for CFR readback twice just to be sure the data is latched
    //  and CFR is updated.
    ADS7253_write16_blocking(ADC_PIO, tmp_arr, ADS_READ_REG_COUNT);
    busy_wait_us_32(150);
    ADS7253_write16_blocking(ADC_PIO, tmp_arr, ADS_READ_REG_COUNT);
    busy_wait_us_32(150);
    
    while(!pio_sm_is_rx_fifo_empty(ADC_PIO, ADS_PIO_SDOA_SM)){
        tmp_arr[0] = ADC_PIO->rxf[ADS_PIO_SDOA_SM];
        tmp_arr[1] = ADC_PIO->rxf[ADS_PIO_SDOB_SM];
        
        // okay like 2624 is hacky, but it is the expected CFR value
        //  EVEN THOUGH IT SHOULDNT BE THAT
        //  Through all testing that's been the correct return.
        //  This DOES NOT match the datasheet, but through testing is
        //  legit... so idk dude, sendit - Joseph 03/31/2024
        if(tmp_arr[0] == 2624){
            // Fill out debug report
            Bandit_Debug_Report.epoch_ads_init_complete_us = timer_hw->timelr;
            Bandit_Debug_Report.adc_init_cfr_readback = tmp_arr[0];
            Bandit_Debug_Report.adc_init_attempts = banditsetup_adc_attempts;

            banditsetup_adc_attempts = 0;   // Set null, indicating successful match
            
            break;
        }
    }


    if(banditsetup_adc_attempts){
        busy_wait_us_32(150);
        goto start_adc_setup;
    }

    // Go Red for DAC calibration cycle
    set_RGB_levels(Bandit_RGBU.R = 200, Bandit_RGBU.G = 0, Bandit_RGBU.B = 0);
    set_ULED_level(Bandit_RGBU.U = 0);

    // ADC Now configured for 16clk mode dual SDO, configure refdac now!
    uint16_t ADC_REFDACVAL = 0x108;
    uint32_t ADC_REFDAC_CAL_ACCUM;
    tmp_arr[1] = 0;
    tmp_arr[2] = 0;

    // Limit of how many times a calibration approaching an exact value can be done.
    const uint16_t DAC_CAL_EXACT_LIMIT = 128;
    const uint16_t DAC_CAL_MAX_ATTEMPTS = 1024;
    uint16_t DAC_CAL_ATTEMPTS = 0;

    // Refdac init at 0x1FF -> 2.5V, we want some fraction of this estimated at 0x108 (2.19V)
start_refdac_cal:
    ADC_REFDAC_CAL_ACCUM = 0;
    tmp_arr[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, ADC_REFDACVAL << 3);
    ADS7253_write16_blocking(ADC_PIO, tmp_arr, ADS_WRITE_DAC_COUNT);

    // ADS7253 Section 7.8: t_refon = 8ms settling time for REFDAC
    //  go 10ms just to be sure
    busy_wait_us_32(10000);

    // Sample a lot to reach steady state
    for(uint16_t n = 0; n < 128; ++n){
        ADS7253_Dual_Sampling(ADC_PIO, tmp_arr, (uint16_t *)&D_N_0[n], (uint16_t *)&X_N_0[n], 1);
        busy_wait_us_32(5);
    }

    // Clear any read values
    clear_adc_read_buffers();

    for(uint16_t n = 0; n < 128; ++n){
        ADS7253_Dual_Sampling(ADC_PIO, tmp_arr, (uint16_t *)&D_N_0[n], (uint16_t *)&X_N_0[n], 1);
        ADC_REFDAC_CAL_ACCUM += (uint16_t)D_N_0[n];
        busy_wait_us_32(5);
    }

    // Clear any read values
    clear_adc_read_buffers();

    // Get average value of all 128 samples collected
    //  over a 896us period idk man it's a cool number
    ADC_REFDAC_CAL_ACCUM >>= 7;

    // Calibration Logic:
    //  The DC bias network on the ADC ideally is at VDDA / 2, thus
    //      the reference DAC (REFDAC) should be equal to VDDA / 2,
    //      This is due to the REFDAC * 2 setting in the CFR register.
    //      The ideal code if the REFDAC is perfect is the midcode of the ADC.
    //  We cannot 100% guarantee these values, so we have the MCU figure it out.
    //
    //  If the reference REFDAC is too low: the code
    //      read from the ADC will be ABOVE the midcode.
    //      Thus: Raise the DAC value
    //
    //  If the REFDAC output is too high: the code 
    //      read from the ADC will be BELOW the midcode.
    //      Thus: Lower the DAC value
    //
    //  Attempts at perfection are made, but highly unlikely to be met.
    //  Eventually a fallback to a +/- 1 range around the midcode is done.
    if( (ADC_REFDAC_CAL_ACCUM != (ADS_MID_CODE_BINARY)) && 
        (DAC_CAL_ATTEMPTS++ < DAC_CAL_EXACT_LIMIT)){
            if(ADC_REFDAC_CAL_ACCUM < ADS_MID_CODE_BINARY) ADC_REFDACVAL--;
            else if(ADC_REFDAC_CAL_ACCUM > ADS_MID_CODE_BINARY) ADC_REFDACVAL++;
            goto start_refdac_cal;
    } else
    if(ADC_REFDAC_CAL_ACCUM < (ADS_MID_CODE_BINARY - 1) &&
        (DAC_CAL_ATTEMPTS++ < DAC_CAL_MAX_ATTEMPTS)){  // If we can't hit exact fallback to +/-1 of exact
            // ADC Reference is HIGH
            ADC_REFDACVAL--;
            goto start_refdac_cal;
    } else
    if(ADC_REFDAC_CAL_ACCUM > (ADS_MID_CODE_BINARY + 1) &&
        (DAC_CAL_ATTEMPTS++ < DAC_CAL_MAX_ATTEMPTS)){
            // ADC Reference is LOW
            ADC_REFDACVAL++;
            goto start_refdac_cal;
    }

    // At this point the DAC value on channel A should be good
    //  Set Channel B such that the relative gain of each channel is 
    //  as close to the same as we can ensure given tolerance.
    //  i.e. set DAC references to be equal. DC offsets in the bias network
    //      will be corrected for later.
    tmp_arr[0] = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, ADC_REFDACVAL << 3);
    ADS7253_write16_blocking(ADC_PIO, tmp_arr, ADS_WRITE_DAC_COUNT);
    
    // Fill out Debug Report
    if(DAC_CAL_ATTEMPTS < DAC_CAL_EXACT_LIMIT) Bandit_Debug_Report.dac_cal_state = BDBG_DAC_EXACT_CAL;
    else Bandit_Debug_Report.dac_cal_state = BDBG_DAC_RANGE_CAL;
    Bandit_Debug_Report.dac_cal_value = ADC_REFDACVAL;
    Bandit_Debug_Report.dac_cal_attempts = DAC_CAL_ATTEMPTS;
    Bandit_Debug_Report.epoch_dac_cal_complete_us = timer_hw->timelr;

    busy_wait_us_32(10000);

    // Clear any read values
    clear_adc_read_buffers();
    

    // Go Purple for DAC cal complete
    set_RGB_levels(Bandit_RGBU.R = 127, Bandit_RGBU.G = 0, Bandit_RGBU.B = 127);
    set_ULED_level(Bandit_RGBU.U = 64);
    

    // Core State Machine Settings
    bool SET_STALL          = false;
    volatile uint16_t CORE_1_STATE   = CORE_1_IDLE;
    uint16_t error_attempts = 0;                // How many times the LMS algorithm has failed to converge
    bool CORE_1_DBG_MODE    = false;            // Are we in debug mode?
    bool CORE_1_WGN_STATE   = false;            // Is WGN always on?
    uint8_t CORE_1_FRANGE   = DOWNSAMPLE_1X_250K_CUT;
    bool CORE_1_SEND_UNPROCESSED = false;
    bool CORE_1_SEND_INTERLEAVED = false;
    bool sending_calib_data = true;

    // Configure- White Noise Generation Signal Chain
    // Setup AWGN Generation from overdriven ROSC -> DMA -> PIO
    setup_rosc_full_tilt();
    setup_PIO_for_switching();
    setup_chained_dma_channels();

    //volatile uint32_t spinlock_irq_status_B = spin_lock_blocking(SETTINGS_LOCK);
    
    Acquire_Lock_Blocking(INTERCORE_SETTINGS_LOCK);
    if(CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_WGN_ON)){
        start_wgn_dma_channels();
        start_randombit_dma_chain(dma_awgn_ctrl_chan);
        CORE_1_WGN_STATE = true;
    }
    //spin_unlock(SETTINGS_LOCK, spinlock_irq_status_B);
    Release_Lock(INTERCORE_SETTINGS_LOCK);

    for(uint16_t n = 0; n < count_of(LMS_H_HATS_AVG); ++n) LMS_H_HATS_AVG[n] = 0;

    // Main Loop for Core 1
    //  All peripherals should be configured by now :)

    // FOR TESTING ONLY
#ifdef FORCE_SAMPLING_4_TESTING
debug_no_adc_setup_label:
    CORE_1_DBG_MODE = true;
    CORE_1_STATE = CORE_1_DEBUG_HANDLER;
#endif


    while(1){
        switch(CORE_1_STATE){
            case CORE_1_IDLE: {
#ifndef NO_DEBUG_LED
                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 0, Bandit_RGBU.B = 0);
#endif
                // If starting conditions are met start the whole DSP cycle
                if(Bandit_Calibration_State == BANDIT_UNCALIBRATED){
                    Bandit_Calibration_State = BANDIT_CAL_DC_BIAS_IN_PROG;
                    CORE_1_STATE = CORE_1_SAMPLE;
                } else {
                    error_attempts = 0;
                    //flush_FIR_buffer_and_taps(&LMS_FIR);

                    //set_ULED_level(Bandit_RGBU.U = 0);
                    
                    LMS_Inst.samples_processed = 0;
                    LMS_Inst.averaging_counts = 0;

                    CORE_1_STATE = CORE_1_SET_SETTINGS;
                }

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;

            case CORE_1_SET_SETTINGS: {
//#ifndef NO_DEBUG_LED
                set_RGB_levels(BANDIT_PINK_SUPER_ARGUMENT);
//#endif
                Acquire_Lock_Blocking(INTERCORE_SETTINGS_LOCK);

                if(Global_Bandit_Settings.updated){
                    // if white noise should be on, and isn't, turn it on
                    if(CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_WGN_ON) && !CORE_1_WGN_STATE){
                        start_wgn_dma_channels();
                        start_randombit_dma_chain(dma_awgn_ctrl_chan);
                        CORE_1_WGN_STATE = true;
                    }

                    // Configure next LMS run
                    CORE_1_FRANGE = Global_Bandit_Settings.manual_freq_range;
                    LMS_Inst.max_error_allowed = Global_Bandit_Settings.manual_error_limit;
                    LMS_Inst.tap_len = Global_Bandit_Settings.manual_tap_len_setting;

                    if(LMS_Inst.tap_len > MAX_TAPS || LMS_Inst.tap_len < DEFAULT_LMS_TAP_LEN) LMS_Inst.tap_len = DEFAULT_LMS_TAP_LEN;

                    LMS_Inst.max_convergence_attempts = get_log_2(LMS_Inst.tap_len);

                    if(!LMS_Inst.max_convergence_attempts || LMS_Inst.max_convergence_attempts > 10) LMS_Inst.max_convergence_attempts = 4;

//#define BANDIT_SHIFT_HT     ((int16_t)((LMS_Inst.tap_len >> 1) + (LMS_Inst.tap_len >> 2) + (LMS_Inst.tap_len >> 3)))
#define BANDIT_SHIFT_HT     ((int16_t)((LMS_Inst.tap_len >> 1)))

                    if(((Global_Bandit_Settings.manual_lms_offset < 0) ? Global_Bandit_Settings.manual_lms_offset * -1 : Global_Bandit_Settings.manual_lms_offset) 
                            > BANDIT_SHIFT_HT) {
                                Global_Bandit_Settings.manual_lms_offset = 0;
                            }

                    // Shift by num taps / 2 + some manual offset 04/24/2024 jdv
                    LMS_Inst.d_n_offset = Global_Bandit_Settings.manual_lms_offset + BANDIT_SHIFT_HT;

#undef BANDIT_SHIFT_HT

                    LMS_Inst.max_convergence_attempts = Global_Bandit_Settings.manual_lms_attempts;

                    LMS_Inst.learning_rate = Global_Bandit_Settings.manual_learning_rate;

                    if(!LMS_Inst.learning_rate) LMS_Inst.learning_rate = 0x0003;

                    setup_Q15_FIR(&LMS_FIR, LMS_Inst.tap_len);

                    for(uint16_t n = 0; n < count_of(LMS_H_HATS_AVG); ++n) LMS_H_HATS_AVG[n] = 0;

                    LMS_FIR.curr_zero = 0;

                    CORE_1_SEND_UNPROCESSED = Global_Bandit_Settings.skip_lms;

                    // Delete me if problem
                    flush_FIR_buffer_and_taps(&LMS_FIR);

                    Bandit_Calibration_State = BANDIT_CAL_AA_TXFR_FUNC_IN_PROG;
                    set_ULED_level(Bandit_RGBU.U = 0);
                    Global_Bandit_Settings.updated = false;
                    sending_calib_data = true;

                    if(CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_AUTO_RUN) || 
                        CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_SINGLE_SHOT_RUN)) CORE_1_STATE = CORE_1_APPLY_SETTINGS;
                } else {
                    if(CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_AUTO_RUN) || 
                        CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_SINGLE_SHOT_RUN)){
                            CLR_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_SINGLE_SHOT_RUN);
                            // Auto run or a single shot run has been requested
                            CORE_1_STATE = CORE_1_APPLY_SETTINGS;
                    } else {
                        // If auto run is off, just kinda hang out until either
                        //  auto run is toggled or a single shot request comes in
                        SET_STALL = true;
                    }
                }

                Release_Lock(INTERCORE_SETTINGS_LOCK);

                if(SET_STALL){
                    busy_wait_ms(50);
                    SET_STALL = 0;
                }

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;

            case CORE_1_APPLY_SETTINGS: {
#ifndef NO_DEBUG_LED
                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 0, Bandit_RGBU.B = 255);
#endif
                if(Bandit_Calibration_State == BANDIT_CAL_AA_TXFR_FUNC_IN_PROG){
                    // if settings changed:
                    //  Bandit_Calibration_State = BANDIT_CAL_AA_TXFR_FUNC_IN_PROG
                    // Switch frontend PGA to White Noise Loopback
                    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_1, PGA_CSN_PAD);
                    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);
                    busy_wait_us_32(MCP6S92_SETTLING_TIME);
                } else
                if(CORE_1_WGN_STATE) {
                    // If WGN State toggled
                    // PGA Settling Time, if WGN already on. If WGN is being tuned on its own delay will deal w this
                    busy_wait_us_32(MCP6S92_SETTLING_TIME);
                } else
                if(!CORE_1_WGN_STATE){
                    // Start white noise for loopback testing
                    //stop_randombit_dma_chain(dma_awgn_data_chan);
                    stop_wgn_dma_channels();
                    busy_wait_us(1);
                    start_wgn_dma_channels();
                    start_randombit_dma_chain(dma_awgn_ctrl_chan);
                    busy_wait_us(WGN_PROP_TIME_US);                     // Allow time to deal with RC time constants
                
                    // 5ms additional delay to ensure white noise settles
                    busy_wait_us_32(5000);
                }

                LMS_Inst.error = 0;
                //for(uint16_t n = 0; n < count_of(LMS_FIR_BANK); ++n){
                //    LMS_FIR.taps[n] = 0;
                //    LMS_FIR.data[n] = 0;
                //}

                CORE_1_STATE = CORE_1_SAMPLE;

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;

            case CORE_1_SAMPLE: {
#ifndef NO_DEBUG_LED
                set_RGB_levels(Bandit_RGBU.R = 255, Bandit_RGBU.G = 255, Bandit_RGBU.B = 255);
#endif

                tmp_arr[0] = 0;
                uint32_t NVIC_ISR_EN = nvic_hw->icer;

                // No interrupts :)
                nvic_hw->icer = 0ul;

                //Start_Sampling();   // You have 500 clocks between each flag, careful!
                
                for(uint_fast16_t n = 0; n < STD_MAX_SAMPLES; ++n){
                    //while(!Sample_Now());
                    //Sample_Flag_Clear();
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    //asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    //asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    //asm("nop"); asm("nop"); asm("nop"); asm("nop");
                    //asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
                    //asm("nop"); asm("nop"); asm("nop");
                    ADS7253_Dual_Sampling(ADC_PIO, tmp_arr, (uint16_t *)&D_N_0[n], (uint16_t *)&X_N_0[n], 1);
                    X_N_0[n] -= Bandit_DC_Offset_Cal;
                    X_N_0[n] -= (Q15)ADS_MID_CODE_BINARY;   // Shift from binary output to 2s complement
                    D_N_0[n] -= (Q15)ADS_MID_CODE_BINARY;   // Shift from binary output to 2s complement

                    X_N_0[n] *= 16;
                    D_N_0[n] *= 16;
                }

                //Stop_Sampling();

                // Interrupts okay :)
                nvic_hw->icer = NVIC_ISR_EN;

                if(Bandit_Calibration_State > BANDIT_CAL_DC_BIAS_COMPLETE){
                    CORE_1_STATE = CORE_1_DOWNSAMPLE;
                } else {
                    CORE_1_STATE = CORE_1_APPLY_DC_CORRECTION;
                }
                
                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;

            case CORE_1_APPLY_DC_CORRECTION: {  // One time startup calibration routine
                int32_t dcavg_zone = 0;
                for(uint16_t n = 0; n < STD_MAX_SAMPLES; ++n){
                    dcavg_zone += ((int32_t)X_N_0[n] - (int32_t)D_N_0[n]);
                } 
                dcavg_zone >>= LOG2_STD_MAX_SAMPLES;
                Bandit_DC_Offset_Cal = (Q15)dcavg_zone;
                Bandit_Calibration_State = BANDIT_CAL_DC_BIAS_COMPLETE;

                // Go white for DC Cal done, Idle return
                set_RGB_levels(Bandit_RGBU.R = 127, Bandit_RGBU.G = 127, Bandit_RGBU.B = 127);
               // busy_wait_ms(1000);
                //set_ULED_level(Bandit_RGBU.U = USER_LED_BANDIT_DC_CAL_DONE);

                CORE_1_STATE = CORE_1_IDLE;
            }
            break;
            
            case CORE_1_DOWNSAMPLE: {
#ifndef NO_DEBUG_LED
                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 127, Bandit_RGBU.B = 127);
#endif
                struct Q15_FIR_PARAMS *torun = 0;
                // FIR go here
                switch(CORE_1_FRANGE){
                    case DOWNSAMPLE_2X_125K_CUT:
                        torun = &CUT_125KHZ;

                        flush_FIR_buffer(&CUT_125KHZ);

                        LMS_Inst.fixed_offset = DOWNSAMPLE_LEN;
                        
                        LMS_Inst.ddsmpl_stride = 2;
                        LMS_Inst.ddsmpl_shift = 1;
                    break;
                    case DOWNSAMPLE_4X_62K5_CUT:
                        torun = &CUT_62KHZ;

                        flush_FIR_buffer(&CUT_62KHZ);
                        
                        LMS_Inst.fixed_offset = DOWNSAMPLE_LEN;
                        
                        LMS_Inst.ddsmpl_stride = 4;
                        LMS_Inst.ddsmpl_shift = 2;
                    break;
                    case DOWNSAMPLE_8X_32K2_CUT:
                        torun = &CUT_31KHZ;
                        
                        flush_FIR_buffer(&CUT_31KHZ);

                        LMS_Inst.fixed_offset = DOWNSAMPLE_LEN;
                        
                        LMS_Inst.ddsmpl_stride = 8;
                        LMS_Inst.ddsmpl_shift = 3;
                    break;

                    case DOWNSAMPLE_1X_250K_CUT:
                    default:
                        torun = 0;
                        LMS_Inst.fixed_offset = 0;
                        LMS_Inst.ddsmpl_stride = 1;
                        LMS_Inst.ddsmpl_shift = 0;
                        
                        LMS_Inst.d_n = D_N_0;
                        LMS_Inst.x_n = X_N_0;
                       // goto skip_downsampling_label;
                    break;
                }
                
                if(torun){
                    uint16_t n;
                    torun->curr_zero = 0;
                    for(n = 0; n < DOWNSAMPLE_LEN; ++n){
                        run_2n_FIR_cycle(torun, D_N_0[n]);
                    }
                    for(; n < STD_MAX_SAMPLES; ++n){
                        D_N_0[n - DOWNSAMPLE_LEN] = run_2n_FIR_cycle(torun, D_N_0[n]);
                    }

                    torun->curr_zero = 0;
                    for(n = 0; n < DOWNSAMPLE_LEN; ++n){
                        run_2n_FIR_cycle(torun, X_N_0[n]);
                    }
                    for(; n < STD_MAX_SAMPLES; ++n){
                        X_N_0[n - DOWNSAMPLE_LEN] = run_2n_FIR_cycle(torun, X_N_0[n]);
                    }
                }

                if(CORE_1_SEND_UNPROCESSED){
                    if(LMS_Inst.ddsmpl_stride > 1){
                        uint16_t idx = 0;
                        for(uint16_t n = 0; n < STD_MAX_SAMPLES; n += LMS_Inst.ddsmpl_stride){
                            D_N_0[idx++] = D_N_0[n];
                        }
                        LMS_Inst.tap_len = STD_MAX_SAMPLES >> LMS_Inst.ddsmpl_shift;
                    }
                    
                    CORE_1_STATE = CORE_1_POST_PROC;
                }
                else CORE_1_STATE = CORE_1_LMS;

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;

            case CORE_1_LMS: {
                set_RGB_levels(Bandit_RGBU.R = 255, Bandit_RGBU.G = 127, Bandit_RGBU.B = 0);

                //Q15 LMS_Error = LMS_Looper(&LMS_Inst, &LMS_FIR);
                LMS_Looper(&LMS_Inst, &LMS_FIR);

                //if(LMS_Error != LMS_OK){
                    // Handle the error
                    //  Sample again and try again
                    CORE_1_STATE = CORE_1_SAMPLE;
                //} else {
                //    CORE_1_STATE = CORE_1_POST_PROC;

                //    set_RGB_levels(Bandit_RGBU.R = 255, Bandit_RGBU.G = 0, Bandit_RGBU.B = 255);
                //}

                // If we've errored out too many times...
                if(++error_attempts > LMS_Inst.max_convergence_attempts){
                    set_RGB_levels(Bandit_RGBU.R = 255, Bandit_RGBU.G = 0, Bandit_RGBU.B = 0);
                    CORE_1_STATE = CORE_1_POST_PROC;
                } 

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;

            case CORE_1_POST_PROC: {
#ifndef NO_DEBUG_LED
                set_RGB_levels(Bandit_RGBU.R = 255, Bandit_RGBU.G = 32, Bandit_RGBU.B = 255);
#endif
                if(Bandit_Calibration_State != BANDIT_FULLY_CALIBRATED){
                    // Apply transfer function correction
                    // Apply correction in time domain, it's linear idk man should be right
                    
                    for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                        LMS_H_HATS_CORRECTION[n] = LMS_FIR.taps[n];
                    }
                    

                    // Set PGA to EXTERNAL input x1 GAIN
                    if(++LMS_Inst.averaging_counts >= LMS_Inst.averaging_counts_limit){
                        MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_0, PGA_CSN_PAD);
                        MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);
    
                        // PGA Settling Time
                        busy_wait_us_32(MCP6S92_SETTLING_TIME);
    
                        Bandit_Calibration_State = BANDIT_FULLY_CALIBRATED;
                    
                        sending_calib_data = true;

                        // Indicate full CAL with green LEDs
                        set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 127, Bandit_RGBU.B = 0);
                        //set_ULED_level(Bandit_RGBU.U = 255);
                    }

                    
                    
                    // Calibration Acquired, go back to do first run
                    if(LMS_Inst.averaging_counts){
                        for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                            LMS_H_HATS_AVG[n] = (LMS_H_HATS_AVG[n] + LMS_FIR.taps[n]) >> 1;
                            //LMS_FIR.taps[n] -= LMS_H_HATS_CORRECTION[n];
                        }
                    } else {
                        for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                            LMS_H_HATS_AVG[n] = LMS_FIR.taps[n];
                        }
                    }

                    if(LMS_Inst.averaging_counts >= LMS_Inst.averaging_counts_limit) CORE_1_STATE = CORE_1_SHIP_RESULTS;
                    else CORE_1_STATE = CORE_1_SAMPLE;

                    //CORE_1_STATE = CORE_1_APPLY_SETTINGS;
                } else {
                    
                    // Select what data is shipped over to core 0
                    sending_calib_data = false;

                    if(CORE_1_SEND_UNPROCESSED){
                        set_ULED_level(Bandit_RGBU.U = 255);
                        if(CORE_1_SEND_INTERLEAVED) {
                            // Send both D_N and X_N data, D_N first then X_N each filling half the buffer
                            for(uint16_t n = 0; n < LMS_Inst.tap_len >> 1; ++n){
                                LMS_FIR.taps[n] = D_N_0[n];
                            }
                            for(uint16_t n = LMS_Inst.tap_len >> 1; n < LMS_Inst.tap_len; ++n){
                                LMS_FIR.taps[n] = X_N_0[n - (LMS_Inst.tap_len >> 1)];
                            }
                        } else {
                            // Send only data read from D_N input port
                            for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n) LMS_FIR.taps[n] = D_N_0[n];
                        }
                        
                        busy_wait_ms(10);
                        CORE_1_STATE = CORE_1_SHIP_RESULTS;

                    } else {

                        if(LMS_Inst.averaging_counts){
                            for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                                LMS_H_HATS_AVG[n] = (LMS_H_HATS_AVG[n] + LMS_FIR.taps[n]) >> 1;
                                //LMS_FIR.taps[n] -= LMS_H_HATS_CORRECTION[n];
                            }
                        } else {
                            for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                                LMS_H_HATS_AVG[n] = LMS_FIR.taps[n];
                            }
                        }

                        if(++LMS_Inst.averaging_counts >= LMS_Inst.averaging_counts_limit) CORE_1_STATE = CORE_1_SHIP_RESULTS;
                        else CORE_1_STATE = CORE_1_APPLY_SETTINGS;
                    }
                    
                    
                }

                // Turn off WGN if always on is disabled
                if(!CORE_1_WGN_STATE){
                    stop_wgn_dma_channels();
                }

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;

            case CORE_1_SHIP_RESULTS: {

                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 0, Bandit_RGBU.B = 127);
                
                // Transfer results from LMS to memory accessible by both cores
                //  only like this to improve speed someday with DMA...
                //      but not today :)
                // If this didn't need to be serialized DMA would be sick here
                //  but really no benefit due to overall processing structure
                //  1000% could be better tho
                if(!CORE_1_SEND_UNPROCESSED){
                    for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                        LMS_FIR.taps[n] = LMS_H_HATS_AVG[n];
                    }
                }
                

                Acquire_Lock_Blocking(INTERCORE_FFTMEM_LOCK_A);

                //Q15 *srcdata_start = LMS_FIR.taps + LMS_Inst.fixed_offset;
                //Q15 *srcdata_start = LMS_FIR.taps + 64u;
                //Q15 *srcdata_start = LMS_FIR.taps;

                transfer_results_to_safe_mem(LMS_FIR.taps, &ICTXFR_A, LMS_Inst.tap_len);
                ICTXFR_A.lms_error = LMS_Inst.error;
                ICTXFR_A.lms_samples_processed = LMS_Inst.samples_processed;
                ICTXFR_A.lms_error_attempts = error_attempts;
                ICTXFR_A.calibration_data = sending_calib_data;

                Release_Lock(INTERCORE_FFTMEM_LOCK_A);

                LMS_Inst.averaging_counts = 0;

                // Open window for potential settings memory access here,
                //  prevent loop from spinning too quickly
                busy_wait_us_32(100);
                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 127, Bandit_RGBU.B = 0);
                //busy_wait_ms(500);


#ifndef NO_DEBUG_LED
                busy_wait_ms(1000);
                set_RGB_levels(Bandit_RGBU.R = 127, Bandit_RGBU.G = 0, Bandit_RGBU.B = 0);
                busy_wait_ms(1000);
                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 127, Bandit_RGBU.B = 0);
                busy_wait_ms(1000);
                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 0, Bandit_RGBU.B = 127);
                busy_wait_ms(1000);
                set_RGB_levels(Bandit_RGBU.R = 0, Bandit_RGBU.G = 0, Bandit_RGBU.B = 0);
#endif
                CORE_1_STATE = CORE_1_IDLE;

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;
            
            case CORE_1_DEBUG_HANDLER: {
#ifdef FORCE_SAMPLING_4_TESTING
                CORE_1_DBG_MODE = true;
                CORE_1_STATE = CORE_1_SAMPLE;
#else
                if(!CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_IDLE;
#endif
            }
            break;
            
            default:
                // Idk should have some out of bounds safety, but whatever
                CORE_1_STATE = CORE_1_IDLE;
                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
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
static void USB_Handler(struct FFT_PARAMS *fft){
    // tusb here!
    // No stall states, this should whizz thru and be fine with the
    //  potential to not be handled for some time.
    //  Either use ISRs or hardware buffers

    uint16_t num_samples = fft->num_samples;
    uint16_t header_data[32];

    //setup header data
    header_data[0] = 0x4242;
    header_data[1] = fft->num_samples;
    header_data[2] = fft->log2_num_samples;
    header_data[3] = fft->shift_amount;
    for(uint16_t n = 4; n < 31; ++n){
        header_data[n] = 0;
    }

    uint32_t curr_time = timer_hw->timerawl;

    header_data[31] = 0xAA40;
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
    send_header_packet(header_data);
    while (tud_cdc_n_read_char(CDC_CTRL_CHAN) != 'a') {
        tud_task(); // wait for ACK from GUI

        // Basically for if BANDIT is unplugged while waiting for an ACK
        if(curr_time + BANDIT_USB_ACK_TIMEOUT_US > timer_hw->timerawl) goto trash_that_packet_label;
    }
    tud_cdc_n_read_flush(CDC_CTRL_CHAN);
    send_f_packets(fft->fr, num_samples);

    return;

trash_that_packet_label:
    tud_cdc_n_read_flush(CDC_CTRL_CHAN);
    tud_cdc_n_read_flush(CDC_DATA_CHAN);
}

//sends header data through data chan to GUI interface
static void send_header_packet(uint16_t *h_data){
    tud_cdc_n_write(CDC_DATA_CHAN, (uint8_t *)h_data, CDC_PACKET_LEN);
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
}

//sends fft data through data chan to GUI interface
static void send_f_packets(Q15 *data, uint16_t num_samples){
    uint16_t NUM_PACKET_PER_BUF = (num_samples * (uint16_t)sizeof(Q15)) >> LOG2_CDC_PACKET_LEN;

#ifdef FORCESEND_TESTING    
    if(NUM_PACKET_PER_BUF){
#endif
        for(uint16_t n = 0; n < NUM_PACKET_PER_BUF; ++n){
            while((uint16_t)tud_cdc_n_write_available(CDC_DATA_CHAN) < CDC_PACKET_LEN){
                tud_task(); // tinyusb device task
            }
            tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)data + (n * CDC_PACKET_LEN)), CDC_PACKET_LEN);
        }
#ifdef FORCESEND_TESTING
    } else {
        tud_cdc_n_write(CDC_DATA_CHAN, ((uint8_t *)data), num_samples * (uint16_t)sizeof(Q15));
    }
#endif
    tud_cdc_n_write_flush(CDC_DATA_CHAN);
}

//triggered when wanted_char is recieved thru ctrl channel
//  Cool ISR that will set configurations for the next LMS Cycle on Core 1
void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char) { 
    if(itf != CDC_CTRL_CHAN) itf = CDC_CTRL_CHAN;

    if (tud_cdc_n_read_char(itf) == wanted_char) {
        while (tud_cdc_n_available(itf) < BS_BF_LEN) {
            tud_task();    
        }
        uint32_t count = tud_cdc_n_read(itf, BS_RX_BF, BS_BF_LEN);

        tud_cdc_n_write_flush(CDC_DATA_CHAN);
        tud_cdc_n_write_flush(CDC_CTRL_CHAN);
        tud_cdc_n_read_flush(CDC_CTRL_CHAN);

        // Do USB Settings application here!!
        //  set UPDATED = true if tap length, errors, or frequency ranges
        //  have been changed
        //  set UPDATED = true if any bitfield settings have been altered
        
        // Settings come in as byte stream for future compatibility w ASCII control modes
        //  Byte[0] If Enabled
        //  Byte[1] Auto run
        //  Byte[2] Auto send
        //  Byte[3] WGN Always on
        //  Byte[4] RESERVED
        //  Byte[5] Manual Error Limit LSB
        //  Byte[6] Manual Error Limit MSB
        //  Byte[7] Manual Tap Length (LSB)
        //  Byte[8] Manual Tap Length (MSB)
        //  Byte[9] F Range (ENUM)

        // Acquire settings memory lock
        Acquire_Lock_Blocking(INTERCORE_SETTINGS_LOCK);

        uint32_t new_settings_value = Global_Bandit_Settings.settings_bf;
        
        if(BS_RX_BF[USBBSRX_EN]) new_settings_value |= (1u << BS_ENABLE);
        else new_settings_value &= ~(1u << BS_ENABLE); 
        if(BS_RX_BF[USBBSRX_AUTORUN]) new_settings_value |= (1u << BS_AUTO_RUN);
        else new_settings_value &= ~(1u << BS_AUTO_RUN);
        if(BS_RX_BF[USBBSRX_AUTOSEND]) new_settings_value |= (1u << BS_AUTO_SEND);
        else new_settings_value &= ~(1u << BS_AUTO_SEND);
        if(BS_RX_BF[USBBSRX_WGN_ALWAYS_ON]) new_settings_value |= (1u << BS_WGN_ON);
        else new_settings_value &= ~(1u << BS_WGN_ON);
       
        Q15 learnin_rate = (BS_RX_BF[USBBSRX_LEARNING_RATE_MSB] << 8) | BS_RX_BF[USBBSRX_LEARNING_RATE_LSB];


        // see Bode_Bandit.h for enum settings map defineds
        uint8_t newfreq_range = BS_RX_BF[USBBSRX_F_FRANGE];
        uint16_t newtaplen = (BS_RX_BF[USBBSRX_TAPLEN_MSB] << 8) | (BS_RX_BF[USBBSRX_TAPLEN_LSB]);
        uint16_t man_error_limit = (BS_RX_BF[USBBSRX_ERR_MSB] << 8) | (BS_RX_BF[USBBSRX_ERR_LSB]);
        uint16_t newlmsoffset = (BS_RX_BF[USBBSRX_OFFSET_MSB] << 8) | (BS_RX_BF[USBBSRX_OFFSET_LSB]);
        uint16_t newlmsmaxattempts = (BS_RX_BF[USBBSRX_ATTEMPTS_MSB] << 8) | BS_RX_BF[USBBSRX_ATTEMPTS_LSB];

        Global_Bandit_Settings.manual_freq_range = newfreq_range;
        Global_Bandit_Settings.manual_error_limit = man_error_limit;
        Global_Bandit_Settings.manual_tap_len_setting = newtaplen;
        Global_Bandit_Settings.manual_lms_offset = *((int16_t *)&newlmsoffset);
        Global_Bandit_Settings.manual_lms_attempts = newlmsmaxattempts;

        Global_Bandit_Settings.manual_learning_rate = learnin_rate;
        //new_settings_value |= (1u << BS_AUTO_RUN);

        Global_Bandit_Settings.settings_bf = new_settings_value;

        SKIP_LMS = Global_Bandit_Settings.skip_lms = (BS_RX_BF[USBBSRX_RAW_RQ]) ? true : false;
        
        SKIP_FFT = (BS_RX_BF[USBBSRX_TIME_DOMAIN_DATA]) ? true : false;

        // Set settings updated flag
        Global_Bandit_Settings.updated = true;

        // LSB First Debug readback of:
        tud_cdc_n_write(CDC_CTRL_CHAN, BS_RX_BF, count);                                    // Raw Received data
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&Global_Bandit_Settings.settings_bf, 4);  // Updated Settings bitfield
        tud_cdc_n_write(CDC_CTRL_CHAN, &newfreq_range, 1);                                  // Decoded frequency range
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&newtaplen, 2);                           // Decoded tap length
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&man_error_limit, 2);                     // Decoded error limit
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&newlmsoffset, 2);                        // Decoded LMS offset   
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&newlmsmaxattempts, 2);                   // Decoded manual error limit
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&learnin_rate, 2);                        // Decoded manual Learning Rate
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&Global_Bandit_Settings.skip_lms, 1);     // Decoded Skip LMS
        tud_cdc_n_write(CDC_CTRL_CHAN, (uint8_t *)&SKIP_FFT, 1);                            // Decoded Skip FFT


        // Flush outgoing transmissions
        tud_cdc_n_write_flush(CDC_DATA_CHAN);
        tud_cdc_n_write_flush(CDC_CTRL_CHAN);
        tud_cdc_n_read_flush(CDC_CTRL_CHAN);

        // Release settings memory lock
        //  settings volatile types ensure data is written before lock is freed
        Release_Lock(INTERCORE_SETTINGS_LOCK);
    } else {
        tud_cdc_n_read_flush(CDC_CTRL_CHAN);
        tud_task();
    }
}



static inline void fft_setup(struct FFT_PARAMS *cool_fft, uint16_t len, uint16_t maxtablesize){
    cool_fft->num_samples = len;
    cool_fft->log2_num_samples = get_log_2(len);
    cool_fft->shift_amount = get_shift_amt(cool_fft->log2_num_samples);
    cool_fft->true_max = maxtablesize;
    get_table_offset_shift(cool_fft, maxtablesize);
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
static void transfer_results_to_safe_mem(Q15 *src, struct Transfer_Data *dst, uint16_t len){
    dst->len = len;
    for(uint16_t n = 0; n < len; ++n){
        dst->data[n] = *src++;
    }
    dst->updated = true;
}

static void clear_adc_read_buffers(){
    while(!pio_sm_is_rx_fifo_empty(ADC_PIO, ADS_PIO_SDOA_SM)){
        (ADC_PIO->rxf[ADS_PIO_SDOA_SM]);
        (ADC_PIO->rxf[ADS_PIO_SDOB_SM]);
    }
}


  /////////////////////////////////////////////////////////
 ////////////////////// Generic Functions :) /////////////
/////////////////////////////////////////////////////////
