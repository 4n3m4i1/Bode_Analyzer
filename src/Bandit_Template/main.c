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
    uint16_t    len;
    Q15         data[TOTAL_ADAPTIVE_FIR_LEN];
};

struct Transfer_Data ICTXFR_A;


struct BANDIT_SETTINGS  Global_Bandit_Settings;


struct BANDIT_LED_COLORS {
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t U;
};

struct BANDIT_LED_COLORS Bandit_RGBU;



  //////////////////////////////////////////////////////////////////////
 /////////////////////    CORE 0 ALLOCATIONS    ///////////////////////
//////////////////////////////////////////////////////////////////////
// FFT Tap Buffers
CORE_0_MEM Q15 FR_BUFF[TOTAL_ADAPTIVE_FIR_LEN];
CORE_0_MEM Q15 FI_BUFF[TOTAL_ADAPTIVE_FIR_LEN];

// Reserve Buffer for Stored Results
CORE_0_MEM Q15 RESULTS_BUFFER[TOTAL_ADAPTIVE_FIR_LEN];


CORE_0_MEM uint16_t USB_STATE;
CORE_0_MEM uint16_t USB_NEXT_STATE;

  //////////////////////////////////////////////////////////////////////
 /////////////////////    CORE 1 ALLOCATIONS    ///////////////////////
//////////////////////////////////////////////////////////////////////
// Sampling Banks
CORE_1_MEM Q15 X_N_0[TOTAL_BUFF_LEN];
CORE_1_MEM Q15 D_N_0[TOTAL_BUFF_LEN];

// LMS FIR For Producing Y[n]
CORE_1_MEM Q15 LMS_FIR_BANK[TOTAL_ADAPTIVE_FIR_LEN];
CORE_1_MEM Q15 LMS_H_HATS[TOTAL_ADAPTIVE_FIR_LEN];

CORE_1_MEM Q15 LMS_H_HATS_CORRECTION[TOTAL_ADAPTIVE_FIR_LEN];

// Downsampling FIR and Tap Sets
CORE_1_MEM Q15 DDSAMP_FIR_BANK[DDSAMP_FIR_LEN];
CORE_1_MEM Q15 DDSAMP_TAP_BANK[DDSAMP_FIR_LEN][DDSAMP_TAP_OPTIONS];

// Core 1 Defines
#define ADCRES_BITS     12u
#define ADC_RES_CODES   (1u << ADCRES_BITS)
#define ADC_VREF_CODE   0x108                   // ~2.15 V from ADS7253 DAC

#define ADC_PIO         pio1
CORE_1_MEM uint16_t     tmp_arr[ADS_READ_REG_COUNT] = {0,0,0};




  //////////////////////////////////////////////////////////////////////
 ///////////////////  CORE 0 FUNCTION DEFINITIONS   ///////////////////
//////////////////////////////////////////////////////////////////////
static void core_0_main();

static void USB_Handler(struct FFT_PARAMS *fft);
static void update_bandit_config();

static void send_header_packet(Q15 *h_data);
static void send_f_packets(Q15 *data, uint16_t num_samples);

static inline void fft_setup(struct FFT_PARAMS *cool_fft, uint16_t len);
static inline void fft_clear_fi(struct FFT_PARAMS *cool_fft);


  //////////////////////////////////////////////////////////////////////
 ///////////////////  CORE 1 FUNCTION DEFINITIONS   ///////////////////
//////////////////////////////////////////////////////////////////////
static void     core_1_main();
static void     transfer_results_to_safe_mem(Q15 *src, struct Transfer_Data *dst, uint16_t len);
static void     clear_adc_read_buffers();

/*
    Spinlocks for mututally exclusive access to shared memory resources.

    When a lock is held no other core can access that memory location, and by
        extention in most cases the entire RAM bank, due to how accesses
        are shared. (see: CORE_0_MEM and CORE_1_MEM)
*/
spin_lock_t *FFTMEMLOCK_A;
spin_lock_t *SETTINGS_LOCK;


  //////////////////////////////////////////////////////////////////////
 ////////////////////////////    Code!    /////////////////////////////
//////////////////////////////////////////////////////////////////////
// Main
// Start up in Core 0
int main(){
    stdio_init_all();
    init_LED_pins();

    Bandit_RGBU.R = 0;
    Bandit_RGBU.G = 0;
    Bandit_RGBU.B = 0;
    Bandit_RGBU.U = 0;
    set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B);
    set_ULED_level(Bandit_RGBU.U);

    gpio_init(VDDA_EN_PAD);
    gpio_set_dir(VDDA_EN_PAD, GPIO_OUT);
    gpio_put(VDDA_EN_PAD, true);

    Setup_Semaphores();

    FFTMEMLOCK_A = spin_lock_init(INTERCORE_FFTMEM_LOCK_A);
    SETTINGS_LOCK = spin_lock_init(INTERCORE_SETTINGS_LOCK);

    // Claim lock on settings until Core 0 can deal with initialization
    //  prevents Core 1 from starting any processing without settings
    //  in place.
    spin_lock_claim(INTERCORE_SETTINGS_LOCK);

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
    //  If blue is seen bus priorities have been applied.
    Bandit_RGBU.R = 0;
    Bandit_RGBU.G = 0;
    Bandit_RGBU.B = 120;
    Bandit_RGBU.U = 0;
    set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B);
    set_ULED_level(Bandit_RGBU.U);

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
    Global_Bandit_Settings.updated = true;
    Global_Bandit_Settings.settings_bf = BANDIT_DFL_SETTINGS;
    Global_Bandit_Settings.manual_tap_len_setting = 0;
    Global_Bandit_Settings.manual_error_limit = 0;
    Global_Bandit_Settings.manual_freq_range = 0;
    // Locked by main core at startup
    spin_lock_unclaim(INTERCORE_SETTINGS_LOCK);

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

    // Initialize tinyUSB with board and start listening for start char from GUI
    tud_init(BOARD_TUD_RHPORT);
    tud_cdc_n_set_wanted_char(CDC_CTRL_CHAN, START_CHAR);
    USB_NEXT_STATE = USB_INIT;

    // USB State Machine, Settings Application, FFT
    while(1){
        static volatile uint16_t a = 0;
        ++a;
        tight_loop_contents();

        tud_task();
        USB_STATE = USB_NEXT_STATE;
        switch(USB_STATE) {
            case USB_INIT: {
                //need idle work until GUI is ready
            }
            break;
            
            case USB_APPLY_SETTINGS: {
                uint32_t spinlock_irq_status = spin_lock_blocking(SETTINGS_LOCK);

                // Do USB Settings application here!!
                //  set UPDATED = true if tap length, errors, or frequency ranges
                //  have been changed
                //  set UPDATED = true if any bitfield settings have been altered

                spin_unlock(SETTINGS_LOCK, spinlock_irq_status);
            }
            break;

            case USB_FFT_DATA_COLLECT: {
                // Acquire safe access to FFT mem, can use 2 locks for ping pong access
                uint32_t spinlock_irq_status;
                spin_lock_t *used_lock;
                struct Transfer_Data *data_src;

                spinlock_irq_status = spin_lock_blocking(FFTMEMLOCK_A);
                used_lock = FFTMEMLOCK_A;
                data_src = &ICTXFR_A;


                // Apply windowing here if needed!!!
                for(uint16_t n = 0; n < data_src->len; ++n){
                    cool_fft.fr[n] = data_src->data[n];
                    cool_fft.fi[n] = 0;
                }

                fft_setup(&cool_fft, data_src->len);
                
                // Free memory constraints
                spin_unlock(used_lock, spinlock_irq_status);

                USB_NEXT_STATE = USB_RUN_DMC_JK_RUN_FFT;
                
                }
            break;
            
            case USB_RUN_DMC_JK_RUN_FFT: {
                FFT_fixdpt(&cool_fft);
                USB_NEXT_STATE = USB_SEND_TUSB;
            }
            break;

            case USB_SEND_TUSB: {
                USB_Handler(&cool_fft); //send data to GUI through tusb
                USB_NEXT_STATE = USB_FFT_DATA_COLLECT;
            }
            break;
            
            default: {
                USB_STATE = USB_INIT;
            }
            break; //need debug for USB_STATE error

        }

        
    }
}

  //////////////////////////////////////////////////////////////////////
 /////////////////    Core 1 Main Loop    /////////////////////////////
//////////////////////////////////////////////////////////////////////
static void core_1_main(){
    uint8_t Bandit_Calibration_State = BANDIT_UNCALIBRATED;
    Q15 Bandit_DC_Offset_Cal = 0;
    
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
    // Don't init frontend PGA until calibration
    //MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_0, PGA_CSN_PAD);
    //MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);
    

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
    
    // Go Red for DAC calibration cycle
    Bandit_RGBU.R = 200;
    Bandit_RGBU.G = 0;
    Bandit_RGBU.B = 0;
    Bandit_RGBU.U = 0;
    set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B);
    set_ULED_level(Bandit_RGBU.U);

    // ADC Now configured for 16clk mode dual SDO, configure refdac now!
    uint16_t ADC_REFDACVAL = 0x108;
    uint32_t ADC_REFDAC_CAL_ACCUM;
    tmp_arr[1] = 0;
    tmp_arr[2] = 0;

    // Limit of how many times a calibration approaching an exact value can be done.
    const uint16_t DAC_CAL_EXACT_LIMIT = 128;
    uint16_t DAC_CAL_ATTEMPTS = 0;

    // Refdac init at 0x1FF -> 2.5V, we want some fraction of this estimated at 0x108 (2.19V)
start_refdac_cal:
    ADC_REFDAC_CAL_ACCUM = 0;
    tmp_arr[0] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, ADC_REFDACVAL << 3);
    ADS7253_write16_blocking(ADC_PIO, tmp_arr, ADS_WRITE_DAC_COUNT);

    // ADS7253 Section 7.8: t_refon = 8ms settling time for REFDAC
    //  go 10ms just to be sure
    busy_wait_us_32(10000);

    // Clear any read values
    clear_adc_read_buffers();

    for(uint16_t n = 0; n < 128; ++n){
        ADS7253_Dual_Sampling(ADC_PIO, tmp_arr, &D_N_0[n], &X_N_0[n], 1);
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
    if(ADC_REFDAC_CAL_ACCUM <= (ADS_MID_CODE_BINARY - 1)){  // If we can't hit exact fallback to +/-1 of exact
            // ADC Reference is HIGH
            ADC_REFDACVAL--;
            goto start_refdac_cal;
    } else
    if(ADC_REFDAC_CAL_ACCUM >= (ADS_MID_CODE_BINARY + 1)){
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
    
    busy_wait_us_32(5);

    // Clear any read values
    clear_adc_read_buffers();
    

    // Go Purple for DAC cal complete
    Bandit_RGBU.R = 127;
    Bandit_RGBU.G = (uint8_t) DAC_CAL_ATTEMPTS;
    Bandit_RGBU.B = 127;
    Bandit_RGBU.U = 0;
    set_RGB_levels(Bandit_RGBU.R, Bandit_RGBU.G, Bandit_RGBU.B);
    set_ULED_level(Bandit_RGBU.U);
    

    // Setup Sampling Pace Flags
    Sampling_Setup(500000);

    uint16_t CORE_1_STATE = CORE_1_IDLE;
    uint16_t error_attempts;
    bool CORE_1_DBG_MODE = false;

    // Main Loop for Core 1
    //  All peripherals should be configured by now :)
    while(1){
        switch(CORE_1_STATE){
            case CORE_1_IDLE: {
                set_ULED_level(Bandit_RGBU.U++);
                busy_wait_ms(20);
                
                // If starting conditions are met start the whole DSP cycle
                if(Bandit_Calibration_State == BANDIT_UNCALIBRATED){
                    Bandit_Calibration_State = BANDIT_CAL_DC_BIAS_IN_PROG;
                    CORE_1_STATE = CORE_1_SAMPLE;
                } else {
                    error_attempts = 0;
                    
                    LMS_Inst.samples_processed = 0;

                    CORE_1_STATE = CORE_1_APPLY_SETTINGS;
                }

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;
            case CORE_1_APPLY_SETTINGS: {
                uint32_t spinlock_irq_status = spin_lock_blocking(SETTINGS_LOCK);

                if(Global_Bandit_Settings.updated){
                    if(Bandit_Calibration_State > BANDIT_CAL_DC_BIAS_IN_PROG){
                        Global_Bandit_Settings.updated = false;

                        // if settings changed:
                        //  Bandit_Calibration_State = BANDIT_CAL_AA_TXFR_FUNC_IN_PROG
                        // Switch frontend PGA to White Noise Loopback
                        MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_1, PGA_CSN_PAD);
                        MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);

                        // Need to recalibrate transfer function if a settings update occurs
                        Bandit_Calibration_State = BANDIT_CAL_AA_TXFR_FUNC_IN_PROG;
                    } else {
                        // Need to do DC Cal
                        // Keep PGA uninitialized
                        //  Sample and adjust for DC
                        CORE_1_STATE = CORE_1_SAMPLE;
                    }
                    
                    // Apply Generic Settings
                    CORE_1_DBG_MODE = (CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_DEBUG_MODE));

                    // Free spinlock on Global Settings
                    spin_unlock(SETTINGS_LOCK, spinlock_irq_status);
                    // PGA Settling Time
                    busy_wait_us_32(MCP6S92_SETTLING_TIME);
                } else {
                    // if auto run or run signal -> CORE_1_STATE = CORE_1_SAMPLE;
                    if(CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_AUTO_RUN) || 
                        CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_SINGLE_SHOT_RUN)){
                            
                            CLR_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_SINGLE_SHOT_RUN);
                            CORE_1_STATE = CORE_1_SAMPLE;
                    }

                    spin_unlock(SETTINGS_LOCK, spinlock_irq_status);
                }

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
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
                    X_N_0[n] -= Bandit_DC_Offset_Cal;
                }

                Stop_Sampling();
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
                    dcavg_zone += ((int32_t)D_N_0[n] - (int32_t)X_N_0[n]);
                } 
                dcavg_zone >>= LOG2_STD_MAX_SAMPLES;
                Bandit_DC_Offset_Cal = (Q15)dcavg_zone;
                Bandit_Calibration_State = BANDIT_CAL_DC_BIAS_COMPLETE;

                CORE_1_STATE = CORE_1_IDLE;
            }
            break;
            case CORE_1_DOWNSAMPLE: {
                
                CORE_1_STATE = CORE_1_LMS;

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;
            case CORE_1_LMS: {
                // If first run: error_attempts = 0, thus we flush FIR buffer
                //  else we should maintain results for more iterations
                Q15 LMS_Error = LMS_Looper(&LMS_Inst, &LMS_FIR, !error_attempts);

                if(LMS_Error == LMS_FAIL_DFL){
                    // Handle the error
                    //  Sample again and try again
                    error_attempts++;

                    if(error_attempts > LMS_Inst.max_convergence_attempts) {
                        // Bring up failure to user here or something
                        CORE_1_STATE = CORE_1_POST_PROC;
                    } else {
                        CORE_1_STATE = CORE_1_SAMPLE;
                    }
                } else {
                    CORE_1_STATE = CORE_1_POST_PROC;
                }
                
                if(Bandit_Calibration_State != BANDIT_FULLY_CALIBRATED);

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;
            case CORE_1_POST_PROC: {
                if(Bandit_Calibration_State != BANDIT_FULLY_CALIBRATED){
                    // Apply transfer function correction
                    // Apply correction in time domain, it's linear idk man should be right
                    
                    for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                        LMS_H_HATS_CORRECTION[n] = LMS_FIR.data[n];
                    }

                    // Set PGA to EXTERNAL input x1 GAIN
                    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_0, PGA_CSN_PAD);
                    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);

                    // PGA Settling Time
                    busy_wait_us_32(MCP6S92_SETTLING_TIME);

                    Bandit_Calibration_State = BANDIT_FULLY_CALIBRATED;
                    
                    CORE_1_STATE = CORE_1_APPLY_SETTINGS;
                } else {
                    // Move on with a normal run
                    for(uint16_t n = 0; n < LMS_Inst.tap_len; ++n){
                        LMS_FIR.data[n] -= LMS_H_HATS_CORRECTION[n];
                    }
                    CORE_1_STATE = CORE_1_SHIP_RESULTS;
                }

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;
            case CORE_1_SHIP_RESULTS: {
                // Transfer results from LMS to memory accessible by both cores
                //  only like this to improve speed someday with DMA...
                //      but not today :)
                // If this didn't need to be serialized DMA would be sick here
                //  but really no benefit due to overall processing structure
                //  1000% could be better tho
                uint32_t spinlock_irq_status;
                spin_lock_t *used_lock;

                spinlock_irq_status = spin_lock_blocking(FFTMEMLOCK_A);
                used_lock = FFTMEMLOCK_A;
                
                transfer_results_to_safe_mem(LMS_FIR.data, &ICTXFR_A, LMS_Inst.tap_len);
                
                spin_unlock(used_lock, spinlock_irq_status);

                CORE_1_STATE = CORE_1_IDLE;

                if(CORE_1_DBG_MODE) CORE_1_STATE = CORE_1_DEBUG_HANDLER;
            }
            break;
            case CORE_1_DEBUG_HANDLER: {

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


// recieve config data from GUI and update BANDIT_SETTINGS accordingly
static void update_bandit_config(){

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
static void transfer_results_to_safe_mem(Q15 *src, struct Transfer_Data *dst, uint16_t len){
    dst->len = len;
    for(uint16_t n = 0; n < len; ++n){
        dst->data[n] = *src++;
    }
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
