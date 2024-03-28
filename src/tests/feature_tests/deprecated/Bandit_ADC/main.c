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
CORE_1_MEM struct ADS7253_Inst_t ADC_Inst;
static void     core_1_main();
static void     setup_ADC(struct ADS7253_Inst_t *ADC);
static void     set_ADC_free_running();
static void     stop_ADC_free_running();
static void     set_ADC_dac(struct ADS7253_Inst_t *adc, uint16_t refA, uint16_t refB);
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
    
    Global_Bandit_Settings.settings_bf = BANDIT_DFL_SETTINGS | BS_WGN_ON;
    Global_Bandit_Settings.manual_tap_len_setting = 0;
    Global_Bandit_Settings.manual_error_limit = 0;
    Global_Bandit_Settings.manual_freq_range = 0;


    // Setup AWGN Generation from overdriven ROSC -> DMA -> PIO
    //setup_rosc_full_tilt();
    //setup_PIO_for_switching();
    //setup_chained_dma_channels();

    //if(CHK_BANDIT_SETTING(Global_Bandit_Settings.settings_bf, BS_WGN_ON)){
    //    start_randombit_dma_chain(dma_awgn_ctrl_chan);
    //}

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

    sleep_ms(1);

    PIO ADC_PIO = pio1;

    

/*
ads7253_pio_ctrl_setup(PIO pio, uint sm, uint prog_offs, uint num_bits,
                    uint clkdiv_i, uint clkdiv_f, uint pin_cs, uint pin_copi){
*/
    uint pioprogramoffset = pio_add_program(ADC_PIO, &ADS7253_SPI_CTRL_program);
    //ads7253_pio_ctrl_setup(ADC_PIO, ADS_PIO_MAIN_SM, pioprogramoffset, 16, 16, 0, ADC_CSN_PAD, ADC_SDI_PAD);

    Q15_Sampling_Bank_ptr = 0;
    sleep_ms(1);

    /*
        Initiate frontend PGA/MUX
            Set channel 0 (D_N), 1x gain
    */
    spi_inst_t *mcp_spi = spi1;
    MCP6S92_Init(mcp_spi, PGA_CSN_PAD, PGA_SCK_PAD, PGA_SI_PAD);
    // Internal loopback for testing!
    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_CHANNEL_REGISTER), MCP6S92_CHAN_0, PGA_CSN_PAD);
    MCP6S92_Send_Command_Raw(mcp_spi, MCP6S92_INSTR(MCP6S92_REG_WRITE, MCP6S92_GAIN_REGISTER), MCP6S92_x1_GAIN, PGA_CSN_PAD);

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t l = 0;
    
    //set_ADC_free_running();
    pwm_hw->slice[PWMSLICE_ISR].csr |= PWM_CH1_CSR_EN_BITS;

    uint8_t a = 0;

    uint16_t refval = 0x108;

    while(1){
        //if(Q15_Sampling_Bank_ptr >= TOTAL_BUFF_LEN){
        //    set_RGB_levels((uint8_t)(D_N_0[0] & 0xFF) >> 8,g, (uint8_t)(D_N_0[0] & 0xFF));
        //    Q15_Sampling_Bank_ptr = 0;
        //    //for(int n = 0; n < TOTAL_BUFF_LEN; ++n){
        //    //    printf("%d\t%d\n", D_N_0[n], X_N_0[n]);
        //    //}
        //    //printf("%3u %d\n", a, D_N_0[0]);
        //    //set_ADC_free_running();
        //} else {
        //    //tight_loop_contents();
        //    //sleep_us(1);
        //}
        //tight_loop_contents();
        char b = getchar_timeout_us(0);
        switch(b){
            case 'c':{   // configure
                printf("Configuring!\n");
                uint16_t sendemcmd[3];
                sendemcmd[0] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (0 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                sendemcmd[1] = 0x0000;
                sendemcmd[2] = 0x0000;
                //gpio_put(ADC_CSN_PAD, false);
                //spi_write16_blocking(ADC_Inst.ads_spi, sendemcmd, 1);
                setup_ADC(&ADC_Inst);
                
                //gpio_put(ADC_CSN_PAD, true);

                sleep_us(1);

                //gpio_put(ADC_CSN_PAD, false);
//
                //gpio_put(ADC_CSN_PAD, true);
            }
            break;

            case 'r':{   // read
                uint16_t sendem[3];
                sendem[0] = ADS7253_CMD(ADS7253_CFR_READ, 0);
                sendem[1] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                sendem[2] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                gpio_put(ADC_CSN_PAD, false);
                spi_write16_blocking(ADC_Inst.ads_spi, sendem, 3);
                gpio_put(ADC_CSN_PAD, true);
                busy_wait_us(1);
                gpio_put(ADC_CSN_PAD, false);
                spi_read16_blocking(ADC_Inst.ads_spi, 0x0000, sendem, 3);
                gpio_put(ADC_CSN_PAD, true);

                printf("CFR: 0x%4X\n", sendem[0]);
            }
            break;

            case 'd':{   // read
                uint16_t sendem[3];
                sendem[0] = ADS7253_CMD(ADS7253_REFDAC_A_READ, 0);
                sendem[1] = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, refval << 4);
                //sendem[2] = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (1 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                
                //sendem[1] = 0; 
                sendem[2] = 0;

                printf("Sending: 0x%04X 0x%04X 0x%04X\n", sendem[0], sendem[1], sendem[2]);

                gpio_put(ADC_CSN_PAD, false);
                spi_write16_blocking(ADC_Inst.ads_spi, sendem, 3);
                gpio_put(ADC_CSN_PAD, true);
                busy_wait_us(1);
                gpio_put(ADC_CSN_PAD, false);
                // 
            //spi_write16_read16_blocking 
                uint16_t rval[3];
                spi_write16_read16_blocking(ADC_Inst.ads_spi, sendem, rval, 3);
                //spi_read16_blocking(ADC_Inst.ads_spi, 0x0000, sendem, 3);
                gpio_put(ADC_CSN_PAD, true);

                printf("DAC: 0x%04X 0x%04X 0x%04X\n", rval[0], rval[1], rval[2]);
            }
            break;

            case '+':
                refval += 16;
                set_ADC_dac(&ADC_Inst, refval, refval);
                printf("Plus\t0x%04X\n", refval);
            break;

            case '-':
                refval -= 16;
                set_ADC_dac(&ADC_Inst, refval, refval);
                printf("Minus\t0x%04X\n", refval);
            break;

            case 'v':{
                uint16_t dst[2];
                spi_read16_blocking(ADC_Inst.ads_spi, 0x0000, dst, 2);

                printf("Read: 0x%04X 0x%04X\n", dst[0], dst[1]);

            }
            break;

            case 'e':{
                printf("Sending COMMAND\n");
                uint16_t command = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (0 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                pio_ADS7253_Write_CMD(ADC_PIO, command);
                busy_wait_us(1);
                command = ADS7253_CMD(ADS7253_REFDAC_A_WRITE, 0x108 << 3);
                pio_ADS7253_Write_CMD(ADC_PIO, command);
                busy_wait_us(1);
                command = ADS7253_CMD(ADS7253_REFDAC_B_WRITE, 0x108 << 3);
                pio_ADS7253_Write_CMD(ADC_PIO, command);
            }
            break;

            case 'p':{
                printf("PIO Instructions:\n");
                uint16_t rval = ADC_PIO->sm[ADS_PIO_MAIN_SM].addr;
                uint16_t rval2 = ADC_PIO->sm[ADS_PIO_SDOA_SM].addr;
                uint16_t rval3 = ADC_PIO->sm[ADS_PIO_SDOB_SM].addr;
                //for(int n = 0; n < 32; ++n){
                //    printf("%2d\t0x%08X\n", n, ADC_PIO->instr_mem[n]);
                //}

                printf("INSTR SM: 0\t%2d -> 0x%08X\n", rval, ADC_PIO->sm[ADS_PIO_MAIN_SM].instr);
                printf("INSTR SM: 1\t%2d -> 0x%08X\n", rval2, ADC_PIO->sm[ADS_PIO_SDOA_SM].instr);
                printf("INSTR SM: 2\t%2d -> 0x%08X\n", rval3, ADC_PIO->sm[ADS_PIO_SDOB_SM].instr);

                printf("\nDone!\n");
            }
            break;

            case 't': {
                printf("Write n Count\n");
                static uint16_t ctr[3][128];
                static uint32_t cc[128];
                static uint32_t fifo[128];
                ctr[0][0] = ADC_PIO->sm[ADS_PIO_MAIN_SM].instr;
                ctr[1][0] = ADC_PIO->sm[ADS_PIO_SDOA_SM].instr;
                ctr[2][0] = ADC_PIO->sm[ADS_PIO_SDOB_SM].instr;
                cc[0] = ADC_PIO->dbg_padout;
                uint16_t command = ADS7253_CMD(ADS7253_CFR_WRITE, ((1 << ADS7253_CFR_RD_CLK_MODE) | (0 << ADS7253_CFR_RD_DATA_LINES) | (1 << ADS7253_CFR_REF_SEL)));
                //ADC_PIO->txf[ADS_PIO_MAIN_SM] = command;
                //ADC_PIO->txf[ADS_PIO_MAIN_SM] = 0x0000;
                //ADC_PIO->txf[ADS_PIO_MAIN_SM] = 0x0000;
                //ADC_PIO->txf[ADS_PIO_MAIN_SM] = 0x0000;
                //pio_ADS7253_Write_CMD(ADC_PIO, command);
                io_rw_16 *txfifo = (io_rw_16 *) &ADC_PIO->txf[ADS_PIO_MAIN_SM];
                *txfifo = command;
                *txfifo = 0xFFFF;
                *txfifo = 0xAAAA;

                fifo[0] = ADC_PIO->flevel;

                for(int n = 1; n < count_of(ctr[0]); ++n){
                    ctr[0][n] = ADC_PIO->sm[ADS_PIO_MAIN_SM].instr;
                    //ctr[1][n] = ADC_PIO->sm[ADS_PIO_SDOA_SM].instr;
                    //ctr[2][n] = ADC_PIO->sm[ADS_PIO_SDOB_SM].instr;
                    cc[n] = ADC_PIO->dbg_padout;
                    fifo[n] = ADC_PIO->flevel;
                }

                printf("State Machine:\n0\t1\t2\n");
                printf("Pins: CSN: %X\tSCK: %X\tMOSI: %X\n", 1u << ADC_CSN_PAD, 1u << ADC_SCK_PAD, 1u << ADC_SDI_PAD);
                for(int n = 0; n < count_of(ctr[0]); ++n){
                    printf("%2d -> 0x%4X\t0x%4X\t0x%4X -> 0x%02X\tFLEVEL: 0x%08X\n", n, ctr[0][n], ctr[1][n], ctr[2][n], (uint8_t)cc[n], fifo[n]);
                }
                printf("\nDirs: 0x%08X\n", ADC_PIO->dbg_padoe);

                printf("State Machine Registers:\n");
                printf("Shift Control: 0x%08X\n", ADC_PIO->sm[ADS_PIO_MAIN_SM].shiftctrl);
                printf("Exec  Control: 0x%08X\n", ADC_PIO->sm[ADS_PIO_MAIN_SM].execctrl);
                uint pinz = ADC_PIO->sm[ADS_PIO_MAIN_SM].pinctrl;
                printf("Pin   Control: 0x%08X\n", ADC_PIO->sm[ADS_PIO_MAIN_SM].pinctrl);
            
                uint sideset_ct = pinz >> 29;
                uint set_count = (pinz >> 26) & 0b111;
                uint out_count = (pinz >> 20) & 0b11111;
                uint in_base = (pinz >> 15) & 0b11111;
                uint side_base = (pinz >> 10) & 0b11111;
                uint set_base = (pinz >> 5) & 0b11111;
                uint out_base = pinz & 0b11111;

                printf("Side Count\t%d\n", sideset_ct);
                printf("Side Base\t%d\n", side_base);

                printf("Set Count\t%d\n", set_count);
                printf("Set Base\t%d\n", set_base);

                printf("Out Count\t%d\n", out_count);
                printf("Out Base\t%d\n", out_base);
                printf("In Base\t%d\n", in_base);
                

                printf("FLEVEL: 0x%08X\tRead: 0x%08X 0x%08X\n", ADC_PIO->flevel, ADC_PIO->rxf[ADS_PIO_SDOA_SM], ADC_PIO->rxf[ADS_PIO_SDOB_SM]);
                printf("FLEVEL: 0x%08X\tRead: 0x%08X 0x%08X\n", ADC_PIO->flevel, ADC_PIO->rxf[ADS_PIO_SDOA_SM], ADC_PIO->rxf[ADS_PIO_SDOB_SM]);
                printf("FLEVEL: 0x%08X\tRead: 0x%08X 0x%08X\n", ADC_PIO->flevel, ADC_PIO->rxf[ADS_PIO_SDOA_SM], ADC_PIO->rxf[ADS_PIO_SDOB_SM]);
                printf("FLEVEL: 0x%08X\tRead: 0x%08X 0x%08X\n", ADC_PIO->flevel, ADC_PIO->rxf[ADS_PIO_SDOA_SM], ADC_PIO->rxf[ADS_PIO_SDOB_SM]);
            }
            break;

            case 'g':{
                uint16_t rval[3];
                rval[0] = 0xDEAD;
                rval[1] = 0xBEEF;
                rval[2] = 0xA349;
                //pio_ADS7253_Read_Reg(ADC_PIO, ADS7253_CFR_READ, rval);
                io_rw_16 *txfifo = (io_rw_16 *) &ADC_PIO->txf[ADS_PIO_MAIN_SM];
                io_rw_16 *rxfifo = (io_rw_16 *) &ADC_PIO->rxf[ADS_PIO_MAIN_SM];
                uint len = 1;
                size_t tx_remain = len, rx_remain = len;
                uint16_t *src;
                src = rval;
                while (tx_remain || rx_remain) {
                    if (tx_remain && !pio_sm_is_tx_fifo_full(ADC_PIO, ADS_PIO_MAIN_SM)) {
                        *txfifo = *src++;
                        --tx_remain;
                    }
                    if (rx_remain && !pio_sm_is_rx_fifo_empty(ADC_PIO, ADS_PIO_MAIN_SM)) {
                        (void) *rxfifo;
                        --rx_remain;
                    }
                }

                printf("CFR: 0x%04X 0x%04X 0x%04X\n", rval[0], rval[1], rval[2]);
            }
            break;
        }

        //if(pwm_hw->slice[PWMSLICE_ISR].ctr == 0){
        //if(pwm_hw->intr & (1u << PWMSLICE_ISR)){
        //    pwm_clear_irq(PWMSLICE_ISR);
        //    set_ULED_level(a++);
        //    sampling_ISR_func();
        //}

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
    
    //gpio_put(ADC_CSN_PAD, 0);
//
    //uint32_t rv = ADS7253_RW_CMD(&ADC_Inst, ADS7253_CMD(
    //                                            ADS7253_CFR_READ, 0
    //                                                ));
    //
    //gpio_put(ADC_CSN_PAD, 1);
    //D_N_0[Q15_Sampling_Bank_ptr] = *(((Q15 *)&rv) + 1u);
    //X_N_0[Q15_Sampling_Bank_ptr++] = *(((Q15 *)&rv));

    //if(Q15_Sampling_Bank_ptr >= TOTAL_BUFF_LEN) stop_ADC_free_running();
};

// Setup ADC and PGA
static void setup_ADC(struct ADS7253_Inst_t *ADC){
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
    ADC->csn_pin = ADC_CSN_PAD;

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
    //pwm_clear_irq(PWMSLICE_ISR);
    //pwm_hw->inte |=         (1 << 1);   // Enable PWM block IRQ output from slice 1
    
    //pwm_set_irq_enabled(PWMSLICE_ISR, true);

    //irq_set_exclusive_handler(PWM_IRQ_WRAP, adc_read_handler);
    //irq_set_exclusive_handler(PWM_IRQ_WRAP, sampling_ISR_func);
    //irq_set_enabled(PWM_IRQ_WRAP, true);

    uint16_t cmd = ADS7253_CMD(ADS7253_CFR_WRITE,
                                            ADS7253_SET_SINGLE_SDO  |
                                            ADS7253_SET_16_CLK_MODE |
                                            ADS7253_USE_INTERNAL_REFERENCE |
                                            ADS7253_2X_REFDAC
                                            );

    ADS7253_Write_Command(ADC, cmd);

    busy_wait_us(1);
    // Reference at 2.195 V
   // ADS7253_Write_Command(ADC, ADS7253_CMD(ADS7253_REFDAC_A_WRITE, 0x109 << 3));
   // busy_wait_us(1);
   // ADS7253_Write_Command(ADC, ADS7253_CMD(ADS7253_REFDAC_B_WRITE, 0x109 << 3));
//
   // // Reference at A = 2.400 V, B = 2.00 V
   // ADS7253_Write_Command(ADC, ADS7253_CMD(ADS7253_REFDAC_A_WRITE, 0x1AE << 3));
   // //busy_wait_us(2);
   // ADS7253_Write_Command(ADC, ADS7253_CMD(ADS7253_REFDAC_B_WRITE, 0x069 << 3));

    set_ADC_dac(&ADC_Inst, 0x108, 0x108);
}

static void set_ADC_dac(struct ADS7253_Inst_t *adc, uint16_t refA, uint16_t refB){
    ADS7253_Write_Command(adc, ADS7253_CMD(ADS7253_REFDAC_A_WRITE, refA << 3));
//    busy_wait_us(1);
    ADS7253_Write_Command(adc, ADS7253_CMD(ADS7253_REFDAC_B_WRITE, refB << 3));
//    busy_wait_us(1);
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



