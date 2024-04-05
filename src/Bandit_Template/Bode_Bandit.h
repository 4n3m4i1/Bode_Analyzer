#ifndef BODE_BANDIT_h
#define BODE_BANDIT_h

#include "Bandit_Pins.h"
#include "fixedpt_include.h"
#include "Memory_Management.h"
#include "Bandit_Sampling.h"
#include "hardware/structs/bus_ctrl.h"
#include "Bandit_Debug.h"

#include "hardware/structs/nvic.h"

#include "FFT/fft_half.h"
#include "circular_buffers/RP2040_Circ_Buffer.h"
#include "FIR/FIR_Fast_Fixed.h"
#include "LMS/LMS_Fixed.h"
#include "AWGN_ROSC.h"
#include "MCP6S92_RP2040/MCP6S92_RP2040.h"
#include "Bandit_LED/Bandit_LED.h"
#include "PIO_ADS7253/PIO_ADS7253.h"
#include "Bandit_InterCore/Bandit_Inter_Core.h"
#include "Downsampling/Fixed_Filters.h"

/*
    Remember:
        e = d[n] - y[n]
        where
        y[n] = x[n] (CONVOLVE) h_hat[n]

        Thus:
            - Read both d[n] and x[n] in
            - Buffer both until:
                - STD max len + expected tap len + offset allowance samples collected
            
            - Run LMS
                - get err
                - update taps
                - process error
            
            - Memcpy (DMA or core) to CORE0MEM
            - Trigger FFT Go

    Example size determination:
        - 2048 words for buffer all in, need buffer room so next 2^n ->
        -       4096 words per buffer
        -   2 bytes / word
        - 8kB (8192 bytes)
        -   2 arrays (x[n] and d[n])
        - 16kB (16384 bytes)
*/


#define SAMPLE_RATE             (500 * 1000)
#define MAX_OFFSET_TIME         0.010
#define MAX_SAMPLES_OFFSET      (MAX_OFFSET_TIME * SAMPLE_RATE)
#define HARDCODED_OFFSET_CT     2048
#define DATA_W                  sizeof(Q15)
#define STD_MAX_SAMPLES         2048
#define LOG2_STD_MAX_SAMPLES    11
#define BYTELEN_STD_BUFFER      (DATA_W * STD_MAX_SAMPLES)
#define TOTAL_BUFF_LEN          (STD_MAX_SAMPLES + HARDCODED_OFFSET_CT)

/*
    Limits for LMS

    Buffers take 16kB total, leaving 48kB to play with in the Core 1 exclusive bank
*/
#define MAX_TAPS                1024
#define TOTAL_ADAPTIVE_FIR_LEN  (MAX_TAPS)

#define DEFAULT_LMS_TAP_LEN     32

#define MAX_ERROR_OUT_ATTEMPTS  16

/*
    42kB left to play with on Core 1, more than enough for like... anything
    
    Downsampling of N length requires:
        N MADDS, 4 * N memaccesses
        1 MADD -> 2 cycles
        4 memaccess -> 8 cycles best case
        Thus:
            10 * N cycles best case per sample
        With final commit and initial load:
            10 * N + 4 ish cycles

    At 500ksps and 125MHz core clk:
        125e6 / 500e3 = 250 cycles per sample

    10N + 4 = 250
    N = 24.6 length allowance, maybe enough?

    Shoot for N = 20, safety margin
    10 * 20 + 4 = 204 cycles. 
    
    This is dumb though, go polyphase so multiply by some B branch count.

    ISR overhead will consume most of the free time.
    Maybe just use wait loops for the sampling? really won't be much deadtime there
    and then there is less context switch overhead. Can better optimize, and split load
    better too...
*/
#define DDSAMP_FIR_LEN      20
/*
    Analysis Ranges:
        f_max   /n
        250k    1   (none)
        125k    2
        62.5k   4
        31.25k  8
        25k     10
*/
enum DDSAMP_FREQ_RANGE_IDX {
    DDSAMP_250K,
    DDSAMP_125K,
    DDSAMP_62K5,
    DDSAMP_31K25,
    DDSAME_25K
};
#define DDSAMP_TAP_OPTIONS  5


/*
    Settings predominantly handled via bitfields for status
*/
enum BANDIT_SETTINGS_BF {
    BS_ENABLE,
    BS_DEBUG_MODE,
    BS_SINGLE_SHOT_RUN,
    BS_AUTO_RUN,
    BS_AUTO_SEND,
    BS_WGN_ON,
    BS_RES1,
    BS_MAN_ERROR,
    BS_MAN_TAP_LEN,
    BS_MAN_FREQ_RANGE
};
#define BS_BF_LEN               10
/*
    Bandit Main Settings
*/

struct BANDIT_SETTINGS {
    uint8_t         updated;
    uint32_t        settings_bf;
    uint32_t        auto_freq_range;
    uint16_t        manual_tap_len_setting;
    uint32_t        manual_error_limit;
    uint32_t        manual_freq_range;
};

#define CHK_BANDIT_SETTING(a, b)        (a & (1u << b))
#define SET_BANDIT_SETTING(a, b)        (a |= (1u << b))
#define CLR_BANDIT_SETTING(a, b)        (a &= (1u << b))

#define BANDIT_DFL_SETTINGS     (1 << BS_ENABLE) | \
                                (1 << BS_AUTO_SEND)


/*
    States for CORE 1
        These must run in sequence with no interruptions!
*/
enum BANDIT_CORE_1_DSP_STATES {
    CORE_1_IDLE,
    CORE_1_APPLY_SETTINGS,
    CORE_1_SAMPLE,
    CORE_1_APPLY_DC_CORRECTION,
    CORE_1_DOWNSAMPLE,
    CORE_1_LMS,
    CORE_1_POST_PROC,
    CORE_1_SHIP_RESULTS,
    CORE_1_DEBUG_HANDLER
};

//tinyUsb definitions
#define START_CHAR              '~'
#define SETTINGS_CHAR           '('
#define CONFIG_CHAR

#define CDC_DATA_CHAN           0
#define CDC_CTRL_CHAN           1
#define CDC_PACKET_LEN          64

//core0 state machine init
//#define STATE             // variables?
//#define NEXT_STATE         
enum USB_STATE_MACHINEEEEE {
    USB_INIT,
    USB_APPLY_SETTINGS,
    USB_FFT_DATA_COLLECT,
    USB_RUN_DMC_JK_RUN_FFT,
    USB_SEND_TUSB,
    USB_RECIEVE_CONFIG,
    USB_UPDATE_CONFIG
};    

enum BANDIT_CALIBRATION_STATES {
    BANDIT_UNCALIBRATED,
    BANDIT_CAL_DC_BIAS_IN_PROG,
    BANDIT_CAL_DC_BIAS_COMPLETE,
    BANDIT_CAL_AA_TXFR_FUNC_IN_PROG,
    BANDIT_CAL_AA_TXFR_FUNC_COMPLETE,
    BANDIT_FULLY_CALIBRATED
};
#endif