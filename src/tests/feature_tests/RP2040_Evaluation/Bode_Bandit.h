#ifndef BODE_BANDIT_h
#define BODE_BANDIT_h

#include "Bandit_Pins.h"
#include "generic_include.h"
#include "Memory_Management.h"
#include "FFT/fft_half.h"
#include "circular_buffers/RP2040_Circ_Buffer.h"
#include "FIR/FIR_Fast_Fixed.h"
#include "LMS/LMS_Fixed.h"
#include "AWGN_ROSC.h"


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
#define BYTELEN_STD_BUFFER      (DATA_W * STD_MAX_SAMPLES)
#define TOTAL_BUFF_LEN          (STD_MAX_SAMPLES + HARDCODED_OFFSET_CT)

/*
    Limits for LMS

    Buffers take 16kB total, leaving 48kB to play with in the Core 1 exclusive bank
*/
#define MAX_TAPS                1024
#define TOTAL_ADAPTIVE_FIR_LEN  (MAX_TAPS)

#define DEFAULT_LMS_TAP_LEN     32

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
    BS_RES0,
    BS_AUTO_RUN,
    BS_AUTO_SEND,
    BS_WGN_ON,
    BS_RES1,
    BS_MAN_ERROR,
    BS_MAN_TAP_LEN,
    BS_MAN_FREQ_RANGE
};

struct BANDIT_SETTINGS {
    uint32_t        settings_bf;
    uint16_t        manual_tap_len_setting;
    uint32_t        manual_error_limit;
    uint32_t        manual_freq_range;
};

#define CHK_BANDIT_SETTING(a, b)        (a & (1u << b))
#define SET_BANDIT_SETTING(a, b)        (a |= (1u << b))
#define CLR_BANDIT_SETTING(a, b)        (a &= (1u << b))

#define BANDIT_DFL_SETTINGS     (1 << BS_ENABLE) | \
                                (1 << BS_AUTO_SEND)

#endif