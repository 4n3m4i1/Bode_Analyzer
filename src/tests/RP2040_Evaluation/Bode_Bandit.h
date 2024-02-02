#ifndef BODE_BANDIT_h
#define BODE_BANDIT_h

#include "generic_include.h"
#include "Memory_Management.h"
#include "FFT/fft_half.h"
#include "circular_buffers/RP2040_Circ_Buffer.h"
#include "FIR/FIR_Fast_Fixed.h"
#include "LMS/LMS_Fixed.h"


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
*/



#endif