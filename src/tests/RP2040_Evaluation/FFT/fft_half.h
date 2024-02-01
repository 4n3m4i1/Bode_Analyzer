#ifndef FFT_h
#define FFT_h

#include <pico/stdlib.h>
#include "generic_include.h"
#include "Q15_sin_table.h"

//#define NUM_SAMPLES         64
//#define NUM_SAMPLES_M_1     64
//#define LOG2_NUM_SAMPLES    6
//#define SHIFT_AMOUNT        ((sizeof(Q15) * 8) - LOG2_NUM_SAMPLES)
// #define LOG2_N_WAVE 10


#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

//#define FIX_2_FLOAT

// Here's where we'll have the DMA channel put ADC samples
//uint8_t sample_array[NUM_SAMPLES] ;
//// And here's where we'll copy those samples for FFT calculation
//Q15 fr[NUM_SAMPLES] ;
//Q15 fi[NUM_SAMPLES] ;

struct FFT_PARAMS {
    uint16_t    num_samples;
    uint16_t    log2_num_samples;
    uint16_t    shift_amount;
    uint16_t    true_max;

    Q15 *fr;
    Q15 *fi;
};


uint16_t get_log_2(uint16_t num_samples);

uint16_t get_shift_amt(uint16_t log2ns);

void FFT_fixdpt(struct FFT_PARAMS *fft);

int FFT_mag(struct FFT_PARAMS *fft);

#endif