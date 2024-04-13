#ifndef FFT_h
#define FFT_h

#include <pico/stdlib.h>
#include "fixedpt_include.h"
#include "Q15_sin_table.h"
//#include "Q15_cos_table.h"

#define TABLE_SIZE_2N       10
#define SINTABLESIZE        1024

#define max_fft(a,b)        ((a > b) ? a : b)
#define min_fft(a,b)        ((a < b) ? a : b)
#define abs_fft(a)      ((a < 0) ? (-1 * a) : (a))


struct FFT_PARAMS {
    uint16_t    num_samples;
    uint16_t    log2_num_samples;
    uint16_t    shift_amount;
    uint16_t    true_max;

    uint16_t    table_scalar_offset;

    Q15 *fr;
    Q15 *fi;
};


// Simple log2() function
uint16_t get_log_2(uint16_t num_samples);

// Shift amount for FFT index bit reversal
uint16_t get_shift_amt(uint16_t log2ns);

// Scale fixed trig table addressing to any 2^n <= 1024 (10)
void get_table_offset_shift(struct FFT_PARAMS *fft, uint16_t table_max_size);

// Cooley Tukey FFT
void FFT_fixdpt(struct FFT_PARAMS *fft);

// Extract magnitude from FFT result
int FFT_mag(struct FFT_PARAMS *fft);

#endif