#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define FIX_2_FLOAT

#include "generic_include.h"
#include "FFT.h"

double dfl_scale = 2.0;
double amp_scale = 1.0;
double phase = 0.0;

#define NUM_SAMPLES 64

void main(int argc, char **argv){
    printf("ZERO 4: 0x%08X\n", zero_point_4);
    struct FFT_PARAMS fft;
    fft.num_samples = NUM_SAMPLES;
    fft.log2_num_samples = get_log_2(NUM_SAMPLES);
    fft.shift_amount = get_shift_amt(fft.log2_num_samples);
    fft.true_max = NUM_SAMPLES;

    printf("NumSamples: %u\n", NUM_SAMPLES);
    printf("Log2NS:     %u\n",fft.log2_num_samples);
    printf("Shift Amt:  %u\n",fft.shift_amount);

    Q15 ffr[NUM_SAMPLES];
    Q15 ffi[NUM_SAMPLES];

    fft.fr = ffr;
    fft.fi = ffi;
    
    if(argc > 1) dfl_scale = strtod(argv[1],NULL);
    if(argc > 2) amp_scale = strtod(argv[2],NULL);
    if(argc > 3) phase = strtod(argv[3],NULL);

    for(int n = 0; n < NUM_SAMPLES; ++n){
        fft.fr[n] = float_2_Q15(amp_scale * sin((dfl_scale * 6.283 * ((float) n) / (float)NUM_SAMPLES) + phase));
        fft.fi[n] = int_2_Q15(0);
    }

    FFT_fixdpt(&fft);
    int max_bin = FFT_mag(&fft);

    for(int n = 0; n < (fft.num_samples >> 1); ++n){
        printf("Bin %3u -> %f\t%f", n, Q15_2_float(fft.fr[n]), Q15_2_float(fft.fi[n]));
        if(n == max_bin){
            printf("\tMAX");
        }
        printf("\n");
    }


}