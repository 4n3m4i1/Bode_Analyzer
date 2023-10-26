#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define FIX_2_FLOAT

#include "generic_include.h"
#include "FFT.h"

double dfl_scale = 2.0;
double amp_scale = 1.0;

void main(int argc, char **argv){
    if(argc > 1) dfl_scale = strtod(argv[1],NULL);
    if(argc > 2) amp_scale = strtod(argv[2],NULL);

    for(int n = 0; n < NUM_SAMPLES; ++n){
        fr[n] = float_2_Q15(amp_scale * sin(dfl_scale * 6.283 * ((float) n) / (float)NUM_SAMPLES));
        fi[n] = int_2_Q15(0);
    }

    FFTfix(fr,fi);
    int max_bin = FFTmag(fr,fi);

    for(int n = 0; n < (NUM_SAMPLES >> 1); ++n){
        printf("Bin %3u -> %f", n, Q15_2_float(fr[n]));
        if(n == max_bin){
            printf("\tMAX");
        }
        printf("\n");
    }


}