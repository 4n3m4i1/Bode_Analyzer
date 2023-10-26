#ifndef FFT_h
#define FFT_h

#include <stdlib.h>
#include "generic_include.h"
#include "Q15_sin_table.h"

//#define NUM_SAMPLES         64
//#define NUM_SAMPLES_M_1     64
//#define LOG2_NUM_SAMPLES    6
//#define SHIFT_AMOUNT        ((sizeof(Q15) * 8) - LOG2_NUM_SAMPLES)
// #define LOG2_N_WAVE 10


#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

#define FIX_2_FLOAT

Q15 zero_point_4 = float_2_Q15(0.4) ;

// Here's where we'll have the DMA channel put ADC samples
//uint8_t sample_array[NUM_SAMPLES] ;
//// And here's where we'll copy those samples for FFT calculation
//Q15 fr[NUM_SAMPLES] ;
//Q15 fi[NUM_SAMPLES] ;

struct FFT_PARAMS {
    uint16_t    num_samples;
    uint16_t    log2_num_samples;
    uint16_t    shift_amount;

    Q15 *fr;
    Q15 *fi;
};


uint16_t get_log_2(uint16_t num_samples){
    for(uint16_t n = 0; n < 16; n++){
        num_samples >>= 1;
        if(!num_samples) return (n);
    }
    return 0;
}

uint16_t get_shift_amt(uint16_t log2ns){
    return ((sizeof(Q15) * 8) - log2ns);
}


void FFT_fixdpt(struct FFT_PARAMS *fft) {

    unsigned short m;   // one of the indices being swapped
    unsigned short mr ; // the other index being swapped (r for reversed)
    Q15 tr, ti ; // for temporary storage while swapping, and during iteration

    int i, j ; // indices being combined in Danielson-Lanczos part of the algorithm
    int L ;    // length of the FFT's being combined
    int k ;    // used for looking up trig values from sine table

    int istep ; // length of the FFT which results from combining two FFT's

    Q15 wr, wi ; // trigonometric values from lookup table
    Q15 qr, qi ; // temporary variables used during DL part of the algorithm

    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// BIT REVERSAL //////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Bit reversal code below based on that found here: 
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious

    for (m = 1; m < fft->num_samples - 1; ++m) {
        // swap odd and even bits
        mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ... 
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= fft->shift_amount;
        // don't swap that which has already been swapped
        if (mr<=m) continue ;
        // swap the bit-reveresed indices
        tr = fft->fr[m] ;
        fft->fr[m] = fft->fr[mr] ;
        fft->fr[mr] = tr ;
        ti = fft->fi[m] ;
        fft->fi[m] = fft->fi[mr] ;
        fft->fi[mr] = ti ;
    }

    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// Danielson-Lanczos //////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    L = 1 ;
    // Log2 of number of samples, minus 1
    k = fft->log2_num_samples - 1 ;
    // While the length of the FFT's being combined is less than the number of gathered samples
    while (L < fft->num_samples) {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L<<1 ;
        // For each element in the FFT's that are being combined . . .
        for (m=0; m<L; ++m) { 
            // Lookup the trig values for that element
            j = m << k ;                         // index of the sine table
            wr =  sin_table[j + (fft->num_samples >> 2)] ; // cos(2pi m/N)
            wi = -sin_table[j] ;                 // sin(2pi m/N)
            wr >>= 1 ;                          // divide by two
            wi >>= 1 ;                          // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i = m; i < fft->num_samples; i += istep) {
                // j gets the index of the FFT element being combined with i
                j = i + L ;
                // compute the trig terms (bottom half of the above matrix)
                tr = mul_Q15(wr, fft->fr[j]) - mul_Q15(wi, fft->fi[j]) ;
                ti = mul_Q15(wr, fft->fi[j]) + mul_Q15(wi, fft->fr[j]) ;
                // divide ith index elements by two (top half of above matrix)
                qr = fft->fr[i] >> 1 ;
                qi = fft->fi[i] >> 1 ;
                // compute the new values at each index
                fft->fr[j] = qr - tr ;
                fft->fi[j] = qi - ti ;
                fft->fr[i] = qr + tr ;
                fft->fi[i] = qi + ti ;
            }    
        }
        --k ;
        L = istep ;
    }
}


int FFT_mag(struct FFT_PARAMS *fft){
    Q15 max_fr = 0;
    int max_fr_dex = -1;

    for (int i = 0; i < (fft->num_samples >> 1); i++) {  
        // get the approx magnitude
        fft->fr[i] = abs(fft->fr[i]); //>>9
        //fft->fi[i] = abs(fft->fi[i]);
        // reuse fr to hold magnitude
        fft->fr[i] = max(fft->fr[i], abs(fft->fi[i])) + mul_Q15(min(fft->fr[i], abs(fft->fi[i])), zero_point_4); 

        // Keep track of maximum
        if ((fft->fr[i] > max_fr) && (i > 0)) {
            max_fr = fft->fr[i] ;
            max_fr_dex = i ;
        }
    }

    return max_fr_dex;
}

#endif