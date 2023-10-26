#ifndef FFT_h
#define FFT_h

#include <stdlib.h>
#include "generic_include.h"
#include "Q15_sin_table.h"

#define NUM_SAMPLES         64
#define NUM_SAMPLES_M_1     63
#define LOG2_NUM_SAMPLES    6
#define SHIFT_AMOUNT        ((sizeof(Q15) * 8) - LOG2_NUM_SAMPLES)
// #define LOG2_N_WAVE 10


#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

#define FIX_2_FLOAT

Q15 zero_point_4 = float_2_Q15(0.4) ;

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES] ;
// And here's where we'll copy those samples for FFT calculation
Q15 fr[NUM_SAMPLES] ;
Q15 fi[NUM_SAMPLES] ;

//Q15 sin_table[NUM_SAMPLES]; // a table of sines for the FFT
//Q15 fr[NUM_SAMPLES], fi[NUM_SAMPLES];


void FFTfix(Q15 *fr, Q15 *fi) {

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
    for (m=1; m<NUM_SAMPLES_M_1; m++) {
        // swap odd and even bits
        mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ... 
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= SHIFT_AMOUNT ;
        // don't swap that which has already been swapped
        if (mr<=m) continue ;
        // swap the bit-reveresed indices
        tr = fr[m] ;
        fr[m] = fr[mr] ;
        fr[mr] = tr ;
        ti = fi[m] ;
        fi[m] = fi[mr] ;
        fi[mr] = ti ;
    }

    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// Danielson-Lanczos //////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    L = 1 ;
    // Log2 of number of samples, minus 1
    k = LOG2_NUM_SAMPLES - 1 ;
    // While the length of the FFT's being combined is less than the number of gathered samples
    while (L < NUM_SAMPLES) {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L<<1 ;
        // For each element in the FFT's that are being combined . . .
        for (m=0; m<L; ++m) { 
            // Lookup the trig values for that element
            j = m << k ;                         // index of the sine table
            wr =  sin_table[j + NUM_SAMPLES/4] ; // cos(2pi m/N)
            wi = -sin_table[j] ;                 // sin(2pi m/N)
            wr >>= 1 ;                          // divide by two
            wi >>= 1 ;                          // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i=m; i<NUM_SAMPLES; i+=istep) {
                // j gets the index of the FFT element being combined with i
                j = i + L ;
                // compute the trig terms (bottom half of the above matrix)
                tr = mul_Q15(wr, fr[j]) - mul_Q15(wi, fi[j]) ;
                ti = mul_Q15(wr, fi[j]) + mul_Q15(wi, fr[j]) ;
                // divide ith index elements by two (top half of above matrix)
                qr = fr[i]>>1 ;
                qi = fi[i]>>1 ;
                // compute the new values at each index
                fr[j] = qr - tr ;
                fi[j] = qi - ti ;
                fr[i] = qr + tr ;
                fi[i] = qi + ti ;
            }    
        }
        --k ;
        L = istep ;
    }
}


int FFTmag(Q15 *fr, Q15 *fi){
    Q15 max_fr = 0;
    int max_fr_dex = -1;

    for (int i = 0; i < (NUM_SAMPLES>>1); i++) {  
        // get the approx magnitude
        fr[i] = abs(fr[i]); //>>9
        fi[i] = abs(fi[i]);
        // reuse fr to hold magnitude
        fr[i] = max(fr[i], fi[i]) + 
                mul_Q15(min(fr[i], fi[i]), zero_point_4); 

        // Keep track of maximum
        if ((fr[i] > max_fr) && (i > 0)) {
            max_fr = fr[i] ;
            max_fr_dex = i ;
        }
    }

    return max_fr_dex;
}

#endif