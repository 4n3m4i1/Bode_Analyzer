#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <inttypes.h>



uint32_t    num_taps = 0;






double generate_awgn_sample();



void main(int argc, char **argv){

    FILE *tapfp;
    tapfp = fopen("test_ideal_tap_weights.txt","r");

    double rval;

    while(fscanf(tapfp,"%lf",&rval) != EOF) num_taps += 1;
    printf("Number of Ideal Taps: %u\n",num_taps);
    rewind(tapfp);

    double *ideal_taps = malloc(num_taps * sizeof(double));
    double *adaptive_taps = malloc(num_taps * sizeof(double));
    double *x = malloc(num_taps * sizeof(double));
    double *y = malloc(num_taps * sizeof(double));
    double *e = malloc(num_taps * sizeof(double));

    if(ideal_taps && adaptive_taps && x){



        free(ideal_taps);
        free(adaptive_taps);
        free(x);
        free(y);
        free(e);
    }
    fclose(tapfp);
}