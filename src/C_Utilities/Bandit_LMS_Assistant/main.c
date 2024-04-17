#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <time.h>
#include <sys/times.h>


#include "fixedpt_include.h"

#ifndef true
#define true    1u
#endif

#ifndef false
#define false   0u
#endif

#define NLMS


Q15 D_N_0[1024];
Q15 X_N_0[1024];

// run from build folder
char dfl_path[] = "../../../../src/Bandit_Template/Debug_Logs/int_debug.log";

uint16_t numsamples;
uint16_t numtaps;



struct GlobalSettings {
    uint8_t     print_sample_data;
    uint8_t     output_float;
    uint8_t     save_error;
    char        *filepath;
    Q15         LMS_LEARNING_RATE;
    uint32_t    ITER_CTS;

    uint32_t    D_N_Delay;

    uint32_t    manlen;

    uint32_t    manruntime;
};

uint8_t printedlearningratemessage = false;

struct GlobalSettings TSA_PreCheck;


struct FIR_Parameters {
    uint16_t len;
    uint16_t address_mask;
    uint16_t current_zero;

    Q15         *taps;
    Q15         *data;
};


// Function prototipica
Q15 run_2n_fir_cycle(struct FIR_Parameters *fir, Q15 new_data);
void update_LMS_taps(struct FIR_Parameters *fir, Q15 error, Q15 learning_rate);


void main(int argc, char **argv){
    struct timespec start_time, end_time;

    TSA_PreCheck.print_sample_data = false;
    TSA_PreCheck.output_float = false;
    TSA_PreCheck.filepath = 0;
#ifdef FLOATFIXED
    TSA_PreCheck.LMS_LEARNING_RATE = 0.001;
#else
    TSA_PreCheck.LMS_LEARNING_RATE = 0x0003;
#endif
    TSA_PreCheck.ITER_CTS = 32;
    TSA_PreCheck.D_N_Delay = 0;
    TSA_PreCheck.manlen = 0;
    TSA_PreCheck.save_error = true;
    
    
    
    if(argc > 1){
        for(int n = 1; n < argc; ++n){
            if(argv[n][0] == '-'){
                switch(argv[n][1]){
                    case 'I':
                    case 'i': {
                        if(n + 1 < argc){
                            TSA_PreCheck.ITER_CTS = (uint16_t)strtoul(argv[++n], NULL, 10);
                            printf("Running %u iterations\n", TSA_PreCheck.ITER_CTS);
                        } else {
                            printf("No length argument provided! Defaulting to %u\n", TSA_PreCheck.ITER_CTS);
                        }
                    }
                    break;

                    case 'R':
                    case 'r': {
                        if(n + 1 < argc){
                            TSA_PreCheck.LMS_LEARNING_RATE = float_2_Q15(strtod(argv[++n], NULL));
                            if(!TSA_PreCheck.LMS_LEARNING_RATE) TSA_PreCheck.LMS_LEARNING_RATE = 1;
                            printf("Set Learning rate to 0x%04X (%.8f)\n", TSA_PreCheck.LMS_LEARNING_RATE, Q15_2_float(TSA_PreCheck.LMS_LEARNING_RATE));
                        } else {
                            printf("No Learning rate provided! Defaulting to 0x%04X (%.8f)\n", TSA_PreCheck.LMS_LEARNING_RATE, Q15_2_float(TSA_PreCheck.LMS_LEARNING_RATE));
                        }
                        printedlearningratemessage = true;
                    }
                    break;

                    case 'L':
                    case 'l': {
                        if(n + 1 < argc){
                            TSA_PreCheck.manlen = (uint16_t)strtoul(argv[++n], NULL, 10);
                            printf("Set %u taps\n", TSA_PreCheck.manlen);
                        } else {
                            printf("Manual tap length not found, using fileread len.\n");
                        }
                    }
                    break;

                    case 'S':
                    case 's': {
                        if(n + 1 < argc){
                            TSA_PreCheck.D_N_Delay = (uint16_t)strtoul(argv[++n], NULL, 10);
                            printf("Set %u samples delayed\n", TSA_PreCheck.D_N_Delay);
                        } else {
                            printf("No sample delay value set! Defaulting to %u\n", TSA_PreCheck.D_N_Delay);
                        }
                    }
                    break;

                    case 'D':
                    case 'd': {
                        TSA_PreCheck.print_sample_data = true;
                    }
                    break;

                    case 'F':
                    case 'f': {
                        TSA_PreCheck.output_float = true;
                    }
                    break;

                    case 'Q':
                    case 'q': {
                        printf("Not saving data!\n");
                        TSA_PreCheck.save_error = false;
                    }
                    break;
                }
            }
        }
    }

    if(!TSA_PreCheck.filepath) TSA_PreCheck.filepath = dfl_path;

    if(TSA_PreCheck.output_float) printf("Output set to float.\n");
    else printf("Output set to int.\n");
    
    if(!printedlearningratemessage){
        printf("Set Learning rate to 0x%04X (%.8f)\n", TSA_PreCheck.LMS_LEARNING_RATE, Q15_2_float(TSA_PreCheck.LMS_LEARNING_RATE));
    }

    FILE *datafp;
    datafp = fopen(TSA_PreCheck.filepath, "r");


    char rvalz[32];
    uint32_t numsamples = 0;
    while(fgets(rvalz, sizeof(rvalz), datafp)){
        numsamples++;
    }

    rewind(datafp);

    numsamples >>= 1;

    FILE *xnfp = fopen("../X_N.txt", "w");
    FILE *dnfp = fopen("../D_N.txt", "w");

    for(uint16_t n = 0; n < numsamples; ++n){
        fscanf(datafp, "%d\n", &D_N_0[n]);
        fprintf(dnfp, "%lf\n", Q15_2_float(D_N_0[n]));
    }

    for(uint16_t n = 0; n < numsamples; ++n){
        fscanf(datafp, "%d\n", &X_N_0[n]);
        fprintf(xnfp, "%lf\n", Q15_2_float(X_N_0[n]));
    }

    fclose(xnfp);
    fclose(dnfp);
    fclose(datafp);


    printf("Read %4u Samples\n", numsamples);
    if(TSA_PreCheck.print_sample_data){
        printf("Num\tDesired\t\tInput Data\n");
        for(uint16_t n = 0; n < numsamples; ++n){
#ifdef LONGFIXED
            D_N_0[n] *= 32768;
            X_N_0[n] *= 32768;
#endif
            //printf("%4u\t%10d (%lf)\t%10d (%lf)\n", n, D_N_0[n], Q15_2_float(D_N_0[n]), X_N_0[n], Q15_2_float(X_N_0[n]));
        }
    }

    // LMS time
    struct FIR_Parameters LMS_FIR;
    LMS_FIR.current_zero = LMS_FIR.len;
    LMS_FIR.len = (TSA_PreCheck.manlen) ? TSA_PreCheck.manlen : numsamples;
    LMS_FIR.address_mask = LMS_FIR.len - 1;
    LMS_FIR.data = malloc(sizeof(Q15) * LMS_FIR.len);
    LMS_FIR.taps = malloc(sizeof(Q15) * LMS_FIR.len);

    uint32_t    countoferrorplot = (numsamples - TSA_PreCheck.D_N_Delay) * TSA_PreCheck.ITER_CTS;
    Q15 *errorplot = (Q15 *)1lu;
    Q15 *y_n_plot = (Q15 *)1lu;

    if(TSA_PreCheck.save_error == true){
        printf("Allocating error buffers.\n");
        errorplot = malloc(countoferrorplot * sizeof(Q15));
        y_n_plot = malloc(countoferrorplot * sizeof(Q15));
    }

    for(uint32_t n = 0; n < LMS_FIR.len; ++n){
        LMS_FIR.data[n] = 0;
        LMS_FIR.taps[n] = 0;
    }

    if(!LMS_FIR.data || !LMS_FIR.taps || !errorplot || !y_n_plot){
        printf("Couldn't allocate memory :(\n");
        return;
    } else {
        if(TSA_PreCheck.save_error == true) printf("Saving %llu error samples\n", countoferrorplot);
        else printf("Not saving error plot!\n");
    }

    clock_gettime(CLOCK_MONOTONIC, &start_time);

    uint32_t ctr = 0;

    // MAIN LOOP

    for(uint32_t iter = 0; iter < TSA_PreCheck.ITER_CTS; ++iter){
        for(uint16_t n = 0; n < (numsamples - TSA_PreCheck.D_N_Delay); ++n){
            //Q15 Y_N = run_2n_fir_cycle(&LMS_FIR, X_N_0[(numsamples - 1) - n]);
            Q15 Y_N = run_2n_fir_cycle(&LMS_FIR, X_N_0[n]);
            //printf("Y_N[%3u] = %d (%lf)\n", n, Y_N, Q15_2_float(Y_N));
            Q15 error = D_N_0[n] - Y_N;
            if(TSA_PreCheck.save_error == true){
                y_n_plot[ctr] = Y_N;
                errorplot[ctr++] = error;
            }
            update_LMS_taps(&LMS_FIR, error, TSA_PreCheck.LMS_LEARNING_RATE);
            LMS_FIR.current_zero--;
        }
    }

    // MAIN LOOP

    clock_gettime(CLOCK_MONOTONIC, &end_time);

    FILE *outfp;
    outfp = fopen("../tap_output.txt", "w");

    FILE *errfp;
    errfp = fopen("../errorplot.txt", "w");

    if(TSA_PreCheck.output_float){
        for(uint32_t n = 0; n < LMS_FIR.len; ++n){
            fprintf(outfp, "%.10lf\n", Q15_2_float(LMS_FIR.taps[n]));
        }
        if(TSA_PreCheck.save_error == true){
            for(uint32_t n = 0; n < countoferrorplot; ++n){
                fprintf(errfp, "%.10f\t%.10lf\n", Q15_2_float(errorplot[n]), Q15_2_float(y_n_plot[n]));
            }
        }
    } else {
        for(uint32_t n = 0; n < LMS_FIR.len; ++n){
            fprintf(outfp, "%d\n", LMS_FIR.taps[n]);
        }
        if(TSA_PreCheck.save_error == true){
            for(uint32_t n = 0; n < countoferrorplot; ++n){
                fprintf(errfp, "%d\t%d\n", errorplot[n], y_n_plot[n]);
            }
        }
    }
    
    fclose(errfp);
    fclose(outfp);

    double execution_time = (end_time.tv_sec - start_time.tv_sec) +
                            (end_time.tv_nsec - start_time.tv_nsec) / 1e9;

    printf("Done in %.8lf seconds (%.2lf minutes)!\n", execution_time, execution_time / 60.0);

    if(TSA_PreCheck.save_error == true){
        free(y_n_plot);
        free(errorplot);
    }
    free(LMS_FIR.data);
    free(LMS_FIR.taps);
    
}

Q15 run_2n_fir_cycle(struct FIR_Parameters *fir, Q15 new_data){
    Q15 retval = 0;
    
    //fir->data[fir->current_zero & fir->address_mask] = new_data;
    //
    //for(uint16_t n = 0; n < fir->len; ++n){
    //    Q15 mulval = fir->data[(fir->current_zero + n) & fir->address_mask];
    //    retval += mul_Q15(mulval, fir->taps[n]);
    //}
    ////fir->current_zero--;
    //

    fir->data[0] = new_data;
    for(uint32_t n = 0; n < fir->len; ++n) retval += mul_Q15(fir->taps[n] , fir->data[n]);
    for(uint32_t n = fir->len - 1; n > 0; --n) fir->data[n] = fir->data[n - 1];

    //printf("FIR: data[0] = %d (%lf)\t->\t%d (%lf)\n", new_data, Q15_2_float(new_data), retval, Q15_2_float(retval));

    return retval;
}



void update_LMS_taps(struct FIR_Parameters *fir, Q15 error, Q15 learning_rate){
#ifdef NLMS
    Q15 data_dot_prod = 0;
    for(uint32_t m = 0; m < fir->len; ++m){
        data_dot_prod += mul_Q15(fir->data[m], fir->data[m]);
    }
    //printf("DORPROD: %d (%lf)\n", data_dot_prod, Q15_2_float(data_dot_prod));
    if(!data_dot_prod) data_dot_prod = 1;
#endif
    
    for(uint16_t n = 0; n < fir->len; ++n){
        Q15 dataval = fir->data[(n + fir->current_zero) & fir->address_mask];
        Q15 tap_error = mul_Q15(dataval, error);
        //printf("Data %0.14lf Err: 0x%04X\t%.14lf\n", Q15_2_float(fir->data[n]), tap_error, Q15_2_float(tap_error));
        Q15 std_lms_update = mul_Q15(learning_rate, tap_error);
#ifdef NLMS
        std_lms_update = div_Q15(std_lms_update, data_dot_prod);
#endif
        fir->taps[n] += std_lms_update;
    }
}



