#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "fixedpt_include.h"

#ifndef true
#define true    1u
#endif

#ifndef false
#define false   0u
#endif


Q15 D_N_0[1024];
Q15 X_N_0[1024];

// run from build folder
char dfl_path[] = "../../../../src/Bandit_Template/Debug_Logs/int_debug.log";

uint16_t numsamples;
uint16_t numtaps;



struct GlobalSettings {
    uint8_t     print_sample_data;
    uint8_t     output_float;
    char        *filepath;
    Q15         LMS_LEARNING_RATE;
    uint16_t    ITER_CTS;

    uint16_t    D_N_Delay;

    uint16_t    manlen;
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
    TSA_PreCheck.print_sample_data = false;
    TSA_PreCheck.output_float = false;
    TSA_PreCheck.filepath = 0;
    TSA_PreCheck.LMS_LEARNING_RATE = 0x0003;
    TSA_PreCheck.ITER_CTS = 32;
    TSA_PreCheck.D_N_Delay = 0;
    TSA_PreCheck.manlen = 0;
    
    
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
    uint16_t numsamples = 0;
    while(fgets(rvalz, sizeof(rvalz), datafp)){
        numsamples++;
    }

    rewind(datafp);

    numsamples >>= 1;

    for(uint16_t n = 0; n < numsamples; ++n){
        fscanf(datafp, "%d\n", &D_N_0[n]);
    }

    for(uint16_t n = 0; n < numsamples; ++n){
        fscanf(datafp, "%d\n", &X_N_0[n]);
    }

    fclose(datafp);


    if(TSA_PreCheck.print_sample_data){
        printf("Read %4u Samples\n", numsamples);
        printf("Num\tDesired\tInput Data\n");
        for(uint16_t n = 0; n < numsamples; ++n){
            printf("%4u\t%10d\t%10d\n", n, D_N_0[n], X_N_0[n]);
        }
    }

    // LMS time
    struct FIR_Parameters LMS_FIR;
    LMS_FIR.current_zero = 0;
    LMS_FIR.len = (TSA_PreCheck.manlen) ? TSA_PreCheck.manlen : numsamples;
    LMS_FIR.address_mask = LMS_FIR.len - 1;
    LMS_FIR.data = malloc(sizeof(Q15) * LMS_FIR.len);
    LMS_FIR.taps = malloc(sizeof(Q15) * LMS_FIR.len);

    for(uint16_t n = 0; n < LMS_FIR.len; ++n){
        LMS_FIR.data[n] = 0;
        LMS_FIR.taps[n] = 0;
    }

    if(!LMS_FIR.data || !LMS_FIR.taps){
        printf("Couldn't allocate memory :(\n");
        return;
    }

    for(uint16_t iter = 0; iter < TSA_PreCheck.ITER_CTS; ++iter){
        for(uint16_t n = 0; n < (numsamples - TSA_PreCheck.D_N_Delay); ++n){
            Q15 error = D_N_0[n] - run_2n_fir_cycle(&LMS_FIR, X_N_0[n + TSA_PreCheck.D_N_Delay]);
            update_LMS_taps(&LMS_FIR, error, TSA_PreCheck.LMS_LEARNING_RATE);
        }
    }

    FILE *outfp;
    outfp = fopen("../tap_output.txt", "w");

    if(TSA_PreCheck.output_float){
        for(uint16_t n = 0; n < LMS_FIR.len; ++n){
            fprintf(outfp, "%.10lf\n", Q15_2_float(LMS_FIR.taps[n]));
        }
    } else {
        for(uint16_t n = 0; n < LMS_FIR.len; ++n){
            fprintf(outfp, "%d\n", LMS_FIR.taps[n]);
        }
    }
    

    fclose(outfp);

    printf("Done!\n");

    free(LMS_FIR.data);
    free(LMS_FIR.taps);
    
}

Q15 run_2n_fir_cycle(struct FIR_Parameters *fir, Q15 new_data){
    fir->data[fir->current_zero & fir->address_mask] = new_data;
    Q15 retval = 0;
    for(uint16_t n = 0; n < fir->len; ++n){
        Q15 mulval = fir->data[(fir->current_zero + n) & fir->address_mask];
        retval += mul_Q15(mulval, fir->taps[n]);
    }
    fir->current_zero++;
    return retval;
}

void update_LMS_taps(struct FIR_Parameters *fir, Q15 error, Q15 learning_rate){
    for(uint16_t n = 0; n < fir->len; ++n){
        fir->taps[n] += mul_Q15(learning_rate, (mul_Q15(fir->data[n], error)));
    }
}



