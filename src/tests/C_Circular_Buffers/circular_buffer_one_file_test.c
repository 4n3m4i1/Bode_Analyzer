#include <stdio.h>
#include <inttypes.h>
//#include <malloc.h>
#include <time.h>
#include <stdlib.h>

#define FIR_TEST

#ifdef FIR_TEST
#define FIR0_LEN    32

#define FLOAT_CONST     32768.0
#define FL_2_INT(a)     ((int32_t)(a * FLOAT_CONST))
#define INT_2_FL(a)     (((double)(a)) / FLOAT_CONST)
#define FIXED_MUL(a,b)  ((int32_t)((((int64_t)a) * ((int64_t)b)) >> 15 ))

double firtaps[FIR0_LEN] = {
0.029183359514055443,
0.029779744780144553,
0.030341314035934788,
0.030866756254078743,
0.031354842164334006,
0.031804427695546707,
0.032214457182525101,
0.032583966327994970,
0.032912084910572369,
0.033198039230461426,
0.033441154285384696,
0.033640855670076741,
0.033796671193517147,
0.033908232208942304,
0.033975274652555849,
0.033997639787750347,
0.033975274652555849,
0.033908232208942304,
0.033796671193517147,
0.033640855670076741,
0.033441154285384696,
0.033198039230461426,
0.032912084910572369,
0.032583966327994970,
0.032214457182525101,
0.031804427695546707,
0.031354842164334006,
0.030866756254078743,
0.030341314035934788,
0.029779744780144553,
0.029183359514055443,
0
};
#endif

struct Circular_Buffer_Int32 {
    uint16_t    size;
    uint16_t    size_mask;      // for 2^n len
    uint16_t    curr_zero;
    int32_t     *data;
};

void setup_circular_buffer(struct Circular_Buffer_Int32 *a, int size){
    if(a){
        a->size = size;
        a->size_mask = 0;
        a->curr_zero = 0;
        a->data = malloc(size * sizeof(int32_t));

        int ones_ct = 0;
        for(int n = 0; n < 32; ++n){
            if(a->size & (1u << n)) ones_ct++;
        }

        if(ones_ct == 1){
            int n;
            for(n = 0; n < 32; ++n){
                if((a->size >> n) == 1) break;
                else {
                    a->size_mask |= (1u << n);
                }
            }
        }
    }
}

void static inline add_sample_to_buffer(struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero] = val;
    if(++a->curr_zero >= a->size) a->curr_zero = 0;
}

void static inline add_sample_to_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero++ & a->size_mask] = val;
}

int32_t static inline recall_sample_from_buffer(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(((offset + a->curr_zero) >= a->size) ? a->curr_zero + (offset -= a->size) : (a->curr_zero + offset))];
}

int32_t static inline recall_sample_from_2n_buffer(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(offset + a->curr_zero) & a->size_mask];
}

void destroy_circular_buffer(struct Circular_Buffer_Int32 *a){
    if(a){
        free(a->data);
    }
}


/*
    Test FIR based on those circ buffers
*/
struct FIR_PARAMS {
    struct Circular_Buffer_Int32    cbuffer;
    int32_t     *tap;
};

void setup_FIR(struct FIR_PARAMS *a, int size){
    if(a){
        a->cbuffer.size = size;
        a->cbuffer.size_mask = 0;
        a->cbuffer.curr_zero = 0;
        a->tap = malloc(size * sizeof(int32_t));
        a->cbuffer.data = malloc(size * sizeof(int32_t));

        int ones_ct = 0;
        for(int n = 0; n < 32; ++n){
            if(a->cbuffer.size & (1u << n)) ones_ct++;
        }

        if(ones_ct == 1){
            int n;
            for(n = 0; n < 32; ++n){
                if((a->cbuffer.size >> n) == 1) break;
                else {
                    a->cbuffer.size_mask |= (1u << n);
                }
            }
        }
    }
}

void setup_FIR_no_malloc(struct FIR_PARAMS *a, int size){
    if(a){
        a->cbuffer.size = size;
        a->cbuffer.size_mask = 0;
        a->cbuffer.curr_zero = 0;

        int ones_ct = 0;
        for(int n = 0; n < 32; ++n){
            if(a->cbuffer.size & (1u << n)) ones_ct++;
        }

        if(ones_ct == 1){
            int n;
            for(n = 0; n < 32; ++n){
                if((a->cbuffer.size >> n) == 1) break;
                else {
                    a->cbuffer.size_mask |= (1u << n);
                }
            }
        }
    }
}

void static inline add_sample_to_FIR(struct FIR_PARAMS *a, int32_t val){
    a->cbuffer.data[a->cbuffer.curr_zero] = val;
    if(++a->cbuffer.curr_zero >= a->cbuffer.size) a->cbuffer.curr_zero = 0;
}

void static inline add_sample_to_2n_FIR_I32(struct FIR_PARAMS *a, int32_t val){
    a->cbuffer.data[a->cbuffer.curr_zero++ & a->cbuffer.size_mask] = val;
}

void static inline add_sample_to_2n_FIR_I32_no_inc(struct FIR_PARAMS *a, int32_t val){
    a->cbuffer.data[a->cbuffer.curr_zero & a->cbuffer.size_mask] = val;
}

int32_t static inline recall_sample_from_FIR(struct FIR_PARAMS *a, int offset){
    return a->cbuffer.data[(((offset + a->cbuffer.curr_zero) >= a->cbuffer.size) ? a->cbuffer.curr_zero + (offset -= a->cbuffer.size) : (a->cbuffer.curr_zero + offset))];
}

int32_t static inline recall_sample_from_2n_FIR(struct FIR_PARAMS *a, int offset){
    return a->cbuffer.data[(offset + a->cbuffer.curr_zero) & a->cbuffer.size_mask];
}

int32_t static inline read_inc_2n_FIR(struct FIR_PARAMS *a){
    return a->cbuffer.data[a->cbuffer.curr_zero++ & a->cbuffer.size_mask];
}

int32_t static inline run_2n_FIR_cycle(struct FIR_PARAMS *a, int32_t new_data){
    add_sample_to_2n_FIR_I32_no_inc(a, new_data);
    int32_t retval = 0;
    for(int n = 0; n < a->cbuffer.size; ++n){
        retval += FIXED_MUL((recall_sample_from_2n_FIR(a, n)), a->tap[n]);
    }
    a->cbuffer.curr_zero++;
    return retval;
}

void destroy_FIR_buffer(struct FIR_PARAMS *a){
    if(a){
        free(a->tap);
        free(a->cbuffer.data);
    }
}


void main(){
#ifdef CIRC_ONLY_TEST
    struct Circular_Buffer_Int32 sbuf;

    int buflen = 64;

    setup_circular_buffer(&sbuf, buflen);

    for(int n = 0; n < buflen; ++n){
        sbuf.data[n] = n;
    }

    // Simulae FIR style fill
    for(int m = 0; m < 12; ++m){        // 12 samples go in
        //add_sample_to_buffer(&sbuf, m + 100);
        add_sample_to_2n_buffer_I32(&sbuf, m+100);
    }

    sbuf.curr_zero = 0;

    for(int n = 0; n < buflen << 1; ++n){
        printf("Buffer[%3d] = %d\n", n % buflen, recall_sample_from_2n_buffer(&sbuf, n));
    }

    destroy_circular_buffer(&sbuf);
#endif

#ifdef FIR_TEST
    FILE *fp;
    fp = fopen("coolnumbers.txt","w");

    srand(1234567);
    struct FIR_PARAMS fir0;
    int32_t fir_d_buffer[FIR0_LEN];
    int32_t fir_taps[FIR0_LEN];

    setup_FIR_no_malloc(&fir0, FIR0_LEN);
    fir0.cbuffer.data = fir_d_buffer;
    fir0.tap = fir_taps;

    double tapsum = 0;
    int32_t tapsum_i = 0;
    for(int n = 0; n < FIR0_LEN; ++n){
        printf("Tap %2d: %lf\t->%d\n", n, firtaps[n], FL_2_INT(firtaps[n]));
        fir0.tap[n] = FL_2_INT(firtaps[n]);
        
        tapsum += firtaps[n];
        tapsum_i += FL_2_INT(firtaps[n]);
        // Clear out Fir buffer
        run_2n_FIR_cycle(&fir0, 0);
    }

    printf("Tap Sum: %lf\t%d -> %lf\n", tapsum, tapsum_i, INT_2_FL(tapsum_i));

    for(int n = 0; n < 4096; ++n){
        //int32_t val = ((n > (8096 / 2)) ? 32768 : 0);     // Step
        int32_t val = (n%512 > 256) * 32768;
        fprintf(fp, "%.14lf\t%.14lf\n", INT_2_FL(val), INT_2_FL(run_2n_FIR_cycle(&fir0, val)));
    }

    fclose(fp);


#endif
}