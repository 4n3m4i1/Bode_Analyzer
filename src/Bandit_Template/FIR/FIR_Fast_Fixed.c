#include "FIR_Fast_Fixed.h"

void setup_Q15_FIR(struct Q15_FIR_PARAMS *a, uint16_t size){
    if(a){
        a->size = size;
        a->size_mask = 0;
        a->curr_zero = 0;
        
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

inline void add_sample_to_FIR(struct Q15_FIR_PARAMS *a, Q15 val){
    a->data[a->curr_zero] = val;
    if(++a->curr_zero >= a->size) a->curr_zero = 0;
}

inline void add_sample_to_2n_FIR_I16(struct Q15_FIR_PARAMS *a, Q15 val){
    a->data[a->curr_zero++ & a->size_mask] = val;
}

inline void add_sample_to_2n_FIR_I16_no_inc(struct Q15_FIR_PARAMS *a, Q15 val){
    a->data[a->curr_zero & a->size_mask] = val;
}

Q15 recall_sample_from_FIR(struct Q15_FIR_PARAMS *a, int offset){
    return a->data[(((offset + a->curr_zero) >= a->size) ? a->curr_zero + (offset -= a->size) : (a->curr_zero + offset))];
}

inline Q15 recall_sample_from_2n_FIR(struct Q15_FIR_PARAMS *a, int offset){
    return a->data[(offset + a->curr_zero) & a->size_mask];
}

inline Q15 read_inc_2n_FIR(struct Q15_FIR_PARAMS *a){
    return a->data[a->curr_zero++ & a->size_mask];
}



inline Q15 run_2n_FIR_cycle(struct Q15_FIR_PARAMS *a, Q15 new_data){
    add_sample_to_2n_FIR_I16_no_inc(a, new_data);
    Q15 retval = 0;
    for(int n = 0; n < a->size; ++n){
        retval += mul_Q15((recall_sample_from_2n_FIR(a, n)), a->taps[n]);
    }
    a->curr_zero++;
    return retval;
}


void flush_FIR_buffer(struct Q15_FIR_PARAMS *a){
    for(int n = 0; n < a->size; ++n){
        a->data[n] = 0;
    }
}

void flush_FIR_taps(struct Q15_FIR_PARAMS *a){
    for(int n = 0; n < a->size; ++n){
        a->taps[n] = 0;
    }
}

void flush_FIR_buffer_and_taps(struct Q15_FIR_PARAMS *a){
    a->curr_zero = 0;
    for(int n = 0; n < a->size; ++n){
        a->data[n] = 0;
        a->taps[n] = 0;
    }
}