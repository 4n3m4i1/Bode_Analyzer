#ifndef FIR_FAST_FIXED_h
#define FIR_FAST_FIXED_h

#include "generic_include.h"

/*
    This library heavily prioritizes FIR structures of 2^n length.
*/

struct Q15_FIR_PARAMS {
    uint16_t    size_true;      // Real size of arrays
    uint16_t    size;           // Virtually this many words long..
    uint16_t    size_mask;      // 2^n size mask for rollover w &
    uint16_t    curr_zero;      // Current write/read ptr
    Q15         *data;          // Shift reg data
    Q15         *taps;          // Taps
};

// Must manually assign pointers for data and taps!!
void setup_Q15_FIR(struct Q15_FIR_PARAMS *a, uint16_t size);

void add_sample_to_FIR(struct Q15_FIR_PARAMS *a, Q15 val);
void add_sample_to_2n_FIR_I16(struct Q15_FIR_PARAMS *a, Q15 val);
void add_sample_to_2n_FIR_I16_no_inc(struct Q15_FIR_PARAMS *a, Q15 val);

Q15 recall_sample_from_FIR(struct Q15_FIR_PARAMS *a, int offset);
Q15 recall_sample_from_2n_FIR(struct Q15_FIR_PARAMS *a, int offset);
Q15 read_inc_2n_FIR(struct Q15_FIR_PARAMS *a);
Q15 run_2n_FIR_cycle(struct Q15_FIR_PARAMS *a, Q15 new_data);

void flush_FIR_buffer(struct Q15_FIR_PARAMS *a);
void flush_FIR_taps(struct Q15_FIR_PARAMS *a);
void flush_FIR_buffer_and_taps(struct Q15_FIR_PARAMS *a);

#endif