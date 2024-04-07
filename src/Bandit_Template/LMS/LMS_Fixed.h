#ifndef LMS_FIXED_h
#define LMS_FIXED_h

#include "generic_include.h"
#include "pico/stdlib.h"
#include "../FIR/FIR_Fast_Fixed.h"
/*
    Sample then run LMS algorithm to maximize execution speed.
    Downsampling should be complete before execution.
*/

/*
    A note on offsets:
        Due to real world concerns the X[n] and D[n] will not be synronized.
        Do correct for this we use X[0] as a reference, such that the delay
        added by a system under test can be known.

        Thus:
            An offset of 0 implies perfect synchronization.
            An offset of >0 implies a delay ( X[n] should be paired with D[n+offset])
            An offset of <0 implies a leading signal ( X[n + offset] is paired with D[n])

            The last case is not possible, but is accounted for.
*/

enum LMS_LOOPER_RETURNS {
    LMS_OK,
    LMS_FAIL_DFL,
    LMS_FAIL_NO_CONVERGENCE,
    LMS_FAIL_ERR_2_HI
};

#define LMS_DFL_MAX_FAILURES    32

struct LMS_Fixed_Inst {
    uint16_t    tap_len;            // Number of taps (h hats)
    uint16_t    max_convergence_attempts;   // Maximum amount of retries before FAILURE
    uint16_t    iteration_ct;       // Max Samples to Run Thru System
    uint16_t    ddsmpl_stride;      // Downsampling stride if not handled in Digi downsampler itself
    uint16_t    fixed_offset;       // Offset to both x[n] and d[n] due to downsampling FIR fillup time
    int16_t     d_n_offset;         // Offset in samples from 0th x[n] wgn sample. If <0 x[n] must be incremented.
                                    //  Synchronizes the sample banks in time, can be weird group delay thru DUT
                                    //  or something like that! Will likely have to be fixed, unless a DUT calibration
                                    //  cycle is to be implemented (hard)

    Q15         max_error_allowed;  // Maximum allowed error, check at runtime expiry
    Q15         target_error;       // Pass go and collect 200 if average e is under this

    Q15         learning_rate;      // beta, alpha, or whatever new greek letter they say this should be

    Q15         *d_n;               // Desired / Target Data. Manually allocate pointer
    Q15         *x_n;               // White Noise Source data. Manually allocate pointer

    uint32_t    samples_processed;  // Number of samples run until convergence
};

void LMS_Struct_Equate(struct LMS_Fixed_Inst *src, struct LMS_Fixed_Inst *dst);
void LMS_Struct_Init(struct LMS_Fixed_Inst *LMS, Q15 tgt_err, Q15 max_acceptable_error, int16_t samples_offset, uint16_t max_runtime, uint16_t start_offset);
//void Associate_LMS_and_FIR_Struct(struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *FIR);
Q15 LMS_Looper(struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR, bool flush_FIR);
void LMS_Update_Taps(const struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR, Q15 error);







#endif