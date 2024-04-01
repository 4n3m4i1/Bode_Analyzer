#include "LMS_Fixed.h"


void LMS_Struct_Init(struct LMS_Fixed_Inst *LMS, Q15 tgt_err, Q15 max_acceptable_error, int16_t samples_offset, uint16_t max_runtime, uint16_t start_offset){
    LMS->target_error = tgt_err;
    LMS->d_n_offset = samples_offset;
    LMS->ddsmpl_stride = 0;
    LMS->max_error_allowed = max_acceptable_error;
    LMS->iteration_ct = max_runtime;
    LMS->fixed_offset = start_offset;
}


Q15 LMS_Looper(const struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR){
    Q15 retval = LMS_FAIL_DFL;
    Q15 *desired = LMS->d_n + LMS->fixed_offset;
    Q15 *white_noise = LMS->x_n + LMS->fixed_offset;
    if(LMS->d_n_offset){
        if(LMS->d_n_offset > 0){
            desired += LMS->d_n_offset;
        } else {
            white_noise -= LMS->d_n_offset;
        }
    }

    //setup_Q15_FIR(WGN_FIR, LMS->tap_len);     // Should be setup by now..
    flush_FIR_buffer_and_taps(WGN_FIR);

    //run_2n_FIR_cycle(struct Q15_FIR_PARAMS *a, Q15 new_data)
    uint16_t n;
    for(n = 0; n < LMS->iteration_ct; ++n){
        retval = *(desired++) - run_2n_FIR_cycle(WGN_FIR, *(white_noise++));
        LMS_Update_Taps(LMS, WGN_FIR, retval);

        // Error processing Stuff goes here!!!!!!!!!!!
        //  break if avg < min err
        if(retval < LMS->target_error) break;
    }

    if(n >= LMS->iteration_ct){
        if(retval > LMS->max_error_allowed) retval = LMS_FAIL_DFL;
    }

    // Error should be acceptable by now, or we've failed and everything sucks..
    return retval;
}

void LMS_Update_Taps(const struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR, Q15 error){
    for(uint16_t n = 0; n < LMS->tap_len; ++n){
        Q15 tmp = mul_Q15(error, WGN_FIR->data[n]);
        //WGN_FIR->taps[n] += (mul_Q15(LMS->learning_rate, mul_Q15(error, WGN_FIR->data[n])));
        WGN_FIR->taps[n] += mul_Q15(LMS->learning_rate, tmp);
    }
}