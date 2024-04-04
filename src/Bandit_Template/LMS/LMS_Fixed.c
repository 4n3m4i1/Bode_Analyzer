#include "LMS_Fixed.h"

void LMS_Struct_Equate(struct LMS_Fixed_Inst *src, struct LMS_Fixed_Inst *dst){
    dst->tap_len = src->tap_len;
    dst->max_convergence_attempts = src->max_convergence_attempts;
    dst->iteration_ct = src->iteration_ct;
    dst->ddsmpl_stride = src->ddsmpl_stride;
    dst->fixed_offset = src->fixed_offset;
    dst->d_n_offset = src->d_n_offset;
    dst->max_error_allowed = src->max_error_allowed;
    dst->target_error = src->target_error;
    dst->learning_rate = src->learning_rate;
    dst->d_n = src->d_n;
    dst->samples_processed = src->samples_processed;
}

void LMS_Struct_Init(struct LMS_Fixed_Inst *LMS, Q15 tgt_err, Q15 max_acceptable_error, int16_t samples_offset, uint16_t max_runtime, uint16_t start_offset){
    LMS->target_error = tgt_err;
    LMS->d_n_offset = samples_offset;
    LMS->ddsmpl_stride = 0;
    LMS->max_error_allowed = max_acceptable_error;
    LMS->iteration_ct = max_runtime;
    LMS->fixed_offset = start_offset;
    LMS->samples_processed = 0;
    LMS->max_convergence_attempts = LMS_DFL_MAX_FAILURES;
}


Q15 LMS_Looper(const struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR, bool flush_FIR){
    Q15 retval = LMS_FAIL_DFL;
    Q15 *desired = LMS->d_n + LMS->fixed_offset;
    Q15 *white_noise = LMS->x_n + LMS->fixed_offset;
    uint32_t *sp_ct = &(LMS->samples_processed);

    if(LMS->d_n_offset){
        if(LMS->d_n_offset > 0){
            desired += LMS->d_n_offset;
        } else {
            white_noise -= LMS->d_n_offset;
        }
    }

    // If new run, flush the FIR buffer
    if(flush_FIR) flush_FIR_buffer_and_taps(WGN_FIR);

    //run_2n_FIR_cycle(struct Q15_FIR_PARAMS *a, Q15 new_data)
    uint16_t n;
    for(n = 0; n < LMS->iteration_ct; ++n){
        retval = *(desired++) - run_2n_FIR_cycle(WGN_FIR, *(white_noise++));
        LMS_Update_Taps(LMS, WGN_FIR, retval);

        *sp_ct++;        

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