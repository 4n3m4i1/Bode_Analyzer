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
    dst->x_n = src->x_n;
    dst->samples_processed = src->samples_processed;
    dst->error = src->error;
}

void LMS_Struct_Init(struct LMS_Fixed_Inst *LMS, Q15 tgt_err, Q15 max_acceptable_error, int16_t samples_offset, uint16_t max_runtime, uint16_t start_offset){
    LMS->target_error = tgt_err;
    LMS->d_n_offset = samples_offset;
    LMS->ddsmpl_stride = 1;
    LMS->max_error_allowed = max_acceptable_error;
    LMS->iteration_ct = max_runtime;
    LMS->fixed_offset = start_offset;
    LMS->samples_processed = 0;
    LMS->max_convergence_attempts = LMS_DFL_MAX_FAILURES;
    LMS->error = 0;
}


inline Q15 LMS_Looper(struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR){
    Q15 retval = LMS_FAIL_DFL;
    //Q15 *desired = LMS->d_n + LMS->fixed_offset;
    //Q15 *white_noise = LMS->x_n + LMS->fixed_offset;
    

    //if(LMS->d_n_offset){
    //    if(LMS->d_n_offset > 0){
    //        desired += LMS->d_n_offset;
    //    } else {
    //        white_noise -= LMS->d_n_offset;
    //    }
    //}

    //run_2n_FIR_cycle(struct Q15_FIR_PARAMS *a, Q15 new_data)
    uint16_t n;
    
   // uint16_t max_iters = (LMS->iteration_ct >> LMS->ddsmpl_shift);

    uint16_t max_iters = LMS->iteration_ct - 
                            LMS->fixed_offset - 
//                            LMS->tap_len - 
                            LMS->d_n_offset;

    uint16_t stride = LMS->ddsmpl_stride;

    Q15 err_accum = 0;
    uint16_t err_timeout = LMS->tap_len >> 2;

    // Prefill
//    for(n = 0; n < LMS->tap_len; ++n){
//        WGN_FIR->data[n] = LMS->x_n[n];
//    }

    for(n = LMS->fixed_offset; n < max_iters; n += stride){
    //for(n = 0; n < max_iters; ++n){

        //retval = *(desired + LMS->ddsmpl_stride) - run_2n_FIR_cycle(WGN_FIR, *(white_noise + LMS->ddsmpl_stride));
        //retval = desired[n] - run_2n_FIR_cycle(WGN_FIR, white_noise[n]);
        //retval = LMS->d_n[n + LMS->fixed_offset] - run_2n_FIR_cycle(WGN_FIR, LMS->x_n[n + LMS->fixed_offset]);

        //Q15 Y_HAT = run_2n_FIR_cycle(WGN_FIR, LMS->x_n[n + LMS->d_n_offset]);
        //LMS->error = LMS->d_n[n] - Y_HAT;
        //LMS_Update_Taps(LMS, WGN_FIR, LMS->error);
        
        // Manual FIR Cycle
        WGN_FIR->data[0] = LMS->x_n[n];
        Q15 error = 0;
        for(uint16_t m = 0; m < LMS->tap_len; ++m){
            error += mul_Q15(WGN_FIR->taps[m], WGN_FIR->data[m]);
        }
        error = LMS->d_n[n + LMS->d_n_offset] - error;
        // Update Taps
        for(uint16_t m = 0; m < LMS->tap_len; ++m){
            Q15 dataval = WGN_FIR->data[m];
            Q15 tap_error = mul_Q15(dataval, error);
            Q15 std_lms_update = mul_Q15(LMS->learning_rate, tap_error);
            WGN_FIR->taps[m] += std_lms_update;
        }
        for(uint16_t m = LMS->tap_len - 1; m > 0; --m){
            WGN_FIR->data[m] = WGN_FIR->data[m - 1];
        }
        
        LMS->samples_processed++;      

        // Error processing Stuff goes here!!!!!!!!!!!
        //  break if avg < min err
        // Cheap abs
        if(LMS->error < 0) retval = mul_Q15(LMS->error, -1);
        else retval = LMS->error;

        err_accum += retval;
        err_accum >>= 1;

        if(err_accum <= LMS->target_error && n > err_timeout){
            retval = LMS_OK;
            break;
        } 
    }

    WGN_FIR->curr_zero = 0;

    if(n >= max_iters){
        if(retval > LMS->max_error_allowed) retval = LMS_FAIL_DFL;
    }

    // Error should be acceptable by now, or we've failed and everything sucks..
    return retval;
}

inline void LMS_Update_Taps(const struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR, Q15 error){
    for(uint16_t n = 0; n < LMS->tap_len; ++n){
        //WGN_FIR->taps[n] += (mul_Q15(LMS->learning_rate, mul_Q15(error, WGN_FIR->data[n])));
        WGN_FIR->taps[n] += mul_Q15(LMS->learning_rate, (mul_Q15(error, recall_sample_from_2n_FIR(WGN_FIR, n - 1))));
    }
}