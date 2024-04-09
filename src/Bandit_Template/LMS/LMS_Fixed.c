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
    LMS->ddsmpl_stride = 1;
    LMS->max_error_allowed = max_acceptable_error;
    LMS->iteration_ct = max_runtime;
    LMS->fixed_offset = start_offset;
    LMS->samples_processed = 0;
    LMS->max_convergence_attempts = LMS_DFL_MAX_FAILURES;
}


Q15 LMS_Looper(struct LMS_Fixed_Inst *LMS, struct Q15_FIR_PARAMS *WGN_FIR, bool flush_FIR){
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

    // If new run, flush the FIR buffer
    if(flush_FIR) flush_FIR_buffer_and_taps(WGN_FIR);

    //run_2n_FIR_cycle(struct Q15_FIR_PARAMS *a, Q15 new_data)
    uint16_t n;
    
    uint16_t max_iters = (LMS->iteration_ct >> LMS->ddsmpl_shift);

    uint16_t stride = LMS->ddsmpl_stride;

    for(n = LMS->fixed_offset; n < max_iters; n += stride){

        //retval = *(desired + LMS->ddsmpl_stride) - run_2n_FIR_cycle(WGN_FIR, *(white_noise + LMS->ddsmpl_stride));
        //retval = desired[n] - run_2n_FIR_cycle(WGN_FIR, white_noise[n]);
        //retval = LMS->d_n[n + LMS->fixed_offset] - run_2n_FIR_cycle(WGN_FIR, LMS->x_n[n + LMS->fixed_offset]);

        retval = LMS->d_n[n] - run_2n_FIR_cycle(WGN_FIR, LMS->x_n[n]);
        
        //retval = LMS->d_n[n * stride] - run_2n_FIR_cycle(WGN_FIR, LMS->x_n[n * stride]);
        //retval = LMS->d_n[n * stride];
        //retval = run_2n_FIR_cycle(WGN_FIR, LMS->x_n[n * stride]);
        //retval = 0x8021;

        retval = 0;
        //add_sample_to_2n_FIR_I16_no_inc(WGN_FIR, LMS->x_n[n]);
        //WGN_FIR->data[(n & WGN_FIR->size_mask)] = LMS->x_n[n];
        
        //uint16_t write_address = WGN_FIR->curr_zero++ & 0x001F;
        //WGN_FIR->data[WGN_FIR->curr_zero++ & WGN_FIR->size_mask] = 2;
        //WGN_FIR->data[0] = LMS->x_n[n];
        

        //for(uint_fast16_t m = 0; m < WGN_FIR->size; ++m){
        //    retval += mul_Q15(recall_sample_from_2n_FIR(WGN_FIR, m), WGN_FIR->taps[m]);
        //}


        LMS_Update_Taps(LMS, WGN_FIR, retval);

        LMS->samples_processed++;      

        // Error processing Stuff goes here!!!!!!!!!!!
        //  break if avg < min err
        // Cheap abs
        if(retval < 0) retval = mul_Q15(retval, -1); 
        if(retval <= LMS->target_error){
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
        Q15 tmp = mul_Q15(error, WGN_FIR->data[n]);
        //WGN_FIR->taps[n] += (mul_Q15(LMS->learning_rate, mul_Q15(error, WGN_FIR->data[n])));
        WGN_FIR->taps[n] += mul_Q15(LMS->learning_rate, tmp);
    }
}