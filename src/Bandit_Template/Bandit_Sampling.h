#ifndef BANDIT_SAMPLING_h
#define BANDIT_SAMPLING_h

#include "hardware/pwm.h"

#define ADC_FS_CALIB_OFFSET     0

/*
    For a million reasons polling for the sampling
        period actually ends up being advantageous.

    Bandit has optimized AMBA bus accesses, so believe
        me that I've considered all the options here.
*/

#define SAMPLING_SLICE  3

static void Sampling_Setup(uint32_t fs_khz){
    pwm_hw->slice[SAMPLING_SLICE].div = (1u) << PWM_CH3_DIV_INT_LSB;
    //pwm_hw->slice[SAMPLING_SLICE].top = ((SYS_CLK_KHZ / fs_khz)) + ADC_FS_CALIB_OFFSET;
    pwm_hw->slice[SAMPLING_SLICE].top = 250;
    pwm_hw->slice[SAMPLING_SLICE].ctr = 0;
}

static inline bool Sample_Now(){
    // 4.5.2.7 "A slice will assert an ISR flag whenever the counter wraps"
    //return (pwm_hw->intr & (1u << SAMPLING_SLICE));
    return (pwm_hw->intr & PWM_INTR_CH3_BITS);
}

static inline void Sample_Flag_Clear(){
    hw_set_bits(&pwm_hw->intr, PWM_INTR_CH3_BITS);
    //pwm_hw->intr |= PWM_INTR_CH3_BITS;
}

static inline void Start_Sampling(){
    //hw_set_bits(&pwm_hw->slice[SAMPLING_SLICE].csr, PWM_CH3_CSR_EN_BITS);
    pwm_hw->slice[SAMPLING_SLICE].csr |= (PWM_CH0_CSR_EN_BITS);
}

static inline void Stop_Sampling(){
    //hw_clear_bits(&pwm_hw->slice[SAMPLING_SLICE].csr, PWM_CH3_CSR_EN_BITS);
    pwm_hw->slice[SAMPLING_SLICE].csr &= ~(PWM_CH0_CSR_EN_BITS);
    pwm_hw->slice[SAMPLING_SLICE].ctr = 0;
}


#endif