#ifndef BANDIT_DEBUG_h
#define BANDIT_DEBUG_h

/*
    Contains ability to debug each state of BANDIT
*/

#include "pico/stdlib.h"
#include "inttypes.h"





struct BANDIT_CORE_1_DEBUG_REPORT {
    uint16_t    adc_init_cfr_readback;  // CFR readback value, should be 2624
    uint16_t    adc_init_attempts;      // Number of attempts at successful configuration of ADC
    uint8_t     dac_cal_state;          // See enum 1
    uint16_t    dac_cal_value;          // Value written to ADC (without << 3 shift)
    uint16_t    dac_cal_attempts;       // Number of times the DAC calibration routine attempted convergence
};

// Enum 1
// Steady state ADC Reference DAC Calibration Routine Status
enum BANDIT_DBG_DAC {
    BDBG_DAC_UNCAL,         // DAC Uncalibrated
    BDBG_DAC_EXACT_CAL,     // DAC Exact midcode calibration
    BDBG_DAC_RANGE_CAL      // DAC +/-1 Range of Midcode
};


#endif