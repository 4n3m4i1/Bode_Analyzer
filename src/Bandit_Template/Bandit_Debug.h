#ifndef BANDIT_DEBUG_h
#define BANDIT_DEBUG_h

/*
    Contains ability to debug each state of BANDIT
*/

#include "pico/stdlib.h"
#include "inttypes.h"



// 32 bit us = 2^32 / 10^6 ~= 4295 seconds ~= 72 minutes

struct BANDIT_CORE_1_DEBUG_REPORT {
    uint32_t    epoch_core_1_boot_us;           // epoch us elapsed until Core 1 boots
    uint16_t    adc_init_cfr_readback;          // CFR readback value, should be 2624
    uint16_t    adc_init_attempts;              // Number of attempts at successful configuration of ADC
    uint32_t    epoch_ads_init_complete_us;     // epoch us elapsed till ADC configuration complete
    uint8_t     dac_cal_state;                  // See enum 1
    uint16_t    dac_cal_value;                  // Value written to ADC (without << 3 shift)
    uint16_t    dac_cal_attempts;               // Number of times the DAC calibration routine attempted convergence
    uint32_t    epoch_dac_cal_complete_us;      // epoch us elapsed till DAC configuration complete
};

// Enum 1
// Steady state ADC Reference DAC Calibration Routine Status
enum BANDIT_DBG_DAC {
    BDBG_DAC_UNCAL,         // DAC Uncalibrated
    BDBG_DAC_EXACT_CAL,     // DAC Exact midcode calibration
    BDBG_DAC_RANGE_CAL      // DAC +/-1 Range of Midcode
};


#endif