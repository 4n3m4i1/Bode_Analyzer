#ifndef CORE_1_EXEC_h
#define CORE_1_EXEC_h

#include "generic_include.h"
#include "Memory_Management.h"


void    core_1_main();
void    setup_ADC();
void    setup_sampling_ISR();
void    sampling_ISR_func();
int     run_adaptive_taps();
void    handle_intercore_h_hat_transfer();  // DMA tap data to other core, or error on overrun, start fft core 0

    

#endif