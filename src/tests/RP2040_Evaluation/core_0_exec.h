#ifndef CORE_0_EXEC_h
#define CORE_0_EXEC_h

#include "generic_include.h"
#include "Memory_Management.h"
#include "FFT/fft_half.h"

    void core_0_main();
    void setup_tiny_usb();
    void execute_fft(FFT_PARAMS *fft);
    void post_process_fft_data(FFT_PARAMS *fft);
    void transmit_final_results(FFT_PARAMS *fft);

#endif