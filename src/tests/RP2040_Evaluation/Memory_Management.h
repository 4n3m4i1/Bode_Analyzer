#ifndef MEMORY_MANAGEMENT_h
#define MEMORY_MANAGEMENT_h

/*
    We have 4 main banks of 64kB and 2 core specific banks (X, Y) of 4kB

    Each bank can support a single accessee, so for maximal throughput
    on fairly memory heavy tasks we must ensure no collisions between
    DMA, CORE0, or CORE1 occur.

    The core tasks are important to keep in mind when defining memory allocation roles:
        - Core 0:
            - PC USB Interface and State Control
            - FFT and final processing tasks
            - Application of post process self correction
            - PGA Management, setup, sample quality analysis

        - Core 1:
            - Sampling handler
            - Digital Downsampling
            - LMS Algorithm

    The banks are allocated as follows:
        - Bank 0:   SRAM0_BASE 0x21000000 General
        - Bank 1:   SRAM1_BASE 0x21010000 General
        - Bank 2:   SRAM2_BASE 0x21020000 Core 0 Data
        - Bank 3:   SRAM3_BASE 0x21030000 Core 1 Data
        - Bank 4:   SRAM4_BASE 0x20040000 Core 0 (X scratch)
        - Bank 5:   SRAM5_BASE 0x20041000 Core 1 (Y scratch)
        - RAM_END:  0x20042000
*/

#ifndef SRAM0_BASE
#define SRAM0_BASE  0x21000000u
#endif

#ifndef SRAM1_BASE
#define SRAM1_BASE  0x21010000u
#endif

#ifndef SRAM2_BASE
#define SRAM2_BASE  0x21020000u
#endif

#ifndef SRAM3_BASE
#define SRAM3_BASE  0x21030000u
#endif

#ifndef SRAM4_BASE
#define SRAM4_BASE  0x20040000u
#endif

#ifndef SRAM5_BASE
#define SRAM5_BASE  0x20041000u
#endif

#define CORE_0_MEM  __attribute__((section(".corezerodata")))
#define CORE_1_MEM  __attribute__((section(".coreonedata")))

#endif