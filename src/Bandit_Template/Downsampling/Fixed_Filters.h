#ifndef FIXED_FILTERS_h
#define FIXED_FILTERS_h

#include "generic_include.h"
#include "../FIR/FIR_Fast_Fixed.h"
#include "../Memory_Management.h"
//Fixed FIR filters in Q15 for different downsampling values (M)

//// M = 2
#define DOWNSAMPLE_LEN  32

enum DOWNSAMPLING_AMOUNTS {
    DOWNSAMPLE_1X_250K_CUT,
    DOWNSAMPLE_2X_125K_CUT,
    DOWNSAMPLE_4X_62K5_CUT,
    DOWNSAMPLE_8X_32K2_CUT
};

CORE_1_MEM Q15 h125[DOWNSAMPLE_LEN] = {
    553, -564, 575, -585, 594, -603, 
    611, -618, 624, -629, 634, -638, 
    641, -643, 644, 31576, 644, -643, 
    641, -638, 634, -629, 624, -618, 
    611, -603, 594, -585, 575, -564, 
    553, 0
};

// M = 4
CORE_1_MEM Q15 h62[DOWNSAMPLE_LEN] = {
    -21, -60, -84, -52, 78, 273, 
    387, 221, -301, -974, -1305, -731, 
    1017, 3642, 6306, 7987, 7987, 6306, 
    3642, 1017, -731, -1305, -974, -301, 
    221, 387, 273, 78, -52, -84, 
    -60, -21
};

// M = 8
CORE_1_MEM Q15 h31[DOWNSAMPLE_LEN] = {
    -10, -36, -75, -132, -198, -244, 
    -231, -112, 152, 582, 1167, 1861, 
    2589, 3257, 3768, 4046, 4046, 3768, 
    3257, 2589, 1861, 1167, 582, 152, 
    -112, -231, -244, -198, -132, -75, 
    -36, -10
};

#endif