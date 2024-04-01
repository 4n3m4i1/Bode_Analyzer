#ifndef BANDIT_INTERCORE_h
#define BANDIT_INTERCORE_h

#include "hardware/sync.h"

#define INTERCORE_SETTINGS_LOCK     2
#define INTERCORE_FFTMEM_LOCK_A     3
#define INTERCORE_FFTMEM_LOCK_B     4


void Setup_Semaphores();

#endif