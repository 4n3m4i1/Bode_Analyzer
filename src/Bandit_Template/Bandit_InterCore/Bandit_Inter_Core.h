#ifndef BANDIT_INTERCORE_h
#define BANDIT_INTERCORE_h

#include "hardware/sync.h"

#define INTERCORE_SETTINGS_CHANGED_LOCK 24
#define INTERCORE_SETTINGS_LOCK         25
#define INTERCORE_FFTMEM_LOCK_A         26
#define INTERCORE_CORE_1_DBG_LOCK       27


void Setup_Semaphores();
bool Acquire_Lock_Blocking(uint32_t lock_num);
void Release_Lock(uint32_t lock_num);

#endif