#ifndef BANDIT_INTERCORE_h
#define BANDIT_INTERCORE_h

#include "hardware/sync.h"

#define INTERCORE_SETTINGS_CHANGED_LOCK 24
#define INTERCORE_SETTINGS_LOCK         25
#define INTERCORE_FFTMEM_LOCK_A         26
#define INTERCORE_CORE_1_DBG_LOCK       27

/*
    For use with intercore messaging,
        triggers for Debug semaphore access requests
*/

enum BANDIT_INTERCORE_MESSAGES {
    BICM_NOP0,
    BICM_NOP1,
    BICM_NOP2,
    BICM_NOP3,
    BICM_NOP4,
    BICM_NOP5,
    BICM_NOP6,
    BICM_NOP7,
    BICM_RELEASE_DEBUG_REPORT_LOCK
};

void Setup_Semaphores();
bool Acquire_Lock_Blocking(uint32_t lock_num);
void Release_Lock(uint32_t lock_num);

void Send_FIFO_Message(uint32_t data_2_send);
uint32_t Read_FIFO_Message();

#endif