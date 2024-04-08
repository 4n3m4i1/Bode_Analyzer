#include "Bandit_Inter_Core.h"

void Setup_Semaphores(){
    spin_locks_reset();
}


// Returns true if acquired
bool Acquire_Lock_Blocking(uint32_t lock_num){
    return !((*((uint32_t *)(SIO_BASE + SIO_SPINLOCK0_OFFSET + (lock_num * 4)))) ? true : false);
}

void Release_Lock(uint32_t lock_num){
    *((uint32_t *)(SIO_BASE + SIO_SPINLOCK0_OFFSET + (lock_num * 4))) = 0xDEADBEEF;
}
