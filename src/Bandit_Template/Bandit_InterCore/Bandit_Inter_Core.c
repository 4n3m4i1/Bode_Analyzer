#include "Bandit_Inter_Core.h"

void Setup_Semaphores(){
    spin_locks_reset();
}


// Returns true if acquired
bool Acquire_Lock_Blocking(uint32_t lock_num){
    while(!(*((io_rw_32 *)(SIO_BASE + SIO_SPINLOCK0_OFFSET + (lock_num * 4)))));
    //return !((*((uint32_t *)(SIO_BASE + SIO_SPINLOCK0_OFFSET + (lock_num * 4)))) ? true : false);
    return true;
}

void Release_Lock(uint32_t lock_num){
    *((io_rw_32 *)(SIO_BASE + SIO_SPINLOCK0_OFFSET + (lock_num * 4))) = 0xDEADBEEF;
}


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

inline void Send_FIFO_Message(uint32_t data_2_send){
    *((io_wo_32 *)(SIO_BASE + SIO_FIFO_WR_OFFSET)) = data_2_send;
}

inline uint32_t Read_FIFO_Message(){
    return ( (uint32_t)(*(io_rw_32 *)(SIO_BASE + SIO_FIFO_ST_OFFSET)) | SIO_FIFO_ST_VLD_BITS) ?
                (uint32_t)(*((io_wo_32 *)(SIO_BASE + SIO_FIFO_RD_OFFSET))) :
                (uint32_t)BICM_NOP0;
}