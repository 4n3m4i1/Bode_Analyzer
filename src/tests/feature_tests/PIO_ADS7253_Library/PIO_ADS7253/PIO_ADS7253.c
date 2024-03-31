#include "PIO_ADS7253.h"





void __time_critical_func(ADS7253_write16_blocking)(const PIO pio, const uint16_t *src, size_t len) {
    size_t tx_remain = len;

    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
    while (tx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, ADS_PIO_MAIN_SM)) {
            *txfifo = *src++;
            --tx_remain;
        }
    }
}

void __time_critical_func(ADS7253_write16_blocking_clr_RX)(const PIO pio, const uint16_t *src, size_t len) {
    size_t tx_remain = len;

    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
    while (tx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, ADS_PIO_MAIN_SM)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if(!pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)){
            (pio->rxf[ADS_PIO_SDOA_SM]);
            (pio->rxf[ADS_PIO_SDOB_SM]);
        }
    }
}

void __time_critical_func(ADS7253_read16_blocking)(const PIO pio, uint16_t *dst, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
    io_rw_16 *rxfifo = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOA_SM];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, ADS_PIO_MAIN_SM)) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(ADS7253_write16_read16_blocking)(const PIO pio, uint16_t *src, uint16_t *dst,
                                                         size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
    io_rw_16 *rxfifo = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOA_SM];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, ADS_PIO_MAIN_SM)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(ADS7253_write16_read16_blocking_dual)(const PIO pio, uint16_t *src, uint16_t *dstA, uint16_t *dstB,
                                                         size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
    io_rw_16 *rxfifo_A = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOA_SM];
    io_rw_16 *rxfifo_B = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOB_SM];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, ADS_PIO_MAIN_SM)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)) {
            *dstA++ = *rxfifo_A;
            *dstB++ = *rxfifo_B;
            --rx_remain;
        }
    }
}

void __time_critical_func(ADS7253_Dual_Sampling)(const PIO pio, uint16_t *src, uint16_t *dstA, uint16_t *dstB,
                                                         size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_16 *txfifo = (io_rw_16 *) &pio->txf[ADS_PIO_MAIN_SM];
    io_rw_16 *rxfifo_A = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOA_SM];
    io_rw_16 *rxfifo_B = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOB_SM];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, ADS_PIO_MAIN_SM)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)) {
            *dstA++ = (*rxfifo_A) >> 2;
            *dstB++ = (*rxfifo_B) >> 2;
            --rx_remain;
        }
    }
}

void __time_critical_func(ADS7253_Read_Dual_Data)(const PIO pio, uint16_t *dst_A, uint16_t *dst_B){
    *dst_A = *((io_rw_16 *)&pio->rxf[ADS_PIO_SDOA_SM]) >> 2;
    *dst_B = *((io_rw_16 *)&pio->rxf[ADS_PIO_SDOB_SM]) >> 2;
}

uint32_t __time_critical_func(ADS7253_read_RX_buffer)(const PIO pio){
    io_rw_16 *data_A = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOA_SM];
    io_rw_16 *data_B = (io_rw_16 *) &pio->rxf[ADS_PIO_SDOB_SM];
    if(!pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM)) return  ((*data_B << 16) | (*data_A));
    else return 0;
}

uint16_t ADS7253_TI_Approved_Configuration(const PIO pio, uint16_t init_args) {
    uint16_t init_command[3] = {0,0,0};
    uint16_t init_returns[3] = {0xFFFF, 0xFFFF, 0xFFFF};

    // Start with a RAW write just to initialize the system,
    //  this will return some bad data in the 2nd word on A and B RX FIFOS
    ADS7253_write16_blocking_clr_RX(pio, init_command, count_of(init_command));

    busy_wait_us(5);

    while(!pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM) || 
            !pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOB_SM)){

        (pio->rxf[ADS_PIO_SDOA_SM]);
        (pio->rxf[ADS_PIO_SDOB_SM]);
    }

    busy_wait_us(4);

    init_command[0] = ADS7253_CMD(ADS7253_CFR_WRITE, init_args);

    // Write to CFR
    ADS7253_write16_blocking_clr_RX(pio, init_command, count_of(init_command));

    busy_wait_us(2);

    init_command[0] = 0;

    // Ensure CFR write is properly latched
    ADS7253_write16_blocking_clr_RX(pio, init_command, 2);
    
    // Really make sure those RX fifos are clear
    while(!pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOA_SM) || 
            !pio_sm_is_rx_fifo_empty(pio, ADS_PIO_SDOB_SM)){

        (pio->rxf[ADS_PIO_SDOA_SM]);
        (pio->rxf[ADS_PIO_SDOB_SM]);
    }

    init_command[0] = ADS7253_CMD(ADS7253_CFR_READ, 0);
    
    ADS7253_write16_read16_blocking(pio, init_command, init_returns, count_of(init_command));
    // As shown in Figure 90 A RX[0] should now contain the CFR

    return init_returns[0];
}