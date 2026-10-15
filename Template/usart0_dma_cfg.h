#ifndef USART0_DMA_CFG_H
#define USART0_DMA_CFG_H

#include "gd32f4xx.h"

/* ---------------------------------------------------------------------------
 * USART0 RX DMA configuration.
 *
 * GD32F4xx User Manual Rev3.4, Table 10-3 "Peripheral requests to DMA1":
 *     PERIEN = 0b100 : USART0_RX -> DMA1 CH2 or CH5, USART0_TX -> DMA1 CH7
 *     PERIEN = 0b101 : USART5_RX -> DMA1 CH1 or CH2, USART5_TX -> DMA1 CH6 or CH7
 *
 * The GPS module is on USART5_RX and already occupies DMA1 CH2 / PERIEN=0b101,
 * so USART0_RX uses its alternative channel DMA1 CH5 / PERIEN=0b100. Both RX DMA
 * channels can then stay active at the same time.
 *
 * The manual also states: "It is forbidden to simultaneously enable these two
 * DMA channels with selecting the same peripheral request", i.e. never enable
 * CH2+PERIEN0b100 and CH5+PERIEN0b100 at the same time.
 * ------------------------------------------------------------------------- */

#define USART0_RX_DMA                (DMA1)
#define USART0_RX_DMA_CH             (DMA_CH5)
#define USART0_RX_DMA_SUBPERI        (DMA_SUBPERI4)

/* must be <= 65535 (DMA transfer counter is 16 bit) */
#define USART0_RX_BUF_SIZE           (1024U)

/* 1 = echo every received frame back to the host, useful to bring the link up */
#define USART0_RX_ECHO               (0)

/* DMA destination, filled by hardware */
extern uint8_t usart0_rx_buf[USART0_RX_BUF_SIZE];
/* copy handed to the task, so the DMA may already restart writing */
extern uint8_t usart0_frame_buf[USART0_RX_BUF_SIZE];
/* frame length snapshotted in the IDLE interrupt */
extern volatile uint16_t usart0_rx_len;

void usart0_dma_config(void);

#endif /* USART0_DMA_CFG_H */
