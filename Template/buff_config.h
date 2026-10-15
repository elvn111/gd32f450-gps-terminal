#ifndef __BUFF_CONFIG_H__
#define __BUFF_CONFIG_H__

#include "gd32f4xx.h"

/* USART0/GPS 的 DMA 缓冲配置：USART0 接收用 DMA1 CH5(空闲中断判帧尾)，
  * USART5(GPS)接收用 DMA1 CH2，USART0 发送用 DMA1 CH7(下行链路)。
  * 同一外设请求的两个通道不能同时使能，具体通道与子外设号见 UM 表 10-3。
  * 接收一律“硬件循环搬运 + 空闲中断判帧尾”，任务侧拿到的是一份拷贝。 */

#define USART0_RX_DMA                (DMA1)
#define USART0_RX_DMA_CH             (DMA_CH5)
#define USART0_RX_DMA_SUBPERI        (DMA_SUBPERI4)

#define USART5_RX_DMA                (DMA1)
#define USART5_RX_DMA_CH             (DMA_CH2)
#define USART5_RX_DMA_SUBPERI        (DMA_SUBPERI5)

/* 必须 <= 65535(DMA 传输计数器是 16 位) */
#define USART0_RX_BUF_SIZE           (1024U)
#define USART5_RX_BUF_SIZE           (1024U)

/* 1=把收到的每一帧回显给主机，用来确认链路是通的 */
#define USART0_RX_ECHO               (0)

/* DMA 的目的地址，由硬件写入 */
extern uint8_t usart0_rx_buf[USART0_RX_BUF_SIZE];
extern uint8_t usart5_rx_buf[USART5_RX_BUF_SIZE];
/* 交给任务的拷贝：此时 DMA 可能已经开始重新写入同一缓冲 */
extern uint8_t usart0_frame_buf[USART0_RX_BUF_SIZE];
extern uint8_t usart5_frame_buf[USART5_RX_BUF_SIZE];
/* 在 IDLE 中断里快照下来的帧长 */
extern volatile uint16_t usart0_rx_len;
extern volatile uint16_t usart5_rx_len;

#endif /* __BUFF_CONFIG_H__ */
