/*!
    \file    gd32f4xx_it.c
    \brief   interrupt service routines

    \version 2026-02-05, V3.3.3, firmware for GD32F4xx
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx_it.h"
#include "gd32f450i_eval.h"
#include "buff_config.h"
#include <FreeRTOS.h>
#include "queue.h"
#include "semphr.h"

extern FlagStatus g_transfer_complete;
extern SemaphoreHandle_t GPSRX_Semaph;
extern SemaphoreHandle_t RX_Semaph;

/* =====================================================================
 * 崩溃自报 + 自愈
 * 以前任何异常/断言失败都是"关中断 + 死循环"：板子从此一个字节都不发，
 * 现场也没有任何线索，只能断电重来。现在改成"打印原因 + 软复位"，
 * 下一次上电看 [rst] 的 SW 位和心跳就能知道板子在崩溃-重启。
 * 这里刻意不用 printf、不取控制台锁、不依赖调度器：直接轮询 USART0，
 * 每个等待都有上限，保证一定能走到复位那一步。
 * ===================================================================== */
static void crash_putc(char c)
{
    uint32_t guard = 0U;

    while((RESET == usart_flag_get(USART0, USART_FLAG_TBE)) && (guard < 400000U)){
        guard++;
    }
    if(guard < 400000U){
        usart_data_transmit(USART0, (uint16_t)(uint8_t)c);
    }
}

static void crash_puts(const char *s)
{
    while('\0' != *s){
        crash_putc(*s++);
    }
}

static void crash_puthex(uint32_t v)
{
    int8_t i;

    for(i = 28; i >= 0; i -= 4){
        crash_putc("0123456789ABCDEF"[(v >> (uint8_t)i) & 0x0FU]);
    }
}

static void crash_putdec(uint32_t v)
{
    char     buf[12];
    uint8_t  n = 0U;

    if(0U == v){
        crash_putc('0');
        return;
    }
    while((0U != v) && (n < (uint8_t)sizeof(buf))){
        buf[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while(0U != n){
        crash_putc(buf[--n]);
    }
}

/* 打印完就复位：不停机、能自愈，原因留在串口上 */
static void crash_reset(void)
{
    volatile uint32_t i;

    crash_puts("\r\n[crash] rebooting\r\n");
    for(i = 0U; i < 800000U; i++){      /* 等最后几个字节移出去 */
    }
    NVIC_SystemReset();
    for(;;){                            /* 不该走到这里 */
    }
}

/* configASSERT 的实现：打印断言位置(文件:行)后复位 */
void vAssertFailed(const char *file, long line)
{
    crash_puts("\r\n*** ASSERT FAILED at ");
    crash_puts((NULL != file) ? file : "?");
    crash_putc(':');
    crash_putdec((uint32_t)line);
    crash_puts(" ***");
    crash_reset();
}

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    crash_puts("\r\n*** NMI ***");
    crash_reset();
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
volatile uint32_t g_fault_msp = 0;
volatile uint32_t g_fault_psp = 0;
volatile uint32_t g_fault_pc_msp = 0;
volatile uint32_t g_fault_pc_psp = 0;
volatile uint32_t g_fault_hfsr = 0;
volatile uint32_t g_fault_cfsr = 0;
volatile uint32_t g_fault_bfar = 0;
volatile uint32_t g_fault_mmfar = 0;

void HardFault_Handler(void)
{
    /* Record the fault context, then halt. Attach the debugger, halt, and read
     * g_fault_pc_psp (task context) or g_fault_pc_msp (handler context) plus
     * g_fault_cfsr in the Watch window - no more guessing which line died.
     *   CFSR bits: 0x0002 DACCVIOL, 0x0100 IBUSERR, 0x0200 PRECISERR,
     *              0x0400 IMPRECISERR, 0x1000 UNDEFINSTR, 0x2000 INVSTATE   */
    g_fault_msp  = __get_MSP();
    g_fault_psp  = __get_PSP();
    g_fault_pc_msp = ((uint32_t *)g_fault_msp)[6];   /* stacked PC on MSP */
    g_fault_pc_psp = ((uint32_t *)g_fault_psp)[6];   /* stacked PC on PSP */
    g_fault_hfsr = SCB->HFSR;
    g_fault_cfsr = SCB->CFSR;
    g_fault_bfar = SCB->BFAR;
    g_fault_mmfar = SCB->MMFAR;

    crash_puts("\r\n*** HardFault pc_msp=0x");
    crash_puthex(g_fault_pc_msp);
    crash_puts(" pc_psp=0x");
    crash_puthex(g_fault_pc_psp);
    crash_puts(" cfsr=0x");
    crash_puthex(g_fault_cfsr);
    crash_puts(" hfsr=0x");
    crash_puthex(g_fault_hfsr);
    crash_puts(" bfar=0x");
    crash_puthex(g_fault_bfar);
    crash_puts(" ***");
    crash_reset();
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    crash_puts("\r\n*** MemManage (cfsr=0x");
    crash_puthex(SCB->CFSR);
    crash_puts(" mmfar=0x");
    crash_puthex(SCB->MMFAR);
    crash_puts(") ***");
    crash_reset();
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    crash_puts("\r\n*** BusFault (cfsr=0x");
    crash_puthex(SCB->CFSR);
    crash_puts(" bfar=0x");
    crash_puthex(SCB->BFAR);
    crash_puts(") ***");
    crash_reset();
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    crash_puts("\r\n*** UsageFault (cfsr=0x");
    crash_puthex(SCB->CFSR);
    crash_puts(") ***");
    crash_reset();
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void SVC_Handler(void)
//{
//    /* if SVC exception occurs, go to infinite loop */
//    while(1) {
//    }
//}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
    /* if DebugMon exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void PendSV_Handler(void)
//{
//    /* if PendSV exception occurs, go to infinite loop */
//    while(1) {
//    }
//}

/*!
    \brief      this function handles DMA1_Channel7_IRQHandler interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DMA1_Channel7_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA1, DMA_CH7, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA1, DMA_CH7, DMA_INT_FLAG_FTF);
        g_transfer_complete = SET;
    }
}

/*!
    \brief      this function handles DMA0_Channel4_IRQHandler interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DMA1_Channel2_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA1, DMA_CH2, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA1, DMA_CH2, DMA_INT_FLAG_FTF);
        g_transfer_complete = SET;
    }
}
/*!
    \brief      this function handles USART interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART5_IRQHandler(void)
{
    uint16_t len = 0;

    if(RESET != usart_interrupt_flag_get(USART5, USART_INT_FLAG_IDLE)){
        /* clear IDLE: the check above already read STAT, so read DATA here */
        (void)usart_data_receive(USART5);

        /* bytes moved by the DMA in this frame */
        len = (uint16_t)(USART5_RX_BUF_SIZE - (uint16_t)dma_transfer_number_get(USART5_RX_DMA, USART5_RX_DMA_CH));
        usart5_rx_len = len;

        if(NULL != GPSRX_Semaph){
            BaseType_t woken = pdFALSE;
            xSemaphoreGiveFromISR(GPSRX_Semaph, &woken);
            portYIELD_FROM_ISR(woken);
        }

        /* RX is fully DMA driven, so the channel is re-armed right here.
         * The old RBNE branch is gone on purpose: it stored the CPU-read byte
         * at buf[0] and then re-enabled the channel from that same address, so
         * the DMA overwrote it with the next byte and every frame lost its
         * first character (which also broke the NMEA '$' + checksum check). */
        dma_channel_disable(USART5_RX_DMA, USART5_RX_DMA_CH);
        dma_flag_clear(USART5_RX_DMA, USART5_RX_DMA_CH, DMA_FLAG_FTF);
        dma_memory_address_config(USART5_RX_DMA, USART5_RX_DMA_CH, DMA_MEMORY_0, (uint32_t)usart5_rx_buf);
        dma_transfer_number_config(USART5_RX_DMA, USART5_RX_DMA_CH, USART5_RX_BUF_SIZE);
        dma_channel_enable(USART5_RX_DMA, USART5_RX_DMA_CH);
    }
}



/*!
    \brief      this function handles USART0 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
    \note       IDLE = one frame finished. Bytes were moved by DMA1 CH5.
                IDLE is cleared by "read USART_STAT then read USART_DATA";
                usart_interrupt_flag_get() already read STAT, so read DATA here.
*/
void USART0_IRQHandler(void)
{
    uint16_t len = 0;

    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE)){
        /* clear IDLE: the check above already read STAT, so read DATA here */
        (void)usart_data_receive(USART0);

        /* bytes moved by the DMA in this frame */
        len = (uint16_t)(USART0_RX_BUF_SIZE - (uint16_t)dma_transfer_number_get(USART0_RX_DMA, USART0_RX_DMA_CH));
        usart0_rx_len = len;

        /* 先搬进软件环形缓冲，再重新装载 DMA：任务哪怕正在跑长命令，
          * 这一段也不会被下一段覆盖。 */
        usart0_rx_ring_push(usart0_rx_buf, len);

        if(NULL != RX_Semaph){
            BaseType_t woken = pdFALSE;
            xSemaphoreGiveFromISR(RX_Semaph, &woken);
            portYIELD_FROM_ISR(woken);
        }

        /* RX is fully DMA driven, so the channel is re-armed right here.
         * The old RBNE branch is gone on purpose: it stored the CPU-read byte
         * at buf[0] and then re-enabled the channel from that same address, so
         * the DMA overwrote it with the next byte and every frame lost its
         * first character (which also broke the NMEA '$' + checksum check). */
        dma_channel_disable(USART0_RX_DMA, USART0_RX_DMA_CH);
        dma_flag_clear(USART0_RX_DMA, USART0_RX_DMA_CH, DMA_FLAG_FTF);
        dma_memory_address_config(USART0_RX_DMA, USART0_RX_DMA_CH, DMA_MEMORY_0, (uint32_t)usart0_rx_buf);
        dma_transfer_number_config(USART0_RX_DMA, USART0_RX_DMA_CH, USART0_RX_BUF_SIZE);
        dma_channel_enable(USART0_RX_DMA, USART0_RX_DMA_CH);
    }
}
