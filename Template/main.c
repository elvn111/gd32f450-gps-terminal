/* 应用主程序：GPS 采集 + 控制台命令(USART0) + 下行链路 + 配置掉电保存。
  * 原模板说明：transmit/receive data using DMA interrupt，V3.3.3。
  * 任务划分：Init_Task 建资源并启动其它任务，RX_Task 收控制台/GPS，
  * TX_Task 解析 GPS 并出报告，LED_Task 心跳，另有一个下行发送任务。 */

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

#include "gd32f4xx.h"
#include "gd32f450i_eval.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "info_bsp.h"
#include "buff_config.h"
#include "cli.h"
#include "nvm_config.h"
#include "gps_link.h"

#include <FreeRTOS.h>
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"

#define USART0_DATA_ADDRESS      ((uint32_t)&USART_DATA(USART0))
#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))

#define STRING_NUM 4
#define STRING_LEN 50

#define INIT_TASK_PRIO (tskIDLE_PRIORITY + 1)
#define TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 3U)   /* printf 需要真正的栈空间：默认的 130 字栈不够，这里按 3 倍分配 */

/*Command buff and GPS buff */
uint8_t usart5_rx_buf[USART5_RX_BUF_SIZE]={0};
uint8_t usart5_frame_buf[USART5_RX_BUF_SIZE]={0};
volatile uint16_t usart5_rx_len = 0;

/* USART0 接收 DMA 的缓冲配置见 buff_config.h */
uint8_t usart0_rx_buf[USART0_RX_BUF_SIZE] = {0};
uint8_t usart0_frame_buf[USART0_RX_BUF_SIZE] = {0};
volatile uint16_t usart0_rx_len = 0;

/* Binary semaphore handle definition. */
SemaphoreHandle_t GPSRX_Semaph;
SemaphoreHandle_t GPSDeal_Semaph;
SemaphoreHandle_t RX_Semaph;
SemaphoreHandle_t Deal_Semaph;
typedef enum{
    WORKED=1,
    RAW=0
}OUTPUT_STATE_T;


OUTPUT_STATE_T OUTPUT_STATE=WORKED;

__IO FlagStatus g_transfer_complete = RESET;
uint8_t rxbuffer[10];
uint8_t txbuffer[] = "\n\rUSART DMA interrupt receive and transmit example, please input 10 bytes:\n\r";

void nvic_config(void);
void dma_config(void);
void usart_config(void);

void Init_Task(void *pvParameters);
void LED_Task(void *pvParameters);
void RX_Task(void *pvParameters);
void TX_Task(void *pvParameters);
void GPSRX_Task(void *pvParameters);
void GPSTX_Task(void *pvParameters);


int fputc(int ch, FILE *f)
{
    /* 写数据寄存器前先等发送空：下行链路会直接用 DMA 把整帧送进 USART0，
      * 若上一个 DMA 字节还停在数据寄存器时就写控制台字符，那个字节会被覆盖丢掉。 */
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));

    return ch;
}
/* 开机预设命令。这里刻意不发 "freset"：那是模块恢复出厂，会清掉 saveconfig 存下的配置
  * 并强制冷启动重收敛，第一次定位要多等几十秒。真需要恢复出厂，用 gps reset 手动发。 */
const char GPS_CMD[STRING_NUM][STRING_LEN]={
	"unlogall\r\n",
	"set appscene survey\r\n",
	"log com1 gpnav ontime 1\r\n",
	"log com1 gpybm ontime 1\r\n",
};
//const char GPS_CMD[STRING_NUM][STRING_LEN]={
//	"LOG COM1 VERSION ONTIME 1\r\n",
//	"LOG COM1 COMCONFIG ONTIME 1\r\n",
//	"LOG COM1 LOGLIST ONTIME 1\r\n",
//	"LOG COM1 BESTPOSA ONTIME 1\r\n",
//	"LOG COM1 PSRDOPA ONTIME 1\r\n",
//	"LOG COM1 HEADINGA ONTIME 1\r\n",
//	"LOG COM1 SYSRTS ONCE1\r\n",
//	"log com1 gpnav ontime 1\r\n",
//	"log com1 gpybm ontime 1\r\n",
//};

void uart_send_string(uint32_t usart, const char *str)
{
    while ('\0' != *str) 
    {
        while (RESET == usart_flag_get(usart, USART_FLAG_TBE));
        usart_data_transmit(usart, (uint16_t)(uint8_t)*str++);
    }
    while (RESET == usart_flag_get(usart, USART_FLAG_TC));
}

/* 按长度发送：二进制安全，0x00 不会截断 */
void uart_send_buf(uint32_t usart, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++) {
        while (RESET == usart_flag_get(usart, USART_FLAG_TBE));
        usart_data_transmit(usart, (uint16_t)buf[i]);
    }
    while (RESET == usart_flag_get(usart, USART_FLAG_TC));
}


int gps_init(void)
{
	int i=0;
	for(i=0;i<STRING_NUM;i++)
    {
		uart_send_string(USART5,GPS_CMD[i]);
	}
}

/* GPS 模块硬件控制：PB4 是模块复位脚，低电平复位、高电平运行。
  * 上电在 main() 里第一时间拉高(内部上拉)，保证调度器起来之前模块就不处于复位态。 */
#define GPS_RST_PORT            GPIOB
#define GPS_RST_CLK             RCU_GPIOB
#define GPS_RST_PIN             GPIO_PIN_4

#define GPS_RST_ASSERT()        gpio_bit_reset(GPS_RST_PORT, GPS_RST_PIN)   /* 低=复位 */
#define GPS_RST_RELEASE()       gpio_bit_set(GPS_RST_PORT, GPS_RST_PIN)     /* 高=运行 */

/* 1=上电时复位模块一次(当前关闭，为了尽快出定位) */
#define GPS_RESET_ON_BOOT       0U
#define GPS_RESET_LOW_MS        100U    /* 复位低电平宽度 */
#define GPS_RESET_BOOT_MS       600U    /* 留给模块启动的时间 */

/* 把 PB4 配成推挽输出并拉高(释放复位) */
void gps_reset_config(void)
{
    rcu_periph_clock_enable(GPS_RST_CLK);
    gpio_mode_set(GPS_RST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPS_RST_PIN);
    gpio_output_options_set(GPS_RST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPS_RST_PIN);
    GPS_RST_RELEASE();
}

/* 拉低复位 ms_low 毫秒后释放；只能在任务里调用(用 vTaskDelay) */
void gps_reset_pulse(uint32_t ms_low)
{
    GPS_RST_ASSERT();
    vTaskDelay(pdMS_TO_TICKS(ms_low));
    GPS_RST_RELEASE();
}

/* =====================================================================
 * CLI 平台钩子: 解释器只认这一层接口, 换串口/换模块只改这里
 * ===================================================================== */
extern GPS_Module_Info GPS_Info;      /* 定义在下面, 这里先声明 */

/* 往 GPS 模块发一行命令(Unicore 命令以 CRLF 结束) */
static void plat_gps_send_line(const char *line)
{
    uart_send_string(USART5, line);
    uart_send_string(USART5, "\r\n");
}

/* 发送上电预设命令表 */
static void plat_gps_preset(void)
{
    (void)gps_init();
}

/* 把控制台收到的字节原样转给 GPS(桥模式用) */
static void plat_gps_send_raw(const uint8_t *buf, uint16_t len)
{
    uart_send_buf(USART5, buf, len);
}

/* 打印最近一次解析到的定位 */
static void plat_gps_info_print(void)
{
    const cli_config_t *cfg = cli_config_get();

    printf("[gps] lat=%.7f lon=%.7f h=%.3f\r\n",
           GPS_Info.latitude, GPS_Info.longitude, GPS_Info.ElpHeight);
    printf("[gps] heading=%.2f baseline=%.3f sat=%u (%s, sat_min=%u)\r\n",
           GPS_Info.Heading_Angle, GPS_Info.Baseline,
           (unsigned)GPS_Info.GPS_Satellite,
           ((uint8_t)GPS_Info.GPS_Satellite >= cfg->sat_min) ? "valid" : "invalid",
           (unsigned)cfg->sat_min);
}

/* 重配 GPS 串口波特率(只动 USART5 参数, 不改 DMA) */
static void plat_gps_baud_set(uint32_t baud)
{
    usart_disable(USART5);
    usart_baudrate_set(USART5, baud);
    usart_enable(USART5);
}

/* 输出模式: 0 = RAW 原始数据直出, 1 = WORKED 解析后输出 */
static void plat_output_mode_set(uint8_t mode)
{
    OUTPUT_STATE = mode ? WORKED : RAW;
}

/* USART0 接收 DMA 还剩多少没搬完(调试用) */
static uint32_t plat_usart0_rx_dma_left(void)
{
    return dma_transfer_number_get(USART0_RX_DMA, USART0_RX_DMA_CH);
}

static void plat_system_reset(void)
{
    NVIC_SystemReset();
}

static uint32_t plat_uptime_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* CLI 挂钩：控制台锁、诊断打印、Flash 配置存取 */

/* printf 不可重入，而多个任务都在打印，用一个互斥量串行化整个控制台。
  * 在 Init_Task 里、cli_init() 之前创建。 */
static SemaphoreHandle_t g_console_lock;

static void plat_out_lock(void)
{
    if(NULL != g_console_lock){
        (void)xSemaphoreTake(g_console_lock, portMAX_DELAY);
    }
}

static void plat_out_unlock(void)
{
    if(NULL != g_console_lock){
        (void)xSemaphoreGive(g_console_lock);
    }
}

/* 任务表(含栈余量)：需要 FreeRTOSConfig.h 里的 configUSE_TRACE_FACILITY
  * 与 configUSE_STATS_FORMATTING_FUNCTIONS 打开 */
static void plat_tasks_info(void)
{
    static char buf[600];

    vTaskList(buf);
    printf("name            state prio  stack num\r\n%s", buf);
    printf("[tasks] stack = words left at the high water mark; small = danger\r\n");
}

static void plat_dma_channel_info(const char *tag, uint32_t dma, dma_channel_enum ch)
{
    uint32_t ctl = DMA_CHCTL(dma, ch);

    printf("[dma] %-11s chn=%lu chen=%lu cmen=%lu perien=%lu left=%4lu ftf=%lu\r\n",
           tag,
           (unsigned long)ch,
           (unsigned long)((ctl & DMA_CHXCTL_CHEN) ? 1U : 0U),
           (unsigned long)((ctl & DMA_CHXCTL_CMEN) ? 1U : 0U),
           (unsigned long)((ctl & DMA_CHXCTL_PERIEN) >> 25),
           (unsigned long)dma_transfer_number_get(dma, ch),
           (unsigned long)((RESET != dma_flag_get(dma, ch, DMA_FLAG_FTF)) ? 1U : 0U));
}

static void plat_dma_info(void)
{
    plat_dma_channel_info("usart0 rx", USART0_RX_DMA, USART0_RX_DMA_CH);
    plat_dma_channel_info("usart5 rx", USART5_RX_DMA, USART5_RX_DMA_CH);
    printf("[dma] perien: 4 = USART0_RX, 5 = USART5_RX (manual table 10-3)\r\n");
}

static int plat_config_save(const cli_config_t *cfg)
{
    return nvm_config_save(cfg);
}

static int plat_config_load(cli_config_t *cfg)
{
    return nvm_config_load(cfg);
}

static void plat_nvm_info(void)
{
    nvm_config_info();
}

/* 用指定初始化器挂钩子：以后往 cli_platform_t 里加钩子不会再让参数错位 */

static void plat_link_send_now(void)
{
    (void)gps_link_send_now();
}

static void plat_link_info(void)
{
    gps_link_info();
}

static const cli_platform_t g_cli_plat = {
    .gps_send_line      = plat_gps_send_line,
    .gps_send_raw       = plat_gps_send_raw,
    .gps_preset         = plat_gps_preset,
    .gps_info_print     = plat_gps_info_print,
    .gps_baud_set       = plat_gps_baud_set,
    .output_mode_set    = plat_output_mode_set,
    .usart0_rx_dma_left = plat_usart0_rx_dma_left,
    .tasks_info         = plat_tasks_info,
    .dma_info           = plat_dma_info,
    .config_save        = plat_config_save,
    .config_load        = plat_config_load,
    .nvm_info           = plat_nvm_info,
    .send_now           = plat_link_send_now,
    .link_info          = plat_link_info,
    .out_lock           = plat_out_lock,
    .out_unlock         = plat_out_unlock,
    .system_reset       = plat_system_reset,
    .uptime_ms          = plat_uptime_ms,
};



/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t flagTimerCnt = 0;
int main(void)
{
	/* configure 4 bits pre-emption priority */
	nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
	/* GPS 复位脚 PB4 尽早拉高：空闲电平=模块运行 */
	gps_reset_config();

	/* init task */
	xTaskCreate(Init_Task, "INIT", TASK_STACK_WORDS, NULL, INIT_TASK_PRIO, NULL);

	/* start scheduler */
	vTaskStartScheduler();

	while (1)
	{
	}
}

/*!
	\brief      init task
	\param[in]  pvParameters not used
	\param[out] none
	\retval     none
*/
void Init_Task(void *pvParameters)
{
    /* 信号量必须在下面使能任何 USART 中断之前创建：
      *  * 开机瞬间 IDLE 中断可能先到，中断里拿到 NULL 会命中 configASSERT，
      *  * 现象就是上电直接卡死(关中断后死在死循环里)。 */
    GPSRX_Semaph = xSemaphoreCreateBinary();
    GPSDeal_Semaph = xSemaphoreCreateBinary();
    RX_Semaph = xSemaphoreCreateBinary();
    Deal_Semaph = xSemaphoreCreateBinary();
    xSemaphoreTake(GPSRX_Semaph,0);
    xSemaphoreTake(GPSDeal_Semaph,0);
    xSemaphoreTake(RX_Semaph,0);
    xSemaphoreTake(Deal_Semaph,0);

    		
    nvic_config();
    dma_config();
    usart_config();

    /* USART0 发送 DMA(DMA1 CH7)：给第二颗 MCU 的下行链路 */
    gps_link_init();

#if GPS_RESET_ON_BOOT
    /* 硬件复位模块(PB4 拉低)，等它启动完再下发命令 */
    gps_reset_pulse(GPS_RESET_LOW_MS);
    vTaskDelay(pdMS_TO_TICKS(GPS_RESET_BOOT_MS));
#endif
	gps_init();
    gd_eval_led_init (LED1);

    g_console_lock = xSemaphoreCreateMutex();
    cli_init(&g_cli_plat);
    xTaskCreate(LED_Task, "LED", TASK_STACK_WORDS, NULL, INIT_TASK_PRIO, NULL);
    xTaskCreate(RX_Task, "RECEIVE", TASK_STACK_WORDS, NULL, INIT_TASK_PRIO, NULL);
    xTaskCreate(TX_Task, "TRANSMIT", TASK_STACK_WORDS, NULL, INIT_TASK_PRIO, NULL);
    xTaskCreate(GPSRX_Task, "GPS_RECEIVE", TASK_STACK_WORDS, NULL, INIT_TASK_PRIO, NULL);
    xTaskCreate(GPSTX_Task, "GPS_TRANSMIT", TASK_STACK_WORDS, NULL, INIT_TASK_PRIO, NULL);
    //xTaskCreate(_Task, "", TASK_STACK_WORDS, NULL, INIT_TASK_PRIO, NULL);
    while(1)
    {
        vTaskDelete(NULL);
    }
}


void LED_Task(void *pvParameters)
{
    while(1)
    {
        gd_eval_led_on(LED1);
        vTaskDelay(500);      
        gd_eval_led_off(LED1);
        vTaskDelay(500);        
}

GPS_Module_Info GPS_Info;
void RX_Task(void *pvParameters)
{
    uint16_t len;
    while(1)
    {
        if(xSemaphoreTake(RX_Semaph, portMAX_DELAY) == pdTRUE)
        {
            len = usart0_rx_len;
            if(len >= USART0_RX_BUF_SIZE)
            {
                len = USART0_RX_BUF_SIZE - 1U;
            }
            memcpy(usart0_frame_buf, usart0_rx_buf, len);
            usart0_frame_buf[len] = '\0';
            /* 命令解释器入口: 进来的可能是一行、半行或多行 */
            cli_input((const uint8_t *)usart0_frame_buf, len);
        }
    }
}


/* 打印 [GPS_DATA] 报告：只输出规范 6.2 规定的 6 行，别的都不输出。
  * 整组在控制台锁内一次发出，保证不会被别的任务从中间截断。
  * 解析层仍然会算 source/valid 等状态，它们供下行链路帧的 flag 位使用，不打到串口。 */
static void gps_report_print(const gps_fix_t *f)
{
    cli_out_lock();
    printf("[GPS_DATA]:latitude:%f\r\n", f->info.latitude);
    printf("[GPS_DATA]:longitude:%f\r\n", f->info.longitude);
    printf("[GPS_DATA]:ElpHeight:%f\r\n", f->info.ElpHeight);
    printf("[GPS_DATA]:Heading_Angle:%f\r\n", f->info.Heading_Angle);
    printf("[GPS_DATA]:Baseline:%f\r\n", f->info.Baseline);
    printf("[GPS_DATA]:GPS_Satellite:%d\r\n", (int)f->info.GPS_Satellite);
    cli_out_unlock();
}

void TX_Task(void *pvParameters)
{
    while(1)
    {
        if(xSemaphoreTake(GPSDeal_Semaph,0)==pdTRUE)
        {
            gps_fix_t   fix;
            const char *p = (const char *)usart5_frame_buf;

            /* 一帧里可能同时含 GPYBM 与 GPNAV，逐句解析。
              * 解析与下发始终执行(下行链路与控制台模式无关)，
              * 只有控制台报告受 mode 与 verbose 控制。 */
            while(NULL != (p = gps_fix_next(p, &fix)))
            {
                if(GPS_SRC_NONE == fix.source){
                    cli_stat_gps(0);                 /* 校验和或格式错：整句丢弃 */
                    continue;
                }
                cli_stat_gps(1);
                GPS_Info = fix.info;                 /* 同步维护旧符号，老代码仍可用 */

                /* 给第二颗 MCU 的最新值；flag bit1 表示数据来源 */
                gps_link_publish_fix(&fix.info,
                                     (GPS_SRC_GPYBM == fix.source) ? 1U : 0U,
                                     fix.valid);

                if((OUTPUT_STATE==WORKED) && cli_config_get()->verbose){
                    gps_report_print(&fix);
                }
            }

            if(OUTPUT_STATE==RAW){
                printf("%s", usart5_frame_buf);
            }
        }

        /* 下行：按 out/auto 决定是否发送，绝不排队(保最新) */
        (void)gps_link_poll();
    }
}

void GPSRX_Task(void *pvParameters)
{
    uint16_t len;
    while(1)
    {
        xSemaphoreTake(GPSRX_Semaph,portMAX_DELAY);
        len = usart5_rx_len;
        if(len >= USART5_RX_BUF_SIZE)
        {
            len = USART5_RX_BUF_SIZE - 1U;
        }
		memcpy(usart5_frame_buf,usart5_rx_buf,len);

		gps_link_publish_raw(usart5_frame_buf, len);
        usart5_frame_buf[len] = '\0';
        xSemaphoreGive(GPSDeal_Semaph);
    }
}

void GPSTX_Task(void *pvParameters)
{
    while(1)
    {
        vTaskDelay(1);
    }
}


/*!
    \brief      configure DMA interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
	nvic_irq_enable(USART5_IRQn, 5, 0);
	nvic_irq_enable(USART0_IRQn, 5, 0);
}

/*!
    \brief      configure DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    dma_single_data_parameter_struct dma_init_struct;
    dma_single_data_para_struct_init(&dma_init_struct);

    rcu_periph_clock_enable(RCU_DMA1);

    dma_deinit(USART5_RX_DMA, USART5_RX_DMA_CH);                                      /* USART5_RX 可用 CH1 或 CH2 */
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.memory0_addr = (uint32_t)usart5_rx_buf;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.number = USART5_RX_BUF_SIZE;                                   /* 缓冲 512 字节，索引 0~511，循环搬运 */
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART5);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_single_data_mode_init(USART5_RX_DMA,USART5_RX_DMA_CH, &dma_init_struct);

    dma_circulation_disable(USART5_RX_DMA, USART5_RX_DMA_CH);                          /* USART5_RX 可用 CH1 或 CH2 */
    dma_channel_subperipheral_select(USART5_RX_DMA, USART5_RX_DMA_CH, USART5_RX_DMA_SUBPERI);   /* 见 UM 表 10-3：USART5_RX 用 CH2，子外设号 5(0b101) */
    dma_flag_clear(USART5_RX_DMA, USART5_RX_DMA_CH, DMA_FLAG_FTF);
    dma_channel_enable(USART5_RX_DMA, USART5_RX_DMA_CH);                               /* USART5_RX 可用 CH1 或 CH2 */
    
    dma_deinit(USART0_RX_DMA, USART0_RX_DMA_CH);
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.memory0_addr = (uint32_t)usart0_rx_buf;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.number = USART0_RX_BUF_SIZE;
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART0);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(USART0_RX_DMA, USART0_RX_DMA_CH, &dma_init_struct);

    /* DMA 配成“每帧一次”：搬完进 IDLE 中断再由中断重新装载 */
    dma_circulation_disable(USART0_RX_DMA, USART0_RX_DMA_CH);
    dma_channel_subperipheral_select(USART0_RX_DMA, USART0_RX_DMA_CH, USART0_RX_DMA_SUBPERI);
    dma_flag_clear(USART0_RX_DMA, USART0_RX_DMA_CH, DMA_FLAG_FTF);
    dma_channel_enable(USART0_RX_DMA, USART0_RX_DMA_CH);
}



void usart_config(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART5);
	rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_6);
	gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_7);
	gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_10);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    /* configure USART */
    usart_deinit(USART5);
    usart_baudrate_set(USART5, 115200U);
    usart_receive_config(USART5, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART5, USART_TRANSMIT_ENABLE);
    usart_dma_receive_config(USART5, USART_RECEIVE_DMA_ENABLE);
    usart_enable(USART5);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* configure USART */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);
    usart_enable(USART0);
    /* 一帧结束：DMA 已经搬完，IDLE 中断负责重新装载并通知任务 */
    usart_interrupt_enable(USART0, USART_INT_IDLE);	
		/*enable interrupt*/
	usart_interrupt_enable(USART5, USART_INT_IDLE);
}

/* FreeRTOS 钩子 */

/* 需要在 FreeRTOSConfig.h 里把 configCHECK_FOR_STACK_OVERFLOW 设为 2：
  *  * 调度器每次任务切换都会检查栈填充图案，溢出时调用本钩子。
  *  * 没有它的话栈溢出只会悄悄破坏相邻内存，很难查。 */
volatile char g_stack_overflow_task[configMAX_TASK_NAME_LEN] = {0};

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    const char *p;

    (void)xTask;
    if(NULL != pcTaskName){
        strncpy((char *)g_stack_overflow_task, pcTaskName, configMAX_TASK_NAME_LEN - 1U);
    }

    /* 本钩子跑在调度器栈上，所以可以直接轮询发送
      * (不用 printf、不取控制台锁、不依赖调度器)。 */
    for(p = "\r\n*** STACK OVERFLOW in task: "; '\0' != *p; p++){
        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE)){
        }
        usart_data_transmit(USART0, (uint16_t)(uint8_t)*p);
    }
    if(NULL != pcTaskName){
        for(p = pcTaskName; '\0' != *p; p++){
            while(RESET == usart_flag_get(USART0, USART_FLAG_TBE)){
            }
            usart_data_transmit(USART0, (uint16_t)(uint8_t)*p);
        }
    }
    for(p = " ***\r\n"; '\0' != *p; p++){
        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE)){
        }
        usart_data_transmit(USART0, (uint16_t)(uint8_t)*p);
    }

    taskDISABLE_INTERRUPTS();
    for(;;){
        gd_eval_led_toggle(LED1);        /* 快闪=任务栈溢出 */
        for(volatile uint32_t i = 0U; i < 1000000U; i++){
        }
    }
}
