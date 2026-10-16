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
#include "version.h"
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

/* 按行组装：一块数据里可能含多条语句，也可能在某条语句中间被切断，
  * 因此把没凑成整行的尾巴留到下一块，只把完整语句交出去。
  * 这样后面每个环节拿到的都是“一条完整语句”。 */
#define GPS_LINE_CARRY_SIZE (2048U)
#define GPS_LINE_MAX        (256U)

static uint8_t  s_gps_carry[GPS_LINE_CARRY_SIZE];
static uint16_t s_gps_carry_len = 0U;
static uint16_t s_gps_carry_drop = 0U;    /* 没有行结束符而被迫丢弃的次数 */
static char     s_gps_line[GPS_LINE_MAX]; /* 一行语句的解析副本 */

/* 从块里取下一行：返回行首，len 含行尾的 CR/LF；返回 NULL 表示没有更多内容。 */
static char *gps_line_next(char **pp, uint16_t *len)
{
    char *s = *pp;
    char *end;
    char *term;

    if(NULL == s){
        return NULL;
    }
    while(('\r' == *s) || ('\n' == *s)){
        s++;
    }
    if('\0' == *s){
        *pp = NULL;
        return NULL;
    }
    end = s;
    while(('\0' != *end) && ('\r' != *end) && ('\n' != *end)){
        end++;
    }
    term = end;
    if('\r' == *term){
        term++;
        if('\n' == *term){
            term++;
        }
    }else if('\n' == *term){
        term++;
    }
    *len = (uint16_t)(term - s);
    *pp  = ('\0' == *term) ? NULL : term;
    return s;
}

/* USART0 接收 DMA 的缓冲配置见 buff_config.h */
uint8_t usart0_rx_buf[USART0_RX_BUF_SIZE] = {0};
volatile uint16_t usart0_rx_len = 0;

/* 控制台输入的软件环形缓冲：中断写、RX_Task 读。
  * 下标按容量取模，读写各占一端，因此不需要加锁。 */
static volatile uint8_t  s_ring[USART0_RING_SIZE];
static volatile uint16_t s_ring_head = 0U;
static volatile uint16_t s_ring_tail = 0U;
static volatile uint16_t s_ring_ovf  = 0U;

void usart0_rx_ring_push(const uint8_t *buf, uint16_t len)
{
    uint16_t used, space, i;

    if((NULL == buf) || (0U == len)){
        return;
    }
    used  = (uint16_t)((s_ring_head - s_ring_tail) & (USART0_RING_SIZE - 1U));
    space = (uint16_t)(USART0_RING_SIZE - 1U - used);
    if(len > space){
        s_ring_ovf = (uint16_t)(s_ring_ovf + (len - space));
        len = space;
    }
    for(i = 0U; i < len; i++){
        s_ring[s_ring_head] = buf[i];
        s_ring_head = (uint16_t)((s_ring_head + 1U) & (USART0_RING_SIZE - 1U));
    }
}

uint16_t usart0_rx_ring_pop(uint8_t *dst, uint16_t max)
{
    uint16_t n = 0U;

    if(NULL == dst){
        return 0U;
    }
    while((n < max) && (s_ring_tail != s_ring_head)){
        dst[n] = s_ring[s_ring_tail];
        s_ring_tail = (uint16_t)((s_ring_tail + 1U) & (USART0_RING_SIZE - 1U));
        n++;
    }
    return n;
}

uint16_t usart0_rx_ring_used(void)
{
    return (uint16_t)((s_ring_head - s_ring_tail) & (USART0_RING_SIZE - 1U));
}

uint16_t usart0_rx_ring_overflow(void)
{
    return s_ring_ovf;
}

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
	"unlogall\r\n",                     /* 0: 只在第一遍发 —— 清掉旧日志 */
	"set appscene survey\r\n",          /* 1: 只在第一遍发 —— 场景设为测量 */
	"log com1 gpnav ontime 1\r\n",      /* 2: 每次都发 —— 1 Hz 导航数据 */
	"log com1 gpybm ontime 1\r\n",      /* 3: 每次都发 —— 1 Hz 观测数据 */
};

/* =====================================================================
 * 预设命令的发送管理(现场实测出来的三条结论，上电"不发"就栽在这里)
 *   1) 命令必须一条一条发、中间留间隔。模块收到 appscene 会重启引擎，
 *      背靠背连发时后面几行会被丢掉：实测上电那一串只有前两条被应答，
 *      最后两条 log 丢了 —— 而 unlogall 已经把日志全部关掉、log 又没生效，
 *      模块从此刻起一个字节都不发。
 *   2) 重发只重发 log 两行。再发一遍 unlogall 只会把刚起来的日志又关掉。
 *   3) "模块活了"只看它以 '$' 开头的语句。模块对命令的应答(OK! /
 *      Command accepted!)不算数，否则第一遍刚发完就被应答骗过去，
 *      日志其实没起来也不会再补发。
 * 所以：一批命令按 250 ms 逐条发；前 3 批间隔 2 s，之后每 10 s 补一次 log，
 *       直到模块真的出语句为止(模块中途被复位/掉电也能自愈)。
 * ===================================================================== */
#define GPS_PRESET_LOG_FIRST    (2U)        /* 表中第一条 log 命令的下标 */
#define GPS_PRESET_FIRST_MS     (300U)      /* 模块启动比 MCU 慢，等它起来再发 */
#define GPS_PRESET_CMD_GAP_MS   (250U)      /* 命令行之间的间隔 */
#define GPS_PRESET_RETRY_MS     (2000U)     /* 前三批的间隔 */
#define GPS_PRESET_MAX_TRIES    (3U)
#define GPS_PRESET_SLOW_MS      (5000U)     /* 之后每 5 s 补发一次 log */

static uint32_t s_preset_next_ms;       /* 下一批 / 下一条的时刻 */
static uint8_t  s_preset_idx;           /* 批内当前要发的命令下标 */
static uint8_t  s_preset_from;          /* 本批从哪条开始(重发跳过前两条) */
static uint8_t  s_preset_tries;         /* 已经发过几批 */
static uint8_t  s_preset_sending;       /* 1 = 一批正在逐条发 */
static uint8_t  s_preset_armed;         /* 0 = 还没排期 */
static uint8_t  s_gps_alive;            /* 1 = 模块回过任何字节 */
static uint8_t  s_gps_data;             /* 1 = 模块回过 '$' 语句(真在出数据) */
static uint32_t s_gps_rx_bytes;         /* 模块一共回了多少字节(心跳里报) */

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


/* 重新走一遍预设流程。真正的发送由 gps_preset_tick() 逐条、带间隔地做
  * (背靠背连发会让模块丢掉后面的命令)，这里只把状态机复位。
  * 顺带修掉了原来缺 return 的 #940-D 告警。 */
int gps_init(void)
{
    s_gps_data       = 0U;
    s_preset_sending = 0U;
    s_preset_tries   = 0U;
    s_preset_armed   = 0U;
    return 0;
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


/* 硬复位模块：PB4 拉低->释放->等它启动->重发预设命令。
  * 只能在任务里调用(vTaskDelay)。 */
static void plat_gps_hw_reset(void)
{
    printf("[gps] hardware reset (PB4 low %u ms)\r\n", (unsigned)GPS_RESET_LOW_MS);
    gps_reset_pulse(GPS_RESET_LOW_MS);
    vTaskDelay(pdMS_TO_TICKS(GPS_RESET_BOOT_MS));
    (void)gps_init();
    printf("[gps] module rebooted, preset commands sent\r\n");
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

/* 递归互斥量，不能用普通互斥量：
  * cli_input() 会一直持锁到整行命令处理完，而 'info'/'fix' 这类命令在锁内
  * 还要发一帧(link_tx() 取的是同一把锁，用来保证帧和控制台文字不交叉)。
  * 普通互斥量遇到同任务重复进入就是自锁：RX_Task 永久阻塞且不释放，
  * TX_Task 随后也卡在锁上 => 控制台与下行帧全部停摆，只能重新上电。 */
static void plat_out_lock(void)
{
    if(NULL != g_console_lock){
        (void)xSemaphoreTakeRecursive(g_console_lock, portMAX_DELAY);
    }
}

static void plat_out_unlock(void)
{
    if(NULL != g_console_lock){
        (void)xSemaphoreGiveRecursive(g_console_lock);
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
    printf("[dma] console ring: used=%lu/%lu overflow=%lu last_seg=%lu\r\n",
           (unsigned long)usart0_rx_ring_used(), (unsigned long)USART0_RING_SIZE,
           (unsigned long)usart0_rx_ring_overflow(), (unsigned long)usart0_rx_len);
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

/* info 命令：把终端现状打包成一帧(命令码 0x01 / 功能码 0x02)发给对端 */
static void plat_link_info_send(void)
{
    const cli_config_t *cfg = cli_config_get();
    cli_stats_t        st;
    gps_link_stats_t   ls;
    gps_link_info_t    info;

    memset(&st, 0, sizeof(st));
    memset(&ls, 0, sizeof(ls));
    memset(&info, 0, sizeof(info));
    cli_stats_get(&st);
    gps_link_stats_get(&ls);

    info.uptime_ms   = plat_uptime_ms();
    info.version     = FW_VERSION_PACKED;   /* 版本号来自 version.h */
    info.tx_frames   = ls.tx_frames;
    info.tx_err      = ls.tx_err;
    info.drop_latest = ls.drop_latest;
    info.gps_ok      = st.gps_ok;
    info.gps_err     = st.gps_err;
    info.out         = cfg->out;
    info.auto_mode   = cfg->auto_mode;
    info.rate        = (uint8_t)cfg->rate;
    info.sat_min     = cfg->sat_min;

    if (0 == gps_link_send_info(&info)) {
        printf("[link] info frame sent (0x90/0x02, %u bytes)\r\n",
               (unsigned)(LINK_N_INFO + 10U));
    } else {
        printf("[link] info frame NOT sent\r\n");
    }
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
    .gps_hw_reset       = plat_gps_hw_reset,
    .link_info_send     = plat_link_info_send,
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
/* 上电打印复位来源（读 RCU_RSTSCK 并清零）。
  * 反复打印开机横幅 = 板子在反复复位，这一行会说明是谁把它复位的：
  *   BOR   = 供电跌落（最常见：模块/天线一工作电流拉低电压）
  *   EPRST = NRST 被外部拉低（调试器、串口工具的 DTR/RTS 都可能）
  *   SW / FWDGT / WWDGT = 软件复位或看门狗
  *   POR   = 正常上电 */
/* 本次上电读到的复位标志，心跳里要重复用 */
static uint32_t s_rst_rstsck = 0U;

static void reset_cause_print(void)
{
    uint32_t f = RCU_RSTSCK;
    const char *src = "unknown";

    s_rst_rstsck = f;

    if (0U != (f & RCU_RSTSCK_PORRSTF)) {
        src = "POR (power-on)";
    } else if (0U != (f & RCU_RSTSCK_BORRSTF)) {
        src = "BOR (BROWN-OUT: supply dropped!)";
    } else if (0U != (f & RCU_RSTSCK_EPRSTF)) {
        src = "EPRST (external NRST pin pulled low!)";
    } else if (0U != (f & RCU_RSTSCK_SWRSTF)) {
        src = "SW (software reset NVIC_SystemReset)";
    } else if (0U != (f & RCU_RSTSCK_FWDGTRSTF)) {
        src = "FWDGT (free watchdog reset!)";
    } else if (0U != (f & RCU_RSTSCK_WWDGTRSTF)) {
        src = "WWDGT (window watchdog reset!)";
    } else if (0U != (f & RCU_RSTSCK_LPRSTF)) {
        src = "LPRST (low-power reset)";
    }

    /* 逐位打印：一次复位可能同时置多个标志(POR 与 EPRST 一起出现很常见)，
      * 只报第一个命中的名字会把另一半信息丢掉。 */
    printf("\r\n[rst] RCU_RSTSCK=0x%08lX -> %s\r\n", (unsigned long)f, src);
    printf("[rst] bits: POR=%u BOR=%u EPRST=%u SW=%u FWDT=%u WWDT=%u LRST=%u\r\n",
           (unsigned)((0U != (f & RCU_RSTSCK_PORRSTF)) ? 1U : 0U),
           (unsigned)((0U != (f & RCU_RSTSCK_BORRSTF)) ? 1U : 0U),
           (unsigned)((0U != (f & RCU_RSTSCK_EPRSTF)) ? 1U : 0U),
           (unsigned)((0U != (f & RCU_RSTSCK_SWRSTF)) ? 1U : 0U),
           (unsigned)((0U != (f & RCU_RSTSCK_FWDGTRSTF)) ? 1U : 0U),
           (unsigned)((0U != (f & RCU_RSTSCK_WWDGTRSTF)) ? 1U : 0U),
           (unsigned)((0U != (f & RCU_RSTSCK_LPRSTF)) ? 1U : 0U));
    RCU_RSTSCK |= RCU_RSTSCK_RSTFC;         /* 清标志：下次复位才能看出新原因 */
}

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

    reset_cause_print();        /* 开机第一行：上一次复位是谁引起的 */

    /* USART0 发送 DMA(DMA1 CH7)：给第二颗 MCU 的下行链路 */
    gps_link_init();

#if GPS_RESET_ON_BOOT
    /* 硬件复位模块(PB4 拉低)，等它启动完再下发命令 */
    gps_reset_pulse(GPS_RESET_LOW_MS);
    vTaskDelay(pdMS_TO_TICKS(GPS_RESET_BOOT_MS));
#endif
    /* 预设命令不在这里发：交给 TX_Task 的 gps_preset_tick()，
      * 它会等 1 s（模块启动更慢）并在模块静默时自动重发。 */
    gd_eval_led_init (LED1);

    g_console_lock = xSemaphoreCreateRecursiveMutex();
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
        vTaskDelay(500);        /* 灭 500 ms */
    }
}


GPS_Module_Info GPS_Info;
void RX_Task(void *pvParameters)
{
    /* 一次取一段交给解释器：进来的可能是一行、半行或多行，解释器自己拼行。
      * 取空为止，避免一次中断只处理一小段而留下积压。 */
    uint8_t  chunk[CLI_LINE_MAX];
    uint16_t n;

    while(1)
    {
        if(xSemaphoreTake(RX_Semaph, portMAX_DELAY) == pdTRUE)
        {
            while(0U != (n = usart0_rx_ring_pop(chunk, (uint16_t)sizeof(chunk))))
            {
                cli_input((const uint8_t *)chunk, n);
            }
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

/* 预设命令逐条发送的状态机：见文件前面"预设命令的发送管理"的说明 */
static void gps_preset_tick(void)
{
    uint32_t now = plat_uptime_ms();

    if (0U != s_gps_data) {
        return;                                 /* 模块已经在出语句：收工 */
    }
    if (0U == s_preset_armed) {                 /* 首次调用 / 命令重新武装 */
        s_preset_armed   = 1U;
        s_preset_next_ms = now + GPS_PRESET_FIRST_MS;
        return;
    }
    if ((int32_t)(now - s_preset_next_ms) < 0) {
        return;
    }

    if (0U == s_preset_sending) {               /* 开一批 */
        s_preset_from    = (0U == s_preset_tries) ? 0U : (uint8_t)GPS_PRESET_LOG_FIRST;
        s_preset_idx     = s_preset_from;
        s_preset_sending = 1U;
        s_preset_next_ms = now;
        return;                                 /* 下个 tick 起一条一条发 */
    }

    uart_send_string(USART5, GPS_CMD[s_preset_idx]);    /* 一次只发一条 */
    s_preset_idx++;
    s_preset_next_ms = now + GPS_PRESET_CMD_GAP_MS;

    if (s_preset_idx >= (uint8_t)STRING_NUM) {          /* 一批发完 */
        s_preset_sending = 0U;
        s_preset_tries++;
        s_preset_next_ms = now + ((s_preset_tries < GPS_PRESET_MAX_TRIES)
                                  ? GPS_PRESET_RETRY_MS : GPS_PRESET_SLOW_MS);
        cli_out_lock();
        if (1U == s_preset_tries) {
            printf("[gps] preset sent one by one, waiting for module data\r\n");
        } else {
            printf("[gps] module quiet, log commands re-sent (%u)\r\n",
                   (unsigned)s_preset_tries);
        }
        cli_out_unlock();
    }
}

/* =====================================================================
 * 链路保活
 * 模块静默时(没接天线、还没定位、模块没起来)以前是发完 3 遍预设就彻底安静，
 * 打开串口的那一刻什么都看不到，看起来就是"不发"。
 * 这里保证下行任何时刻都不会超过 1 s 没有帧：
 *   每秒检查一次已发帧数，这一秒里有帧出去过就不动(正常定位时不增加流量)，
 *   一帧都没有才补发一帧终端信息帧(证明终端活着，对端也能据此知道链路通)。
 *   命令 'out off' 是明确要求不发，此时不补。
 * ===================================================================== */
#define GPS_KEEPALIVE_MS        (1000U)
#define GPS_KEEPALIVE_FIRST_MS  (300U)      /* 上电第一帧只等 300 ms：先让线上有帧 */

static uint32_t s_keepalive_next_ms;    /* 下一次检查的时刻 */
static uint32_t s_keepalive_frames;     /* 上次检查时已发出的帧数 */

static void gps_keepalive_tick(void)
{
    gps_link_stats_t    ls;
    const cli_config_t *cfg = cli_config_get();
    uint32_t            now = plat_uptime_ms();

    if (0U == s_keepalive_next_ms) {            /* 首次调用：排期 */
        s_keepalive_next_ms = now + GPS_KEEPALIVE_FIRST_MS;
        return;
    }
    if ((int32_t)(now - s_keepalive_next_ms) < 0) {
        return;
    }
    s_keepalive_next_ms = now + GPS_KEEPALIVE_MS;

    memset(&ls, 0, sizeof(ls));
    gps_link_stats_get(&ls);
    if (ls.tx_frames != s_keepalive_frames) {   /* 这一秒里已经有帧发出去了 */
        s_keepalive_frames = ls.tx_frames;
        return;
    }
    if (LINK_OUT_OFF == cfg->out) {             /* 用户要求不发 */
        return;
    }
    (void)plat_link_info_send();
    memset(&ls, 0, sizeof(ls));
    gps_link_stats_get(&ls);                    /* 把自己这一帧也算进去，下一轮才判得准 */
    s_keepalive_frames = ls.tx_frames;
}

/* =====================================================================
 * 上电心跳：把"板子活着吗 / 帧发出去没有 / 模块回话没有"打成一行，
 * 上电后前 20 秒每秒一行，然后自动安静(verbose 打开时一直打)。
 * 排查"不发"时先看这一行：
 *   up 一直在涨        -> 没有复位，板子在跑
 *   up 反复回到 1000   -> 复位循环(看 rst 位判断是哪种复位)
 *   tx 一直不涨        -> 帧根本没组出来/没发出去
 *   rx5 = 0            -> 模块一个字节都没回来(预设没生效或模块没上电)
 *   rx0 不涨           -> 控制台输入进不来(RX_Task 卡住)
 * ===================================================================== */
#define GPS_HB_MS       (1000U)
#define GPS_HB_WINDOW   (20000U)

static uint32_t s_hb_next_ms;

static void gps_heartbeat_tick(void)
{
    cli_stats_t      st;
    gps_link_stats_t ls;
    uint32_t         now = plat_uptime_ms();

    if ((now > GPS_HB_WINDOW) && (0U == cli_config_get()->verbose)) {
        return;                                 /* 过了观察窗口就不再占控制台 */
    }
    if (0U == s_hb_next_ms) {
        s_hb_next_ms = now + GPS_HB_MS;
        return;
    }
    if ((int32_t)(now - s_hb_next_ms) < 0) {
        return;
    }
    s_hb_next_ms = now + GPS_HB_MS;

    memset(&st, 0, sizeof(st));
    memset(&ls, 0, sizeof(ls));
    cli_stats_get(&st);
    gps_link_stats_get(&ls);

    cli_out_lock();
    printf("[hb] up=%lums tx=%lu rx5=%lu alive=%u data=%u rx0=%lu gps_ok=%lu gps_err=%lu "
           "rst=0x%08lX POR=%u EPRST=%u SW=%u FWDT=%u WWDT=%u\r\n",
           (unsigned long)now,
           (unsigned long)ls.tx_frames,
           (unsigned long)s_gps_rx_bytes,
           (unsigned)s_gps_alive,
           (unsigned)s_gps_data,
           (unsigned long)st.rx_bytes,
           (unsigned long)st.gps_ok,
           (unsigned long)st.gps_err,
           (unsigned long)s_rst_rstsck,
           (unsigned)((0U != (s_rst_rstsck & RCU_RSTSCK_PORRSTF)) ? 1U : 0U),
           (unsigned)((0U != (s_rst_rstsck & RCU_RSTSCK_EPRSTF)) ? 1U : 0U),
           (unsigned)((0U != (s_rst_rstsck & RCU_RSTSCK_SWRSTF)) ? 1U : 0U),
           (unsigned)((0U != (s_rst_rstsck & RCU_RSTSCK_FWDGTRSTF)) ? 1U : 0U),
           (unsigned)((0U != (s_rst_rstsck & RCU_RSTSCK_WWDGTRSTF)) ? 1U : 0U));
    cli_out_unlock();
}

void TX_Task(void *pvParameters)
{
    while(1)
    {
        if(xSemaphoreTake(GPSDeal_Semaph,0)==pdTRUE)
        {
            char     *p = (char *)usart5_frame_buf;
            char     *line;
            uint16_t  line_len;
            uint16_t  copy;

            /* 逐条语句处理：解析、下行、控制台都按“一条完整语句”走。
              * 解析与下发始终执行(下行链路与控制台模式无关)，
              * 只有控制台报告受 mode 与 verbose 控制。 */
            while(NULL != (line = gps_line_next(&p, &line_len)))
            {
                gps_fix_t fix;
                uint8_t   valid;

                memset(&fix, 0, sizeof(fix));

                /* 解析用去掉 CR/LF 的副本，下行仍发原始字节 */
                copy = line_len;
                if(copy >= (uint16_t)sizeof(s_gps_line))
                {
                    copy = (uint16_t)(sizeof(s_gps_line) - 1U);
                }
                memcpy(s_gps_line, line, copy);
                s_gps_line[copy] = '\0';
                while((copy > 0U) &&
                      (('\r' == s_gps_line[copy - 1U]) || ('\n' == s_gps_line[copy - 1U])))
                {
                    s_gps_line[--copy] = '\0';
                }

                if('$' == s_gps_line[0])
                {
                    s_gps_data = 1U;   /* 模块真的在出语句：预设流程可以收工 */
                }

                if(0 != gps_fix_parse_one(s_gps_line, &fix))
                {
                    cli_stat_gps(1);
                    valid = fix.valid;
                    if((uint8_t)fix.info.GPS_Satellite < cli_config_get()->sat_min)
                    {
                        valid = 0U;            /* 星数门槛：sat_min 参与有效性判定 */
                    }
                    GPS_Info = fix.info;       /* 同步维护旧符号，老代码仍可用 */
                    gps_link_publish_fix(&fix.info,
                                         (GPS_SRC_GPYBM == fix.source) ? 1U : 0U,
                                         valid);

                    if((OUTPUT_STATE==WORKED) && cli_config_get()->verbose){
                        gps_report_print(&fix);
                    }
                }
                else if('$' == s_gps_line[0])
                {
                    cli_stat_gps(0);           /* '$' 语句校验和或格式错：整句丢弃 */
                }
                /* 模块对命令的应答(OK! / Command accepted!)不是语句，不计数 */

                /* 原报文下行：一条语句一帧，含它自己的 CRLF */
                gps_link_publish_raw((const uint8_t *)line, line_len);

                if(OUTPUT_STATE==RAW){
                    printf("%s\r\n", s_gps_line);
                }
            }
        }

        /* GPS 预设重发管理 + 下行：按 out/auto 决定是否发送，绝不排队(保最新) */
        gps_preset_tick();
        gps_keepalive_tick();       /* 每秒保证至少一帧出去，见上 */
        gps_heartbeat_tick();       /* 上电 20 s 内每秒一行状态，见上 */
        (void)gps_link_poll();
    }
}

void GPSRX_Task(void *pvParameters)
{
    uint16_t len, keep, cut, i, off;

    while(1)
    {
        xSemaphoreTake(GPSRX_Semaph, portMAX_DELAY);

        len = usart5_rx_len;
        if(len >= USART5_RX_BUF_SIZE)
        {
            len = USART5_RX_BUF_SIZE - 1U;
        }

        /* 接回上一块没凑成整行的尾巴 */
        keep = s_gps_carry_len;
        if((uint32_t)keep + (uint32_t)len > (uint32_t)sizeof(s_gps_carry))
        {
            keep = 0U;                      /* 一直收不到行结束符：当垃圾丢掉 */
            s_gps_carry_len = 0U;
            s_gps_carry_drop++;
        }
        if(0U != len){
            s_gps_alive = 1U;       /* 模块有任何数据回来就不再重发预设 */
            s_gps_rx_bytes += len;
        }
        memcpy(&s_gps_carry[keep], usart5_rx_buf, len);
        keep = (uint16_t)(keep + len);

        /* 找到最后一个行结束符：它之前(含)都是完整语句，之后留给下一块 */
        cut = 0U;
        for(i = 0U; i < keep; i++)
        {
            if('\n' == s_gps_carry[i])
            {
                cut = (uint16_t)(i + 1U);
            }
            else if('\r' == s_gps_carry[i])
            {
                cut = (uint16_t)(i + 1U);
                if(((uint16_t)(i + 1U) < keep) && ('\n' == s_gps_carry[i + 1U]))
                {
                    cut = (uint16_t)(i + 2U);
                    i++;
                }
            }
        }

        if(0U != cut)
        {
            off = 0U;
            if(cut > (uint16_t)(USART5_RX_BUF_SIZE - 1U))
            {
                /* 极端情况：积压的完整行超过缓冲，丢掉最前面一段 */
                off = (uint16_t)(cut - (USART5_RX_BUF_SIZE - 1U));
                cut = (uint16_t)(USART5_RX_BUF_SIZE - 1U);
                s_gps_carry_drop++;
            }
            memcpy(usart5_frame_buf, &s_gps_carry[off], cut);
            usart5_frame_buf[cut] = '\0';

            s_gps_carry_len = (uint16_t)(keep - off - cut);
            if(0U != s_gps_carry_len)
            {
                memmove(s_gps_carry, &s_gps_carry[off + cut], s_gps_carry_len);
            }
            xSemaphoreGive(GPSDeal_Semaph);
        }
        else
        {
            /* 还没有完整语句：留着，不惊动解析任务 */
            s_gps_carry_len = keep;
            if(keep >= (uint16_t)(sizeof(s_gps_carry) - 1U))
            {
                s_gps_carry_len = 0U;
                s_gps_carry_drop++;
            }
        }
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
    for(p = " *** rebooting\r\n"; '\0' != *p; p++){
        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE)){
        }
        usart_data_transmit(USART0, (uint16_t)(uint8_t)*p);
    }

    /* 不再停机：栈溢出后复位自愈，下一次上电的 [rst] 会带 SW=1，
      * 串口上也留下了是哪个任务溢出的。 */
    for(volatile uint32_t i = 0U; i < 800000U; i++){
    }
    NVIC_SystemReset();
    taskDISABLE_INTERRUPTS();
    for(;;){
    }
}
