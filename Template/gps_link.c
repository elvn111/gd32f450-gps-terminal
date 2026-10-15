/* GPS 下行链路的实现：原报文透传 / 解析后的二进制帧。
  * 三条设计约束：
  *   1) 与控制台共用 USART0，所以整帧必须一次原子发出，不能和 printf 交叉；
  *   2) 语义是“保最新”，只保存一份最新值，发不出就覆盖并计数，绝不排队(延迟不累积)；
  *   3) 乒乓双缓冲：任务写一个、DMA 发另一个，发送期间不需要停任务。 */
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cli.h"
#include "buff_config.h"
#include "gps_link.h"

/* USART0 发送用 DMA1 CH7，PERIEN=0b100(GD32F4xx UM Rev3.4 表 10-3) */
#define LINK_TX_DMA         (DMA1)
#define LINK_TX_CH          (DMA_CH7)

/* 等待循环的粗略超时：115200 下约 87 us/字节，再乘安全系数，
  * 这样即使 DMA 异常，任务也只是丢一帧而不会永久卡住。 */
#define LINK_WAIT_PER_BYTE  (10000U)
#define LINK_WAIT_BASE      (200000U)

typedef struct
{
    uint8_t  buf[LINK_FRAME_MAX];
    uint16_t len;
    uint8_t  used;              /* 1=缓冲里有一个可以发送的值 */
} link_frame_t;

/* --- 解析帧内容：两个包缓冲(乒乓) --- */
static link_frame_t s_pkt[2];
static uint8_t      s_pkt_active;       /* 最新包的序号 */
static uint8_t      s_pkt_busy = 0xFFU; /* 正在发送的包序号，0xFF=没有 */
/* --- 原报文内容：两个报告缓冲(乒乓) --- */
static uint8_t      s_raw[2][USART5_RX_BUF_SIZE];
static uint16_t     s_raw_len[2];
static uint8_t      s_raw_active;
static uint8_t      s_raw_busy = 0xFFU;

static uint8_t      s_pending;          /* 有还没发出去的新值 */
static uint8_t      s_have_last;        /* 至少成功发送过一次 */
static uint32_t     s_last_tx_tick;
static uint16_t     s_seq;

/* 统计计数 */
static uint32_t s_tx_frames, s_tx_bytes, s_drop_latest, s_tx_err;
static int32_t  s_last_err;

/* CRC-16/CCITT-FALSE：多项式 0x1021，初值 0xFFFF，输入输出都不反转、不异或 */
static uint16_t link_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  b;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0U; b < 8U; b++) {
            if (0U != (crc & 0x8000U)) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* 发送：一次原子 DMA 突发 */
static int link_tx(const uint8_t *buf, uint16_t len)
{
    uint32_t guard;
    uint32_t limit = LINK_WAIT_BASE + ((uint32_t)len * LINK_WAIT_PER_BYTE);

    if ((NULL == buf) || (0U == len)) {
        return -1;
    }

    cli_out_lock();                     /* 控制台与帧不会交叉：与 printf 共用同一把控制台锁 */

    dma_channel_disable(LINK_TX_DMA, LINK_TX_CH);
    dma_memory_address_config(LINK_TX_DMA, LINK_TX_CH, DMA_MEMORY_0, (uint32_t)buf);
    dma_transfer_number_config(LINK_TX_DMA, LINK_TX_CH, (uint32_t)len);
    dma_flag_clear(LINK_TX_DMA, LINK_TX_CH, DMA_FLAG_FTF | DMA_FLAG_TAE | DMA_FLAG_SDE);
    dma_channel_enable(LINK_TX_DMA, LINK_TX_CH);

    /* 等 DMA 把所有字节交给 USART */
    guard = 0U;
    while (RESET == dma_flag_get(LINK_TX_DMA, LINK_TX_CH, DMA_FLAG_FTF)) {
        if (++guard > limit) {
            dma_channel_disable(LINK_TX_DMA, LINK_TX_CH);
            s_tx_err++;
            s_last_err = -1;            /* DMA 没发完：解锁、计数、报错，不在这儿死等 */
            cli_out_unlock();
            return -1;
        }
    }
    dma_flag_clear(LINK_TX_DMA, LINK_TX_CH, DMA_FLAG_FTF);

    /* 再等 TC：FTF 只说明字节进了数据寄存器，最后一个字节可能还在移位输出，
      * 这时控制台插进来写字符就会把它吃掉。 */
    guard = 0U;
    while (RESET == usart_flag_get(USART0, USART_FLAG_TC)) {
        if (++guard > limit) {
            s_tx_err++;
            s_last_err = -2;            /* USART 没发完 */
            cli_out_unlock();
            return -1;
        }
    }

    s_tx_frames++;
    s_tx_bytes += len;
    s_last_err = 0;
    cli_out_unlock();
    return 0;
}

/* 发布新值(保最新) */
static void link_publish_packet(const uint8_t *frame, uint16_t len)
{
    uint8_t target = (s_pkt_busy == 0U) ? 1U : 0U;   /* 不写正在发送的那个缓冲 */

    memcpy(s_pkt[target].buf, frame, len);
    s_pkt[target].len  = len;
    s_pkt[target].used = 1U;
    s_pkt_active = target;

    if (0U != s_pending) {
        s_drop_latest++;                /* 上一个值还没发出去：覆盖它并计数 */
    }
    s_pending = 1U;
}

void gps_link_publish_fix(const GPS_Module_Info *info, uint8_t source, uint8_t valid)
{
    uint8_t     frame[LINK_FRAME_MAX];
    gps_fix_payload_t pl;
    uint16_t    crc;
    uint16_t    len = LINK_TYPE_FIX + 2U + LINK_PAYLOAD_FIX;
    uint16_t    i;

    if (NULL == info) {
        return;
    }

    pl.lat_e7 = (int32_t)((info->latitude >= 0.0) ? (info->latitude * 10000000.0 + 0.5)
                                                 : (info->latitude * 10000000.0 - 0.5));
    pl.lon_e7 = (int32_t)((info->longitude >= 0.0) ? (info->longitude * 10000000.0 + 0.5)
                                                  : (info->longitude * 10000000.0 - 0.5));
    pl.h_mm   = (int32_t)((info->ElpHeight >= 0.0) ? (info->ElpHeight * 1000.0 + 0.5)
                                                   : (info->ElpHeight * 1000.0 - 0.5));
    pl.head_cd = (uint16_t)(info->Heading_Angle * 100.0);
    pl.base_mm = (uint32_t)(info->Baseline * 1000.0);
    pl.sat     = (uint8_t)info->GPS_Satellite;
    pl.flag    = (uint8_t)((0U != valid ? LINK_FLAG_VALID : 0U) |
                           (0U != source ? LINK_FLAG_SRC_GPYBM : 0U));
    pl.t_ms    = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    frame[0] = LINK_SOF0;
    frame[1] = LINK_SOF1;
    frame[2] = (uint8_t)len;
    frame[3] = LINK_TYPE_FIX;
    frame[4] = (uint8_t)(s_seq & 0xFFU);
    frame[5] = (uint8_t)(s_seq >> 8);

    i = 6U;
    memcpy(&frame[i], &pl.lat_e7, 4U);  i += 4U;
    memcpy(&frame[i], &pl.lon_e7, 4U);  i += 4U;
    memcpy(&frame[i], &pl.h_mm,   4U);  i += 4U;
    memcpy(&frame[i], &pl.head_cd, 2U); i += 2U;
    memcpy(&frame[i], &pl.base_mm, 4U); i += 4U;
    frame[i++] = pl.sat;
    frame[i++] = pl.flag;
    memcpy(&frame[i], &pl.t_ms, 4U);    i += 4U;

    crc = link_crc16(&frame[3], (uint16_t)len);      /* TYPE..PAYLOAD 参与 CRC 计算 */
    frame[i++] = (uint8_t)(crc & 0xFFU);
    frame[i++] = (uint8_t)(crc >> 8);

    link_publish_packet(frame, i);
}

void gps_link_publish_raw(const uint8_t *buf, uint16_t len)
{
    uint8_t target = (s_raw_busy == 0U) ? 1U : 0U;   /* 不写正在发送的那个缓冲 */

    if ((NULL == buf) || (0U == len) || (len > USART5_RX_BUF_SIZE)) {
        return;
    }
    memcpy(s_raw[target], buf, len);
    s_raw_len[target] = len;
    s_raw_active = target;

    if (0U != s_pending) {
        s_drop_latest++;
    }
    s_pending = 1U;
}

/* 发送与轮询 */
static int link_send_latest(void)
{
    const cli_config_t *cfg = cli_config_get();
    int rc;

    if (LINK_OUT_PARSED == cfg->out) {
        if ((0U == s_pkt[s_pkt_active].used) || (0U == s_pkt[s_pkt_active].len)) {
            return -1;
        }
        s_pkt_busy = s_pkt_active;
        rc = link_tx(s_pkt[s_pkt_active].buf, s_pkt[s_pkt_active].len);
        s_pkt_busy = 0xFFU;
    } else {
        if (0U == s_raw_len[s_raw_active]) {
            return -1;
        }
        s_raw_busy = s_raw_active;
        rc = link_tx(s_raw[s_raw_active], s_raw_len[s_raw_active]);
        s_raw_busy = 0xFFU;
    }

    if (0 == rc) {
        s_pending    = 0U;
        s_have_last  = 1U;
        s_last_tx_tick = xTaskGetTickCount();
        s_seq++;
    }
    return rc;
}

int gps_link_poll(void)
{
    const cli_config_t *cfg = cli_config_get();
    uint32_t now;
    uint32_t period_ticks;
    uint16_t rate;

    if (LINK_OUT_OFF == cfg->out) {
        return 0;
    }
    if (LINK_AUTO_OFF == cfg->auto_mode) {
        return 0;
    }
    if ((0U == s_pending) && (LINK_AUTO_TICK != cfg->auto_mode)) {
        return 0;                       /* 没有新值 */
    }
    if ((0U == s_pending) && (0U == s_have_last)) {
        return 0;                       /* 定时模式，但还没有值可发 */
    }

    rate = cfg->rate;
    if ((rate < 1U) || (rate > LINK_RATE_MAX)) {
        rate = 1U;
    }
    period_ticks = (uint32_t)configTICK_RATE_HZ / (uint32_t)rate;

    now = xTaskGetTickCount();
    if ((now - s_last_tx_tick) < period_ticks) {
        return 0;                       /* 限流：没到时间就留着最新值，超时的中间值直接丢 */
    }
    return (0 == link_send_latest()) ? 1 : 0;
}

int gps_link_send_now(void)
{
    const cli_config_t *cfg = cli_config_get();

    if (LINK_OUT_OFF == cfg->out) {
        printf("[link] out=off, nothing to send (try 'out raw' or 'out parsed')\r\n");
        return -1;
    }
    if (0 == link_send_latest()) {
        return 0;
    }
    printf("[link] no value available yet\r\n");
    return -1;
}

void gps_link_stats_get(gps_link_stats_t *out)
{
    if (NULL == out) {
        return;
    }
    out->tx_frames   = s_tx_frames;
    out->tx_bytes    = s_tx_bytes;
    out->tx_err      = s_tx_err;
    out->drop_latest = s_drop_latest;
    out->seq         = s_seq;
}

/* 终端信息帧：定长 40 字节，TYPE=0x02 */
int gps_link_send_info(const gps_link_info_t *info)
{
    uint8_t  frame[LINK_FRAME_MAX];
    uint16_t len = (uint16_t)(LINK_TYPE_INFO + 2U + LINK_PAYLOAD_INFO);
    uint16_t crc;
    uint16_t i = 6U;
    uint32_t v;

    if (NULL == info) {
        return -1;
    }

    frame[0] = LINK_SOF0;
    frame[1] = LINK_SOF1;
    frame[2] = (uint8_t)len;
    frame[3] = LINK_TYPE_INFO;
    frame[4] = (uint8_t)(s_seq & 0xFFU);
    frame[5] = (uint8_t)(s_seq >> 8U);

    v = info->uptime_ms;  frame[i++] = (uint8_t)(v & 0xFFU); frame[i++] = (uint8_t)((v >> 8) & 0xFFU);
    frame[i++] = (uint8_t)((v >> 16) & 0xFFU); frame[i++] = (uint8_t)((v >> 24) & 0xFFU);
    v = info->version;    frame[i++] = (uint8_t)(v & 0xFFU); frame[i++] = (uint8_t)((v >> 8) & 0xFFU);
    frame[i++] = (uint8_t)((v >> 16) & 0xFFU); frame[i++] = (uint8_t)((v >> 24) & 0xFFU);
    v = info->tx_frames;  frame[i++] = (uint8_t)(v & 0xFFU); frame[i++] = (uint8_t)((v >> 8) & 0xFFU);
    frame[i++] = (uint8_t)((v >> 16) & 0xFFU); frame[i++] = (uint8_t)((v >> 24) & 0xFFU);
    v = info->tx_err;     frame[i++] = (uint8_t)(v & 0xFFU); frame[i++] = (uint8_t)((v >> 8) & 0xFFU);
    frame[i++] = (uint8_t)((v >> 16) & 0xFFU); frame[i++] = (uint8_t)((v >> 24) & 0xFFU);
    v = info->drop_latest;frame[i++] = (uint8_t)(v & 0xFFU); frame[i++] = (uint8_t)((v >> 8) & 0xFFU);
    frame[i++] = (uint8_t)((v >> 16) & 0xFFU); frame[i++] = (uint8_t)((v >> 24) & 0xFFU);
    v = info->gps_ok;     frame[i++] = (uint8_t)(v & 0xFFU); frame[i++] = (uint8_t)((v >> 8) & 0xFFU);
    frame[i++] = (uint8_t)((v >> 16) & 0xFFU); frame[i++] = (uint8_t)((v >> 24) & 0xFFU);
    v = info->gps_err;    frame[i++] = (uint8_t)(v & 0xFFU); frame[i++] = (uint8_t)((v >> 8) & 0xFFU);
    frame[i++] = (uint8_t)((v >> 16) & 0xFFU); frame[i++] = (uint8_t)((v >> 24) & 0xFFU);

    frame[i++] = info->out;
    frame[i++] = info->auto_mode;
    frame[i++] = info->rate;
    frame[i++] = info->sat_min;

    crc = link_crc16(&frame[3], len);                /* TYPE..PAYLOAD 参与 CRC 计算 */
    frame[i++] = (uint8_t)(crc & 0xFFU);
    frame[i++] = (uint8_t)(crc >> 8U);

    if (0 != link_tx(frame, i)) {
        return -1;
    }
    s_have_last    = 1U;
    s_last_tx_tick = xTaskGetTickCount();
    s_seq++;
    return 0;
}

/* 初始化与信息 */
void gps_link_init(void)
{
    dma_single_data_parameter_struct tx;

    rcu_periph_clock_enable(RCU_DMA1);

    dma_deinit(LINK_TX_DMA, LINK_TX_CH);
    dma_single_data_para_struct_init(&tx);
    tx.periph_addr         = (uint32_t)&USART_DATA(USART0);
    tx.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    tx.memory0_addr        = (uint32_t)s_raw[0];
    tx.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    tx.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    tx.direction           = DMA_MEMORY_TO_PERIPH;
    tx.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(LINK_TX_DMA, LINK_TX_CH, &tx);
    dma_circulation_disable(LINK_TX_DMA, LINK_TX_CH);
    dma_channel_subperipheral_select(LINK_TX_DMA, LINK_TX_CH, DMA_SUBPERI4);

    /* USART0 侧：允许发送器产生 DMA 请求 */
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);
}

void gps_link_info(void)
{
    const cli_config_t *cfg = cli_config_get();
    static const char *out_name[]  = { "off", "raw", "parsed" };
    static const char *auto_name[] = { "off", "fix", "tick" };

    printf("[link] out=%s auto=%s rate=%lu Hz fmt=bin\r\n",
           (cfg->out <= LINK_OUT_PARSED) ? out_name[cfg->out] : "?",
           (cfg->auto_mode <= LINK_AUTO_TICK) ? auto_name[cfg->auto_mode] : "?",
           (unsigned long)cfg->rate);
    printf("[link] tx frames=%lu bytes=%lu dropped(latest)=%lu err=%lu seq=%u\r\n",
           (unsigned long)s_tx_frames, (unsigned long)s_tx_bytes,
           (unsigned long)s_drop_latest, (unsigned long)s_tx_err, (unsigned)s_seq);
    printf("[link] pending=%u last_err=%ld dma=%lu/%s\r\n",
           (unsigned)s_pending, (long)s_last_err,
           (unsigned long)LINK_TX_CH,
           (RESET != dma_flag_get(LINK_TX_DMA, LINK_TX_CH, DMA_FLAG_FTF)) ? "done" : "idle");
    if (0U != s_drop_latest) {
        printf("[link] drops mean the peer/report rate is faster than 'rate' "
               "(latest value wins by design)\r\n");
    }
}
