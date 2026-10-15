/* GPS 下行链路：把 GPS 原报文或解析后的定位值发给第二颗 MCU。
  * 内容由 out 决定，触发由 auto/rate 决定(见 cli_config_t)。
  * 语义是“保最新”：只留一份最新数据，发不出去就覆盖并计数，不排队、不重传、
  * 不流控，所以延迟不会累积。
  * 与控制台共用 USART0，因此每帧以一次原子 DMA 发出(详见 gps_link.c)。
  * 帧格式见 GPS_LINK_PROTOCOL.md；本文件只写 ASCII 字符串。 */
#ifndef GPS_LINK_H
#define GPS_LINK_H

#include <stdint.h>
#include "info_bsp.h"       /* 对应控制台的 6 行 GPS_Module_Info 报告 */

/* 下行内容(cli_config_t.out) */
#define LINK_OUT_OFF        (0U)    /* 不发任何下行数据 */
#define LINK_OUT_RAW        (1U)    /* GPS 原报文，字节级透传 */
#define LINK_OUT_PARSED     (2U)    /* 解析后的二进制帧 */

/* 发送触发方式(cli_config_t.auto_mode) */
#define LINK_AUTO_OFF       (0U)    /* 只由 fix 命令手动触发 */
#define LINK_AUTO_FIX       (1U)    /* 每个新值都发(受 rate 限流) */
#define LINK_AUTO_TICK      (2U)    /* 每 1/rate 秒发一次最新值(心跳) */

/* 限流范围(cli_config_t.rate，Hz) */
#define LINK_RATE_MIN       (1U)
#define LINK_RATE_MAX       (20U)

/* ---- 帧格式 ----
  * SOF(A5 5A) | LEN(1) | TYPE(1) | SEQ(2) | PAYLOAD | CRC16(2)
  * LEN = TYPE + SEQ + PAYLOAD 的字节数(不含 SOF/LEN/CRC)
  * CRC = 对 LEN 覆盖范围做 CRC-16/CCITT-FALSE，线上小端 */
#define LINK_SOF0           (0xA5U)
#define LINK_SOF1           (0x5AU)
#define LINK_TYPE_FIX       (0x01U)
#define LINK_PAYLOAD_FIX    (24U)
#define LINK_FRAME_MAX      (40U)

/* 定位帧 flag 位定义 */
#define LINK_FLAG_VALID     (0x01U)     /* 卫星数达到 sat_min，定位可信 */
#define LINK_FLAG_SRC_GPYBM (0x02U)     /* 数据来源：0=GPNAV，1=GPYBM */

/* 定位帧载荷(小端) */
typedef struct
{
    int32_t  lat_e7;        /* 纬度 ×1e7 */
    int32_t  lon_e7;        /* 经度 ×1e7 */
    int32_t  h_mm;          /* 椭球高，mm */
    uint16_t head_cd;       /* 航向 ×0.01° */
    uint32_t base_mm;       /* 双天线基线长，mm */
    uint8_t  sat;           /* 参与解算卫星数 */
    uint8_t  flag;          /* LINK_FLAG_*              */
    uint32_t t_ms;          /* 该帧生成的时刻(上电毫秒) */
} gps_fix_payload_t;

/* 初始化 USART0 发送 DMA 通道(只需调用一次) */
void gps_link_init(void);

/* 发布最新定位值，未发出的旧值被覆盖 */
void gps_link_publish_fix(const GPS_Module_Info *info, uint8_t source, uint8_t valid);

/* 发布最新原报文，未发出的旧值被覆盖 */
void gps_link_publish_raw(const uint8_t *buf, uint16_t len);

/* 在任务里频繁调用：按 out/auto/rate 决定是否发送，返回 1 表示发了一帧 */
int gps_link_poll(void);

/* 立刻发送当前最新值(忽略 auto 与限流) */
int gps_link_send_now(void);

/* gstat 命令用的状态报告 */
void gps_link_info(void);

#endif /* GPS_LINK_H */
