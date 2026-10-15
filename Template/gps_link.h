/* GPS 下行链路：把 GPS 原报文或解析后的定位值发给第二颗 MCU。
  * 内容由 out 决定，触发由 auto/rate 决定(见 cli_config_t)。
  * 语义是“保最新”：只留一份最新数据，发不出去就覆盖并计数，不排队、不重传、
  * 不流控，所以延迟不会累积。
  * 与控制台共用 USART0，因此每帧以一次原子 DMA 发出(详见 gps_link.c)。
  * 帧格式见 GD32F450_串行接口协议规范.md 第 3 章；本文件只写 ASCII 字符串。 */
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

/* ---- TYPE = 0x02：终端信息帧 ----
  * 给对端做上电自检与排障：固件版本、上电时长、计数器与当前下行配置。
  * 载荷 32 字节、定长，帧总长 40 字节(等于 LINK_FRAME_MAX)。全部小端。 */
#define LINK_TYPE_INFO      (0x02U)
#define LINK_PAYLOAD_INFO   (32U)

typedef struct
{
    uint32_t uptime_ms;      /* 终端上电毫秒数 */
    uint32_t version;        /* 固件版本 major<<16 | minor<<8 | patch */
    uint32_t tx_frames;      /* 已成功发出的下行帧数 */
    uint32_t tx_err;         /* 发送失败次数 */
    uint32_t drop_latest;    /* 被“保最新”覆盖丢弃的值 */
    uint32_t gps_ok;         /* 解析成功的语句数 */
    uint32_t gps_err;        /* 被丢弃的语句数 */
    uint8_t  out;            /* 当前下行内容 0=off 1=raw 2=parsed */
    uint8_t  auto_mode;      /* 当前触发方式 0=off 1=fix 2=tick */
    uint8_t  rate;           /* 当前速率(Hz) */
    uint8_t  sat_min;        /* 定位有效所需最少星数 */
} gps_link_info_t;

/* 下行链路统计快照 */
typedef struct
{
    uint32_t tx_frames;
    uint32_t tx_bytes;
    uint32_t tx_err;
    uint32_t drop_latest;
    uint16_t seq;
} gps_link_stats_t;

/* 读取统计快照 */
void gps_link_stats_get(gps_link_stats_t *out);

/* 立刻发一帧终端信息(与 out/auto/rate 无关)，0 = 已发出 */
int  gps_link_send_info(const gps_link_info_t *info);

#endif /* GPS_LINK_H */
