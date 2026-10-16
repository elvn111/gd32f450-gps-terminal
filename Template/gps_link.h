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

/* ---- 帧格式（本工程统一格式）----
  *   帧头(2) | 命令码(1) | 功能码(1) | 长度N(2,小端) | 数据(N) | CRC16(2,小端) | 帧尾(2)
  *   帧头 = A6 6A，帧尾 = B7 7B，帧总长 = 10 + N
  *   CRC = CRC-16/MODBUS，覆盖 帧头…数据段结束(6 + N 字节)，不含帧尾
  *   定帧必须靠 N：数据段里可能出现 B7 7B（例如透传的模块文本），不能搜索帧尾
  * 命令码 = 命令集，功能码 = 集内具体内容；上报帧的数据段前 2 字节是终端帧序号 */
#define LINK_HEAD0          (0xA6U)
#define LINK_HEAD1          (0x6AU)
#define LINK_TAIL0          (0xB7U)
#define LINK_TAIL1          (0x7BU)

/* 字节2 分区：bit7 = 0 是发给模块的命令，bit7 = 1 是模块往外发的
  *   0x01..0x0F 命令码(命令集)      —— 下行
  *   0x81..0x8F 应答(= 命令码 | 0x80) —— 上行
  *   0x90..0x9F 数据上报             —— 上行(本文件实现)
  *   0xA0..0xAF 事件上报             —— 上行
  *   其它保留 */
#define LINK_UP_DATA        (0x90U)     /* 上行·数据上报类 */
#define LINK_SUB_FIX        (0x01U)     /*   子类：定位数据 */
#define LINK_SUB_INFO       (0x02U)     /*   子类：终端信息 */

/* 预留（命令通道实现时使用） */
#define LINK_ACK_BASE       (0x80U)     /* 应答命令码 = 请求命令码 | LINK_ACK_BASE */
#define LINK_UP_EVENT       (0xA0U)     /* 上行·事件上报类 */

#define LINK_PAYLOAD_FIX    (24U)       /* 定位量字节数 */
#define LINK_PAYLOAD_INFO   (32U)       /* 信息量字节数 */
#define LINK_N_FIX          (2U + LINK_PAYLOAD_FIX)    /* 26：帧序号 + 定位量 */
#define LINK_N_INFO         (2U + LINK_PAYLOAD_INFO)   /* 34：帧序号 + 信息量 */
#define LINK_FRAME_MAX      (266U)      /* 10 + N，N 上限 256（命令通道预留） */

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

/* ---- 终端信息量（命令码 0x01 / 功能码 0x02 的数据段，去掉前 2 字节帧序号）----
  * 给对端做上电自检与排障：固件版本、上电时长、计数器与当前下行配置。全部小端。 */

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
