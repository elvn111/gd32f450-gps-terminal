/* USART0 控制台上的命令解释器：改配置、操作 GPS、转发数据。
  * 数据流(全部在任务里执行)：DMA+IDLE 中断 -> cli_input(字节段)
  * -> 行装配 -> 分词 -> 命令表 -> 处理函数 -> printf / 平台钩子。
  * 交互：回显、退格、Ctrl+C 丢弃本行、Ctrl+L 清屏、TAB 补全、上下键历史。
  * 扩展：加命令=加一行 s_cmds[]，加配置项=加一行 s_items[]，
  * 换硬件=实现一个平台钩子(cli_platform_t)。
  * 注意：本文件只写 ASCII 字符串；中文只出现在注释里，
  * 因为 AC5 只对字符串字面量里的非 ASCII 报 #870-D 并打乱后续语法解析。 */
#ifndef CLI_H
#define CLI_H

#include <stdint.h>

/* 可调参数 */
#define CLI_LINE_MAX        96U     /* 单行最大长度(含结尾 '\0') */
#define CLI_ARG_MAX         12U     /* 单行最多参数个数(含命令名) */
#define CLI_HIST_MAX        4U      /* 历史记录条数(上下键) */

/* 运行配置：get/set 直接读写这些字段，save/load 整块存取 */
typedef struct
{
    uint8_t  mode;          /* 控制台输出：0=GPS 原报文，1=解析后的 6 行 [GPS_DATA] */
    uint8_t  echo;          /* 1=回显输入并显示提示符(交互终端用) */
    uint8_t  verbose;       /* 1=每解析到一句就打印 6 行 */
    uint8_t  sat_min;       /* 判定定位有效所需最少卫星数 */
    uint32_t gps_baud;      /* GPS(USART5) 波特率，改动会重配串口 */
    uint8_t  out;           /* 下行(发给第二颗 MCU)：0=不发，1=原报文，2=二进制帧 */
    uint8_t  auto_mode;     /* 下行触发：0=只手动，1=每个新定位，2=定时心跳 */
    uint16_t rate;          /* 下行限流/心跳频率(1..20 Hz) */
    uint8_t  prefix;        /* 上行前缀字符(默认 '@')，0=关闭前缀路由 */
} cli_config_t;

/* 平台钩子：任一可为 NULL，对应命令回“未实现”，解释器本身不依赖具体硬件 */
typedef struct
{
    void     (*gps_send_line)(const char *line);   /* 向 GPS 模块发一行(内部补 CRLF) */
    void     (*gps_send_raw)(const uint8_t *buf, uint16_t len);  /* 原样转发字节(桥模式，不加任何修饰) */
    void     (*gps_preset)(void);                  /* 发送开机预设命令表 */
    void     (*gps_info_print)(void);              /* 打印最近一次解析结果(gps info) */
    void     (*gps_baud_set)(uint32_t baud);       /* 重配 USART5 波特率 */
    void     (*output_mode_set)(uint8_t mode);     /* 切换控制台输出形态 */
    uint32_t (*usart0_rx_dma_left)(void);          /* 查询 USART0 接收 DMA 剩余计数 */
    void     (*tasks_info)(void);                  /* 打印任务表与栈余量(tasks) */
    void     (*dma_info)(void);                    /* 打印接收 DMA 通道状态(dma) */
    int      (*config_save)(const cli_config_t *cfg);  /* 0 = ok */
    int      (*config_load)(cli_config_t *cfg);        /* 0 = ok */
    void     (*nvm_info)(void);                    /* 打印 Flash 配置记录状态 */
    void     (*send_now)(void);                    /* 立刻下发一份最新值(fix) */
    void     (*link_info)(void);                   /* 打印下行链路状态(gstat) */
    void     (*out_lock)(void);                    /* 控制台输出加锁/解锁，供其它打印任务复用 */
    void     (*out_unlock)(void);
    void     (*system_reset)(void);
    uint32_t (*uptime_ms)(void);
    void     (*gps_hw_reset)(void);                /* 拉 PB4 硬复位 GPS 模块(gps hwreset) */
    void     (*link_info_send)(void);              /* 下发一帧终端信息(字节2 = 0x90 数据上报 / 字节3 = 0x02 终端信息) */
} cli_platform_t;

/* 接口 */
void cli_init(const cli_platform_t *plat);

/* 喂入收到的字节：一段里可能含 0~N 行，一行也可能跨多段，
  * 行缓冲负责拼接。桥模式下每个字节原样转给 GPS。 */
void cli_input(const uint8_t *data, uint16_t len);

/* 直接执行一行(自测用)，会就地修改传入的字符串 */
void cli_exec_line(char *line);

/* 读取当前配置(只读指针，不要改) */
const cli_config_t *cli_config_get(void);

/* GPS 解析统计：1=解析成功，0=被丢弃 */
void cli_stat_gps(int ok);

/* 供其它任务使用的控制台输出锁：包住整段输出，
  * 避免别的任务把一行/一组数据从中截断；CLI 内部自己也会加锁。 */
void cli_out_lock(void);
void cli_out_unlock(void);

/* 统计快照：status 打印与终端信息帧共用同一份口径 */
typedef struct
{
    uint32_t rx_frames;      /* 控制台收到的数据段数 */
    uint32_t rx_bytes;       /* 控制台收到的字节数 */
    uint32_t cmds;           /* 执行过的命令数 */
    uint32_t unknown;        /* 未知命令数 */
    uint32_t gps_ok;         /* 解析成功的语句数 */
    uint32_t gps_err;        /* 被丢弃的语句数 */
} cli_stats_t;

void cli_stats_get(cli_stats_t *out);

#endif /* CLI_H */
