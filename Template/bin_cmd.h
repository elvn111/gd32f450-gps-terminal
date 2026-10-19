/* 二进制命令通道：收帧(定帧/CRC/半帧超时重同步) + 命令分派 + 应答构造 + 事件上报。
  *
  * 协议见《时变TXP系列定位终端通信协议集 V1.0.0》第 2、3 章：
  *   A6 6A | 命令码 | 功能码 | 数据长度 N(高字节先发) | 数据(N) | CRC16/MODBUS(低字节在前) | B7 7B
  *   帧总长 = 10 + N；CRC 覆盖帧头起、数据段结束止(6 + N 字节)，不含帧尾。
  *
  * 设计要点：
  *   1) 本模块只做"字节进、字节出"，不依赖硬件：需要的东西全部由 bin_cmd_plat_t 注入，
  *      所以整份逻辑可以在 PC 上编译成回归测试(见 tests/test_bin_cmd.c)；
  *   2) 定帧必须靠 N，不能搜帧尾：数据段里可能出现 B7 7B；
  *   3) 半帧超时(100 ms)在"下一个字节到来时"判定并复位，因此状态机只被一个任务访问，
  *      不需要加锁；坏帧静默丢弃并计数，不回应答，避免坏帧引发应答风暴。 */
#ifndef BIN_CMD_H
#define BIN_CMD_H

#include <stdint.h>

/* 帧结构 */
#define BIN_HEAD0           (0xA6U)
#define BIN_HEAD1           (0x6AU)
#define BIN_TAIL0           (0xB7U)
#define BIN_TAIL1           (0x7BU)
#define BIN_ACK_BASE        (0x80U)     /* 应答命令码 = 请求命令码 | 0x80 */
#define BIN_UP_DATA         (0x90U)     /* 上行数据上报 */
#define BIN_UP_EVENT        (0xA0U)     /* 上行事件上报 */

/* 命令码：0x01~0x0F 是主机下发的命令集 */
#define BIN_CMD_SYS         (0x01U)
#define BIN_CMD_LINK        (0x02U)
#define BIN_CMD_GPS         (0x03U)

/* 0x01 系统信息 */
#define BIN_SYS_VER         (0x01U)
#define BIN_SYS_CFG_GET     (0x02U)
#define BIN_SYS_CFG_SET     (0x03U)
#define BIN_SYS_SAVE        (0x04U)
#define BIN_SYS_LOAD        (0x05U)
#define BIN_SYS_DEFAULT     (0x06U)
#define BIN_SYS_RESET       (0x07U)
#define BIN_SYS_CLRSTAT     (0x08U)
#define BIN_SYS_GETSTAT     (0x09U)

/* 0x02 数据链路 */
#define BIN_LINK_SET        (0x01U)
#define BIN_LINK_SENDNOW    (0x02U)
#define BIN_LINK_INFO       (0x03U)
#define BIN_LINK_LASTFIX    (0x04U)
#define BIN_LINK_BRIDGE     (0x05U)
#define BIN_LINK_DEFAULT    (0x06U)

/* 0x03 定位模块 */
#define BIN_GPS_PASSTHRU    (0x01U)
#define BIN_GPS_HWRESET     (0x02U)
#define BIN_GPS_PRESET      (0x03U)
#define BIN_GPS_FACTORY     (0x04U)

/* 0xA0 事件上报 */
#define BIN_EV_LOST         (0x01U)     /* 定位丢失 */
#define BIN_EV_NORESP       (0x02U)     /* 模块无响应 */
#define BIN_EV_CFGCHG       (0x03U)     /* 配置变更 */

/* 状态码(协议文档 3.1) */
#define BIN_ST_OK              (0x00U)
#define BIN_ST_UNKNOWN_FUNC    (0x01U)
#define BIN_ST_BAD_PARAM       (0x02U)
#define BIN_ST_BUSY            (0x03U)
#define BIN_ST_UNSUPPORTED     (0x04U)
#define BIN_ST_NEED_CONFIRM    (0x06U)
#define BIN_ST_WRITE_PROTECTED (0x07U)
#define BIN_ST_NOT_READY       (0x08U)
#define BIN_ST_CMD_CHANNEL_OFF (0x09U)

#define BIN_CONFIRM_WORD    (0x5AA5U)   /* 破坏性命令的确认字 */
#define BIN_TEXT_MAX        (64U)       /* 0x03/0x01 透传文本上限 */
#define BIN_FRAME_N_MAX     (256U)      /* 终端侧数据段上限(协议文档 3.7) */
#define BIN_HALF_FRAME_MS   (100U)      /* 半帧超时重同步 */

/* 链路统计快照(读 0x01/0x09 与 0x90/0x03 共用同一个口径) */
typedef struct
{
    uint32_t frames_ok;      /* 收下并执行完的帧数 */
    uint32_t frames_bad;     /* CRC/长度/帧尾错而丢弃的帧数 */
    uint32_t cmds_ok;        /* 返回 OK 的命令数 */
    uint32_t cmds_err;       /* 返回非 OK 状态的命令数 */
    uint8_t  last_status;    /* 最近一次应答的状态码 */
} bin_cmd_stats_t;

/* 平台钩子：由 main.c 注入，本模块不直接碰硬件 */
typedef struct
{
    int      (*gps_line)(const char *line);   /* 0x03/0x01 往定位模块发一行(内部补 CRLF)，0=ok */
    void     (*gps_hw_reset)(void);           /* 0x03/0x02 硬复位定位模块并重下预设 */
    void     (*gps_preset)(void);             /* 0x03/0x03 重发模块预设 */
    void     (*reset)(void);                  /* 0x01/0x07 软复位，不返回 */
    uint32_t (*uptime_ms)(void);              /* 上电毫秒(半帧超时与事件时间戳用) */
} bin_cmd_plat_t;

/* 初始化(帧状态机置空闲) */
void bin_cmd_init(const bin_cmd_plat_t *plat);

/* 喂入一个字节。
  * 返回 1：该字节属于二进制帧(已吃进状态机)；返回 0：不是帧字节，交给文本控制台。 */
uint8_t bin_cmd_feed(uint8_t byte);

void bin_cmd_stats_get(bin_cmd_stats_t *out);
void bin_cmd_stats_clear(void);

/* 事件上报：数据段 = 帧序号(2) + data(n)，n 为附加字节数(0/1/4) */
void bin_cmd_event(uint8_t func, const uint8_t *data, uint16_t n);

#endif /* BIN_CMD_H */
