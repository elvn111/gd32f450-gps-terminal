/* 二进制命令通道的实现。协议见 bin_cmd.h 顶部说明。
  * 本文件只写 ASCII 字符串(AC5 对字符串里的非 ASCII 会报 #870-D)。
  *
  * 数据流：USART0 RX 环形缓冲 -> RX_Task 逐字节喂 bin_cmd_feed()
  *         帧收全 -> 分派 -> 组应答帧 -> gps_link_send_frame() 从 USART0 发出。 */
#include <string.h>
#include "bin_cmd.h"
#include "cli.h"
#include "gps_link.h"
#include "version.h"

/* 收帧状态 */
enum
{
    RX_IDLE = 0,
    RX_HEAD1,
    RX_CMD,
    RX_FUNC,
    RX_LEN0,
    RX_LEN1,
    RX_DATA,
    RX_CRC0,
    RX_CRC1,
    RX_TAIL0,
    RX_TAIL1
};

/* 收帧缓冲：6 字节帧头区(帧头+命令码+功能码+长度) + 最多 256 字节数据段 */
static uint8_t  s_buf[6U + BIN_FRAME_N_MAX];

static struct
{
    uint8_t  state;
    uint16_t n;          /* 本帧数据段长度 */
    uint16_t idx;        /* 已收到的数据段字节数 */
    uint8_t  crc0;       /* 收到的 CRC 低字节 */
    uint32_t last_ms;    /* 上一个字节的时刻(半帧超时用) */
} s_rx;

static bin_cmd_plat_t s_plat;
static bin_cmd_stats_t s_stat;

/* ---------------- 小工具：大端读写(与协议文档一致) ---------------- */

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8U);
    p[1] = (uint8_t)(v & 0xFFU);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 24U) & 0xFFU);
    p[1] = (uint8_t)((v >> 16U) & 0xFFU);
    p[2] = (uint8_t)((v >> 8U) & 0xFFU);
    p[3] = (uint8_t)(v & 0xFFU);
}

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24U) | ((uint32_t)p[1] << 16U) |
           ((uint32_t)p[2] << 8U) | (uint32_t)p[3];
}

/* ---------------- 应答构造 ---------------- */

/* 回一帧应答：数据段 = 状态(1) + 回带序号(1) + 附加数据 */
static void reply(uint8_t cmd, uint8_t func, uint8_t status, uint8_t seq,
                  const uint8_t *extra, uint16_t exlen)
{
    uint8_t  out[2U + 64U];
    uint16_t n = 2U;

    if (exlen > (uint16_t)sizeof(out) - 2U) {
        exlen = 0U;
    }
    out[0] = status;
    out[1] = seq;
    if ((0U != exlen) && (NULL != extra)) {
        memcpy(&out[2], extra, exlen);
        n = (uint16_t)(n + exlen);
    }
    (void)gps_link_send_frame((uint8_t)(cmd | BIN_ACK_BASE), func, out, n);

    s_stat.last_status = status;
    if (BIN_ST_OK == status) {
        s_stat.cmds_ok++;
    } else {
        s_stat.cmds_err++;
    }
}

/* 26 字节统计量(协议文档 表 14)，0x01/0x09 与 0x90/0x03 共用 */
static void build_stats(uint8_t out[26])
{
    cli_stats_t      st;
    gps_link_stats_t ls;

    memset(&st, 0, sizeof(st));
    memset(&ls, 0, sizeof(ls));
    cli_stats_get(&st);
    gps_link_stats_get(&ls);

    put_u32(&out[0], st.rx_bytes);                 /* 接收字节数 */
    put_u16(&out[4], (uint16_t)st.rx_frames);      /* 接收帧数 */
    put_u16(&out[6], (uint16_t)st.cmds);           /* 有效命令数 */
    put_u16(&out[8], (uint16_t)st.unknown);        /* 未知命令数 */
    put_u32(&out[10], st.gps_ok);                  /* 解析成功语句数 */
    put_u32(&out[14], st.gps_err);                 /* 丢弃语句数 */
    put_u32(&out[18], ls.tx_frames);               /* 已发帧数 */
    put_u16(&out[22], (uint16_t)ls.tx_err);        /* 发送失败次数 */
    put_u16(&out[24], (uint16_t)ls.drop_latest);   /* 覆盖丢弃数 */
}

/* ---------------- 事件上报 ---------------- */

void bin_cmd_event(uint8_t func, const uint8_t *data, uint16_t n)
{
    uint8_t          out[2U + 8U];
    uint16_t         i = 0U;
    gps_link_stats_t ls;
    uint32_t         up;

    memset(&ls, 0, sizeof(ls));
    gps_link_stats_get(&ls);
    up = (NULL != s_plat.uptime_ms) ? s_plat.uptime_ms() : 0U;

    put_u16(&out[0], ls.seq);           /* 事件帧数据段前 2 字节 = 帧序号 */
    i = 2U;
    switch (func) {
    case BIN_EV_LOST:
    case BIN_EV_NORESP:
        put_u32(&out[i], up);           /* 上电毫秒数 */
        i = (uint16_t)(i + 4U);
        break;
    case BIN_EV_CFGCHG:
        if ((NULL != data) && (n >= 5U)) {
            memcpy(&out[i], data, 5U);  /* 配置项 ID(1) + 新值(4) */
            i = (uint16_t)(i + 5U);
        }
        break;
    default:
        return;
    }
    (void)gps_link_send_frame(BIN_UP_EVENT, func, out, i);
}

/* ---------------- 命令处理 ---------------- */

static void cmd_sys(uint8_t func, const uint8_t *d, uint16_t n, uint8_t seq)
{
    uint8_t  v[64];
    uint32_t val = 0U;
    int      rc;

    switch (func) {
    case BIN_SYS_VER:                       /* 0x01/0x01 读取版本 */
        put_u32(v, FW_VERSION_PACKED);
        reply(BIN_CMD_SYS, func, BIN_ST_OK, seq, v, 4U);
        break;

    case BIN_SYS_CFG_GET:                   /* 0x01/0x02 读取配置项 */
        if (n < 2U) {
            reply(BIN_CMD_SYS, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        if (0 != cli_config_item_get(d[1], &val)) {
            reply(BIN_CMD_SYS, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        put_u32(v, val);
        reply(BIN_CMD_SYS, func, BIN_ST_OK, seq, v, 4U);
        break;

    case BIN_SYS_CFG_SET:                   /* 0x01/0x03 设置配置项 */
        if (n < 6U) {
            reply(BIN_CMD_SYS, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        rc = cli_config_item_set(d[1], get_u32(&d[2]));
        /* 配置变更事件 0xA0/0x03 由 main.c 的快照比对统一上报（文本 set 也能覆盖），这里不重复发 */
        reply(BIN_CMD_SYS, func, (0 == rc) ? BIN_ST_OK : BIN_ST_BAD_PARAM, seq, NULL, 0U);
        break;

    case BIN_SYS_SAVE:                      /* 0x01/0x04 保存配置 */
        rc = cli_config_save();
        reply(BIN_CMD_SYS, func, (0 == rc) ? BIN_ST_OK : BIN_ST_UNSUPPORTED, seq, NULL, 0U);
        break;

    case BIN_SYS_LOAD:                      /* 0x01/0x05 读取已保存配置 */
        rc = cli_config_load();
        reply(BIN_CMD_SYS, func, (0 == rc) ? BIN_ST_OK : BIN_ST_NOT_READY, seq, NULL, 0U);
        break;

    case BIN_SYS_DEFAULT:                   /* 0x01/0x06 恢复默认配置(需确认字) */
        if ((n < 3U) || (((uint16_t)d[1] << 8U | d[2]) != BIN_CONFIRM_WORD)) {
            reply(BIN_CMD_SYS, func, BIN_ST_NEED_CONFIRM, seq, NULL, 0U);
            break;
        }
        cli_config_default();
        reply(BIN_CMD_SYS, func, BIN_ST_OK, seq, NULL, 0U);
        break;

    case BIN_SYS_RESET:                     /* 0x01/0x07 系统重启(需确认字，无应答) */
        if ((n < 3U) || (((uint16_t)d[1] << 8U | d[2]) != BIN_CONFIRM_WORD)) {
            reply(BIN_CMD_SYS, func, BIN_ST_NEED_CONFIRM, seq, NULL, 0U);
            break;
        }
        if (NULL != s_plat.reset) {
            s_plat.reset();                 /* 不返回 */
        }
        break;

    case BIN_SYS_CLRSTAT:                   /* 0x01/0x08 清除统计 */
        cli_stats_clear();
        bin_cmd_stats_clear();
        reply(BIN_CMD_SYS, func, BIN_ST_OK, seq, NULL, 0U);
        break;

    case BIN_SYS_GETSTAT:                   /* 0x01/0x09 读取统计 */
        build_stats(v);
        reply(BIN_CMD_SYS, func, BIN_ST_OK, seq, v, 26U);
        break;

    default:
        reply(BIN_CMD_SYS, func, BIN_ST_UNKNOWN_FUNC, seq, NULL, 0U);
        break;
    }
}

static void cmd_link(uint8_t func, const uint8_t *d, uint16_t n, uint8_t seq)
{
    uint8_t           v[26];
    gps_link_info_t   info;
    cli_stats_t       st;
    gps_link_stats_t  ls;
    const cli_config_t *cfg = cli_config_get();

    switch (func) {
    case BIN_LINK_SET:                      /* 0x02/0x01 设置下行参数 */
        if (n < 4U) {
            reply(BIN_CMD_LINK, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        if ((d[1] > 2U) || (d[2] > 2U) || (d[3] < 1U) || (d[3] > 20U)) {
            reply(BIN_CMD_LINK, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        (void)cli_config_item_set(6U, d[1]);     /* out  */
        (void)cli_config_item_set(7U, d[2]);     /* auto */
        (void)cli_config_item_set(8U, d[3]);     /* rate */
        reply(BIN_CMD_LINK, func, BIN_ST_OK, seq, NULL, 0U);
        break;

    case BIN_LINK_SENDNOW:                  /* 0x02/0x02 立即发送一帧 */
        reply(BIN_CMD_LINK, func, (0 == gps_link_send_now()) ? BIN_ST_OK : BIN_ST_NOT_READY,
              seq, NULL, 0U);
        break;

    case BIN_LINK_INFO:                     /* 0x02/0x03 发送终端信息帧 */
        memset(&info, 0, sizeof(info));
        memset(&st, 0, sizeof(st));
        memset(&ls, 0, sizeof(ls));
        cli_stats_get(&st);
        gps_link_stats_get(&ls);
        info.uptime_ms   = (NULL != s_plat.uptime_ms) ? s_plat.uptime_ms() : 0U;
        info.version     = FW_VERSION_PACKED;
        info.tx_frames   = ls.tx_frames;
        info.tx_err      = ls.tx_err;
        info.drop_latest = ls.drop_latest;
        info.gps_ok      = st.gps_ok;
        info.gps_err     = st.gps_err;
        info.out         = cfg->out;
        info.auto_mode   = cfg->auto_mode;
        info.rate        = (uint8_t)cfg->rate;
        info.sat_min     = cfg->sat_min;
        reply(BIN_CMD_LINK, func, BIN_ST_OK, seq, NULL, 0U);
        (void)gps_link_send_info(&info);
        break;

    case BIN_LINK_LASTFIX:                  /* 0x02/0x04 读取最近定位 */
        if (0U == gps_link_last_fix(v)) {
            reply(BIN_CMD_LINK, func, BIN_ST_NOT_READY, seq, NULL, 0U);
            break;
        }
        reply(BIN_CMD_LINK, func, BIN_ST_OK, seq, v, 24U);
        break;

    case BIN_LINK_BRIDGE:                   /* 0x02/0x05 桥模式开关 */
        if (n < 2U) {
            reply(BIN_CMD_LINK, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        if (0 != cli_bridge_switch((uint8_t)((0U != d[1]) ? 1U : 0U))) {
            reply(BIN_CMD_LINK, func, BIN_ST_UNSUPPORTED, seq, NULL, 0U);
            break;
        }
        reply(BIN_CMD_LINK, func, BIN_ST_OK, seq, NULL, 0U);
        break;

    case BIN_LINK_DEFAULT:                  /* 0x02/0x06 恢复默认发送 */
        (void)cli_config_item_set(6U, 2U);   /* out  = 二进制帧 */
        (void)cli_config_item_set(7U, 1U);   /* auto = 新值触发 */
        (void)cli_config_item_set(8U, 20U);  /* rate = 20 Hz */
        reply(BIN_CMD_LINK, func, BIN_ST_OK, seq, NULL, 0U);
        break;

    default:
        reply(BIN_CMD_LINK, func, BIN_ST_UNKNOWN_FUNC, seq, NULL, 0U);
        break;
    }
}

static void cmd_gps(uint8_t func, const uint8_t *d, uint16_t n, uint8_t seq)
{
    char text[BIN_TEXT_MAX + 1U];
    uint16_t i;
    uint8_t  bad = 0U;

    switch (func) {
    case BIN_GPS_PASSTHRU:                  /* 0x03/0x01 模块命令透传 */
        if ((n < 2U) || ((n - 1U) > BIN_TEXT_MAX)) {
            reply(BIN_CMD_GPS, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        for (i = 0U; i < (uint16_t)(n - 1U); i++) {
            text[i] = (char)d[1U + i];
            if ((text[i] < 0x20) || (text[i] > 0x7E)) {   /* 只收可打印 ASCII */
                bad = 1U;
            }
        }
        text[n - 1U] = '\0';
        if ((0U != bad) || (NULL == s_plat.gps_line)) {
            reply(BIN_CMD_GPS, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
            break;
        }
        (void)s_plat.gps_line(text);
        reply(BIN_CMD_GPS, func, BIN_ST_OK, seq, NULL, 0U);
        break;

    case BIN_GPS_HWRESET:                   /* 0x03/0x02 硬复位定位模块 */
        reply(BIN_CMD_GPS, func, BIN_ST_OK, seq, NULL, 0U);   /* 先应答，再做会阻塞的动作 */
        if (NULL != s_plat.gps_hw_reset) {
            s_plat.gps_hw_reset();
        }
        break;

    case BIN_GPS_PRESET:                    /* 0x03/0x03 重发模块预设 */
        reply(BIN_CMD_GPS, func, BIN_ST_OK, seq, NULL, 0U);
        if (NULL != s_plat.gps_preset) {
            s_plat.gps_preset();
        }
        break;

    case BIN_GPS_FACTORY:                   /* 0x03/0x04 模块工厂复位(需确认字) */
        if ((n < 3U) || (((uint16_t)d[1] << 8U | d[2]) != BIN_CONFIRM_WORD)) {
            reply(BIN_CMD_GPS, func, BIN_ST_NEED_CONFIRM, seq, NULL, 0U);
            break;
        }
        reply(BIN_CMD_GPS, func, BIN_ST_OK, seq, NULL, 0U);
        if (NULL != s_plat.gps_line) {
            (void)s_plat.gps_line("freset");
        }
        break;

    default:
        reply(BIN_CMD_GPS, func, BIN_ST_UNKNOWN_FUNC, seq, NULL, 0U);
        break;
    }
}

/* 收全一帧后分派 */
static void dispatch(uint8_t cmd, uint8_t func, const uint8_t *d, uint16_t n)
{
    uint8_t seq = (n > 0U) ? d[0] : 0U;     /* 请求数据段第 1 字节是序号，原样回带 */

    s_stat.frames_ok++;

    if (0U == n) {                          /* 协议要求至少带序号 */
        reply(cmd, func, BIN_ST_BAD_PARAM, seq, NULL, 0U);
        return;
    }
    switch (cmd) {
    case BIN_CMD_SYS:
        cmd_sys(func, d, n, seq);
        break;
    case BIN_CMD_LINK:
        cmd_link(func, d, n, seq);
        break;
    case BIN_CMD_GPS:
        cmd_gps(func, d, n, seq);
        break;
    default:                                /* 未知命令集：按协议文档 E4 的写法回 0x81/功能码 */
        reply(BIN_CMD_SYS, func, BIN_ST_UNKNOWN_FUNC, seq, NULL, 0U);
        break;
    }
}

/* ---------------- 收帧状态机 ---------------- */

void bin_cmd_init(const bin_cmd_plat_t *plat)
{
    if (NULL != plat) {
        s_plat = *plat;
    }
    s_rx.state  = RX_IDLE;
    s_rx.n      = 0U;
    s_rx.idx    = 0U;
    s_rx.last_ms = 0U;
}

void bin_cmd_stats_get(bin_cmd_stats_t *out)
{
    if (NULL != out) {
        *out = s_stat;
    }
}

void bin_cmd_stats_clear(void)
{
    memset(&s_stat, 0, sizeof(s_stat));
}

uint8_t bin_cmd_feed(uint8_t byte)
{
    uint32_t now = (NULL != s_plat.uptime_ms) ? s_plat.uptime_ms() : 0U;

    /* 半帧超时：上一个字节距现在超过 100 ms，说明上一帧没收全，复位状态机 */
    if ((RX_IDLE != s_rx.state) && ((uint32_t)(now - s_rx.last_ms) > BIN_HALF_FRAME_MS)) {
        s_rx.state = RX_IDLE;
    }
    s_rx.last_ms = now;

    switch (s_rx.state) {
    case RX_IDLE:
        if (BIN_HEAD0 != byte) {
            return 0U;                      /* 不是帧，交给文本控制台 */
        }
        s_buf[0]   = byte;
        s_rx.state = RX_HEAD1;
        return 1U;

    case RX_HEAD1:
        if (BIN_HEAD1 == byte) {
            s_buf[1]   = byte;
            s_rx.state = RX_CMD;
            return 1U;
        }
        s_rx.state = (BIN_HEAD0 == byte) ? RX_HEAD1 : RX_IDLE;   /* 0xA6 0xA6 重新开始 */
        if (BIN_HEAD0 == byte) {
            s_buf[0] = byte;
            return 1U;
        }
        return 0U;

    case RX_CMD:
        s_buf[2]   = byte;
        s_rx.state = RX_FUNC;
        return 1U;

    case RX_FUNC:
        s_buf[3]   = byte;
        s_rx.state = RX_LEN0;
        return 1U;

    case RX_LEN0:
        s_buf[4]   = byte;
        s_rx.state = RX_LEN1;
        return 1U;

    case RX_LEN1:
        s_buf[5] = byte;
        s_rx.n   = (uint16_t)(((uint16_t)s_buf[4] << 8U) | (uint16_t)byte);
        s_rx.idx = 0U;
        if (s_rx.n > (uint16_t)BIN_FRAME_N_MAX) {
            s_stat.frames_bad++;
            s_rx.state = RX_IDLE;
            return 1U;
        }
        s_rx.state = (0U == s_rx.n) ? RX_CRC0 : RX_DATA;
        return 1U;

    case RX_DATA:
        s_buf[6U + s_rx.idx] = byte;
        s_rx.idx++;
        if (s_rx.idx >= s_rx.n) {
            s_rx.state = RX_CRC0;
        }
        return 1U;

    case RX_CRC0:
        s_rx.crc0  = byte;                  /* CRC 低字节在前 */
        s_rx.state = RX_CRC1;
        return 1U;

    case RX_CRC1: {
        uint16_t crc_rx = (uint16_t)((uint16_t)s_rx.crc0 | ((uint16_t)byte << 8U));
        uint16_t crc_calc = gps_link_crc16(s_buf, (uint16_t)(6U + s_rx.n));

        if (crc_rx != crc_calc) {
            s_stat.frames_bad++;
            s_rx.state = RX_IDLE;
            return 1U;
        }
        s_rx.state = RX_TAIL0;
        return 1U;
    }

    case RX_TAIL0:
        if (BIN_TAIL0 != byte) {
            s_stat.frames_bad++;
            s_rx.state = RX_IDLE;
            return 1U;
        }
        s_rx.state = RX_TAIL1;
        return 1U;

    case RX_TAIL1:
        if (BIN_TAIL1 != byte) {
            s_stat.frames_bad++;
            s_rx.state = RX_IDLE;
            return 1U;
        }
        s_rx.state = RX_IDLE;
        dispatch(s_buf[2], s_buf[3], &s_buf[6], s_rx.n);
        return 1U;

    default:
        s_rx.state = RX_IDLE;
        return 0U;
    }
}
