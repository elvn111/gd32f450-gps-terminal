/* USART0 控制台命令解释器：表驱动，C99，本文件的字符串一律 ASCII。
  * 数据流：USART0 RX --DMA(CH5)--> cli_input() --> 行缓冲 --> 分词
  *         --> 命令表 --> 处理函数 --> printf / 平台钩子。
  * 三条入口互不干扰：普通命令行、'@' 前缀转给 GPS 的行、桥模式的纯字节转发。
  * 加命令：写一个 cmd_xxx()，再往 s_cmds[] 加一行；加配置项：往 s_items[] 加一行，
  * get/set/范围检查/help 会自动跟上。 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cli.h"
#include "version.h"

/* 版本来自 version.h：文本界面用字符串，帧内用数字，两处不会再不一致 */
#define CLI_VERSION   FW_VERSION_STRING

/* printf 不可重入：本模块只在 RX_Task 里跑，并且整段输出都在控制台锁内，
  * 所以同一个时刻只有一个任务在打印。 */

/* 内部状态 */

static cli_config_t   s_cfg;
static cli_platform_t s_plat;
static char           s_line[CLI_LINE_MAX];
static uint16_t       s_len;
static uint8_t        s_last_cr;
static uint8_t        s_overflow;
static char           s_tx[CLI_LINE_MAX];          /* GPS 行解析用的暂存区 */

/* 输入转义状态机：0=普通，1=收到 ESC，2=收到 ESC '[' */
static uint8_t        s_esc;
/* 历史环形缓冲：s_hist_head 指向最新一条的下一个位置 */
static char           s_hist[CLI_HIST_MAX][CLI_LINE_MAX];
static uint8_t        s_hist_count;
static uint8_t        s_hist_head;
static int8_t         s_hist_idx;                  /* -1=正在编辑新行(历史还没生效) */

/* 统计：喂入字节数、执行行数、未知命令数 */
static uint32_t s_rx_frames, s_rx_bytes, s_cmd_count, s_err_unknown;
static uint32_t s_gps_ok, s_gps_err;
/* 桥状态：故意不存进配置。若存成“开”，板子上电就变成一根哑管道，
  * 既没有提示符也没有命令，用户会以为板子坏了。 */
static uint8_t  s_bridge;
static uint8_t  s_bridge_prev_out;
static uint8_t  s_bridge_prev_verbose;
static uint8_t  s_bridge_prev_echo;
static uint8_t  s_bridge_prev_auto;
static uint16_t s_bridge_prev_rate;
static uint8_t  s_plus_run;           /* 控制台连续收到的 '+' 个数 */
static uint32_t s_bridge_last_input;   /* 最后一个转发字节的时刻(ms)，用来算静默 */

#define BRIDGE_PLUS_SILENCE_MS (1000U)   /* 识别 '+++' 所需的静默时间 */

static const cli_config_t s_cfg_default = {
    /* 控制台输出形态 */ 1U,        /* WORKED=在控制台输出 6 行解析结果 */
    /* 回显 */ 1U,
    /* 逐句打印(0 可让控制台只剩报告) */ 1U,
    /* 判定定位有效所需最少星数 */ 4U,
    /* GPS 串口波特率 */ 115200U,
    /* 下行内容 */ 0U,        /* 下行关闭：这条线保持成干净的控制台 */
    /* 下行触发方式 */ 0U,
    /* 下行频率(Hz) */ 1U,
    /* 上行前缀字符 */ 0x40U,     /* '@' 后面的整行内容转给 GPS 模块 */
};

/* 输出锁：与其它打印任务共用，包住整段输出 */

void cli_out_lock(void)
{
    if (s_plat.out_lock) {
        s_plat.out_lock();
    }
}

void cli_out_unlock(void)
{
    if (s_plat.out_unlock) {
        s_plat.out_unlock();
    }
}

/* 小工具函数 */

static void cli_prompt(void)
{
    if (s_cfg.echo) {
        printf("> ");
    }
}

/* 把刚写入的配置项同步到硬件。item==NULL 表示全部应用(load/default 用)。
  * 只有真正要动硬件的项才处理：mode 切输出形态，gps_baud 重配 USART5；
  * 其余项在下次使用时自然生效，不必每次写配置都动串口。 */
static void cfg_apply(const char *item)
{
    if ((NULL == item) || (0 == strcmp(item, "mode"))) {
        if (s_plat.output_mode_set) {
            s_plat.output_mode_set(s_cfg.mode);
        }
    }
    if ((NULL == item) || (0 == strcmp(item, "gps_baud"))) {
        if (s_plat.gps_baud_set) {
            s_plat.gps_baud_set(s_cfg.gps_baud);
        }
    }
}

static int s_tx_reset(void)
{
    s_tx[0] = '\0';
    return 0;
}

static int s_tx_add(const char *s)
{
    size_t n = strlen(s_tx);
    size_t l = strlen(s);

    if ((n + l + 1U) > sizeof(s_tx)) {
        return -1;
    }
    memcpy(&s_tx[n], s, l + 1U);
    return 0;
}

static int cli_tokenize(char *s, char **argv, int max)
{
    int   n   = 0;
    char *p   = s;
    int   inq = 0;

    while ('\0' != *p) {
        while ((' ' == *p) || ('\t' == *p)) {
            p++;
        }
        if ('\0' == *p) {
            break;
        }
        if (n >= max) {
            break;
        }
        argv[n++] = p;

        while ('\0' != *p) {
            if ('"' == *p) {
                memmove(p, p + 1, strlen(p));    /* 去掉引号 */
                inq = !inq;
                continue;
            }
            if ((0 == inq) && ((' ' == *p) || ('\t' == *p))) {
                break;
            }
            p++;
        }
        if ('\0' != *p) {
            *p = '\0';
            p++;
        }
    }
    return n;
}

static int parse_mode(const char *s, uint32_t *out)
{
    char *end = NULL;

    if (0 == strcmp(s, "raw")) {
        *out = 0U;
        return 0;
    }
    if (0 == strcmp(s, "worked")) {
        *out = 1U;
        return 0;
    }
    *out = (uint32_t)strtoul(s, &end, 0);
    if ((NULL == end) || (end == s) || ('\0' != *end)) {
        return -1;
    }
    return 0;
}

/* 解析无符号整数：整个 token 必须都是数字 */
static int parse_u32(const char *s, uint32_t *out)
{
    char *end = NULL;

    *out = (uint32_t)strtoul(s, &end, 0);
    if ((NULL == end) || (end == s) || ('\0' != *end)) {
        return -1;
    }
    return 0;
}

/* 配置项表 */

typedef enum {
    CFG_U8 = 0,
    CFG_U16,
    CFG_U32
} cfg_type_t;

typedef struct {
    const char *name;
    cfg_type_t  type;
    void       *ptr;
    uint32_t    min;
    uint32_t    max;
    const char *help;
} cfg_item_t;

static cfg_item_t s_items[] = {
    { "mode",     CFG_U8,  &s_cfg.mode,      0U,    1U,      "console: 0=RAW passthrough, 1=WORKED parsed" },
    { "echo",     CFG_U8,  &s_cfg.echo,      0U,    1U,      "1=echo input + prompt" },
    { "verbose",  CFG_U8,  &s_cfg.verbose,   0U,    1U,      "1=print every parsed GPS fix" },
    { "sat_min",  CFG_U8,  &s_cfg.sat_min,   0U,    40U,     "satellites needed for a valid fix" },
    { "gps_baud", CFG_U32, &s_cfg.gps_baud,  9600U, 921600U, "GPS (USART5) baud rate" },
    { "out",      CFG_U8,  &s_cfg.out,       0U,    2U,      "down-link: 0=off, 1=raw report, 2=parsed frame" },
    { "auto",     CFG_U8,  &s_cfg.auto_mode, 0U,    2U,      "down-link: 0=off, 1=on new fix, 2=heartbeat" },
    { "rate",     CFG_U16, &s_cfg.rate,      1U,    20U,     "down-link rate limit in Hz" },
    { "prefix",   CFG_U8,  &s_cfg.prefix,    0U,    126U,    "up-link: lines starting with this char go to the GPS (0=off)" },
};

#define CFG_ITEM_NUM (sizeof(s_items) / sizeof(s_items[0]))

static cfg_item_t *cfg_find(const char *name)
{
    uint32_t i;

    for (i = 0U; i < CFG_ITEM_NUM; i++) {
        if (0 == strcmp(s_items[i].name, name)) {
            return &s_items[i];
        }
    }
    return NULL;
}

static uint32_t cfg_read(const cfg_item_t *it)
{
    if (CFG_U8 == it->type) {
        return (uint32_t)(*(uint8_t *)it->ptr);
    }
    if (CFG_U16 == it->type) {
        return (uint32_t)(*(uint16_t *)it->ptr);
    }
    return *(uint32_t *)it->ptr;
}

static void cfg_write(cfg_item_t *it, uint32_t v)
{
    if (CFG_U8 == it->type) {
        *(uint8_t *)it->ptr = (uint8_t)v;
    } else if (CFG_U16 == it->type) {
        *(uint16_t *)it->ptr = (uint16_t)v;
    } else {
        *(uint32_t *)it->ptr = v;
    }
    cfg_apply(it->name);
}

/* 命令表类型与各处理函数声明 */

typedef struct {
    const char *name;
    const char *help;
    int       (*fn)(int argc, char **argv);
} cli_cmd_t;

static int cmd_help(int argc, char **argv);
static int cmd_ver(int argc, char **argv);
static int cmd_status(int argc, char **argv);
static int cmd_tasks(int argc, char **argv);
static int cmd_dma(int argc, char **argv);
static int cmd_get(int argc, char **argv);
static int cmd_set(int argc, char **argv);
static int cmd_mode(int argc, char **argv);
static int cmd_out(int argc, char **argv);
static int cmd_auto(int argc, char **argv);
static int cmd_fix(int argc, char **argv);
static int cmd_gstat(int argc, char **argv);
static int cmd_info(int argc, char **argv);
static int cmd_bridge(int argc, char **argv);
static int cmd_gps(int argc, char **argv);
static int cmd_save(int argc, char **argv);
static int cmd_load(int argc, char **argv);
static int cmd_default(int argc, char **argv);
static int cmd_clr(int argc, char **argv);
static int cmd_reboot(int argc, char **argv);

static const cli_cmd_t s_cmds[] = {
    { "help",    "help [cmd]           list commands / one command",          cmd_help },
    { "?",       "?                     same as help",                         cmd_help },
    { "ver",     "ver                   version / build time / uptime",        cmd_ver },
    { "status",  "status                config, counters, stored config",      cmd_status },
    { "tasks",   "tasks                 task list with stack high water mark", cmd_tasks },
    { "dma",     "dma                   USART0/USART5 RX DMA channel state",   cmd_dma },
    { "get",     "get [item]            show all or one config item",          cmd_get },
    { "set",     "set <item> <value>    change a config item (range checked)", cmd_set },
    { "mode",    "mode raw|worked       shortcut for the console output mode",  cmd_mode },
    { "out",     "out off|raw|parsed   down-link content: off / raw report / parsed frame", cmd_out },
    { "auto",    "auto off|fix|tick [hz]  down-link trigger and rate limit",    cmd_auto },
    { "fix",     "fix                   send the newest value once, right now", cmd_fix },
    { "gstat",   "gstat                 console + down-link statistics",        cmd_gstat },
    { "info",    "info                  send the terminal info frame (TYPE=0x02)", cmd_info },
    { "bridge",  "bridge on|off         transparent pipe GPS<->port, commands off", cmd_bridge },
    { "gps",     "gps <cmd...>          GPS passthrough / log / init / reset / info", cmd_gps },
    { "save",    "save                  store config to flash",                cmd_save },
    { "load",    "load                  restore config from flash",            cmd_load },
    { "default", "default               restore default config (RAM only)",    cmd_default },
    { "clr",     "clr                   clear the counters",                   cmd_clr },
    { "reboot",  "reboot                software reset",                       cmd_reboot },
};

#define CMD_NUM (sizeof(s_cmds) / sizeof(s_cmds[0]))

/* 命令处理函数 */

static int cmd_help(int argc, char **argv)
{
    uint32_t i;

    if (argc >= 2) {
        for (i = 0U; i < CMD_NUM; i++) {
            if (0 == strcmp(s_cmds[i].name, argv[1])) {
                printf("%s\r\n", s_cmds[i].help);
                return 0;
            }
        }
        for (i = 0U; i < CFG_ITEM_NUM; i++) {
            if (0 == strcmp(s_items[i].name, argv[1])) {
                printf("config '%s' [%lu..%lu] %s\r\n", s_items[i].name,
                       (unsigned long)s_items[i].min, (unsigned long)s_items[i].max,
                       s_items[i].help);
                return 0;
            }
        }
        printf("[cli] no such command or item '%s'\r\n", argv[1]);
        return -1;
    }

    printf("[cli] GD32F450 GPS console v%s\r\n", CLI_VERSION);
    printf("[cli] commands:\r\n");
    for (i = 0U; i < CMD_NUM; i++) {
        printf("  %s\r\n", s_cmds[i].help);
    }
    printf("[cli] config items: ");
    for (i = 0U; i < CFG_ITEM_NUM; i++) {
        printf("%s ", s_items[i].name);
    }
    printf("\r\n");
    printf("[cli] keys: TAB completes, Up/Down recalls history, Ctrl+C drops the\r\n");
    if (0U != cli_config_get()->prefix) {
        printf("[cli] up-link: a line starting with '%c' is forwarded to the GPS\r\n",
               (int)cli_config_get()->prefix);
    }
    printf("      line, Ctrl+L clears the screen, BS edits\r\n");
    return 0;
}

static int cmd_ver(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("[cli] version %s, built %s %s\r\n", CLI_VERSION, __DATE__, __TIME__);
    if (s_plat.uptime_ms) {
        printf("[cli] uptime %lu ms\r\n", (unsigned long)s_plat.uptime_ms());
    }
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("[cfg] mode=%s echo=%u verbose=%u sat_min=%u gps_baud=%lu\r\n",
           s_cfg.mode ? "WORKED" : "RAW",
           (unsigned)s_cfg.echo, (unsigned)s_cfg.verbose,
           (unsigned)s_cfg.sat_min, (unsigned long)s_cfg.gps_baud);
    printf("[rx ] frames=%lu bytes=%lu\r\n",
           (unsigned long)s_rx_frames, (unsigned long)s_rx_bytes);
    printf("[cli] cmds=%lu unknown=%lu\r\n",
           (unsigned long)s_cmd_count, (unsigned long)s_err_unknown);
    printf("[gps] parsed=%lu dropped=%lu\r\n",
           (unsigned long)s_gps_ok, (unsigned long)s_gps_err);
    if (s_plat.usart0_rx_dma_left) {
        printf("[dma] usart0 rx left=%lu\r\n",
               (unsigned long)s_plat.usart0_rx_dma_left());
    }
    if (s_plat.nvm_info) {
        s_plat.nvm_info();
    }
    return 0;
}

static int cmd_tasks(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (NULL == s_plat.tasks_info) {
        printf("[cli] tasks: hook not implemented\r\n");
        return -1;
    }
    s_plat.tasks_info();
    return 0;
}

static int cmd_dma(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (NULL == s_plat.dma_info) {
        printf("[cli] dma: hook not implemented\r\n");
        return -1;
    }
    s_plat.dma_info();
    return 0;
}

static int cmd_get(int argc, char **argv)
{
    uint32_t i;

    if (argc >= 2) {
        cfg_item_t *it = cfg_find(argv[1]);
        if (NULL == it) {
            printf("[cli] no such item '%s'\r\n", argv[1]);
            return -1;
        }
        printf("%s = %lu  (%s)\r\n", it->name,
               (unsigned long)cfg_read(it), it->help);
        return 0;
    }
    for (i = 0U; i < CFG_ITEM_NUM; i++) {
        printf("%-9s = %-7lu %s\r\n", s_items[i].name,
               (unsigned long)cfg_read(&s_items[i]), s_items[i].help);
    }
    return 0;
}

static int cmd_set(int argc, char **argv)
{
    cfg_item_t *it;
    uint32_t    v = 0U;

    if (argc < 3) {
        printf("[cli] usage: set <item> <value>   ('get' lists every item)\r\n");
        return -1;
    }
    it = cfg_find(argv[1]);
    if (NULL == it) {
        printf("[cli] no such item '%s'\r\n", argv[1]);
        return -1;
    }
    if (0 == strcmp(it->name, "mode")) {
        if (0 != parse_mode(argv[2], &v)) {
            printf("[cli] usage: set mode raw|worked|0|1\r\n");
            return -1;
        }
    } else if (0 == strcmp(it->name, "prefix")) {
        if (1U == strlen(argv[2])) {            /* 单字符 */
            v = (uint32_t)(uint8_t)argv[2][0];
        } else if (0 != parse_u32(argv[2], &v)) {
            printf("[cli] usage: set prefix <one char>|0..126\r\n");
            return -1;
        }
        if ((0U != v) && ((v < 0x21U) || (v > 0x7EU))) {
            printf("[cli] prefix must be a printable character or 0 (off)\r\n");
            return -1;
        }
    } else if (0 != parse_u32(argv[2], &v)) {
        printf("[cli] '%s' is not a number\r\n", argv[2]);
        return -1;
    }
    if ((v < it->min) || (v > it->max)) {
        printf("[cli] %s out of range [%lu..%lu]\r\n", it->name,
               (unsigned long)it->min, (unsigned long)it->max);
        return -1;
    }
    cfg_write(it, v);
    printf("%s = %lu%s\r\n", it->name, (unsigned long)cfg_read(it),
           (0 == strcmp(it->name, "mode")) ? (v ? " (WORKED)" : " (RAW)") : "");
    return 0;
}

static int cmd_mode(int argc, char **argv)
{
    cfg_item_t *it = cfg_find("mode");
    uint32_t    v  = 0U;

    if ((argc != 2) || (0 != parse_mode(argv[1], &v)) || (v > 1U)) {
        printf("[cli] usage: mode raw|worked\r\n");
        return -1;
    }
    cfg_write(it, v);
    printf("mode = %s\r\n", v ? "WORKED" : "RAW");
    return 0;
}

/* ---- 下行内容 ---- */
static int cmd_out(int argc, char **argv)
{
    cfg_item_t *it = cfg_find("out");
    uint32_t    v  = 0U;

    if (argc != 2) {
        printf("[cli] usage: out off|raw|parsed\r\n");
        return -1;
    }
    if (0 == strcmp(argv[1], "off")) {
        v = 0U;
    } else if (0 == strcmp(argv[1], "raw")) {
        v = 1U;
    } else if (0 == strcmp(argv[1], "parsed")) {
        v = 2U;
    } else if ((0 != parse_u32(argv[1], &v)) || (v > 2U)) {
        printf("[cli] usage: out off|raw|parsed\r\n");
        return -1;
    }
    cfg_write(it, v);
    printf("out = %s%s\r\n", (0U == v) ? "off" : ((1U == v) ? "raw" : "parsed"),
           (v && (0U == cli_config_get()->auto_mode))
               ? " (use 'auto fix' or 'fix' to actually send)" : "");
    return 0;
}

/* ---- 下行触发 ---- */
static int cmd_auto(int argc, char **argv)
{
    uint32_t v = 0U;

    if (argc < 2) {
        printf("[cli] usage: auto off|fix|tick [rate_hz]\r\n");
        return -1;
    }
    if (0 == strcmp(argv[1], "off")) {
        v = 0U;
    } else if (0 == strcmp(argv[1], "fix")) {
        v = 1U;
    } else if (0 == strcmp(argv[1], "tick")) {
        v = 2U;
    } else if ((0 != parse_u32(argv[1], &v)) || (v > 2U)) {
        printf("[cli] usage: auto off|fix|tick [rate_hz]\r\n");
        return -1;
    }
    cfg_write(cfg_find("auto"), v);

    if (argc >= 3) {                        /* 同一行可以带频率参数 */
        uint32_t r = 0U;
        if ((0 != parse_u32(argv[2], &r)) || (r < 1U) || (r > 20U)) {
            printf("[cli] rate out of range [1..20]\r\n");
            return -1;
        }
        cfg_write(cfg_find("rate"), r);
    }
    printf("auto = %s, rate = %lu Hz\r\n",
           (0U == v) ? "off" : ((1U == v) ? "fix" : "tick"),
           (unsigned long)cli_config_get()->rate);
    return 0;
}

/* ---- 立刻发一次 ---- */
static int cmd_fix(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (NULL == s_plat.send_now) {
        printf("[cli] fix: hook not implemented\r\n");
        return -1;
    }
    s_plat.send_now();
    return 0;
}

/* ---- 统计 ---- */
static int cmd_gstat(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("[cli ] rx frames=%lu bytes=%lu cmds=%lu unknown=%lu\r\n",
           (unsigned long)s_rx_frames, (unsigned long)s_rx_bytes,
           (unsigned long)s_cmd_count, (unsigned long)s_err_unknown);
    printf("[gps ] parsed=%lu dropped=%lu\r\n",
           (unsigned long)s_gps_ok, (unsigned long)s_gps_err);
    if (NULL == s_plat.link_info) {
        printf("[link] hook not implemented\r\n");
        return -1;
    }
    s_plat.link_info();
    return 0;
}

/* 退出桥模式：恢复保存的配置，把控制台还给用户。
  * 'bridge off'、'+++' 转义、重启前的清理都走这里。 */
static void cli_bridge_exit(const char *why)
{
    if (0U == s_bridge) {
        return;
    }
    s_bridge   = 0U;
    s_len      = 0U;
    s_overflow = 0U;
    s_hist_idx = -1;
    s_plus_run = 0U;

    cfg_write(cfg_find("out"),     s_bridge_prev_out);
    cfg_write(cfg_find("verbose"), s_bridge_prev_verbose);
    cfg_write(cfg_find("echo"),    s_bridge_prev_echo);
    cfg_write(cfg_find("auto"),    s_bridge_prev_auto);
    cfg_write(cfg_find("rate"),    s_bridge_prev_rate);

    /* 控制台锁已由调用者持有：cli_input() 整段持锁，cli_dispatch()/cli_exec_line() 都在锁内，
      * 所以这里只恢复状态，绝不加锁——互斥量不可重入，再加锁会把 RX_Task 连同整个打印路径一起锁死。 */
    printf("\r\n[cli] bridge off (%s), console back\r\n", why);
    cli_prompt();
}

/* ---- 上行桥：控制台字节直接送进 GPS 模块 ---- */
static int cmd_bridge(int argc, char **argv)
{
    uint32_t on = 0U;

    if (argc != 2) {
        printf("[cli] usage: bridge on|off   (console input -> GPS module, verbatim)\r\n");
        return -1;
    }
    if (0 == strcmp(argv[1], "on")) {
        on = 1U;
    } else if (0 == strcmp(argv[1], "off")) {
        on = 0U;
    } else if ((0 != parse_u32(argv[1], &on)) || (on > 1U)) {
        printf("[cli] usage: bridge on|off\r\n");
        return -1;
    }
    if ((0U != on) && ((NULL == s_plat.gps_send_raw) || (NULL == s_plat.uptime_ms))) {
        printf("[cli] bridge: hook missing (gps_send_raw / uptime_ms)\r\n");
        return -1;
    }
    if (0U != on) {
        const cli_config_t *c = cli_config_get();

        /* 先记下桥要改动的配置，再把 MCU 变成一根管道 */
        s_bridge_prev_out     = c->out;
        s_bridge_prev_verbose = c->verbose;
        s_bridge_prev_echo    = c->echo;
        s_bridge_prev_auto     = c->auto_mode;
        s_bridge_prev_rate     = c->rate;

        cfg_write(cfg_find("out"),     1U);     /* GPS 报文原样出去 */
        cfg_write(cfg_find("verbose"), 0U);     /* 总线上不要 [GPS_DATA] 报告 */
        cfg_write(cfg_find("echo"),    0U);     /* 不回显、不显示提示符：命令系统让位 */
        cfg_write(cfg_find("auto"),     1U);     /* 每个新值都发 */
        cfg_write(cfg_find("rate"),    20U);    /* 用最高频率：管道不该限流 */

        s_bridge_last_input = s_plat.uptime_ms();
        s_len      = 0U;
        s_overflow = 0U;
        s_plus_run = 0U;
        s_bridge   = 1U;

        printf("[cli] bridge ON: transparent pipe, GPS <-> this port\r\n");
        printf("[cli] commands are NOT accepted while bridging.\r\n");
        printf("[cli] to leave: send '+++' after >= 1 s of silence, or reset\r\n");
    } else {
        cli_bridge_exit("command");
    }
    return 0;
}


/* 把准备好的一行发给 GPS，并回显发出去的内容 */
static int gps_send(const char *line)
{
    if (NULL == s_plat.gps_send_line) {
        printf("[cli] gps: hook not implemented\r\n");
        return -1;
    }
    s_plat.gps_send_line(line);
    printf("[gps] -> %s\r\n", line);
    return 0;
}

static int cmd_info(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (NULL == s_plat.link_info_send) {
        printf("[cli] info: hook not implemented\r\n");
        return -1;
    }
    s_plat.link_info_send();
    return 0;
}

static int cmd_gps(int argc, char **argv)
{
    int i;

    if (argc < 2) {
        printf("[cli] usage:\r\n"
               "      gps <command...>        passthrough, e.g. gps log com1 gpnav ontime 1\r\n"
               "      gps log <what> <hz>     e.g. gps log gpnav 1  -> log com1 gpnav ontime 1\r\n"
               "      gps unlog               -> unlogall\r\n"
               "      gps save                -> saveconfig\r\n"
               "      gps ver                 -> version\r\n"
               "      gps scene <name>        -> set appscene <name>\r\n"
               "      gps baud <rate>         -> config com1 <rate>, then local USART5\r\n"
               "      gps init                -> re-send the boot preset list\r\n"
               "      gps hwreset             -> pull PB4 low: the module reboots\r\n"
               "      gps freset              -> module FACTORY reset (clears its config)\r\n"
               "      gps info                -> print the last parsed fix\r\n");
        return -1;
    }
    if (0 == strcmp(argv[1], "info")) {
        if (s_plat.gps_info_print) {
            s_plat.gps_info_print();
        } else {
            printf("[cli] gps info: hook not implemented\r\n");
        }
        return 0;
    }
    if (0 == strcmp(argv[1], "init")) {
        if (s_plat.gps_preset) {
            s_plat.gps_preset();
            printf("[gps] preset commands sent\r\n");
        } else {
            printf("[cli] gps init: hook not implemented\r\n");
        }
        return 0;
    }
    if (0 == strcmp(argv[1], "freset")) {
        printf("[gps] WARNING: freset is a module FACTORY reset\r\n");
        return gps_send("freset");
    }
    if (0 == strcmp(argv[1], "hwreset")) {
        if (NULL == s_plat.gps_hw_reset) {
            printf("[cli] gps hwreset: hook not implemented\r\n");
            return -1;
        }
        s_plat.gps_hw_reset();
        return 0;
    }
    if (0 == strcmp(argv[1], "reset")) {
        printf("[gps] 'gps reset' is gone: use 'gps hwreset' (PB4) or 'gps freset' (factory)\r\n");
        return -1;
    }
    if (0 == strcmp(argv[1], "unlog")) {
        return gps_send("unlogall");
    }
    if (0 == strcmp(argv[1], "save")) {
        return gps_send("saveconfig");
    }
    if (0 == strcmp(argv[1], "ver")) {
        return gps_send("version");
    }
    if ((argc >= 3) && (0 == strcmp(argv[1], "scene"))) {
        s_tx_reset();
        if ((0 != s_tx_add("set appscene ")) || (0 != s_tx_add(argv[2]))) {
            printf("[cli] command too long\r\n");
            return -1;
        }
        return gps_send(s_tx);
    }
    if ((argc >= 3) && (0 == strcmp(argv[1], "baud"))) {
        uint32_t rate;
        if (0 != parse_u32(argv[2], &rate)) {
            printf("[cli] usage: gps baud <9600..921600>\r\n");
            return -1;
        }
        s_tx_reset();
        if ((0 != s_tx_add("config com1 ")) || (0 != s_tx_add(argv[2]))) {
            printf("[cli] command too long\r\n");
            return -1;
        }
        (void)gps_send(s_tx);
        printf("[gps] switching local USART5 to %lu bit/s\r\n", (unsigned long)rate);
        cfg_write(cfg_find("gps_baud"), rate);
        printf("[gps] hint: 'gps save' on the module to make it permanent\r\n");
        return 0;
    }
    if ((argc >= 4) && (0 == strcmp(argv[1], "log"))) {
        s_tx_reset();
        if ((0 != s_tx_add("log com1 ")) || (0 != s_tx_add(argv[2])) ||
            (0 != s_tx_add(" ontime ")) || (0 != s_tx_add(argv[3]))) {
            printf("[cli] command too long\r\n");
            return -1;
        }
        return gps_send(s_tx);
    }

    /* 其它内容原样通过 */
    s_tx_reset();
    for (i = 1; i < argc; i++) {
        if ((i > 1) && (0 != s_tx_add(" "))) {
            printf("[cli] command too long\r\n");
            return -1;
        }
        if (0 != s_tx_add(argv[i])) {
            printf("[cli] command too long\r\n");
            return -1;
        }
    }
    return gps_send(s_tx);
}

static int cmd_save(int argc, char **argv)
{
    int rc;

    (void)argc;
    (void)argv;

    if (NULL == s_plat.config_save) {
        printf("[cli] save: not implemented (no non-volatile backend)\r\n");
        return -1;
    }
    printf("[cli] erasing/writing flash, CPU stalls for a few ms...\r\n");
    rc = s_plat.config_save(&s_cfg);
    if (0 != rc) {
        printf("[cli] save failed (rc=%d)\r\n", rc);
        return -1;
    }
    printf("[cli] config saved\r\n");
    if (s_plat.nvm_info) {
        s_plat.nvm_info();
    }
    return 0;
}

static int cmd_load(int argc, char **argv)
{
    int rc;

    (void)argc;
    (void)argv;

    if (NULL == s_plat.config_load) {
        printf("[cli] load: not implemented (no non-volatile backend)\r\n");
        return -1;
    }
    rc = s_plat.config_load(&s_cfg);
    if (0 != rc) {
        printf("[cli] nothing valid stored (rc=%d), keeping current settings\r\n", rc);
        return -1;
    }
    cfg_apply(NULL);
    printf("[cli] config loaded\r\n");
    (void)cmd_get(0, NULL);
    return 0;
}

static int cmd_default(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    s_cfg = s_cfg_default;
    cfg_apply(NULL);
    printf("[cli] default config restored (RAM only, use 'save' to persist)\r\n");
    return 0;
}

static int cmd_clr(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    s_rx_frames   = 0U;
    s_rx_bytes    = 0U;
    s_cmd_count   = 0U;
    s_err_unknown = 0U;
    s_gps_ok      = 0U;
    s_gps_err     = 0U;
    printf("[cli] counters cleared\r\n");
    return 0;
}

static int cmd_reboot(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (NULL == s_plat.system_reset) {
        printf("[cli] reboot: hook not implemented\r\n");
        return -1;
    }
    printf("[cli] rebooting...\r\n");
    s_plat.system_reset();
    return 0;   /* 不会走到这里 */
}

/* * 分发 */

/* 命令打错时，列出前两个字符相同的命令 */
static void suggest(const char *word)
{
    uint32_t i;
    int      hits = 0;

    if (strlen(word) < 2U) {
        return;
    }
    for (i = 0U; i < CMD_NUM; i++) {
        if (0 == strncmp(s_cmds[i].name, word, 2U)) {
            printf("%s%s", hits ? " " : "[cli] did you mean: ", s_cmds[i].name);
            hits++;
        }
    }
    if (hits) {
        printf("\r\n");
    }
}

static int cli_dispatch(char *line)
{
    char            *argv[CLI_ARG_MAX];
    const cli_cmd_t *hit = NULL;
    int              argc;
    int              n = 0;
    uint32_t         i;

    argc = cli_tokenize(line, argv, (int)CLI_ARG_MAX);
    if (0 == argc) {
        return 0;
    }

    for (i = 0U; i < CMD_NUM; i++) {
        if (0 == strcmp(s_cmds[i].name, argv[0])) {
            hit = &s_cmds[i];
            n   = 1;
            break;
        }
    }
    if (NULL == hit) {
        for (i = 0U; i < CMD_NUM; i++) {
            if (0 == strncmp(s_cmds[i].name, argv[0], strlen(argv[0]))) {
                hit = &s_cmds[i];
                n++;
            }
        }
    }
    if (NULL == hit) {
        s_err_unknown++;
        printf("[cli] unknown command '%s', try 'help'\r\n", argv[0]);
        suggest(argv[0]);
        return -1;
    }
    if (n > 1) {
        printf("[cli] ambiguous command '%s', try 'help'\r\n", argv[0]);
        return -1;
    }
    s_cmd_count++;
    return hit->fn(argc, argv);
}

void cli_exec_line(char *line)
{
    if (NULL == line) {
        return;
    }
    cli_out_lock();
    (void)cli_dispatch(line);
    cli_out_unlock();
}
/* 行编辑辅助 */

static void hist_push(const char *line)
{
    uint8_t last = (uint8_t)((s_hist_head + CLI_HIST_MAX - 1U) % CLI_HIST_MAX);

    if ((s_hist_count > 0U) && (0 == strcmp(s_hist[last], line))) {
        return;                              /* 紧挨着的重复行不记入历史 */
    }
    strncpy(s_hist[s_hist_head], line, CLI_LINE_MAX - 1U);
    s_hist[s_hist_head][CLI_LINE_MAX - 1U] = '\0';
    s_hist_head = (uint8_t)((s_hist_head + 1U) % CLI_HIST_MAX);
    if (s_hist_count < CLI_HIST_MAX) {
        s_hist_count++;
    }
}

/* 重画当前行(历史回调和补全后调用) */
static void line_redraw(const char *newline)
{
    uint16_t i = 0U;

    if (0U == s_cfg.echo) {
        return;
    }
    printf("\r");
    for (i = 0U; i < 78U; i++) {             /* 擦掉旧行 */
        printf(" ");
    }
    printf("\r> %s", newline);
    fflush(stdout);
}

static void hist_recall(int8_t idx)
{
    const char *txt;

    if ((s_hist_count == 0U) || (idx < 0)) {
        s_hist_idx = -1;
        s_len = 0U;
        line_redraw("");
        return;
    }
    if (idx >= (int8_t)s_hist_count) {
        return;
    }
    txt = s_hist[(s_hist_head + CLI_HIST_MAX - 1U - (uint8_t)idx) % CLI_HIST_MAX];
    strncpy(s_line, txt, CLI_LINE_MAX - 1U);
    s_line[CLI_LINE_MAX - 1U] = '\0';
    s_len = (uint16_t)strlen(s_line);
    s_hist_idx = idx;
    line_redraw(s_line);
}

/* TAB：补全命令名；get/set 之后补全配置项名 */
static void tab_complete(void)
{
    const char *last_space;
    const char *prefix;
    uint32_t    i;
    int         hits = 0;
    const char *cand = NULL;

    if (0U == s_cfg.echo) {
        return;
    }
    s_line[s_len] = '\0';
    last_space = strrchr(s_line, ' ');
    prefix = last_space ? (last_space + 1) : s_line;

    if (NULL == last_space) {
        for (i = 0U; i < CMD_NUM; i++) {
            if (0 == strncmp(s_cmds[i].name, prefix, strlen(prefix))) {
                cand = s_cmds[i].name;
                hits++;
            }
        }
    } else if ((0 == strncmp(s_line, "get ", 4U)) || (0 == strncmp(s_line, "set ", 4U)) ||
               (0 == strncmp(s_line, "help ", 5U))) {
        for (i = 0U; i < CFG_ITEM_NUM; i++) {
            if (0 == strncmp(s_items[i].name, prefix, strlen(prefix))) {
                cand = s_items[i].name;
                hits++;
            }
        }
    }

    if (1 == hits) {
        size_t keep = (size_t)(prefix - s_line);
        strncpy(&s_line[keep], cand, CLI_LINE_MAX - 1U - keep);
        s_line[CLI_LINE_MAX - 1U] = '\0';
        s_len = (uint16_t)strlen(s_line);
        if (s_len < (CLI_LINE_MAX - 1U)) {
            s_line[s_len++] = ' ';
            s_line[s_len] = '\0';
        }
        line_redraw(s_line);
    } else if (hits > 1) {
        printf("\r\n");
        if (NULL == last_space) {
            for (i = 0U; i < CMD_NUM; i++) {
                if (0 == strncmp(s_cmds[i].name, prefix, strlen(prefix))) {
                    printf("  %s", s_cmds[i].name);
                }
            }
        } else {
            for (i = 0U; i < CFG_ITEM_NUM; i++) {
                if (0 == strncmp(s_items[i].name, prefix, strlen(prefix))) {
                    printf("  %s", s_items[i].name);
                }
            }
        }
        printf("\r\n");
        line_redraw(s_line);
    }
}

/* 字节流 -> 行 */

void cli_input(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint8_t  c;

    if ((NULL == data) || (0U == len)) {
        return;
    }
    cli_out_lock();

    s_rx_frames++;
    s_rx_bytes += len;

    /* ---- 桥模式：纯管道，每个字节都进 GPS 模块 ---- */
    if (0U != s_bridge) {
        uint16_t k;
        uint32_t now = (NULL != s_plat.uptime_ms) ? s_plat.uptime_ms() : 0U;
        uint32_t gap = now - s_bridge_last_input;   /* 本段数据之前的静默时间 */

        /* 退出桥(Hayes 风格)：只有 '+++' 之前静默够长才认，
          * 否则报文里出现的 '+++' 会被误判成退出 */
        for (k = 0U; k < len; k++) {
            if ('+' == data[k]) {
                if ((0U == s_plus_run) && (gap >= BRIDGE_PLUS_SILENCE_MS)) {
                    s_plus_run = 1U;                /* 静默够长就置位 */
                } else if (0U != s_plus_run) {
                    s_plus_run++;
                }
                if (s_plus_run >= 3U) {
                    cli_bridge_exit("+++ escape");
                    cli_out_unlock();
                    return;
                }
            } else {
                s_plus_run = 0U;                    /* 其它任何字节都会清掉这个标志 */
            }
        }

        s_bridge_last_input = now;
        if (NULL != s_plat.gps_send_raw) {
            s_plat.gps_send_raw(data, len);
        }
        cli_out_unlock();
        return;
    }

    for (i = 0U; i < len; i++) {
        c = data[i];

        /* ---- 转义序列：ESC [ A/B 是历史，其它忽略 ---- */
        if (1U == s_esc) {
            if ('[' == c) {
                s_esc = 2U;
                continue;
            }
            s_esc = 0U;              /* 只有一个 ESC：当普通字符处理 */
        }
        if (2U == s_esc) {
            s_esc = 0U;
            if ((s_cfg.echo) && ('A' == c)) {          /* 上键：更早的命令 */
                hist_recall((int8_t)(s_hist_idx + 1));
            } else if ((s_cfg.echo) && ('B' == c)) {   /* 下键：更新的命令 */
                hist_recall((int8_t)(s_hist_idx - 1));
            }
            continue;                                  /* C/D/H/F：忽略 */
        }
        if (0x1BU == c) {
            s_esc = 1U;
            continue;
        }

        /* ---- 行结束：CRLF 只算一次 ---- */
        if (('\r' == c) || ('\n' == c)) {
            if (('\n' == c) && (0U != s_last_cr)) {
                s_last_cr = 0U;
                continue;
            }
            s_last_cr = (uint8_t)(('\r' == c) ? 1U : 0U);

            if (s_cfg.echo) {
                printf("\r\n");
            }
            if ((s_len > 0U) && (0U == s_overflow)) {
                s_line[s_len] = '\0';
                hist_push(s_line);
                if ((0U != s_cfg.prefix) && ((uint8_t)s_line[0] == s_cfg.prefix)) {
                    /* 前缀路由：本行剩余内容转给 GPS */
                    if (NULL != s_plat.gps_send_line) {
                        s_plat.gps_send_line(&s_line[1]);
                    } else {
                        printf("[cli] prefix: hook not implemented\r\n");
                    }
                } else {
                    (void)cli_dispatch(s_line);
                }
            }
            s_len      = 0U;
            s_overflow = 0U;
            s_hist_idx = -1;
            if (s_cfg.echo) {
                cli_prompt();
            }
            continue;
        }
        s_last_cr = 0U;

        /* ---- TAB：补全 ---- */
        if ('\t' == c) {
            tab_complete();
            continue;
        }

        /* ---- 退格 / DEL ---- */
        if ((0x08 == c) || (0x7F == c)) {
            if (s_len > 0U) {
                s_len--;
                if (s_cfg.echo) {
                    printf("\b \b");
                }
            }
            continue;
        }

        /* ---- Ctrl+C 丢弃本行，Ctrl+L 清屏 ---- */
        if (0x03 == c) {
            s_len      = 0U;
            s_overflow = 0U;
            s_hist_idx = -1;
            if (s_cfg.echo) {
                printf("^C\r\n");
                cli_prompt();
            }
            continue;
        }
        if (0x0C == c) {
            if (s_cfg.echo) {
                printf("\033[2J\033[H> ");
                printf("%s", s_line);
            }
            continue;
        }

        /* ---- 只接受可打印 ASCII ---- */
        if ((c < 0x20U) || (c > 0x7EU)) {
            continue;
        }
        if (s_len >= (CLI_LINE_MAX - 1U)) {
            if (0U == s_overflow) {
                s_overflow = 1U;
                printf("\r\n[cli] line too long (max %u), ignored\r\n",
                       (unsigned)(CLI_LINE_MAX - 1U));
            }
            continue;
        }
        s_line[s_len++] = (char)c;
        if (s_cfg.echo) {
            printf("%c", (int)c);
        }
    }
    cli_out_unlock();
}

/* 对外接口 */

const cli_config_t *cli_config_get(void)
{
    return &s_cfg;
}

void cli_stat_gps(int ok)
{
    if (ok) {
        s_gps_ok++;
    } else {
        s_gps_err++;
    }
}

void cli_stats_get(cli_stats_t *out)
{
    if (NULL == out) {
        return;
    }
    out->rx_frames = s_rx_frames;
    out->rx_bytes  = s_rx_bytes;
    out->cmds      = s_cmd_count;
    out->unknown   = s_err_unknown;
    out->gps_ok    = s_gps_ok;
    out->gps_err   = s_gps_err;
}

void cli_init(const cli_platform_t *plat)
{
    s_cfg = s_cfg_default;
    if (NULL != plat) {
        s_plat = *plat;
    } else {
        memset(&s_plat, 0, sizeof(s_plat));
    }
    s_len         = 0U;
    s_last_cr     = 0U;
    s_overflow    = 0U;
    s_esc         = 0U;
    s_bridge              = 0U;
    s_bridge_prev_out     = 0U;
    s_bridge_prev_verbose = 0U;
    s_bridge_prev_echo    = 0U;
    s_bridge_prev_auto    = 0U;
    s_bridge_prev_rate    = 0U;
    s_plus_run            = 0U;
    s_bridge_last_input = 0U;
    s_hist_count  = 0U;
    s_hist_head   = 0U;
    s_hist_idx    = -1;
    s_rx_frames   = 0U;
    s_rx_bytes    = 0U;
    s_cmd_count   = 0U;
    s_err_unknown = 0U;
    s_gps_ok      = 0U;
    s_gps_err     = 0U;

    /* 让硬件与配置保持一致。这里故意不动 GPS 波特率：USART5 重配有开销，
      * 而 load 时改波特率容易和模块当前设置打架，需要时用 set gps_baud 显式改。 */
    if (s_plat.output_mode_set) {
        s_plat.output_mode_set(s_cfg.mode);
    }

    cli_out_lock();
    printf("\r\n[cli] GD32F450 GPS console v%s, type 'help'\r\n", CLI_VERSION);
    cli_prompt();
    cli_out_unlock();
}

