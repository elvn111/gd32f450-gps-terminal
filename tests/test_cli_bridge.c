/* 控制台桥模式的主机侧回归：直接编译 ../Template/cli.c，桩掉平台钩子。
 *
 * 覆盖用户关心的"转发 / 退出"两条：
 *   - 进桥后字节是否真的整段原样转给定位模块（gps_send_raw）；
 *   - 退出桥（'+++' 转义 / bridge off 命令 / 二进制 0x02-0x05 调用）后，
 *     out/verbose/echo/auto/rate 是否精确恢复到进桥前的值，且在桥模式下它们被改成管道参数。
 *
 * 构建：见 run_tests.cmd / run_tests.sh
 */
#include <stdio.h>
#include <string.h>
#include "cli.h"
#include "version.h"

static int g_fail, g_pass;

#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL: %s (line %d)\n", msg, __LINE__); } \
} while (0)

/* ---------------- 平台钩子桩 ---------------- */

static uint8_t  g_raw[512];
static uint16_t g_raw_len;
static int      g_raw_calls;
static uint32_t g_now_ms;
static int      g_reset_calls;
static int      g_save_calls, g_load_calls;
static int      g_preset_calls;
static char     g_last_line[96];

static void stub_gps_line(const char *line)
{
    snprintf(g_last_line, sizeof(g_last_line), "%s", line);
}

static void stub_gps_raw(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    g_raw_calls++;
    for (i = 0U; (i < len) && (g_raw_len < (uint16_t)sizeof(g_raw)); i++) {
        g_raw[g_raw_len++] = buf[i];
    }
}

static void stub_preset(void) { g_preset_calls++; }
static uint32_t stub_uptime(void) { return g_now_ms; }
static void stub_reset(void) { g_reset_calls++; }
static int  stub_save(const cli_config_t *cfg) { (void)cfg; g_save_calls++; return 0; }
static int  stub_load(cli_config_t *cfg) { (void)cfg; g_load_calls++; return 0; }
static void stub_tasks(void) {}
static void stub_dma(void) {}
static void stub_nvm(void) {}
static void stub_send_now(void) {}
static void stub_link_info(void) {}
static void stub_info_print(void) {}
static void stub_baud(uint32_t b) { (void)b; }
static void stub_outmode(uint8_t m) { (void)m; }
static void stub_link_info_send(void) {}
static void stub_hw_reset(void) {}

static const cli_platform_t g_plat = {
    stub_gps_line, stub_gps_raw, stub_preset, stub_info_print, stub_baud, stub_outmode,
    NULL, stub_tasks, stub_dma, stub_save, stub_load, stub_nvm, stub_send_now,
    stub_link_info, NULL, NULL, stub_reset, stub_uptime, stub_hw_reset, stub_link_info_send
};

/* ---------------- 工具 ---------------- */

static void feed(const char *s)
{
    cli_input((const uint8_t *)s, (uint16_t)strlen(s));
}

static uint32_t item(uint8_t id)
{
    uint32_t v = 0xFFFFFFFFU;

    (void)cli_config_item_get(id, &v);
    return v;
}

static void snapshot(uint32_t *out, uint32_t *verbose, uint32_t *echo,
                     uint32_t *aut, uint32_t *rate)
{
    *out     = item(6);
    *verbose = item(3);
    *echo    = item(2);
    *aut     = item(7);
    *rate    = item(8);
}

int main(void)
{
    uint32_t o0, v0, e0, a0, r0;

    memset(g_raw, 0, sizeof(g_raw));
    cli_init(&g_plat);
    printf("== cli bridge regression ==\n");

    /* 1) 默认配置与 ID 映射 */
    CHECK(item(1) == 1U, "默认 mode=1(WORKED)");
    CHECK(item(2) == 1U, "默认 echo=1");
    CHECK(item(3) == 0U, "默认 verbose=0");
    CHECK(item(4) == 4U, "默认 sat_min=4");
    CHECK(item(5) == 115200U, "默认 gps_baud=115200");
    CHECK(item(6) == 2U, "默认 out=2(二进制帧)");
    CHECK(item(7) == 1U, "默认 auto=1(fix)");
    CHECK(item(8) == 20U, "默认 rate=20");
    CHECK(item(9) == 0x40U, "默认 prefix='@'");
    CHECK(cli_config_item_set(8, 0) == -2, "rate=0 越界被拒");
    CHECK(cli_config_item_set(10, 1) == -1, "不存在的配置项编号被拒");
    CHECK(cli_bridge_active() == 0U, "上电桥模式为关");

    /* 2) 进桥：配置被改成管道参数，字节整段原样转给模块 */
    snapshot(&o0, &v0, &e0, &a0, &r0);
    g_now_ms = 1000U;
    CHECK(cli_bridge_switch(1U) == 0, "cli_bridge_switch(1) 成功（0x02/0x05 走这条）");
    CHECK(cli_bridge_active() == 1U, "桥模式已开");
    CHECK((item(6) == 1U) && (item(3) == 0U) && (item(2) == 0U) &&
          (item(7) == 1U) && (item(8) == 20U),
          "进桥后 out=1 verbose=0 echo=0 auto=1 rate=20");

    g_raw_len = 0;
    g_raw_calls = 0;
    feed("log com1 gpnav ontime 1\r\n");
    CHECK(g_raw_len == 25U, "转发字节数 = 25（命令 23 字符 + CRLF）");
    CHECK(0 == memcmp(g_raw, "log com1 gpnav ontime 1\r\n", 25),
          "转发内容与输入逐字节一致（含 CRLF，未加修饰）");

    g_raw_len = 0;
    feed("bridge on\r\n");                     /* 桥模式下走真实数据路径：整行转发 */
    CHECK(g_raw_len == 11U, "桥模式下输入不当作命令执行，照样转发(11 字节)");

    /* 3) 退出：'+++' 需要 >= 1 s 静默；退出后配置精确恢复 */
    g_now_ms += 500U;
    feed("+++");
    CHECK(cli_bridge_active() == 1U, "静默不足 1 s 时 '+++' 不退出（防误触发）");
    g_now_ms += 1200U;
    feed("+++");
    CHECK(cli_bridge_active() == 0U, "静默 1.7 s 后 '+++' 退出桥模式");
    CHECK((item(6) == o0) && (item(3) == v0) && (item(2) == e0) &&
          (item(7) == a0) && (item(8) == r0),
          "退出后 out/verbose/echo/auto/rate 全部恢复为进桥前的值");
    CHECK(item(2) == 1U, "退出后回显恢复(echo=1)");

    /* 4) 退出后命令系统恢复：set 生效、'+++' 不再被吞 */
    {
        char c1[] = "set rate 5";               /* 就地分词，必须是可写缓冲 */
        cli_exec_line(c1);
    }
    CHECK(item(8) == 5U, "退出后 set rate 5 生效");
    {
        char c2[] = "set rate 20";
        cli_exec_line(c2);
    }

    /* 5) 再次进桥，改用 bridge off 命令退出（另一条出口） */
    g_now_ms += 100U;
    cli_bridge_switch(1U);
    CHECK(cli_bridge_active() == 1U, "第二次进桥");
    {
        char c3[] = "bridge off";
        cli_exec_line(c3);
    }
    CHECK(cli_bridge_active() == 0U, "bridge off 命令退出桥模式");
    CHECK(item(6) == o0, "bridge off 后 out 也恢复");

    /* 6) 重复调 switch(0) 无副作用（幂等） */
    CHECK(cli_bridge_switch(0U) == 0, "重复退出不报错");
    CHECK(cli_bridge_active() == 0U, "仍然是不在桥模式");

    /* 7) 默认/清统计接口（0x01/0x06、0x01/0x08 用） */
    cli_config_item_set(6, 1);
    cli_config_default();
    CHECK(item(6) == 2U, "cli_config_default 恢复默认 out=2");
    {
        cli_stats_t st;
        cli_input((const uint8_t *)"abc\r\n", 5U);
        cli_stats_get(&st);
        CHECK(st.rx_bytes >= 5U, "cli_stats_clear/get 可用");
        cli_stats_clear();
        cli_stats_get(&st);
        CHECK(st.rx_bytes == 0U, "清统计后 rx_bytes 归零");
    }

    printf("pass %d, fail %d\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
