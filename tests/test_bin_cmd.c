/* 二进制命令通道的 PC 侧回归：直接编译 ../Template/bin_cmd.c，
 * 喂《协议文档附录 B》里的请求帧（大端），核对应答帧的字节。
 *
 * 覆盖：0x01/0x01~0x09、0x02/0x01~0x06、0x03/0x01~0x04、事件、坏帧(CRC/帧尾/超长/坏长度)、
 *       未知命令集/未知功能码、确认字、逐字节分片、半帧超时重同步。
 *
 * 构建：见 run_tests.cmd / run_tests.sh
 */
#include <stdio.h>
#include <string.h>
#include "bin_cmd.h"
#include "cli.h"
#include "gps_link.h"
#include "version.h"

/* ---------------- 桩：配置/统计/链路由测试控制 ---------------- */

static uint32_t g_cfg[CLI_CFG_NUM + 1U];        /* 下标 1..9 有效 */
static cli_config_t g_cfg_view;
static cli_stats_t  g_stats;
static gps_link_stats_t g_lstats;
static int g_saved, g_loaded, g_defaulted, g_reset_called;
static int g_gps_lines;
static char g_gps_last_line[80];
static int g_hw_reset, g_preset, g_send_now_rc;
static uint8_t g_last_fix_ok;
static uint8_t g_last_fix[24];

/* 捕获应答帧(一帧一条，最多 8 条) */
#define MAXF 8
static uint8_t  g_fr[MAXF][320];
static uint16_t g_flen[MAXF];
static int      g_tx_count;

static int g_fail, g_pass;

#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL: %s (line %d)\n", msg, __LINE__); } \
} while (0)

/* ---- cli.h 接口桩 ---- */
const cli_config_t *cli_config_get(void) { return &g_cfg_view; }

int cli_config_item_get(uint8_t id, uint32_t *value)
{
    if ((NULL == value) || (0U == id) || (id > CLI_CFG_NUM)) { return -1; }
    *value = g_cfg[id];
    return 0;
}

int cli_config_item_set(uint8_t id, uint32_t value)
{
    if ((0U == id) || (id > CLI_CFG_NUM)) { return -1; }
    if ((id <= 3U) && (value > 1U)) { return -2; }
    if (id == 4U) { if (value > 40U) { return -2; } }
    if (id == 6U) { if (value > 2U) { return -2; } }
    if (id == 7U) { if (value > 2U) { return -2; } }
    if (id == 8U) { if ((value < 1U) || (value > 20U)) { return -2; } }
    g_cfg[id] = value;
    return 0;
}

int  cli_config_save(void)    { g_saved = 1; return 0; }
int  cli_config_load(void)    { g_loaded = 1; return 0; }
void cli_config_default(void) { g_defaulted = 1; }
void cli_stats_clear(void)    { memset(&g_stats, 0, sizeof(g_stats)); }
void cli_stats_get(cli_stats_t *out) { if (out) { *out = g_stats; } }
int  cli_bridge_switch(uint8_t on) { (void)on; return 0; }

/* ---- gps_link.h 接口桩 ---- */
uint16_t gps_link_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  b;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (b = 0U; b < 8U; b++) {
            crc = (0U != (crc & 1U)) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

int gps_link_send_frame(uint8_t cmd, uint8_t func, const uint8_t *data, uint16_t n)
{
    uint8_t *p;
    uint16_t i, crc;

    if (g_tx_count >= MAXF) { return -1; }
    p = g_fr[g_tx_count];
    p[0] = 0xA6U; p[1] = 0x6AU; p[2] = cmd; p[3] = func;
    p[4] = (uint8_t)(n >> 8U); p[5] = (uint8_t)(n & 0xFFU);
    for (i = 0U; i < n; i++) { p[6U + i] = data[i]; }
    crc = gps_link_crc16(p, (uint16_t)(6U + n));
    p[6U + n]      = (uint8_t)(crc & 0xFFU);
    p[7U + n]      = (uint8_t)(crc >> 8U);
    p[8U + n]      = 0xB7U;
    p[9U + n]      = 0x7BU;
    g_flen[g_tx_count] = (uint16_t)(10U + n);
    g_tx_count++;
    return 0;
}

int gps_link_send_info(const gps_link_info_t *info) { (void)info; return 0; }
int gps_link_send_now(void) { return g_send_now_rc; }
void gps_link_stats_get(gps_link_stats_t *out) { if (out) { *out = g_lstats; } }
uint8_t gps_link_last_fix(uint8_t out24[24])
{
    if (0U == g_last_fix_ok) { return 0U; }
    memcpy(out24, g_last_fix, 24U);
    return 1U;
}

/* ---- 平台钩子桩 ---- */
static int  stub_line(const char *line) { g_gps_lines++; snprintf(g_gps_last_line, sizeof(g_gps_last_line), "%s", line); return 0; }
static void stub_hw_reset(void) { g_hw_reset++; }
static void stub_preset(void) { g_preset++; }
static void stub_reset(void) { g_reset_called++; }
static uint32_t g_now_ms;
static uint32_t stub_uptime(void) { return g_now_ms; }

static const bin_cmd_plat_t g_plat = { stub_line, stub_hw_reset, stub_preset, stub_reset, stub_uptime };

/* ---------------- 工具 ---------------- */

static void feed_all(const uint8_t *b, uint16_t n)
{
    uint16_t i;

    for (i = 0U; i < n; i++) {
        (void)bin_cmd_feed(b[i]);
    }
}

static uint16_t hex2buf(const char *s, uint8_t *out, uint16_t max)
{
    uint16_t n = 0U;
    int      hi = -1;
    int      v;

    for (; '\0' != *s; s++) {
        int c = (int)(unsigned char)*s;
        v = -1;
        if ((c >= '0') && (c <= '9')) { v = c - '0'; }
        else if ((c >= 'A') && (c <= 'F')) { v = c - 'A' + 10; }
        else if ((c >= 'a') && (c <= 'f')) { v = c - 'a' + 10; }
        if (v < 0) { continue; }
        if (hi < 0) { hi = v; continue; }
        if (n < max) { out[n++] = (uint8_t)((hi << 4) | v); }
        hi = -1;
    }
    return n;
}

static void reset_state(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
    memset(&g_lstats, 0, sizeof(g_lstats));
    g_tx_count = 0;
    g_saved = g_loaded = g_defaulted = g_reset_called = 0;
    g_gps_lines = 0;
    g_gps_last_line[0] = '\0';
    g_hw_reset = g_preset = 0;
    g_send_now_rc = 0;
    g_last_fix_ok = 0;
    bin_cmd_stats_clear();
    bin_cmd_init(&g_plat);
}

/* 校验第 idx 条应答帧 */
static int reply_ok(int idx, uint8_t cmd, uint8_t func, uint8_t status, uint8_t seq)
{
    const uint8_t *p;
    uint16_t n, crc;

    if ((idx >= g_tx_count) || (idx >= MAXF)) { return 0; }
    p = g_fr[idx];
    if ((p[0] != 0xA6U) || (p[1] != 0x6AU)) { return 0; }
    if ((p[2] != (uint8_t)(cmd | 0x80U)) || (p[3] != func)) { return 0; }
    n = (uint16_t)(((uint16_t)p[4] << 8U) | p[5]);
    if ((uint16_t)(10U + n) != g_flen[idx]) { return 0; }
    crc = (uint16_t)((uint16_t)p[6U + n] | ((uint16_t)p[7U + n] << 8U));
    if (crc != gps_link_crc16(p, (uint16_t)(6U + n))) { return 0; }
    if ((p[8U + n] != 0xB7U) || (p[9U + n] != 0x7BU)) { return 0; }
    if (p[6] != status) { return 0; }
    if (p[7] != seq) { return 0; }
    return 1;
}

static const uint8_t *frame(int idx) { return g_fr[idx]; }
static uint16_t frame_len(int idx) { return g_flen[idx]; }

/* ---------------- 用例 ---------------- */

static void test_read_version(void)
{
    uint8_t f[64];
    uint16_t n;

    reset_state();
    n = hex2buf("A6 6A 01 01 00 01 07 A8 6E B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x01, BIN_ST_OK, 7), "0x01/0x01 reply");
    CHECK(16U == frame_len(0), "0x01/0x01 reply length 16 (N=6)");
    CHECK((uint8_t)(FW_VERSION_PACKED >> 24U) == frame(0)[8], "version byte0");
    CHECK((uint8_t)(FW_VERSION_PACKED & 0xFFU) == frame(0)[11], "version byte3");
}

static void test_cfg_get_set(void)
{
    uint8_t f[64];
    uint16_t n;

    /* 读配置项 out(id=6) = 2 */
    reset_state();
    g_cfg[6] = 2U;
    n = hex2buf("A6 6A 01 02 00 02 08 06 5E 4C B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x02, BIN_ST_OK, 8), "0x01/0x02 reply");
    CHECK((0U == frame(0)[8]) && (2U == frame(0)[11]), "0x01/0x02 value 2 BE");

    /* CRC 错帧：丢弃，不回应答 */
    reset_state();
    n = hex2buf("A6 6A 01 02 00 02 08 0A 5E 4C B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(0 == g_tx_count, "bad CRC dropped");

    /* 设置 out=2 */
    reset_state();
    n = hex2buf("A6 6A 01 03 00 06 09 06 00 00 00 02 1A 7F B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x03, BIN_ST_OK, 9), "0x01/0x03 reply OK");
    CHECK(2U == g_cfg[6], "0x01/0x03 wrote out=2");
    /* 配置变更事件(0xA0/0x03)由 main.c 的快照比对统一发，bin_cmd 这里只回应答 */
    CHECK(1 == g_tx_count, "0x01/0x03 只回应答（事件由 main.c 发）");

    /* 越界：rate = 0 -> BAD_PARAM（手工组帧，大端长度） */
    reset_state();
    {
        uint8_t req[16];
        uint16_t i = 0U, crc;
        req[i++] = 0xA6; req[i++] = 0x6A; req[i++] = 0x01; req[i++] = 0x03;
        req[i++] = 0x00; req[i++] = 0x06;
        req[i++] = 0x0A; req[i++] = 0x08;
        req[i++] = 0x00; req[i++] = 0x00; req[i++] = 0x00; req[i++] = 0x00;
        crc = gps_link_crc16(req, i);
        req[i++] = (uint8_t)(crc & 0xFFU); req[i++] = (uint8_t)(crc >> 8U);
        req[i++] = 0xB7; req[i++] = 0x7B;
        feed_all(req, i);
        CHECK(reply_ok(0, 0x01, 0x03, BIN_ST_BAD_PARAM, 0x0A), "rate=0 -> BAD_PARAM");
    }
}

static void test_save_load_default_reset(void)
{
    uint8_t f[64];
    uint16_t n;

    reset_state();
    n = hex2buf("A6 6A 01 04 00 01 0B A8 A7 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x04, BIN_ST_OK, 0x0B), "0x01/0x04 save");
    CHECK(1 == g_saved, "save hook called");

    reset_state();
    n = hex2buf("A6 6A 01 05 00 01 0C E8 99 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x05, BIN_ST_OK, 0x0C), "0x01/0x05 load");
    CHECK(1 == g_loaded, "load hook called");

    reset_state();
    n = hex2buf("A6 6A 01 06 00 03 0D 5A A5 24 FA B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x06, BIN_ST_OK, 0x0D), "0x01/0x06 default w/ confirm");
    CHECK(1 == g_defaulted, "default hook called");

    reset_state();
    {
        uint8_t req[16];
        uint16_t i = 0U, crc;
        req[i++] = 0xA6; req[i++] = 0x6A; req[i++] = 0x01; req[i++] = 0x06;
        req[i++] = 0x00; req[i++] = 0x03;
        req[i++] = 0x0D; req[i++] = 0x00; req[i++] = 0x00;
        crc = gps_link_crc16(req, i);
        req[i++] = (uint8_t)(crc & 0xFFU); req[i++] = (uint8_t)(crc >> 8U);
        req[i++] = 0xB7; req[i++] = 0x7B;
        feed_all(req, i);
        CHECK(reply_ok(0, 0x01, 0x06, BIN_ST_NEED_CONFIRM, 0x0D), "0x01/0x06 no confirm");
        CHECK(0 == g_defaulted, "default not applied without confirm");
    }

    reset_state();
    n = hex2buf("A6 6A 01 07 00 03 0C 5A A5 74 EB B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(0 == g_tx_count, "0x01/0x07 no reply");
    CHECK(1 == g_reset_called, "reset hook called");
}

static void test_clear_and_stats(void)
{
    uint8_t f[64];
    uint16_t n;

    reset_state();
    n = hex2buf("A6 6A 01 08 00 01 0E 6B F4 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x08, BIN_ST_OK, 0x0E), "0x01/0x08 clear stats");

    reset_state();
    g_stats.rx_bytes = 48641U; g_stats.gps_ok = 2411U; g_stats.gps_err = 3U;
    g_lstats.tx_frames = 1204U; g_lstats.drop_latest = 37U;
    n = hex2buf("A6 6A 01 09 00 01 0F AB C8 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x09, BIN_ST_OK, 0x0F), "0x01/0x09 stats");
    CHECK(38U == frame_len(0), "0x01/0x09 reply 28-byte payload");
    CHECK((0x00U == frame(0)[8]) && (0x00U == frame(0)[9]) &&
          (0xBEU == frame(0)[10]) && (0x01U == frame(0)[11]), "stats rx_bytes=48641 BE");
}

static void test_link_cmds(void)
{
    uint8_t f[80];
    uint16_t n;

    reset_state();
    g_cfg[6] = 2U; g_cfg[7] = 1U; g_cfg[8] = 20U;
    n = hex2buf("A6 6A 02 01 00 04 0A 02 01 14 83 66 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x01, BIN_ST_OK, 0x0A), "0x02/0x01 set link");
    CHECK((2U == g_cfg[6]) && (1U == g_cfg[7]) && (20U == g_cfg[8]), "0x02/0x01 wrote out/auto/rate");

    /* auto=9 越界（改坏 CRC 的帧应被丢弃，这里用正确 CRC 手工组） */
    reset_state();
    {
        uint8_t req[16];
        uint16_t i = 0U, crc;
        req[i++] = 0xA6; req[i++] = 0x6A; req[i++] = 0x02; req[i++] = 0x01;
        req[i++] = 0x00; req[i++] = 0x04;
        req[i++] = 0x0A; req[i++] = 0x02; req[i++] = 0x09; req[i++] = 0x14;
        crc = gps_link_crc16(req, i);
        req[i++] = (uint8_t)(crc & 0xFFU); req[i++] = (uint8_t)(crc >> 8U);
        req[i++] = 0xB7; req[i++] = 0x7B;
        feed_all(req, i);
        CHECK(reply_ok(0, 0x02, 0x01, BIN_ST_BAD_PARAM, 0x0A), "auto=9 -> BAD_PARAM");
    }

    reset_state();
    g_send_now_rc = 0;
    n = hex2buf("A6 6A 02 02 00 01 10 AC 24 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x02, BIN_ST_OK, 0x10), "0x02/0x02 send now");

    reset_state();
    g_send_now_rc = -1;
    n = hex2buf("A6 6A 02 02 00 01 10 AC 24 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x02, BIN_ST_NOT_READY, 0x10), "0x02/0x02 no value -> NOT_READY");

    reset_state();
    n = hex2buf("A6 6A 02 03 00 01 11 6C 18 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x03, BIN_ST_OK, 0x11), "0x02/0x03 info");

    reset_state();
    n = hex2buf("A6 6A 02 04 00 01 12 2D 6D B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x04, BIN_ST_NOT_READY, 0x12), "0x02/0x04 no fix -> NOT_READY");

    reset_state();
    g_last_fix_ok = 1U;
    memset(g_last_fix, 0x11, sizeof(g_last_fix));
    n = hex2buf("A6 6A 02 04 00 01 12 2D 6D B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x04, BIN_ST_OK, 0x12), "0x02/0x04 with fix");
    CHECK(36U == frame_len(0), "0x02/0x04 reply carries 24-byte payload");

    reset_state();
    n = hex2buf("A6 6A 02 05 00 02 13 01 A0 8D B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x05, BIN_ST_OK, 0x13), "0x02/0x05 bridge");

    reset_state();
    g_cfg[6] = 0U; g_cfg[7] = 0U; g_cfg[8] = 5U;
    n = hex2buf("A6 6A 02 06 00 01 01 6D 18 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x02, 0x06, BIN_ST_OK, 0x01), "0x02/0x06 default send");
    CHECK((2U == g_cfg[6]) && (1U == g_cfg[7]) && (20U == g_cfg[8]), "0x02/0x06 defaults applied");
}

static void test_gps_cmds(void)
{
    uint8_t f[80];
    uint16_t n;

    reset_state();
    n = hex2buf("A6 6A 03 01 00 18 0D 6C 6F 67 20 63 6F 6D 31 20 67 70 6E 61 76 20 6F 6E 74 69 6D 65 20 31 13 D2 B7 7B",
                f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x03, 0x01, BIN_ST_OK, 0x0D), "0x03/0x01 passthru");
    CHECK((1 == g_gps_lines) && (0 == strcmp(g_gps_last_line, "log com1 gpnav ontime 1")),
          "0x03/0x01 text forwarded");

    reset_state();
    n = hex2buf("A6 6A 03 02 00 01 0E 11 EC B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x03, 0x02, BIN_ST_OK, 0x0E), "0x03/0x02 hwreset reply");
    CHECK(1 == g_hw_reset, "hwreset hook called");

    reset_state();
    n = hex2buf("A6 6A 03 03 00 01 0F D1 D0 B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x03, 0x03, BIN_ST_OK, 0x0F), "0x03/0x03 preset reply");
    CHECK(1 == g_preset, "preset hook called");

    reset_state();
    n = hex2buf("A6 6A 03 04 00 03 10 5A A5 96 DE B7 7B", f, sizeof(f));
    feed_all(f, n);
    CHECK(reply_ok(0, 0x03, 0x04, BIN_ST_OK, 0x10), "0x03/0x04 factory reply");
    CHECK((1 == g_gps_lines) && (0 == strcmp(g_gps_last_line, "freset")), "freset sent");

    reset_state();
    {
        uint8_t req[16];
        uint16_t i = 0U, crc;
        req[i++] = 0xA6; req[i++] = 0x6A; req[i++] = 0x03; req[i++] = 0x04;
        req[i++] = 0x00; req[i++] = 0x03;
        req[i++] = 0x10; req[i++] = 0x12; req[i++] = 0x34;
        crc = gps_link_crc16(req, i);
        req[i++] = (uint8_t)(crc & 0xFFU); req[i++] = (uint8_t)(crc >> 8U);
        req[i++] = 0xB7; req[i++] = 0x7B;
        feed_all(req, i);
        CHECK(reply_ok(0, 0x03, 0x04, BIN_ST_NEED_CONFIRM, 0x10), "0x03/0x04 no confirm");
        CHECK(0 == g_gps_lines, "no freset without confirm");
    }
}

static void test_bad_frames(void)
{
    uint8_t f[80];
    uint16_t n;

    /* CRC 错 */
    reset_state();
    n = hex2buf("A6 6A 01 01 00 01 07 A8 6E B7 7B", f, sizeof(f));
    f[7] ^= 0x5AU;
    feed_all(f, n);
    CHECK(0 == g_tx_count, "bad CRC -> no reply");

    /* 帧尾错 */
    reset_state();
    n = hex2buf("A6 6A 01 01 00 01 07 A8 6E B7 7B", f, sizeof(f));
    f[n - 1U] = 0x00U;
    feed_all(f, n);
    CHECK(0 == g_tx_count, "bad tail -> no reply");

    /* N 超上限 */
    reset_state();
    {
        uint8_t req[16];
        uint16_t i = 0U, crc;
        req[i++] = 0xA6; req[i++] = 0x6A; req[i++] = 0x01; req[i++] = 0x01;
        req[i++] = 0x01; req[i++] = 0x2C;               /* N = 300 */
        req[i++] = 0x07;
        crc = gps_link_crc16(req, i);
        req[i++] = (uint8_t)(crc & 0xFFU); req[i++] = (uint8_t)(crc >> 8U);
        req[i++] = 0xB7; req[i++] = 0x7B;
        feed_all(req, i);
        CHECK(0 == g_tx_count, "N>256 dropped");
    }

    /* 未知命令集 0x05/0x01 -> 0x81/0x01 UNKNOWN_FUNC */
    reset_state();
    {
        uint8_t req[16];
        uint16_t i = 0U, crc;
        req[i++] = 0xA6; req[i++] = 0x6A; req[i++] = 0x05; req[i++] = 0x01;
        req[i++] = 0x00; req[i++] = 0x01;
        req[i++] = 0x0F;
        crc = gps_link_crc16(req, i);
        req[i++] = (uint8_t)(crc & 0xFFU); req[i++] = (uint8_t)(crc >> 8U);
        req[i++] = 0xB7; req[i++] = 0x7B;
        feed_all(req, i);
        CHECK(reply_ok(0, 0x01, 0x01, BIN_ST_UNKNOWN_FUNC, 0x0F), "unknown cmd set");
    }

    /* 已知命令集的未知功能码 */
    reset_state();
    {
        uint8_t req[16];
        uint16_t i = 0U, crc;
        req[i++] = 0xA6; req[i++] = 0x6A; req[i++] = 0x01; req[i++] = 0x7F;
        req[i++] = 0x00; req[i++] = 0x01;
        req[i++] = 0x0F;
        crc = gps_link_crc16(req, i);
        req[i++] = (uint8_t)(crc & 0xFFU); req[i++] = (uint8_t)(crc >> 8U);
        req[i++] = 0xB7; req[i++] = 0x7B;
        feed_all(req, i);
        CHECK(reply_ok(0, 0x01, 0x7F, BIN_ST_UNKNOWN_FUNC, 0x0F), "unknown func");
    }
}

static void test_half_frame_timeout(void)
{
    uint8_t f[80];
    uint16_t n;
    uint16_t i;

    reset_state();
    n = hex2buf("A6 6A 01 01 00 01 07 A8 6E B7 7B", f, sizeof(f));
    for (i = 0U; i < 5U; i++) { (void)bin_cmd_feed(f[i]); }
    g_now_ms += 200U;
    feed_all(f, n);
    CHECK(reply_ok(0, 0x01, 0x01, BIN_ST_OK, 7), "half frame timed out, next frame ok");

    reset_state();
    for (i = 0U; i < n; i++) { (void)bin_cmd_feed(f[i]); }
    CHECK(reply_ok(0, 0x01, 0x01, BIN_ST_OK, 7), "byte-by-byte frame ok");
}

static void test_events(void)
{
    uint8_t chg[5];

    reset_state();
    g_lstats.seq = 1300U;
    bin_cmd_event(BIN_EV_LOST, NULL, 0U);
    CHECK(1 == g_tx_count, "event sent");
    CHECK(0xA0U == frame(0)[2], "event cmd 0xA0");
    CHECK((0x00U == frame(0)[4]) && (0x06U == frame(0)[5]), "event N = 6");
    CHECK((0x05U == frame(0)[6]) && (0x14U == frame(0)[7]), "event seq = 1300 BE");

    reset_state();
    chg[0] = 6U; chg[1] = 0U; chg[2] = 0U; chg[3] = 0U; chg[4] = 2U;
    bin_cmd_event(BIN_EV_CFGCHG, chg, 5U);
    CHECK((0xA0U == frame(0)[2]) && (0x03U == frame(0)[3]), "cfg-change event");
    CHECK((0x00U == frame(0)[4]) && (0x07U == frame(0)[5]), "cfg-change N = 7");
}

int main(void)
{
    int i;

    reset_state();
    for (i = 1; i <= (int)CLI_CFG_NUM; i++) { g_cfg[i] = 0U; }
    memset(&g_cfg_view, 0, sizeof(g_cfg_view));

    printf("== bin_cmd regression ==\n");
    test_read_version();
    test_cfg_get_set();
    test_save_load_default_reset();
    test_clear_and_stats();
    test_link_cmds();
    test_gps_cmds();
    test_bad_frames();
    test_half_frame_timeout();
    test_events();

    printf("pass %d, fail %d\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
