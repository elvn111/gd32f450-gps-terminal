/**
 * @file    test_info_bsp.c
 * @brief   PC test harness for info_bsp.c, using the vectors of
 *          GPS_DATA_协议规范.md appendix A (A.1 / A.2 / A.5) plus the A.4
 *          anti-cases and the frame-level rules.
 *
 * build+run: see run_tests.cmd / run_tests.sh in this directory
 *            (gcc -std=c99 -Wall -Wextra -I../Template test_info_bsp.c ../Template/info_bsp.c -lm)
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "info_bsp.h"

static int g_fail;
static int g_pass;

static void ok(int cond, const char *what)
{
    if (cond) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL: %s\n", what);
    }
}

static int near(double a, double b, double eps)
{
    return (fabs(a - b) <= eps);
}

/* render the six published lines exactly like main.c must print them */
static void render6(const GPS_Module_Info *i, char *out, size_t n)
{
    snprintf(out, n,
             "[GPS_DATA]:latitude:%f\n"
             "[GPS_DATA]:longitude:%f\n"
             "[GPS_DATA]:ElpHeight:%f\n"
             "[GPS_DATA]:Heading_Angle:%f\n"
             "[GPS_DATA]:Baseline:%f\n"
             "[GPS_DATA]:GPS_Satellite:%d\n",
             i->latitude, i->longitude, i->ElpHeight, i->Heading_Angle,
             i->Baseline, (int)i->GPS_Satellite);
}

/* ---- spec appendix A test data (exact copies) ---- */
static const char *A1 =
    "$GPYBM,SN00016361,092402.00,+27.852472239,+112.965327110,55.516,122.537,"
    "85.762,0.003,-0.010,-0.042,0.010,3102058.824,-193871.697,,,1,5,22,0,,0.951,22,*4E\r\n";

static const char *A2 =
    "$GPNAV,20261013,092415.00,18,4,,27.85247447971,112.96532616184,72.4029,"
    "-16.2776,272.830,104.600,80.830,,-0.014,-0.000,0.005,0.014,1,VN,5,1.254,"
    "7,0,11,,,7,0,11,,,,,,,*64\r\n";

static const char *A5 =
    "$GPYBM,SN00016361,092401.00,+27.852470107,+112.965320508,55.385,0.000,"
    "0.000,-0.018,-0.008,0.012,0.019,3102058.624,-193872.364,,,1,0,15,0,,0.000,0,*4E\r\n";

static const char *EXPECT_A1 =
    "[GPS_DATA]:latitude:27.852472\n"
    "[GPS_DATA]:longitude:112.965327\n"
    "[GPS_DATA]:ElpHeight:55.516000\n"
    "[GPS_DATA]:Heading_Angle:122.537000\n"
    "[GPS_DATA]:Baseline:0.951000\n"
    "[GPS_DATA]:GPS_Satellite:22\n";

static const char *EXPECT_A2 =
    "[GPS_DATA]:latitude:27.852474\n"
    "[GPS_DATA]:longitude:112.965326\n"
    "[GPS_DATA]:ElpHeight:72.402900\n"
    "[GPS_DATA]:Heading_Angle:104.600000\n"
    "[GPS_DATA]:Baseline:1.254000\n"
    "[GPS_DATA]:GPS_Satellite:18\n";

static const char *EXPECT_A5 =
    "[GPS_DATA]:latitude:27.852470\n"
    "[GPS_DATA]:longitude:112.965321\n"
    "[GPS_DATA]:ElpHeight:55.385000\n"
    "[GPS_DATA]:Heading_Angle:0.000000\n"
    "[GPS_DATA]:Baseline:0.000000\n"
    "[GPS_DATA]:GPS_Satellite:15\n";

int main(void)
{
    gps_fix_t fix;
    char      got[512];
    char      frame[1200];
    const char *p;
    int       blocks;

    /* ---------------- A.1 $GPYBM ---------------- */
    printf("A.1 $GPYBM\n");
    memset(&fix, 0, sizeof(fix));
    ok(1 == gps_fix_parse_one(A1, &fix), "A1 accepted");
    ok(GPS_SRC_GPYBM == fix.source, "A1 source GPYBM");
    ok(near(fix.info.latitude, 27.852472239, 1e-9), "A1 lat");
    ok(near(fix.info.longitude, 112.965327110, 1e-9), "A1 lon");
    ok(near(fix.info.ElpHeight, 55.516, 1e-9), "A1 height");
    ok(near(fix.info.Heading_Angle, 122.537, 1e-9), "A1 heading");
    ok(near(fix.info.Baseline, 0.951, 1e-9), "A1 baseline");
    ok(22 == fix.info.GPS_Satellite, "A1 satellites");
    ok(1 == fix.position_status, "A1 position_status");
    ok(5 == fix.heading_status, "A1 heading_status");
    ok(1 == fix.heading_valid, "A1 heading_valid");
    ok(0 == strcmp(fix.utc, "092402.00"), "A1 utc");
    ok(1 == fix.valid, "A1 valid");
    render6(&fix.info, got, sizeof(got));
    ok(0 == strcmp(got, EXPECT_A1), "A1 six output lines match the spec");
    if (0 != strcmp(got, EXPECT_A1)) {
        printf("--- got ---\n%s--- expected ---\n%s", got, EXPECT_A1);
    }

    /* A.4 anti-cases must NOT happen on A.1 */
    ok(!near(fix.info.latitude, fix.info.longitude, 1e-6), "A4 lat != lon");
    ok(!near(fix.info.ElpHeight, fix.info.Heading_Angle, 1e-6), "A4 height != heading");
    ok(fix.info.Baseline > 0.1, "A4 baseline not zero");
    ok(0 != fix.info.GPS_Satellite, "A4 satellites not zero");

    /* ---------------- A.2 $GPNAV ---------------- */
    printf("A.2 $GPNAV\n");
    memset(&fix, 0, sizeof(fix));
    ok(1 == gps_fix_parse_one(A2, &fix), "A2 accepted");
    ok(GPS_SRC_GPNAV == fix.source, "A2 source GPNAV");
    ok(near(fix.info.latitude, 27.85247447971, 1e-9), "A2 lat");
    ok(near(fix.info.longitude, 112.96532616184, 1e-9), "A2 lon");
    ok(near(fix.info.ElpHeight, 72.4029, 1e-9), "A2 height");
    ok(near(fix.info.Heading_Angle, 104.600, 1e-9), "A2 heading");
    ok(near(fix.info.Baseline, 1.254, 1e-9), "A2 baseline");
    ok(18 == fix.info.GPS_Satellite, "A2 satellites = 7+0+11 (spec 9.4)");
    ok(1 == fix.position_status, "A2 status1");
    ok(0 == strcmp(fix.hdg_text, "VN"), "A2 status2 text");
    ok(0 == fix.heading_valid, "A2 heading invalid (spec 7.4: contains N)");
    ok(0 == fix.valid, "A2 valid = 0 (heading not usable)");
    render6(&fix.info, got, sizeof(got));
    ok(0 == strcmp(got, EXPECT_A2), "A2 six output lines match the spec");
    if (0 != strcmp(got, EXPECT_A2)) {
        printf("--- got ---\n%s--- expected ---\n%s", got, EXPECT_A2);
    }

    /* ---------------- A.5 boundary: real zeroes ---------------- */
    printf("A.5 boundary (heading solution absent)\n");
    memset(&fix, 0, sizeof(fix));
    ok(1 == gps_fix_parse_one(A5, &fix), "A5 accepted");
    ok(0 == fix.heading_status, "A5 heading_status 0");
    ok(0 == fix.heading_valid, "A5 heading not usable");
    ok(0 == fix.valid, "A5 valid = 0");
    render6(&fix.info, got, sizeof(got));
    ok(0 == strcmp(got, EXPECT_A5), "A5 six output lines match the spec");
    if (0 != strcmp(got, EXPECT_A5)) {
        printf("--- got ---\n%s--- expected ---\n%s", got, EXPECT_A5);
    }

    /* ---------------- frame with both sentences (spec 5.4) ---------------- */
    printf("frame level: GPYBM + GPNAV back to back\n");
    snprintf(frame, sizeof(frame), "%s%s", A1, A2);
    blocks = 0;
    p = frame;
    while (NULL != (p = gps_fix_next(p, &fix))) {
        if (GPS_SRC_NONE == fix.source) {
            continue;
        }
        blocks++;
        if (1 == blocks) {
            ok(GPS_SRC_GPYBM == fix.source, "first block is GPYBM");
            ok(near(fix.info.latitude, 27.852472239, 1e-9), "first block lat");
        } else if (2 == blocks) {
            ok(GPS_SRC_GPNAV == fix.source, "second block is GPNAV");
            ok(near(fix.info.latitude, 27.85247447971, 1e-9), "second block lat");
        }
    }
    ok(2 == blocks, "both sentences in one frame are parsed");

    /* ---------------- rejection rules ---------------- */
    printf("rejection rules\n");
    {
        char bad[600];
        /* wrong checksum */
        strcpy(bad, A1);
        bad[strlen(bad) - 4] = 'F';             /* *4E -> *FE ... corrupt */
        memset(&fix, 0, sizeof(fix));
        (void)gps_fix_next(bad, &fix);
        ok(GPS_SRC_NONE == fix.source, "bad checksum rejected");

        /* truncated GPYBM (too few fields) */
        strcpy(bad, "$GPYBM,SN00016361,092402.00,+27.852472239,+112.965327110,55.516,*4E\r\n");
        memset(&fix, 0, sizeof(fix));
        (void)gps_fix_next(bad, &fix);
        ok(GPS_SRC_NONE == fix.source, "too few fields rejected");

        /* empty field must not become 0 (spec 5.2.2): blank latitude */
        strcpy(bad, A1);
        {
            char *lat = strstr(bad, ",+27.852472239");
            ok(NULL != lat, "test vector has the latitude field");
            if (NULL != lat) {
                memmove(lat + 1, lat + 16, strlen(lat + 16) + 1);  /* erase the value */
                memset(&fix, 0, sizeof(fix));
                (void)gps_fix_parse_one(bad, &fix);
                ok(0 == fix.valid, "empty field -> block invalid");
            }
        }
    }

    /* ---------------- legacy entry point ---------------- */
    printf("legacy gps_info_rsl()\n");
    {
        GPS_Module_Info info;
        memset(&info, 0, sizeof(info));
        ok(1 == gps_info_rsl((char *)A1, &info), "legacy returns 1");
        ok(near(info.latitude, 27.852472239, 1e-9), "legacy lat");
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
