/* K922 的 $GPYBM / $GPNAV 解析器，按 GPS_DATA 协议规范实现。
  * 要点：字段按逗号位置取(ID1 是语句名)，不能“跳过非数字字段”，
  * 否则 SN00016361、VN 这类文本字段会让后面所有索引平移；空字段表示无效而不是 0；
  * 校验和错、缺 '*'、字段数不足都整句丢弃；一帧里可能连着多句，逐句解析。 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "info_bsp.h"

#define GPS_FIELD_MAX       (40)    /* GPNAV 实际有 36 个字段 */
#define GPS_SENTENCE_MAX    (512)   /* 单句的本地副本：不改动接收缓冲，原帧还能原样转发 */
#define GPYBM_MIN_FIELDS    (23)    /* 规范 7.1：字段数不足就整句丢弃 */
#define GPNAV_MIN_FIELDS    (22)

/* 工作缓冲放静态区是故意的：512 字节行缓冲加 40 个字段指针曾放在栈上，
  * 直接把 TX_Task(约 1 KB 栈)压爆，现象是 LED 快闪加完全没输出。
  * 改静态后本层栈占用只剩几十字节。代价是解析不可重入，本工程只有 TX_Task 调用。 */
static char  s_work[GPS_SENTENCE_MAX];
static char *s_fields[GPS_FIELD_MAX];

/* 字段访问：按 ID(逗号位置)取，空字段=无效 */
static int field_double(char *fields[], int nf, int id, double *out)
{
	char *end = NULL;
	double v;

	if ((id < 1) || (id > nf) || ('\0' == fields[id - 1][0])) {
		return 0;                       /* 缺失或为空：不是 0(规范 5.2.2) */
	}
	v = strtod(fields[id - 1], &end);
	if ((NULL == end) || (end == fields[id - 1]) || ('\0' != *end)) {
		return 0;                       /* 文本字段(如 SN00016361、VN)：不是数字也正常 */
	}
	*out = v;
	return 1;
}

static int field_int(char *fields[], int nf, int id, int *out)
{
	char *end = NULL;
	long v;

	if ((id < 1) || (id > nf) || ('\0' == fields[id - 1][0])) {
		return 0;
	}
	v = strtol(fields[id - 1], &end, 10);
	if ((NULL == end) || (end == fields[id - 1]) || ('\0' != *end)) {
		return 0;
	}
	*out = (int)v;
	return 1;
}

static int field_text(char *fields[], int nf, int id, char *out, size_t outsz)
{
	size_t len;

	if ((NULL == out) || (0U == outsz)) {
		return 0;
	}
	out[0] = '\0';
	if ((id < 1) || (id > nf)) {
		return 0;
	}
	len = strlen(fields[id - 1]);
	if ((0U == len) || (len >= outsz)) {
		return 0;
	}
	memcpy(out, fields[id - 1], len + 1U);
	return 1;
}

/* 规范 7.2：物理量范围检查 */
static int range_ok(double lat, double lon, double heading)
{
	if ((lat < -90.0) || (lat > 90.0)) {
		return 0;
	}
	if ((lon < -180.0) || (lon > 180.0)) {
		return 0;
	}
	if ((heading < 0.0) || (heading > 360.0)) {
		return 0;
	}
	return 1;
}

/* 校验和与语句类型(规范 3.1) */
info_type_def Info_Check(char *ch)
{
	const char *star;
	const char *p;
	unsigned char xr = 0U;
	int want = 0;
	int i;

	if ((NULL == ch) || ('$' != ch[0])) {
		return INFO_ERROE;
	}
	star = strchr(ch, '*');
	if (NULL == star) {
		return INFO_ERROE;
	}
	for (p = ch + 1; p < star; p++) {
		xr ^= (unsigned char)*p;
	}
	for (i = 0; i < 2; i++) {
		char c = star[1 + i];
		int d;
		if ((c >= '0') && (c <= '9')) {
			d = c - '0';
		} else if ((c >= 'A') && (c <= 'F')) {
			d = c - 'A' + 10;
		} else if ((c >= 'a') && (c <= 'f')) {
			d = c - 'a' + 10;
		} else {
			return INFO_ERROE;
		}
		want = (want << 4) | d;
	}
	if (want != (int)xr) {
		return INFO_ERROE;
	}
	if (0 == strncmp(ch, "$GPNAV", 6)) {
		return GPNAV;
	}
	if (0 == strncmp(ch, "$GPYBM", 6)) {
		return GPYBM;
	}
	return INFO_ERROE;
}

/* 一句 -> gps_fix_t */
int gps_fix_parse_one(const char *sentence, gps_fix_t *out)
{
	char        *q;
	int          nf = 0;
	const char  *star;
	size_t       len;
	info_type_def type;
	double       lat = 0.0;
	double       lon = 0.0;
	double       hgt = 0.0;
	double       hdg = 0.0;
	double       base = 0.0;
	int          sat = 0;
	int          pos_status = -1;
	int          hdg_status = -1;
	int          field_ok;
	int          i;

	if ((NULL == sentence) || (NULL == out) || ('$' != sentence[0])) {
		return 0;
	}
	memset(out, 0, sizeof(*out));
	out->position_status = GPS_STATUS_NONE;
	out->heading_status  = GPS_STATUS_NONE;

	star = strchr(sentence, '*');
	if (NULL == star) {
		return 0;
	}
	/* 校验和与类型在原句上检查(调用者已补 '\0')，Info_Check() 不动接收缓冲 */
	type = Info_Check((char *)sentence);
	if (INFO_ERROE == type) {
		return 0;
	}

	len = (size_t)(star - sentence);
	if ((0U == len) || (len >= sizeof(s_work))) {
		return 0;
	}
	memcpy(s_work, sentence, len);
	s_work[len] = '\0';

	/* 只按逗号切分：文本字段不能让后面的索引平移 */
	s_fields[nf++] = s_work;
	for (q = s_work; ('\0' != *q) && (nf < GPS_FIELD_MAX); q++) {
		if (',' == *q) {
			*q = '\0';
			s_fields[nf++] = (q + 1);
		}
	}
	if ((GPYBM == type) && (nf < GPYBM_MIN_FIELDS)) {
		return 0;                       /* 规范 7.1 */
	}
	if ((GPNAV == type) && (nf < GPNAV_MIN_FIELDS)) {
		return 0;
	}

	out->source = (GPYBM == type) ? GPS_SRC_GPYBM : GPS_SRC_GPNAV;

	if (GPYBM == type) {
		field_ok  = field_double(s_fields, nf, 4, &lat);
		field_ok &= field_double(s_fields, nf, 5, &lon);
		field_ok &= field_double(s_fields, nf, 6, &hgt);
		field_ok &= field_double(s_fields, nf, 7, &hdg);
		field_ok &= field_double(s_fields, nf, 22, &base);
		field_ok &= field_int(s_fields, nf, 19, &sat);
		out->valid = (uint8_t)field_ok;

		if (field_int(s_fields, nf, 17, &pos_status)) {
			out->position_status = (uint8_t)pos_status;
		}
		if (field_int(s_fields, nf, 18, &hdg_status)) {
			out->heading_status = (uint8_t)hdg_status;
			out->heading_valid  = (0 != hdg_status) ? 1U : 0U;   /* 0=没有定向解 */
		}
		(void)field_text(s_fields, nf, 3, out->utc, sizeof(out->utc));
	} else {
		int sum = 0;
		int got_sv = 0;

		field_ok  = field_double(s_fields, nf, 7, &lat);
		field_ok &= field_double(s_fields, nf, 8, &lon);
		field_ok &= field_double(s_fields, nf, 9, &hgt);   /* 海拔，规范 5.3 */
		field_ok &= field_double(s_fields, nf, 12, &hdg);
		field_ok &= field_double(s_fields, nf, 22, &base);

		for (i = 23; i <= 27; i++) {            /* 规范 9.4：取参与解算卫星数之和 */
			int sv;
			if (field_int(s_fields, nf, i, &sv)) {
				sum += sv;
				got_sv = 1;
			}
		}
		sat = got_sv ? sum : 0;
		field_ok &= got_sv;
		out->valid = (uint8_t)field_ok;

		if (field_int(s_fields, nf, 19, &pos_status)) {
			out->position_status = (uint8_t)pos_status;
		}
		if (field_text(s_fields, nf, 20, out->hdg_text, sizeof(out->hdg_text))) {
			/* 规范 7.4：状态里出现 'N' 就表示定向解不可用 */
			out->heading_valid = (NULL == strchr(out->hdg_text, 'N')) ? 1U : 0U;
		}
		(void)field_text(s_fields, nf, 3, out->utc, sizeof(out->utc));
	}

	if (0 == range_ok(lat, lon, hdg)) {
		out->valid = 0U;                /* 规范 7.2 */
	}
	if ((0U != out->valid) &&
	    ((GPS_STATUS_NONE == out->position_status) || (0U == out->position_status))) {
		out->valid = 0U;                /* 规范 6.6：定位结果必须可用 */
	}
	if ((0U != out->valid) && (0U == out->heading_valid)) {
		out->valid = 0U;                /* 规范 6.6：定向结果必须可用 */
	}

	out->info.latitude      = lat;
	out->info.longitude     = lon;
	out->info.ElpHeight     = hgt;
	out->info.Heading_Angle = hdg;
	out->info.Baseline      = base;
	out->info.GPS_Satellite = (uint8_t)sat;
	return 1;
}

/* 逐句遍历一帧 */
const char *gps_fix_next(const char *from, gps_fix_t *out)
{
	const char *p = from;
	const char *star;

	if ((NULL == out) || (NULL == p)) {
		return NULL;
	}
	while ('\0' != *p) {
		if ('$' == *p) {
			star = strchr(p, '*');
			if (NULL == star) {
				return NULL;            /* 后面没有校验和字段了：当垃圾丢弃 */
			}
			(void)gps_fix_parse_one(p, out);   /* 该句被丢弃时把 out->source 置 NONE，调用者据此跳过 */
			if (('\0' != star[1]) && ('\0' != star[2])) {
				return (star + 3);      /* 跳过两个十六进制字符继续找下一句 */
			}
			return (star + 1);
		}
		p++;
	}
	return NULL;
}

/* 兼容旧接口：取缓冲里第一句通过的语句 */
int gps_info_rsl(char* ptr, GPS_Module_Info* pInfo)
{
	gps_fix_t   fix;
	const char *p = ptr;

	if ((NULL == ptr) || (NULL == pInfo)) {
		return 0;
	}
	while (NULL != (p = gps_fix_next(p, &fix))) {
		if (GPS_SRC_NONE != fix.source) {
			*pInfo = fix.info;
			return 1;
		}
	}
	return 0;
}
