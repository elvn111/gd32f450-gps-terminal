/* K922 的 $GPYBM / $GPNAV 解析(规则见 GPS_DATA_协议规范.md)。
  * 解析约定：按逗号位置取字段(ID1 是语句名本身)，不得“跳过非数字字段”
  * (SN00016361、VN 这类文本字段会让后面所有索引平移)；空字段表示无效，不等于 0；
  * 校验和错、缺 '*'、字段数不足都整句丢弃；一帧里可能含多句，逐句解析。
  * 解析不改动接收缓冲(内部用副本)，所以同一帧还能原样转发。
  * 本文件只写 ASCII 字符串。 */
#ifndef INFO_BSP_H
#define INFO_BSP_H

#include <stdlib.h>
#include <stdint.h>

typedef struct GPS_Module_Info {
	double latitude;
	double longitude;
	double ElpHeight;
	double Heading_Angle;
	double Baseline;
	double default_base_line;
	uint8_t GPS_Satellite;
} GPS_Module_Info;

typedef enum {
	INFO_ERROE = 0,
	GPNAV = 1,
	GPYBM = 2
}info_type_def;

/* 一句解析出来的结果 */
typedef enum {
	GPS_SRC_NONE = 0,       /* 该句被丢弃(校验和或格式错) */
	GPS_SRC_GPYBM = 1,
	GPS_SRC_GPNAV = 2
} gps_src_t;

#define GPS_STATUS_NONE     (0xFFU)     /* 状态字段缺失或非数字 */

typedef struct {
	GPS_Module_Info info;        /* 对外发布的 6 个数值 */
	gps_src_t  source;           /* 本组数据来自哪一句 */
	uint8_t    valid;            /* 1=定位与定向均有效 */
	uint8_t    position_status;  /* 状态来源：GPYBM ID17 / GPNAV ID19 */
	uint8_t    heading_status;   /* GPYBM ID18(整数) */
	uint8_t    heading_valid;    /* 1=定向结果可用 */
	char       hdg_text[3];      /* GPNAV ID20 文本 */
	char       utc[10];          /* ID3 的 UTC 时间，缺失时为空 */
} gps_fix_t;

/* 遍历一帧里的各条语句，每次调用处理一句。
  * from 传 NULL 之外的起点即可连续调用；返回已消费到的位置，
  * 返回 NULL 表示后面没有 '$' 语句了。out->source 为 NONE 表示该句被丢弃，继续遍历即可。 */
const char *gps_fix_next(const char *from, gps_fix_t *out);

/* 解析单独一句：1=通过，0=丢弃 */
int gps_fix_parse_one(const char *sentence, gps_fix_t *out);

/* 校验和与语句类型(0=不是本协议语句) */
info_type_def Info_Check(char *ch);

/* 兼容旧接口：解析缓冲里第一句有效语句 */
int gps_info_rsl(char* ptr, GPS_Module_Info* pInfo);

#endif /* INFO_BSP_H */
