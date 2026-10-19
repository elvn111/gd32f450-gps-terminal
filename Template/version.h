#ifndef VERSION_H
#define VERSION_H

/* 固件版本的唯一来源：文本命令 ver/help 的字符串、下行信息帧的 version 字段、
  * 以及文档里的版本号都由这里推导，改版本只改下面三行。 */
/* 不带 U 后缀：FW_VERSION_STRING 由 #x 字符串化，带上 U 会变成 "1U.0U.0U" */
#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    6

/* "1.0.0" —— 文本界面用（宏拼接，无需手工同步） */
#define FW__STR(x)          #x
#define FW__XSTR(x)         FW__STR(x)
#define FW_VERSION_STRING   FW__XSTR(FW_VERSION_MAJOR) "." \
                            FW__XSTR(FW_VERSION_MINOR) "." \
                            FW__XSTR(FW_VERSION_PATCH)

/* 0x010000 —— 帧内 version 字段用：主<<16 | 次<<8 | 修订 */
#define FW_VERSION_PACKED   ((uint32_t)((FW_VERSION_MAJOR << 16) | \
                                        (FW_VERSION_MINOR << 8) | \
                                        (FW_VERSION_PATCH)))

#endif /* VERSION_H */
