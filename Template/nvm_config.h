/* 配置的掉电存储：把 cli_config_t 存进片内 Flash，作为 save/load 的后端。
  * 位置是 Flash bank1 起始的 16 KB 扇区(0x08100000)：链接脚本把镜像放在 bank0，
  * 而 GD32F450 没有 fmc_page_erase()(该 API 只对 F425/F427/F470 编译)，只能用扇区擦除，
  * bank0 的 16 KB 扇区已被程序占用。
  * 存储方式是追加日志：每次 save 只写下一个 24 字节槽位，不擦扇区；682 个槽写满才擦一次。
  * load 取序号最大且校验通过的那条，因此掉电最多丢最后一次(上一条仍有效)。
  * 本文件只写 ASCII 字符串。 */
#ifndef NVM_CONFIG_H
#define NVM_CONFIG_H

#include <stdint.h>
#include "cli.h"

/* 存储几何 */
#define NVM_CONFIG_ADDR         (0x08100000U)   /* bank1 起始，扇区名 12 */
#define NVM_CONFIG_SECTOR       (CTL_SN(16))    /* 寄存器值 CTL_SN(16) */
#define NVM_SECTOR_SIZE         (16U * 1024U)
#define NVM_RECORD_SIZE         (24U)           /* 6 个 32 位字 */
#define NVM_SLOT_MAX            (NVM_SECTOR_SIZE / NVM_RECORD_SIZE)

/* 保存配置：0=成功，负数为错误码 */
int nvm_config_save(const cli_config_t *cfg);

/* 读取最新记录：0=成功，负数为“没有有效记录” */
int nvm_config_load(cli_config_t *cfg);

/* 是否存在有效记录 */
int nvm_config_valid(void);

/* 最新记录的序号(无记录时为 0) */
uint32_t nvm_config_seq(void);

/* 当前已用槽位数 */
uint32_t nvm_config_used(void);

/* 打印一行记录状态(供 status/gstat 使用) */
void nvm_config_info(void);

#endif /* NVM_CONFIG_H */
