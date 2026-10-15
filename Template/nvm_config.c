/* cli_config_t 的 Flash 存储：在 16 KB 扇区里做追加日志(save/load 的后端)。
  * 记录布局：魔数 + 序号 + 长度 + 载荷 + CRC。每次 save 只写下一个空槽，不擦扇区；
  * 槽写满才整扇区擦一次，load 取序号最大且校验通过的那条，因此掉电最多丢最后一次保存。 */
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "nvm_config.h"

#define NVM_MAGIC       (0x4E564D31U)   /* 魔数 'N','V','M','1' */
#define NVM_VERSION     (1U)
#define NVM_ERASED_WORD (0xFFFFFFFFU)

typedef struct
{
    uint32_t     magic;
    uint32_t     version;
    uint32_t     size;
    uint32_t     sequence;
    cli_config_t payload;
    uint32_t     checksum;
} nvm_record_t;

/* 日志从 NVM_CONFIG_ADDR 开始，一个槽一条记录 */
#define NVM_SLOT_ADDR(i)    (NVM_CONFIG_ADDR + ((uint32_t)(i) * NVM_RECORD_SIZE))

/* 芯片在 0x1FFF7A22 公布 Flash 容量(KB，与库里的 FMC 例程一致)。
  * 只有 2 MB/3 MB 型号才有 bank1，所以擦写前先判断容量。 */
#define NVM_FLASH_SIZE_KB   (*(const uint16_t *)0x1FFF7A22U)

static uint32_t nvm_checksum(const nvm_record_t *rec)
{
    const uint8_t *p = (const uint8_t *)rec;
    uint32_t sum = 0U;
    uint32_t i;

    for (i = 0U; i < (sizeof(nvm_record_t) - sizeof(uint32_t)); i++) {
        sum = (sum << 1) + (sum >> 31) + (uint32_t)p[i];    /* 写满了就擦掉整扇区，从头开始记 */
    }
    return sum;
}

static int nvm_record_valid(const nvm_record_t *rec)
{
    if ((NVM_MAGIC != rec->magic) || (NVM_VERSION != rec->version)) {
        return 0;
    }
    if ((uint32_t)sizeof(cli_config_t) != rec->size) {
        return 0;                       /* 结构体变了：旧记录作废 */
    }
    if (nvm_checksum(rec) != rec->checksum) {
        return 0;
    }
    return 1;
}

/* 扫描日志：返回最新有效记录的槽号、序号和已用槽数；没有有效记录返回 -1 */
static int nvm_scan(uint32_t *seq_out, uint32_t *used_out)
{
    uint32_t i;
    int      best = -1;
    uint32_t best_seq = 0U;
    uint32_t used = 0U;
    nvm_record_t rec;

    for (i = 0U; i < NVM_SLOT_MAX; i++) {
        memcpy(&rec, (const void *)NVM_SLOT_ADDR(i), sizeof(rec));
        if (NVM_ERASED_WORD == rec.magic) {
            break;                      /* 第一个空槽就是日志末尾 */
        }
        used++;
        if (nvm_record_valid(&rec)) {
            if ((best < 0) || (rec.sequence >= best_seq)) {
                best = (int)i;
                best_seq = rec.sequence;
            }
        }
    }
    if (seq_out) {
        *seq_out = (best < 0) ? 0U : best_seq;
    }
    if (used_out) {
        *used_out = used;
    }
    return best;
}

static int nvm_erase_sector(void)
{
    fmc_state_enum state;

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR |
                   FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    state = fmc_sector_erase(NVM_CONFIG_SECTOR);
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR |
                   FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    fmc_lock();
    return (FMC_READY == state) ? 0 : -10;
}

static int nvm_program_slot(uint32_t slot, const nvm_record_t *rec)
{
    const uint32_t *words = (const uint32_t *)rec;
    uint32_t i;
    fmc_state_enum state;

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR |
                   FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    for (i = 0U; i < (NVM_RECORD_SIZE / 4U); i++) {
        state = fmc_word_program(NVM_SLOT_ADDR(slot) + (i * 4U), words[i]);
        if (FMC_READY != state) {
            fmc_lock();
            return -11;
        }
    }
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR |
                   FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    fmc_lock();
    return 0;
}

int nvm_config_valid(void)
{
    return (nvm_scan(NULL, NULL) >= 0) ? 1 : 0;
}

uint32_t nvm_config_seq(void)
{
    uint32_t seq = 0U;
    (void)nvm_scan(&seq, NULL);
    return seq;
}

uint32_t nvm_config_used(void)
{
    uint32_t used = 0U;
    (void)nvm_scan(NULL, &used);
    return used;
}

int nvm_config_load(cli_config_t *cfg)
{
    nvm_record_t rec;
    int          idx;

    if (NULL == cfg) {
        return -1;
    }
    idx = nvm_scan(NULL, NULL);
    if (idx < 0) {
        return -2;                      /* 没有存过任何有效记录 */
    }
    memcpy(&rec, (const void *)NVM_SLOT_ADDR((uint32_t)idx), sizeof(rec));
    *cfg = rec.payload;

    /* 防御性限幅：固件升级后老记录里的值可能超出新范围 */
    if (cfg->mode > 1U) {
        cfg->mode = 1U;
    }
    if (cfg->echo > 1U) {
        cfg->echo = 1U;
    }
    if (cfg->verbose > 1U) {
        cfg->verbose = 1U;
    }
    if (cfg->sat_min > 40U) {
        cfg->sat_min = 4U;
    }
    if ((cfg->gps_baud < 9600U) || (cfg->gps_baud > 921600U)) {
        cfg->gps_baud = 115200U;
    }
    return 0;
}

int nvm_config_save(const cli_config_t *cfg)
{
    nvm_record_t rec;
    uint32_t     seq = 0U;
    uint32_t     used = 0U;
    uint32_t     slot;
    nvm_record_t verify;

    if (NULL == cfg) {
        return -1;
    }
    /* 1 MB 型号没有 bank1：宁可报错也不去擦 bank0 */
    if (NVM_FLASH_SIZE_KB < 2048U) {
        return -3;
    }

    (void)nvm_scan(&seq, &used);
    if (used >= NVM_SLOT_MAX) {
        if (0 != nvm_erase_sector()) {
            return -4;
        }
        seq = 0U;
        used = 0U;
    }
    slot = used;

    memset(&rec, 0, sizeof(rec));
    rec.magic    = NVM_MAGIC;
    rec.version  = NVM_VERSION;
    rec.size     = (uint32_t)sizeof(cli_config_t);
    rec.sequence = seq + 1U;
    rec.payload  = *cfg;
    rec.checksum = nvm_checksum(&rec);

    if (0 != nvm_program_slot(slot, &rec)) {
        return -5;
    }

    /* 回读校验：Flash 悄悄写失败比直接报错更麻烦 */
    memcpy(&verify, (const void *)NVM_SLOT_ADDR(slot), sizeof(verify));
    if (0 == nvm_record_valid(&verify)) {
        return -6;
    }
    if (0 != memcmp(&verify.payload, cfg, sizeof(*cfg))) {
        return -7;
    }
    return 0;
}

void nvm_config_info(void)
{
    uint32_t seq = 0U;
    uint32_t used = 0U;
    int      idx = nvm_scan(&seq, &used);

    printf("[nvm] 0x%08lX sector=%lu: ",
           (unsigned long)NVM_CONFIG_ADDR, (unsigned long)NVM_CONFIG_SECTOR);
    if (idx < 0) {
        printf("empty (0/%lu slots used)\r\n", (unsigned long)NVM_SLOT_MAX);
        return;
    }
    printf("seq=%lu, slot %d/%lu used\r\n",
           (unsigned long)seq, idx, (unsigned long)used);
}
