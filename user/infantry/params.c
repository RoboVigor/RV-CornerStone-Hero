#include "params.h"
#include "stm32f4xx.h"
#include <stddef.h>
#include <string.h>

/*******************************************************************************
 * flash 存储: Sector 23 (Bank 2 最后一个 128KB 扇区, 0x081E0000)
 * - 程序约 120KB, 全部位于 Bank 1; F427 2MB 为双 Bank, 擦写 Bank 2 时 Bank 1 仍可取指 (RWW),
 *   因此擦除期间中断与其他任务照常运行, 只有调用 Params_Save 的任务阻塞
 * - 若程序将来大到进入 Bank 2 (>1MB), 需更换存储位置
 *******************************************************************************/
#define PARAMS_FLASH_SECTOR FLASH_Sector_23
#define PARAMS_FLASH_ADDR   0x081E0000u
#define PARAMS_MAGIC        0x50415231u // "PAR1"
#define PARAMS_VERSION      1u          // 修改 PitchParams_Type 布局时 +1, 旧数据自动作废

typedef struct {
    uint32_t         magic;
    uint32_t         version;
    uint32_t         size;
    PitchParams_Type pitch;
    uint32_t         crc;
} ParamsRecord_Type;

// 编译期默认值 (2026-11 实测整定结果, 来历见 tasks.c 中 pitch 控制器注释)
const PitchParams_Type PitchParamsDefault = {
    40.0f,   // kAngle
    220.0f,  // kpSpeed
    3.0f,    // kiSpeed
    4500.0f, // iMax
    3700.0f, // fc: 实测 Fc=5340±250 的 70%
    8.0f,    // fcEps
    3000.0f, // aBrake
    -815.0f, // loadC
    185.7f,  // loadD
};

volatile PitchParams_Type PitchParams;

static uint32_t Crc32(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    int      k;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

void Params_Set_Default(void) {
    memcpy((void *) &PitchParams, &PitchParamsDefault, sizeof(PitchParams_Type));
}

ParamsSource_e Params_Init(void) {
    const ParamsRecord_Type *rec = (const ParamsRecord_Type *) PARAMS_FLASH_ADDR;

    Params_Set_Default();
    if (rec->magic != PARAMS_MAGIC || rec->version != PARAMS_VERSION || rec->size != sizeof(ParamsRecord_Type)) {
        return PARAMS_SRC_DEFAULT;
    }
    if (Crc32((const uint8_t *) rec, offsetof(ParamsRecord_Type, crc)) != rec->crc) {
        return PARAMS_SRC_DEFAULT;
    }
    memcpy((void *) &PitchParams, &rec->pitch, sizeof(PitchParams_Type));
    return PARAMS_SRC_FLASH;
}

int Params_Save(void) {
    ParamsRecord_Type rec;
    const uint32_t   *words = (const uint32_t *) &rec;
    uint32_t          i;
    int               ret = 0;

    memset(&rec, 0, sizeof(rec));
    rec.magic   = PARAMS_MAGIC;
    rec.version = PARAMS_VERSION;
    rec.size    = sizeof(ParamsRecord_Type);
    memcpy(&rec.pitch, (const void *) &PitchParams, sizeof(PitchParams_Type));
    rec.crc = Crc32((const uint8_t *) &rec, offsetof(ParamsRecord_Type, crc));

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (FLASH_EraseSector(PARAMS_FLASH_SECTOR, VoltageRange_3) != FLASH_COMPLETE) {
        ret = -1;
    } else {
        for (i = 0; i < sizeof(rec) / 4; i++) {
            if (FLASH_ProgramWord(PARAMS_FLASH_ADDR + i * 4, words[i]) != FLASH_COMPLETE) {
                ret = -2;
                break;
            }
        }
    }
    FLASH_Lock();

    // 回读校验
    if (ret == 0 && memcmp((const void *) PARAMS_FLASH_ADDR, &rec, sizeof(rec)) != 0) ret = -3;
    return ret;
}
