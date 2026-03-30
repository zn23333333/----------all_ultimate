/**
 * @file    runtime_config.c
 * @brief   运行时配置Flash持久化模块实现
 *
 * 功能概述：
 *   1. 将 LogicThresholds_t + 风扇档位表 打包为 RuntimeConfigBlob_t
 *   2. 使用 CRC32 校验确保数据完整性
 *   3. 写入 STM32F407 内部 Flash Sector 11（0x080E0000，128KB）
 *   4. 写Flash时挂起调度器（vTaskSuspendAll）避免任务中断
 *   5. 支持延迟保存（1秒去抖）和工厂复位
 */
#include "runtime_config.h"

#include "stm32f4xx_flash.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

/* ======================== Flash存储常量 ======================== */
#define RUNTIME_CFG_FLASH_ADDR      0x080E0000UL  /**< Flash Sector 11 起始地址 */
#define RUNTIME_CFG_FLASH_SECTOR    FLASH_Sector_11 /**< 擦除扇区编号 */
#define RUNTIME_CFG_MAGIC           0x52434647UL  /**< 魔数标识 'RCFG'（判断数据是否有效） */
#define RUNTIME_CFG_VERSION         0x00010002UL  /**< 数据版本号（结构变更时递增） */
#define RUNTIME_CFG_SAVE_DELAY_MS   1000U         /**< 延迟保存时间（毫秒，去抖防抢） */

/**
 * @brief   Flash持久化数据块结构体
 *
 * 按顺序存储：魔数 -> 版本号 -> 阈值 -> 风扇档位 -> 预留 -> CRC32
 */
typedef struct
{
    uint32_t magic;                        /**< 魔数标识，用于判断数据是否已写入 */
    uint32_t version;                      /**< 数据版本号，结构变更时更新 */
    LogicThresholds_t thresholds;           /**< 自动控制逻辑阈值集合 */
    uint8_t fan_levels[MOTOR_LEVEL_COUNT];  /**< 5级风扇博比表 */
    uint8_t reserved[3];                   /**< 对齐预留字节 */
    uint32_t crc32;                        /**< CRC32校验值（覆盖magic~reserved区域） */
} RuntimeConfigBlob_t;

/* ======================== 静态变量 ======================== */
static LogicThresholds_t s_factory_thresholds;     /**< 工厂默认阈值（Flash无效时回退用） */
static uint8_t s_factory_fan_levels[MOTOR_LEVEL_COUNT] = {0U, 25U, 50U, 75U, 100U}; /**< 工厂默认风扇档位表 */
static uint8_t s_factory_valid = 0U;               /**< 工厂默认值是否已设置 */
static volatile uint8_t s_save_pending = 0U;       /**< 延迟保存挂起标志 */
static volatile TickType_t s_last_save_request_tick = 0U; /**< 最后一次保存请求的时间戳 */


/* 下面 4 个变量用于“延迟保存 + 防并发/防重复写”的状态管理 */

/* 当前是否正在执行 Flash 保存（1=正在写Flash，0=空闲）
   用于防止 RuntimeConfig_Process() 在上一次保存尚未完成时再次触发保存。 */
static volatile uint8_t s_save_in_progress = 0U;

/* 保存请求版本号（每次 RuntimeConfig_RequestSave() 都会递增一次）
   相当于“保存请求序列号”，用于标记最新一次保存请求，便于去抖/合并多次请求。 */
static volatile uint32_t s_save_request_version = 0UL;

/* 当前正在处理（写 Flash）的那一次保存请求版本号
   在真正开始写 Flash 时从 s_save_request_version 快照得到，用于区分“正在保存哪一版”。 */
static volatile uint32_t s_save_active_version = 0UL;

/* 最近一次成功完成写 Flash 的保存请求版本号
   用于判断“最新请求是否已落盘”，避免重复写入，并在保存完成后清除 pending 标志等状态。 */
static volatile uint32_t s_save_completed_version = 0UL;

static uint32_t RuntimeConfig_NextRequestVersion(uint32_t current)
{
    current++;
    if (current == 0UL)
    {
        current = 1UL;
    }
    return current;
}

static uint8_t RuntimeConfig_TryBeginSave(uint32_t *version_out)
{
    uint8_t locked = 0U;
    uint32_t active_version = 0UL;

    taskENTER_CRITICAL();
    if (s_save_in_progress == 0U)
    {
        s_save_in_progress = 1U;
        s_save_active_version = s_save_request_version;
        active_version = s_save_active_version;
        locked = 1U;
    }
    taskEXIT_CRITICAL();

    if (version_out != NULL)
    {
        *version_out = active_version;
    }

    return locked;
}

static void RuntimeConfig_EndSave(uint8_t save_ok, uint32_t active_version)
{
    taskENTER_CRITICAL();
    if (save_ok != 0U)
    {
        s_save_completed_version = active_version;
        if (s_save_request_version == active_version)
        {
            s_save_pending = 0U;
        }
    }
    s_save_active_version = 0UL;
    s_save_in_progress = 0U;
    taskEXIT_CRITICAL();
}

/**
 * @brief   计算CRC32校验值
 *
 * 使用标准CRC32多项式 0xEDB88320（位反转）。
 *
 * @param   data  数据指针
 * @param   len   数据长度
 * @return  CRC32值
 */
static uint32_t RuntimeConfig_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t bit;
    uint32_t x;

    if (data == NULL)
    {
        return 0UL;
    }

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            x = crc & 1UL;
            crc >>= 1U;
            if (x != 0UL)
            {
                crc ^= 0xEDB88320UL;
            }
        }
    }

    return ~crc;
}

/** @brief 检查风扇档位表是否合法（每个值不超过100） */
static uint8_t RuntimeConfig_IsFanTableValid(const uint8_t fan_levels[MOTOR_LEVEL_COUNT])
{
    uint8_t i;

    if (fan_levels == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < MOTOR_LEVEL_COUNT; i++)
    {
        if (fan_levels[i] > 100U)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief   构建Flash数据块（填充魔数、版本、数据、CRC）
 */
static void RuntimeConfig_BuildBlob(RuntimeConfigBlob_t *blob,
                                    const LogicThresholds_t *thresholds,
                                    const uint8_t fan_levels[MOTOR_LEVEL_COUNT])
{
    if ((blob == NULL) || (thresholds == NULL) || (fan_levels == NULL))
    {
        return;
    }

    memset(blob, 0, sizeof(*blob));
    blob->magic = RUNTIME_CFG_MAGIC;
    blob->version = RUNTIME_CFG_VERSION;
    blob->thresholds = *thresholds;
    memcpy(blob->fan_levels, fan_levels, MOTOR_LEVEL_COUNT);
    blob->crc32 = RuntimeConfig_Crc32((const uint8_t *)blob, (uint32_t)offsetof(RuntimeConfigBlob_t, crc32));
}

/** @brief 校验Flash数据块是否有效（魔数+版本+风扇表+CRC） */
static uint8_t RuntimeConfig_IsBlobValid(const RuntimeConfigBlob_t *blob)
{
    uint32_t crc;

    if (blob == NULL)
    {
        return 0U;
    }

    if ((blob->magic != RUNTIME_CFG_MAGIC) || (blob->version != RUNTIME_CFG_VERSION))
    {
        return 0U;
    }

    if (!RuntimeConfig_IsFanTableValid(blob->fan_levels))
    {
        return 0U;
    }

    crc = RuntimeConfig_Crc32((const uint8_t *)blob, (uint32_t)offsetof(RuntimeConfigBlob_t, crc32));
    if (crc != blob->crc32)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief   将数据块写入Flash
 *
 * 工作流程：
 *   1. 对比旧数据，相同则跳过（避免无谓擦除）
 *   2. 挂起FreeRTOS调度器（Flash擦写期间AHB总线会被阻塞）
 *   3. 擦除 Sector 11 -> 按字（4字节）写入
 *   4. 恢复调度器
 *
 * @return  1=成功, 0=失败
 */
static uint8_t RuntimeConfig_FlashWriteBlob(const RuntimeConfigBlob_t *blob)
{
    const RuntimeConfigBlob_t *old_blob = (const RuntimeConfigBlob_t *)RUNTIME_CFG_FLASH_ADDR;
    const uint32_t *src;
    uint32_t *dst;
    uint32_t i;
    uint32_t word_count;
    FLASH_Status status;

    if (blob == NULL)
    {
        return 0U;
    }

    if (memcmp((const void *)old_blob, (const void *)blob, sizeof(RuntimeConfigBlob_t)) == 0)
    {
        return 1U;
    }

    /* Suspend scheduler so other tasks don't run during Flash erase/program
       (Flash controller stalls AHB during operations on STM32F4).           
       Interrupts stay enabled — UART ISRs still execute from Flash wait     
       states, but since they never call FreeRTOS API from ISR during        
       this window, this is safe.                                            */
    vTaskSuspendAll();

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    status = FLASH_EraseSector(RUNTIME_CFG_FLASH_SECTOR, VoltageRange_3);
    if (status != FLASH_COMPLETE)
    {
        FLASH_Lock();
        (void)xTaskResumeAll();
        return 0U;
    }

    src = (const uint32_t *)blob;
    dst = (uint32_t *)RUNTIME_CFG_FLASH_ADDR;
    word_count = (uint32_t)(sizeof(RuntimeConfigBlob_t) / sizeof(uint32_t));

    for (i = 0U; i < word_count; i++)
    {
        status = FLASH_ProgramWord((uint32_t)&dst[i], src[i]);
        if (status != FLASH_COMPLETE)
        {
            FLASH_Lock();
            (void)xTaskResumeAll();
            return 0U;
        }
    }

    FLASH_Lock();
    (void)xTaskResumeAll();
    return 1U;
}

/** @brief 应用工厂默认参数到逻辑模块和风扇模块 */
static void RuntimeConfig_ApplyFactoryDefaults(void)
{
    LogicThresholds_t defaults;

    if (s_factory_valid == 0U)
    {
        Logic_SetDefaultThresholds(&defaults);
        s_factory_thresholds = defaults;
        s_factory_fan_levels[0] = 0U;
        s_factory_fan_levels[1] = 25U;
        s_factory_fan_levels[2] = 50U;
        s_factory_fan_levels[3] = 75U;
        s_factory_fan_levels[4] = 100U;
        s_factory_valid = 1U;
    }

    Logic_SetRuntimeThresholds(&s_factory_thresholds);
    Motor_SetLevelTable(s_factory_fan_levels);
}

/**
 * @brief   设置工厂默认参数
 *
 * 在 RuntimeConfig_LoadAndApply 之前调用，设定当Flash无效时的回退值。
 */
void RuntimeConfig_SetFactoryDefaults(const LogicThresholds_t *thresholds,
                                      const uint8_t fan_levels[MOTOR_LEVEL_COUNT])
{
    LogicThresholds_t defaults;

    if (thresholds != NULL)
    {
        s_factory_thresholds = *thresholds;
    }
    else
    {
        Logic_SetDefaultThresholds(&defaults);
        s_factory_thresholds = defaults;
    }

    if ((fan_levels != NULL) && RuntimeConfig_IsFanTableValid(fan_levels))
    {
        memcpy(s_factory_fan_levels, fan_levels, MOTOR_LEVEL_COUNT);
    }
    else
    {
        s_factory_fan_levels[0] = 0U;
        s_factory_fan_levels[1] = 25U;
        s_factory_fan_levels[2] = 50U;
        s_factory_fan_levels[3] = 75U;
        s_factory_fan_levels[4] = 100U;
    }

    s_factory_valid = 1U;
}

/**
 * @brief   从Flash加载配置并应用
 *
 * 成功则将Flash中的阈值和风扇表应用到逻辑模块，
 * 失败则回退到工厂默认值并保存。
 *
 * @return  1=从Flash成功加载, 0=使用工厂默认值
 */
uint8_t RuntimeConfig_LoadAndApply(void)
{
    const RuntimeConfigBlob_t *blob = (const RuntimeConfigBlob_t *)RUNTIME_CFG_FLASH_ADDR;

    if (RuntimeConfig_IsBlobValid(blob))
    {
        Logic_SetRuntimeThresholds(&blob->thresholds);
        Motor_SetLevelTable(blob->fan_levels);
        return 1U;
    }

    RuntimeConfig_ApplyFactoryDefaults();
    (void)RuntimeConfig_SaveCurrent();
    return 0U;
}

/**
 * @brief   立即保存当前配置到Flash
 *
 * 读取当前逻辑阈值和风扇档位表，构建Blob并写入Flash。
 *
 * @return  1=成功, 0=失败
 */
uint8_t RuntimeConfig_SaveCurrent(void)
{
    RuntimeConfigBlob_t blob;
    LogicThresholds_t thresholds;
    const uint8_t *levels = Motor_GetLevelTable();
    uint8_t local_levels[MOTOR_LEVEL_COUNT];
    uint32_t active_version;
    uint8_t save_ok;

    if (!RuntimeConfig_TryBeginSave(&active_version))
    {
        RuntimeConfig_RequestSave();
        return 0U;
    }

    Logic_GetRuntimeThresholds(&thresholds);

    if ((levels != NULL) && RuntimeConfig_IsFanTableValid(levels))
    {
        memcpy(local_levels, levels, MOTOR_LEVEL_COUNT);
    }
    else
    {
        local_levels[0] = 0U;
        local_levels[1] = 25U;
        local_levels[2] = 50U;
        local_levels[3] = 75U;
        local_levels[4] = 100U;
    }

    RuntimeConfig_BuildBlob(&blob, &thresholds, local_levels);
    save_ok = RuntimeConfig_FlashWriteBlob(&blob);
    RuntimeConfig_EndSave(save_ok, active_version);
    return save_ok;
}

/**
 * @brief   请求延迟保存
 *
 * 记录当前时间戳和挂起标志，RuntimeConfig_Process 会在
 * 1秒后实际执行保存。这种去抖机制避免频繁擦写Flash。
 */
void RuntimeConfig_RequestSave(void)
{
    TickType_t now = 0U;

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        now = xTaskGetTickCount();
    }

    taskENTER_CRITICAL();
    s_save_pending = 1U;
    s_save_request_version = RuntimeConfig_NextRequestVersion(s_save_request_version);
    s_last_save_request_tick = now;
    taskEXIT_CRITICAL();
}

/**
 * @brief   周期性处理：延迟保存检查
 *
 * 若有挂起的保存请求且已超过1秒延迟，执行实际Flash写入。
 * 应在主控制任务中周期调用。
 */
void RuntimeConfig_Process(void)
{
    uint8_t pending;
    uint8_t save_busy;
    TickType_t req_tick;
    TickType_t now;
    TickType_t wait_ticks;

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return;
    }

    taskENTER_CRITICAL();
    pending = s_save_pending;
    save_busy = s_save_in_progress;
    req_tick = s_last_save_request_tick;
    taskEXIT_CRITICAL();

    if ((pending == 0U) || (save_busy != 0U))
    {
        return;
    }

    now = xTaskGetTickCount();
    wait_ticks = pdMS_TO_TICKS(RUNTIME_CFG_SAVE_DELAY_MS);
    if ((now - req_tick) < wait_ticks)
    {
        return;
    }

    (void)RuntimeConfig_SaveCurrent();
}

void RuntimeConfig_ResetToFactoryAndRequestSave(void)
{
    Logic_ResetAllToDefaultsManual();
    RuntimeConfig_ApplyFactoryDefaults();
    RuntimeConfig_RequestSave();
}

/**
 * @brief   恢复工厂设置并保存
 *
 * 先重置逻辑模块手动模式，再应用工厂默认值，最后写入Flash。
 */
void RuntimeConfig_ResetToFactoryAndSave(void)
{
    Logic_ResetAllToDefaultsManual();
    RuntimeConfig_ApplyFactoryDefaults();
    (void)RuntimeConfig_SaveCurrent();
}
