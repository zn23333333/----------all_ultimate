/**
 * @file    runtime_config.h
 * @brief   运行时配置Flash持久化模块（头文件）
 *
 * 将逻辑阈值（LogicThresholds_t）和风扇档位表保存到
 * STM32F407内部Flash Sector 11（0x080E0000，128KB）。
 * 支持CRC32校验、延迟保存、工厂默认值回退。
 */
#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <stdint.h>

#include "logic.h"
#include "motor_fan.h"

/** @brief 设置工厂默认参数（用于Flash无效时回退）
 *  @param  thresholds  阈值结构体，NULL则用内置默认值
 *  @param  fan_levels  5级风扇博比数组，NULL则用默认[0,25,50,75,100] */
void RuntimeConfig_SetFactoryDefaults(const LogicThresholds_t *thresholds,
                                      const uint8_t fan_levels[MOTOR_LEVEL_COUNT]);

/** @brief 从Flash加载配置并应用，若Flash无效则用工厂默认值并保存
 *  @return 1=从Flash加载成功, 0=使用工厂默认值 */
uint8_t RuntimeConfig_LoadAndApply(void);

/** @brief 立即将当前配置保存到Flash
 *  @return 1=保存成功, 0=失败 */
uint8_t RuntimeConfig_SaveCurrent(void);

/** @brief 请求延迟保存（在下次 RuntimeConfig_Process 时执行，延迟1秒） */
void RuntimeConfig_RequestSave(void);

/** @brief 周期性处理：检查是否有待保存的配置，延迟到期后执行实际保存 */
void RuntimeConfig_Process(void);

/** @brief 恢复工厂默认参数，并通过延迟保存机制写入 Flash */
void RuntimeConfig_ResetToFactoryAndRequestSave(void);

/** @brief 恢复工厂默认参数并立即写入Flash */
void RuntimeConfig_ResetToFactoryAndSave(void);

#endif /* RUNTIME_CONFIG_H */
