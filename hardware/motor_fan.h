/**
 * @file    motor_fan.h
 * @brief   PWM风扇控制模块（头文件）
 *
 * 使用TIM3_CH1（PA6）输出10kHz PWM信号控制风扇转速。
 * 支持两种调速方式：
 *   1. 百分比直接设置（Motor_SetDutyPercent）
 *   2. 5级档位设置（Motor_SetLevel），每级对应可配置的博比值
 *
 * 风扇电源继电器：当博比>0%时自动闭合，0%时自动断开。
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ======================== PWM硬件配置宏 ======================== */
#define MOTOR_PWM_TIM                    TIM3                /**< 定时器实例 */
#define MOTOR_PWM_TIM_CLK                RCC_APB1Periph_TIM3  /**< TIM3时钟（APB1, 84MHz） */

#define MOTOR_PWM_GPIO_PORT              GPIOA               /**< PWM输出GPIO端口 */
#define MOTOR_PWM_GPIO_CLK               RCC_AHB1Periph_GPIOA /**< GPIO时钟 */
#define MOTOR_PWM_PIN                    GPIO_Pin_6           /**< PWM引脚: PA6 */
#define MOTOR_PWM_PINSOURCE              GPIO_PinSource6      /**< 引脚复用源 */
#define MOTOR_PWM_GPIO_AF                GPIO_AF_TIM3         /**< 复用功能: TIM3 */

/* 84MHz / 84 = 1MHz定时器时钟，1MHz / 100 = 10kHz PWM频率 */
#define MOTOR_PWM_PRESCALER              84U                  /**< 预分频系数 */
#define MOTOR_PWM_PERIOD                 100U                 /**< PWM周期（计数值） */

#define MOTOR_LEVEL_COUNT                5U                   /**< 风扇档位总数（0~4） */
/* WriteRelay()的索引从0开始，第6路继电器对应索引5 */
#define MOTOR_POWER_RELAY_INDEX          5U                   /**< 风扇电源继电器索引 */

/** @brief 初始化风扇PWM（GPIO+TIM3）并设为停止状态 */
void Motor_Init(void);

/** @brief 直接设置PWM博比（0~100%），并自动管理电源继电器 */
void Motor_SetDutyPercent(uint8_t duty_percent);

/** @brief 获取当前PWM博比（0~100%） */
uint8_t Motor_GetDutyPercent(void);

/** @brief 设置风扇档位（0~4），自动查表得到博比 */
void Motor_SetLevel(uint8_t level_index);

/** @brief 获取当前风扇档位索引 */
uint8_t Motor_GetLevel(void);

/** @brief 修改单个档位的博比值，若当前正在该档位则立即生效 */
void Motor_SetLevelValue(uint8_t level_index, uint8_t duty_percent);

/** @brief 批量设置全部档位的博比表，并按当前档位重新应用 */
void Motor_SetLevelTable(const uint8_t level_table[MOTOR_LEVEL_COUNT]);

/** @brief 获取当前档位表指针（只读） */
const uint8_t *Motor_GetLevelTable(void);

#endif /* __MOTOR_H */
