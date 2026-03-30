/**
 * @file    delay.h
 * @brief   DWT硬件延时模块（头文件）
 *
 * 基于Cortex-M4内核的DWT周期计数器实现精确延时。
 * 不占用定时器资源，不影响FreeRTOS调度（但是忙等待）。
 * 主要用于I2C位模拟和硬件初始化时序。
 */
#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"
#include <stdint.h>

void Delay_DWT_Init(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_TaskMs(uint32_t ms);

#endif
