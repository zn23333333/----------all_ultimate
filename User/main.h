/**
 * @file    main.h
 * @brief   环境保障系统主头文件
 * @note    提供 STM32F4xx 库引用及全局函数声明
 */
  
/* 头文件保护宏，防止重复包含 */
#ifndef __MAIN_H
#define __MAIN_H

/* 包含 STM32F4 标准外设库主头文件 */
#include "stm32f4xx.h"


/* 导出函数声明 ------------------------------------------------ */
void TimingDelay_Decrement(void);  /* SysTick 延时递减回调（未使用，保留兼容） */

#endif /* __MAIN_H */

