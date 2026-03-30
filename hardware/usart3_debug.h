/**
 * @file    usart3_debug.h
 * @brief   USART3调试串口驱动（头文件）
 *
 * 基于STM32F407的USART3（PD8 TX / PD9 RX）实现调试串口，
 * 波特率115200bps。重定向 fputc 实现 printf 输出。
 *
 * 特点：
 *   - 禁用半主机模式（no-semihosting），无调试器也可正常工作
 *   - 256字节环形接收缓冲区，支持指令接收
 *   - 中断优先级7（最低优先级）
 */
#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx.h"
#include <stdint.h>

/* ======================== USART3硬件配置宏 ======================== */
#define USART_DBG                  USART3              /**< USART外设实例 */
#define USART_DBG_IRQn             USART3_IRQn          /**< USART3中断号 */
#define USART_DBG_BaudRate         115200U              /**< 调试串口波特率 */

#define USART_DBG_TX_PIN           GPIO_Pin_8           /**< TX引脚: PD8 */
#define USART_DBG_TX_PORT          GPIOD                /**< TX端口 */
#define USART_DBG_TX_CLK           RCC_AHB1Periph_GPIOD /**< TX GPIO时钟 */
#define USART_DBG_TX_PinSource     GPIO_PinSource8      /**< TX引脚复用源 */

#define USART_DBG_RX_PIN           GPIO_Pin_9           /**< RX引脚: PD9 */
#define USART_DBG_RX_PORT          GPIOD                /**< RX端口 */
#define USART_DBG_RX_CLK           RCC_AHB1Periph_GPIOD /**< RX GPIO时钟 */
#define USART_DBG_RX_PinSource     GPIO_PinSource9      /**< RX引脚复用源 */

#define USART_DBG_RX_BUF_SIZE      256U                 /**< 接收环形缓冲区大小 */

/** @brief 初始化USART3（GPIO、串口参数、接收中断） */
void Usart_Config(void);

/** @brief 发送单字节（阻塞等待TXE） */
void Usart_SendByte(uint8_t byte);

/** @brief 发送字节缓冲区 */
void Usart_SendBuffer(const uint8_t *buffer, uint16_t length);

/** @brief 发送以'\0'结尾的字符串 */
void Usart_SendString(const char *str);

/** @brief 清空接收环形缓冲区 */
void Usart_ClearRxBuffer(void);

/** @brief 查询可读字节数 */
uint16_t Usart_GetRxCount(void);

/** @brief 从环形缓冲区读取一个字节
 *  @return 1=成功, 0=缓冲区为空 */
uint8_t Usart_ReadByte(uint8_t *out);

#endif /* __USART_H */
