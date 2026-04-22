/**
 * @file    uart4_screen.h
 * @brief   UART5串口屏驱动（头文件）
 *
 * 基于STM32F407的UART5（PC12 TX / PD2 RX）实现与串口屏的通信，
 * 默认波特率512000bps（高速，适应屏幕协议需求）。
 *
 * 特点：
 *   - 256字节环形接收缓冲区（接收屏幕触摸/按键事件）
 *   - 可选USART3镜像Hex调试输出（UART_SCREEN_MIRROR_TO_USART3）
 */
#ifndef UART_SCREEN_H
#define UART_SCREEN_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== UART5硬件配置宏 ======================== */
#define UART_SCREEN_PORT               UART5                /**< UART外设实例 */
#define UART_SCREEN_PORT_CLK           RCC_APB1Periph_UART5 /**< UART5时钟（APB1总线） */
#define UART_SCREEN_IRQn_NAME          UART5_IRQn           /**< UART5中断号 */
#define UART_SCREEN_DEFAULT_BAUDRATE   512000U              /**< 默认波特率（512kbps高速） */
#define UART_SCREEN_RX_BUFFER_SIZE     256U                 /**< 接收环形缓冲区大小 */

#define UART_SCREEN_TX_PIN             GPIO_Pin_12          /**< TX引脚: PC12 */
#define UART_SCREEN_TX_PORT            GPIOC                /**< TX端口 */
#define UART_SCREEN_TX_CLK             RCC_AHB1Periph_GPIOC /**< TX GPIO时钟 */
#define UART_SCREEN_TX_PINSOURCE       GPIO_PinSource12     /**< TX引脚复用源 */

#define UART_SCREEN_RX_PIN             GPIO_Pin_2           /**< RX引脚: PD2 */
#define UART_SCREEN_RX_PORT            GPIOD                /**< RX端口 */
#define UART_SCREEN_RX_CLK             RCC_AHB1Periph_GPIOD /**< RX GPIO时钟 */
#define UART_SCREEN_RX_PINSOURCE       GPIO_PinSource2      /**< RX引脚复用源 */

/** @brief 初始化UART5（GPIO复用、串口参数、接收中断） */
void UART_Screen_Init(void);

/** @brief 发送单字节（阻塞等待TXE） */
void UART_Screen_SendByte(uint8_t data);

/** @brief 发送字节缓冲区 */
void UART_Screen_SendBuffer(const uint8_t *data, uint16_t len);

/** @brief 发送以'\0'结尾的字符串 */
void UART_Screen_SendString(const char *str);

/** @brief 从环形缓冲区读取一个字节（屏幕触摸/按键事件）
 *  @return 1=成功, 0=缓冲区为空 */
uint8_t UART_Screen_ReadByte(uint8_t *out);

/** @brief 清空接收环形缓冲区 */
void UART_Screen_ClearRxBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_SCREEN_H */
