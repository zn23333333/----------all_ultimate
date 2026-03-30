/**
 * @file    uart2_esp01s.h
 * @brief   USART2串口驱动（ESP01S WiFi模块通信）
 *
 * 基于STM32F407的USART2（PA2 TX / PA3 RX）实现与 ESP01S WiFi模块
 * 的串口通信，默认波特率115200bps。
 *
 * 特点：
 *   - 中断接收 + 2KB环形缓冲区（适应WiFi模块的突发大数据量）
 *   - 缓冲区满时自动覆盖最旧数据（不丢弃新数据）
 *   - 提供字节、缓冲区、字符串发送接口
 */
#ifndef UART2_H
#define UART2_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== USART2硬件配置宏 ======================== */
#define UART2_PORT                 USART2              /**< USART外设实例 */
#define UART2_PORT_CLK             RCC_APB1Periph_USART2 /**< USART2时钟（APB1总线） */
#define UART2_IRQn_NAME            USART2_IRQn          /**< USART2中断号 */

#define UART2_TX_PIN               GPIO_Pin_2           /**< TX引脚: PA2 */
#define UART2_TX_PORT              GPIOA                /**< TX端口 */
#define UART2_TX_CLK               RCC_AHB1Periph_GPIOA /**< TX GPIO时钟 */
#define UART2_TX_PINSOURCE         GPIO_PinSource2      /**< TX引脚复用源 */

#define UART2_RX_PIN               GPIO_Pin_3           /**< RX引脚: PA3 */
#define UART2_RX_PORT              GPIOA                /**< RX端口 */
#define UART2_RX_CLK               RCC_AHB1Periph_GPIOA /**< RX GPIO时钟 */
#define UART2_RX_PINSOURCE         GPIO_PinSource3      /**< RX引脚复用源 */

#define UART2_DEFAULT_BAUDRATE     115200U              /**< 默认波特率 */
#define UART2_RX_BUFFER_SIZE       2048U                /**< 接收环形缓冲区大小（字节） */

/** @brief 初始化USART2（GPIO、串口参数、接收中断）
 *  @param  baudrate  波特率，0则使用默认值115200 */
void UART2_Init(uint32_t baudrate);

/** @brief 发送单字节（阻塞等待TXE） */
void UART2_SendByte(uint8_t data);

/** @brief 发送字节缓冲区 */
void UART2_SendBuffer(const uint8_t *data, uint16_t len);

/** @brief 发送以'\0'结尾的字符串 */
void UART2_SendString(const char *str);

/** @brief 查询环形缓冲区中可读字节数
 *  @return 可读字节数 */
uint16_t UART2_Available(void);

/** @brief 从环形缓冲区读取一个字节
 *  @param  out  输出字节指针
 *  @return 1=读取成功, 0=缓冲区为空 */
uint8_t UART2_ReadByte(uint8_t *out);

/** @brief 清空接收环形缓冲区 */
void UART2_ClearRxBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* UART2_H */
