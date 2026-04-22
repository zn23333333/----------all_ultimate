/**
 * @file    uart1_modbus.h
 * @brief   UART4 Modbus RTU通信驱动（头文件）
 *
 * 基于STM32F407的UART4（PC10/PC11）实现Modbus RTU主站通信，
 * 波特率9600，用于与Modbus从设备（传感器、继电器板）通信。
 *
 * 特点：
 *   - 中断接收 + 快照缓冲区（避免ISR与任务的数据竞争）
 *   - CRC16自动计算与校验
 *   - 根据功能码动态确定响应帧长度
 */
#ifndef SERIAL_H
#define SERIAL_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ======================== UART4硬件配置宏 ======================== */
#define SERIAL_USART            UART4                  /**< UART外设实例 */
#define SERIAL_USART_CLK        RCC_APB1Periph_UART4   /**< UART4时钟（挂在APB1总线） */
#define SERIAL_USART_IRQn       UART4_IRQn             /**< UART4中断号 */

#define SERIAL_BAUDRATE         9600                   /**< Modbus RTU波特率 */

#define SERIAL_TX_PIN           GPIO_Pin_10            /**< TX引脚: PC10 */
#define SERIAL_TX_PORT          GPIOC                  /**< TX端口: GPIOC */
#define SERIAL_TX_CLK           RCC_AHB1Periph_GPIOC   /**< TX GPIO时钟 */
#define SERIAL_TX_PinSource     GPIO_PinSource10       /**< TX引脚复用源 */

#define SERIAL_RX_PIN           GPIO_Pin_11            /**< RX引脚: PC11 */
#define SERIAL_RX_PORT          GPIOC                  /**< RX端口: GPIOC */
#define SERIAL_RX_CLK           RCC_AHB1Periph_GPIOC   /**< RX GPIO时钟 */
#define SERIAL_RX_PinSource     GPIO_PinSource11       /**< RX引脚复用源 */

typedef struct
{
    uint32_t rx_frame_count;   /**< Completed RX frames */
    uint32_t rx_ore_count;     /**< Overrun error count */
    uint32_t rx_fe_count;      /**< Framing error count */
    uint32_t rx_ne_count;      /**< Noise error count */
} SerialDiagSnapshot_t;

/** @brief 初始化UART4（GPIO、串口参数、接收中断） */
void Serial_Init(void);

/** @brief 发送单字节（阻塞等待发送完成） */
void Serial_SendByte(uint8_t byte);

/** @brief 发送字节数组 */
void Serial_SendArray(uint8_t *array, uint16_t length);

/** @brief 发送Modbus数据包（6字节数据 + 自动追加2字节CRC16） */
void Serial_SendPacket(uint8_t *packet);

/** @brief 检查是否收到完整帧，若是则拍快照到内部缓冲区
 *  @return 1=有新帧, 0=无数据 */
uint8_t Serial_GetRxFlag(void);

/** @brief 获取接收快照缓冲区指针（仅在 Serial_GetRxFlag()==1 后有效） */
uint8_t* Serial_GetRxBuffer(void);

/** @brief 清空接收缓冲区和状态标志 */
void Serial_ClearRxBuffer(void);

/** @brief 校验接收数据的CRC16
 *  @return 1=CRC正确, 0=CRC错误 */
uint8_t Serial_ValidateCRC(uint8_t *data, uint8_t length);

/** @brief 获取接收快照数据长度 */
uint8_t Serial_GetRxLength(void);

/** @brief 获取 UART4 接收诊断快照 */
void Serial_GetDiagSnapshot(SerialDiagSnapshot_t *out);

#endif /* SERIAL_H */
