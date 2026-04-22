/**
 * @file    uart1_modbus.c
 * @brief   UART4 Modbus RTU通信驱动（实现）
 *
 * 通过UART4（PC10 TX / PC11 RX，9600bps）实现Modbus RTU主站通信。
 * 接收基于中断方式，根据功能码动态确定响应帧长度。
 *
 * 数据流：
 *   ISR接收 -> s_rx_buffer -> Serial_GetRxFlag()拍快照 -> s_rx_snapshot
 *   任务侧通过 Serial_GetRxBuffer() 读取快照，避免ISR数据竞争。
 */
#include "stm32f4xx.h"            // 主设备头文件
#include "stm32f4xx_rcc.h"        // 时钟管理相关头文件
#include "stm32f4xx_gpio.h"       // GPIO相关头文件
#include <string.h>               // memcpy

#include "uart1_modbus.h"

/* ======================== RS485方向控制引脚定义 ======================== */
/*
 * 默认按“RE与DE并在一起，由PA15统一控制”处理：
 * PA15 = 1 -> 发送模式
 * PA15 = 0 -> 接收模式
 */
#define RS485_DIR_CLK        RCC_AHB1Periph_GPIOA
#define RS485_DIR_PORT       GPIOA
#define RS485_DIR_PIN        GPIO_Pin_15

/* ======================== ISR接收缓冲区（volatile，仅ISR写入） ======================== */
static volatile uint8_t s_rx_buffer[64];      /**< ISR中断接收缓冲区 */
static volatile uint8_t s_rx_index = 0;       /**< 当前接收字节索引 */
static volatile uint8_t s_expected_length = 8;/**< 当前帧的期望长度（动态计算） */
static volatile uint8_t s_rx_complete = 0;    /**< 接收完成标志（1=有新帧） */
static volatile uint8_t s_rx_length = 0;      /**< 已接收帧的实际长度 */

/* 任务侧快照缓冲区：调用 Serial_GetRxFlag() 时从 ISR缓冲区拷贝而来 */
static uint8_t s_rx_snapshot[64];             /**< 接收帧快照（任务侧安全读取） */
static uint8_t s_rx_snapshot_length = 0;      /**< 快照帧长度 */

static volatile uint32_t s_rx_frame_count = 0U;
static volatile uint32_t s_rx_ore_count = 0U;
static volatile uint32_t s_rx_fe_count = 0U;
static volatile uint32_t s_rx_ne_count = 0U;

/**
 * @brief   初始化RS485方向控制脚
 */
static void RS485_DirInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RS485_DIR_CLK, ENABLE);

    GPIO_InitStruct.GPIO_Pin = RS485_DIR_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RS485_DIR_PORT, &GPIO_InitStruct);

    /* 默认进入接收模式 */
    GPIO_ResetBits(RS485_DIR_PORT, RS485_DIR_PIN);
}

/**
 * @brief   切换到485发送模式
 */
static __inline void RS485_SetTxMode(void)
{
    GPIO_SetBits(RS485_DIR_PORT, RS485_DIR_PIN);
}

/**
 * @brief   切换到485接收模式
 */
static __inline void RS485_SetRxMode(void)
{
    GPIO_ResetBits(RS485_DIR_PORT, RS485_DIR_PIN);
}

static uint8_t Serial_HandleRxErrors(void)
{
    uint16_t status = SERIAL_USART->SR;

    if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) == 0U)
    {
        return 0U;
    }

    if ((status & USART_SR_ORE) != 0U)
    {
        s_rx_ore_count++;
    }
    if ((status & USART_SR_FE) != 0U)
    {
        s_rx_fe_count++;
    }
    if ((status & USART_SR_NE) != 0U)
    {
        s_rx_ne_count++;
    }

    (void)SERIAL_USART->DR;
    s_rx_index = 0;
    s_expected_length = 8;
    s_rx_complete = 0;
    s_rx_length = 0;
    return 1U;
}

/**
 * @brief   计算Modbus RTU CRC16校验值
 *
 * 使用标准Modbus CRC16算法（多项式 0xA001）。
 *
 * @param   data    数据指针
 * @param   length  数据长度
 * @return  CRC16校验值（低字节在前）
 */
static uint16_t CalculateCRC16(uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

/**
 * @brief   初始化UART4串口
 *
 * 配置PC10(TX)/PC11(RX)为复用功能，设置9600bps，8N1，
 * 启用接收中断（RXNE）。中断优先级设为1（未调用FreeRTOS API，
 * 可高于 configMAX_SYSCALL 阈值）。
 */
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHB1PeriphClockCmd(SERIAL_TX_CLK | SERIAL_RX_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(SERIAL_USART_CLK, ENABLE);

    /* 初始化RS485方向控制脚 */
    RS485_DirInit();

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStruct.GPIO_Pin = SERIAL_TX_PIN;
    GPIO_Init(SERIAL_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = SERIAL_RX_PIN;
    GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStruct);

    GPIO_PinAFConfig(SERIAL_TX_PORT, SERIAL_TX_PinSource, GPIO_AF_UART4);
    GPIO_PinAFConfig(SERIAL_RX_PORT, SERIAL_RX_PinSource, GPIO_AF_UART4);

    USART_InitStruct.USART_BaudRate = SERIAL_BAUDRATE;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(SERIAL_USART, &USART_InitStruct);

    USART_ITConfig(SERIAL_USART, USART_IT_RXNE, ENABLE);
    USART_ITConfig(SERIAL_USART, USART_IT_ERR, ENABLE);
    NVIC_EnableIRQ(SERIAL_USART_IRQn);
    NVIC_SetPriority(SERIAL_USART_IRQn, 1);  /* ISR 不调用 FreeRTOS API，可高于 configMAX_SYSCALL 阈值 */

    USART_Cmd(SERIAL_USART, ENABLE);

    /* 串口初始化完成后，默认保持接收模式 */
    RS485_SetRxMode();
}

/**
 * @brief   发送单字节（阻塞）
 * @param   byte  待发送字节
 */
void Serial_SendByte(uint8_t byte)
{
    RS485_SetTxMode();

    USART_SendData(SERIAL_USART, byte);
    while (USART_GetFlagStatus(SERIAL_USART, USART_FLAG_TXE) == RESET);
    while (USART_GetFlagStatus(SERIAL_USART, USART_FLAG_TC) == RESET);

    RS485_SetRxMode();
}

/**
 * @brief   发送字节数组
 * @param   array   数据指针
 * @param   length  数据长度
 */
void Serial_SendArray(uint8_t *array, uint16_t length)
{
    if ((array == NULL) || (length == 0U))
    {
        return;
    }

    /* 发前切到发送模式 */
    RS485_SetTxMode();

    for (uint16_t i = 0; i < length; i++)
    {
        USART_SendData(SERIAL_USART, array[i]);
        while (USART_GetFlagStatus(SERIAL_USART, USART_FLAG_TXE) == RESET);
    }

    /* 一定要等待最后1字节真正发送完成 */
    while (USART_GetFlagStatus(SERIAL_USART, USART_FLAG_TC) == RESET);

    /* 发完切回接收模式，准备收从机响应 */
    RS485_SetRxMode();
}

/**
 * @brief   发送Modbus RTU数据包
 *
 * 将 packet[0..5] 的 6 字节数据自动计算 CRC16，
 * 并填充到 packet[6..7]，然后发送完整的 8 字节帧。
 *
 * @param   packet  8字节缓冲区，前6字节为有效数据
 */
void Serial_SendPacket(uint8_t *packet)
{
    uint16_t crc = CalculateCRC16(packet, 6);
    packet[6] = crc & 0xFF;
    packet[7] = (crc >> 8) & 0xFF;

    Serial_SendArray(packet, 8);
}

/**
 * @brief   检查是否收到完整Modbus响应帧
 *
 * 若 ISR 已置位 s_rx_complete，则禁止中断后将 ISR缓冲区
 * 拷贝到快照缓冲区（s_rx_snapshot），确保任务侧安全读取。
 *
 * @return  1=有新帧, 0=无数据
 */
uint8_t Serial_GetRxFlag(void)
{
    uint8_t len;

    if (!s_rx_complete)
    {
        return 0;
    }

    /* Take a snapshot of the received frame while IRQ is disabled
       so the ISR cannot modify the buffer concurrently. */
    NVIC_DisableIRQ(SERIAL_USART_IRQn);
    len = s_rx_length;
    if (len > sizeof(s_rx_snapshot))
    {
        len = sizeof(s_rx_snapshot);
    }
    memcpy(s_rx_snapshot, (const void *)s_rx_buffer, len);
    s_rx_snapshot_length = len;
    s_rx_complete = 0;
    NVIC_EnableIRQ(SERIAL_USART_IRQn);

    return 1;
}

/**
 * @brief   获取接收快照缓冲区指针
 * @return  快照缓冲区起始地址
 */
uint8_t* Serial_GetRxBuffer(void)
{
    return s_rx_snapshot;
}

/**
 * @brief   获取快照帧长度
 * @return  快照数据长度（字节）
 */
uint8_t Serial_GetRxLength(void)
{
    return s_rx_snapshot_length;
}

void Serial_GetDiagSnapshot(SerialDiagSnapshot_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    out->rx_frame_count = s_rx_frame_count;
    out->rx_ore_count = s_rx_ore_count;
    out->rx_fe_count = s_rx_fe_count;
    out->rx_ne_count = s_rx_ne_count;
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

/**
 * @brief   清空接收缓冲区和状态标志
 *
 * 禁止中断期间复位 ISR缓冲区索引、完成标志和期望长度。
 */
void Serial_ClearRxBuffer(void)
{
    NVIC_DisableIRQ(SERIAL_USART_IRQn);
    s_rx_index = 0;
    s_rx_complete = 0;
    s_expected_length = 8;
    s_rx_length = 0;
    NVIC_EnableIRQ(SERIAL_USART_IRQn);
}

/**
 * @brief   校验接收数据的CRC16
 * @param   data    数据指针（包含末尾2字节CRC）
 * @param   length  总长度（数据+CRC）
 * @return  1=CRC校验通过, 0=校验失败
 */
uint8_t Serial_ValidateCRC(uint8_t *data, uint8_t length)
{
    if (length < 2) return 0;

    uint16_t received_crc = (data[length - 1] << 8) | data[length - 2];
    uint16_t calculated_crc = CalculateCRC16(data, length - 2);

    return (received_crc == calculated_crc);
}

/**
 * @brief   UART4接收中断服务函数
 *
 * 工作流程：
 *   1. 将接收字节存入 s_rx_buffer
 *   2. 接收第2字节时，根据功能码确定期望长度：
 *      - 0x03/0x04（读寄存器）：先设5，等第3字节确定实际长度
 *      - 0x06（写单寄存器）：固定8字节
 *   3. 接收字节数达到期望长度时，置位完成标志
 */
void UART4_IRQHandler(void)
{
    uint8_t rx_data;

    if (Serial_HandleRxErrors() != 0U)
    {
        return;
    }

    if (USART_GetITStatus(SERIAL_USART, USART_IT_RXNE) == RESET)
    {
        return;
    }

    rx_data = (uint8_t)USART_ReceiveData(SERIAL_USART);

    if (s_rx_index < sizeof(s_rx_buffer))
    {
        s_rx_buffer[s_rx_index++] = rx_data;
    }
    else
    {
        s_rx_index = 0;
        s_rx_buffer[s_rx_index++] = rx_data;
    }

    if (s_rx_index == 2U)
    {
        if ((s_rx_buffer[1] & 0x80U) != 0U)
        {
            /* Modbus exception response:
               addr + func|0x80 + ex-code + CRC16 = 5 bytes */
            s_expected_length = 5U;
        }
        else
        {
            switch (s_rx_buffer[1])
            {
            case 0x04:
            case 0x03:
                s_expected_length = 5U;
                break;
            case 0x06:
                s_expected_length = 8U;
                break;
            default:
                s_expected_length = 8U;
                break;
            }
        }
    }

    if ((s_rx_index == 3U) && ((s_rx_buffer[1] == 0x03U) || (s_rx_buffer[1] == 0x04U)))
    {
        s_expected_length = (uint8_t)(5U + s_rx_buffer[2]);
    }

    if ((s_rx_index >= s_expected_length) && (s_rx_index > 0U))
    {
        s_rx_complete = 1U;
        s_rx_length = s_rx_index;
        s_rx_frame_count++;
        s_rx_index = 0U;
    }

    USART_ClearITPendingBit(SERIAL_USART, USART_IT_RXNE);
}