/**
 * @file    uart2_esp01s.c
 * @brief   USART2串口驱动实现（ESP01S WiFi模块通信）
 *
 * 使用环形缓冲区（Ring Buffer）实现中断接收，缓冲区大小2KB。
 * 当缓冲区满时自动丢弃最旧数据（推进 tail），确保新数据始终可写入。
 *
 * 中断优先级设为6（在 FreeRTOS configMAX_SYSCALL 范围内）。
 */
#include "uart2_esp01s.h"

/* ======================== 环形缓冲区变量 ======================== */
static volatile uint8_t s_uart2_rx_buf[UART2_RX_BUFFER_SIZE]; /**< 接收环形缓冲区 */
static volatile uint16_t s_uart2_rx_head = 0U;  /**< 写入位置（ISR写入） */
static volatile uint16_t s_uart2_rx_tail = 0U;  /**< 读取位置（任务侧读取） */

static volatile uint32_t s_uart2_rx_ore_count = 0U;
static volatile uint32_t s_uart2_rx_fe_count = 0U;
static volatile uint32_t s_uart2_rx_ne_count = 0U;

static uint8_t UART2_HandleRxErrors(void)
{
    uint16_t status = UART2_PORT->SR;

    if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) == 0U)
    {
        return 0U;
    }

    if ((status & USART_SR_ORE) != 0U)
    {
        s_uart2_rx_ore_count++;
    }
    if ((status & USART_SR_FE) != 0U)
    {
        s_uart2_rx_fe_count++;
    }
    if ((status & USART_SR_NE) != 0U)
    {
        s_uart2_rx_ne_count++;
    }

    (void)UART2_PORT->DR;
    return 1U;
}

/**
 * @brief   环形缓冲区索引进位（到达末尾则回绕到0）
 * @param   index  当前索引
 * @return  下一个索引
 */
static uint16_t UART2_NextIndex(uint16_t index)
{
    index++;
    if (index >= UART2_RX_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

/**
 * @brief   初始化USART2（GPIO复用、串口参数、接收中断）
 * @param   baudrate  波特率，0则使用默认值115200
 */
void UART2_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;

    if (baudrate == 0U)
    {
        baudrate = UART2_DEFAULT_BAUDRATE;
    }

    RCC_AHB1PeriphClockCmd(UART2_TX_CLK | UART2_RX_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(UART2_PORT_CLK, ENABLE);

    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init.GPIO_Pin = UART2_TX_PIN;
    GPIO_Init(UART2_TX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = UART2_RX_PIN;
    GPIO_Init(UART2_RX_PORT, &gpio_init);

    GPIO_PinAFConfig(UART2_TX_PORT, UART2_TX_PINSOURCE, GPIO_AF_USART2);
    GPIO_PinAFConfig(UART2_RX_PORT, UART2_RX_PINSOURCE, GPIO_AF_USART2);

    usart_init.USART_BaudRate = baudrate;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART2_PORT, &usart_init);

    UART2_ClearRxBuffer();

    USART_ITConfig(UART2_PORT, USART_IT_RXNE, ENABLE);
    USART_ITConfig(UART2_PORT, USART_IT_ERR, ENABLE);
    NVIC_SetPriority(UART2_IRQn_NAME, 6U);
    NVIC_EnableIRQ(UART2_IRQn_NAME);

    USART_Cmd(UART2_PORT, ENABLE);
}

/** @brief 发送单字节（阻塞等待TXE标志） */
void UART2_SendByte(uint8_t data)
{
    USART_SendData(UART2_PORT, data);
    while (USART_GetFlagStatus(UART2_PORT, USART_FLAG_TXE) == RESET)
    {
    }
}

/** @brief 发送字节缓冲区 */
void UART2_SendBuffer(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        UART2_SendByte(data[i]);
    }
}

/** @brief 发送以'\0'结尾的字符串 */
void UART2_SendString(const char *str)
{
    if (str == 0)
    {
        return;
    }

    while (*str != '\0')
    {
        UART2_SendByte((uint8_t)*str);
        str++;
    }
}

/**
 * @brief   查询环形缓冲区中可读字节数
 * @return  当前可读字节数（head - tail，处理回绕）
 */
uint16_t UART2_Available(void)
{
    uint16_t head = s_uart2_rx_head;
    uint16_t tail = s_uart2_rx_tail;

    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(UART2_RX_BUFFER_SIZE - tail + head);
}

/**
 * @brief   从环形缓冲区读取一个字节
 * @param   out  输出字节
 * @return  1=读取成功, 0=缓冲区为空
 */
uint8_t UART2_ReadByte(uint8_t *out)
{
    uint16_t tail;

    if (out == 0)
    {
        return 0U;
    }

    tail = s_uart2_rx_tail;
    if (tail == s_uart2_rx_head)
    {
        return 0U;
    }

    *out = s_uart2_rx_buf[tail];
    s_uart2_rx_tail = UART2_NextIndex(tail);
    return 1U;
}

/** @brief 清空接收环形缓冲区（禁中断保护） */
void UART2_ClearRxBuffer(void)
{
    __disable_irq();
    s_uart2_rx_head = 0U;
    s_uart2_rx_tail = 0U;
    __enable_irq();
}

/**
 * @brief   USART2接收中断服务函数
 *
 * 将接收字节写入环形缓冲区。当缓冲区满时，
 * 自动推进 tail 指针丢弃最旧数据，确保新数据始终可写入。
 */
void USART2_IRQHandler(void)
{
    uint16_t next;
    uint8_t data;

    if (UART2_HandleRxErrors() != 0U)
    {
        return;
    }

    if (USART_GetITStatus(UART2_PORT, USART_IT_RXNE) == RESET)
    {
        return;
    }

    data = (uint8_t)USART_ReceiveData(UART2_PORT);
    next = UART2_NextIndex(s_uart2_rx_head);

    if (next == s_uart2_rx_tail)
    {
        s_uart2_rx_tail = UART2_NextIndex(s_uart2_rx_tail);
    }

    s_uart2_rx_buf[s_uart2_rx_head] = data;
    s_uart2_rx_head = next;

    USART_ClearITPendingBit(UART2_PORT, USART_IT_RXNE);
}
