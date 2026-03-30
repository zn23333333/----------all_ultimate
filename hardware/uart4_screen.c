/**
 * @file    uart4_screen.c
 * @brief   UART4串口屏驱动实现
 *
 * 使用环形缓冲区（256字节）中断接收屏幕上传数据。
 * 支持可选的USART3 Hex镜像调试输出（UART_SCREEN_MIRROR_TO_USART3），
 * 开启后所有UART4收发数据会以Hex格式输出到调试串口。
 */
#include "uart4_screen.h"
#include "usart3_debug.h"

/** 镜像开关：1=将UART4收发以Hex输出到USART3调试口，0=关闭 */
#define UART_SCREEN_MIRROR_TO_USART3   0U

/* ======================== 环形缓冲区变量 ======================== */
static volatile uint8_t s_uart4_rx_buf[UART_SCREEN_RX_BUFFER_SIZE]; /**< 接收环形缓冲区 */
static volatile uint16_t s_uart4_rx_head = 0U;  /**< 写入位置（ISR更新） */
static volatile uint16_t s_uart4_rx_tail = 0U;  /**< 读取位置（任务侧更新） */

static volatile uint32_t s_uart4_rx_ore_count = 0U;
static volatile uint32_t s_uart4_rx_fe_count = 0U;
static volatile uint32_t s_uart4_rx_ne_count = 0U;

static uint8_t UART4_HandleRxErrors(void)
{
    uint16_t status = UART_SCREEN_PORT->SR;

    if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) == 0U)
    {
        return 0U;
    }

    if ((status & USART_SR_ORE) != 0U)
    {
        s_uart4_rx_ore_count++;
    }
    if ((status & USART_SR_FE) != 0U)
    {
        s_uart4_rx_fe_count++;
    }
    if ((status & USART_SR_NE) != 0U)
    {
        s_uart4_rx_ne_count++;
    }

    (void)UART_SCREEN_PORT->DR;
    return 1U;
}

#if UART_SCREEN_MIRROR_TO_USART3
static void UART_Screen_MirrorStart(uint8_t dir)
{
    static uint8_t s_last_dir = 0U;

    if (s_last_dir == dir)
    {
        return;
    }

    Usart_SendString("\r\n[UART4 ");
    if (dir == 1U)
    {
        Usart_SendString("TX] ");
    }
    else
    {
        Usart_SendString("RX] ");
    }
    s_last_dir = dir;
}

static void UART_Screen_MirrorHexByte(uint8_t dir, uint8_t data)
{
    static const char hex[] = "0123456789ABCDEF";

    UART_Screen_MirrorStart(dir);
    Usart_SendByte((uint8_t)hex[(data >> 4) & 0x0FU]);
    Usart_SendByte((uint8_t)hex[data & 0x0FU]);
    Usart_SendByte((uint8_t)' ');
}
#endif

/** @brief 环形缓冲区索引进位（到末尾回绕0） */
static uint16_t UART4_NextIndex(uint16_t index)
{
    index++;
    if (index >= UART_SCREEN_RX_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

/**
 * @brief   初始化UART4（GPIO复用、512000bps、接收中断）
 *
 * 中断优先级设为6（在 FreeRTOS configMAX_SYSCALL 范围内）。
 */
void UART_Screen_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef uart_init;

    RCC_AHB1PeriphClockCmd(UART_SCREEN_TX_CLK | UART_SCREEN_RX_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(UART_SCREEN_PORT_CLK, ENABLE);

    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init.GPIO_Pin = UART_SCREEN_TX_PIN;
    GPIO_Init(UART_SCREEN_TX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = UART_SCREEN_RX_PIN;
    GPIO_Init(UART_SCREEN_RX_PORT, &gpio_init);

    GPIO_PinAFConfig(UART_SCREEN_TX_PORT, UART_SCREEN_TX_PINSOURCE, GPIO_AF_UART4);
    GPIO_PinAFConfig(UART_SCREEN_RX_PORT, UART_SCREEN_RX_PINSOURCE, GPIO_AF_UART4);

    uart_init.USART_BaudRate = UART_SCREEN_DEFAULT_BAUDRATE;
    uart_init.USART_WordLength = USART_WordLength_8b;
    uart_init.USART_StopBits = USART_StopBits_1;
    uart_init.USART_Parity = USART_Parity_No;
    uart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    uart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART_SCREEN_PORT, &uart_init);

    UART_Screen_ClearRxBuffer();

    USART_ITConfig(UART_SCREEN_PORT, USART_IT_RXNE, ENABLE);
    USART_ITConfig(UART_SCREEN_PORT, USART_IT_ERR, ENABLE);
    NVIC_SetPriority(UART_SCREEN_IRQn_NAME, 6U);
    NVIC_EnableIRQ(UART_SCREEN_IRQn_NAME);

    USART_Cmd(UART_SCREEN_PORT, ENABLE);
}

/** @brief 发送单字节（阻塞等待TXE，可选镜像Hex到调试口） */
void UART_Screen_SendByte(uint8_t data)
{
    USART_SendData(UART_SCREEN_PORT, data);
    while (USART_GetFlagStatus(UART_SCREEN_PORT, USART_FLAG_TXE) == RESET)
    {
    }
#if UART_SCREEN_MIRROR_TO_USART3
    UART_Screen_MirrorHexByte(1U, data);
#endif
}

/** @brief 发送字节缓冲区 */
void UART_Screen_SendBuffer(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        UART_Screen_SendByte(data[i]);
    }
}

/** @brief 发送以'\0'结尾的字符串 */
void UART_Screen_SendString(const char *str)
{
    if (str == 0)
    {
        return;
    }

    while (*str != '\0')
    {
        UART_Screen_SendByte((uint8_t)*str);
        str++;
    }
}

/**
 * @brief   从环形缓冲区读取一个字节（可选镜像Hex到调试口）
 * @param   out  输出字节
 * @return  1=成功, 0=缓冲区为空
 */
uint8_t UART_Screen_ReadByte(uint8_t *out)
{
    uint16_t tail;

    if (out == 0)
    {
        return 0U;
    }

    tail = s_uart4_rx_tail;
    if (tail == s_uart4_rx_head)
    {
        return 0U;
    }

    *out = s_uart4_rx_buf[tail];
    s_uart4_rx_tail = UART4_NextIndex(tail);
#if UART_SCREEN_MIRROR_TO_USART3
    UART_Screen_MirrorHexByte(2U, *out);
#endif
    return 1U;
}

/** @brief 清空接收环形缓冲区（禁中断保护） */
void UART_Screen_ClearRxBuffer(void)
{
    __disable_irq();
    s_uart4_rx_head = 0U;
    s_uart4_rx_tail = 0U;
    __enable_irq();
}

/**
 * @brief   UART4接收中断服务函数
 *
 * 将接收字节写入环形缓冲区，缓冲区满时丢弃最旧数据。
 */
void UART4_IRQHandler(void)
{
    uint8_t data;
    uint16_t next;

    if (UART4_HandleRxErrors() != 0U)
    {
        return;
    }

    if (USART_GetITStatus(UART_SCREEN_PORT, USART_IT_RXNE) == RESET)
    {
        return;
    }

    data = (uint8_t)USART_ReceiveData(UART_SCREEN_PORT);
    next = UART4_NextIndex(s_uart4_rx_head);

    if (next == s_uart4_rx_tail)
    {
        s_uart4_rx_tail = UART4_NextIndex(s_uart4_rx_tail);
    }

    s_uart4_rx_buf[s_uart4_rx_head] = data;
    s_uart4_rx_head = next;

    USART_ClearITPendingBit(UART_SCREEN_PORT, USART_IT_RXNE);
}
