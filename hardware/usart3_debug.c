/**
 * @file    usart3_debug.c
 * @brief   USART1调试串口驱动实现
 *
 * 功能：
 *   1. USART1 初始化（PA9 TX / PA10 RX，115200bps）
 *   2. 环形缓冲区中断接收（256字节）
 *   3. 重定向 fputc 实现 printf 输出到串口
 *   4. 禁用半主机模式（_sys_exit / _ttywrch / ferror 桩函数）
 */
#include "usart3_debug.h"

/* 禁用半主机模式，使 printf 在无调试器时也可正常工作 */
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };  /**< 最小化的FILE结构体 */
FILE __stdout;                  /**< 标准输出 */
FILE __stdin;                   /**< 标准输入 */

/* ======================== 环形接收缓冲区 ======================== */
static volatile uint8_t s_rx_buf[USART_DBG_RX_BUF_SIZE]; /**< 接收环形缓冲区 */
static volatile uint16_t s_rx_head = 0U;  /**< 写入位置（ISR更新） */
static volatile uint16_t s_rx_tail = 0U;  /**< 读取位置（任务侧更新） */

static volatile uint32_t s_rx_ore_count = 0U;
static volatile uint32_t s_rx_fe_count = 0U;
static volatile uint32_t s_rx_ne_count = 0U;

static uint8_t Usart_HandleRxErrors(void)
{
    uint16_t status = USART_DBG->SR;

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

    (void)USART_DBG->DR;
    return 1U;
}

/** @brief 环形缓冲区索引进位 */
static uint16_t Usart_RxNext(uint16_t idx)
{
    idx++;
    if (idx >= USART_DBG_RX_BUF_SIZE)
    {
        idx = 0U;
    }
    return idx;
}

/** @brief 配置USART1的GPIO引脚（PA9 TX / PA10 RX，复用功能） */
static void Usart_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_AHB1PeriphClockCmd(USART_DBG_TX_CLK | USART_DBG_RX_CLK, ENABLE);

    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init.GPIO_Pin = USART_DBG_TX_PIN;
    GPIO_Init(USART_DBG_TX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = USART_DBG_RX_PIN;
    GPIO_Init(USART_DBG_RX_PORT, &gpio_init);

    GPIO_PinAFConfig(USART_DBG_TX_PORT, USART_DBG_TX_PinSource, GPIO_AF_USART1);
    GPIO_PinAFConfig(USART_DBG_RX_PORT, USART_DBG_RX_PinSource, GPIO_AF_USART1);
}

/**
 * @brief   初始化USART1调试串口
 *
 * 配置GPIO复用、串口参数（115200bps, 8N1）、接收中断。
 * 中断优先级设为7（最低，调试口不影响关键业务）。
 */
void Usart_Config(void)
{
    USART_InitTypeDef usart_init;

    RCC_APB2PeriphClockCmd(USART_DBG_CLK, ENABLE);

    Usart_GPIO_Config();

    usart_init.USART_BaudRate = USART_DBG_BaudRate;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART_DBG, &usart_init);

    USART_ITConfig(USART_DBG, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART_DBG, USART_IT_ERR, ENABLE);
    NVIC_SetPriority(USART_DBG_IRQn, 7U);
    NVIC_EnableIRQ(USART_DBG_IRQn);

    USART_Cmd(USART_DBG, ENABLE);
}

/** @brief 发送单字节（阻塞等待TXE） */
void Usart_SendByte(uint8_t byte)
{
    USART_SendData(USART_DBG, byte);
    while (USART_GetFlagStatus(USART_DBG, USART_FLAG_TXE) == RESET)
    {
    }
}

/** @brief 发送字节缓冲区 */
void Usart_SendBuffer(const uint8_t *buffer, uint16_t length)
{
    uint16_t i;

    if (buffer == 0)
    {
        return;
    }

    for (i = 0U; i < length; i++)
    {
        Usart_SendByte(buffer[i]);
    }
}

/** @brief 发送以'\0'结尾的字符串 */
void Usart_SendString(const char *str)
{
    if (str == 0)
    {
        return;
    }

    while (*str != '\0')
    {
        Usart_SendByte((uint8_t)(*str));
        str++;
    }
}

/** @brief 清空接收环形缓冲区（禁中断保护） */
void Usart_ClearRxBuffer(void)
{
    __disable_irq();
    s_rx_tail = s_rx_head;
    __enable_irq();
}

/** @brief 查询环形缓冲区中可读字节数（处理回绕） */
uint16_t Usart_GetRxCount(void)
{
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(USART_DBG_RX_BUF_SIZE - tail + head);
}

/**
 * @brief   从环形缓冲区读取一个字节
 * @param   out  输出字节
 * @return  1=成功, 0=缓冲区为空
 */
uint8_t Usart_ReadByte(uint8_t *out)
{
    uint16_t tail = s_rx_tail;

    if (out == 0)
    {
        return 0U;
    }

    if (tail == s_rx_head)
    {
        return 0U;
    }

    *out = s_rx_buf[tail];
    s_rx_tail = Usart_RxNext(tail);
    return 1U;
}

/**
 * @brief   USART1接收中断服务函数
 *
 * 将接收字节写入环形缓冲区，缓冲区满时丢弃最旧数据。
 */
void USART1_IRQHandler(void)
{
    if (Usart_HandleRxErrors() != 0U)
    {
        return;
    }

    if (USART_GetITStatus(USART_DBG, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART_DBG);
        uint16_t next = Usart_RxNext(s_rx_head);

        if (next == s_rx_tail)
        {
            s_rx_tail = Usart_RxNext(s_rx_tail);
        }

        s_rx_buf[s_rx_head] = data;
        s_rx_head = next;

        USART_ClearITPendingBit(USART_DBG, USART_IT_RXNE);
    }
}

/**
 * @brief   重定向 fputc 实现 printf 输出到USART1
 *
 * 这是 C 标准库的底层输出函数，每个 printf 字符都会调用此函数。
 */
int fputc(int c, FILE *fp)
{
    (void)fp;
    Usart_SendByte((uint8_t)c);
    return c;
}

/** @brief 桩函数：错误处理（禁用半主机要求） */
int ferror(FILE *f)
{
    (void)f;
    return EOF;
}

/** @brief 桩函数：字符输出（禁用半主机要求） */
void _ttywrch(int ch)
{
    Usart_SendByte((uint8_t)ch);
}

/** @brief 桩函数：系统退出（死循环，禁用半主机要求） */
void _sys_exit(int x)
{
    (void)x;
    while (1)
    {
    }
}
