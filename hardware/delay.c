#include "delay.h"

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  ��ʼ�� DWT ���ڼ�����
 * @note   �����ȵ���һ�Σ������� main() ��ϵͳ��ʼ���
 */
void Delay_DWT_Init(void)
{
    /* ʹ�� DWT ���裨Cortex-M �ں˵���/׷��ģ�飩 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* ����/ʹ�� CYCCNT������� STM32F4 ����Ҫ�����Ĵ�����ֱ���ü��ɣ� */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  ΢����ʱ��æ�ȣ�
 * @param  us: ��ʱ΢����
 * @note   ���� SystemCoreClock ��ȷ��ͨ�� SystemInit ��� OK��
 */
void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    /* ���� CYCCNT ��������޷��ż�����Ȼ֧�ֻ��� */
    while ((DWT->CYCCNT - start) < ticks)
    {
        __NOP();
    }
}

/**
 * @brief  ������ʱ��æ�ȣ�
 * @param  ms: ��ʱ������
 */
void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}

/**
 * @brief  任务态毫秒延时
 * @param  ms: 延时毫秒数
 * @note   当 FreeRTOS 调度器已启动时，使用 vTaskDelay() 让出 CPU；
 *         对于小于 1 个 tick 的正毫秒数，自动向上取整为 1 tick，
 *         避免 pdMS_TO_TICKS() 截断后出现“0 tick 不延时”。
 *         调度器未启动或未处于 running 状态时，回退为阻塞式 DWT 延时。
 */
void Delay_TaskMs(uint32_t ms)
{
    TickType_t ticks;

    if (ms == 0U)
    {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        ticks = pdMS_TO_TICKS(ms);
        if (ticks == 0U)
        {
            ticks = 1U;
        }
        vTaskDelay(ticks);
        return;
    }

    Delay_ms(ms);
}
