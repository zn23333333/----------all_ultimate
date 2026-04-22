/**
 * @file    motor_fan.c
 * @brief   PWM风扇控制模块实现
 *
 * 功能概述：
 *   1. TIM1_CH4（PE14）输出10kHz PWM信号控制风扇转速
 *   2. 5级档位查表（默认0/25/50/75/100%），支持运行时修改
 *   3. 自动管理电源继电器（第6路）：博比>0闭合，博比=0断开
 *   4. 使用FreeRTOS临界区保护共享变量（多任务安全）
 */
#include "motor_fan.h"
#include "Modbus_Relay.h"
#include "FreeRTOS.h"
#include "task.h"

/* ======================== 静态变量 ======================== */
static uint8_t s_level_table[MOTOR_LEVEL_COUNT] = {0U, 25U, 50U, 75U, 100U}; /**< 档位博比查找表（%），可运行时修改 */
static uint8_t s_current_level = 0U;          /**< 当前档位索引（0~4） */
static uint8_t s_current_duty_percent = 0U;    /**< 当前实际博比（0~100%） */
static uint8_t s_relay_state_valid = 0U;       /**< 继电器状态是否已知 */
static uint8_t s_relay_state = 0U;             /**< 继电器当前状态（1=闭合, 0=断开） */

/** @brief 博比值限幅（不超过100%） */
static uint8_t Motor_ClampDuty(uint8_t duty_percent)
{
    if (duty_percent > 100U)
    {
        return 100U;
    }
    return duty_percent;
}

/**
 * @brief   根据目标博比自动控制电源继电器
 *
 * 博比>0：闭合继电器供电；博比=0：断开继电器断电。
 * 含状态缓存，避免重复发送Modbus命令。
 */
static void Motor_UpdatePowerRelay(uint8_t duty_percent)
{
    uint8_t target_state = (duty_percent > 0U) ? 1U : 0U;

    if ((s_relay_state_valid != 0U) && (s_relay_state == target_state))
    {
        return;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return;
    }

    if (WriteRelay(MOTOR_POWER_RELAY_INDEX, target_state))
    {
        s_relay_state = target_state;
        s_relay_state_valid = 1U;
    }
}

/** @brief 初始化PWM输出GPIO（PE14复用为TIM1_CH4） */
static void Motor_PWM_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_AHB1PeriphClockCmd(MOTOR_PWM_GPIO_CLK, ENABLE);

    gpio_init.GPIO_Pin = MOTOR_PWM_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_PWM_GPIO_PORT, &gpio_init);

    GPIO_PinAFConfig(MOTOR_PWM_GPIO_PORT, MOTOR_PWM_PINSOURCE, MOTOR_PWM_GPIO_AF);
}

/**
 * @brief   初始化TIM1 PWM输出
 *
 * 配置TIM1为向上计数模式，PWM1模式，
 * 频率: 168MHz / 168(PSC) / 100(ARR) = 10kHz。
 * 初始博比为0%（风扇停止）。
 */
static void Motor_PWM_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base_init;
    TIM_OCInitTypeDef tim_oc_init;

    RCC_APB2PeriphClockCmd(MOTOR_PWM_TIM_CLK, ENABLE);

    TIM_TimeBaseStructInit(&tim_base_init);
    tim_base_init.TIM_Prescaler = (uint16_t)(MOTOR_PWM_PRESCALER - 1U);
    tim_base_init.TIM_Period = (uint32_t)(MOTOR_PWM_PERIOD - 1U);
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(MOTOR_PWM_TIM, &tim_base_init);

    TIM_OCStructInit(&tim_oc_init);
    tim_oc_init.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_oc_init.TIM_Pulse = 0U;
    TIM_OC4Init(MOTOR_PWM_TIM, &tim_oc_init);
    TIM_OC4PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(MOTOR_PWM_TIM, ENABLE);
    TIM_CtrlPWMOutputs(MOTOR_PWM_TIM, ENABLE);
    TIM_Cmd(MOTOR_PWM_TIM, ENABLE);
}

/** @brief 初始化风扇模块（GPIO + TIM1），启动0档（停止） */
void Motor_Init(void)
{
    Motor_PWM_GPIO_Init();
    Motor_PWM_TIM_Init();
    Motor_SetLevel(0U);
}

/**
 * @brief   设置PWM博比（0~100%）
 *
 * 在临界区内更新博比值和TIM1_CH4比较寄存器，
 * 然后更新电源继电器状态。
 *
 * @param   duty_percent  空占比百分比（0~100）
 */
void Motor_SetDutyPercent(uint8_t duty_percent)
{
    uint32_t pulse;

    taskENTER_CRITICAL();
    s_current_duty_percent = Motor_ClampDuty(duty_percent);
    pulse = ((uint32_t)MOTOR_PWM_PERIOD * (uint32_t)s_current_duty_percent) / 100U;
    TIM_SetCompare4(MOTOR_PWM_TIM, (uint16_t)pulse);
    taskEXIT_CRITICAL();
    Motor_UpdatePowerRelay(s_current_duty_percent);
}

/** @brief 获取当前PWM博比百分比 */
uint8_t Motor_GetDutyPercent(void)
{
    return s_current_duty_percent;
}

/**
 * @brief   设置风扇档位（0~4）
 *
 * 从档位表查找对应博比，在临界区内更新档位索引，
 * 然后调用 Motor_SetDutyPercent 应用。
 *
 * @param   level_index  档位索引（0~4）
 */
void Motor_SetLevel(uint8_t level_index)
{
    uint8_t duty;

    if (level_index >= MOTOR_LEVEL_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_current_level = level_index;
    duty = s_level_table[level_index];
    taskEXIT_CRITICAL();
    Motor_SetDutyPercent(duty);
}

/** @brief 获取当前档位索引 */
uint8_t Motor_GetLevel(void)
{
    return s_current_level;
}

/**
 * @brief   修改单个档位的博比值
 *
 * 如果当前风扇正好在该档位，修改后立即生效。
 *
 * @param   level_index    档位索引（0~4）
 * @param   duty_percent   新的博比值（0~100）
 */
void Motor_SetLevelValue(uint8_t level_index, uint8_t duty_percent)
{
    uint8_t value;
    uint8_t need_update;

    if (level_index >= MOTOR_LEVEL_COUNT)
    {
        return;
    }

    value = Motor_ClampDuty(duty_percent);

    taskENTER_CRITICAL();
    s_level_table[level_index] = value;
    need_update = (s_current_level == level_index) ? 1U : 0U;
    taskEXIT_CRITICAL();

    if (need_update)
    {
        Motor_SetDutyPercent(value);
    }
}

/**
 * @brief   批量设置全部档位博比表
 *
 * 在临界区内更新全部档位值，然后按当前档位重新应用博比。
 *
 * @param   level_table  5个博比值的数组指针
 */
void Motor_SetLevelTable(const uint8_t level_table[MOTOR_LEVEL_COUNT])
{
    uint8_t i;
    uint8_t duty;

    if (level_table == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    for (i = 0U; i < MOTOR_LEVEL_COUNT; i++)
    {
        s_level_table[i] = Motor_ClampDuty(level_table[i]);
    }
    duty = s_level_table[s_current_level];
    taskEXIT_CRITICAL();

    Motor_SetDutyPercent(duty);
}

/** @brief 获取当前档位博比表指针（只读） */
const uint8_t *Motor_GetLevelTable(void)
{
    return s_level_table;
}
