/**
 * @file    main.c
 * @brief   环境保障系统 —— 主入口文件
 *
 * @details
 *   本项目基于 STM32F407ZGT6 + FreeRTOS V11 构建了一套环境保障系统，
 *   包含以下子系统：
 *     - Modbus RTU 有线传感器（温湿度、PM2.5/PM10、气体）
 *     - ESP01S WiFi 模块，通过 MQTT 进行远程监控与控制
 *     - 两个esp32无线传感器（人体检测、光照、门磁）经 MQTT/UDP 上报
 *     - 8 路继电器(实用6路)控制（加热器、制冷器、加湿器、LED灯光、声光报警器、风扇电源）
 *     - PWM 风扇 5 挡调速
 *     - 串口屏（UART5）实时显示传感器数据与设备状态，并且提供本地按键控制界面
 *     - OLED 屏（I2C, PB8/PB9）本地循环显示
 *     - 运行参数持久化到片内 Flash
 *
 *   FreeRTOS 任务分配：
 *     DisplayCycleTask  : OLED 轮播（温湿度 → PM → 气体 → 继电器状态）
 *     ControlTask       : 传感器轮询 + 自动控制逻辑 + 参数持久化
 *     Task_PG14         : 心跳 LED 闪烁指示系统运行
 *     Screen_Task       : 串口屏数据刷新与用户交互处理
 *     MqttApp_Task      : WiFi/MQTT 连接管理与远程数据收发
 */

/* ======================== 头文件包含 ======================== */
#include "stm32f4xx.h"           /* STM32F4 外设库主头文件 */
#include "FreeRTOS.h"            /* FreeRTOS 内核头文件 */
#include "task.h"                /* FreeRTOS 任务管理 API */
#include <stdio.h>
#include <string.h>

#include "delay.h"               /* DWT 精确延时（微秒/毫秒） */
#include "OLED.h"                /* 0.96" OLED 显示驱动（I2C, PB8-SCL, PB9-SDA） */
#include "uart1_modbus.h"        /* UART4 Modbus RTU 底层驱动 (PC10/PC11, 9600bps) */
#include "key.h"                 /* 按键驱动 (PA15) */
#include "Modbus_Relay.h"        /* Modbus RTU 继电器/传感器读写封装 */
#include "all_sensor_data.h"     /* 传感器数据集中管理（有线 + 无线） */
#include "logic.h"               /* 自动控制逻辑（温度/湿度/光照/报警/风扇） */
#include "motor_fan.h"           /* PWM 风扇控制驱动 (TIM1_CH4, PE14) */
#include "usart3_debug.h"        /* USART1 调试串口 (PA9/PA10, 115200bps) */
#include "esp01s.h"              /* ESP01S WiFi/MQTT 驱动（AT 指令层） */
#include "screen.h"              /* 串口屏逻辑层 */
#include "uart4_screen.h"        /* UART5 串口屏底层驱动 (PC12/PD2, 512000bps) */
#include "runtime_config.h"      /* 运行参数 Flash 持久化模块 */
#include "mqtt_app.h"            /* MQTT 应用层任务（远程监控与命令处理） */
#include "diag_task.h"            /* UART3 诊断输出任务（仅日志，不改业务） */

/* ======================== 编译开关宏 ======================== */
#define ESP01S_TEST_ENABLE           1U   /**< 1=启用 MQTT 任务, 0=禁用（调试用） */
#define UART4_TX_TEST_ENABLE         0U   /**< 1=启用串口屏发送测试任务（仅调试）, 0=正常屏幕任务 */
#define CONTROL_TASK_STACK_WORDS     896U /**< Control 任务栈大小（单位：字，即 3584 字节） */
#define SCREEN_TASK_STACK_WORDS      768U /**< Screen 任务栈大小（单位：字，即 3072 字节） */
#define MQTT_TASK_STACK_WORDS        1152U /**< MQTT 任务栈大小（单位：字，即 4608 字节） */
#define DIAG_TASK_STACK_WORDS        256U  /**< DIAG 任务栈大小（单位：字，即 1024 字节） */

/* ================================================================
 *  FreeRTOS 栈溢出检测钩子回调
 *  当 configCHECK_FOR_STACK_OVERFLOW == 2 时，内核在每次任务切换时
 *  检查栈底的填充标记是否被破坏，如果被破坏则调用此回调函数。
 *  这里打印出任务名称后关中断死循环，方便调试定位。
 * ================================================================ */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("ERR: stack overflow in task: %s\r\n", pcTaskName ? pcTaskName : "?");
    taskDISABLE_INTERRUPTS();  /* 关闭所有中断，防止进一步破坏 */
    for (;;)
    {
    }  /* 死循环挂起，等待调试器介入 */
}

/* ================================================================
 *  OLED 显示轮播任务
 *  每隔 2 秒切换一页，依次展示：
 *    第 0 页：温度 & 湿度
 *    第 1 页：PM2.5 & PM10
 *    第 2 页：气体浓度
 *    第 3 页：继电器通道状态
 * ================================================================ */
void DisplayCycleTask(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();  /* 用于精确周期延时 */
    uint8_t page = 0;                            /* 当前显示页码 */
    (void)arg;

    while (1)
    {
        switch (page)
        {
        case 0:
            DisplayTemperatureHumidity();   /* 温湿度页 */
            break;
        case 1:
            DisplayPM25PM10();              /* 粉尘浓度页 */
            break;
        case 2:
            DisplayGas();                   /* 气体浓度页 */
            break;
        default:
            DisplayRelayStatus();           /* 继电器状态页 */
            break;
        }

        page = (page + 1) % 4;              /* 循环切页 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2000));  /* 精确 2 秒周期 */
    }
}

/**
 * @brief  构建项目出厂默认阈值
 * @param  thresholds  输出参数，接收构建好的阈值结构体
 * @note   先载入 Logic 模块的通用默认值，再覆盖为本项目自定义的出厂参数。
 *         这些值在首次烧录时会被写入 Flash Sector 11，后续可通过
 *         串口屏或 MQTT 在线修改。
 */
static void App_BuildProjectFactoryThresholds(LogicThresholds_t *thresholds)
{
    if (thresholds == NULL)
    {
        return;
    }

    Logic_SetDefaultThresholds(thresholds);      /* 先填入通用默认值 */
    
    /* ---------- 这下面的只改传入的*thresholds里的值，默认的参数在Logic_SetDefaultThresholds()里 ---------- */

    /* ---------- 温控阈值 ---------- */
    thresholds->temp_low = 16.0f;                /* 低于此温度 → 开启加热器 */
    thresholds->temp_high = 25.0f;               /* 高于此温度 → 开启制冷器 */

    /* ---------- 湿度阈值 ---------- */
    thresholds->humidity_low = 50.0f;            /* 低于此湿度 → 开启加湿器 */
    thresholds->humidity_fan_high = 80.0f;       /* 高于此湿度 → 启动除湿风扇 */
    thresholds->humidity_fan_level = 1U;         /* 除湿时风扇档位（索引 1 = 第 2 档） */

    /* ---------- 光照 / 气体 / 粉尘 报警阈值 ---------- */
    thresholds->light_low = 50U;                 /* 光照低于 50 lux → 开灯 */
    thresholds->gas_high = 50U;                  /* 气体浓度超标阈值 */
    thresholds->pm25_high = 50U;                 /* PM2.5 超标阈值 */

    /* ---------- PM2.5 → 风扇档位分段上限 ---------- */
    thresholds->pm25_fan_upper[0] = 50U;         /* 1 档上限 */
    thresholds->pm25_fan_upper[1] = 100U;        /* 2 档上限 */
    thresholds->pm25_fan_upper[2] = 200U;        /* 3 档上限 */
    thresholds->pm25_fan_upper[3] = 400U;        /* 4 档上限 */
    thresholds->pm25_fan_upper[4] = 999U;       /* 5 档上限（兜底） */

    /* ---------- 气体浓度 → 风扇档位分段上限 ---------- */
    thresholds->gas_fan_upper[0] = 20U;
    thresholds->gas_fan_upper[1] = 40U;
    thresholds->gas_fan_upper[2] = 80U;
    thresholds->gas_fan_upper[3] = 160U;
    thresholds->gas_fan_upper[4] = 999U;

    /* ---------- 报警延时（设为 0 表示立即响应） ---------- */
    thresholds->door_open_alarm_ms = 0UL * 60UL * 1000UL;   /* 门未关报警延时 */
    thresholds->gas_high_alarm_ms = 0UL * 60UL * 1000UL;    /* 气体超标报警延时 */
    thresholds->pm25_high_alarm_ms = 0UL * 60UL * 1000UL;   /* PM2.5 超标报警延时 */

    /* ---------- 迟滞量（防止设备在临界值附近反复开关） ---------- */
    thresholds->hyst_temp = 1.0f;                /* 温度迟滞 ±1°C */
    thresholds->hyst_humi = 5.0f;                /* 湿度迟滞 ±5%RH */
    thresholds->hyst_light = 20U;                /* 光照迟滞 20 lux */
    thresholds->hyst_gas = 5U;                   /* 气体迟滞 */
    thresholds->hyst_pm25 = 5U;                  /* PM2.5 迟滞 */

    /* ---------- 设备最小切换间隔（30 秒） ---------- */
    thresholds->min_switch_ms = 30000UL;
}

/**
 * @brief  控制任务 —— 系统核心循环
 * @note   每秒执行一次以下流程：
 *         1. 通过 Modbus RTU 轮询有线传感器（温湿度、PM、气体）
 *         2. 检查继电器模块通信状态
 *         3. 获取全部传感器数据快照
 *         4. 读取当前运行阈值
 *         5. 执行自动控制逻辑（温控、湿控、灯光、报警、风扇）
 *         6. 处理延迟写入 Flash 的参数保存请求
 */
void ControlTask(void *arg)
{
    AllSensorData_t sensor_data;      /* 传感器数据快照 */
    LogicThresholds_t thresholds;     /* 当前运行阈值 */

    (void)arg;

    (void)AllSensorData_RefreshModbus();  /* 启动后先做一次完整传感器轮询 */

    while (1)
    {
        (void)AllSensorData_RefreshModbus();     /* 1. 轮询 Modbus 传感器（包括连接状态检查） */
        MqttApp_PollRelayLink();                 /* 2. 检查继电器板通信链路（只是因为标志位是mqtt要用的，所以在mqtt里写） */
        AllSensorData_GetSnapshot(&sensor_data); /* 3. 获取数据快照（线程安全） */
        Logic_GetRuntimeThresholds(&thresholds); /* 4. 获取运行阈值（临界区保护） */
        Logic_Run(&sensor_data, &thresholds);    /* 5. 执行自动控制决策 */
        RuntimeConfig_Process();                 /* 6. 通过标志位判断是否要保存参数到 Flash，当前每秒保存一次 */

        vTaskDelay(pdMS_TO_TICKS(1000));         /* 1 秒控制周期 */
    }
}

/**
 * @brief  LED GPIO 初始化
 * @note   PG14 配置为推挽输出，默认低电平（LED 亮）。
 *         PG14 用作系统心跳指示灯
 */
static void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);  /* 使能 GPIOG 时钟 */

    gpio.GPIO_Pin = GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_OUT;       /* 通用输出 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;      /* 推挽输出 */
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;    /* 无上下拉 */
    GPIO_Init(GPIOG, &gpio);

    GPIO_ResetBits(GPIOG, GPIO_Pin_14);  /* 初始输出低电平 */
}

/**
 * @brief  心跳 LED 任务
 * @note   每 500ms 翻转 PG14 电平，用于指示系统正常运行。
 *         优先级最低（1），不影响业务任务。
 */
static void Task_PG14(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        GPIO_ToggleBits(GPIOG, GPIO_Pin_14);   /* 翻转 LED 电平 */
        vTaskDelay(pdMS_TO_TICKS(500));         /* 500ms 闪烁周期 */
    }
}

/**
 * @brief  系统主入口
 * @note   完成所有硬件外设初始化、加载 Flash 运行参数、
 *         创建 FreeRTOS 任务，最后启动调度器。
 *         调度器启动后，main() 不再返回。
 */
int main(void)
{
    BaseType_t ret;                      /* 任务创建返回值 */
    LogicThresholds_t factory_thresholds;/* 出厂默认阈值 */
    /* 出厂默认风扇转速表：0/25/50/75/100 对应 5 个档位的 PWM 百分比 */
    static const uint8_t factory_fan_levels[MOTOR_LEVEL_COUNT] = {0U, 25U, 50U, 75U, 100U};

    /* -------- 系统时钟与基础外设初始化 -------- */
    SystemInit();                        /* 配置系统时钟 168MHz */
    Delay_DWT_Init();                    /* 初始化 DWT 周期计数器（微秒级延时） */

    /* -------- 调试串口（最先初始化以便输出后续日志） -------- */
    Usart_Config();                      /* USART1: PA9(TX)/PA10(RX)，115200bps */
    printf("Boot: USART1 debug ready (PA9/PA10)\r\n");

    /* -------- 各硬件模块初始化 -------- */
    OLED_Init();                         /* OLED 显示屏初始化（I2C, PB8/PB9） */
    Modbus_Relay_Init();                 /* UART4 Modbus 主站 + 互斥信号量 */
    AllSensorData_Init();                /* 传感器数据管理模块初始化 */
    Logic_Init();                        /* 自动控制逻辑模块初始化 */
    Motor_Init();                        /* PWM 风扇初始化（TIM1_CH4, PE14） */

    /* -------- 加载运行参数 -------- */
    App_BuildProjectFactoryThresholds(&factory_thresholds);                   /* 默认阈值，可在上面修改 */
    RuntimeConfig_SetFactoryDefaults(&factory_thresholds, factory_fan_levels);/* 检验阈值和风扇值 */
    (void)RuntimeConfig_LoadAndApply();  /* 从 Flash 加载可能屏幕或者网页端修改过的值，若无效则回退上面函数赋的值 */

    /* -------- 串口屏初始化 -------- */
    // UART_Screen_Init();                  /* 仅初始化底层 UART5 硬件（测试模式） */
    Screen_Init();                       /* 初始化串口屏完整协议栈 */


    OLED_Clear();                        /* 清空 OLED 显存 */
    LED_GPIO_Init();                     /* PG14 心跳 LED 初始化 */

    /* ============ 创建 FreeRTOS 任务 ============ */

    /* OLED 显示轮播任务：栈 1024 字 = 4KB，优先级 2 */
    ret = xTaskCreate(DisplayCycleTask, "Display", 1024, NULL, 2, NULL);
    if (ret != pdPASS)
    {
        printf("ERR: create Display failed\r\n");
        while (1)
        {
        }
    }

    /* 主控制任务：栈 768 字 = 3KB，给控制链路 + Screen 记录调用留更充足余量 */
    ret = xTaskCreate(ControlTask, "Control", CONTROL_TASK_STACK_WORDS, NULL, 2, NULL);
    if (ret != pdPASS)
    {
        printf("ERR: create Control failed\r\n");
        while (1)
        {
        }
    }

    /* 心跳 LED 闪烁任务：栈 128 字 = 512B，优先级 1（最低） */
    ret = xTaskCreate(Task_PG14, "LED_PG14", 128, NULL, 1, NULL);
    if (ret != pdPASS)
    {
        printf("ERR: create LED task failed\r\n");
        while (1)
        {
        }
    }

#if UART4_TX_TEST_ENABLE
    /* 串口屏发送测试任务（仅调试用） */
    ret = xTaskCreate(UART4_TxTestTask, "U4_TX", 256, NULL, 1, NULL);
    if (ret != pdPASS)
    {
        printf("ERR: create U4_TX failed\r\n");
        while (1)
        {
        }
    }
    printf("OK: create U4_TX test task\r\n");
#else
    /* 串口屏数据刷新与交互任务：栈 640 字 = 2.5KB，覆盖较深 snprintf/发送调用链 */
    ret = xTaskCreate(Screen_Task, "Screen", SCREEN_TASK_STACK_WORDS, NULL, 1, NULL);
    if (ret != pdPASS)
    {
        printf("ERR: create Screen failed\r\n");
        while (1)
        {
        }
    }
    printf("OK: create Screen task\r\n");
#endif

    /* 启用 MQTT 远程通信任务 */
    if (ESP01S_TEST_ENABLE)
    {
        printf("INFO: free heap before MQTT tasks=%lu\r\n", (unsigned long)xPortGetFreeHeapSize());

        /* MQTT 任务：栈 1024 字 = 4KB（需要较大栈空间处理 JSON），优先级 2 */
        ret = xTaskCreate(MqttApp_Task, "MQTT", MQTT_TASK_STACK_WORDS, NULL, 2, NULL);
        if (ret != pdPASS)
        {
            printf("ERR: create MQTT failed\r\n");
            while (1)
            {
            }
        }

        printf("INFO: free heap after MQTT tasks=%lu\r\n", (unsigned long)xPortGetFreeHeapSize());
    }

    /* UART3 诊断输出任务（低优先级，仅打印）
       注意：诊断任务不是业务必需，若内存不足创建失败，不能卡死系统。 */
    if (xPortGetFreeHeapSize() > (size_t)2048U)
    {
        ret = xTaskCreate(Diag_Task, "DIAG", DIAG_TASK_STACK_WORDS, NULL, 1, NULL);
        if (ret != pdPASS)
        {
            printf("WARN: create DIAG failed (heap=%lu)\r\n",
                   (unsigned long)xPortGetFreeHeapSize());
        }
    }
    else
    {
        printf("WARN: skip DIAG (heap=%lu)\r\n",
               (unsigned long)xPortGetFreeHeapSize());
    }

    /* ============ 启动 FreeRTOS 调度器 ============ */
    vTaskStartScheduler();  /* 正常情况下不会返回 */

    printf("ERR: scheduler start failed\r\n");
    
    while (1)
    {
    }  /* 调度器启动失败，死循环 */
}

