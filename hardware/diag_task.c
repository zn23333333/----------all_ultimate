/*
 * @file  diag_task.c
 * @brief UART3 诊断输出任务（FreeRTOS）
 *
 * 用于定位“运行一段时间后 Web 端失联”的现场问题：
 * - 周期打印 heap / 最小 heap
 * - 周期打印 WiFi/MQTT/Web 在线状态和 ESP01S 错误
 * - 低频打印各任务剩余栈（HighWaterMark）
 *
 * 不改变任何业务逻辑，仅增加串口诊断输出。
 */

#include "diag_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "esp01s.h"
#include "mqtt_app.h"

/* 输出周期：适当克制，避免 printf 影响系统实时性 */
#define DIAG_STATUS_INTERVAL_MS   5000U
#define DIAG_STACK_INTERVAL_MS    30000U

static void Diag_PrintStacks(void)
{
    TaskHandle_t task;

    printf("[DIAG][STACK] ---- high-water (words) ----\r\n");

    task = xTaskGetHandle("MQTT");
    if (task != NULL)
    {
        printf("[DIAG][STACK] MQTT=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    task = xTaskGetHandle("Control");
    if (task != NULL)
    {
        printf("[DIAG][STACK] Control=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    task = xTaskGetHandle("Screen");
    if (task != NULL)
    {
        printf("[DIAG][STACK] Screen=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    task = xTaskGetHandle("Display");
    if (task != NULL)
    {
        printf("[DIAG][STACK] Display=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    task = xTaskGetHandle("LED_PC13");
    if (task != NULL)
    {
        printf("[DIAG][STACK] LED_PC13=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    task = xTaskGetIdleTaskHandle();
    if (task != NULL)
    {
        printf("[DIAG][STACK] IDLE=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    printf("[DIAG][STACK] ---------------------------\r\n");
}

void Diag_Task(void *arg)
{
    TickType_t last_stack_tick;
    TickType_t now;
    (void)arg;

    /* 等系统稳定（避免与 ESP 初始化打印/命令冲突） */
    vTaskDelay(pdMS_TO_TICKS(3000U));
    last_stack_tick = xTaskGetTickCount();

    for (;;)
    {
        size_t free_heap = xPortGetFreeHeapSize();
        size_t min_heap = xPortGetMinimumEverFreeHeapSize();
        uint8_t wifi = ESP01S_IsConnected();
        uint8_t mqtt = MqttApp_IsBrokerConnected();
        uint8_t web = ESP01S_IsWebConnected();
        uint16_t err = ESP01S_GetLastErrorCode();
        const char *err_text = ESP01S_GetLastErrorText();

        printf("[DIAG] heap=%lu min=%lu wifi=%u mqtt=%u web=%u err=%u(%s)\r\n",
               (unsigned long)free_heap,
               (unsigned long)min_heap,
               (unsigned int)wifi,
               (unsigned int)mqtt,
               (unsigned int)web,
               (unsigned int)err,
               (err_text != NULL) ? err_text : "-");

        now = xTaskGetTickCount();
        if ((now - last_stack_tick) >= pdMS_TO_TICKS(DIAG_STACK_INTERVAL_MS))
        {
            Diag_PrintStacks();
            last_stack_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(DIAG_STATUS_INTERVAL_MS));
    }
}

