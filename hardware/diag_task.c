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
#include "Modbus_Relay.h"
#include "mqtt_app.h"
#include "uart1_modbus.h"

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

    task = xTaskGetHandle("LED_PG14");
    if (task != NULL)
    {
        printf("[DIAG][STACK] LED_PG14=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    task = xTaskGetIdleTaskHandle();
    if (task != NULL)
    {
        printf("[DIAG][STACK] IDLE=%lu\r\n", (unsigned long)uxTaskGetStackHighWaterMark(task));
    }

    printf("[DIAG][STACK] ---------------------------\r\n");
}

static void Diag_PrintModbus(void)
{
    ModbusDiagSnapshot_t diag;
    SerialDiagSnapshot_t serial_diag;
    uint8_t i;

    Modbus_GetLastDiag(&diag);
    Serial_GetDiagSnapshot(&serial_diag);
    printf("[DIAG][MODBUS] cmd=%s err=%s retry=%u req=%02X/%02X resp=%02X/%02X len=%u bytes=%u\r\n",
           Modbus_CommandName(diag.command),
           Modbus_ErrorText(diag.error),
           (unsigned int)diag.retry_index,
           (unsigned int)diag.request_addr,
           (unsigned int)diag.request_func,
           (unsigned int)diag.response_addr,
           (unsigned int)diag.response_func,
           (unsigned int)diag.response_len,
           (unsigned int)diag.response_byte_count);
    if (diag.response_preview_len > 0U)
    {
        printf("[DIAG][MODBUS] raw=");
        for (i = 0U; i < diag.response_preview_len; i++)
        {
            printf("%02X", (unsigned int)diag.response_preview[i]);
            if ((i + 1U) < diag.response_preview_len)
            {
                printf(" ");
            }
        }
        printf("\r\n");
    }
    printf("[DIAG][UART4] frames=%lu ore=%lu fe=%lu ne=%lu\r\n",
           (unsigned long)serial_diag.rx_frame_count,
           (unsigned long)serial_diag.rx_ore_count,
           (unsigned long)serial_diag.rx_fe_count,
           (unsigned long)serial_diag.rx_ne_count);
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
        Diag_PrintModbus();

        now = xTaskGetTickCount();
        if ((now - last_stack_tick) >= pdMS_TO_TICKS(DIAG_STACK_INTERVAL_MS))
        {
            Diag_PrintStacks();
            last_stack_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(DIAG_STATUS_INTERVAL_MS));
    }
}
