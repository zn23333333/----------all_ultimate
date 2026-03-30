#ifndef DIAG_TASK_H
#define DIAG_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 低频 UART3 诊断输出任务。
 * - 周期输出：heap、最小heap、WiFi/MQTT/Web 状态、ESP01S 错误码。
 * - 低频输出各任务剩余栈（HighWaterMark）。
 */
void Diag_Task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_TASK_H */
