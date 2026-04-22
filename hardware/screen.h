/**
 * @file    screen.h
 * @brief   串口屏控制模块 —— 对外接口头文件
 *
 * 本模块通过 UART5（512000bps, PC12/PD2）与带触摸功能的串口屏通信，
 * 使用自定义二进制协议（0x55 头 / 0xAA 尾帧格式）和 Nextion 文本指令。
 *
 * 主要功能：
 *   - 多页面管理（主页/传感器页/阈值设置页/手动控制页/继电器状态页/系统设置页）
 *   - 传感器数据实时显示
 *   - 阈值参数在线编辑
 *   - 设备手动控制（继电器开关、风扇档位调节）
 *   - 设备操作记录（动作队列异步上报）
 *   - NTP 时间同步与本地时钟
 *
 * 在 FreeRTOS 环境下运行，Screen_Task 作为独立任务周期调度。
 */

#ifndef SCREEN_H
#define SCREEN_H

#include <stdint.h>

/**
 * @brief 设备动作类型枚举 —— 标识被操作的设备
 *
 * 用于记录设备操作事件时，标明是对哪个执行器的操作。
 */
typedef enum
{
    SCREEN_ACTION_DEV_HEATER = 0U,      /**< 制热器（加热器） */
    SCREEN_ACTION_DEV_COOLER,           /**< 制冷器 */
    SCREEN_ACTION_DEV_HUMIDIFIER,       /**< 加湿器 */
    SCREEN_ACTION_DEV_ALARM,            /**< 声光报警器 */
    SCREEN_ACTION_DEV_LIGHT,            /**< LED灯光 */
    SCREEN_ACTION_DEV_FAN               /**< 风扇 */
} ScreenActionDevice_t;

/**
 * @brief 设备操作事件结构体
 *
 * 由 Screen_RecordDeviceAction() 写入动作队列，
 * 由 Screen_PopActionEvent() 弹出供 MQTT 上报使用。
 */
typedef struct
{
    ScreenActionDevice_t device;        /**< 被操作的设备类型 */
    uint8_t is_manual;                  /**< 操作模式：1=手动, 0=自动 */
    uint8_t action_value;               /**< 操作值：开关类设备为0/1, 风扇为档位1~5 */
    char date_text[24];                 /**< 操作日期字符串, 如 "2026/3/2" */
    char time_text[8];                  /**< 操作时间字符串, 如 "14:30" */
} ScreenActionEvent_t;

/**
 * @brief   初始化串口屏模块
 *
 * 初始化 UART5 硬件、发送互斥锁、帧解析窗口、文本缓存、
 * 本地时钟等所有内部状态，并关闭屏幕端返回码噪声。
 */
void Screen_Init(void);

/**
 * @brief   串口屏主任务函数（FreeRTOS 任务入口）
 * @param   arg  任务参数（未使用）
 *
 * 在无限循环中：
 *   1. 从 UART5 接收缓冲区逐字节读取并解析触摸/配置帧
 *   2. 根据当前活动页面周期刷新屏幕显示内容
 *   3. 处理即时刷新请求（页面切换、时间同步等触发）
 */
void Screen_Task(void *arg);

/**
 * @brief   从 MQTT 时间消息同步本地时钟
 * @param   ts         UNIX 时间戳
 * @param   has_ts     时间戳是否有效: 1=有效, 0=无效
 * @param   local_str  本地时间字符串, 格式 "YYYY-MM-DD HH:MM:SS"
 * @param   tz_str     时区字符串, 如 "+08:00"
 *
 * 解析 local_str 并校准本地时钟；包含接收延迟补偿和秒级抖动过滤。
 */
void Screen_SetTimeFromMqtt(uint32_t ts,
                            uint8_t has_ts,
                            const char *local_str,
                            const char *tz_str);

/**
 * @brief   记录一次设备操作事件
 * @param   device        被操作的设备
 * @param   is_manual     操作模式: 1=手动, 0=自动
 * @param   action_value  操作值: 开关类为0/1, 风扇为档位1~5
 *
 * 同时向串口屏 recorder 控件写入记录，并推入动作队列供外部弹出。
 */
void Screen_RecordDeviceAction(ScreenActionDevice_t device,
                               uint8_t is_manual,
                               uint8_t action_value);

/**
 * @brief   从动作队列弹出一条设备操作事件
 * @param   out_event  输出参数，弹出的事件数据
 * @return  1=成功弹出, 0=队列为空
 *
 * 供 MQTT 上报模块调用，按 FIFO 顺序取出事件。
 */
uint8_t Screen_PopActionEvent(ScreenActionEvent_t *out_event);

#endif /* SCREEN_H */
