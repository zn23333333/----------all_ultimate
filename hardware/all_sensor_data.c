/**
 * @file    all_sensor_data.c
 * @brief   传感器数据集中管理模块（实现）
 *
 * 将有线Modbus传感器（温湿度/PM2.5&PM10/燃气）和无线JSON传感器
 * （人体存在/光照/门磁）的数据统一存储在单一结构体中。
 * 使用FreeRTOS互斥锁保护共享数据，确保多任务并发访问安全。
 *
 * 功能概述：
 *   1. Modbus刷新：读取“温湿度/PM/燃气”并更新内部数据
 *   2. 无线JSON解析：从 WiFi转发的JSON负载中提取人体/光照/门磁数据
 *   3. 连接状态监测：基于最后成功读取时间判断传感器是否在线
 *   4. 线程安全快照：提供完整数据拷贝供其他任务使用
 */
#include "all_sensor_data.h"

#include "Modbus_Relay.h"
#include "json_parse.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

/* ======================== 静态变量 ======================== */
static AllSensorData_t s_data;                    /**< 全局传感器数据聚合结构体 */
static SemaphoreHandle_t s_data_mutex = NULL;      /**< 数据保护互斥锁 */
static uint8_t s_temp_humi_seen = 0U;              /**< 温湿度传感器是否曾经成功读取 */
static uint8_t s_pm_seen = 0U;                     /**< PM传感器是否曾经成功读取 */
static uint8_t s_gas_seen = 0U;                    /**< 燃气传感器是否曾经成功读取 */
static uint8_t s_human_seen = 0U;                  /**< 人体传感器是否曾经收到数据 */
static uint8_t s_light_seen = 0U;                  /**< 光照传感器是否曾经收到数据 */
static uint8_t s_door_seen = 0U;                   /**< 门磁传感器是否曾经收到数据 */
static TickType_t s_temp_humi_tick = 0U;            /**< 温湿度最后成功读取时的打拍计数值 */
static TickType_t s_pm_tick = 0U;                   /**< PM最后成功读取时的打拍计数值 */
static TickType_t s_gas_tick = 0U;                  /**< 燃气最后成功读取时的打拍计数值 */
static TickType_t s_human_tick = 0U;                /**< 人体最后收到数据时的打拍计数值 */
static TickType_t s_light_tick = 0U;                /**< 光照最后收到数据时的打拍计数值 */
static TickType_t s_door_tick = 0U;                 /**< 门磁最后收到数据时的打拍计数值 */

/** 传感器连接超时时间（毫秒），超过该时间未读取则判定为离线 */
#define ALL_SENSOR_CONN_TIMEOUT_MS   25000U

/* ======================== 内部工具函数 ======================== */

/**
 * @brief   将超时时间转换为Tick数，至少为1
 * @return  超时对应的Tick数
 */
static TickType_t ConnTimeoutTicks(void)
{
    TickType_t ticks = pdMS_TO_TICKS(ALL_SENSOR_CONN_TIMEOUT_MS);
    if (ticks == 0U)
    {
        ticks = 1U;
    }
    return ticks;
}

/**
 * @brief   获取当前系统Tick计数值
 * @return  当前FreeRTOS Tick值
 */
static TickType_t NowTick(void)
{
    return xTaskGetTickCount();
}

/**
 * @brief   基于Tick差值判断传感器是否在线
 * @param   seen       是否曾经成功读取过
 * @param   last_tick  最后一次成功读取的Tick值
 * @param   now        当前Tick值
 * @param   timeout    超时阈值（Tick数）
 * @return  1=在线, 0=离线
 */
static uint8_t IsConnectedByTick(uint8_t seen, TickType_t last_tick, TickType_t now, TickType_t timeout)
{
    if (seen == 0U) return 0U;
    return ((now - last_tick) <= timeout) ? 1U : 0U;
}

/**
 * @brief   获取互斥锁等待时间（50ms）
 * @return  等待对应的Tick数
 */
static TickType_t LockWaitTicks(void)
{
    return pdMS_TO_TICKS(50);
}

/**
 * @brief   获取数据互斥锁
 * @return  1=成功获取, 0=失败（互斥锁未创建或超时）
 */
static uint8_t LockData(void)
{
    if (s_data_mutex == NULL)
    {
        return 0U;
    }
    return (xSemaphoreTake(s_data_mutex, LockWaitTicks()) == pdTRUE) ? 1U : 0U;
}

/**
 * @brief   释放数据互斥锁
 */
static void UnlockData(void)
{
    if (s_data_mutex != NULL)
    {
        xSemaphoreGive(s_data_mutex);
    }
}

/* ======================== 公开接口实现 ======================== */

/**
 * @brief   初始化传感器数据模块
 *
 * 清零所有传感器数据和状态标志，创建FreeRTOS互斥锁。
 * 应在系统启动时调用一次。
 */
void AllSensorData_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    s_temp_humi_seen = 0U;
    s_pm_seen = 0U;
    s_gas_seen = 0U;
    s_human_seen = 0U;
    s_light_seen = 0U;
    s_door_seen = 0U;
    s_temp_humi_tick = 0U;
    s_pm_tick = 0U;
    s_gas_tick = 0U;
    s_human_tick = 0U;
    s_light_tick = 0U;
    s_door_tick = 0U;

    if (s_data_mutex == NULL)
    {
        s_data_mutex = xSemaphoreCreateMutex();
    }
}

/**
 * @brief   通过Modbus总线刷新有线传感器数据
 *
 * 依次读取温湿度、PM2.5/PM10、燃气传感器，
 * 将成功读取的数据更新到内部结构体并记录时间戳。
 *
 * @return  成功读取的传感器有效位掩码，0表示全部读取失败
 */
uint32_t AllSensorData_RefreshModbus(void)
{
    float temp;
    float humi;
    uint16_t pm25;
    uint16_t pm10;
    uint16_t gas;
    uint32_t mask = 0U;
    TickType_t now = NowTick();

    if (ReadTemperatureHumidity(&temp, &humi))
    {
        mask |= ALL_SENSOR_VALID_TEMP_HUMI;
    }

    if (ReadPM25PM10(&pm25, &pm10))
    {
        mask |= ALL_SENSOR_VALID_PM;
    }

    if (ReadGas(&gas))
    {
        mask |= ALL_SENSOR_VALID_GAS;
    }

    if ((mask == 0U) || !LockData())
    {
        return 0U;
    }

    if ((mask & ALL_SENSOR_VALID_TEMP_HUMI) != 0U)
    {
        s_data.temperature = temp;
        s_data.humidity = humi;
        s_temp_humi_seen = 1U;
        s_temp_humi_tick = now;
    }
    if ((mask & ALL_SENSOR_VALID_PM) != 0U)
    {
        s_data.pm25 = pm25;
        s_data.pm10 = pm10;
        s_pm_seen = 1U;
        s_pm_tick = now;
    }
    if ((mask & ALL_SENSOR_VALID_GAS) != 0U)
    {
        s_data.gas = gas;
        s_gas_seen = 1U;
        s_gas_tick = now;
    }
    s_data.valid_mask |= mask;
    UnlockData();
    return mask;
}

/**
 * @brief   从无线传感器的JSON负载中解析并更新数据
 *
 * 支持三种类型JSON:
 *   - {"type":"human", "presence":true, "distance":150}
 *   - {"type":"light", "lux":500}
 *   - {"type":"door",  "closed":true, "level":200}
 *
 * @param   payload  JSON字符串
 * @return  1=解析成功并更新, 0=失败
 */
uint8_t AllSensorData_UpdateFromWirelessJson(const char *payload)
{
    char type[16];
    int int_value;
    uint16_t value;
    uint8_t bit_value;

    if (payload == NULL)
    {
        return 0U;
    }

    if (!JsonParse_GetString(payload, "type", type, sizeof(type)))
    {
        return 0U;
    }

    if (strcmp(type, "human") == 0)
    {
        if (!JsonParse_GetBool(payload, "presence", &bit_value))
        {
            return 0U;
        }

        if (!JsonParse_GetInt(payload, "distance", &int_value) || !LockData())
        {
            return 0U;
        }

        if (int_value < 0)
        {
            int_value = 0;
        }
        if (int_value > 65535)
        {
            int_value = 65535;
        }
        value = (uint16_t)int_value;

        s_data.human_presence = bit_value;
        s_data.human_distance = value;
        s_data.valid_mask |= ALL_SENSOR_VALID_HUMAN;
        s_human_seen = 1U;
        s_human_tick = NowTick();
        UnlockData();
        return 1U;
    }

    if (strcmp(type, "light") == 0)
    {
        if (!JsonParse_GetInt(payload, "lux", &int_value) || !LockData())
        {
            return 0U;
        }

        if (int_value < 0)
        {
            int_value = 0;
        }
        if (int_value > 65535)
        {
            int_value = 65535;
        }
        value = (uint16_t)int_value;

        s_data.light_lux = value;
        s_data.valid_mask |= ALL_SENSOR_VALID_LIGHT;
        s_light_seen = 1U;
        s_light_tick = NowTick();
        UnlockData();
        return 1U;
    }

    if (strcmp(type, "door") == 0)
    {
        if (!JsonParse_GetBool(payload, "closed", &bit_value))
        {
            return 0U;
        }

        if (!JsonParse_GetInt(payload, "level", &int_value) || !LockData())
        {
            return 0U;
        }

        if (int_value < 0)
        {
            int_value = 0;
        }
        if (int_value > 255)
        {
            int_value = 255;
        }

        s_data.door_closed = bit_value;
        s_data.door_level = (uint8_t)int_value;
        s_data.valid_mask |= ALL_SENSOR_VALID_DOOR;
        s_door_seen = 1U;
        s_door_tick = NowTick();
        UnlockData();
        return 1U;
    }

    return 0U;
}

/**
 * @brief   标记所有无线传感器为活跃状态
 *
 * 当无线传感器节点整体在线时调用，刷新全部无线传感器的
 * 时间戳，防止因单个传感器未报告而被判定为离线。
 */
void AllSensorData_MarkWirelessAlive(void)
{
    TickType_t now;

    if (!LockData())
    {
        return;
    }

    now = NowTick();
    s_human_seen = 1U;
    s_light_seen = 1U;
    s_door_seen = 1U;
    s_human_tick = now;
    s_light_tick = now;
    s_door_tick = now;
    UnlockData();
}

/**
 * @brief   获取所有传感器数据的完整快照（线程安全）
 * @param   out_data  输出缓冲区，接收当前数据副本
 */
void AllSensorData_GetSnapshot(AllSensorData_t *out_data)
{
    if (out_data == NULL)
    {
        return;
    }

    if (!LockData())
    {
        memset(out_data, 0, sizeof(*out_data));
        return;
    }

    *out_data = s_data;
    UnlockData();
}

/**
 * @brief   获取当前数据有效位掩码（线程安全）
 * @return  valid_mask 位掩码
 */
uint32_t AllSensorData_GetValidMask(void)
{
    uint32_t mask;

    if (!LockData())
    {
        return 0U;
    }

    mask = s_data.valid_mask;
    UnlockData();
    return mask;
}

/**
 * @brief   获取各传感器连接状态（基于超时检测）
 *
 * 判断标准：当前时间 - 最后读取时间 > 25秒 则判定为离线。
 * 如果传感器从未成功读取过，也判定为离线。
 *
 * @param   out_status  输出状态缓冲区
 */
void AllSensorData_GetConnectionStatus(AllSensorConnectionStatus_t *out_status)
{
    TickType_t now;
    TickType_t timeout;

    if (out_status == NULL)
    {
        return;
    }

    memset(out_status, 0, sizeof(*out_status));

    if (!LockData())
    {
        return;
    }

    now = NowTick();
    timeout = ConnTimeoutTicks();

    out_status->temp_humi_connected = IsConnectedByTick(s_temp_humi_seen, s_temp_humi_tick, now, timeout);
    out_status->pm_connected = IsConnectedByTick(s_pm_seen, s_pm_tick, now, timeout);
    out_status->gas_connected = IsConnectedByTick(s_gas_seen, s_gas_tick, now, timeout);
    out_status->human_connected = IsConnectedByTick(s_human_seen, s_human_tick, now, timeout);
    out_status->light_connected = IsConnectedByTick(s_light_seen, s_light_tick, now, timeout);
    out_status->door_connected = IsConnectedByTick(s_door_seen, s_door_tick, now, timeout);

    UnlockData();
}
