/**
 * @file    all_sensor_data.h
 * @brief   传感器数据集中管理模块（头文件）
 *
 * 本模块将所有传感器数据（有线Modbus + 无线JSON）统一管理，
 * 提供线程安全的数据读写接口（FreeRTOS互斥锁保护）。
 *
 * 数据来源：
 *   - 有线Modbus：温湿度、PM2.5/PM10、燃气（通过 Modbus_Relay 模块读取）
 *   - 无线JSON：  人体存在、光照强度、门磁状态（通过 ESP01S/MQTT 接收）
 *
 * 每种传感器有独立的有效位掩码(valid_mask)和连接超时检测机制。
 */
#ifndef ALL_SENSOR_DATA_H
#define ALL_SENSOR_DATA_H

#include <stdint.h>

/** @name 传感器数据有效位掩码（用于 valid_mask 字段） */
/** @{ */
#define ALL_SENSOR_VALID_TEMP_HUMI  (1UL << 0)  /**< 温湿度数据有效 */
#define ALL_SENSOR_VALID_PM         (1UL << 1)  /**< PM2.5/PM10数据有效 */
#define ALL_SENSOR_VALID_GAS        (1UL << 2)  /**< 燃气浓度数据有效 */
#define ALL_SENSOR_VALID_HUMAN      (1UL << 3)  /**< 人体存在检测数据有效 */
#define ALL_SENSOR_VALID_LIGHT      (1UL << 4)  /**< 光照强度数据有效 */
#define ALL_SENSOR_VALID_DOOR       (1UL << 5)  /**< 门磁状态数据有效 */
/** @} */

/**
 * @brief   全部传感器数据聚合结构体
 *
 * 集中存储所有传感器的最新读数，通过 valid_mask 标识哪些数据有效。
 */
typedef struct
{
    float temperature;       /**< 温度值（℃），有线Modbus读取 */
    float humidity;          /**< 湿度值（%RH），有线Modbus读取 */
    uint16_t pm25;           /**< PM2.5浓度（μg/m³），有线Modbus读取 */
    uint16_t pm10;           /**< PM10浓度（μg/m³），有线Modbus读取 */
    uint16_t gas;            /**< 燃气浓度（ADC原始值），有线Modbus读取 */

    uint8_t human_presence;  /**< 人体存在标志（1=有人, 0=无人），无线JSON */
    uint16_t human_distance; /**< 人体距离（mm），无线JSON */

    uint16_t light_lux;      /**< 光照强度（lux），无线JSON */

    uint8_t door_closed;     /**< 门磁状态（1=关闭, 0=打开），无线JSON */
    uint8_t door_level;      /**< 门磁传感器 GPIO 的原始电平值（0 或 1）（没用） */

    uint32_t valid_mask;     /**< 数据有效位掩码，由 ALL_SENSOR_VALID_xxx 位组合 */
} AllSensorData_t;

/**
 * @brief   各传感器连接状态结构体
 *
 * 基于最后一次成功读取时间与超时阈值判断传感器是否在线。
 */
typedef struct
{
    uint8_t temp_humi_connected;  /**< 温湿度传感器在线（1=在线, 0=离线） */
    uint8_t pm_connected;         /**< PM颗粒物传感器在线 */
    uint8_t gas_connected;        /**< 燃气传感器在线 */
    uint8_t human_connected;      /**< 人体存在传感器在线 */
    uint8_t light_connected;      /**< 光照传感器在线 */
    uint8_t door_connected;       /**< 门磁传感器在线 */
} AllSensorConnectionStatus_t;

/** @brief 初始化传感器数据模块，清零数据并创建互斥锁 */
void AllSensorData_Init(void);

/** @brief 通过Modbus读取有线传感器（温湿度/PM/燃气），更新内部数据
 *  @return 成功读取的传感器有效位掩码，0表示全部失败 */
uint32_t AllSensorData_RefreshModbus(void);

/** @brief 从无线JSON载荷中解析并更新传感器数据（人体/光照/门磁）
 *  @param  payload  JSON字符串（如 {"type":"human","presence":true,"distance":150}）
 *  @return 1=解析成功并更新, 0=失败 */
uint8_t AllSensorData_UpdateFromWirelessJson(const char *payload);

/** @brief 标记所有无线传感器为活跃状态（刷新超时计时器） */
void AllSensorData_MarkWirelessAlive(void);

/** @brief 获取当前所有传感器数据的快照（线程安全拷贝）
 *  @param  out_data  输出数据缓冲区 */
void AllSensorData_GetSnapshot(AllSensorData_t *out_data);

/** @brief 获取当前数据有效位掩码
 *  @return valid_mask 位掩码 */
uint32_t AllSensorData_GetValidMask(void);

/** @brief 获取各传感器连接状态（基于超时检测）
 *  @param  out_status  输出状态缓冲区 */
void AllSensorData_GetConnectionStatus(AllSensorConnectionStatus_t *out_status);

#endif /* ALL_SENSOR_DATA_H */
