/**
 * @file    logic.h
 * @brief   环境保障系统 —— 自动控制逻辑模块（头文件）
 *
 * 本模块是系统的自动控制核心，负责：
 *   1. 根据温度传感器数据自动控制加热器/制冷器（带迟滞）
 *   2. 根据湿度传感器数据自动控制加湿器/除湿风扇（带迟滞）
 *   3. 根据人体感应+光照数据自动控制灯光
 *   4. 根据门磁/气体/PM2.5数据触发报警（带延时）
 *   5. 多源风扇档位控制（PM2.5/气体/除湿分段取最高档）
 *   6. 手动覆盖机制（定时手动控制到期自动恢复）
 *   7. 最小切换间隔保护（防止继电器频繁抖动）
 *   8. 7组阈值修改接口，供串口屏/MQTT远程调参
 *
 * 硬件平台: STM32F407ZGT6 + FreeRTOS
 */
#ifndef LOGIC_H
#define LOGIC_H

#include <stdint.h>

#include "all_sensor_data.h"

/* ========== 继电器通道编号（对应 Modbus 继电器硬件地址） ========== */
#define LOGIC_RELAY_HEATER      0U  /**< 加热器继电器通道 */
#define LOGIC_RELAY_COOLER      1U  /**< 制冷器继电器通道 */
#define LOGIC_RELAY_HUMIDIFIER  2U  /**< 加湿器继电器通道 */
#define LOGIC_RELAY_LIGHT       3U  /**< 灯光继电器通道 */
#define LOGIC_RELAY_ALARM       4U  /**< 报警器继电器通道 */
// 风扇的继电器通道由风扇控制模块直接控制，因此不在此定义

/**
 * @defgroup LOGIC_DEV 设备索引定义
 * @brief    用于手动覆盖 API 的设备编号（包含继电器设备 + 风扇）
 * @note     LOGIC_DEV_DEHUMIDIFIER 无独立继电器，通过风扇档位实现除湿
 * @{
 */
#define LOGIC_DEV_HEATER        0U  /**< 加热器设备索引 */
#define LOGIC_DEV_COOLER        1U  /**< 制冷器设备索引 */
#define LOGIC_DEV_HUMIDIFIER    2U  /**< 加湿器设备索引 */
#define LOGIC_DEV_DEHUMIDIFIER  3U  /**< 除湿器设备索引（映射到风扇） */
#define LOGIC_DEV_ALARM         4U  /**< 报警器设备索引 */
#define LOGIC_DEV_LIGHT         5U  /**< 灯光设备索引 */
#define LOGIC_DEV_RELAY_COUNT   6U  /**< 继电器类设备总数 */
#define LOGIC_DEV_FAN           6U  /**< 风扇设备索引（PWM电机，非继电器） */
#define LOGIC_DEV_COUNT         7U  /**< 所有可控设备总数 */
/** @} */

/**
 * @brief 自动控制阈值参数结构体
 *
 * 包含所有传感器判断阈值、迟滞量、分段风扇映射、
 * 报警延时及最小继电器切换间隔等完整参数集。
 * 可通过 Change1~Change7 接口在线修改并持久化保存。
 */
typedef struct
{
    /* ---- 温度控制阈值 ---- */
    float temp_low;                     /**< 温度下限(°C)，低于此值开启加热器 */
    float temp_high;                    /**< 温度上限(°C)，高于此值开启制冷器 */

    /* ---- 湿度控制阈值 ---- */
    float humidity_low;                 /**< 湿度下限(%RH)，低于此值开启加湿器 */
    float humidity_fan_high;            /**< 湿度上限(%RH)，高于此值开启除湿（风扇） */
    uint8_t humidity_fan_level;         /**< 除湿时风扇档位（MOTOR_LEVEL 索引） */

    /* ---- 灯光控制阈值 ---- */
    uint16_t light_low;                 /**< 光照下限(lux)，低于此值且有人时开灯 */
    uint16_t human_light_distance_max;  /**< 人体感应最大有效距离(cm)，超过则忽略 */

    /* ---- 报警阈值 ---- */
    uint16_t gas_high;                  /**< 气体浓度报警上限(ppm) */
    uint16_t pm25_high;                 /**< PM2.5 报警上限(ug/m3) */

    /* ---- PM2.5 分段风扇映射（5段） ---- */
    uint16_t pm25_fan_lower[5];         /**< PM2.5 各段下界 */
    uint16_t pm25_fan_upper[5];         /**< PM2.5 各段上界 */
    uint8_t pm25_fan_level[5];          /**< PM2.5 各段对应风扇档位 */

    /* ---- 气体分段风扇映射（5段） ---- */
    uint16_t gas_fan_lower[5];          /**< 气体浓度各段下界 */
    uint16_t gas_fan_upper[5];          /**< 气体浓度各段上界 */
    uint8_t gas_fan_level[5];           /**< 气体浓度各段对应风扇档位 */

    /* ---- 报警延时 ---- */
    uint32_t door_open_alarm_ms;        /**< 门持续打开报警延时(ms) */
    uint32_t gas_high_alarm_ms;         /**< 气体超标持续报警延时(ms) */
    uint32_t pm25_high_alarm_ms;        /**< PM2.5 超标持续报警延时(ms) */

    /* ---- 迟滞量（防止临界值附近频繁切换） ---- */
    float hyst_temp;                    /**< 温度迟滞量(°C) */
    float hyst_humi;                    /**< 湿度迟滞量(%RH) */
    uint16_t hyst_light;                /**< 光照迟滞量(lux) */
    uint16_t hyst_gas;                  /**< 气体浓度迟滞量(ppm) */
    uint16_t hyst_pm25;                 /**< PM2.5 迟滞量(ug/m3) */

    /* ---- 继电器保护 ---- */
    uint32_t min_switch_ms;             /**< 继电器最小切换间隔(ms)，防抖动 */
} LogicThresholds_t;

/**
 * @brief 设备手动/自动状态枚举
 *
 * 用于串口屏或MQTT查询某设备当前处于手动还是自动模式，
 * 以及该设备的开/关状态（含"待执行"状态，表示受最小切换间隔保护尚未生效）。
 */
typedef enum
{
    LOGIC_MANUAL_STATUS_MANUAL_OFF = 0U,         /**< 手动模式 - 已关闭 */
    LOGIC_MANUAL_STATUS_MANUAL_ON  = 1U,         /**< 手动模式 - 已开启 */
    LOGIC_MANUAL_STATUS_AUTO_OFF   = 2U,         /**< 自动模式 - 已关闭 */
    LOGIC_MANUAL_STATUS_AUTO_ON    = 3U,         /**< 自动模式 - 已开启 */
    LOGIC_MANUAL_STATUS_MANUAL_OFF_PENDING = 4U, /**< 手动关闭 - 待执行（受切换间隔保护） */
    LOGIC_MANUAL_STATUS_MANUAL_ON_PENDING  = 5U  /**< 手动开启 - 待执行（受切换间隔保护） */
} LogicManualStatus_t;

/**
 * @brief  初始化控制逻辑模块
 * @note   复位所有继电器状态、报警跟踪器、手动覆盖结构，加载默认阈值
 */
void Logic_Init(void);

/**
 * @brief  将阈值结构体填充为出厂默认值
 * @param  out_thresholds 输出指针，不可为 NULL
 */
void Logic_SetDefaultThresholds(LogicThresholds_t *out_thresholds);

/**
 * @brief  设置运行时阈值（整体覆盖）
 * @param  thresholds 要写入的阈值结构体指针
 */
void Logic_SetRuntimeThresholds(const LogicThresholds_t *thresholds);

/**
 * @brief  获取当前运行时阈值的副本
 * @param  out_thresholds 输出指针
 */
void Logic_GetRuntimeThresholds(LogicThresholds_t *out_thresholds);
/**
 * @brief  第1组阈值修改：温湿度控制阈值（加热/制冷/加湿/除湿）
 * @param  threshold_id 参数ID (0=加热温度, 1=制冷温度, 2=加湿湿度, 3=除湿湿度)
 * @param  value        新阈值
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange1ThresholdById(uint8_t threshold_id, float value);

/**
 * @brief  获取第1组阈值当前值
 */
void Logic_GetChange1Thresholds(float *heater_on_threshold,
                                float *cooler_on_threshold,
                                float *humidifier_on_threshold,
                                float *dehumidifier_on_threshold);

/**
 * @brief  第2组阈值修改：PM2.5/气体报警延时及报警阈值
 * @param  threshold_id 参数ID (4=PM2.5延时, 5=气体延时, 6=PM2.5阈值, 7=气体阈值)
 * @param  value        新阈值（延时单位为秒）
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange2ThresholdById(uint8_t threshold_id, float value);

/** @brief  获取第2组阈值当前值 */
void Logic_GetChange2Thresholds(float *pm25_alarm_delay_seconds,
                                float *gas_alarm_delay_seconds,
                                float *pm25_alarm_threshold,
                                float *gas_alarm_threshold);

/**
 * @brief  第3组阈值修改：门报警延时、光照阈值、人体距离、风扇档1速度
 * @param  threshold_id 参数ID (8=门报警, 9=光照, 10=人体距离, 11=风扇L1)
 * @param  value        新阈值
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange3ThresholdById(uint8_t threshold_id, float value);

/** @brief  获取第3组阈值当前值 */
void Logic_GetChange3Thresholds(float *door_open_alarm_seconds,
                                float *light_on_threshold,
                                float *human_light_distance_threshold,
                                float *fan_level1_speed);

/**
 * @brief  第4组阈值修改：风扇档2~5的PWM速度百分比
 * @param  threshold_id 参数ID (12=L2, 13=L3, 14=L4, 15=L5)
 * @param  value        速度百分比 (0~100)
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange4ThresholdById(uint8_t threshold_id, float value);

/** @brief  获取第4组阈值当前值 */
void Logic_GetChange4Thresholds(float *fan_level2_speed,
                                float *fan_level3_speed,
                                float *fan_level4_speed,
                                float *fan_level5_speed);

/**
 * @brief  第5组阈值修改：PM2.5分5段风扇范围上下界
 * @param  threshold_id 参数ID (17~26，偶数=下界，奇数=上界)
 * @param  value        范围值 (0~9999)
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange5ThresholdById(uint8_t threshold_id, uint32_t value);

/** @brief  获取第5组阈值：PM2.5分5段上下界 */
void Logic_GetChange5Thresholds(uint16_t lower_bounds[5], uint16_t upper_bounds[5]);

/**
 * @brief  第6组阈值修改：气体分5段风扇范围上下界
 * @param  threshold_id 参数ID (33~42，偶数=下界，奇数=上界)
 * @param  value        范围值 (0~9999)
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange6ThresholdById(uint8_t threshold_id, uint32_t value);

/** @brief  获取第6组阈值：气体分5段上下界 */
void Logic_GetChange6Thresholds(uint16_t lower_bounds[5], uint16_t upper_bounds[5]);
/**
 * @brief  将所有阈值、风扇档位表、手动覆盖状态恢复为出厂默认值
 */
void Logic_ResetAllToDefaultsManual(void);

/**
 * @brief  设置指定设备进入手动模式
 * @param  dev              设备索引 (LOGIC_DEV_xxx)
 * @param  duration_seconds 手动持续时长(秒)，到期自动恢复
 * @param  value            继电器: 0=关/1=开；风扇: 档位(1~5)
 */
void Logic_SetDeviceManual(uint8_t dev, uint32_t duration_seconds, uint8_t value);

/**
 * @brief  立即恢复指定设备为自动控制模式
 * @param  dev  设备索引 (LOGIC_DEV_xxx)
 */
void Logic_RestoreDeviceAuto(uint8_t dev);

/**
 * @brief  查询指定设备的手动/自动状态
 * @param  dev  设备索引 (LOGIC_DEV_xxx, 不含风扇)
 * @return 当前状态枚举值
 */
LogicManualStatus_t Logic_GetDeviceManualStatus(uint8_t dev);

/**
 * @brief  获取风扇当前显示档位 (1~5)
 * @return 档位值
 */
uint8_t Logic_GetFanDisplayGear(void);

/**
 * @brief  获取风扇模式和档位
 * @param  is_manual      输出: 0=自动, 1=手动
 * @param  fan_gear_1_to_5 输出: 档位(1~5)
 */
void Logic_GetFanModeAndGear(uint8_t *is_manual, uint8_t *fan_gear_1_to_5);

/* ========== 第7组阈值参数ID：迟滞量和最小切换间隔 ========== */
#define LOGIC_CHANGE7_HYST_TEMP_ID     48U  /**< 温度迟滞量ID */
#define LOGIC_CHANGE7_HYST_HUMI_ID     49U  /**< 湿度迟滞量ID */
#define LOGIC_CHANGE7_HYST_LIGHT_ID    50U  /**< 光照迟滞量ID */
#define LOGIC_CHANGE7_HYST_GAS_ID      51U  /**< 气体迟滞量ID */
#define LOGIC_CHANGE7_HYST_PM25_ID     52U  /**< PM2.5迟滞量ID */
#define LOGIC_CHANGE7_MIN_SWITCH_ID    53U  /**< 最小切换间隔ID */

/**
 * @brief  第7组阈值修改：迟滞量及最小切换间隔
 * @param  threshold_id 参数ID (48~53)
 * @param  value        新值（最小切换间隔单位为秒）
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange7ThresholdById(uint8_t threshold_id, float value);

/** @brief  获取第7组阈值当前值 */
void Logic_GetChange7Thresholds(float *hyst_temp, float *hyst_humi,
                                float *hyst_light, float *hyst_gas,
                                float *hyst_pm25, float *min_switch_s);

/**
 * @brief  温度控制（带迟滞）
 * @param  temperature 当前温度(°C)
 * @param  temp_low    加热开启阈值
 * @param  temp_high   制冷开启阈值
 */
void Logic_ControlTemperature(float temperature, float temp_low, float temp_high);

/**
 * @brief  湿度控制（带迟滞）
 * @param  humidity     当前湿度(%RH)
 * @param  humidity_low 加湿开启阈值
 */
void Logic_ControlHumidity(float humidity, float humidity_low);

/**
 * @brief  灯光控制（人体感应 + 光照判断，带迟滞）
 * @param  human_presence          人体存在标志 (0/1)
 * @param  human_distance          人体距离(mm)
 * @param  light_lux               当前光照强度(lux)
 * @param  light_low               光照开灯阈值
 * @param  human_light_distance_max 人体感应有效距离上限
 */
void Logic_ControlLighting(uint8_t human_presence,
                           uint16_t human_distance,
                           uint16_t light_lux,
                           uint16_t light_low,
                           uint16_t human_light_distance_max);

/**
 * @brief  报警控制（多条件延时报警：门未关/气体超标/PM2.5超标）
 * @param  door_closed       门磁状态 (1=关闭, 0=打开)
 * @param  gas               当前气体浓度(ppm)
 * @param  pm25              当前 PM2.5 浓度(ug/m3)
 * @param  door_open_alarm_ms 门报警延时(ms)
 * @param  gas_high_alarm_ms  气体报警延时(ms)
 * @param  pm25_high_alarm_ms PM2.5报警延时(ms)
 * @param  gas_high           气体报警阈值
 * @param  pm25_high          PM2.5报警阈值
 */
void Logic_ControlAlarm(uint8_t door_closed,
                        uint16_t gas,
                        uint16_t pm25,
                        uint32_t door_open_alarm_ms,
                        uint32_t gas_high_alarm_ms,
                        uint32_t pm25_high_alarm_ms,
                        uint16_t gas_high,
                        uint16_t pm25_high);

/**
 * @brief  主控制函数，由FreeRTOS任务周期性调用
 * @param  data       所有传感器数据汇聚结构体指针
 * @param  thresholds 阈值参数指针，为NULL时使用默认值
 * @note   内部依次调用温度/湿度/灯光/报警/风扇控制函数，
 *         传感器无效时使用安全默认值，保证手动覆盖始终可解析
 */
void Logic_Run(const AllSensorData_t *data, const LogicThresholds_t *thresholds);

#endif /* LOGIC_H */
