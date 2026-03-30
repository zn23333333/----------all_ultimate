/**
 * @file    logic.c
 * @brief   环境保障系统 —— 自动控制逻辑模块（实现）
 *
 * 实现内容：
 *   1. 带迟滞的温度控制（加热器/制冷器）
 *   2. 带迟滞的湿度控制（加湿器/除湿=风扇）
 *   3. 带人体感应的灯光控制
 *   4. 多条件报警控制（门未关/气体超标/PM超标，含延时）
 *   5. 多源风扇档位控制（PM2.5分段、气体分段、除湿，取最高档）
 *   6. 手动覆盖机制（定时手动控制，到期自动恢复）
 *   7. 最小切换间隔保护（防止继电器抖动）
 *   8. 7组阈值修改接口 (Change1~Change7)
 */
#include "logic.h"

#include "Modbus_Relay.h"
#include "motor_fan.h"
#include "screen.h"
#include "runtime_config.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========== 继电器状态跟踪 ========== */
static uint8_t s_last_relay_state[5];             /**< 各继电器上次输出状态 (0=关, 1=开, 0xFF=未初始化) */
static TickType_t s_last_relay_switch_tick[5];     /**< 各继电器上次切换时刻 (tick)，用于最小切换间隔保护 */

/* ========== 报警延时跟踪器 ========== */
static uint8_t s_door_open_tracking = 0U;          /**< 门打开状态跟踪标志 */
static TickType_t s_door_open_start_tick = 0U;     /**< 门打开状态起始时刻 */
static uint8_t s_gas_high_tracking = 0U;           /**< 气体超标跟踪标志 */
static TickType_t s_gas_high_start_tick = 0U;      /**< 气体超标起始时刻 */
static uint8_t s_pm25_high_tracking = 0U;          /**< PM2.5超标跟踪标志 */
static TickType_t s_pm25_high_start_tick = 0U;     /**< PM2.5超标起始时刻 */

/* ========== 运行时阈值 & 风扇状态报告 ========== */
static LogicThresholds_t s_runtime_thresholds;     /**< 当前运行时阈值（可在线修改） */
static uint8_t s_last_fan_report_valid = 0U;       /**< 风扇状态报告有效标志（首次报告前为0） */
static uint8_t s_last_fan_report_level = 0U;       /**< 上次报告的风扇档位 */
static uint8_t s_last_fan_report_manual = 0U;      /**< 上次报告的风扇手动模式标志 */

/**
 * @brief 继电器设备手动覆盖结构体
 * @note  记录手动模式的激活状态、强制开/关、起始时刻和持续时长
 */
typedef struct
{
    uint8_t active;           /**< 手动覆盖是否激活 */
    uint8_t force_on;         /**< 强制状态: 0=关, 1=开 */
    TickType_t start_tick;    /**< 手动模式起始时刻 */
    TickType_t duration_ticks;/**< 手动模式持续 tick 数 */
} LogicManualOverride_t;

/**
 * @brief 风扇手动覆盖结构体
 * @note  与继电器类似，但强制的是风扇档位而非开/关
 */
typedef struct
{
    uint8_t active;           /**< 手动覆盖是否激活 */
    uint8_t force_level;      /**< 强制风扇档位 (MOTOR_LEVEL 索引) */
    TickType_t start_tick;    /**< 手动模式起始时刻 */
    TickType_t duration_ticks;/**< 手动模式持续 tick 数 */
} LogicFanManualOverride_t;

/* ========== 手动覆盖实例 & 自动目标状态 ========== */
static LogicManualOverride_t s_relay_manual[LOGIC_DEV_RELAY_COUNT]; /**< 各继电器设备手动覆盖实例 */
static LogicFanManualOverride_t s_fan_manual = {0U, 0U, 0U, 0U};   /**< 风扇手动覆盖实例 */
static uint8_t s_auto_target[LOGIC_DEV_RELAY_COUNT]; /**< 各继电器设备自动控制目标状态 (0=关, 1=开) */
static uint8_t s_fan_auto_target_level = 0U;         /**< 风扇自动控制目标档位 */

/**
 * @brief 设备索引 -> 继电器通道映射表
 * @note  0xFF 表示无直接对应继电器（如除湿器通过风扇实现）
 */
static const uint8_t s_dev_to_relay[LOGIC_DEV_RELAY_COUNT] = {
    LOGIC_RELAY_HEATER,       /* LOGIC_DEV_HEATER      -> 加热器继电器 */
    LOGIC_RELAY_COOLER,       /* LOGIC_DEV_COOLER      -> 制冷器继电器 */
    LOGIC_RELAY_HUMIDIFIER,   /* LOGIC_DEV_HUMIDIFIER  -> 加湿器继电器 */
    0xFFU,                    /* LOGIC_DEV_DEHUMIDIFIER-> 无继电器，用风扇 */
    LOGIC_RELAY_ALARM,        /* LOGIC_DEV_ALARM       -> 报警继电器 */
    LOGIC_RELAY_LIGHT,        /* LOGIC_DEV_LIGHT       -> 灯光继电器 */
};

/* ========== Change1~Change6 阈值参数ID定义 ========== */
/* Change1: 温湿度控制阈值 */
#define LOGIC_CHANGE1_HEATER_ID       0U   /**< 加热器开启温度 */
#define LOGIC_CHANGE1_COOLER_ID       1U   /**< 制冷器开启温度 */
#define LOGIC_CHANGE1_HUMIDIFIER_ID   2U   /**< 加湿器开启湿度 */
#define LOGIC_CHANGE1_DEHUMID_ID      3U   /**< 除湿风扇开启湿度 */
/* Change2: PM2.5/气体报警参数 */
#define LOGIC_CHANGE2_PM25_DELAY_ID   4U   /**< PM2.5报警延时(秒) */
#define LOGIC_CHANGE2_GAS_DELAY_ID    5U   /**< 气体报警延时(秒) */
#define LOGIC_CHANGE2_PM25_ALARM_ID   6U   /**< PM2.5报警阈值 */
#define LOGIC_CHANGE2_GAS_ALARM_ID    7U   /**< 气体报警阈值 */
/* Change3: 门报警/光照/人体/风扇L1 */
#define LOGIC_CHANGE3_DOOR_ALARM_ID   8U   /**< 门报警延时(秒) */
#define LOGIC_CHANGE3_LIGHT_LOW_ID    9U   /**< 光照开灯阈值 */
#define LOGIC_CHANGE3_HUMAN_DIST_ID   10U  /**< 人体感应有效距离 */
#define LOGIC_CHANGE3_FAN_L1_ID       11U  /**< 风扇档1速度% */
/* Change4: 风扇L2~L5速度 */
#define LOGIC_CHANGE4_FAN_L2_ID       12U  /**< 风扇档2速度% */
#define LOGIC_CHANGE4_FAN_L3_ID       13U  /**< 风扇档3速度% */
#define LOGIC_CHANGE4_FAN_L4_ID       14U  /**< 风扇档4速度% */
#define LOGIC_CHANGE4_FAN_L5_ID       15U  /**< 风扇档5速度% */
/* Change5: PM2.5分5段风扇范围 (偶数ID=下界, 奇数ID=上界) */
#define LOGIC_CHANGE5_PM_L1_LOW_ID    17U
#define LOGIC_CHANGE5_PM_L1_HIGH_ID   18U
#define LOGIC_CHANGE5_PM_L2_LOW_ID    19U
#define LOGIC_CHANGE5_PM_L2_HIGH_ID   20U
#define LOGIC_CHANGE5_PM_L3_LOW_ID    21U
#define LOGIC_CHANGE5_PM_L3_HIGH_ID   22U
#define LOGIC_CHANGE5_PM_L4_LOW_ID    23U
#define LOGIC_CHANGE5_PM_L4_HIGH_ID   24U
#define LOGIC_CHANGE5_PM_L5_LOW_ID    25U
#define LOGIC_CHANGE5_PM_L5_HIGH_ID   26U
/* Change6: 气体分5段风扇范围 (偶数ID=下界, 奇数ID=上界) */
#define LOGIC_CHANGE6_GAS_L1_LOW_ID   33U
#define LOGIC_CHANGE6_GAS_L1_HIGH_ID  34U
#define LOGIC_CHANGE6_GAS_L2_LOW_ID   35U
#define LOGIC_CHANGE6_GAS_L2_HIGH_ID  36U
#define LOGIC_CHANGE6_GAS_L3_LOW_ID   37U
#define LOGIC_CHANGE6_GAS_L3_HIGH_ID  38U
#define LOGIC_CHANGE6_GAS_L4_LOW_ID   39U
#define LOGIC_CHANGE6_GAS_L4_HIGH_ID  40U
#define LOGIC_CHANGE6_GAS_L5_LOW_ID   41U
#define LOGIC_CHANGE6_GAS_L5_HIGH_ID  42U

/**
 * @brief  将风扇档位限制在有效范围内
 * @param  level 输入档位
 * @return 限制后的档位 (0 ~ MOTOR_LEVEL_COUNT-1)
 */
static uint8_t ClampFanLevel(uint8_t level)
{
    if (level >= MOTOR_LEVEL_COUNT)
    {
        return (uint8_t)(MOTOR_LEVEL_COUNT - 1U);
    }
    return level;
}

/**
 * @brief  将浮点数安全转换为 uint16_t，带四舍五入和边界限制
 * @param  value 输入浮点值
 * @return 转换后的 uint16_t 值 (0~65535)
 */
static uint16_t ClampToU16FromFloat(float value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)(value + 0.5f);
}

/**
 * @brief  将秒数转换为毫秒，带四舍五入和溢出保护
 * @param  seconds 秒数（浮点）
 * @return 对应毫秒数 (uint32_t)
 */
static uint32_t SecondsToMsClamped(float seconds)
{
    if (seconds <= 0.0f)
    {
        return 0UL;
    }
    if (seconds >= 4294967.0f)
    {
        return 4294967295UL;
    }
    return (uint32_t)(seconds * 1000.0f + 0.5f);
}

/**
 * @brief  将浮点值限制为 0~100 的风扇PWM速度百分比
 * @param  value 输入浮点值
 * @return 0~100 的 uint8_t 值
 */
static uint8_t ClampFanSpeedPercent(float value)
{
    uint8_t speed = (uint8_t)ClampToU16FromFloat(value);

    if (speed > 100U)
    {
        speed = 100U;
    }

    return speed;
}

/**
 * @brief  将PM/气体范围值限制在 0~9999
 * @param  value 输入值
 * @return 限制后的 uint16_t 值
 */
static uint16_t ClampPmRangeValue(uint32_t value)
{
    if (value > 9999U)
    {
        return 9999U;
    }
    return (uint16_t)value;
}

/**
 * @brief  获取当前FreeRTOS tick计数
 * @return 当前 tick 值
 */
static TickType_t Logic_NowTick(void)
{
    return xTaskGetTickCount();
}

/**
 * @brief  将秒数转换为FreeRTOS tick数，带溢出保护
 * @param  duration_seconds 持续时间(秒)
 * @return 对应的 tick 数（至少1）
 */
static TickType_t SecondsToTicksClamped(uint32_t duration_seconds)
{
    uint64_t ms64 = (uint64_t)duration_seconds * 1000ULL;
    uint64_t ticks64;
    TickType_t ticks;

    ticks64 = (ms64 * (uint64_t)configTICK_RATE_HZ + 999ULL) / 1000ULL;
    if (ticks64 == 0ULL)
    {
        ticks64 = 1ULL;
    }
    if (ticks64 > (uint64_t)0xFFFFFFFFUL)
    {
        ticks64 = (uint64_t)0xFFFFFFFFUL;
    }

    ticks = (TickType_t)ticks64;
    if (ticks == 0U)
    {
        ticks = 1U;
    }
    return ticks;
}

/**
 * @brief  激活继电器手动覆盖
 * @param  ov               手动覆盖结构体指针
 * @param  duration_seconds 持续时间(秒)，0则不激活
 * @param  force_on         强制状态: 0=关, 非0=开
 */
static void ManualOverride_Set(LogicManualOverride_t *ov, uint32_t duration_seconds, uint8_t force_on)
{
    if (duration_seconds == 0U) return;
    ov->active = 1U;
    ov->force_on = (force_on != 0U) ? 1U : 0U;
    ov->start_tick = Logic_NowTick();
    ov->duration_ticks = SecondsToTicksClamped(duration_seconds);
}

/**
 * @brief  取消继电器手动覆盖，恢复自动控制
 * @param  ov  手动覆盖结构体指针
 */
static void ManualOverride_RestoreAuto(LogicManualOverride_t *ov)
{
    ov->active = 0U;
    ov->force_on = 0U;
    ov->start_tick = 0U;
    ov->duration_ticks = 0U;
}

/**
 * @brief  将风扇档位(1~5)转换为电机 level 索引(0~N)
 * @param  fan_gear_1_to_5 用户输入的档位 (1起步)
 * @return 电机 level 索引 (0起步)
 */
static uint8_t ClampFanLevelFromGear(uint8_t fan_gear_1_to_5)
{
    if (fan_gear_1_to_5 <= 1U)
    {
        return 0U;
    }
    if (fan_gear_1_to_5 >= MOTOR_LEVEL_COUNT)
    {
        return (uint8_t)(MOTOR_LEVEL_COUNT - 1U);
    }
    return (uint8_t)(fan_gear_1_to_5 - 1U);
}

/**
 * @brief  激活风扇手动覆盖
 * @param  ov               风扇手动覆盖结构体指针
 * @param  duration_seconds 持续时间(秒)，0则不激活
 * @param  fan_gear_1_to_5  强制档位 (1~5)
 */
static void FanManualOverride_Set(LogicFanManualOverride_t *ov, uint32_t duration_seconds, uint8_t fan_gear_1_to_5)
{
    if (duration_seconds == 0U) return;
    ov->active = 1U;
    ov->force_level = ClampFanLevelFromGear(fan_gear_1_to_5);
    ov->start_tick = Logic_NowTick();
    ov->duration_ticks = SecondsToTicksClamped(duration_seconds);
}

/**
 * @brief  取消风扇手动覆盖，恢复自动控制
 * @param  ov  风扇手动覆盖结构体指针
 */
static void FanManualOverride_RestoreAuto(LogicFanManualOverride_t *ov)
{
    ov->active = 0U;
    ov->force_level = 0U;
    ov->start_tick = 0U;
    ov->duration_ticks = 0U;
}

/**
 * @brief  解析风扇手动覆盖：判断是否超时，返回最终风扇档位
 * @param  ov               风扇手动覆盖结构体指针
 * @param  auto_target_level 自动控制目标档位
 * @param  manual_active    输出: 手动模式是否仍然激活 (NULL则不输出)
 * @return 最终使用的风扇档位
 * @note   在临界区内使用，保证超时检查和状态读取的原子性
 */
static uint8_t FanManualOverride_Resolve(LogicFanManualOverride_t *ov,
                                         uint8_t auto_target_level,
                                         uint8_t *manual_active)
{
    uint8_t active;
    uint8_t target_level;
    TickType_t now;

    taskENTER_CRITICAL();
    active = ov->active;
    if (active != 0U)
    {
        now = Logic_NowTick();
        /* 检查手动覆盖是否已超时，超时则自动取消 */
        if ((now - ov->start_tick) >= ov->duration_ticks)
        {
            ov->active = 0U;
            active = 0U;
        }
    }

    /* 手动激活时使用强制档位，否则使用自动目标档位 */
    target_level = active ? ClampFanLevel(ov->force_level) : ClampFanLevel(auto_target_level);
    if (manual_active) *manual_active = active;
    taskEXIT_CRITICAL();

    return target_level;
}

/**
 * @brief  将继电器通道号转换为串口屏设备枚举值
 * @param  relay_num 继电器通道号 (LOGIC_RELAY_xxx)
 * @return 对应的串口屏设备枚举值
 */
static ScreenActionDevice_t RelayNumToScreenDevice(uint8_t relay_num)
{
    switch (relay_num)
    {
    case LOGIC_RELAY_HEATER:
        return SCREEN_ACTION_DEV_HEATER;
    case LOGIC_RELAY_COOLER:
        return SCREEN_ACTION_DEV_COOLER;
    case LOGIC_RELAY_HUMIDIFIER:
        return SCREEN_ACTION_DEV_HUMIDIFIER;
    case LOGIC_RELAY_LIGHT:
        return SCREEN_ACTION_DEV_LIGHT;
    case LOGIC_RELAY_ALARM:
        return SCREEN_ACTION_DEV_ALARM;
    default:
        return SCREEN_ACTION_DEV_HEATER;
    }
}

/**
 * @brief  解析继电器手动覆盖：判断是否超时，返回最终目标状态
 * @param  ov            手动覆盖结构体指针
 * @param  auto_target   自动控制目标状态 (0=关, 1=开)
 * @param  manual_active 输出: 手动模式是否仍然激活 (NULL则不输出)
 * @return 最终的继电器目标状态
 * @note   在临界区内执行，保证超时检查和状态读取的原子性
 */
static uint8_t ManualOverride_Resolve(LogicManualOverride_t *ov, uint8_t auto_target, uint8_t *manual_active)
{
    uint8_t active;
    uint8_t target;
    TickType_t now;

    taskENTER_CRITICAL();
    active = ov->active;
    if (active != 0U)
    {
        now = Logic_NowTick();
        /* 检查手动覆盖是否已超时，超时则自动取消 */
        if ((now - ov->start_tick) >= ov->duration_ticks)
        {
            ov->active = 0U;
            active = 0U;
        }
    }

    /* 手动激活时使用强制状态，否则使用自动目标 */
    target = active ? ov->force_on : auto_target;
    if (manual_active) *manual_active = active;
    taskEXIT_CRITICAL();

    return target;
}

/**
 * @brief  在5段范围映射中查找匹配的最高风扇档位
 * @param  value        当前传感器值 (PM2.5或气体浓度)
 * @param  lower_bounds 各5段下界数组
 * @param  upper_bounds 各5段上界数组
 * @param  level_map    各5段对应的风扇档位数组
 * @param  matched      输出: 是否命中任一段 (NULL则不输出)
 * @return 命中段中的最高档位，未命中返回0
 * @note   多段可能重叠匹配，始终取最高档位
 */
static uint8_t SelectLevelByRanges(uint16_t value,
                                   const uint16_t lower_bounds[5],
                                   const uint16_t upper_bounds[5],
                                   const uint8_t level_map[5],
                                   uint8_t *matched)
{
    uint8_t i;
    uint8_t has_match = 0U;
    uint8_t target_level = 0U;
    uint8_t candidate;

    for (i = 0U; i < 5U; i++)
    {
        /* 检查当前值是否落在第i段 [lower, upper] 区间内 */
        if ((value >= lower_bounds[i]) && (value <= upper_bounds[i]))
        {
            candidate = ClampFanLevel(level_map[i]);
            /* 取所有匹配段中的最高档位 */
            if ((!has_match) || (candidate > target_level))
            {
                target_level = candidate;
                has_match = 1U;
            }
        }
    }

    if (matched != NULL)
    {
        *matched = has_match;
    }

    return target_level;
}

/**
 * @brief  多源风扇档位控制
 *
 * 综合考虑以下三个风扇需求源，取最高档位：
 *   - 除湿需求（湿度超过上限时开启指定档位）
 *   - PM2.5 分段映射（5段范围匹配）
 *   - 气体浓度分段映射（5段范围匹配）
 *
 * @param  data 传感器数据
 * @param  cfg  阈值配置
 */
static void Logic_ControlFanLevel(const AllSensorData_t *data, const LogicThresholds_t *cfg)
{
    uint8_t has_pm;
    uint8_t has_gas;
    uint8_t has_humidity;
    uint8_t dehumidifier_auto = 0U;   /* 除湿自动控制目标 */
    uint8_t dehumidifier_target;
    uint8_t level_valid = 0U;         /* 是否已有有效档位 */
    uint8_t target_level = 0U;        /* 综合最高档位 */
    uint8_t candidate;
    uint8_t pm_matched;
    uint8_t gas_matched;
    uint8_t fan_manual_active = 0U;

    has_humidity = ((data->valid_mask & ALL_SENSOR_VALID_TEMP_HUMI) != 0U) ? 1U : 0U;
    has_pm = ((data->valid_mask & ALL_SENSOR_VALID_PM) != 0U) ? 1U : 0U;
    has_gas = ((data->valid_mask & ALL_SENSOR_VALID_GAS) != 0U) ? 1U : 0U;

    /* ---- 除湿迟滞控制 ---- */
    if (has_humidity)
    {
        uint8_t dehumid_prev;
        float hyst = s_runtime_thresholds.hyst_humi;

        taskENTER_CRITICAL();
        dehumid_prev = s_auto_target[LOGIC_DEV_DEHUMIDIFIER];
        taskEXIT_CRITICAL();

        /* 迟滞逻辑: 已开启时需降到 (high-hyst) 才关闭，未开启时需超过 high 才开启 */
        if (dehumid_prev)
        {
            dehumidifier_auto = (data->humidity < (cfg->humidity_fan_high - hyst)) ? 0U : 1U;
        }
        else
        {
            dehumidifier_auto = (data->humidity > cfg->humidity_fan_high) ? 1U : 0U;
        }
    }

    taskENTER_CRITICAL();
    s_auto_target[LOGIC_DEV_DEHUMIDIFIER] = dehumidifier_auto;
    taskEXIT_CRITICAL();

    /* 解析除湿手动覆盖 */
    dehumidifier_target = ManualOverride_Resolve(&s_relay_manual[LOGIC_DEV_DEHUMIDIFIER], dehumidifier_auto, NULL);

    /* 除湿激活时，以配置的除湿风扇档位作为候选 */
    if (dehumidifier_target != 0U)
    {
        target_level = ClampFanLevel(cfg->humidity_fan_level);
        level_valid = 1U;
    }

    /* ---- PM2.5 分段风扇档位 ---- */
    if (has_pm)
    {
        candidate = SelectLevelByRanges(data->pm25,
                                        cfg->pm25_fan_lower,
                                        cfg->pm25_fan_upper,
                                        cfg->pm25_fan_level,
                                        &pm_matched);
        /* PM2.5匹配的档位与已有档位取较高者 */
        if (pm_matched && ((!level_valid) || (candidate > target_level)))
        {
            target_level = candidate;
            level_valid = 1U;
        }
    }

    /* ---- 气体分段风扇档位 ---- */
    if (has_gas)
    {
        candidate = SelectLevelByRanges(data->gas,
                                        cfg->gas_fan_lower,
                                        cfg->gas_fan_upper,
                                        cfg->gas_fan_level,
                                        &gas_matched);
        /* 气体匹配的档位与已有档位取较高者 */
        if (gas_matched && ((!level_valid) || (candidate > target_level)))
        {
            target_level = candidate;
            level_valid = 1U;
        }
    }

    if (!level_valid)
    {
        target_level = 0U; /* 无任何风扇需求源命中，默认最低档 */
    }

    taskENTER_CRITICAL();
    s_fan_auto_target_level = ClampFanLevel(target_level); /* 保存自动目标档位 */
    taskEXIT_CRITICAL();

    /* 解析风扇手动覆盖，得到最终输出档位 */
    target_level = FanManualOverride_Resolve(&s_fan_manual, target_level, &fan_manual_active);
    Motor_SetLevel(target_level); /* 设置电机PWM输出 */

    /* 风扇状态变化时通知串口屏 */
    if ((s_last_fan_report_valid == 0U) ||
        (s_last_fan_report_level != target_level) ||
        (s_last_fan_report_manual != fan_manual_active))
        {
        Screen_RecordDeviceAction(SCREEN_ACTION_DEV_FAN,
                                  fan_manual_active,
                                  (uint8_t)(target_level + 1U));
        s_last_fan_report_valid = 1U;
        s_last_fan_report_level = target_level;
        s_last_fan_report_manual = fan_manual_active;
    }
}

/**
 * @brief  带保护的继电器状态切换
 *
 * 仅在目标状态与当前状态不同时才执行切换，
 * 并检查最小切换间隔保护，防止继电器频繁抖动。
 *
 * @param  relay_num    继电器通道号 (0~4)
 * @param  target_state 目标状态 (0=关, 1=开)
 * @param  is_manual    是否手动模式 (用于串口屏显示)
 */
static void SetRelayIfChanged(uint8_t relay_num, uint8_t target_state, uint8_t is_manual)
{
    TickType_t now;
    TickType_t min_hold_ticks;

    if (relay_num >= 5U)
    {
        return;
    }

    /* 状态未变化，无需操作 */
    if (s_last_relay_state[relay_num] == target_state)
    {
        return;
    }

    /* 最小切换间隔保护：距离上次切换未超过 min_switch_ms 则跳过 */
    now = Logic_NowTick();
    min_hold_ticks = pdMS_TO_TICKS(s_runtime_thresholds.min_switch_ms);
    if ((s_last_relay_switch_tick[relay_num] != 0U) &&
        ((now - s_last_relay_switch_tick[relay_num]) < min_hold_ticks))
        {
        return;
    }

    /* 执行 Modbus 写继电器并记录状态 */
    if (WriteRelay(relay_num, target_state))
    {
        s_last_relay_state[relay_num] = target_state;
        s_last_relay_switch_tick[relay_num] = now;
        /* 通知串口屏记录设备动作 */
        Screen_RecordDeviceAction(RelayNumToScreenDevice(relay_num),
                                  is_manual,
                                  target_state);
    }
}

/**
 * @brief  初始化控制逻辑模块
 * @note   复位所有继电器状态、报警跟踪器、手动覆盖状态，加载默认阈值
 */
void Logic_Init(void)
{
    /* 复位继电器状态为“未初始化”(0xFF)，确保首次运行时强制写入 */
    for (uint8_t i = 0U; i < 5U; i++)
    {
        s_last_relay_state[i] = 0xFFU;
        s_last_relay_switch_tick[i] = 0U;
    }

    /* 复位报警延时跟踪器 */
    s_door_open_tracking = 0U;
    s_door_open_start_tick = 0U;
    s_gas_high_tracking = 0U;
    s_gas_high_start_tick = 0U;
    s_pm25_high_tracking = 0U;
    s_pm25_high_start_tick = 0U;
    /* 复位风扇状态报告 */
    s_fan_auto_target_level = 0U;
    s_last_fan_report_valid = 0U;
    s_last_fan_report_level = 0U;
    s_last_fan_report_manual = 0U;
    /* 复位所有设备手动覆盖，恢复自动模式 */
    for (uint8_t i = 0U; i < LOGIC_DEV_RELAY_COUNT; i++)
    {
        s_auto_target[i] = 0U;
        ManualOverride_RestoreAuto(&s_relay_manual[i]);
    }
    FanManualOverride_RestoreAuto(&s_fan_manual);
    /* 加载默认阈值 */
    Logic_SetDefaultThresholds(&s_runtime_thresholds);
}

/**
 * @brief  将阈值结构体填充为出厂默认值
 * @param  out_thresholds 输出指针，不可为 NULL
 * @note   默认值: 温度16~25°C, 湿度50~80%RH, 光照50lux, 气体/PM报警50,
 *         报警延时5分钟, 迟滞温1°C/5%RH/20lux/5ppm/5ug, 最小切换间隔30秒
 */
void Logic_SetDefaultThresholds(LogicThresholds_t *out_thresholds)
{
    if (out_thresholds == NULL)
    {
        return;
    }

    out_thresholds->temp_low = 16.0f;
    out_thresholds->temp_high = 25.0f;
    out_thresholds->humidity_low = 50.0f;
    out_thresholds->humidity_fan_high = 80.0f;
    out_thresholds->humidity_fan_level = 1U;
    out_thresholds->light_low = 50U;
    out_thresholds->human_light_distance_max = 999U;
    out_thresholds->gas_high = 50U;
    out_thresholds->pm25_high = 50U;
    out_thresholds->pm25_fan_lower[0] = 0U;
    out_thresholds->pm25_fan_lower[1] = 51U;
    out_thresholds->pm25_fan_lower[2] = 101U;
    out_thresholds->pm25_fan_lower[3] = 201U;
    out_thresholds->pm25_fan_lower[4] = 401U;
    out_thresholds->pm25_fan_upper[0] = 50U;
    out_thresholds->pm25_fan_upper[1] = 100U;
    out_thresholds->pm25_fan_upper[2] = 200U;
    out_thresholds->pm25_fan_upper[3] = 400U;
    out_thresholds->pm25_fan_upper[4] = 999U;
    out_thresholds->pm25_fan_level[0] = 0U;
    out_thresholds->pm25_fan_level[1] = 1U;
    out_thresholds->pm25_fan_level[2] = 2U;
    out_thresholds->pm25_fan_level[3] = 3U;
    out_thresholds->pm25_fan_level[4] = 4U;
    out_thresholds->gas_fan_upper[0] = 20U;
    out_thresholds->gas_fan_upper[1] = 40U;
    out_thresholds->gas_fan_upper[2] = 80U;
    out_thresholds->gas_fan_upper[3] = 160U;
    out_thresholds->gas_fan_upper[4] = 999U;
    out_thresholds->gas_fan_lower[0] = 0U;
    out_thresholds->gas_fan_lower[1] = 21U;
    out_thresholds->gas_fan_lower[2] = 41U;
    out_thresholds->gas_fan_lower[3] = 81U;
    out_thresholds->gas_fan_lower[4] = 161U;
    out_thresholds->gas_fan_level[0] = 0U;
    out_thresholds->gas_fan_level[1] = 1U;
    out_thresholds->gas_fan_level[2] = 2U;
    out_thresholds->gas_fan_level[3] = 3U;
    out_thresholds->gas_fan_level[4] = 4U;
    out_thresholds->door_open_alarm_ms = 0UL * 60UL * 1000UL;
    out_thresholds->gas_high_alarm_ms = 0UL * 60UL * 1000UL;
    out_thresholds->pm25_high_alarm_ms = 0UL * 60UL * 1000UL;
    out_thresholds->hyst_temp = 1.0f;
    out_thresholds->hyst_humi = 5.0f;
    out_thresholds->hyst_light = 20U;
    out_thresholds->hyst_gas = 5U;
    out_thresholds->hyst_pm25 = 5U;
    out_thresholds->min_switch_ms = 30000UL;
}

/**
 * @brief  设置运行时阈值（整体覆盖）
 * @param  thresholds 要写入的阈值结构体指针
 * @note   在临界区内执行整体拷贝，保证原子性
 */
void Logic_SetRuntimeThresholds(const LogicThresholds_t *thresholds)
{
    if (thresholds == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_runtime_thresholds = *thresholds;
    taskEXIT_CRITICAL();
}

/**
 * @brief  获取当前运行时阈值的副本
 * @param  out_thresholds 输出指针
 * @note   在临界区内执行整体拷贝，保证原子性
 */
void Logic_GetRuntimeThresholds(LogicThresholds_t *out_thresholds)
{
    if (out_thresholds == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out_thresholds = s_runtime_thresholds;
    taskEXIT_CRITICAL();
}

/**
 * @brief  第1组阈值修改：温湿度控制阈值
 * @param  threshold_id 参数ID (0=加热温度, 1=制冷温度, 2=加湿湿度, 3=除湿湿度)
 * @param  value        新阈值（负值会被限制为0）
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange1ThresholdById(uint8_t threshold_id, float value)
{
    if (value < 0.0f)
    {
        value = 0.0f;
    }

    taskENTER_CRITICAL();
    switch (threshold_id)
    {
    case LOGIC_CHANGE1_HEATER_ID:
        s_runtime_thresholds.temp_low = value;
        break;
    case LOGIC_CHANGE1_COOLER_ID:
        s_runtime_thresholds.temp_high = value;
        break;
    case LOGIC_CHANGE1_HUMIDIFIER_ID:
        s_runtime_thresholds.humidity_low = value;
        break;
    case LOGIC_CHANGE1_DEHUMID_ID:
        s_runtime_thresholds.humidity_fan_high = value;
        break;
    default:
        taskEXIT_CRITICAL();
        return 0U;
    }
    taskEXIT_CRITICAL();

    RuntimeConfig_RequestSave();
    return 1U;
}

/**
 * @brief  获取第1组阈值当前值（加热/制冷/加湿/除湿阈值）
 */
void Logic_GetChange1Thresholds(float *heater_on_threshold,
                                float *cooler_on_threshold,
                                float *humidifier_on_threshold,
                                float *dehumidifier_on_threshold)
{
    taskENTER_CRITICAL();
    if (heater_on_threshold != NULL)
    {
        *heater_on_threshold = s_runtime_thresholds.temp_low;
    }
    if (cooler_on_threshold != NULL)
    {
        *cooler_on_threshold = s_runtime_thresholds.temp_high;
    }
    if (humidifier_on_threshold != NULL)
    {
        *humidifier_on_threshold = s_runtime_thresholds.humidity_low;
    }
    if (dehumidifier_on_threshold != NULL)
    {
        *dehumidifier_on_threshold = s_runtime_thresholds.humidity_fan_high;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  第2组阈值修改：PM2.5/气体报警延时及报警阈值
 * @param  threshold_id 参数ID (4=PM2.5延时, 5=气体延时, 6=PM2.5阈值, 7=气体阈值)
 * @param  value        新阈值（延时单位为秒）
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange2ThresholdById(uint8_t threshold_id, float value)
{
    if (value < 0.0f)
    {
        value = 0.0f;
    }

    taskENTER_CRITICAL();
    switch (threshold_id)
    {
    case LOGIC_CHANGE2_PM25_DELAY_ID:
        s_runtime_thresholds.pm25_high_alarm_ms = SecondsToMsClamped(value);
        break;
    case LOGIC_CHANGE2_GAS_DELAY_ID:
        s_runtime_thresholds.gas_high_alarm_ms = SecondsToMsClamped(value);
        break;
    case LOGIC_CHANGE2_PM25_ALARM_ID:
        s_runtime_thresholds.pm25_high = ClampToU16FromFloat(value);
        break;
    case LOGIC_CHANGE2_GAS_ALARM_ID:
        s_runtime_thresholds.gas_high = ClampToU16FromFloat(value);
        break;
    default:
        taskEXIT_CRITICAL();
        return 0U;
    }
    taskEXIT_CRITICAL();

    RuntimeConfig_RequestSave();
    return 1U;
}

/**
 * @brief  获取第2组阈值当前值（报警延时转换为秒返回）
 */
void Logic_GetChange2Thresholds(float *pm25_alarm_delay_seconds,
                                float *gas_alarm_delay_seconds,
                                float *pm25_alarm_threshold,
                                float *gas_alarm_threshold)
{
    taskENTER_CRITICAL();
    if (pm25_alarm_delay_seconds != NULL)
    {
        *pm25_alarm_delay_seconds = (float)s_runtime_thresholds.pm25_high_alarm_ms / 1000.0f;
    }
    if (gas_alarm_delay_seconds != NULL)
    {
        *gas_alarm_delay_seconds = (float)s_runtime_thresholds.gas_high_alarm_ms / 1000.0f;
    }
    if (pm25_alarm_threshold != NULL)
    {
        *pm25_alarm_threshold = (float)s_runtime_thresholds.pm25_high;
    }
    if (gas_alarm_threshold != NULL)
    {
        *gas_alarm_threshold = (float)s_runtime_thresholds.gas_high;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  第3组阈值修改：门报警延时、光照阈值、人体距离、风扇档1速度
 * @param  threshold_id 参数ID (8=门报警延时, 9=光照, 10=人体距离, 11=风扇L1)
 * @param  value        新阈值
 * @return 1=成功, 0=ID无效
 * @note   风扇L1速度直接写入电机档位表，不经过阈值结构体
 */
uint8_t Logic_SetChange3ThresholdById(uint8_t threshold_id, float value)
{
    if (value < 0.0f)
    {
        value = 0.0f;
    }

    if (threshold_id == LOGIC_CHANGE3_FAN_L1_ID)
    {
        Motor_SetLevelValue(0U, ClampFanSpeedPercent(value));
        RuntimeConfig_RequestSave();
        return 1U;
    }

    taskENTER_CRITICAL();
    switch (threshold_id)
    {
    case LOGIC_CHANGE3_DOOR_ALARM_ID:
        s_runtime_thresholds.door_open_alarm_ms = SecondsToMsClamped(value);
        break;
    case LOGIC_CHANGE3_LIGHT_LOW_ID:
        s_runtime_thresholds.light_low = ClampToU16FromFloat(value);
        break;
    case LOGIC_CHANGE3_HUMAN_DIST_ID:
        s_runtime_thresholds.human_light_distance_max = ClampToU16FromFloat(value);
        break;
    default:
        taskEXIT_CRITICAL();
        return 0U;
    }
    taskEXIT_CRITICAL();

    RuntimeConfig_RequestSave();
    return 1U;
}

/**
 * @brief  获取第3组阈值当前值
 */
void Logic_GetChange3Thresholds(float *door_open_alarm_seconds,
                                float *light_on_threshold,
                                float *human_light_distance_threshold,
                                float *fan_level1_speed)
{
    const uint8_t *level_table;

    taskENTER_CRITICAL();
    if (door_open_alarm_seconds != NULL)
    {
        *door_open_alarm_seconds = (float)s_runtime_thresholds.door_open_alarm_ms / 1000.0f;
    }
    if (light_on_threshold != NULL)
    {
        *light_on_threshold = (float)s_runtime_thresholds.light_low;
    }
    if (human_light_distance_threshold != NULL)
    {
        *human_light_distance_threshold = (float)s_runtime_thresholds.human_light_distance_max;
    }
    if (fan_level1_speed != NULL)
    {
        level_table = Motor_GetLevelTable();
        *fan_level1_speed = (level_table != NULL) ? (float)level_table[0] : 0.0f;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  第4组阈值修改：风扇档2~5的PWM速度百分比
 * @param  threshold_id 参数ID (12=L2, 13=L3, 14=L4, 15=L5)
 * @param  value        速度百分比 (0~100)
 * @return 1=成功, 0=ID无效
 * @note   直接写入电机档位表，不经过阈值结构体
 */
uint8_t Logic_SetChange4ThresholdById(uint8_t threshold_id, float value)
{
    uint8_t speed;
    uint8_t level_index;

    if (value < 0.0f)
    {
        value = 0.0f;
    }

    speed = ClampFanSpeedPercent(value);

    switch (threshold_id)
    {
    case LOGIC_CHANGE4_FAN_L2_ID:
        level_index = 1U;
        break;
    case LOGIC_CHANGE4_FAN_L3_ID:
        level_index = 2U;
        break;
    case LOGIC_CHANGE4_FAN_L4_ID:
        level_index = 3U;
        break;
    case LOGIC_CHANGE4_FAN_L5_ID:
        level_index = 4U;
        break;
    default:
        return 0U;
    }

    Motor_SetLevelValue(level_index, speed);
    RuntimeConfig_RequestSave();
    return 1U;
}

/**
 * @brief  获取第4组阈值当前值（从电机档位表读取）
 */
void Logic_GetChange4Thresholds(float *fan_level2_speed,
                                float *fan_level3_speed,
                                float *fan_level4_speed,
                                float *fan_level5_speed)
{
    const uint8_t *level_table = Motor_GetLevelTable();

    if (fan_level2_speed != NULL)
    {
        *fan_level2_speed = (level_table != NULL) ? (float)level_table[1] : 0.0f;
    }
    if (fan_level3_speed != NULL)
    {
        *fan_level3_speed = (level_table != NULL) ? (float)level_table[2] : 0.0f;
    }
    if (fan_level4_speed != NULL)
    {
        *fan_level4_speed = (level_table != NULL) ? (float)level_table[3] : 0.0f;
    }
    if (fan_level5_speed != NULL)
    {
        *fan_level5_speed = (level_table != NULL) ? (float)level_table[4] : 0.0f;
    }
}

/**
 * @brief  第5组阈值修改：PM2.5分5段风扇范围上下界
 * @param  threshold_id 参数ID (17~26)，偶数偏移=下界，奇数偏移=上界
 * @param  value        范围值 (0~9999)
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange5ThresholdById(uint8_t threshold_id, uint32_t value)
{
    uint16_t range_value = ClampPmRangeValue(value);
    uint8_t offset, idx;

    /* 检查ID范围有效性 */
    if (threshold_id < LOGIC_CHANGE5_PM_L1_LOW_ID || threshold_id > LOGIC_CHANGE5_PM_L5_HIGH_ID)
        return 0U;

    /* 通过偏移计算段索引和上/下界: offset/2=段, offset%2=0下界/1上界 */
    offset = threshold_id - LOGIC_CHANGE5_PM_L1_LOW_ID;
    idx = offset / 2U;
    if (idx >= 5U) return 0U;

    taskENTER_CRITICAL();
    if (offset & 1U)
        s_runtime_thresholds.pm25_fan_upper[idx] = range_value;
    else
        s_runtime_thresholds.pm25_fan_lower[idx] = range_value;
    taskEXIT_CRITICAL();

    RuntimeConfig_RequestSave();
    return 1U;
}

/**
 * @brief  获取第5组阈值：PM2.5分5段上下界
 */
void Logic_GetChange5Thresholds(uint16_t lower_bounds[5], uint16_t upper_bounds[5])
{
    uint8_t i;

    taskENTER_CRITICAL();
    if (lower_bounds != NULL)
    {
        for (i = 0U; i < 5U; i++)
        {
            lower_bounds[i] = s_runtime_thresholds.pm25_fan_lower[i];
        }
    }
    if (upper_bounds != NULL)
    {
        for (i = 0U; i < 5U; i++)
        {
            upper_bounds[i] = s_runtime_thresholds.pm25_fan_upper[i];
        }
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  第6组阈值修改：气体分5段风扇范围上下界
 * @param  threshold_id 参数ID (33~42)，偶数偏移=下界，奇数偏移=上界
 * @param  value        范围值 (0~9999)
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange6ThresholdById(uint8_t threshold_id, uint32_t value)
{
    uint16_t range_value = ClampPmRangeValue(value);
    uint8_t offset, idx;

    /* 检查ID范围有效性 */
    if (threshold_id < LOGIC_CHANGE6_GAS_L1_LOW_ID || threshold_id > LOGIC_CHANGE6_GAS_L5_HIGH_ID)
        return 0U;

    /* 通过偏移计算段索引和上/下界 */
    offset = threshold_id - LOGIC_CHANGE6_GAS_L1_LOW_ID;
    idx = offset / 2U;
    if (idx >= 5U) return 0U;

    taskENTER_CRITICAL();
    if (offset & 1U)
        s_runtime_thresholds.gas_fan_upper[idx] = range_value;
    else
        s_runtime_thresholds.gas_fan_lower[idx] = range_value;
    taskEXIT_CRITICAL();

    RuntimeConfig_RequestSave();
    return 1U;
}

/**
 * @brief  获取第6组阈值：气体分5段上下界
 */
void Logic_GetChange6Thresholds(uint16_t lower_bounds[5], uint16_t upper_bounds[5])
{
    uint8_t i;

    taskENTER_CRITICAL();
    if (lower_bounds != NULL)
    {
        for (i = 0U; i < 5U; i++)
        {
            lower_bounds[i] = s_runtime_thresholds.gas_fan_lower[i];
        }
    }
    if (upper_bounds != NULL)
    {
        for (i = 0U; i < 5U; i++)
        {
            upper_bounds[i] = s_runtime_thresholds.gas_fan_upper[i];
        }
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  将所有阈值、风扇档位表、手动覆盖状态恢复为出厂默认值
 * @note   同时复位报警跟踪器和继电器切换时刻记录
 */
void Logic_ResetAllToDefaultsManual(void)
{
    LogicThresholds_t defaults;
    static const uint8_t default_fan_levels[MOTOR_LEVEL_COUNT] = {0U, 25U, 50U, 75U, 100U};

    Logic_SetDefaultThresholds(&defaults);
    taskENTER_CRITICAL();
    s_runtime_thresholds = defaults;
    s_door_open_tracking = 0U;
    s_door_open_start_tick = 0U;
    s_gas_high_tracking = 0U;
    s_gas_high_start_tick = 0U;
    s_pm25_high_tracking = 0U;
    s_pm25_high_start_tick = 0U;
    {
        uint8_t i;
        for (i = 0U; i < 5U; i++)
        {
            s_last_relay_switch_tick[i] = 0U;
        }
    }
    taskEXIT_CRITICAL();

    Motor_SetLevelTable(default_fan_levels);

    {
        uint8_t i;
        for (i = 0U; i < LOGIC_DEV_COUNT; i++)
        {
            Logic_RestoreDeviceAuto(i);
        }
    }
    taskENTER_CRITICAL();
    s_last_fan_report_valid = 0U;
    s_last_fan_report_level = 0U;
    s_last_fan_report_manual = 0U;
    taskEXIT_CRITICAL();
}

/**
 * @brief  设置指定设备进入手动模式
 * @param  dev              设备索引 (LOGIC_DEV_xxx)
 * @param  duration_seconds 手动持续时长(秒)，0则不激活
 * @param  value            继电器: 0=关/1=开；风扇: 档位(1~5)
 */
void Logic_SetDeviceManual(uint8_t dev, uint32_t duration_seconds, uint8_t value)
{
    if (dev == LOGIC_DEV_FAN)
    {
        taskENTER_CRITICAL();
        FanManualOverride_Set(&s_fan_manual, duration_seconds, value);
        taskEXIT_CRITICAL();
        return;
    }
    if (dev >= LOGIC_DEV_RELAY_COUNT) return;
    taskENTER_CRITICAL();
    ManualOverride_Set(&s_relay_manual[dev], duration_seconds, value);
    taskEXIT_CRITICAL();
}

/**
 * @brief  立即恢复指定设备为自动控制模式
 * @param  dev  设备索引 (LOGIC_DEV_xxx)
 */
void Logic_RestoreDeviceAuto(uint8_t dev)
{
    if (dev == LOGIC_DEV_FAN)
    {
        taskENTER_CRITICAL();
        FanManualOverride_RestoreAuto(&s_fan_manual);
        taskEXIT_CRITICAL();
        return;
    }
    if (dev >= LOGIC_DEV_RELAY_COUNT) return;
    taskENTER_CRITICAL();
    ManualOverride_RestoreAuto(&s_relay_manual[dev]);
    taskEXIT_CRITICAL();
}

/**
 * @brief  获取继电器设备的手动/自动状态（内部实现）
 * @param  ov           手动覆盖结构体指针
 * @param  auto_target  自动控制目标状态
 * @param  relay_index  继电器通道号（用于读取实际输出状态）
 * @return 设备状态枚举值
 * @note   当实际输出与目标不一致时（受最小切换间隔保护），返回 PENDING 状态
 */
static LogicManualStatus_t Logic_GetManualStatusInternal(LogicManualOverride_t *ov,
                                                         uint8_t auto_target,
                                                         uint8_t relay_index)
{
    uint8_t manual_active;
    uint8_t target_on;
    uint8_t output_on;

    /* 解析手动覆盖，判断是否已超时 */
    (void)ManualOverride_Resolve(ov, auto_target, &manual_active);

    /* 确定目标状态：手动激活时用强制值，否则用自动目标 */
    target_on = auto_target;
    if ((manual_active != 0U) && (ov != NULL))
    {
        target_on = ov->force_on;
    }

    /* 读取实际继电器输出状态 */
    output_on = target_on;
    if ((relay_index < 5U) && (s_last_relay_state[relay_index] != 0xFFU))
    {
        output_on = s_last_relay_state[relay_index];
    }

    if (manual_active != 0U)
    {
        /* 手动模式: 实际输出与目标不一致时返回 PENDING */
        if (output_on != target_on)
        {
            return (target_on != 0U) ? LOGIC_MANUAL_STATUS_MANUAL_ON_PENDING
                                     : LOGIC_MANUAL_STATUS_MANUAL_OFF_PENDING;
        }
        return (target_on != 0U) ? LOGIC_MANUAL_STATUS_MANUAL_ON : LOGIC_MANUAL_STATUS_MANUAL_OFF;
    }

    /* 自动模式 */
    return (output_on != 0U) ? LOGIC_MANUAL_STATUS_AUTO_ON : LOGIC_MANUAL_STATUS_AUTO_OFF;
}

/**
 * @brief  查询指定设备的手动/自动状态
 * @param  dev  设备索引 (LOGIC_DEV_xxx, 不含风扇)
 * @return 当前状态枚举值
 */
LogicManualStatus_t Logic_GetDeviceManualStatus(uint8_t dev)
{
    LogicManualStatus_t status;

    if (dev >= LOGIC_DEV_RELAY_COUNT) return LOGIC_MANUAL_STATUS_AUTO_OFF;
    taskENTER_CRITICAL();
    status = Logic_GetManualStatusInternal(&s_relay_manual[dev],
                                           s_auto_target[dev],
                                           s_dev_to_relay[dev]);
    taskEXIT_CRITICAL();
    return status;
}

/**
 * @brief  获取风扇当前显示档位 (1~5)
 * @return 档位值
 */
uint8_t Logic_GetFanDisplayGear(void)
{
    uint8_t is_manual = 0U;
    uint8_t fan_gear = 1U;

    Logic_GetFanModeAndGear(&is_manual, &fan_gear);
    (void)is_manual;
    return fan_gear;
}

/**
 * @brief  获取风扇当前模式和档位
 * @param  is_manual      输出: 0=自动, 1=手动 (NULL则不输出)
 * @param  fan_gear_1_to_5 输出: 档位 1~5 (NULL则不输出)
 * @note   内部会检查手动覆盖是否超时，超时自动取消
 */
void Logic_GetFanModeAndGear(uint8_t *is_manual, uint8_t *fan_gear_1_to_5)
{
    uint8_t active;
    uint8_t level;
    TickType_t now;

    taskENTER_CRITICAL();
    active = s_fan_manual.active;
    if (active != 0U)
    {
        now = Logic_NowTick();
        if ((now - s_fan_manual.start_tick) >= s_fan_manual.duration_ticks)
        {
            s_fan_manual.active = 0U;
            active = 0U;
        }
    }

    if (active != 0U)
    {
        level = ClampFanLevel(s_fan_manual.force_level);
    }
    else
    {
        level = ClampFanLevel(s_fan_auto_target_level);
    }
    taskEXIT_CRITICAL();

    if (is_manual != NULL)
    {
        *is_manual = active;
    }
    if (fan_gear_1_to_5 != NULL)
    {
        *fan_gear_1_to_5 = (uint8_t)(level + 1U);
    }
}

/**
 * @brief  温度控制（带迟滞）
 *
 * 迟滞逻辑：
 *   - 加热器已开: 温度超过 (temp_low + hyst) 才关闭
 *   - 加热器已关: 温度低于 temp_low 才开启
 *   - 制冷器已开: 温度低于 (temp_high - hyst) 才关闭
 *   - 制冷器已关: 温度超过 temp_high 才开启
 *
 * @param  temperature 当前温度(°C)
 * @param  temp_low    加热开启阈值
 * @param  temp_high   制冷开启阈值
 */
void Logic_ControlTemperature(float temperature, float temp_low, float temp_high)
{
    uint8_t heater_current = (s_last_relay_state[LOGIC_RELAY_HEATER] == 1U) ? 1U : 0U;
    uint8_t cooler_current = (s_last_relay_state[LOGIC_RELAY_COOLER] == 1U) ? 1U : 0U;
    float hyst = s_runtime_thresholds.hyst_temp;
    uint8_t heater_auto;
    uint8_t cooler_auto;
    uint8_t heater_target;
    uint8_t cooler_target;
    uint8_t heater_manual = 0U;
    uint8_t cooler_manual = 0U;

    /* 加热器迟滞控制: 已开启时需温度升超过 (low+hyst) 才关闭 */
    if (heater_current)
    {
        heater_auto = (temperature > (temp_low + hyst)) ? 0U : 1U;
    }
    else
    {
        heater_auto = (temperature < temp_low) ? 1U : 0U;
    }

    /* 制冷器迟滞控制: 已开启时需温度降低到 (high-hyst) 才关闭 */
    if (cooler_current)
    {
        cooler_auto = (temperature < (temp_high - hyst)) ? 0U : 1U;
    }
    else
    {
        cooler_auto = (temperature > temp_high) ? 1U : 0U;
    }

    taskENTER_CRITICAL();
    s_auto_target[LOGIC_DEV_HEATER] = heater_auto;
    s_auto_target[LOGIC_DEV_COOLER] = cooler_auto;
    taskEXIT_CRITICAL();

    /* 解析手动覆盖，得到最终目标状态 */
    heater_target = ManualOverride_Resolve(&s_relay_manual[LOGIC_DEV_HEATER], heater_auto, &heater_manual);
    cooler_target = ManualOverride_Resolve(&s_relay_manual[LOGIC_DEV_COOLER], cooler_auto, &cooler_manual);

    /* 执行继电器切换（含最小间隔保护） */
    SetRelayIfChanged(LOGIC_RELAY_HEATER, heater_target, heater_manual);
    SetRelayIfChanged(LOGIC_RELAY_COOLER, cooler_target, cooler_manual);
}

/**
 * @brief  湿度控制 —— 加湿器（带迟滞）
 *
 * 迟滞逻辑：
 *   - 加湿器已开: 湿度超过 (low + hyst) 才关闭
 *   - 加湿器已关: 湿度低于 low 才开启
 *
 * @param  humidity     当前湿度(%RH)
 * @param  humidity_low 加湿开启阈值
 * @note   除湿逻辑在 Logic_ControlFanLevel 中处理
 */
void Logic_ControlHumidity(float humidity, float humidity_low)
{
    uint8_t current = (s_last_relay_state[LOGIC_RELAY_HUMIDIFIER] == 1U) ? 1U : 0U;
    float hyst = s_runtime_thresholds.hyst_humi;
    uint8_t humidifier_auto;
    uint8_t humidifier_target;
    uint8_t humidifier_manual = 0U;

    /* 加湿器迟滞控制 */
    if (current)
    {
        humidifier_auto = (humidity > (humidity_low + hyst)) ? 0U : 1U;
    }
    else
    {
        humidifier_auto = (humidity < humidity_low) ? 1U : 0U;
    }

    taskENTER_CRITICAL();
    s_auto_target[LOGIC_DEV_HUMIDIFIER] = humidifier_auto;
    taskEXIT_CRITICAL();

    humidifier_target = ManualOverride_Resolve(&s_relay_manual[LOGIC_DEV_HUMIDIFIER], humidifier_auto, &humidifier_manual);
    SetRelayIfChanged(LOGIC_RELAY_HUMIDIFIER, humidifier_target, humidifier_manual);
}

/**
 * @brief  灯光控制（人体感应 + 光照判断，带迟滞）
 *
 * 开灯条件: 人体在有效距离内 OR 光照低于阈值
 * 迟滞逻辑: 灯已开时用 (light_low + hyst) 作为关灯阈值，灯已关时用 light_low
 *
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
                           uint16_t human_light_distance_max)
{
    uint8_t current = (s_last_relay_state[LOGIC_RELAY_LIGHT] == 1U) ? 1U : 0U;
    uint16_t hyst = s_runtime_thresholds.hyst_light;
    /* 关灯阈值 = 开灯阈值 + 迟滞量，防止溢出 */
    uint16_t light_off = (light_low <= (uint16_t)(65535U - hyst)) ? (uint16_t)(light_low + hyst) : 65535U;
    /* 人体感应触发: 有人且在有效距离内 */
    uint8_t human_trigger = ((human_presence != 0U) && (human_distance <= human_light_distance_max)) ? 1U : 0U;
    uint8_t light_auto;
    uint8_t light_target;
    uint8_t light_manual = 0U;

    /* 迟滞控制: 灯已开时用较高的关灯阈值，灯已关时用较低的开灯阈值 */
    if (current)
    {
        light_auto = (human_trigger || (light_lux < light_off)) ? 1U : 0U;
    }
    else
    {
        light_auto = (human_trigger || (light_lux < light_low)) ? 1U : 0U;
    }

    taskENTER_CRITICAL();
    s_auto_target[LOGIC_DEV_LIGHT] = light_auto;
    taskEXIT_CRITICAL();

    light_target = ManualOverride_Resolve(&s_relay_manual[LOGIC_DEV_LIGHT], light_auto, &light_manual);
    SetRelayIfChanged(LOGIC_RELAY_LIGHT, light_target, light_manual);
}

/**
 * @brief  报警控制（多条件延时报警）
 *
 * 三个报警源：
 *   1. 门持续打开超过指定时间 -> 触发报警
 *   2. 气体浓度持续超标超过指定时间 -> 触发报警
 *   3. PM2.5持续超标超过指定时间 -> 触发报警
 *   任一条件满足即触发报警（OR逻辑）
 *
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
                        uint16_t pm25_high)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t door_open_ticks = pdMS_TO_TICKS(door_open_alarm_ms);
    TickType_t gas_high_ticks = pdMS_TO_TICKS(gas_high_alarm_ms);
    TickType_t pm25_high_ticks = pdMS_TO_TICKS(pm25_high_alarm_ms);
    uint8_t door_timeout = 0U;   /* 门报警超时标志 */
    uint8_t gas_timeout = 0U;    /* 气体报警超时标志 */
    uint8_t pm25_timeout = 0U;   /* PM2.5报警超时标志 */
    uint8_t alarm_auto;
    uint8_t alarm_target;
    uint8_t alarm_manual = 0U;

    /* ---- 门报警延时跟踪 ---- */
    if (door_closed == 0U)
    {
        /* 门打开: 开始计时 */
        if (!s_door_open_tracking)
        {
            s_door_open_tracking = 1U;
            s_door_open_start_tick = now;
        }
        /* 检查是否超过报警延时 */
        if ((now - s_door_open_start_tick) >= door_open_ticks)
        {
            door_timeout = 1U;
        }
    }
    else
    {
        /* 门关闭: 复位跟踪器 */
        s_door_open_tracking = 0U;
    }

    /* ---- 气体超标延时跟踪（带迟滞） ---- */
    if (gas > gas_high)
    {
        /* 气体超标: 开始计时 */
        if (!s_gas_high_tracking)
        {
            s_gas_high_tracking = 1U;
            s_gas_high_start_tick = now;
        }
        /* 检查是否超过报警延时 */
        if ((now - s_gas_high_start_tick) >= gas_high_ticks)
        {
            gas_timeout = 1U;
        }
    }
    else if (gas <= ((gas_high > s_runtime_thresholds.hyst_gas) ?
               (uint16_t)(gas_high - s_runtime_thresholds.hyst_gas) : 0U))
    {
        /* 气体降到 (阈值 - 迟滞量) 以下才复位跟踪器，防止临界值附近反复触发 */
        s_gas_high_tracking = 0U;
    }

    /* ---- PM2.5超标延时跟踪（带迟滞） ---- */
    if (pm25 > pm25_high)
    {
        /* PM2.5超标: 开始计时 */
        if (!s_pm25_high_tracking)
        {
            s_pm25_high_tracking = 1U;
            s_pm25_high_start_tick = now;
        }
        /* 检查是否超过报警延时 */
        if ((now - s_pm25_high_start_tick) >= pm25_high_ticks)
        {
            pm25_timeout = 1U;
        }
    }
    else if (pm25 <= ((pm25_high > s_runtime_thresholds.hyst_pm25) ?
               (uint16_t)(pm25_high - s_runtime_thresholds.hyst_pm25) : 0U))
    {
        /* PM2.5降到 (阈值 - 迟滞量) 以下才复位跟踪器 */
        s_pm25_high_tracking = 0U;
    }

    /* 三个报警源OR逻辑: 任一超时即触发报警 */
    alarm_auto = ((door_timeout != 0U) ||
                  (gas_timeout != 0U) ||
                  (pm25_timeout != 0U)) ? 1U : 0U;

    taskENTER_CRITICAL();
    s_auto_target[LOGIC_DEV_ALARM] = alarm_auto;
    taskEXIT_CRITICAL();

    alarm_target = ManualOverride_Resolve(&s_relay_manual[LOGIC_DEV_ALARM], alarm_auto, &alarm_manual);
    SetRelayIfChanged(LOGIC_RELAY_ALARM, alarm_target, alarm_manual);
}

/**
 * @brief  第7组阈值修改：迟滞量及最小切换间隔
 * @param  threshold_id 参数ID (48~53)
 * @param  value        新值（最小切换间隔单位为秒）
 * @return 1=成功, 0=ID无效
 */
uint8_t Logic_SetChange7ThresholdById(uint8_t threshold_id, float value)
{
    if (value < 0.0f)
    {
        value = 0.0f;
    }

    taskENTER_CRITICAL();
    switch (threshold_id)
    {
    case LOGIC_CHANGE7_HYST_TEMP_ID:
        s_runtime_thresholds.hyst_temp = value;
        break;
    case LOGIC_CHANGE7_HYST_HUMI_ID:
        s_runtime_thresholds.hyst_humi = value;
        break;
    case LOGIC_CHANGE7_HYST_LIGHT_ID:
        s_runtime_thresholds.hyst_light = ClampToU16FromFloat(value);
        break;
    case LOGIC_CHANGE7_HYST_GAS_ID:
        s_runtime_thresholds.hyst_gas = ClampToU16FromFloat(value);
        break;
    case LOGIC_CHANGE7_HYST_PM25_ID:
        s_runtime_thresholds.hyst_pm25 = ClampToU16FromFloat(value);
        break;
    case LOGIC_CHANGE7_MIN_SWITCH_ID:
        s_runtime_thresholds.min_switch_ms = SecondsToMsClamped(value);
        break;
    default:
        taskEXIT_CRITICAL();
        return 0U;
    }
    taskEXIT_CRITICAL();

    RuntimeConfig_RequestSave();
    return 1U;
}

/**
 * @brief  获取第7组阈值当前值（迟滞量及最小切换间隔，间隔转换为秒返回）
 */
void Logic_GetChange7Thresholds(float *hyst_temp, float *hyst_humi,
                                float *hyst_light, float *hyst_gas,
                                float *hyst_pm25, float *min_switch_s)
{
    taskENTER_CRITICAL();
    if (hyst_temp != NULL)
    {
        *hyst_temp = s_runtime_thresholds.hyst_temp;
    }
    if (hyst_humi != NULL)
    {
        *hyst_humi = s_runtime_thresholds.hyst_humi;
    }
    if (hyst_light != NULL)
    {
        *hyst_light = (float)s_runtime_thresholds.hyst_light;
    }
    if (hyst_gas != NULL)
    {
        *hyst_gas = (float)s_runtime_thresholds.hyst_gas;
    }
    if (hyst_pm25 != NULL)
    {
        *hyst_pm25 = (float)s_runtime_thresholds.hyst_pm25;
    }
    if (min_switch_s != NULL)
    {
        *min_switch_s = (float)s_runtime_thresholds.min_switch_ms / 1000.0f;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  主控制函数，由FreeRTOS任务周期性调用
 *
 * 执行流程：
 *   1. 校验输入参数，无阈值时使用默认值
 *   2. 检查各传感器有效标志，无效时使用安全默认值
 *   3. 依次调用: 温度控制 -> 湿度控制 -> 灯光控制 -> 报警控制 -> 风扇控制
 *
 * @param  data       所有传感器数据汇聚结构体指针，不可为 NULL
 * @param  thresholds 阈值参数指针，为 NULL 时使用默认值
 * @note   即使传感器断开，仍会调用各控制函数以解析手动覆盖
 */
void Logic_Run(const AllSensorData_t *data, const LogicThresholds_t *thresholds)
{
    LogicThresholds_t default_thresholds;
    const LogicThresholds_t *cfg = thresholds;
    uint8_t has_temp_humi;
    uint8_t has_human;
    uint8_t has_light;
    uint8_t has_door;
    uint8_t has_gas;
    uint8_t has_pm;
    uint8_t human_presence;
    uint16_t human_distance;
    uint16_t light_lux;
    uint8_t door_closed;
    uint16_t gas;
    uint16_t pm25;

    if (data == NULL)
    {
        return;
    }

    if (cfg == NULL)
    {
        Logic_SetDefaultThresholds(&default_thresholds);
        cfg = &default_thresholds;
    }

    has_temp_humi = ((data->valid_mask & ALL_SENSOR_VALID_TEMP_HUMI) != 0U) ? 1U : 0U;
    has_human = ((data->valid_mask & ALL_SENSOR_VALID_HUMAN) != 0U) ? 1U : 0U;
    has_light = ((data->valid_mask & ALL_SENSOR_VALID_LIGHT) != 0U) ? 1U : 0U;
    has_door = ((data->valid_mask & ALL_SENSOR_VALID_DOOR) != 0U) ? 1U : 0U;
    has_gas = ((data->valid_mask & ALL_SENSOR_VALID_GAS) != 0U) ? 1U : 0U;
    has_pm = ((data->valid_mask & ALL_SENSOR_VALID_PM) != 0U) ? 1U : 0U;

    /*
     * 始终调用各控制函数，即使传感器断开也能解析手动覆盖。
     * 传感器无效时使用安全默认值：
     *   - 温度: 取上下限中值（不触发加热/制冷）
     *   - 湿度: 取中间值（不触发加湿）
     *   - 人体: 无人；光照: 最大值（不开灯）
     *   - 门: 关闭；气体/PM: 0（不报警）
     */
    Logic_ControlTemperature(has_temp_humi ? data->temperature : ((cfg->temp_low + cfg->temp_high) / 2.0f),
                             cfg->temp_low, cfg->temp_high);
    Logic_ControlHumidity(has_temp_humi ? data->humidity : ((cfg->humidity_low + 100.0f) / 2.0f),
                          cfg->humidity_low);

    human_presence = has_human ? data->human_presence : 0U;
    human_distance = has_human ? data->human_distance : 65535U;
    light_lux = has_light ? data->light_lux : 65535U;
    Logic_ControlLighting(human_presence,
                          human_distance,
                          light_lux,
                          cfg->light_low,
                          cfg->human_light_distance_max);

    door_closed = has_door ? data->door_closed : 1U;
    gas = has_gas ? data->gas : 0U;
    pm25 = has_pm ? data->pm25 : 0U;
    Logic_ControlAlarm(door_closed,
                       gas,
                       pm25,
                       cfg->door_open_alarm_ms,
                       cfg->gas_high_alarm_ms,
                       cfg->pm25_high_alarm_ms,
                       cfg->gas_high,
                       cfg->pm25_high);

    Logic_ControlFanLevel(data, cfg);
}
