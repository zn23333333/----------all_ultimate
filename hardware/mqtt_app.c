/**
 * @file  mqtt_app.c
 * @brief MQTT 应用层实现 —— WiFi/MQTT 连接管理、命令分发、
 *        周期遥测/状态/阈值发布、ACK 队列管理、UDP 发现协议。
 *
 * 本文件是环境保障系统实现了 STM32 网关与云端之间的全部 MQTT 通信逻辑。
 *
 * 主要功能模块:
 *   1. WiFi/MQTT 连接管理 —— ESP01S 初始化、TCP 连接、MQTT 握手、
 *      自动重连（指数退避）、PING 心跳保活、链路超时检测
 *   2. MQTT 命令分发 —— 解析 cmd/threshold（阈值修改）、cmd/manual
 *      （手动控制设备）、cmd/query（查询传感器/状态/阈值）、
 *      cmd/relay（继电器直控）、cmd/fan（风扇 PWM）
 *   3. 周期遥测发布 —— 传感器数据 3 秒、设备状态 3 秒、阈值参数 10 秒
 *   4. ACK 确认队列 —— 延迟发送（避免阻塞 poll 循环）、失败重试、
 *      请求去重缓存（防止重复执行同一命令）
 *   5. UDP 发现协议 —— 响应 gateway_discover 广播，回复 gateway_announce
 *   6. 设备动作事件上报 —— tele/action，记录设备开关/档位变化
 *   7. 时间同步 —— 从 MQTT 接收 NTP 时间并转发给串口屏显示
 */

/* ======================== 头文件包含 ======================== */
#include "mqtt_app.h"           /* 本模块头文件 */
#include "json_parse.h"         /* JSON 轻量解析工具 */

#include "stm32f4xx.h"          /* STM32F4 外设库 */
#include "FreeRTOS.h"           /* FreeRTOS 内核 */
#include "task.h"               /* FreeRTOS 任务管理 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "Modbus_Relay.h"       /* Modbus 继电器驱动 */
#include "all_sensor_data.h"    /* 传感器数据汇总 */
#include "logic.h"              /* 业务逻辑层（阈值、手动模式等） */
#include "motor_fan.h"          /* 风扇 PWM 驱动 */
#include "esp01s.h"             /* ESP01S WiFi 模块底层驱动 */
#include "screen.h"             /* 串口屏通信 */
#include "runtime_config.h"     /* 运行时配置 */

/* ================================================================
 *  日志宏定义
 *  - ESP_LOG:  ESP01S 底层通信日志（默认关闭，调试时开启）
 *  - PERF_LOG: 性能/连接状态日志（默认开启，用于监控连接与循环耗时）
 *  - CMD_LOG:  命令处理日志（默认开启，记录命令收发与 ACK 状态）
 * ================================================================ */
#define ESP_LOG_ENABLE               0U   /* ESP 底层日志开关: 0=关闭, 1=开启 */
#if ESP_LOG_ENABLE
#define ESP_LOG(...) printf(__VA_ARGS__)
#else
#define ESP_LOG(...) do { } while (0)
#endif

#define PERF_LOG_ENABLE              0U   /* 性能日志开关: 0=关闭, 1=开启 */
#if PERF_LOG_ENABLE
#define PERF_LOG(...) printf(__VA_ARGS__)
#else
#define PERF_LOG(...) do { } while (0)
#endif

#define CMD_LOG_ENABLE               1U   /* 命令日志开关: 0=关闭, 1=开启 */
#if CMD_LOG_ENABLE
#define CMD_LOG(...) printf(__VA_ARGS__)
#else
#define CMD_LOG(...) do { } while (0)
#endif

/* ================================================================
 *  MQTT / WiFi 配置常量
 * ================================================================ */

/* ---- WiFi 连接参数 ---- */
#define ESP01S_WIFI_SSID             "ieeee"            /* WiFi 热点名称 */
#define ESP01S_WIFI_PASSWORD         "1234567899"       /* WiFi 密码 */
#define ESP01S_LOCAL_PORT            4001U              /* 本地 UDP 监听端口（用于设备发现） */
#define ESP01S_GATEWAY_ID            "stm32_gw"         /* 网关设备 ID（UDP 发现协议中使用） */

/* ---- MQTT 连接参数 ---- */
#define ESP01S_MQTT_ENABLE           1U                 /* MQTT 功能总开关 */
#define ESP01S_MQTT_LINK_ID          1U                 /* ESP01S TCP 链路 ID（link_id=1 专用于 MQTT） */
#define ESP01S_MQTT_BROKER_IP        "8.212.157.225"    /* MQTT Broker IP 地址 */
#define ESP01S_MQTT_BROKER_PORT      1883U              /* MQTT Broker 端口 */
#define ESP01S_MQTT_CLIENT_ID        "gw001"            /* MQTT 客户端 ID */

/* ---- MQTT 主题定义 ---- */
#define ESP01S_MQTT_SUB_TOPIC        "ws/gw001/cmd/#"             /* 订阅: 所有命令主题（通配符） */
#define ESP01S_MQTT_TIME_TOPIC       "ws/gw001/tele/time"         /* 订阅: 时间同步主题 */
#define ESP01S_MQTT_PUB_TOPIC        "ws/gw001/tele/sensors"      /* 发布: 传感器遥测数据 */
#define ESP01S_MQTT_ACK_TOPIC        "ws/gw001/tele/ack"          /* 发布: 命令确认响应 */
#define ESP01S_MQTT_ACTION_TOPIC     "ws/gw001/tele/action"       /* 发布: 设备动作事件 */
#define ESP01S_MQTT_CMD_THRESHOLD    "ws/gw001/cmd/threshold"     /* 订阅: 阈值修改命令 */
#define ESP01S_MQTT_CMD_MANUAL       "ws/gw001/cmd/manual"        /* 订阅: 手动控制命令 */
#define ESP01S_MQTT_CMD_QUERY        "ws/gw001/cmd/query"         /* 订阅: 查询命令 */
#define ESP01S_MQTT_TELE_STATUS      "ws/gw001/tele/status"       /* 发布: 设备状态 */
#define ESP01S_MQTT_TELE_THRESHOLDS  "ws/gw001/tele/thresholds"   /* 发布: 全部阈值（扁平格式） */
#define ESP01S_MQTT_TELE_CHANGE1     "ws/gw001/tele/thresholds/change1"  /* 发布: 第1组阈值（温湿度） */
#define ESP01S_MQTT_TELE_CHANGE2     "ws/gw001/tele/thresholds/change2"  /* 发布: 第2组阈值（PM2.5/燃气报警） */
#define ESP01S_MQTT_TELE_CHANGE3     "ws/gw001/tele/thresholds/change3"  /* 发布: 第3组阈值（门/灯/人体） */
#define ESP01S_MQTT_TELE_CHANGE4     "ws/gw001/tele/thresholds/change4"  /* 发布: 第4组阈值（风扇档位） */
#define ESP01S_MQTT_TELE_CHANGE5     "ws/gw001/tele/thresholds/change5"  /* 发布: 第5组阈值（PM2.5分档） */
#define ESP01S_MQTT_TELE_CHANGE6     "ws/gw001/tele/thresholds/change6"  /* 发布: 第6组阈值（燃气分档） */
#define ESP01S_MQTT_TELE_CHANGE7     "ws/gw001/tele/thresholds/change7"  /* 发布: 第7组阈值（近回差/最小切换） */
/* ---- 定时器与超时参数 ---- */
#define ESP01S_MQTT_PING_INTERVAL_MS 30000U   /* MQTT PING 心跳间隔 (ms) */
#define ESP01S_MQTT_PUB_INTERVAL_MS  3000U    /* 传感器遥测发布间隔 (ms) */
#define ESP01S_MQTT_STATUS_INTERVAL_MS 3000U  /* 设备状态发布间隔 (ms) */
#define ESP01S_MQTT_PUB_GAP_MS      400U     /* 任意两次发布之间的最小间隔 (ms) */
#define ESP01S_LINK_CHECK_INTERVAL_MS 10000U  /* WiFi 链路状态检查间隔 (ms) */
#define ESP01S_WIFI_FAIL_REINIT_COUNT 3U      /* WiFi 检查连续失败次数阈值，超过则重新初始化 */
#define ESP01S_LINK_STALE_REINIT_MS  20000U   /* ESP 无响应超时，超过则强制重新初始化 (ms) */
#define ESP01S_PING_IDLE_MIN_MS      8000U    /* PING 前要求的最小空闲时间，避免频繁 PING (ms) */

/* ---- 应用层定时参数 ---- */
#define APP_RELAY_STATUS_POLL_INTERVAL_MS 1000U  /* 继电器状态轮询间隔 (ms) */
#define APP_THRESHOLDS_PUB_INTERVAL_S 10U        /* 阈值参数发布间隔 (秒) */
#define APP_DISCOVER_REPLY_MIN_GAP_MS 800U       /* UDP 发现回复最小间隔，防止重复回复 (ms) */
#define APP_CMD_QUIET_PUB_GAP_MS     1500U       /* 收到命令后的静默期，暂停遥测发布 (ms) */
#define APP_CMD_PUBLISH_BYPASS_MS    3500U        /* s_mqtt_ready==0时，ack在此时间内可重试发送 (ms) */
#define APP_QUERY_MIN_GAP_MS         2000U        /* 查询命令最小间隔，防止频繁查询冲击 (ms) */
#define MQTT_TASK_STACK_WORDS        1024U        /* MQTT 任务栈大小 (words) */
#define APP_MQTT_PUB_FAIL_TOLERANCE  10U          /* 发布失败容忍次数，超过则强制重连 */

/* ---- 设备动作事件参数 ---- */
#define APP_ACTION_DEDUP_WINDOW_MS   5000U        /* 动作事件去重窗口 (ms)，同一设备相同动作不重复上报 */
#define APP_ACTION_PUB_MIN_GAP_MS    700U         /* 动作事件发布最小间隔 (ms) */
#define APP_ENABLE_ACTION_TELEMETRY  1U           /* 动作遥测编译开关: 0=禁用, 1=启用 */

/* ---- ACK 缓存与延迟队列参数 ---- */
#define APP_REQ_ACK_CACHE_SIZE       16U          /* 请求去重缓存大小（最多缓存16条已处理的请求） */
#define APP_REQ_ACK_CACHE_KEEP_MS    120000U      /* 缓存条目保留时间 (ms)，过2分钟自动过期 */
#define APP_ACK_JSON_SIZE            240U         /* 单条 ACK JSON 缓冲区大小 (bytes) */
#define APP_ACK_DEFERRED_QUEUE_SIZE  40U          /* 延迟 ACK 队列大小（最多缓剀40条待发 ACK） */
#define APP_ACK_RETRY_BASE_MS        30U          /* ACK 重试基础延迟 (ms) */
#define APP_ACK_RETRY_MAX_MS         200U         /* ACK 重试最大延迟 (ms) */

/* ================================================================
 *  模块级静态变量
 * ================================================================ */

/* ---- 核心运行状态标志 ---- */
static volatile uint8_t s_esp_runtime_ready = 0U;    /* ESP01S 运行时已初始化标志（WiFi已连接） */
static volatile uint8_t s_mqtt_ready = 0U;            /* MQTT 连接已建立标志 */
static volatile uint8_t s_ack_defer_only = 0U;        /* ACK 仅延迟模式标志（poll循环内置1，避免阻塞） */
static volatile uint32_t s_mqtt_seq = 0U;             /* MQTT 消息序列号计数器（单调递增） */
static volatile uint8_t s_mqtt_pub_fail_count = 0U;   /* MQTT 发布连续失败计数（超阈值则重连） */
static volatile TickType_t s_esp_last_alive_tick = 0U; /* ESP01S 最后活跃时间戳（用于链路超时检测） */
static volatile TickType_t s_last_cmd_rx_tick = 0U;   /* 最后收到命令的时间戳（用于静默期控制） */

/* ---- 设备动作事件状态（条件编译） ---- */
#if (APP_ENABLE_ACTION_TELEMETRY != 0U)
static uint8_t s_action_pending_valid = 0U;           /* 是否有待重试的动作事件 */
static ScreenActionEvent_t s_action_pending;          /* 待重试的动作事件缓存 */
#endif

/* ---- 继电器链路状态缓存 ---- */
static uint8_t s_relay_link_cached = 0U;              /* 继电器 Modbus 链路状态缓存: 1=在线, 0=断开 */
static TickType_t s_relay_link_last_poll = 0U;        /* 继电器链路上次轮询时间戳 */
static uint8_t s_relay_link_fail_count = 0U;           /* 继电器轮询连续失败计数 */

/* ---- UDP 发现协议去重 ---- */
static TickType_t s_last_discover_reply_tick = 0U;    /* 上次发现回复时间戳 */
static char s_last_discover_reply_ip[20] = {0};       /* 上次发现回复的目标 IP */
static uint16_t s_last_discover_reply_port = 0U;      /* 上次发现回复的目标端口 */

/* ---- 当前命令的 req_id（用于 ACK 关联） ---- */
static uint8_t s_ack_req_id_valid = 0U;               /* 当前 req_id 是否有效 */
static char s_ack_req_id[40] = {0};                   /* 当前命令的请求 ID */

/* ---- 动作事件去重与重试状态（条件编译） ---- */
#if (APP_ENABLE_ACTION_TELEMETRY != 0U)
static uint8_t s_last_action_pub_valid = 0U;          /* 上次动作发布是否有效（用于去重判断） */
static ScreenActionDevice_t s_last_action_pub_device = SCREEN_ACTION_DEV_HEATER; /* 上次发布的设备类型 */
static uint8_t s_last_action_pub_manual = 0U;         /* 上次发布是否手动模式 */
static uint8_t s_last_action_pub_value = 0U;          /* 上次发布的动作值 */
static TickType_t s_last_action_pub_tick = 0U;        /* 上次动作发布时间戳（去重用） */
static TickType_t s_last_action_tx_tick = 0U;         /* 上次动作尝试（无论失败与否）发送时间戳（最小间隔控制） */

/* 待重试动作事件的 JSON 缓存，避免重试时序列号变化导致重复 */
static char    s_action_pending_json[256];             /* 缓存的动作 JSON 字符串 */
static uint8_t s_action_pending_json_valid = 0U;      /* 缓存 JSON 是否有效 */
#endif

static TickType_t s_last_query_tick = 0U;             /* 上次查询命令时间戳（用于频率限制） */

/* ---- 延迟 ACK 队列（环形缓冲区） ---- */
typedef struct
{
    uint8_t valid;                          /* 条目是否有效 */
    uint8_t retry_count;                    /* 已重试次数 */
    TickType_t next_tick;                   /* 下次允许发送的时间戳 */
    char ack_json[APP_ACK_JSON_SIZE];       /* 待发送的 ACK JSON 字符串 */
} AppDeferredAckItem_t;

static AppDeferredAckItem_t s_ack_deferred_queue[APP_ACK_DEFERRED_QUEUE_SIZE]; /* 延迟 ACK 环形队列 */
static uint8_t s_ack_deferred_head = 0U;   /* 队列头指针（出队位置） */
static uint8_t s_ack_deferred_tail = 0U;   /* 队列尾指针（入队位置） */
static uint8_t s_ack_deferred_count = 0U;  /* 队列当前元素数量 */

/* ---- 请求 ACK 去重缓存（防止重复执行命令） ---- */
typedef struct
{
    uint8_t valid;                          /* 条目是否有效 */
    TickType_t tick;                        /* 缓存时间戳（用于过期淘汰） */
    char req_id[40];                        /* 请求 ID */
    char ack_json[APP_ACK_JSON_SIZE];       /* 已发送的 ACK JSON（重复请求时直接回放） */
} AppReqAckCacheItem_t;

static AppReqAckCacheItem_t s_req_ack_cache[APP_REQ_ACK_CACHE_SIZE]; /* 请求 ACK 缓存数组 */
static uint8_t s_req_ack_cache_next = 0U;  /* 下一个可用的缓存槽位索引 */

/* 前向声明 */
static uint8_t App_MqttPublishRaw(const char *topic, const char *json_payload);

/* ================================================================
 *  小型工具函数
 * ================================================================ */

/**
 * @brief 获取当前系统时间（毫秒）
 * @return 当前 tick 转换后的毫秒值，避免中间计算溢出
 */
static uint32_t App_GetMsNow(void)
{
    /* 避免溢出: 先除再乘，余数部分单独处理
       tick / rate * 1000 + tick % rate * 1000 / rate */
    TickType_t tick = xTaskGetTickCount();
    return (uint32_t)((tick / configTICK_RATE_HZ) * 1000U
                    + (tick % configTICK_RATE_HZ) * 1000U / configTICK_RATE_HZ);
}

/**
 * @brief 获取重连延迟时间（粗略随机化）
 * @return 延迟毫秒数 (500~1500ms)，避免多设备同时重连
 */
static uint32_t App_GetReconnectDelayMs(void)
{
    return (uint32_t)((xTaskGetTickCount() % 3U) + 1U) * 500U;
}

/**
 * @brief 标记 ESP01S 活跃（更新最后活跃时间戳）
 */
static void App_EspMarkAlive(void)
{
    taskENTER_CRITICAL();
    s_esp_last_alive_tick = xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

/**
 * @brief 获取 ESP01S 沉默时间（距离上次活跃的毫秒数）
 * @return 沉默时间 (ms)，用于判断链路是否超时
 */
static uint32_t App_EspSilentMs(void)
{
    TickType_t now;
    TickType_t last;

    now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    last = s_esp_last_alive_tick;
    taskEXIT_CRITICAL();

    return (uint32_t)((now - last) * 1000U / configTICK_RATE_HZ);
}

/**
 * @brief 获取下一个 MQTT 消息序列号（原子操作）
 * @return 当前序列号，调用后自动递增
 */
static uint32_t App_MqttNextSeq(void)
{
    uint32_t seq;

    taskENTER_CRITICAL();
    seq = s_mqtt_seq++;
    taskEXIT_CRITICAL();

    return seq;
}

/**
 * @brief 获取缓存的继电器链路状态（线程安全）
 * @return 1=在线, 0=断开
 */
uint8_t MqttApp_GetRelayLinkCached(void)
{
    uint8_t cached;

    taskENTER_CRITICAL();
    cached = s_relay_link_cached;
    taskEXIT_CRITICAL();

    return cached;
}

/**
 * @brief 将毫秒转换为 tick 数，保证至少返回 1 tick
 * @param ms 毫秒数
 * @return 对应的 tick 数（最小为1）
 */
static TickType_t SafeMsToTicks(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    return (t != 0U) ? t : 1U;
}

/**
 * @brief 判断是否应当强制重连 MQTT（基于错误码或失败次数）
 * @param err_code ESP01S 错误码（如9=链路无效）
 * @param fail_count 连续失败次数
 * @return 1=应当重连, 0=无需
 */
static uint8_t App_ShouldForceReconnectOnPublishError(uint16_t err_code,
                                                      uint8_t fail_count)
{
    if (err_code == 9U) return 1U;        /* 错误码9: TCP 链路无效，必须重连 */
    if (err_code == 4U) return (fail_count >= 3U) ? 1U : 0U; /* CIPSEND prompt timeout */
    return (fail_count >= APP_MQTT_PUB_FAIL_TOLERANCE) ? 1U : 0U;
}

/**
 * @brief 判断是否应跳过查询命令的发布（频率限制）
 * @return 1=跳过(过于频繁), 0=允许发布
 */
static uint8_t App_ShouldSkipQueryPublish(void)
{
    TickType_t now_tick = xTaskGetTickCount();
    TickType_t gap_ticks = SafeMsToTicks(APP_QUERY_MIN_GAP_MS);
    if ((s_last_query_tick != 0U) && ((now_tick - s_last_query_tick) < gap_ticks))
        return 1U;
    s_last_query_tick = now_tick;
    return 0U;
}

/* ================================================================
 *  ACK 延迟队列 + 请求去重缓存 函数实现
 * ================================================================ */

/**
 * @brief 获取 ACK 缓存保留时间对应的 tick 数
 * @return 保留时间的 tick 表示
 */
static TickType_t App_ReqAckKeepTicks(void)
{
    return SafeMsToTicks(APP_REQ_ACK_CACHE_KEEP_MS);
}

/**
 * @brief 在 ACK 缓存中查找指定 req_id 的索引
 *
 * 遍历缓存数组，过期条目自动淘汰。找到匹配的 req_id 则返回索引。
 *
 * @param req_id  要查找的请求 ID 字符串
 * @return 找到时返回索引 (0~15)，未找到返回 -1
 */
static int16_t App_FindReqAckCacheIndex(const char *req_id)
{
    uint8_t i;
    TickType_t now_tick = 0U;
    TickType_t keep_ticks = 0U;

    if ((req_id == NULL) || (req_id[0] == '\0'))
    {
        return -1;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        now_tick = xTaskGetTickCount();
        keep_ticks = App_ReqAckKeepTicks();
    }

    for (i = 0U; i < APP_REQ_ACK_CACHE_SIZE; i++)
    {
        if (s_req_ack_cache[i].valid == 0U)
        {
            continue;
        }

        if ((keep_ticks != 0U) && ((now_tick - s_req_ack_cache[i].tick) > keep_ticks))
        {
            s_req_ack_cache[i].valid = 0U;
            continue;
        }

        if (strcmp(s_req_ack_cache[i].req_id, req_id) == 0)
        {
            return (int16_t)i;
        }
    }

    return -1;
}

/**
 * @brief 将已处理的请求 ID 及其 ACK JSON 存入缓存
 *
 * 如果 req_id 已存在则更新，否则占用下一个槽位（环形覆盖）。
 *
 * @param req_id   请求 ID 字符串
 * @param ack_json 对应的 ACK JSON 字符串
 */
static void App_RememberReqAck(const char *req_id, const char *ack_json)
{
    int16_t idx;
    uint8_t slot;

    if ((req_id == NULL) || (req_id[0] == '\0') || (ack_json == NULL) || (ack_json[0] == '\0'))
    {
        return;
    }

    idx = App_FindReqAckCacheIndex(req_id);
    if (idx >= 0)
    {
        slot = (uint8_t)idx;
    }
    else
    {
        slot = s_req_ack_cache_next;
        s_req_ack_cache_next = (uint8_t)((s_req_ack_cache_next + 1U) % APP_REQ_ACK_CACHE_SIZE);
    }

    s_req_ack_cache[slot].valid = 1U;
    s_req_ack_cache[slot].tick = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
                                 ? xTaskGetTickCount()
                                 : 0U;
    strncpy(s_req_ack_cache[slot].req_id, req_id, sizeof(s_req_ack_cache[slot].req_id) - 1U);
    s_req_ack_cache[slot].req_id[sizeof(s_req_ack_cache[slot].req_id) - 1U] = '\0';
    strncpy(s_req_ack_cache[slot].ack_json, ack_json, sizeof(s_req_ack_cache[slot].ack_json) - 1U);
    s_req_ack_cache[slot].ack_json[sizeof(s_req_ack_cache[slot].ack_json) - 1U] = '\0';
}

/**
 * @brief 清空延迟 ACK 队列（重连时调用）
 */
static void App_ClearDeferredAckQueue(void)
{
    memset(s_ack_deferred_queue, 0, sizeof(s_ack_deferred_queue));
    s_ack_deferred_head = 0U;
    s_ack_deferred_tail = 0U;
    s_ack_deferred_count = 0U;
}

/**
 * @brief 将一条 ACK JSON 加入延迟发送队列
 *
 * 队列满时丢弃最旧的一条。新条目插入队尾，等待主循环 flush。
 *
 * @param ack_json 待延迟发送的 ACK JSON 字符串
 */
static void App_SaveDeferredAck(const char *ack_json)
{
    AppDeferredAckItem_t *slot;
    TickType_t now_tick = 0U;

    if ((ack_json == NULL) || (ack_json[0] == '\0'))
    {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        now_tick = xTaskGetTickCount();
    }

    if (s_ack_deferred_count >= APP_ACK_DEFERRED_QUEUE_SIZE)
    {
        s_ack_deferred_head = (uint8_t)((s_ack_deferred_head + 1U) % APP_ACK_DEFERRED_QUEUE_SIZE);
        s_ack_deferred_count--;
        CMD_LOG("[CMD][ACK][DEFERRED_DROP_OLDEST]\r\n");
    }

    slot = &s_ack_deferred_queue[s_ack_deferred_tail];
    memset(slot, 0, sizeof(*slot));
    strncpy(slot->ack_json, ack_json, sizeof(slot->ack_json) - 1U);
    slot->ack_json[sizeof(slot->ack_json) - 1U] = '\0';
    slot->valid = 1U;
    slot->retry_count = 0U;
    slot->next_tick = now_tick;

    s_ack_deferred_tail = (uint8_t)((s_ack_deferred_tail + 1U) % APP_ACK_DEFERRED_QUEUE_SIZE);
    s_ack_deferred_count++;
}

/**
 * @brief 检查请求是否为重复请求，若是则回放缓存的 ACK
 *
 * 用于幂等性保证: 同一 req_id 的命令不会被重复执行，
 * 而是直接返回上次的 ACK 结果。
 *
 * @param req_id 请求 ID
 * @return 1=是重复请求已回放, 0=首次请求
 */
static uint8_t App_ReplayCachedAckIfDuplicate(const char *req_id)
{
    int16_t idx;
    AppReqAckCacheItem_t *item;

    idx = App_FindReqAckCacheIndex(req_id);
    if (idx < 0)
    {
        return 0U;
    }

    item = &s_req_ack_cache[(uint8_t)idx];
    if ((item->ack_json[0] == '\0') || (item->valid == 0U))
    {
        return 0U;
    }

    CMD_LOG("[CMD][DUP][REQ] req=%s replay cached ACK\r\n", req_id);
    if (!App_MqttPublishRaw(ESP01S_MQTT_ACK_TOPIC, item->ack_json))
    {
        App_SaveDeferredAck(item->ack_json);
    }

    item->tick = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
                 ? xTaskGetTickCount()
                 : item->tick;
    return 1U;
}

/**
 * @brief 尝试发送队列头部的延迟 ACK
 *
 * 每次调用最多发送一条。跳过无效条目，发送失败则增加重试计数。
 *
 * @return 1=成功发送了一条, 0=无可发或发送失败
 */
static uint8_t App_TryFlushDeferredAck(void)
{
    AppDeferredAckItem_t *item;

    if (s_ack_deferred_count == 0U)
    {
        return 0U;
    }
    if ((s_esp_runtime_ready == 0U) || (s_mqtt_ready == 0U))
    {
        return 0U;
    }
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return 0U;
    }

    /* Skip invalid items at head */
    while (s_ack_deferred_count > 0U)
    {
        item = &s_ack_deferred_queue[s_ack_deferred_head];
        if ((item->valid != 0U) && (item->ack_json[0] != '\0'))
        {
            break;
        }
        memset(item, 0, sizeof(*item));
        s_ack_deferred_head = (uint8_t)((s_ack_deferred_head + 1U) % APP_ACK_DEFERRED_QUEUE_SIZE);
        s_ack_deferred_count--;
    }

    if (s_ack_deferred_count == 0U)
    {
        return 0U;
    }

    /* No next_tick gating — caller controls inter-publish timing */
    item = &s_ack_deferred_queue[s_ack_deferred_head];
    if (App_MqttPublishRaw(ESP01S_MQTT_ACK_TOPIC, item->ack_json))
    {
        CMD_LOG("[CMD][ACK][DEFERRED_OK] retry=%u pending=%u\r\n",
                (unsigned int)(item->retry_count + 1U),
                (unsigned int)s_ack_deferred_count);
        memset(item, 0, sizeof(*item));
        s_ack_deferred_head = (uint8_t)((s_ack_deferred_head + 1U) % APP_ACK_DEFERRED_QUEUE_SIZE);
        s_ack_deferred_count--;
        return 1U;
    }

    if (item->retry_count < 0xFFU)
    {
        item->retry_count++;
    }
    CMD_LOG("[CMD][ACK][DEFERRED_RETRY] retry=%u pending=%u err=%u(%s)\r\n",
            (unsigned int)item->retry_count,
            (unsigned int)s_ack_deferred_count,
            (unsigned int)ESP01S_GetLastErrorCode(),
            ESP01S_GetLastErrorText());
    return 0U;
}

/* ================================================================
 *  继电器链路轮询（公共接口 —— 由 ControlTask 调用）
 * ================================================================ */

/**
 * @brief 轮询继电器 Modbus 链路状态
 *
 * 每隔 APP_RELAY_STATUS_POLL_INTERVAL_MS 读取一次继电器状态。
 * 读取成功则置位链路标志并清零失败计数；
 * 连续失败 3 次则标记链路断开。
 */
void MqttApp_PollRelayLink(void)
{
    TickType_t now_tick;
    TickType_t interval_tick;
    uint8_t relay_states[RELAY_CHANNEL_COUNT] = {0U};
    uint8_t relay_ok;

    now_tick = xTaskGetTickCount();
    interval_tick = SafeMsToTicks(APP_RELAY_STATUS_POLL_INTERVAL_MS);

    if ((s_relay_link_last_poll == 0U) || ((now_tick - s_relay_link_last_poll) >= interval_tick))
    {
        relay_ok = ReadRelayStatus(relay_states, RELAY_CHANNEL_COUNT);
        taskENTER_CRITICAL();
        if (relay_ok != 0U)
        {
            s_relay_link_cached = 1U;
            s_relay_link_fail_count = 0U;
        }
        else
        {
            if (s_relay_link_fail_count < 0xFFU)
            {
                s_relay_link_fail_count++;
            }
            if (s_relay_link_fail_count >= 3U)
            {
                s_relay_link_cached = 0U;
            }
        }
        taskEXIT_CRITICAL();
        s_relay_link_last_poll = now_tick;
    }
}

/**
 * @brief 获取 MQTT Broker 当前连接状态
 *
 * s_mqtt_ready 表示 MQTT CONNECT / SUBSCRIBE 已完成，且链路仍被认为有效。
 * 供屏幕等上层模块显示 EMQX/Broker 在线状态。
 */
uint8_t MqttApp_IsBrokerConnected(void)
{
    uint8_t ready;

    taskENTER_CRITICAL();
    ready = s_mqtt_ready;
    taskEXIT_CRITICAL();

    return (ready != 0U) ? 1U : 0U;
}

/* ================================================================
 *  MQTT 原始发布函数
 * ================================================================ */

/**
 * @brief 底层 MQTT 发布函数（所有发布的统一入口）
 *
 * 处理逻辑:
 * 1. 检查 MQTT 就绪状态（ACK 主题有旁路机制可在未就绪时短暂发送）
 * 2. 调用 ESP01S_MqttPublish 发送数据
 * 3. 失败时累加失败计数，超过容忍度则强制重连
 * 4. 成功时清零失败计数并更新活跃时间戳
 *
 * @param topic        MQTT 主题字符串
 * @param json_payload JSON 负载字符串
 * @return 1=发布成功, 0=发布失败
 */
static uint8_t App_MqttPublishRaw(const char *topic, const char *json_payload)
{
    uint16_t len;
    uint16_t last_err;

    if (s_mqtt_ready == 0U)
    {
        /* MQTT 未就绪时，ACK 主题有旁路机制:
           如果刚收到命令不久，允许尝试发送 ACK */
        if (strcmp(topic, ESP01S_MQTT_ACK_TOPIC) == 0)
        {
            TickType_t now_tick = xTaskGetTickCount();
            TickType_t bypass_ticks = SafeMsToTicks(APP_CMD_PUBLISH_BYPASS_MS);
            TickType_t last_cmd;
            taskENTER_CRITICAL();
            last_cmd = s_last_cmd_rx_tick;
            taskEXIT_CRITICAL();
            if ((last_cmd == 0U) || ((now_tick - last_cmd) > bypass_ticks))
            {
                return 0U;
            }
        }
        else
        {
            return 0U;
        }
    }

    len = (uint16_t)strlen(json_payload);
    if (len == 0U) return 0U;

    /* 调用 ESP01S 底层发布接口 */
    if (!ESP01S_MqttPublish(ESP01S_MQTT_LINK_ID, topic,
                            (const uint8_t *)json_payload, len))
                            {
        last_err = ESP01S_GetLastErrorCode();
        /* ACK 发布以前从不触发重连，但如果 TCP 链路已死（错误 9 =
           链路无效）则必须重连，否则延迟 ACK 永远重试在死链路上，
           且周期遥测被非空延迟队列阻塞，造成永久死锁 */
        if (strcmp(topic, ESP01S_MQTT_ACK_TOPIC) != 0)
        {
            /* 非 ACK 发布错误处理: 累加失败计数 */
            /*
               NOTE:
               err=4 (CIPSEND prompt timeout) may indicate the ESP AT state machine
               is stuck. Count it so we can force a reconnect after a few repeats.
            */
            if ((last_err != 8U) && (last_err != 3U))
            {
                if (s_mqtt_pub_fail_count < 0xFFU) s_mqtt_pub_fail_count++;
            }
            if (App_ShouldForceReconnectOnPublishError(last_err, s_mqtt_pub_fail_count) != 0U)
                s_mqtt_ready = 0U;
        }
        else
        {
            /* ACK 发布: 仅在致命链路错误时强制重连 */
            if (last_err == 9U)
            {
                CMD_LOG("[CMD][ACK][LINK_DEAD] err=9, force reconnect\r\n");
                s_mqtt_ready = 0U;
            }
        }
        return 0U;
    }

    s_mqtt_pub_fail_count = 0U;     /* 发布成功，清零失败计数 */
    ESP01S_SetConnected(1U);         /* 标记 WiFi 已连接 */
    App_EspMarkAlive();              /* 更新活跃时间戳 */
    return 1U;
}

/* ================================================================
 *  设备控制辅助函数
 * ================================================================ */

/**
 * @brief 设置继电器通道开关
 * @param ch 通道号 (1~4)
 * @param on 1=合闸, 0=断开
 * @return 1=成功, 0=参数错误或通信失败
 */
static uint8_t Relay_Set(uint8_t ch, uint8_t on)
{
    if ((ch < 1U) || (ch > 4U))
    {
        return 0U;
    }
    return WriteRelay((uint8_t)(ch - 1U), (on != 0U) ? 1U : 0U);
}

/**
 * @brief 设置风扇 PWM 占空比
 * @param pwm 占空比百分比 (0~100)
 * @return 1=成功, 0=参数超范围
 */
static uint8_t Fan_SetPWM(uint8_t pwm)
{
    if (pwm > 100U)
    {
        return 0U;
    }
    Motor_SetDutyPercent(pwm);
    return 1U;
}

/* ================================================================
 *  手动控制设备分发
 *  将字符串设备名映射到枚举值，用于 cmd/manual 命令解析
 * ================================================================ */

/** 手动控制设备枚举（值与 Logic 层设备索引一一对应，INVALID=0） */
typedef enum
{
    APP_MANUAL_DEV_INVALID = 0,
    APP_MANUAL_DEV_HEATER,
    APP_MANUAL_DEV_COOLER,
    APP_MANUAL_DEV_HUMIDIFIER,
    APP_MANUAL_DEV_DEHUMIDIFIER,
    APP_MANUAL_DEV_ALARM,
    APP_MANUAL_DEV_LIGHT,
    APP_MANUAL_DEV_FAN
} AppManualDevice_t;

/** 设备名称字符串 → 枚举值映射表 */
static const struct { const char *name; AppManualDevice_t dev; } s_manual_dev_map[] = {
    {"heater",       APP_MANUAL_DEV_HEATER},
    {"cooler",       APP_MANUAL_DEV_COOLER},
    {"humidifier",   APP_MANUAL_DEV_HUMIDIFIER},
    {"dehumidifier", APP_MANUAL_DEV_DEHUMIDIFIER},
    {"alarm",        APP_MANUAL_DEV_ALARM},
    {"light",        APP_MANUAL_DEV_LIGHT},
    {"fan",          APP_MANUAL_DEV_FAN}
};
#define MANUAL_DEV_MAP_COUNT (sizeof(s_manual_dev_map)/sizeof(s_manual_dev_map[0]))

/**
 * @brief 解析设备名称字符串为枚举值
 * @param dev 设备名称字符串（如 "heater", "fan"）
 * @return 对应的 AppManualDevice_t 枚举值，无效返回 APP_MANUAL_DEV_INVALID
 */
static AppManualDevice_t App_ParseManualDevice(const char *dev)
{
    uint8_t i;
    if (dev == NULL) return APP_MANUAL_DEV_INVALID;
    for (i = 0U; i < MANUAL_DEV_MAP_COUNT; i++)
    {
        if (strcmp(dev, s_manual_dev_map[i].name) == 0)
            return s_manual_dev_map[i].dev;
    }
    return APP_MANUAL_DEV_INVALID;
}

/* ================================================================
 *  阈值 ID / 键名 / 分组 查找表
 *  
 *  将所有可配置阈值编号、对应的 JSON key 名称、所属分组统一管理。
 *  分组说明:
 *    group 1: 温湿度基础阈值 (temp_low/high, humidity_low/fan_high)
 *    group 2: PM2.5/燃气报警阈值及延迟
 *    group 3: 门/灯光/人体检测阈值
 *    group 4: 风扇档位速度 (l1~l5 PWM值)
 *    group 5: PM2.5 分档范围 (5档上下限)
 *    group 6: 燃气分档范围 (5档上下限)
 *    group 7: 近回差/最小切换时间等运行参数
 * ================================================================ */
typedef struct { uint8_t id; uint8_t group; const char *key; } ThresholdMapEntry_t;
static const ThresholdMapEntry_t s_threshold_map[] = {
    { 0,1,"temp_low"}, { 1,1,"temp_high"}, { 2,1,"humidity_low"}, { 3,1,"humidity_fan_high"},
    { 4,2,"pm25_alarm_delay_s"}, { 5,2,"gas_alarm_delay_s"}, { 6,2,"pm25_alarm_threshold"}, { 7,2,"gas_alarm_threshold"},
    { 8,3,"door_open_alarm_s"}, { 9,3,"light_on_threshold"}, {10,3,"human_light_distance"}, {11,3,"fan_speed_l1"},
    {12,4,"fan_speed_l2"}, {13,4,"fan_speed_l3"}, {14,4,"fan_speed_l4"}, {15,4,"fan_speed_l5"},
    {17,5,"pm_l1_low"}, {18,5,"pm_l1_high"}, {19,5,"pm_l2_low"}, {20,5,"pm_l2_high"},
    {21,5,"pm_l3_low"}, {22,5,"pm_l3_high"}, {23,5,"pm_l4_low"}, {24,5,"pm_l4_high"},
    {25,5,"pm_l5_low"}, {26,5,"pm_l5_high"},
    {33,6,"gas_l1_low"}, {34,6,"gas_l1_high"}, {35,6,"gas_l2_low"}, {36,6,"gas_l2_high"},
    {37,6,"gas_l3_low"}, {38,6,"gas_l3_high"}, {39,6,"gas_l4_low"}, {40,6,"gas_l4_high"},
    {41,6,"gas_l5_low"}, {42,6,"gas_l5_high"},
    {48,7,"hyst_temp"}, {49,7,"hyst_humi"}, {50,7,"hyst_light"},
    {51,7,"hyst_gas"}, {52,7,"hyst_pm25"}, {53,7,"min_switch_s"}
};
#define THRESHOLD_MAP_COUNT (sizeof(s_threshold_map)/sizeof(s_threshold_map[0]))

/**
 * @brief 根据阈值 ID 获取所属分组
 * @param id        阈值编号
 * @param group_out 输出分组号 (1~7)
 * @return 1=找到, 0=未找到
 */
static uint8_t App_ThresholdGroupFromId(uint8_t id, uint8_t *group_out)
{
    uint32_t i;
    for (i = 0; i < THRESHOLD_MAP_COUNT; i++)
    {
        if (s_threshold_map[i].id == id)
        {
            if (group_out) *group_out = s_threshold_map[i].group;
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief 根据阈值 ID 获取对应的 JSON key 名称
 * @param id 阈值编号
 * @return key 字符串，未找到返回空字符串
 */
static const char *App_ThresholdKeyFromId(uint8_t id)
{
    uint32_t i;
    for (i = 0; i < THRESHOLD_MAP_COUNT; i++)
    {
        if (s_threshold_map[i].id == id) return s_threshold_map[i].key;
    }
    return "";
}

/**
 * @brief 根据 JSON key 名称获取阈值 ID
 * @param key key 字符串（如 "temp_low"）
 * @return 阈值 ID，未找到返回 -1
 */
static int App_ThresholdIdFromKey(const char *key)
{
    uint32_t i;
    if (!key) return -1;
    for (i = 0; i < THRESHOLD_MAP_COUNT; i++)
    {
        if (strcmp(s_threshold_map[i].key, key) == 0) 
        {
            return (int)s_threshold_map[i].id;
        }
    }
    return -1;
}

/**
 * @brief 根据阈值 ID 和 JSON 数据设置阈值
 *
 * 根据分组类型决定解析浮点数还是整数:
 *   - group 1~4, 7: 浮点数阈值（如温度、湿度、近回差）
 *   - group 5~6: 整数阈值（PM2.5/燃气分档范围）
 *
 * @param id              阈值编号
 * @param json            包含 "value" 字段的 JSON 字符串
 * @param out_group       输出: 所属分组号
 * @param out_float_value 输出: 浮点阈值（group 1~4,7）
 * @param out_int_value   输出: 整数阈值（group 5~6）
 * @param out_is_int      输出: 1=整数类型, 0=浮点类型
 * @return 1=设置成功, 0=失败
 */
static uint8_t App_SetThresholdById(uint8_t id,
                                    const char *json,
                                    uint8_t *out_group,
                                    float *out_float_value,
                                    int *out_int_value,
                                    uint8_t *out_is_int)
{
    uint8_t group;
    uint8_t set_ok = 0U;
    float fvalue = 0.0f;
    int ivalue = 0;

    if ((json == NULL) || !App_ThresholdGroupFromId(id, &group))
    {
        return 0U;
    }

    /* group 1~4, 7: 浮点数阈值（温度、湿度、延迟、风速、近回差等） */
    if (group <= 4U || group == 7U)
    {
        if (!JsonParse_GetFloat(json, "value", &fvalue))
        {
            if (!JsonParse_GetInt(json, "value", &ivalue))
            {
                return 0U;
            }
            fvalue = (float)ivalue;
        }
        if (group == 1U)
        {
            set_ok = Logic_SetChange1ThresholdById(id, fvalue);
        }
        else if (group == 2U)
        {
            set_ok = Logic_SetChange2ThresholdById(id, fvalue);
        }
        else if (group == 3U)
        {
            set_ok = Logic_SetChange3ThresholdById(id, fvalue);
        }
        else if (group == 4U)
        {
            set_ok = Logic_SetChange4ThresholdById(id, fvalue);
        }
        else
        {
            set_ok = Logic_SetChange7ThresholdById(id, fvalue);
        }
        if (set_ok != 0U)
        {
            if (out_float_value != NULL)
            {
                *out_float_value = fvalue;
            }
            if (out_is_int != NULL)
            {
                *out_is_int = 0U;
            }
        }
    }
    else
    {
        /* group 5~6: 整数阈值（PM2.5/燃气分档范围） */
        if (!JsonParse_GetInt(json, "value", &ivalue))
        {
            return 0U;
        }
        if (ivalue < 0)
        {
            return 0U;
        }
        if (group == 5U)
        {
            set_ok = Logic_SetChange5ThresholdById(id, (uint32_t)ivalue);
        }
        else
        {
            set_ok = Logic_SetChange6ThresholdById(id, (uint32_t)ivalue);
        }
        if (set_ok != 0U)
        {
            if (out_int_value != NULL)
            {
                *out_int_value = ivalue;
            }
            if (out_is_int != NULL)
            {
                *out_is_int = 1U;
            }
        }
    }

    if ((set_ok != 0U) && (out_group != NULL))
    {
        *out_group = group;
    }

    return set_ok;
}

/* ================================================================
 *  手动状态快照辅助结构与函数
 *  一次性获取所有设备的手动/自动状态，避免多次调用
 * ================================================================ */
typedef struct
{
    LogicManualStatus_t heater, cooler, humid, dehumid, alarm, light; /* 各设备手动状态 */
    uint8_t fan_is_manual, fan_gear;  /* 风扇: 手动模式标志 + 当前档位 */
} AppManualSnapshot_t;

/**
 * @brief 获取所有设备的手动状态快照
 * @param m 输出结构体指针
 */
static void App_GetManualSnapshot(AppManualSnapshot_t *m)
{
    m->heater  = Logic_GetDeviceManualStatus(LOGIC_DEV_HEATER);
    m->cooler  = Logic_GetDeviceManualStatus(LOGIC_DEV_COOLER);
    m->humid   = Logic_GetDeviceManualStatus(LOGIC_DEV_HUMIDIFIER);
    m->dehumid = Logic_GetDeviceManualStatus(LOGIC_DEV_DEHUMIDIFIER);
    m->alarm   = Logic_GetDeviceManualStatus(LOGIC_DEV_ALARM);
    m->light   = Logic_GetDeviceManualStatus(LOGIC_DEV_LIGHT);
    Logic_GetFanModeAndGear(&m->fan_is_manual, &m->fan_gear);
}

/* ================================================================
 *  发布函数组
 *  负责构造 JSON 并通过 App_MqttPublishRaw 发布到对应主题
 * ================================================================ */

/**
 * @brief 发布设备状态 JSON 到 tele/status 主题
 *
 * 包含: WiFi状态、继电器链路、各传感器连接状态、
 *       人体/光照/门磁实时值、各设备手动状态
 *
 * @return 1=发布成功, 0=失败或未就绪
 */
static uint8_t App_MqttPublishStatus(void)
{
    AllSensorConnectionStatus_t conn;
    AllSensorData_t sensor;
    AppManualSnapshot_t m;
    uint8_t relay_connected;
    uint32_t seq;
    char msg[320];
    int len;

    if ((s_esp_runtime_ready == 0U) || (s_mqtt_ready == 0U))
    {
        return 0U;
    }

    AllSensorData_GetConnectionStatus(&conn);   /* 获取传感器连接状态 */
    AllSensorData_GetSnapshot(&sensor);           /* 获取传感器数据快照 */
    relay_connected = MqttApp_GetRelayLinkCached();   /* 获取继电器链路缓存 */
    App_GetManualSnapshot(&m);                    /* 获取手动状态快照 */

    seq = App_MqttNextSeq();
    /* 构建状态 JSON: id, seq, wifi, relay, conn{}, sensor{}, manual{} */
    len = snprintf(msg,
                   sizeof(msg),
                   "{\"id\":\"%s\",\"seq\":%lu,\"wifi\":%u,\"relay_link\":%u,"
                   "\"conn\":{\"th\":%u,\"pm\":%u,\"gas\":%u,\"human\":%u,\"light\":%u,\"door\":%u},"
                   "\"sensor\":{\"human\":%u,\"dist\":%u,\"light\":%u,\"door\":%u},"
                   "\"manual\":{\"heater\":%u,\"cooler\":%u,\"humidifier\":%u,\"dehumidifier\":%u,\"alarm\":%u,\"light\":%u,\"fan_mode\":%u,\"fan_gear\":%u}}",
                   ESP01S_MQTT_CLIENT_ID,
                   (unsigned long)seq,
                   (unsigned int)ESP01S_IsConnected(),
                   (unsigned int)relay_connected,
                   (unsigned int)conn.temp_humi_connected,
                   (unsigned int)conn.pm_connected,
                   (unsigned int)conn.gas_connected,
                   (unsigned int)conn.human_connected,
                   (unsigned int)conn.light_connected,
                   (unsigned int)conn.door_connected,
                   (unsigned int)sensor.human_presence,
                   (unsigned int)sensor.human_distance,
                   (unsigned int)sensor.light_lux,
                   (unsigned int)sensor.door_closed,
                   (unsigned int)m.heater,
                   (unsigned int)m.cooler,
                   (unsigned int)m.humid,
                   (unsigned int)m.dehumid,
                   (unsigned int)m.alarm,
                   (unsigned int)m.light,
                   (unsigned int)m.fan_is_manual,
                   (unsigned int)m.fan_gear);
    if ((len <= 0) || (len >= (int)sizeof(msg)))
    {
        return 0U;
    }

    return App_MqttPublishRaw(ESP01S_MQTT_TELE_STATUS, msg);
}

/**
 * @brief 发布指定分组的阈值参数 JSON
 *
 * 根据分组号调用对应的 Logic_GetChangeX 函数获取阈值，
 * 构造 JSON 并发布到对应的 tele/thresholds/changeX 主题。
 *
 * @param group 分组号 (1~7)
 * @return 1=发布成功, 0=失败
 */
static uint8_t App_MqttPublishThresholdGroup(uint8_t group)
{
    char msg[300];
    int len;
    uint32_t seq;
    float a, b, c, d;
    uint16_t low[5], high[5];
    const char *topic;

    if ((s_esp_runtime_ready == 0U) || (s_mqtt_ready == 0U))
    {
        return 0U;
    }

    seq = App_MqttNextSeq();

    /* group 1~4: 四个浮点阈值参数 */
    if ((group >= 1U) && (group <= 4U))
    {
        switch (group)
        {
        case 1U: Logic_GetChange1Thresholds(&a,&b,&c,&d); topic = ESP01S_MQTT_TELE_CHANGE1; break;
        case 2U: Logic_GetChange2Thresholds(&a,&b,&c,&d); topic = ESP01S_MQTT_TELE_CHANGE2; break;
        case 3U: Logic_GetChange3Thresholds(&a,&b,&c,&d); topic = ESP01S_MQTT_TELE_CHANGE3; break;
        default: Logic_GetChange4Thresholds(&a,&b,&c,&d); topic = ESP01S_MQTT_TELE_CHANGE4; break;
        }
        len = snprintf(msg, sizeof(msg),
                       "{\"id\":\"%s\",\"seq\":%lu,\"group\":%u,\"v\":[%.1f,%.1f,%.1f,%.1f]}",
                       ESP01S_MQTT_CLIENT_ID, (unsigned long)seq, (unsigned int)group,
                       (double)a, (double)b, (double)c, (double)d);
    }
    else if ((group == 5U) || (group == 6U))
    {
        /* group 5~6: 五档上下限整数阈值 (PM2.5 或 燃气) */
        if (group == 5U) { Logic_GetChange5Thresholds(low, high); topic = ESP01S_MQTT_TELE_CHANGE5; }
        else              { Logic_GetChange6Thresholds(low, high); topic = ESP01S_MQTT_TELE_CHANGE6; }
        len = snprintf(msg, sizeof(msg),
                       "{\"id\":\"%s\",\"seq\":%lu,\"group\":%u,"
                       "\"low\":[%u,%u,%u,%u,%u],\"high\":[%u,%u,%u,%u,%u]}",
                       ESP01S_MQTT_CLIENT_ID, (unsigned long)seq, (unsigned int)group,
                       (unsigned int)low[0], (unsigned int)low[1], (unsigned int)low[2], (unsigned int)low[3], (unsigned int)low[4],
                       (unsigned int)high[0], (unsigned int)high[1], (unsigned int)high[2], (unsigned int)high[3], (unsigned int)high[4]);
    }
    else if (group == 7U)
    {
        /* group 7: 六个运行参数（近回差、最小切换时间等） */
        float e, f;
        Logic_GetChange7Thresholds(&a, &b, &c, &d, &e, &f);
        topic = ESP01S_MQTT_TELE_CHANGE7;
        len = snprintf(msg, sizeof(msg),
                       "{\"id\":\"%s\",\"seq\":%lu,\"group\":7,\"v\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}",
                       ESP01S_MQTT_CLIENT_ID, (unsigned long)seq,
                       (double)a, (double)b, (double)c, (double)d, (double)e, (double)f);
    }
    else
    {
        return 0U;
    }

    if ((len > 0) && (len < (int)sizeof(msg)))
        return App_MqttPublishRaw(topic, msg);
    return 0U;
}

/**
 * @brief 发布所有阈值的扁平 JSON 到 tele/thresholds 主题
 *
 * 将 7 组阈值参数全部打包在一个 JSON 中发布。
 * 使用 static 局部变量保存消息缓冲区，节省任务栈空间。
 *
 * @return 1=发布成功, 0=失败
 */
static uint8_t App_MqttPublishThresholdFlat(void)
{
    static char msg[600];  /* static: 节省约 600 字节任务栈空间 */
    int len;
    uint32_t seq;
    float c1_0, c1_1, c1_2, c1_3;
    float c2_0, c2_1, c2_2, c2_3;
    float c3_0, c3_1, c3_2, c3_3;
    float c4_0, c4_1, c4_2, c4_3;
    float c7_0, c7_1, c7_2, c7_3, c7_4, c7_5;
    uint16_t pm_low[5];
    uint16_t pm_high[5];
    uint16_t gas_low[5];
    uint16_t gas_high[5];

    if ((s_esp_runtime_ready == 0U) || (s_mqtt_ready == 0U))
    {
        return 0U;
    }

    /* 获取所有 7 组阈值参数 */
    Logic_GetChange1Thresholds(&c1_0, &c1_1, &c1_2, &c1_3);
    Logic_GetChange2Thresholds(&c2_0, &c2_1, &c2_2, &c2_3);
    Logic_GetChange3Thresholds(&c3_0, &c3_1, &c3_2, &c3_3);
    Logic_GetChange4Thresholds(&c4_0, &c4_1, &c4_2, &c4_3);
    Logic_GetChange5Thresholds(pm_low, pm_high);
    Logic_GetChange6Thresholds(gas_low, gas_high);
    Logic_GetChange7Thresholds(&c7_0, &c7_1, &c7_2, &c7_3, &c7_4, &c7_5);

    seq = App_MqttNextSeq();
    len = snprintf(msg,
                   sizeof(msg),
                   "{\"id\":\"%s\",\"seq\":%lu,\"v\":{"
                   "\"0\":%.1f,\"1\":%.1f,\"2\":%.1f,\"3\":%.1f,"
                   "\"4\":%.1f,\"5\":%.1f,\"6\":%.1f,\"7\":%.1f,"
                   "\"8\":%.1f,\"9\":%.1f,\"10\":%.1f,\"11\":%.1f,"
                   "\"12\":%.1f,\"13\":%.1f,\"14\":%.1f,\"15\":%.1f,"
                   "\"17\":%u,\"18\":%u,\"19\":%u,\"20\":%u,\"21\":%u,"
                   "\"22\":%u,\"23\":%u,\"24\":%u,\"25\":%u,\"26\":%u,"
                   "\"33\":%u,\"34\":%u,\"35\":%u,\"36\":%u,\"37\":%u,"
                   "\"38\":%u,\"39\":%u,\"40\":%u,\"41\":%u,\"42\":%u,"
                   "\"48\":%.1f,\"49\":%.1f,\"50\":%.1f,"
                   "\"51\":%.1f,\"52\":%.1f,\"53\":%.1f}}",
                   ESP01S_MQTT_CLIENT_ID,
                   (unsigned long)seq,
                   (double)c1_0, (double)c1_1, (double)c1_2, (double)c1_3,
                   (double)c2_0, (double)c2_1, (double)c2_2, (double)c2_3,
                   (double)c3_0, (double)c3_1, (double)c3_2, (double)c3_3,
                   (double)c4_0, (double)c4_1, (double)c4_2, (double)c4_3,
                   (unsigned int)pm_low[0], (unsigned int)pm_high[0],
                   (unsigned int)pm_low[1], (unsigned int)pm_high[1],
                   (unsigned int)pm_low[2], (unsigned int)pm_high[2],
                   (unsigned int)pm_low[3], (unsigned int)pm_high[3],
                   (unsigned int)pm_low[4], (unsigned int)pm_high[4],
                   (unsigned int)gas_low[0], (unsigned int)gas_high[0],
                   (unsigned int)gas_low[1], (unsigned int)gas_high[1],
                   (unsigned int)gas_low[2], (unsigned int)gas_high[2],
                   (unsigned int)gas_low[3], (unsigned int)gas_high[3],
                   (unsigned int)gas_low[4], (unsigned int)gas_high[4],
                   (double)c7_0, (double)c7_1, (double)c7_2,
                   (double)c7_3, (double)c7_4, (double)c7_5);
    if ((len <= 0) || (len >= (int)sizeof(msg)))
    {
        return 0U;
    }

    return App_MqttPublishRaw(ESP01S_MQTT_TELE_THRESHOLDS, msg);
}

/**
 * @brief 发布所有阈值参数（当前实现为扁平格式）
 * @return 1=发布成功, 0=失败
 */
static uint8_t App_MqttPublishAllThresholds(void)
{
    return App_MqttPublishThresholdFlat();
}

/**
 * @brief 安全 JSON 字符串追加工具
 *
 * 使用 vsnprintf 安全追加内容到缓冲区，任何截断或格式错误立即返回失败。
 *
 * @param buf      目标缓冲区
 * @param buf_size 缓冲区大小
 * @param pos_io   当前写入位置（输入/输出）
 * @param fmt      printf 格式字符串
 * @return 1=追加成功, 0=缓冲区不足或格式错误
 */
static uint8_t App_AckJsonAppend(char *buf, uint16_t buf_size, int *pos_io, const char *fmt, ...)
{
    int pos;
    size_t remain;
    int wrote;
    va_list ap;

    if ((buf == NULL) || (pos_io == NULL) || (fmt == NULL) || (buf_size == 0U))
    {
        return 0U;
    }

    pos = *pos_io;
    if ((pos < 0) || (pos >= (int)buf_size))
    {
        return 0U;
    }

    remain = (size_t)buf_size - (size_t)pos;
    if (remain == 0U)
    {
        return 0U;
    }

    va_start(ap, fmt);
    wrote = vsnprintf(buf + pos, remain, fmt, ap);
    va_end(ap);

    if ((wrote < 0) || ((size_t)wrote >= remain))
    {
        return 0U;
    }

    *pos_io = pos + wrote;
    return 1U;
}

/**
 * @brief 构造并发布 ACK 确认响应 JSON
 *
 * ACK JSON 结构: {id, ok, cmd, [reason], [extra_kv], [req_id], seq}
 * 发布流程:
 *   1. 构造 JSON 字符串
 *   2. 将 req_id + JSON 存入去重缓存
 *   3. 若处于 batch-defer 模式，入延迟队列
 *   4. 否则尝试直接发布，失败则入延迟队列
 *
 * @param cmd           命令名称字符串（如 "threshold", "manual"）
 * @param ok            执行结果 1=成功, 0=失败
 * @param reason        失败原因（ok=0 时使用）
 * @param extra_json_kv 额外 JSON 键值对（如 "\"id\":1,\"value\":25.0"）
 * @return 1=即时发布成功, 0=已延迟或失败
 */
static uint8_t App_MqttPublishAck(const char *cmd,
                                  uint8_t ok,
                                  const char *reason,
                                  const char *extra_json_kv)
{
    char ack_json[APP_ACK_JSON_SIZE];
    const char *req_id;
    uint32_t seq;
    int pos;

    if (s_esp_runtime_ready == 0U) return 0U;

    req_id = (s_ack_req_id_valid != 0U) ? s_ack_req_id : NULL;
    seq = App_MqttNextSeq();

    /* —— 构建 ACK JSON —— */
    pos = 0;
    /* 基础字段: id, ok, cmd */
    if (!App_AckJsonAppend(ack_json, sizeof(ack_json), &pos,
                           "{\"id\":\"%s\",\"ok\":%u,\"cmd\":\"%s\"",
                           ESP01S_MQTT_CLIENT_ID, (unsigned int)ok,
                           cmd ? cmd : ""))
                           {
        CMD_LOG("[CMD][ACK][BUILD_FAIL] cmd=%s ok=%u\r\n",
                cmd ? cmd : "", (unsigned int)ok);
        return 0U;
    }

    /* 失败时添加 reason 字段 */
    if (!ok)
    {
        if (!App_AckJsonAppend(ack_json, sizeof(ack_json), &pos,
                               ",\"reason\":\"%s\"", reason ? reason : "busy"))
                               {
            CMD_LOG("[CMD][ACK][BUILD_FAIL] cmd=%s ok=%u\r\n",
                    cmd ? cmd : "", (unsigned int)ok);
            return 0U;
        }
    }

    /* 添加额外键值对（如阈值值、设备名等） */
    if (extra_json_kv && extra_json_kv[0])
    {
        if (!App_AckJsonAppend(ack_json, sizeof(ack_json), &pos,
                               ",%s", extra_json_kv))
                               {
            CMD_LOG("[CMD][ACK][BUILD_FAIL] cmd=%s ok=%u\r\n",
                    cmd ? cmd : "", (unsigned int)ok);
            return 0U;
        }
    }

    /* 添加请求 ID（用于上位机关联响应） */
    if (req_id && req_id[0])
    {
        if (!App_AckJsonAppend(ack_json, sizeof(ack_json), &pos,
                               ",\"req_id\":\"%s\"", req_id))
                               {
            CMD_LOG("[CMD][ACK][BUILD_FAIL] cmd=%s ok=%u\r\n",
                    cmd ? cmd : "", (unsigned int)ok);
            return 0U;
        }
    }

    /* 添加序列号并关闭 JSON */
    if (!App_AckJsonAppend(ack_json, sizeof(ack_json), &pos,
                           ",\"seq\":%lu}", (unsigned long)seq))
                           {
        CMD_LOG("[CMD][ACK][BUILD_FAIL] cmd=%s ok=%u\r\n",
                cmd ? cmd : "", (unsigned int)ok);
        return 0U;
    }

    if ((pos <= 0) || (pos >= (int)sizeof(ack_json)))
    {
        CMD_LOG("[CMD][ACK][BUILD_FAIL] cmd=%s ok=%u\r\n",
                cmd ? cmd : "", (unsigned int)ok);
        return 0U;
    }

    CMD_LOG("[CMD][ACK][TX] cmd=%s ok=%u req=%s payload=%s\r\n",
            cmd ? cmd : "", (unsigned int)ok,
            (req_id && req_id[0]) ? req_id : "-", ack_json);

    /* 将请求 ID 与 ACK JSON 存入去重缓存 */
    if (req_id && req_id[0])
        App_RememberReqAck(req_id, ack_json);

    /* Burst 模式: 在 MQTT poll 循环内，延迟 ACK 以避免阻塞
       poll。但队列已积压较多时，尝试内联发送以追赶进度 */
    if (s_ack_defer_only != 0U)
    {
        if (s_ack_deferred_count < 6U)
        {
            /* 队列较空，直接入队延迟发送 */
            CMD_LOG("[CMD][ACK][BATCH_DEFER] cmd=%s pending=%u\r\n",
                    cmd ? cmd : "", (unsigned int)s_ack_deferred_count);
            App_SaveDeferredAck(ack_json);
            return 0U;
        }
        /* 队列积压较多，尝试内联立即发送 */
        CMD_LOG("[CMD][ACK][INLINE_FORCE] cmd=%s pending=%u\r\n",
                cmd ? cmd : "", (unsigned int)s_ack_deferred_count);
    }

    if (App_MqttPublishRaw(ESP01S_MQTT_ACK_TOPIC, ack_json))
    {
        CMD_LOG("[CMD][ACK][OK] cmd=%s attempt=1\r\n", cmd ? cmd : "");
        return 1U;
    }

    /* 直接发布失败，入延迟队列等待重试 */
    CMD_LOG("[CMD][ACK][DEFER] cmd=%s err=%u(%s)\r\n",
            cmd ? cmd : "",
            (unsigned int)ESP01S_GetLastErrorCode(),
            ESP01S_GetLastErrorText());
    App_SaveDeferredAck(ack_json);
    return 0U;
}

/* ================================================================
 *  设备动作遥测（编译时可开关）
 *  记录设备开关、档位变化事件，发布到 tele/action 主题
 * ================================================================ */
#if (APP_ENABLE_ACTION_TELEMETRY != 0U)

/** 设备动作名称字符串数组（与 ScreenActionDevice_t 枚举对应） */
static const char * const s_action_dev_names[] = {
    "heater", "cooler", "humidifier", "alarm", "light", "fan"
};

/**
 * @brief 获取设备动作名称字符串
 * @param device 设备枚举值
 * @return 设备名称字符串，无效返回 "unknown"
 */
static const char *App_ActionDeviceName(ScreenActionDevice_t device)
{
    if ((uint8_t)device < (sizeof(s_action_dev_names) / sizeof(s_action_dev_names[0])))
        return s_action_dev_names[(uint8_t)device];
    return "unknown";
}

/**
 * @brief 发布单个设备动作事件到 tele/action 主题
 *
 * 包含去重逻辑: 同一设备、同一动作在去重窗口内不重复发送。
 * 发布失败时缓存 JSON，下次重试时复用同一序列号。
 *
 * @param evt 动作事件结构体指针
 * @return 1=发布成功或去重跳过, 0=发布失败
 */
static uint8_t App_MqttPublishActionEvent(const ScreenActionEvent_t *evt)
{
    char action_json[256];
    const char *dev;
    const char *mode;
    char action[24];
    uint32_t seq;
    int len;
    TickType_t now_tick;
    TickType_t dedup_ticks;
    TickType_t min_gap_ticks;

    if ((evt == NULL) || (s_esp_runtime_ready == 0U) || (s_mqtt_ready == 0U))
    {
        return 0U;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        now_tick = xTaskGetTickCount();
        dedup_ticks = SafeMsToTicks(APP_ACTION_DEDUP_WINDOW_MS);

        /* 去重判断: 同一设备、同一模式、同一值在窗口内不重复上报 */
        if ((s_last_action_pub_valid != 0U) &&
            (s_last_action_pub_device == evt->device) &&
            (s_last_action_pub_manual == ((evt->is_manual != 0U) ? 1U : 0U)) &&
            (s_last_action_pub_value == evt->action_value) &&
            ((now_tick - s_last_action_pub_tick) <= dedup_ticks))
            {
            PERF_LOG("[PERF][ACTION][DEDUP] dev=%u manual=%u value=%u\r\n",
                     (unsigned int)evt->device,
                     (unsigned int)evt->is_manual,
                     (unsigned int)evt->action_value);
            return 1U;
        }
    }
    else
    {
        now_tick = 0U;
    }

    /* 最小发送间隔控制，避免短时间内大量发送 */
    min_gap_ticks = SafeMsToTicks(APP_ACTION_PUB_MIN_GAP_MS);
    if ((now_tick != 0U) && (s_last_action_tx_tick != 0U) &&
        ((now_tick - s_last_action_tx_tick) < min_gap_ticks))
        {
        return 0U;
    }

    dev = App_ActionDeviceName(evt->device);
    mode = (evt->is_manual != 0U) ? "manual" : "auto";

    /* 风扇设备用档位名称（gear_1~gear_5），其他设备用 on/off */
    if (evt->device == SCREEN_ACTION_DEV_FAN)
    {
        uint8_t gear = evt->action_value;
        if (gear < 1U)
        {
            gear = 1U;
        }
        else if (gear > 5U)
        {
            gear = 5U;
        }
        snprintf(action, sizeof(action), "gear_%u", (unsigned int)gear);
    }
    else
    {
        snprintf(action, sizeof(action), "%s", (evt->action_value != 0U) ? "on" : "off");
    }

    seq = App_MqttNextSeq();
    len = snprintf(action_json,
                   sizeof(action_json),
                   "{\"id\":\"%s\",\"seq\":%lu,\"cmd\":\"device\",\"dev\":\"%s\","
                   "\"mode\":\"%s\",\"action\":\"%s\",\"manual\":%u,\"value\":%u,"
                   "\"date\":\"%s\",\"time\":\"%s\"}",
                   ESP01S_MQTT_CLIENT_ID,
                   (unsigned long)seq,
                   dev,
                   mode,
                   action,
                   (unsigned int)evt->is_manual,
                   (unsigned int)evt->action_value,
                   evt->date_text,
                   evt->time_text);
    if ((len <= 0) || (len >= (int)sizeof(action_json)))
    {
        return 0U;
    }

    /* 缓存 JSON，便于发布失败时重试时复用同一 seq */
    memcpy(s_action_pending_json, action_json, (size_t)len + 1U);
    s_action_pending_json_valid = 1U;

    if (!App_MqttPublishRaw(ESP01S_MQTT_ACTION_TOPIC, action_json))
    {
        return 0U;
    }

    s_action_pending_json_valid = 0U;  /* 发送成功，清除缓存 */
    s_last_action_pub_valid = 1U;
    s_last_action_pub_device = evt->device;
    s_last_action_pub_manual = (evt->is_manual != 0U) ? 1U : 0U;
    s_last_action_pub_value = evt->action_value;
    s_last_action_pub_tick = now_tick;
    s_last_action_tx_tick = now_tick;
    return 1U;
}
#endif /* APP_ENABLE_ACTION_TELEMETRY */

/**
 * @brief 排空动作事件队列（重试待发 + 从屏幕模块弹出新事件）
 *
 * 优先重试上次失败的待发事件，然后从 Screen 队列中取新事件。
 * 在命令静默期内不发送（避免与 ACK 发布冲突）。
 *
 * @return 1=成功发布了一个事件, 0=无事件或发布失败
 */
static uint8_t App_MqttDrainActionEvents(void)
{
#if (APP_ENABLE_ACTION_TELEMETRY == 0U)
    return 0U;
#else
    ScreenActionEvent_t evt;
    TickType_t now_tick;
    TickType_t last_cmd_tick;
    TickType_t quiet_ticks;

    if ((s_esp_runtime_ready == 0U) || (s_mqtt_ready == 0U))
    {
        return 0U;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        now_tick = xTaskGetTickCount();
        quiet_ticks = SafeMsToTicks(APP_CMD_QUIET_PUB_GAP_MS);
        taskENTER_CRITICAL();
        last_cmd_tick = s_last_cmd_rx_tick;
        taskEXIT_CRITICAL();
        /* 命令静默期: 刚收到命令后暂停动作事件发布，避免与 ACK 冲突（收到命令后 1.5 秒内，暂停所有非 ACK 的发布） */
        if ((last_cmd_tick != 0U) && ((now_tick - last_cmd_tick) < quiet_ticks))
        {
            return 0U;
        }
    }

    /* ---- 重试待发事件（复用缓存的 JSON，同一 seq） ---- */
    if (s_action_pending_valid != 0U)
    {
        if (s_action_pending_json_valid != 0U)
        {
            /* 复用完全相同的 JSON（相同 seq），避免重复消息 */
            TickType_t min_gap_ticks = SafeMsToTicks(APP_ACTION_PUB_MIN_GAP_MS);
            TickType_t now_t = xTaskGetTickCount();
            if ((s_last_action_tx_tick != 0U) &&
                ((now_t - s_last_action_tx_tick) < min_gap_ticks))
            {
                return 0U;
            }
            if (!App_MqttPublishRaw(ESP01S_MQTT_ACTION_TOPIC,
                                    s_action_pending_json))
            {
                return 0U;
            }
            /* Update dedup state on success */
            s_last_action_pub_valid   = 1U;
            s_last_action_pub_device  = s_action_pending.device;
            s_last_action_pub_manual  = (s_action_pending.is_manual != 0U) ? 1U : 0U;
            s_last_action_pub_value   = s_action_pending.action_value;
            s_last_action_pub_tick    = now_t;
            s_last_action_tx_tick     = now_t;
        }
        else
        {
            /* 备用: 首次构建（正常不应发生） */
            if (!App_MqttPublishActionEvent(&s_action_pending))
            {
                return 0U;
            }
        }
        s_action_pending_valid      = 0U;
        s_action_pending_json_valid = 0U;
        memset(&s_action_pending, 0, sizeof(s_action_pending));
        return 1U;
    }

    /* ---- 从屏幕模块弹出新事件 ---- */
    if (Screen_PopActionEvent(&evt))
    {
        if (!App_MqttPublishActionEvent(&evt))
        {
            /* 发布失败，缓存事件等待下次重试 */
            s_action_pending = evt;
            s_action_pending_valid = 1U;
            /* s_action_pending_json 已由 App_MqttPublishActionEvent
               在构建 JSON 时填充，虽然发布本身失败了 */
            return 0U;
        }
        return 1U;
    }

    return 0U;
#endif
}

/**
 * @brief 发布传感器遥测数据 JSON 到 tele/sensors 主题
 *
 * 包含: 温度、湿度、PM2.5、燃气、PWM、人体、距离、光照、门磁、
 *       各设备手动状态
 *
 * @return 1=发布成功, 0=失败或未就绪
 */
static uint8_t App_MqttPublishTelemetry(void)
{
    AllSensorData_t sensor_data;
    AppManualSnapshot_t m;
    char mqtt_json[512];
    int mqtt_len;
    uint32_t seq;

    if ((s_esp_runtime_ready == 0U) || (s_mqtt_ready == 0U))
    {
        return 0U;
    }

    AllSensorData_GetSnapshot(&sensor_data);  /* 获取传感器数据快照 */
    App_GetManualSnapshot(&m);                 /* 获取手动状态快照 */
    seq = App_MqttNextSeq();
    /* 构建传感器遥测 JSON */
    mqtt_len = snprintf(mqtt_json,
                        sizeof(mqtt_json),
                        "{\"id\":\"%s\",\"seq\":%lu,"
                        "\"temp\":%.1f,\"humi\":%.1f,\"pm25\":%u,\"gas\":%u,\"pwm\":%u,"
                        "\"human\":%u,\"dist\":%u,\"light\":%u,\"door\":%u,"
                        "\"manual\":{\"heater\":%u,\"cooler\":%u,\"humidifier\":%u,\"dehumidifier\":%u,\"alarm\":%u,\"light\":%u,\"fan_mode\":%u,\"fan_gear\":%u}}",
                        ESP01S_MQTT_CLIENT_ID,
                        (unsigned long)seq,
                        (double)sensor_data.temperature,
                        (double)sensor_data.humidity,
                        (unsigned int)sensor_data.pm25,
                        (unsigned int)sensor_data.gas,
                        (unsigned int)Motor_GetDutyPercent(),
                        (unsigned int)sensor_data.human_presence,
                        (unsigned int)sensor_data.human_distance,
                        (unsigned int)sensor_data.light_lux,
                        (unsigned int)sensor_data.door_closed,
                        (unsigned int)m.heater,
                        (unsigned int)m.cooler,
                        (unsigned int)m.humid,
                        (unsigned int)m.dehumid,
                        (unsigned int)m.alarm,
                        (unsigned int)m.light,
                        (unsigned int)m.fan_is_manual,
                        (unsigned int)m.fan_gear);
    if ((mqtt_len <= 0) || (mqtt_len >= (int)sizeof(mqtt_json)))
    {
        return 0U;
    }

    if (!App_MqttPublishRaw(ESP01S_MQTT_PUB_TOPIC, mqtt_json))
    {
        ESP_LOG("[MQTT] telemetry publish failed\r\n");
        return 0U;
    }

    return 1U;
}

/* ================================================================
 *  命令分发器（MQTT 消息回调）
 *  根据主题和负载 JSON 执行对应操作，并发送 ACK 确认
 * ================================================================ */

/**
 * @brief MQTT 消息回调处理函数
 *
 * 支持的命令主题:
 *   - tele/time:      时间同步（NTP 时间发给串口屏）
 *   - cmd/relay:      继电器直控 (ch=1~4, on=0/1)
 *   - cmd/fan:        风扇 PWM 控制 (pwm=0~100)
 *   - cmd/threshold:  阈值修改 (id/key + value)
 *   - cmd/manual:     手动控制设备 (dev + mode + sec + value)
 *   - cmd/query:      查询传感器/状态/阈值
 *
 * @param topic   MQTT 主题字符串
 * @param payload 消息负载原始字节
 * @param len     负载长度
 */
static void App_MqttOnMessage(const char *topic, const uint8_t *payload, uint16_t len)
{
    char json[ESP01S_MQTT_MAX_PAYLOAD_LEN + 1U];
    char local_time[20];
    char tz_str[8];
    char dev[20];
    char mode[12];
    char what[20];
    char req_id[40];
    char extra[120];
    uint16_t copy_len;
    float fvalue;
    int ch;
    int on;
    int pwm;
    int id;
    int sec;
    int ts_value;
    int value;
    uint8_t set_ok;
    uint8_t has_ts;
    uint8_t query_need_ack;
    AppManualDevice_t manual_dev;

    if ((topic == NULL) || (payload == NULL))
    {
        return;
    }

    /* 仅 cmd 主题标记网页在线；    tele/time 是服务器推送，不算 */
    if (strncmp(topic, "ws/gw001/cmd/", 13U) == 0)
    {
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        {
            taskENTER_CRITICAL();
            s_last_cmd_rx_tick = xTaskGetTickCount();
            taskEXIT_CRITICAL();
        }
        ESP01S_MarkWebAlive();
    }

    /* 将负载复制到本地缓冲区并添加结尾符 */
    copy_len = len;
    if (copy_len > ESP01S_MQTT_MAX_PAYLOAD_LEN)
    {
        copy_len = ESP01S_MQTT_MAX_PAYLOAD_LEN;
    }
    if (copy_len > 0U)
    {
        memcpy(json, payload, copy_len);
    }
    json[copy_len] = '\0';

    /* 提取请求 ID（用于 ACK 关联和去重） */
    s_ack_req_id_valid = 0U;
    s_ack_req_id[0] = '\0';
    if (JsonParse_GetString(json, "req_id", req_id, sizeof(req_id)))
    {
        strncpy(s_ack_req_id, req_id, sizeof(s_ack_req_id) - 1U);
        s_ack_req_id[sizeof(s_ack_req_id) - 1U] = '\0';
        if (s_ack_req_id[0] != '\0')
        {
            s_ack_req_id_valid = 1U;
        }
    }

    /* 重复请求去重: 如果 req_id 已在缓存中，直接回放上次的 ACK，不重复执行 */
    if ((strncmp(topic, "ws/gw001/cmd/", 13U) == 0) &&
        (s_ack_req_id_valid != 0U) &&
        App_ReplayCachedAckIfDuplicate(s_ack_req_id))
        {
        return;
    }

    if (strncmp(topic, "ws/gw001/cmd/", 13U) == 0)
    {
        CMD_LOG("[CMD][RX] topic=%s len=%u req=%s payload=%s\r\n",
                topic,
                (unsigned int)copy_len,
                (s_ack_req_id_valid != 0U) ? s_ack_req_id : "-",
                json);
    }

    /* ==== TIME 命令: 时间同步 ==== */
    if (strcmp(topic, ESP01S_MQTT_TIME_TOPIC) == 0)
    {
        local_time[0] = '\0';
        tz_str[0] = '\0';
        ts_value = 0;
        has_ts = JsonParse_GetInt(json, "ts", &ts_value);
        if ((has_ts != 0U) && (ts_value < 0))
        {
            has_ts = 0U;
            ts_value = 0;
        }

        (void)JsonParse_GetString(json, "local", local_time, sizeof(local_time));
        (void)JsonParse_GetString(json, "tz", tz_str, sizeof(tz_str));

        /* 将时间信息发送给串口屏显示 */
        Screen_SetTimeFromMqtt((has_ts != 0U) ? (uint32_t)ts_value : 0U,
                               has_ts,
                               local_time,
                               tz_str);
        return;
    }

    /* ==== RELAY 命令: 继电器直接控制（没用了，可以直接删了，实际用的下面的manual主题） ==== */
    if (strcmp(topic, "ws/gw001/cmd/relay") == 0)
    {
        if (!JsonParse_GetInt(json, "ch", &ch) || !JsonParse_GetInt(json, "on", &on))
        {
            (void)App_MqttPublishAck("relay", 0U, "missing", NULL);
            return;
        }
        if ((ch < 1) || (ch > 4) || ((on != 0) && (on != 1)))
        {
            (void)App_MqttPublishAck("relay", 0U, "range", NULL);
            return;
        }
        if (!Relay_Set((uint8_t)ch, (uint8_t)on))
        {
            (void)App_MqttPublishAck("relay", 0U, "busy", NULL);
            return;
        }
        snprintf(extra, sizeof(extra), "\"ch\":%d,\"on\":%d", ch, on);
        (void)App_MqttPublishAck("relay", 1U, NULL, extra);
        return;
    }

    /* ==== FAN 命令: 风扇 PWM 控制 ==== */
    if (strcmp(topic, "ws/gw001/cmd/fan") == 0)
    {
        if (!JsonParse_GetInt(json, "pwm", &pwm))
        {
            (void)App_MqttPublishAck("fan", 0U, "missing", NULL);
            return;
        }
        if ((pwm < 0) || (pwm > 100))
        {
            (void)App_MqttPublishAck("fan", 0U, "range", NULL);
            return;
        }
        if (!Fan_SetPWM((uint8_t)pwm))
        {
            (void)App_MqttPublishAck("fan", 0U, "busy", NULL);
            return;
        }
        snprintf(extra, sizeof(extra), "\"pwm\":%d", pwm);
        (void)App_MqttPublishAck("fan", 1U, NULL, extra);
        return;
    }

    /* ==== THRESHOLD 命令: 阈值参数修改 ====
     * 支持两种主题格式:
     *   cmd/threshold       —— JSON 中指定 id/key
     *   cmd/threshold/{id}  —— URL 路径中指定 id
     */
    if ((strcmp(topic, ESP01S_MQTT_CMD_THRESHOLD) == 0) ||
        (strncmp(topic, "ws/gw001/cmd/threshold/", 23U) == 0))
        {
        char key[32];
        const char *suffix = NULL;
        const char *threshold_key;
        int parsed_id = -1;
        uint8_t threshold_group = 0U;
        uint8_t value_is_int = 0U;

        if (strncmp(topic, "ws/gw001/cmd/threshold/", 23U) == 0)
        {
            suffix = topic + 23U;
            if (suffix[0] != '\0')
            {
                parsed_id = atoi(suffix);
                if ((parsed_id == 0) && (suffix[0] != '0'))
                    parsed_id = App_ThresholdIdFromKey(suffix);
            }
        }

        if (JsonParse_GetString(json, "key", key, sizeof(key)))
        {
            parsed_id = App_ThresholdIdFromKey(key);
        }
        else if (JsonParse_GetInt(json, "id", &id))
        {
            parsed_id = id;
        }

        if ((parsed_id < 0) || (parsed_id > 255))
        {
            (void)App_MqttPublishAck("threshold", 0U, "range", NULL);
            return;
        }

        set_ok = App_SetThresholdById((uint8_t)parsed_id,
                                      json,
                                      &threshold_group,
                                      &fvalue,
                                      &value,
                                      &value_is_int);
        if (set_ok == 0U)
        {
            (void)App_MqttPublishAck("threshold", 0U, "range", NULL);
            return;
        }

        threshold_key = App_ThresholdKeyFromId((uint8_t)parsed_id);
        if (value_is_int != 0U)
        {
            snprintf(extra, sizeof(extra),
                     "\"id\":%d,\"key\":\"%s\",\"value\":%d",
                     parsed_id, threshold_key, value);
        }
        else
        {
            snprintf(extra, sizeof(extra),
                     "\"id\":%d,\"key\":\"%s\",\"value\":%.1f",
                     parsed_id, threshold_key, (double)fvalue);
        }

        (void)App_MqttPublishAck("threshold", 1U, NULL, extra);
        return;
    }

    /* ==== MANUAL 命令: 手动控制设备 ====
     * 支持两种主题格式:
     *   cmd/manual         —— JSON 中指定 dev
     *   cmd/manual/{dev}   —— URL 路径中指定设备名
     * mode="auto"  → 恢复自动控制
     * mode="manual" → 手动控制，需要 sec(秒) + value(开关或档位)
     */
    if ((strcmp(topic, ESP01S_MQTT_CMD_MANUAL) == 0) ||
        (strncmp(topic, "ws/gw001/cmd/manual/", 20U) == 0))
        {
        uint8_t dev_from_topic = 0U;

        if (strncmp(topic, "ws/gw001/cmd/manual/", 20U) == 0)
        {
            const char *suffix_dev = topic + 20U;
            if ((suffix_dev != NULL) && (suffix_dev[0] != '\0'))
            {
                strncpy(dev, suffix_dev, sizeof(dev) - 1U);
                dev[sizeof(dev) - 1U] = '\0';
                dev_from_topic = 1U;
            }
        }

        if ((dev_from_topic == 0U) && !JsonParse_GetString(json, "dev", dev, sizeof(dev)))
        {
            (void)App_MqttPublishAck("manual", 0U, "missing", NULL);
            return;
        }
        if (!JsonParse_GetString(json, "mode", mode, sizeof(mode)))
        {
            (void)App_MqttPublishAck("manual", 0U, "missing", NULL);
            return;
        }

        manual_dev = App_ParseManualDevice(dev);
        if (manual_dev == APP_MANUAL_DEV_INVALID)
        {
            (void)App_MqttPublishAck("manual", 0U, "range", NULL);
            return;
        }

        if (strcmp(mode, "auto") == 0)
        {
            CMD_LOG("[CMD][MANUAL] dev=%s mode=auto req=%s\r\n",
                    dev,
                    (s_ack_req_id_valid != 0U) ? s_ack_req_id : "-");
            Logic_RestoreDeviceAuto((uint8_t)(manual_dev - 1));  /* 恢复自动模式 */
            snprintf(extra, sizeof(extra), "\"dev\":\"%s\",\"mode\":\"auto\"", dev);
            (void)App_MqttPublishAck("manual", 1U, NULL, extra);
            return;
        }

        if (strcmp(mode, "manual") != 0)
        {
            (void)App_MqttPublishAck("manual", 0U, "range", NULL);
            return;
        }

        if (!JsonParse_GetInt(json, "sec", &sec) ||
            !JsonParse_GetInt(json, "value", &value))
            {
            (void)App_MqttPublishAck("manual", 0U, "missing", NULL);
            return;
        }
        if ((sec < 0) || (sec > 86400))
        {
            (void)App_MqttPublishAck("manual", 0U, "range", NULL);
            return;
        }

        CMD_LOG("[CMD][MANUAL] dev=%s mode=manual sec=%d value=%d req=%s\r\n",
                dev,
                sec,
                value,
                (s_ack_req_id_valid != 0U) ? s_ack_req_id : "-");

        if (manual_dev == APP_MANUAL_DEV_FAN)
        {
            /* 风扇: 手动模式 value 为档位 (1~5) */
            if ((value < 1) || (value > 5))
            {
                (void)App_MqttPublishAck("manual", 0U, "range", NULL);
                return;
            }
        }
        else
        {
            /* 其他设备: 手动模式 value 为开关 (0/1) */
            if ((value != 0) && (value != 1))
            {
                (void)App_MqttPublishAck("manual", 0U, "range", NULL);
                return;
            }
        }

        Logic_SetDeviceManual((uint8_t)(manual_dev - 1), (uint32_t)sec, (uint8_t)value);  /* 设置手动控制 */

        snprintf(extra, sizeof(extra), "\"dev\":\"%s\",\"mode\":\"manual\",\"sec\":%d,\"value\":%d",
                 dev, sec, value);
        (void)App_MqttPublishAck("manual", 1U, NULL, extra);
        return;
    }

    /* ==== QUERY 命令: 查询传感器/状态/阈值 ====
     * what 参数: "all"、"sensors"、"status"、"thresholds"、"change1"~"change7"
     * 查询有频率限制，避免短时间内大量查询冲击
     */
    if (strcmp(topic, ESP01S_MQTT_CMD_QUERY) == 0)
    {
        query_need_ack = (s_ack_req_id_valid != 0U) ? 1U : 0U;
        if (!JsonParse_GetString(json, "what", what, sizeof(what)))
            strcpy(what, "all");

        CMD_LOG("[CMD][QUERY] what=%s req=%s\r\n",
                what,
                (s_ack_req_id_valid != 0U) ? s_ack_req_id : "-");

        if (App_ShouldSkipQueryPublish() != 0U)
        {
            /* 频率限制: 跳过所有查询发布 */
        }
        else if (strcmp(what, "all") == 0)
        {
            (void)App_MqttPublishStatus();
            (void)App_MqttPublishTelemetry();
        }
        else if (strcmp(what, "sensors") == 0)
        {
            (void)App_MqttPublishTelemetry();
        }
        else if (strcmp(what, "status") == 0)
        {
            (void)App_MqttPublishStatus();
        }
        else if (strcmp(what, "thresholds") == 0)
        {
            (void)App_MqttPublishAllThresholds();
        }
        else if (strncmp(what, "change", 6) == 0 &&
                   what[6] >= '1' && what[6] <= '7' && what[7] == '\0')
        {
            (void)App_MqttPublishThresholdGroup((uint8_t)(what[6] - '0'));
        }
        else
        {
            if (query_need_ack != 0U)
            {
                (void)App_MqttPublishAck("query", 0U, "range", NULL);
            }
            return;
        }

        if (query_need_ack != 0U)
        {
            snprintf(extra, sizeof(extra), "\"what\":\"%s\"", what);
            (void)App_MqttPublishAck("query", 1U, NULL, extra);
        }
        return;
    }
}

/* ================================================================
 *  UDP 处理函数
 *  处理来自 ESP32 子节点的 UDP 数据和网关发现协议
 * ================================================================ */

/**
 * @brief 处理接收到的 UDP 数据包
 *
 * 功能:
 *   1. 尝试解析无线传感器 JSON 数据（更新传感器数据汇总）
 *   2. 检测 gateway_discover 广播，回复 gateway_announce
 *
 * @param payload     UDP 负载字符串
 * @param payload_len 负载长度
 * @param src_ip      源 IP 地址字符串
 * @param src_port    源端口号
 */
static void App_HandleUdpPacket(const char *payload,
                                uint16_t payload_len,
                                const char *src_ip,
                                uint16_t src_port)
{
    char offer[128];
    int offer_len;
    uint8_t update_ok;
    TickType_t now_tick;
    TickType_t min_gap_ticks;

    if ((payload == NULL) || (src_ip == NULL))
    {
        return;
    }

    ESP01S_SetConnected(1U);
    App_EspMarkAlive();

    ESP_LOG("[ESP][UDP] %s:%u len=%u payload=%s\r\n",
           src_ip,
           src_port,
           payload_len,
           payload);

    /* 尝试解析为无线传感器 JSON 数据（ESP32 子节点上报） */
    update_ok = AllSensorData_UpdateFromWirelessJson(payload);
    if (update_ok == 0U)
    {
        ESP_LOG("[ESP][UDP] ignore non-sensor json\r\n");
    }

    /* 检查是否为 gateway_discover 广播，若是则回复 gateway_announce */
    if ((strstr(payload, "gateway_discover") == NULL) ||
        (src_ip[0] == '\0') ||
        (src_port == 0U))
        {
        return;
    }

    now_tick = xTaskGetTickCount();
    min_gap_ticks = SafeMsToTicks(APP_DISCOVER_REPLY_MIN_GAP_MS);
    /* 去重: 同一 IP:PORT 在最小间隔内不重复回复 */
    if ((s_last_discover_reply_port == src_port) &&
        (strcmp(s_last_discover_reply_ip, src_ip) == 0) &&
        ((now_tick - s_last_discover_reply_tick) < min_gap_ticks))
        {
        return;
    }

    /* 构建 gateway_announce 响应 JSON */
    offer_len = snprintf(offer,
                         sizeof(offer),
                         "{\"type\":\"gateway_announce\",\"id\":\"%s\",\"port\":%u}\n",
                         ESP01S_GATEWAY_ID,
                         ESP01S_LOCAL_PORT);
    if ((offer_len <= 0) || (offer_len >= (int)sizeof(offer)))
    {
        return;
    }

    if (ESP01S_SendUdpTo(0U, src_ip, src_port, offer, (uint16_t)offer_len))
    {
        s_last_discover_reply_tick = now_tick;
        s_last_discover_reply_port = src_port;
        strncpy(s_last_discover_reply_ip, src_ip, sizeof(s_last_discover_reply_ip) - 1U);
        s_last_discover_reply_ip[sizeof(s_last_discover_reply_ip) - 1U] = '\0';
        ESP_LOG("[ESP][DISCOVER] announce sent to %s:%u\r\n", src_ip, src_port);
    }
    else
    {
        ESP_LOG("[ESP][DISCOVER] announce failed\r\n");
    }
}

/* ================================================================
 *  MQTT 主任务函数（公共接口 —— 由 main.c 创建）
 *  
 *  任务主循环流程:
 *    1. ESP01S 运行时初始化（WiFi 连接、UDP 监听）
 *    2. MQTT TCP 连接 + CONNECT + SUBSCRIBE
 *    3. 轮询循环:
 *       a. 发送延迟 ACK / 排空动作事件 (pre-poll)
 *       b. WiFi 链路状态检查
 *       c. MQTT 重连（若断开）
 *       d. MQTT 消息轮询 + 命令分发 (batch-defer 模式)
 *       e. 后置 ACK flush
 *       f. UDP 数据包轮询
 *       g. MQTT PING 心跳
 *       h. 周期遥测发布（传感器/状态/阈值）
 *       i. 性能日志 + 任务延时
 * ================================================================ */
void MqttApp_Task(void *arg)
{
    uint32_t loop_start_ms;
    uint32_t loop_end_ms;
    uint32_t loop_cost_ms;
    uint32_t last_perf_loop_log_ms = 0U;
    uint8_t mqtt_frame_count;
    uint8_t ipd_frame_count;
    uint8_t published_in_loop;
    char udp_payload[320];
    char src_ip[20];
    uint16_t payload_len;
    uint16_t src_port;
    uint8_t link_id;
    uint8_t loop_count;
    uint8_t handled_any;
    TickType_t last_link_check = 0U;
    TickType_t last_ping_tick = 0U;
    TickType_t next_mqtt_retry_tick = 0U;
    TickType_t last_pub_tick = 0U;
    TickType_t last_status_pub_tick = 0U;
    TickType_t last_threshold_pub_tick = 0U;
    TickType_t last_any_pub_tick = 0U;
    ESP01S_MqttEvent_t evt;
    uint8_t wifi_check_fail_count = 0U;
    uint32_t silent_ms = 0U;
    uint8_t cmd_quiet_ready;
    TickType_t now_tick;
    TickType_t last_cmd_tick;
    TickType_t cmd_quiet_ticks;

    (void)arg;

    /* ---- 任务初始化: 延时2秒等待硬件稳定，初始化 UART 和状态变量 ---- */
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP01S_UART_Init();            /* 初始化 ESP01S 串口 */
    ESP01S_SetConnected(0U);
    s_esp_runtime_ready = 0U;
    s_mqtt_ready = 0U;
    s_mqtt_pub_fail_count = 0U;
#if (APP_ENABLE_ACTION_TELEMETRY != 0U)
    s_action_pending_valid = 0U;
    memset(&s_action_pending, 0, sizeof(s_action_pending));
    s_last_action_tx_tick = 0U;
#endif
    s_esp_last_alive_tick = xTaskGetTickCount();
    s_last_cmd_rx_tick = 0U;
    s_last_query_tick = 0U;
    App_ClearDeferredAckQueue();           /* 清空延迟 ACK 队列 */
    memset(s_req_ack_cache, 0, sizeof(s_req_ack_cache));  /* 清空去重缓存 */
    s_req_ack_cache_next = 0U;

    /* ==================== 主循环开始 ==================== */
    for (;;)
    {
        loop_start_ms = App_GetMsNow();
        mqtt_frame_count = 0U;
        ipd_frame_count = 0U;
        published_in_loop = 0U;

        /* ---- 步骤 1: ESP01S 运行时初始化（WiFi 连接 + UDP 监听） ---- */
        if (s_esp_runtime_ready == 0U)
        {
            ESP_LOG("[ESP] runtime init start\r\n");
            if (!ESP01S_RuntimeInit(ESP01S_WIFI_SSID, ESP01S_WIFI_PASSWORD, ESP01S_LOCAL_PORT))
            {
                ESP01S_SetConnected(0U);
                ESP_LOG("[ESP] runtime init fail, retry\r\n");
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }

            s_esp_runtime_ready = 1U;
            s_mqtt_ready = 0U;
            s_mqtt_pub_fail_count = 0U;
            ESP01S_SetConnected(1U);
            App_EspMarkAlive();
            MqttApp_PollRelayLink();      /* 初始化时立即轮询一次继电器 */
            wifi_check_fail_count = 0U;
            last_link_check = xTaskGetTickCount();
            last_ping_tick = xTaskGetTickCount();
            next_mqtt_retry_tick = xTaskGetTickCount();
            last_pub_tick = xTaskGetTickCount();
            last_status_pub_tick = xTaskGetTickCount();
            last_threshold_pub_tick = xTaskGetTickCount();
            ESP_LOG("[ESP] runtime init OK, UDP listen=%u\r\n", ESP01S_LOCAL_PORT);
        }

#if ESP01S_MQTT_ENABLE
        /* ---- 步骤 2a: pre-poll 发送之前延迟没发的 ACK（最多1条，避免阻塞） ---- */
        if ((s_mqtt_ready != 0U) && (s_ack_deferred_count > 0U))
        {
            if (!App_TryFlushDeferredAck())
            {
                /* 单次失败可容忍，post-poll flush 会重试 */
            }
            else
            {
                published_in_loop = 1U;
                last_any_pub_tick = xTaskGetTickCount();
            }
        }

        /* ---- 步骤 2b: pre-poll 排空动作事件（优先级低于 ACK） ---- */
        if ((published_in_loop == 0U) && (App_MqttDrainActionEvents() != 0U))
        {
            published_in_loop = 1U;
            last_any_pub_tick = xTaskGetTickCount();
        }
#endif

        /* ---- 步骤 3: 周期性 WiFi 链路状态检查 ---- */
        if ((xTaskGetTickCount() - last_link_check) >= pdMS_TO_TICKS(ESP01S_LINK_CHECK_INTERVAL_MS))
        {
            silent_ms = App_EspSilentMs();

            if (s_mqtt_ready != 0U)
            {
                /* MQTT 已连接，直接标记在线 */
                ESP01S_SetConnected(1U);
                wifi_check_fail_count = 0U;
            }
            else
            {
                /* MQTT 未连接，检查 WiFi 状态 */
                if (silent_ms >= 3000U)
                {
                    if (ESP01S_CheckWifiConnected())
                    {
                        ESP01S_SetConnected(1U);
                        wifi_check_fail_count = 0U;
                    }
                    else
                    {
                        if (wifi_check_fail_count < 0xFFU)
                        {
                            wifi_check_fail_count++;
                        }

                        if (wifi_check_fail_count >= ESP01S_WIFI_FAIL_REINIT_COUNT)
                        {
                            ESP_LOG("[ESP] wifi check failed x%u, re-init\r\n",
                                   (unsigned int)wifi_check_fail_count);
                            ESP01S_SetConnected(0U);
                            s_mqtt_ready = 0U;
                            s_mqtt_pub_fail_count = 0U;
                            s_esp_runtime_ready = 0U;
                            vTaskDelay(pdMS_TO_TICKS(1000U));
                            continue;   //重头开始循环，重新初始化 ESP01S 运行时
                        }
                    }
                }

                if (silent_ms > ESP01S_LINK_STALE_REINIT_MS)
                {
                    /* 链路超时: ESP 无响应超过 20 秒，强制重新初始化 */
                    ESP_LOG("[ESP] link stale %lu ms, force re-init\r\n", (unsigned long)silent_ms);
                    ESP01S_SetConnected(0U);
                    s_mqtt_ready = 0U;
                    s_mqtt_pub_fail_count = 0U;
                    s_esp_runtime_ready = 0U;
                    vTaskDelay(pdMS_TO_TICKS(1000U));
                    continue;
                }
            }

            last_link_check = xTaskGetTickCount();
        }

#if ESP01S_MQTT_ENABLE
        /* ---- 步骤 4: MQTT 连接/重连 ----
         * MQTT 未连接时，按顺序执行:
         *   1. 打开 TCP 连接
         *   2. 发送 MQTT CONNECT
         *   3. 订阅命令主题 cmd/#
         *   4. 订阅时间主题 tele/time
         */
        if ((s_mqtt_ready == 0U) &&
            ((int32_t)(xTaskGetTickCount() - next_mqtt_retry_tick) >= 0))
            {
            uint8_t mqtt_setup_ok = 0U;
            PERF_LOG("[PERF][MQTT] open TCP %s:%u\r\n", ESP01S_MQTT_BROKER_IP, ESP01S_MQTT_BROKER_PORT);
            do
            {
                if (!ESP01S_OpenTcp(ESP01S_MQTT_LINK_ID, ESP01S_MQTT_BROKER_IP, ESP01S_MQTT_BROKER_PORT))
                {
                    PERF_LOG("[PERF][MQTT] TCP open failed err=%u(%s)\r\n",
                             (unsigned int)ESP01S_GetLastErrorCode(), ESP01S_GetLastErrorText());
                    break;
                }
                if (!ESP01S_MqttConnect(ESP01S_MQTT_LINK_ID, ESP01S_MQTT_CLIENT_ID, 1U, ESP01S_MQTT_KEEPALIVE_S))
                {
                    PERF_LOG("[PERF][MQTT] CONNECT failed err=%u(%s)\r\n",
                             (unsigned int)ESP01S_GetLastErrorCode(), ESP01S_GetLastErrorText());
                    break;
                }
                if (!ESP01S_MqttSubscribe(ESP01S_MQTT_LINK_ID, ESP01S_MQTT_SUB_TOPIC))
                {
                    PERF_LOG("[PERF][MQTT] SUBSCRIBE cmd failed err=%u(%s)\r\n",
                             (unsigned int)ESP01S_GetLastErrorCode(), ESP01S_GetLastErrorText());
                    break;
                }
                if (!ESP01S_MqttSubscribe(ESP01S_MQTT_LINK_ID, ESP01S_MQTT_TIME_TOPIC))
                {
                    PERF_LOG("[PERF][MQTT] SUBSCRIBE time failed err=%u(%s)\r\n",
                             (unsigned int)ESP01S_GetLastErrorCode(), ESP01S_GetLastErrorText());
                    break;
                }
                mqtt_setup_ok = 1U;
            }
            while (0);

            if (mqtt_setup_ok == 0U)
            {
                /* 连接失败，计算下次重试时间（粗略随机化退避） */
                next_mqtt_retry_tick = xTaskGetTickCount() + SafeMsToTicks(App_GetReconnectDelayMs());
            }
            else
            {
                /* MQTT 连接成功，初始化发布计时器 */
                PERF_LOG("[PERF][MQTT] CONNECT+SUBSCRIBE OK: %s, %s\r\n",
                         ESP01S_MQTT_SUB_TOPIC,
                         ESP01S_MQTT_TIME_TOPIC);
                s_mqtt_ready = 1U;
                s_mqtt_pub_fail_count = 0U;
                ESP01S_SetConnected(1U);
                App_EspMarkAlive();
                last_ping_tick = xTaskGetTickCount();
                last_pub_tick = xTaskGetTickCount() - pdMS_TO_TICKS(ESP01S_MQTT_PUB_INTERVAL_MS);
                last_status_pub_tick = xTaskGetTickCount() - pdMS_TO_TICKS(ESP01S_MQTT_STATUS_INTERVAL_MS);
                last_threshold_pub_tick = xTaskGetTickCount();
                last_any_pub_tick = xTaskGetTickCount();
                s_last_query_tick = 0U;
            }
        }

        /* ---- 步骤 5: 轮询 MQTT 消息 ----
         * poll 循环内的 ACK 全部延迟，避免 ESP 串口阻塞导致
         * 消息排队溢出。poll 结束后统一 flush */
        s_ack_defer_only = 1U;

        handled_any = 0U;
        /* 最多轮询16帧 MQTT 消息，避免单次轮询耗时过长 */
        for (loop_count = 0U; loop_count < 16U; loop_count++)
        {
            if (!ESP01S_MqttPoll(ESP01S_MQTT_LINK_ID, &evt))
            {
                break;
            }
            handled_any = 1U;
            mqtt_frame_count++;    /* 记录本轮主循环里实际处理了多少帧 MQTT 报文 */
            s_mqtt_ready = 1U;
            ESP01S_SetConnected(1U);
            App_EspMarkAlive();
            if (evt.type == ESP01S_MQTT_PKT_PUBLISH)
            {
                /* 收到 PUBLISH 消息，分发到命令处理器 */
                if (strncmp(evt.topic, "ws/gw001/cmd/", 13U) == 0)
                {
                    CMD_LOG("[CMD][MQTT][PUBLISH] topic=%s qos=%u len=%u deferred=%u\r\n",
                            evt.topic,
                            (unsigned int)evt.qos,
                            (unsigned int)evt.payload_len,
                            (unsigned int)s_ack_deferred_count);
                }
                App_MqttOnMessage(evt.topic, evt.payload, evt.payload_len);
            }
        }

        s_ack_defer_only = 0U;    /* 退出 batch-defer 模式 */
        if (mqtt_frame_count > 0U)
        {
            CMD_LOG("[CMD][POLL] frames=%u deferred=%u\r\n",
                    (unsigned int)mqtt_frame_count,
                    (unsigned int)s_ack_deferred_count);
        }

        /* ---- 步骤 6: 后置 ACK flush ----
         * 最多发送2条延迟 ACK，然后返回 poll 以及时消费新消息。
         * 连续失败2次则强制重连 */
        if ((s_mqtt_ready != 0U) && (s_ack_deferred_count > 0U))
        {
            uint8_t ack_flush_i;
            uint8_t ack_flush_fail = 0U;
            for (ack_flush_i = 0U; ack_flush_i < 2U; ack_flush_i++)
            {
                if (s_ack_deferred_count == 0U) break;
                if (s_mqtt_ready == 0U) break;
                if (App_TryFlushDeferredAck())
                {
                    published_in_loop = 1U;
                    last_any_pub_tick = xTaskGetTickCount();
                    ack_flush_fail = 0U;
                }
                else
                {
                    ack_flush_fail++;
                    if (ack_flush_fail >= 2U)
                    {
                        CMD_LOG("[CMD][ACK][POST_FLUSH_GIVEUP] %u fails, force reconnect\r\n",
                                (unsigned int)ack_flush_fail);
                        s_mqtt_ready = 0U;
                        break;
                    }
                }
            }
            if (ack_flush_i > 0U)
            {
                CMD_LOG("[CMD][ACK][FLUSH] sent=%u remain=%u\r\n",
                        (unsigned int)ack_flush_i,
                        (unsigned int)s_ack_deferred_count);
            }
        }
#endif

        /* ---- 步骤 7: UDP 数据包轮询（最多24帧） ---- */
        for (loop_count = 0U; loop_count < 24U; loop_count++)
        {
            payload_len = 0U;
            src_port = 0U;
            link_id = 0U;
            src_ip[0] = '\0';

            if (!ESP01S_TryReadIpd(udp_payload,
                                   sizeof(udp_payload),
                                   &payload_len,
                                   &link_id,
                                   src_ip,
                                   sizeof(src_ip),
                                   &src_port))
                                   {
                break;
            }
            handled_any = 1U;
            ipd_frame_count++;
            ESP01S_SetConnected(1U);
            App_EspMarkAlive();
            if (link_id == 0U)
            {
                /* link_id=0: UDP 数据包（无线传感器/发现协议） */
                App_HandleUdpPacket(udp_payload, payload_len, src_ip, src_port);
            }
            else if (link_id == ESP01S_MQTT_LINK_ID)
            {
                ESP_LOG("[MQTT] frame bypassed parser, len=%u\r\n", payload_len);
            }
            else
            {
                ESP_LOG("[ESP][IPD] other link=%u len=%u\r\n", link_id, payload_len);
            }
        }

#if ESP01S_MQTT_ENABLE
        /* ---- 步骤 8: MQTT PING 心跳 ----
         * 每 30 秒发送一次 PING，但如果最近有数据交互（空闲<8秒）则跳过。
         * PING 失败则标记断开、触发重连 */
        if ((s_mqtt_ready != 0U) &&
            ((xTaskGetTickCount() - last_ping_tick) >= pdMS_TO_TICKS(ESP01S_MQTT_PING_INTERVAL_MS)))
            {
            silent_ms = App_EspSilentMs();
            if (silent_ms < ESP01S_PING_IDLE_MIN_MS)
            {
                /* 最近有数据交互，跳过本次 PING */
                last_ping_tick = xTaskGetTickCount();
            }
            else if (!ESP01S_MqttPing(ESP01S_MQTT_LINK_ID))
            {
                s_mqtt_ready = 0U;
                s_mqtt_pub_fail_count = 0U;
                PERF_LOG("[PERF][MQTT] PING failed, reconnect err=%u(%s)\r\n",
                         (unsigned int)ESP01S_GetLastErrorCode(),
                         ESP01S_GetLastErrorText());
                next_mqtt_retry_tick = xTaskGetTickCount() + pdMS_TO_TICKS(1000U);
            }
            else
            {
                ESP01S_SetConnected(1U);
                App_EspMarkAlive();
                last_ping_tick = xTaskGetTickCount();
            }
        }
#endif

        /* ---- 步骤 9: 周期遥测发布 ----
         * 条件: MQTT 已连接 + 本轮未发布 + 延迟队列为空 +
         *       命令静默期已过 + 距上次发布超过最小间隔
         * 优先级: 传感器 > 状态 > 阈值 */
        now_tick = xTaskGetTickCount();
        cmd_quiet_ticks = SafeMsToTicks(APP_CMD_QUIET_PUB_GAP_MS);
        taskENTER_CRITICAL();
        last_cmd_tick = s_last_cmd_rx_tick;
        taskEXIT_CRITICAL();
        cmd_quiet_ready = ((last_cmd_tick == 0U) || ((now_tick - last_cmd_tick) >= cmd_quiet_ticks)) ? 1U : 0U;

        if ((s_mqtt_ready != 0U) && (published_in_loop == 0U) &&
            (s_ack_deferred_count == 0U) &&
            (cmd_quiet_ready != 0U) &&
            ((xTaskGetTickCount() - last_any_pub_tick) >= SafeMsToTicks(ESP01S_MQTT_PUB_GAP_MS)))
            {
            uint8_t pub_done = 0U;
            now_tick = xTaskGetTickCount();
            if ((now_tick - last_pub_tick) >= SafeMsToTicks(ESP01S_MQTT_PUB_INTERVAL_MS))
            {
                (void)App_MqttPublishTelemetry();
                last_pub_tick = now_tick;
                pub_done = 1U;
            }
            else if ((now_tick - last_status_pub_tick) >= SafeMsToTicks(ESP01S_MQTT_STATUS_INTERVAL_MS))
            {
                (void)App_MqttPublishStatus();
                last_status_pub_tick = now_tick;
                pub_done = 1U;
            }
            else if ((now_tick - last_threshold_pub_tick) >= SafeMsToTicks(APP_THRESHOLDS_PUB_INTERVAL_S * 1000U))
            {
                (void)App_MqttPublishAllThresholds();
                last_threshold_pub_tick = now_tick;
                pub_done = 1U;
            }
            if (pub_done != 0U)
            {
                last_any_pub_tick = xTaskGetTickCount();
                published_in_loop = 1U;
            }
        }

        loop_end_ms = App_GetMsNow();
        loop_cost_ms = (loop_end_ms >= loop_start_ms) ? (loop_end_ms - loop_start_ms) : 0U;
        if ((published_in_loop != 0U) ||
            (loop_cost_ms >= 100U) ||
            (((loop_end_ms - last_perf_loop_log_ms) >= 2000U) &&
             ((mqtt_frame_count != 0U) || (ipd_frame_count != 0U))))
             {
            PERF_LOG("[PERF][LOOP] start=%lu end=%lu dt=%lu handled=%u mqtt=%u ipd=%u ready=%u conn=%u\r\n",
                     (unsigned long)loop_start_ms,
                     (unsigned long)loop_end_ms,
                     (unsigned long)loop_cost_ms,
                     (unsigned int)handled_any,
                     (unsigned int)mqtt_frame_count,
                     (unsigned int)ipd_frame_count,
                     (unsigned int)s_mqtt_ready,
                     (unsigned int)ESP01S_IsConnected());
            last_perf_loop_log_ms = loop_end_ms;
        }

        /* ---- 步骤 10: 任务延时 ---- */
        if (handled_any != 0U)
        {
            vTaskDelay(1);  /* 处理了数据: 最小延时 1 tick，让出 CPU 给低优先级任务 (LED, Screen) */
            continue;
        }

        vTaskDelay(2);  /* 空闲: 2 tick 延时，确保 LED 任务平滑运行 */
    }
}
