/**
 * @file    esp01s.c
 * @brief   ESP01S WiFi/MQTT 驱动模块实现
 *
 * 本文件实现了 STM32F407 环境保障系统中 ESP01S WiFi 模组的全部驱动功能：
 *
 * 【AT 指令层】
 *   - AT 指令收发与响应等待（支持多关键字匹配、超时、busy/ERROR 检测）
 *   - 波特率自动探测与回退扫描机制
 *   - 透传模式退出（+++）
 *
 * 【网络传输层】
 *   - WiFi STA 连接管理（CWMODE/CWJAP/CWAUTOCONN）
 *   - 多路连接管理（CIPMUX=1，link0=UDP 监听，link1=MQTT TCP）
 *   - 环形 RX 缓存（2KB）+ +IPD 数据包缓冲队列（12 条）
 *   - UDP 发送（支持带目标地址和基本语法两种模式）
 *
 * 【MQTT 协议层】
 *   - 软件 MQTT 3.1.1 客户端（不依赖 AT+MQTT，在 TCP 链路上手动编解码报文）
 *   - CONNECT/SUBSCRIBE/PUBLISH/PINGREQ 报文编码
 *   - CONNACK/SUBACK/PUBLISH/PINGRESP 报文解码
 *   - 递归互斥锁保护 MQTT 发送操作
 *   - Remaining Length 变长编码/解码
 *
 * 【连接状态层】
 *   - WiFi 连接状态管理与查询
 *   - Web 前端在线检测（90 秒超时机制）
 *   - 错误码管理与描述文本
 *
 * 硬件连接：USART2 (PA2-TX / PA3-RX)，115200bps，8N1
 */

#include "esp01s.h"

#include "uart2_esp01s.h"  /* UART2 底层收发接口 */
#include "delay.h"         /* 阻塞延时（调度器未启动时使用） */
#include "FreeRTOS.h"      /* FreeRTOS 内核 */
#include "task.h"           /* 任务管理与延时 */
#include "semphr.h"         /* 信号量 / 互斥锁 */

#include <stdio.h>
#include <string.h>

/* ======================== 调试日志开关 ======================== */

#define ESP01S_DEBUG_LOG_ENABLE 0U   /**< 1=开启调试日志输出（通过 printf），0=关闭 */
#if ESP01S_DEBUG_LOG_ENABLE
#define ESP01S_LOG(...) printf(__VA_ARGS__)
#else
#define ESP01S_LOG(...) do { } while (0)  /**< 空操作，不产生任何代码 */
#endif

/* ======================== 内部缓冲区大小常量 ======================== */

#define ESP01S_RX_CACHE_SIZE    2048U  /**< 环形 RX 缓存大小（字节），缓存 UART 接收的原始数据 */
#define ESP01S_MQTT_TX_BUF_SIZE 768U   /**< MQTT 发送缓冲区大小（字节），用于编码 MQTT 报文 */
#define ESP01S_MQTT_RX_BUF_SIZE 768U   /**< MQTT 接收缓冲区大小（字节），用于暂存接收到的报文 */
#define ESP01S_PENDING_IPD_COUNT 12U   /**< IPD 缓冲队列最大条数（不同 link 的数据包暂存） */
#define ESP01S_PENDING_IPD_PAYLOAD_SIZE 512U  /**< 单个 IPD 缓冲条目的最大载荷大小 */
#define ESP01S_PENDING_IPD_SRCIP_SIZE 20U     /**< 单个 IPD 缓冲条目的源 IP 字符串最大长度 */
#define ESP01S_WEB_ONLINE_TIMEOUT_MS  90000U  /**< Web 前端在线判断超时（90 秒无心跳视为离线） */
#define ESP01S_PROMPT_TIMEOUT_MS      400U    /**< 等待 AT+CIPSEND 返回 ">" 提示符的超时时间 */

/* ======================== 错误码定义 ======================== */

#define ESP01S_ERR_NONE                    0U  /**< 无错误 */
#define ESP01S_ERR_INVALID_ARG             1U  /**< 参数无效 */
#define ESP01S_ERR_MUTEX_CREATE_FAIL       2U  /**< 互斥锁创建失败 */
#define ESP01S_ERR_TX_LOCK_TIMEOUT         3U  /**< 获取发送互斥锁超时 */
#define ESP01S_ERR_CIPSEND_PROMPT_TIMEOUT  4U  /**< 等待 CIPSEND ">" 提示符超时 */
#define ESP01S_ERR_SEND_OK_TIMEOUT         5U  /**< 等待 "SEND OK" 超时 */
#define ESP01S_ERR_MQTT_RL_ENCODE_FAIL     6U  /**< MQTT Remaining Length 编码失败 */
#define ESP01S_ERR_MQTT_FRAME_TOO_LARGE    7U  /**< MQTT 帧超过发送缓冲区大小 */
#define ESP01S_ERR_BUSY_RESPONSE           8U  /**< ESP 返回 busy 响应 */
#define ESP01S_ERR_LINK_INVALID            9U  /**< 链路无效或已关闭 */

/* ======================== 静态变量定义 ======================== */

static char s_rx_cache[ESP01S_RX_CACHE_SIZE];       /**< RX 环形缓存，存储从 UART2 读入的原始数据 */
static uint16_t s_rx_len = 0U;                       /**< RX 缓存中当前有效数据长度 */
static volatile uint8_t s_connected = 0U;            /**< WiFi 连接状态标志（由外部任务设置） */
static volatile uint8_t s_web_alive_valid = 0U;      /**< Web 前端心跳是否有效（至少收到过一次心跳） */
static volatile TickType_t s_web_alive_tick = 0U;    /**< Web 前端最后一次心跳的 tick 时间戳 */
static uint16_t s_mqtt_packet_id = 1U;               /**< MQTT 报文标识符自增计数器（SUBSCRIBE 等使用） */
static uint8_t s_mqtt_tx_buf[ESP01S_MQTT_TX_BUF_SIZE]; /**< MQTT 发送缓冲区（编码后的报文） */
static uint8_t s_mqtt_rx_buf[ESP01S_MQTT_RX_BUF_SIZE]; /**< MQTT 接收缓冲区（从 +IPD 提取的报文） */
static SemaphoreHandle_t s_mqtt_tx_mutex = NULL;     /**< MQTT 发送递归互斥锁（保护多任务并发发送） */
static uint8_t s_runtime_init_fail_count = 0U;       /**< 连续初始化失败计数（>8 时触发全波特率扫描） */
static volatile uint16_t s_last_error_code = ESP01S_ERR_NONE; /**< 最近一次操作的错误码 */

/**
 * @brief IPD 缓冲条目结构体
 *
 * 当 MQTT 轮询时收到属于其他 link_id 的 +IPD 数据包时，
 * 将其暂存到 pending 队列中，等待对应链路的读取操作取走。
 */
typedef struct
{
    uint8_t link_id;                                  /**< 该数据包所属的 ESP 连接 ID */
    uint16_t src_port;                                /**< 数据来源端口号 */
    uint16_t payload_len;                             /**< 载荷实际长度 */
    char src_ip[ESP01S_PENDING_IPD_SRCIP_SIZE];       /**< 数据来源 IP 地址字符串 */
    uint8_t payload[ESP01S_PENDING_IPD_PAYLOAD_SIZE]; /**< 载荷数据 */
} ESP01S_PendingIpd_t;

static ESP01S_PendingIpd_t s_pending_ipd[ESP01S_PENDING_IPD_COUNT]; /**< IPD 缓冲队列数组 */
static uint8_t s_pending_ipd_count = 0U;  /**< 当前缓冲队列中的条目数 */

/**
 * @brief 设置最近一次操作的错误码
 * @param err 错误码（ESP01S_ERR_xxx）
 */
static void ESP01S_SetLastError(uint16_t err)
{
    s_last_error_code = err;
}

/**
 * @brief 延时指定毫秒数（自动适配调度器状态）
 *
 * 若 FreeRTOS 调度器已启动，使用 vTaskDelay() 让出 CPU；
 * 否则使用阻塞式 Delay_ms()。
 *
 * @param ms 延时时间（毫秒，0 时立即返回）
 */
static void ESP01S_SleepMs(uint32_t ms)
{
    TickType_t ticks;

    if (ms == 0U)
    {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        ticks = pdMS_TO_TICKS(ms);
        if (ticks == 0U)
        {
            ticks = 1U;  /* 至少延时 1 个 tick */
        }
        vTaskDelay(ticks);
    }
    else
    {
        Delay_ms(ms);  /* 调度器未启动，使用硬件延时 */
    }
}

/**
 * @brief 获取 MQTT 发送递归互斥锁
 *
 * 使用递归互斥锁以支持同一任务内嵌套调用（如 MqttConnect 内部调用 SendLinkData）。
 * 首次调用时自动创建互斥锁；调度器未启动时直接返回成功。
 *
 * @param timeout_ms 等待超时（毫秒）
 * @return 1=获取成功，0=超时或创建失败
 */
static uint8_t ESP01S_MqttTxLock(uint32_t timeout_ms)
{
    TickType_t ticks;

    /* 首次调用时懒初始化递归互斥锁 */
    if (s_mqtt_tx_mutex == NULL)
    {
        s_mqtt_tx_mutex = xSemaphoreCreateRecursiveMutex();
        if (s_mqtt_tx_mutex == NULL)
        {
            ESP01S_SetLastError(ESP01S_ERR_MUTEX_CREATE_FAIL);
            return 0U;
        }
    }

    /* 调度器未启动时无需锁，直接放行 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return 1U;
    }

    ticks = pdMS_TO_TICKS(timeout_ms);
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    if (xSemaphoreTakeRecursive(s_mqtt_tx_mutex, ticks) == pdTRUE)
    {
        return 1U;
    }

    ESP01S_SetLastError(ESP01S_ERR_TX_LOCK_TIMEOUT);
    return 0U;
}

/**
 * @brief 释放 MQTT 发送递归互斥锁
 */
static void ESP01S_MqttTxUnlock(void)
{
    if (s_mqtt_tx_mutex == NULL)
    {
        return;
    }

    /* 调度器未启动时不需要释放 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    (void)xSemaphoreGiveRecursive(s_mqtt_tx_mutex);
}

/**
 * @brief 向 RX 环形缓存追加一个字节
 *
 * 若缓存未满，直接追加；若已满，将整个缓存左移 1 字节（丢弃最老数据），
 * 然后将新字节放到末尾，保证缓存始终以 '\0' 结尾。
 *
 * @param ch 要追加的字节
 */
static void ESP01S_AppendByte(char ch)
{
    if (s_rx_len < (ESP01S_RX_CACHE_SIZE - 1U))
    {
        /* 缓存未满，直接追加 */
        s_rx_cache[s_rx_len++] = ch;
    }
    else
    {
        /* 缓存已满，整体左移一个字节（环形覆盖最旧数据） */
        memmove(s_rx_cache, s_rx_cache + 1, ESP01S_RX_CACHE_SIZE - 2U);
        s_rx_cache[ESP01S_RX_CACHE_SIZE - 2U] = ch;
        s_rx_len = ESP01S_RX_CACHE_SIZE - 1U;
    }

    s_rx_cache[s_rx_len] = '\0';  /* 始终保持 C 字符串结尾 */
}

/**
 * @brief 从 UART2 硬件 FIFO 读取所有可用字节到 RX 缓存
 *
 * 循环调用 UART2_ReadByte() 直到 FIFO 为空，
 * 每个字节通过 ESP01S_AppendByte() 存入 s_rx_cache。
 */
static void ESP01S_FetchRx(void)
{
    uint8_t ch;

    while (UART2_ReadByte(&ch))
    {
        ESP01S_AppendByte((char)ch);
    }
}

/**
 * @brief 丢弃 RX 缓存前 N 个字节（左移剩余数据）
 *
 * 用于在解析完 +IPD 数据或旧数据后释放缓存空间。
 *
 * @param count 要丢弃的字节数
 */
static void ESP01S_DropPrefix(uint16_t count)
{
    if (count == 0U)
    {
        return;
    }

    if (count >= s_rx_len)
    {
        /* 丢弃全部数据 */
        s_rx_len = 0U;
        s_rx_cache[0] = '\0';
        return;
    }

    /* 将剩余数据左移到缓存开头 */
    memmove(s_rx_cache, s_rx_cache + count, s_rx_len - count);
    s_rx_len = (uint16_t)(s_rx_len - count);
    s_rx_cache[s_rx_len] = '\0';
}

/* 前向声明：带起始位置的多关键字等待函数 */
static uint8_t ESP01S_WaitForAnyAfter(const char *key1,
                                      const char *key2,
                                      const char *key3,
                                      uint32_t timeout_ms,
                                      uint16_t start_pos);

/**
 * @brief 等待 RX 缓存中出现任一关键字（从缓存起始位置搜索）
 *
 * 包装 ESP01S_WaitForAnyAfter()，start_pos=0。
 *
 * @param key1       关键字 1（可为 NULL）
 * @param key2       关键字 2（可为 NULL）
 * @param key3       关键字 3（可为 NULL）
 * @param timeout_ms 超时时间（毫秒）
 * @return 0=超时/错误，1=匹配 key1，2=匹配 key2，3=匹配 key3
 */
static uint8_t ESP01S_WaitForAny(const char *key1, const char *key2, const char *key3, uint32_t timeout_ms)
{
    return ESP01S_WaitForAnyAfter(key1, key2, key3, timeout_ms, 0U);
}

/**
 * @brief 将字符串解析为无符号 32 位整数
 *
 * 仅支持纯数字字符串，不支持前导空格、负号或十六进制。
 *
 * @param s    待解析的字符串
 * @param out  [out] 解析结果
 * @return 1=解析成功，0=输入无效
 */
static uint8_t ESP01S_ParseU32(const char *s, uint32_t *out)
{
    uint32_t v = 0U;
    uint8_t has_digit = 0U;
    char ch;

    if ((s == NULL) || (out == NULL) || (*s == '\0'))
    {
        return 0U;
    }

    while ((ch = *s++) != '\0')
    {
        if ((ch < '0') || (ch > '9'))
        {
            return 0U;
        }
        has_digit = 1U;
        v = (v * 10U) + (uint32_t)(ch - '0');
    }

    if (!has_digit)
    {
        return 0U;
    }

    *out = v;
    return 1U;
}

/**
 * @brief 获取 RX 缓存中从指定位置开始的字符串视图
 *
 * 用于在等待函数中仅搜索新接收的数据（跳过旧数据）。
 *
 * @param start_pos 起始位置偏移
 * @return 指向 s_rx_cache + start_pos 的指针，或空字符串
 */
static const char *ESP01S_RxFrom(uint16_t start_pos)
{
    if (start_pos >= s_rx_len)
    {
        return "";
    }
    return &s_rx_cache[start_pos];
}

/*
 * NOTE:
 * s_rx_cache stores RAW UART bytes, including '\0' bytes from +IPD payload.
 * MQTT packets are binary, so we must not use C-string search functions on
 * s_rx_cache (strstr/strchr/strlen etc.).
 *
 * Helpers below provide binary-safe searching within [0, s_rx_len).
 */
static int16_t ESP01S_RxFindBytesAfter(const void *needle,
                                      uint16_t needle_len,
                                      uint16_t start_pos)
{
    uint16_t i;

    if ((needle == NULL) || (needle_len == 0U))
    {
        return -1;
    }
    if (start_pos >= s_rx_len)
    {
        return -1;
    }

    for (i = start_pos; (uint32_t)i + (uint32_t)needle_len <= (uint32_t)s_rx_len; i++)
    {
        if (memcmp((const void *)&s_rx_cache[i], needle, needle_len) == 0)
        {
            return (int16_t)i;
        }
    }

    return -1;
}

static int16_t ESP01S_RxFindStrAfter(const char *needle, uint16_t start_pos)
{
    size_t needle_len;

    if (needle == NULL)
    {
        return -1;
    }

    needle_len = strlen(needle);
    if ((needle_len == 0U) || (needle_len > 0xFFFFU))
    {
        return -1;
    }

    return ESP01S_RxFindBytesAfter((const void *)needle, (uint16_t)needle_len, start_pos);
}

static uint8_t ESP01S_RxHasStrAfter(const char *needle, uint16_t start_pos)
{
    return (ESP01S_RxFindStrAfter(needle, start_pos) >= 0) ? 1U : 0U;
}

static uint8_t ESP01S_RxStartsWithStrAfter(const char *prefix, uint16_t start_pos)
{
    size_t prefix_len;

    if (prefix == NULL)
    {
        return 0U;
    }

    prefix_len = strlen(prefix);
    if ((prefix_len == 0U) || (prefix_len > 0xFFFFU))
    {
        return 0U;
    }
    if ((uint32_t)start_pos + (uint32_t)prefix_len > (uint32_t)s_rx_len)
    {
        return 0U;
    }

    return (memcmp((const void *)&s_rx_cache[start_pos], (const void *)prefix, prefix_len) == 0)
               ? 1U
               : 0U;
}

static int16_t ESP01S_RxFindByteAfter(char ch, uint16_t start_pos)
{
    uint16_t i;

    if (start_pos >= s_rx_len)
    {
        return -1;
    }

    for (i = start_pos; i < s_rx_len; i++)
    {
        if (s_rx_cache[i] == ch)
        {
            return (int16_t)i;
        }
    }

    return -1;
}

/**
 * @brief 带起始位置的多关键字等待函数（核心 AT 响应等待逻辑）
 *
 * 在超时时间内循环轮询 RX 缓存，检测以下内容：
 *   - 失败检测："ERROR"、"FAIL"、"busy"、"CLOSED"、"link not valid"
 *   - 成功检测：依次匹配 key1、key2、key3
 *
 * @param key1       期望关键字 1（可为 NULL）
 * @param key2       期望关键字 2（可为 NULL）
 * @param key3       期望关键字 3（可为 NULL）
 * @param timeout_ms 超时时间（毫秒）
 * @param start_pos  从 RX 缓存的该位置开始搜索（跳过旧数据）
 * @return 0=超时或检测到错误，1=匹配 key1，2=匹配 key2，3=匹配 key3
 */
static uint8_t ESP01S_WaitForAnyAfter(const char *key1,
                                      const char *key2,
                                      const char *key3,
                                      uint32_t timeout_ms,
                                      uint16_t start_pos)
{
    TickType_t start_tick;
    TickType_t timeout_ticks;

    start_tick = xTaskGetTickCount();
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0U) timeout_ticks = 1U;

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        ESP01S_FetchRx();  /* 从 UART FIFO 拉取新数据 */

        /* --- 失败检测：优先检查错误响应 --- */
        if (ESP01S_RxHasStrAfter("ERROR", start_pos) ||
            ESP01S_RxHasStrAfter("FAIL", start_pos))
            {
            ESP01S_SetLastError(ESP01S_ERR_LINK_INVALID);
            return 0U;
        }
        /* 检查 busy 响应（ESP 模组正在处理上一条指令） */
        if (ESP01S_RxHasStrAfter("\r\nbusy ", start_pos) ||
            ESP01S_RxStartsWithStrAfter("busy ", start_pos) ||
            ESP01S_RxHasStrAfter("\r\nBUSY ", start_pos) ||
            ESP01S_RxStartsWithStrAfter("BUSY ", start_pos))
            {
            ESP01S_SetLastError(ESP01S_ERR_BUSY_RESPONSE);
            return 0U;
        }
        /* 检查链路关闭 / 无效状态 */
        if (ESP01S_RxHasStrAfter("link is not valid", start_pos) ||
            ESP01S_RxHasStrAfter("link not valid", start_pos) ||
            ESP01S_RxHasStrAfter("CLOSED", start_pos))
            {
            ESP01S_SetLastError(ESP01S_ERR_LINK_INVALID);
            return 0U;
        }

        /* --- 成功检测：按优先级匹配关键字 --- */
        if (ESP01S_RxHasStrAfter(key1, start_pos))
        {
            return 1U;  /* 匹配第一个关键字 */
        }
        if (ESP01S_RxHasStrAfter(key2, start_pos))
        {
            return 2U;  /* 匹配第二个关键字 */
        }
        if (ESP01S_RxHasStrAfter(key3, start_pos))
        {
            return 3U;  /* 匹配第三个关键字 */
        }

        ESP01S_SleepMs(2U);  /* 短暂休眠后继续轮询 */
    }

    return 0U;  /* 超时 */
}

/**
 * @brief 轻量级等待函数：仅检测目标字符串和 busy 状态
 *
 * 与 WaitForAnyAfter 不同，本函数不做广泛的 "ERROR"/"FAIL" 匹配，
 * 避免并发 +IPD 数据中偶然包含这些字符串导致误判。
 * 主要用于 AT+CIPSEND 流程中等待 ">" 和 "SEND OK"。
 *
 * @param target     目标字符串（如 ">" 或 "SEND OK"）
 * @param timeout_ms 超时时间（毫秒）
 * @param start_pos  RX 缓存中的搜索起始位置
 * @return 1=找到目标字符串，0=超时或 busy/CLOSED
 */
/* Lightweight wait: only checks for target string and "busy",                    
   ignores ERROR/FAIL/CLOSED to avoid false positives from
   concurrent IPD data in the RX buffer. */
static uint8_t ESP01S_WaitForTarget(const char *target,
                                    uint32_t timeout_ms,
                                    uint16_t start_pos)
{
    TickType_t start_tick;
    TickType_t timeout_ticks;

    start_tick = xTaskGetTickCount();
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0U) timeout_ticks = 1U;

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        ESP01S_FetchRx();

        /* 检查是否找到目标字符串 */
        if (ESP01S_RxHasStrAfter(target, start_pos))
        {
            return 1U;
        }
        /* 仅对 busy 状态失败（ESP 真实返回的忙状态） */
        if (ESP01S_RxHasStrAfter("\r\nbusy ", start_pos) ||
            ESP01S_RxStartsWithStrAfter("busy ", start_pos) ||
            ESP01S_RxHasStrAfter("\r\nBUSY ", start_pos) ||
            ESP01S_RxStartsWithStrAfter("BUSY ", start_pos))
            {
            ESP01S_SetLastError(ESP01S_ERR_BUSY_RESPONSE);
            return 0U;
        }
        /* 仅对明确的 TCP 关闭通知失败 */
        if (ESP01S_RxHasStrAfter("\r\n1,CLOSED", start_pos) ||
            ESP01S_RxHasStrAfter("\r\nCLOSED", start_pos))
            {
            ESP01S_SetLastError(ESP01S_ERR_LINK_INVALID);
            return 0U;
        }
        /* 检查明确的 "SEND FAIL" 响应（ESP 发送失败） */
        if (ESP01S_RxHasStrAfter("SEND FAIL", start_pos))
        {
            ESP01S_SetLastError(ESP01S_ERR_SEND_OK_TIMEOUT);
            return 0U;
        }

        ESP01S_SleepMs(2U);  /* 短暂休眠后继续轮询 */
    }

    return 0U;  /* 超时 */
}

/**
 * @brief 清空 IPD 缓冲队列
 */
static void ESP01S_PendingIpdClear(void)
{
    s_pending_ipd_count = 0U;
}

/**
 * @brief 将一个 +IPD 数据包压入缓冲队列尾部
 *
 * 当 MQTT 轮询时收到属于其他 link_id 的数据时，暂存到这里。
 *
 * @param payload     载荷数据
 * @param payload_len 载荷长度
 * @param link_id     连接 ID
 * @param src_ip      源 IP 地址
 * @param src_port    源端口号
 * @return 1=压入成功，0=队列已满或参数无效
 */
static uint8_t ESP01S_PendingIpdPush(const uint8_t *payload,
                                     uint16_t payload_len,
                                     uint8_t link_id,
                                     const char *src_ip,
                                     uint16_t src_port)
{
    ESP01S_PendingIpd_t *slot;

    if ((payload == NULL) || (payload_len == 0U))
    {
        return 0U;
    }
    if (payload_len > ESP01S_PENDING_IPD_PAYLOAD_SIZE)
    {
        return 0U;
    }
    if (s_pending_ipd_count >= ESP01S_PENDING_IPD_COUNT)
    {
        return 0U;
    }

    slot = &s_pending_ipd[s_pending_ipd_count];
    slot->link_id = link_id;
    slot->src_port = src_port;
    slot->payload_len = payload_len;
    if (src_ip != NULL)
    {
        strncpy(slot->src_ip, src_ip, sizeof(slot->src_ip) - 1U);
        slot->src_ip[sizeof(slot->src_ip) - 1U] = '\0';
    }
    else
    {
        slot->src_ip[0] = '\0';
    }
    memcpy(slot->payload, payload, payload_len);
    s_pending_ipd_count++;

    return 1U;
}

/**
 * @brief 从缓冲队列中按索引弹出一个 IPD 条目
 *
 * 弹出后，队列中后续条目前移填补空位。
 *
 * @param pop_index      要弹出的索引
 * @param payload_out    [out] 载荷输出缓冲区
 * @param payload_buf_size 输出缓冲区大小
 * @param payload_len    [out] 实际载荷长度
 * @param link_id        [out] 连接 ID
 * @param src_ip         [out] 源 IP
 * @param src_ip_size    源 IP 缓冲区大小
 * @param src_port       [out] 源端口
 * @return 1=成功，0=索引无效或缓冲区不足
 */
static uint8_t ESP01S_PendingIpdPopIndex(uint8_t pop_index,
                                         uint8_t *payload_out,
                                         uint16_t payload_buf_size,
                                         uint16_t *payload_len,
                                         uint8_t *link_id,
                                         char *src_ip,
                                         uint16_t src_ip_size,
                                         uint16_t *src_port)
{
    ESP01S_PendingIpd_t pkt;
    uint8_t i;

    if (pop_index >= s_pending_ipd_count)
    {
        return 0U;
    }

    pkt = s_pending_ipd[pop_index];
    for (i = pop_index; (uint16_t)(i + 1U) < s_pending_ipd_count; i++)
    {
        s_pending_ipd[i] = s_pending_ipd[i + 1U];
    }
    s_pending_ipd_count--;

    if (payload_len != NULL)
    {
        *payload_len = pkt.payload_len;
    }
    if (link_id != NULL)
    {
        *link_id = pkt.link_id;
    }
    if (src_port != NULL)
    {
        *src_port = pkt.src_port;
    }
    if ((src_ip != NULL) && (src_ip_size > 0U))
    {
        strncpy(src_ip, pkt.src_ip, src_ip_size - 1U);
        src_ip[src_ip_size - 1U] = '\0';
    }

    if ((payload_out != NULL) && (payload_buf_size > 0U))
    {
        if (pkt.payload_len >= payload_buf_size)
        {
            return 0U;
        }
        memcpy(payload_out, pkt.payload, pkt.payload_len);
        payload_out[pkt.payload_len] = '\0';
    }

    return 1U;
}

/**
 * @brief 从缓冲队列头部弹出一个 IPD 条目（不限 link_id）
 * @return 1=成功，0=队列为空
 */
static uint8_t ESP01S_PendingIpdPopAny(uint8_t *payload_out,
                                       uint16_t payload_buf_size,
                                       uint16_t *payload_len,
                                       uint8_t *link_id,
                                       char *src_ip,
                                       uint16_t src_ip_size,
                                       uint16_t *src_port)
{
    if (s_pending_ipd_count == 0U)
    {
        return 0U;
    }

    return ESP01S_PendingIpdPopIndex(0U,
                                     payload_out,
                                     payload_buf_size,
                                     payload_len,
                                     link_id,
                                     src_ip,
                                     src_ip_size,
                                     src_port);
}

/**
 * @brief 从缓冲队列中弹出指定 link_id 的第一个 IPD 条目
 *
 * 遍历队列，找到第一个匹配 wanted_link_id 的条目并弹出。
 *
 * @param wanted_link_id 期望的连接 ID
 * @return 1=找到并弹出，0=未找到
 */
static uint8_t ESP01S_PendingIpdPopByLink(uint8_t wanted_link_id,
                                          uint8_t *payload_out,
                                          uint16_t payload_buf_size,
                                          uint16_t *payload_len,
                                          uint8_t *link_id,
                                          char *src_ip,
                                          uint16_t src_ip_size,
                                          uint16_t *src_port)
{
    uint8_t i;

    for (i = 0U; i < s_pending_ipd_count; i++)
    {
        if (s_pending_ipd[i].link_id == wanted_link_id)
        {
            return ESP01S_PendingIpdPopIndex(i,
                                             payload_out,
                                             payload_buf_size,
                                             payload_len,
                                             link_id,
                                             src_ip,
                                             src_ip_size,
                                             src_port);
        }
    }

    return 0U;
}

/**
 * @brief 生成下一个 MQTT 报文标识符（Packet Identifier）
 *
 * 自增计数，跳过 0（MQTT 规定 Packet ID 不能为 0）。
 *
 * @return 16 位报文标识符（1~65535）
 */
static uint16_t ESP01S_MqttNextPacketId(void)
{
    uint16_t pid = s_mqtt_packet_id;

    s_mqtt_packet_id++;
    if (s_mqtt_packet_id == 0U)
    {
        s_mqtt_packet_id = 1U;
    }

    if (pid == 0U)
    {
        pid = 1U;
    }
    return pid;
}

/**
 * @brief 编码 MQTT Remaining Length 字段（变长编码）
 *
 * MQTT 协议使用 1~4 字节的变长编码表示剩余长度：
 *   - 每字节低 7 位为有效数据，最高位为续传标志
 *   - 最多可表示 268,435,455 字节
 *
 * @param value  要编码的长度值
 * @param out    [out] 编码结果数组（至少 4 字节）
 * @return 编码使用的字节数（1~4），或 0 表示失败
 */
static uint8_t ESP01S_MqttEncodeRemainingLength(uint32_t value, uint8_t out[4])
{
    uint8_t count = 0U;
    uint8_t byte;

    if (out == NULL)
    {
        return 0U;
    }

    do
    {
        byte = (uint8_t)(value % 128U);   /* 取低 7 位 */
        value /= 128U;
        if (value > 0U)
        {
            byte |= 0x80U;  /* 设置续传标志：后面还有更多字节 */
        }
        out[count++] = byte;
    }
    while ((value > 0U) && (count < 4U));

    if (value > 0U)
    {
        return 0U;  /* 值太大，无法用 4 字节编码 */
    }
    return count;
}

/**
 * @brief 解码 MQTT Remaining Length 字段（变长解码）
 *
 * 从字节流中解析变长编码的剩余长度值。
 *
 * @param in          输入字节流（指向固定头后的第一个字节）
 * @param in_len      输入字节流可用长度
 * @param value       [out] 解码后的长度值
 * @param used_bytes  [out] 解码消耗的字节数
 * @return 1=解码成功，0=数据不足或格式错误
 */
static uint8_t ESP01S_MqttDecodeRemainingLength(const uint8_t *in,
                                                uint16_t in_len,
                                                uint32_t *value,
                                                uint8_t *used_bytes)
{
    uint32_t multiplier = 1U;
    uint32_t v = 0U;
    uint8_t i = 0U;
    uint8_t byte;

    if ((in == NULL) || (value == NULL) || (used_bytes == NULL))
    {
        return 0U;
    }

    while (i < 4U)
    {
        if ((uint16_t)i >= in_len)
        {
            return 0U;  /* 输入数据不足 */
        }
        byte = in[i];
        v += (uint32_t)(byte & 0x7FU) * multiplier;  /* 累加低 7 位有效数据 */
        i++;
        if ((byte & 0x80U) == 0U)
        {
            /* 最高位为 0，表示这是最后一个字节 */
            *value = v;
            *used_bytes = i;
            return 1U;
        }
        multiplier *= 128U;  /* 下一个字节的权重 x128 */
    }

    return 0U;  /* 超过 4 字节仍有续传标志，格式错误 */
}

/**
 * @brief 通过指定 TCP/UDP 链路发送原始数据（AT+CIPSEND 流程）
 *
 * 发送流程：
 *   1. 发送 AT+CIPSEND=<link>,<len>  等待 ">" 提示符
 *   2. 发送实际数据  等待 "SEND OK"
 * AT+CIPSEND 最多重试 2 次以应对瞬时错误。
 *
 * @param link_id 连接 ID
 * @param data    要发送的数据
 * @param len     数据长度
 * @return 1=发送成功，0=失败
 */
static uint8_t ESP01S_SendLinkData(uint8_t link_id, const uint8_t *data, uint16_t len)
{
    char cmd[48];
    uint16_t mark;
    uint8_t attempt;

    if ((data == NULL) || (len == 0U))
    {
        ESP01S_SetLastError(ESP01S_ERR_INVALID_ARG);
        return 0U;
    }

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u",
             (unsigned int)link_id,
             (unsigned int)len);

    /* 重试 AT+CIPSEND 最多 2 次，以应对任何瞬时错误 */
    for (attempt = 0U; attempt < 2U; attempt++)
    {
        /* 排空待处理的 IPD/非请求响应数据，避免误判 */
        ESP01S_FetchRx();
        if (s_rx_len > 512U)
        {
            ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
        }
        mark = s_rx_len;  /* 记录当前位置，仅搜索新接收的响应 */
        ESP01S_SetLastError(ESP01S_ERR_NONE);
        ESP01S_SendATLine(cmd);
        /* 使用轻量级等待：仅检查 ">" 和 busy/CLOSED，
           不做广泛的 ERROR/FAIL 匹配，避免并发 IPD 数据干扰 */
        if (ESP01S_WaitForTarget(">", ESP01S_PROMPT_TIMEOUT_MS, mark))
        {
            break;   /* 收到提示符，可以发送数据了 */
        }
        /* 失败后短暂等待再重试 */
        ESP01S_SleepMs(50U);
    }
    if (attempt >= 2U)
    {
        if (ESP01S_GetLastErrorCode() == ESP01S_ERR_NONE)
            ESP01S_SetLastError(ESP01S_ERR_CIPSEND_PROMPT_TIMEOUT);
        return 0U;
    }

    /* 发送实际数据并等待 "SEND OK" */
    ESP01S_FetchRx();
    if (s_rx_len > 512U)
    {
        ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
    }
    mark = s_rx_len;
    ESP01S_SetLastError(ESP01S_ERR_NONE);
    UART2_SendBuffer(data, len);
    /* 同样使用轻量级等待检查 SEND OK */
    if (!ESP01S_WaitForTarget("SEND OK", ESP01S_SEND_TIMEOUT_MS, mark))
    {
        if (ESP01S_GetLastErrorCode() == ESP01S_ERR_NONE)
        {
            ESP01S_SetLastError(ESP01S_ERR_SEND_OK_TIMEOUT);
        }
        return 0U;
    }

    ESP01S_SetLastError(ESP01S_ERR_NONE);
    return 1U;
}

/**
 * @brief 解析一个完整的 MQTT 报文
 *
 * 根据固定头的 type 字段分发到不同的解析分支：
 *   - type=2: CONNACK（连接确认）
 *   - type=9: SUBACK（订阅确认）
 *   - type=13: PINGRESP（心跳响应）
 *   - type=3: PUBLISH（发布消息）
 *
 * @param link_id     连接 ID（记录到 event_out 中）
 * @param packet      原始 MQTT 报文字节流
 * @param packet_len  报文长度
 * @param event_out   [out] 解析结果
 * @return 1=解析成功，0=格式错误
 */
static uint8_t ESP01S_MqttParsePacket(uint8_t link_id,
                                      const uint8_t *packet,
                                      uint16_t packet_len,
                                      ESP01S_MqttEvent_t *event_out)
{
    uint8_t type;           /* MQTT 报文类型（固定头高 4 位） */
    uint8_t flags;          /* MQTT 报文标志（固定头低 4 位） */
    uint32_t remaining_len; /* 剩余长度 */
    uint8_t rl_bytes;       /* 剩余长度字段占用的字节数 */
    uint16_t pos;           /* 当前解析位置 */
    uint16_t payload_total; /* PUBLISH 载荷总长度 */
    uint16_t topic_len;     /* PUBLISH 主题长度 */
    uint16_t copy_len;      /* 实际复制长度（截断保护） */

    if ((packet == NULL) || (packet_len < 2U) || (event_out == NULL))
    {
        return 0U;
    }

    memset(event_out, 0, sizeof(*event_out));
    event_out->type = ESP01S_MQTT_PKT_NONE;
    event_out->link_id = link_id;

    type = (uint8_t)(packet[0] >> 4);      /* 取固定头高 4 位作为报文类型 */
    flags = (uint8_t)(packet[0] & 0x0FU);   /* 取固定头低 4 位作为标志 */

    /* 解码 Remaining Length 字段 */
    if (!ESP01S_MqttDecodeRemainingLength(&packet[1],
                                          (uint16_t)(packet_len - 1U),
                                          &remaining_len,
                                          &rl_bytes))
    {
        return 0U;
    }

    /* 校验报文总长度是否超出实际接收长度 */
    if ((1U + (uint32_t)rl_bytes + remaining_len) > (uint32_t)packet_len)
    {
        return 0U;
    }

    pos = (uint16_t)(1U + rl_bytes);  /* 跳过固定头，指向可变头/载荷 */

    switch (type)
    {
    case 2U: /* CONNACK —— 连接确认报文 */
        if (remaining_len < 2U)
        {
            return 0U;
        }
        event_out->type = ESP01S_MQTT_PKT_CONNACK;
        event_out->session_present = (uint8_t)(packet[pos] & 0x01U); /* 会话存在标志 */
        event_out->return_code = packet[pos + 1U];  /* 连接返回码（0=成功） */
        return 1U;

    case 9U: /* SUBACK —— 订阅确认报文 */
        if (remaining_len < 3U)
        {
            return 0U;
        }
        event_out->type = ESP01S_MQTT_PKT_SUBACK;
        event_out->packet_id = (uint16_t)(((uint16_t)packet[pos] << 8) | packet[pos + 1U]); /* 报文 ID */
        event_out->return_code = packet[pos + 2U];  /* 订阅返回码（0x80=失败） */
        return 1U;

    case 13U: /* PINGRESP —— 心跳响应报文 */
        if (remaining_len != 0U)
        {
            return 0U;
        }
        event_out->type = ESP01S_MQTT_PKT_PINGRESP;
        return 1U;

    case 3U: /* PUBLISH —— 发布消息报文 */
        if (remaining_len < 2U)
        {
            return 0U;
        }

        event_out->type = ESP01S_MQTT_PKT_PUBLISH;
        event_out->retain = (uint8_t)(flags & 0x01U);          /* 保留标志 */
        event_out->qos = (uint8_t)((flags >> 1) & 0x03U);      /* QoS 等级 */

        /* 解析主题名长度（大端序 2 字节） */
        topic_len = (uint16_t)(((uint16_t)packet[pos] << 8) | packet[pos + 1U]);
        pos = (uint16_t)(pos + 2U);
        if ((uint32_t)pos + (uint32_t)topic_len > (1U + (uint32_t)rl_bytes + remaining_len))
        {
            return 0U;  /* 主题长度超出报文范围 */
        }

        /* 复制主题名（截断保护） */
        copy_len = topic_len;
        if (copy_len >= ESP01S_MQTT_MAX_TOPIC_LEN)
        {
            copy_len = (uint16_t)(ESP01S_MQTT_MAX_TOPIC_LEN - 1U);
        }
        if (copy_len > 0U)
        {
            memcpy(event_out->topic, &packet[pos], copy_len);
        }
        event_out->topic[copy_len] = '\0';
        pos = (uint16_t)(pos + topic_len);

        /* QoS > 0 时，PUBLISH 报文含有 2 字节的 Packet Identifier */
        if (event_out->qos > 0U)
        {
            if ((uint32_t)pos + 2U > (1U + (uint32_t)rl_bytes + remaining_len))
            {
                return 0U;
            }
            event_out->packet_id = (uint16_t)(((uint16_t)packet[pos] << 8) | packet[pos + 1U]);
            pos = (uint16_t)(pos + 2U);
        }

        /* 计算并复制载荷数据 */
        payload_total = (uint16_t)((1U + (uint32_t)rl_bytes + remaining_len) - (uint32_t)pos);
        event_out->payload_len = payload_total;
        copy_len = payload_total;
        if (copy_len > ESP01S_MQTT_MAX_PAYLOAD_LEN)
        {
            copy_len = ESP01S_MQTT_MAX_PAYLOAD_LEN;
        }
        if (copy_len > 0U)
        {
            memcpy(event_out->payload, &packet[pos], copy_len);
        }
        return 1U;

    default:
        /* 其他未识别的报文类型 */
        event_out->type = ESP01S_MQTT_PKT_OTHER;
        return 1U;
    }
}

/**
 * @brief 初始化 ESP01S 使用的 UART 外设（USART2）
 *
 * 以默认波特率初始化 UART2，并清空接收缓存。
 */
void ESP01S_UART_Init(void)
{
    UART2_Init(UART2_DEFAULT_BAUDRATE);
    ESP01S_ClearRx();
}

/**
 * @brief 清空所有接收缓冲区
 *
 * 包括 UART2 硬件 FIFO、内部 RX 环形缓存和 IPD 缓冲队列。
 */
void ESP01S_ClearRx(void)
{
    UART2_ClearRxBuffer();
    s_rx_len = 0U;
    s_rx_cache[0] = '\0';
    ESP01S_PendingIpdClear();
}

/**
 * @brief 发送一条 AT 指令（自动追加 \r\n）
 * @param cmd AT 指令字符串（如 "AT+GMR"）
 */
void ESP01S_SendATLine(const char *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    UART2_SendString(cmd);
    UART2_SendString("\r\n");
}

/**
 * @brief 从内部 RX 缓存中读取原始数据并清除已读部分
 *
 * 读取之前先从 UART FIFO 拉取最新数据。
 * 读取后已读数据从缓存中移除。
 *
 * @param out       输出缓冲区
 * @param out_size  输出缓冲区大小（至少 2 字节）
 * @return 实际读取的字节数，0=无数据
 */
uint16_t ESP01S_ReadRaw(char *out, uint16_t out_size)
{
    uint16_t n;

    if ((out == NULL) || (out_size < 2U))
    {
        return 0U;
    }

    ESP01S_FetchRx();
    if (s_rx_len == 0U)
    {
        return 0U;
    }

    n = s_rx_len;
    if (n > (uint16_t)(out_size - 1U))
    {
        n = (uint16_t)(out_size - 1U);
    }

    memcpy(out, s_rx_cache, n);
    out[n] = '\0';
    ESP01S_DropPrefix(n);
    return n;
}

/**
 * @brief 发送 AT 指令并等待期望的响应关键字
 *
 * 清空 RX 缓存 → 发送指令 → 等待指定关键字。
 *
 * @param cmd         AT 指令字符串
 * @param expected    期望的响应关键字（NULL 时默认 "OK"）
 * @param timeout_ms  超时时间（0 时使用默认值）
 * @return 1=收到期望响应，0=超时或错误
 */
uint8_t ESP01S_SendLineWait(const char *cmd, const char *expected, uint32_t timeout_ms)
{
    if (cmd == NULL)
    {
        return 0U;
    }

    if (expected == NULL)
    {
        expected = "OK";
    }
    if (timeout_ms == 0U)
    {
        timeout_ms = ESP01S_CMD_TIMEOUT_MS;
    }

    ESP01S_ClearRx();
    ESP01S_SendATLine(cmd);
    return (ESP01S_WaitForAny(expected, NULL, NULL, timeout_ms) != 0U) ? 1U : 0U;
}

/**
 * @brief 发送 AT 指令并等待 "OK" 响应
 * @param cmd         AT 指令字符串
 * @param timeout_ms  超时时间（毫秒）
 * @return 1=成功，0=失败
 */
uint8_t ESP01S_SendLineWaitOK(const char *cmd, uint32_t timeout_ms)
{
    return ESP01S_SendLineWait(cmd, "OK", timeout_ms);
}

/**
 * @brief 在 link0 上打开 UDP 监听
 *
 * 先关闭 link0 上可能存在的旧连接，然后发送：
 * AT+CIPSTART=0,"UDP","0.0.0.0",0,<local_port>,2
 * 其中 mode=2 表示接受任意远端地址的数据。
 *
 * @param local_port 本地监听端口号
 * @return 1=成功，0=失败
 */
uint8_t ESP01S_OpenUdpListener(uint16_t local_port)
{
    char cmd[96];

    if (local_port == 0U)
    {
        return 0U;
    }

    ESP01S_ClearRx();
    ESP01S_SendATLine("AT+CIPCLOSE=0");
    (void)ESP01S_WaitForAny("OK", "ERROR", "UNLINK", 3000U);

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=0,\"UDP\",\"0.0.0.0\",0,%u,2",
             (unsigned int)local_port);
    ESP01S_ClearRx();
    ESP01S_SendATLine(cmd);
    if (!ESP01S_WaitForAny("OK", "CONNECT", "ALREADY CONNECTED", ESP01S_UDP_OPEN_TIMEOUT_MS))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 在指定链路上建立 TCP 连接
 *
 * 先关闭旧连接，再发送 AT+CIPSTART=<link>,"TCP","<ip>",<port>。
 * 全程持有 MQTT TX 互斥锁保护。
 *
 * @param link_id 多路连接 ID（0~4）
 * @param ip      目标 IP 地址
 * @param port    目标端口
 * @return 1=连接成功，0=失败
 */
uint8_t ESP01S_OpenTcp(uint8_t link_id, const char *ip, uint16_t port)
{
    char cmd[96];
    uint8_t ok = 0U;

    if ((ip == NULL) || (ip[0] == '\0') || (port == 0U))
    {
        return 0U;
    }
    if (!ESP01S_MqttTxLock(ESP01S_MQTT_TIMEOUT_MS))
    {
        return 0U;
    }

    snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%u", (unsigned int)link_id);
    ESP01S_ClearRx();
    ESP01S_SendATLine(cmd);
    (void)ESP01S_WaitForAny("OK", "ERROR", "UNLINK", 5000U);

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=%u,\"TCP\",\"%s\",%u",
             (unsigned int)link_id,
             ip,
             (unsigned int)port);
    ESP01S_ClearRx();
    ESP01S_SendATLine(cmd);
    if (!ESP01S_WaitForAny("OK", "CONNECT", "ALREADY CONNECTED", ESP01S_MQTT_TIMEOUT_MS))
    {
        goto done;
    }

    ok = 1U;

done:
    ESP01S_MqttTxUnlock();
    return ok;
}

/**
 * @brief 确保 ESP 工作在多路连接模式（CIPMUX=1）
 *
 * 若直接设置失败，尝试恢复路径：
 *   1. 关闭透传模式（CIPMODE=0）
 *   2. 关闭 TCP 服务器（CIPSERVER=0）
 *   3. 关闭所有连接（CIPCLOSE=5）
 *   4. 重试 CIPMUX=1
 *   5. 最后手段：软复位（AT+RST）后重试
 *
 * @return 1=成功设置多路模式，0=失败
 */
static uint8_t ESP01S_EnsureCipMux1(void)
{
    /* 直接尝试设置 CIPMUX=1 */
    if (ESP01S_SendLineWaitOK("AT+CIPMUX=1", ESP01S_CMD_TIMEOUT_MS))
    {
        return 1U;
    }

    /* 恢复路径：依次尝试清除可能阻止 CIPMUX 设置的状态 */
    ESP01S_ClearRx();
    ESP01S_SendATLine("AT+CIPMODE=0");  /* 退出透传模式 */
    (void)ESP01S_WaitForAny("OK", "ERROR", NULL, 5000U);  /* 读走没用的数据 */

    ESP01S_ClearRx();
    ESP01S_SendATLine("AT+CIPSERVER=0");  /* 关闭 TCP 服务器 */
    (void)ESP01S_WaitForAny("OK", "ERROR", "no change", 5000U);  /* 读走没用的数据 */

    ESP01S_ClearRx();
    ESP01S_SendATLine("AT+CIPCLOSE=5");  /* 关闭所有连接 */
    (void)ESP01S_WaitForAny("OK", "ERROR", "UNLINK", 5000U);  /* 读走没用的数据 */

    if (ESP01S_SendLineWaitOK("AT+CIPMUX=1", ESP01S_CMD_TIMEOUT_MS))
    {
        return 1U;
    }

    /* 最后手段：软复位 ESP 并重试 */
    ESP01S_ClearRx();
    ESP01S_SendATLine("AT+RST");
    (void)ESP01S_WaitForAny("ready", "Ready", "OK", 15000U);
    ESP01S_SleepMs(800U);  /* 等待固件完全启动 */

    if (!ESP01S_SendLineWaitOK("AT", 5000U))
    {
        ESP01S_LOG("[ESP][INIT] CIPMUX final retry no AT\r\n");
        return 0U;
    }
    (void)ESP01S_SendLineWaitOK("ATE0", ESP01S_CMD_TIMEOUT_MS);
    if (ESP01S_SendLineWaitOK("AT+CIPMUX=1", ESP01S_CMD_TIMEOUT_MS))
    {
        return 1U;
    }

    ESP01S_LOG("[ESP][INIT] CIPMUX raw=%s\r\n", s_rx_cache);
    return 0U;
}

/**
 * @brief 尝试与 ESP 进行 AT 指令握手
 *
 * 发送 "AT" 并等待 "OK"，多次重试。
 *
 * @param attempts   最大尝试次数
 * @param timeout_ms 每次尝试的超时时间
 * @param gap_ms     每次失败后的等待间隔
 * @return 1=握手成功，0=全部失败
 */
static uint8_t ESP01S_TryAtHandshake(uint8_t attempts, uint32_t timeout_ms, uint32_t gap_ms)
{
    uint8_t i;

    if (attempts == 0U)
    {
        attempts = 1U;
    }

    for (i = 0U; i < attempts; i++)
    {
        if (ESP01S_SendLineWaitOK("AT", timeout_ms))
        {
            return 1U;
        }
        ESP01S_SleepMs(gap_ms);
    }

    return 0U;
}

/**
 * @brief 尝试退出 AT 透传模式
 *
 * AT 透传模式要求在 "+++" 前后保持至少 1 秒的静默期（Guard Time）。
 * 发的 "+++" 不能带 \r\n。
 */
static void ESP01S_TryEscapeTransparent(void)
{
    /* Guard time: 前 1.1 秒静默 */
    ESP01S_SleepMs(1100U);
    UART2_SendString("+++");  /* 退出透传模式的转义序列 */
    /* Guard time: 后 1.1 秒静默 */
    ESP01S_SleepMs(1100U);
    ESP01S_ClearRx();
}

/**
 * @brief 探测 AT 固件（含波特率回退扫描）
 *
 * 探测顺序：
 *   1. 快速路径：在当前波特率下尝试 4 次 AT 握手
 *   2. 尝试退出透传模式（+++）后再握手
 *   3. 重新初始化默认波特率后握手
 *   4. （full_scan=1 时）遍历常见波特率列表（115200/9600/57600/38400）
 *
 * @param full_scan 是否进行全波特率扫描（1=是，0=仅快速探测）
 * @return 1=握手成功，0=所有尝试均失败
 */
static uint8_t ESP01S_ProbeAtWithBaudFallback(uint8_t full_scan)
{
    static const uint32_t k_baud_list[] = {
        115200U,  /* 默认波特率 */
        9600U,    /* 部分旧固件的默认值 */
        57600U,
        38400U
    };
    uint8_t i;

    /* 快速路径：大多数情况下 2 秒内即可成功 */
    if (ESP01S_TryAtHandshake(4U, 800U, 100U))
    {
        return 1U;
    }

    /* 若 ESP 残留在透传模式，尝试退出后再探测 */
    ESP01S_TryEscapeTransparent();
    if (ESP01S_TryAtHandshake(3U, 900U, 120U))
    {
        return 1U;
    }

    /* 确保从项目默认波特率开始回退 */
    UART2_Init(UART2_DEFAULT_BAUDRATE);
    ESP01S_ClearRx();
    ESP01S_SleepMs(80U);
    if (ESP01S_TryAtHandshake(2U, 900U, 120U))
    {
        return 1U;
    }

    if (full_scan == 0U)
    {
        return 0U;  /* 不做全扫描，直接失败返回 */
    }

    /* 全波特率扫描：遍历常见波特率 */
    for (i = 0U; i < (uint8_t)(sizeof(k_baud_list) / sizeof(k_baud_list[0])); i++)
    {
        UART2_Init(k_baud_list[i]);  /* 切换到该波特率 */
        ESP01S_ClearRx();
        ESP01S_SleepMs(80U);

        if (ESP01S_TryAtHandshake(2U, 700U, 80U))
        {
            ESP01S_LOG("[ESP][INIT] AT baud=%lu\r\n", (unsigned long)k_baud_list[i]);
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 运行时初始化 ESP01S 全流程
 *
 * 完整流程：
 *   1. 等待 ESP 固件启动（1s）
 *   2. AT 握手探测（含波特率回退）
 *   3. 关闭回显（ATE0）
 *   4. 检查是否已有 IP 地址（AT+CIFSR）
 *   5. 若无 IP：设置 STA 模式 → 连接 WiFi → 设置自动重连
 *   6. 启用多路连接（CIPMUX=1）
 *   7. 启用 +IPD 源地址信息（CIPDINFO=1）
 *   8. 在 link0 上打开 UDP 监听
 *
 * @param ssid       WiFi SSID
 * @param password   WiFi 密码
 * @param local_port 本地 UDP 监听端口
 * @return 1=初始化成功，0=失败
 */
uint8_t ESP01S_RuntimeInit(const char *ssid, const char *password, uint16_t local_port)
{
    char cmd[160];
    uint8_t has_ip = 0U;
    uint8_t do_full_scan = 0U;

    /* 给 ESP boot ROM 和 AT 固件足够的启动时间 */
    ESP01S_SleepMs(1000U);

    /* 连续失败超过 8 次后触发全波特率扫描 */
    if (s_runtime_init_fail_count >= 8U)
    {
        do_full_scan = 1U;
    }

    /* 第 1 步：AT 指令握手 */
    if (!ESP01S_ProbeAtWithBaudFallback(do_full_scan))
    {
        if (s_runtime_init_fail_count < 0xFFU)
        {
            s_runtime_init_fail_count++;
        }
        ESP01S_LOG("[ESP][INIT] AT fail\r\n");
        return 0U;
    }
    s_runtime_init_fail_count = 0U;  /* 握手成功，重置失败计数 */

    /* 第 2 步：确保使用默认波特率并关闭回显 */
    UART2_Init(UART2_DEFAULT_BAUDRATE);
    ESP01S_ClearRx();
    if (!ESP01S_SendLineWaitOK("ATE0", ESP01S_CMD_TIMEOUT_MS))
    {
        /* Some firmwares may reject ATE0, do not abort init. */
        ESP01S_LOG("[ESP][INIT] ATE0 fail, continue\r\n");
    }

    /* 第 3 步：检查是否已有 IP 地址（可能已自动重连 WiFi） */
    ESP01S_ClearRx();
    ESP01S_SendATLine("AT+CIFSR");
    if (ESP01S_WaitForAny("OK", NULL, NULL, 5000U))
    {
        if (ESP01S_RxHasStrAfter("STAIP,\"", 0U))
        {
            has_ip = 1U;  /* 已有 STA IP 地址，无需重新连接 WiFi */
        }
    }

    /* 第 4 步：若无 IP，执行 WiFi 连接序列 */
    if (!has_ip)
    {
        if ((ssid == NULL) || (password == NULL) || (ssid[0] == '\0') || (password[0] == '\0'))
        {
            ESP01S_LOG("[ESP][INIT] no ssid/password\r\n");
            return 0U;
        }

        /* 设置为 STA 模式 */
        if (!ESP01S_SendLineWaitOK("AT+CWMODE=1", ESP01S_CMD_TIMEOUT_MS))
        {
            ESP01S_LOG("[ESP][INIT] CWMODE fail\r\n");
            return 0U;
        }

        /* 连接 WiFi AP：优先使用 AT+CWJAP_DEF（保存到 Flash），回退到 AT+CWJAP */
        snprintf(cmd, sizeof(cmd), "AT+CWJAP_DEF=\"%s\",\"%s\"", ssid, password);
        if (!ESP01S_SendLineWaitOK(cmd, ESP01S_JOINAP_TIMEOUT_MS))
        {
            /* Fallback for older firmwares that only support AT+CWJAP */
            snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
            if (!ESP01S_SendLineWaitOK(cmd, ESP01S_JOINAP_TIMEOUT_MS))
            {
                ESP01S_LOG("[ESP][INIT] CWJAP fail\r\n");
                return 0U;
            }
        }

        /* 启用掉电自动重连 */
        if (!ESP01S_SendLineWaitOK("AT+CWAUTOCONN=1", ESP01S_CMD_TIMEOUT_MS))
        {
            ESP01S_LOG("[ESP][INIT] CWAUTOCONN fail, continue\r\n");
        }
    }

    /* 第 5 步：启用多路连接模式 */
    if (!ESP01S_EnsureCipMux1())
    {
        ESP01S_LOG("[ESP][INIT] CIPMUX fail\r\n");
        return 0U;
    }

    /* 第 6 步：启用 +IPD 报文中的源 IP/端口信息（可选） */
    if (!ESP01S_SendLineWaitOK("AT+CIPDINFO=1", ESP01S_CMD_TIMEOUT_MS))
    {
        /* CIPDINFO 为可选功能，解析器同时支持有无源地址字段 */
        ESP01S_LOG("[ESP][INIT] CIPDINFO unsupported, continue\r\n");
    }

    /* 第 7 步：打开 UDP 监听端口 */
    if (!ESP01S_OpenUdpListener(local_port))
    {
        ESP01S_LOG("[ESP][INIT] OpenUdpListener fail\r\n");
        return 0U;
    }

    (void)ESP01S_SendLineWaitOK("AT+CIPSTATUS", ESP01S_CMD_TIMEOUT_MS);
    return 1U;
}

/**
 * @brief 通过指定链路向目标地址发送 UDP 数据
 *
 * 发送策略：
 *   1. 优先使用带目标地址的语法：AT+CIPSEND=<link>,<len>,"<ip>",<port>
 *   2. 若失败，回退到基本语法：AT+CIPSEND=<link>,<len>
 * 全程持有 MQTT TX 互斥锁。
 *
 * @param link_id     多路连接 ID
 * @param dst_ip      目标 IP 地址
 * @param dst_port    目标端口
 * @param payload     待发送数据
 * @param payload_len 数据长度
 * @return 1=发送成功，0=失败
 */
uint8_t ESP01S_SendUdpTo(uint8_t link_id,
                         const char *dst_ip,
                         uint16_t dst_port,
                         const char *payload,
                         uint16_t payload_len)
{
    char cmd[96];
    uint8_t got_prompt = 0U;
    uint16_t mark;
    uint8_t ok = 0U;

    if ((dst_ip == NULL) || (payload == NULL) || (payload_len == 0U) || (dst_port == 0U))
    {
        return 0U;
    }
    if (!ESP01S_MqttTxLock(ESP01S_CMD_TIMEOUT_MS))
    {
        return 0U;
    }

    ESP01S_SetLastError(ESP01S_ERR_NONE);

    /* 尝试带目标地址的语法发送 */
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u,\"%s\",%u",
             (unsigned int)link_id,
             (unsigned int)payload_len,
             dst_ip,
             (unsigned int)dst_port);

    /*
       IMPORTANT:
       RX cache is a sliding window buffer (memmove when full). If the cache is
       already near-full, taking mark=s_rx_len can result in mark==s_rx_len
       permanently (view becomes empty) which causes prompt/SEND OK wait timeout.
       Keep the tail part only before taking mark, similar to ESP01S_SendLinkData().
    */
    ESP01S_FetchRx();
    if (s_rx_len > 512U)
    {
        ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
    }
    mark = s_rx_len;
    ESP01S_SendATLine(cmd);
    if (ESP01S_WaitForTarget(">", ESP01S_PROMPT_TIMEOUT_MS, mark))
    {
        got_prompt = 1U;
    }
    else
    {
        /* 回退方案：旧固件仅支持基本语法 AT+CIPSEND=<link>,<len> */
        snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u",
                 (unsigned int)link_id,
                 (unsigned int)payload_len);
        ESP01S_FetchRx();
        if (s_rx_len > 512U)
        {
            ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
        }
        mark = s_rx_len;
        ESP01S_SetLastError(ESP01S_ERR_NONE);
        ESP01S_SendATLine(cmd);
        if (ESP01S_WaitForTarget(">", ESP01S_PROMPT_TIMEOUT_MS, mark))
        {
            got_prompt = 1U;
        }
    }

    if (!got_prompt)
    {
        if (ESP01S_GetLastErrorCode() == ESP01S_ERR_NONE)
        {
            ESP01S_SetLastError(ESP01S_ERR_CIPSEND_PROMPT_TIMEOUT);
        }
        goto done;
    }

    /* 收到提示符后发送实际数据并等待 SEND OK */
    ESP01S_FetchRx();
    if (s_rx_len > 512U)
    {
        ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
    }
    mark = s_rx_len;
    UART2_SendBuffer((const uint8_t *)payload, payload_len);
    if (!ESP01S_WaitForTarget("SEND OK", ESP01S_SEND_TIMEOUT_MS, mark))
    {
        if (ESP01S_GetLastErrorCode() == ESP01S_ERR_NONE)
        {
            ESP01S_SetLastError(ESP01S_ERR_SEND_OK_TIMEOUT);
        }
        goto done;
    }

    ok = 1U;  /* 发送成功 */

done:
    ESP01S_MqttTxUnlock();
    return ok;
}

/**
 * @brief 从 RX 缓存中解析一个 +IPD 数据包（内部实现）
 *
 * +IPD 格式解析：
 *   “+IPD,” 后跟逗号分隔的字段，以 ':' 结尾，然后是二进制载荷。
 *
 * 支持的 token 数量与含义：
 *   - 1 个 token: <len>                      （单路，无源地址）
 *   - 2 个 token: <link_id>,<len>             （多路，无源地址）
 *   - 3 个 token: <len>,<ip>,<port>           （单路+CIPDINFO）
 *   - 4 个 token: <link_id>,<len>,<ip>,<port> （多路+CIPDINFO）
 *
 * @return 1=成功解析，0=无可用数据或格式错误
 */
static uint8_t ESP01S_TryReadIpdFromRx(uint8_t *payload_out,
                                       uint16_t payload_buf_size,
                                       uint16_t *payload_len,
                                       uint8_t *link_id,
                                       char *src_ip,
                                       uint16_t src_ip_size,
                                       uint16_t *src_port)
{
    char *ipd;        /* 指向 "+IPD," 的位置 */
    char *colon;       /* 指向 ':' 的位置 */
    uint16_t head_len; /* +IPD, 到 : 之间的头部长度 */
    char head[96];     /* 头部字符串的临时副本 */
    char *tokens[5];   /* 逗号分隔后的 token 数组 */
    uint8_t token_count = 0U;  /* token 数量 */
    char *p;           /* 解析指针 */
    char *comma;       /* 逗号位置 */
    uint32_t len = 0U; /* 解析出的载荷长度 */
    uint32_t n = 0U;   /* 临时数值 */
    uint16_t consumed; /* +IPD 头部消耗的字节数 */
    uint8_t out_link = 0U;     /* 解析出的连接 ID */
    uint16_t out_port = 0U;    /* 解析出的源端口 */
    const char *out_ip = "";   /* 解析出的源 IP */

    ESP01S_FetchRx();  /* 从 UART FIFO 拉取新数据 */

    /* 在 RX 缓存中搜索 "+IPD," 标记 */
    {
        int16_t pos = ESP01S_RxFindStrAfter("+IPD,", 0U);
        if (pos < 0)
        {
            /* 未找到 +IPD，若缓存过大则丢弃旧数据防止溢出 */
            if (s_rx_len > 768U)
            {
                ESP01S_DropPrefix((uint16_t)(s_rx_len - 256U));
            }
            return 0U;
        }
        ipd = &s_rx_cache[(uint16_t)pos];
    }

    /* 丢弃 +IPD 前面的无关数据 */
    if (ipd > s_rx_cache)
    {
        ESP01S_DropPrefix((uint16_t)(ipd - s_rx_cache));
        ipd = s_rx_cache;
    }

    /* 找到 ':' 分隔符（头部与载荷的分界） */
    {
        int16_t pos = ESP01S_RxFindByteAfter(':', (uint16_t)(ipd - s_rx_cache));
        if (pos < 0)
        {
            return 0U;  /* 数据未接收完整，等待更多数据 */
        }
        colon = &s_rx_cache[(uint16_t)pos];
    }

    /* 提取头部字符串（+IPD, 后到 : 之间的内容） */
    head_len = (uint16_t)(colon - (ipd + 5));
    if ((head_len == 0U) || (head_len >= sizeof(head)))
    {
        ESP01S_DropPrefix(1U);
        return 0U;
    }

    memcpy(head, ipd + 5, head_len);
    head[head_len] = '\0';

    /* 按逗号分割头部字符串为多个 token */
    p = head;
    while ((token_count < 5U) && (p != NULL) && (*p != '\0'))
    {
        tokens[token_count++] = p;
        comma = strchr(p, ',');
        if (comma == NULL)
        {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }

    /* 根据 token 数量解析不同的 +IPD 格式 */
    if (token_count == 1U)
    {
        /* 格式: +IPD,<len>:<payload> —— 单路模式 */
        if (!ESP01S_ParseU32(tokens[0], &len))
        {
            ESP01S_DropPrefix(1U);
            return 0U;
        }
    }
    else if (token_count == 2U)
    {
        /* 格式: +IPD,<id>,<len>:<payload> —— 多路模式 */
        if (!ESP01S_ParseU32(tokens[0], &n) || !ESP01S_ParseU32(tokens[1], &len))
        {
            ESP01S_DropPrefix(1U);
            return 0U;
        }
        out_link = (uint8_t)n;
    }
    else if (token_count == 3U)
    {
        /* 格式: +IPD,<len>,<ip>,<port>:<payload> —— 单路+CIPDINFO */
        if (!ESP01S_ParseU32(tokens[0], &len) || !ESP01S_ParseU32(tokens[2], &n))
        {
            ESP01S_DropPrefix(1U);
            return 0U;
        }
        out_ip = tokens[1];
        out_port = (uint16_t)n;
    }
    else
    {
        /* 格式: +IPD,<id>,<len>,<ip>,<port>:<payload> —— 多路+CIPDINFO */
        uint32_t n_link;
        uint32_t n_port;
        if (!ESP01S_ParseU32(tokens[0], &n_link) ||
            !ESP01S_ParseU32(tokens[1], &len) ||
            !ESP01S_ParseU32(tokens[3], &n_port))
            {
            ESP01S_DropPrefix(1U);
            return 0U;
        }
        out_link = (uint8_t)n_link;
        out_ip = tokens[2];
        out_port = (uint16_t)n_port;
    }

    /* 计算头部消耗的总字节数（从 +IPD, 开头到 : 后的第一个载荷字节） */
    consumed = (uint16_t)((colon + 1) - s_rx_cache);
    /* 检查载荷数据是否已完全接收 */
    if ((uint32_t)(s_rx_len - consumed) < len)
    {
        return 0U;  /* 载荷未接收完整，等待更多数据 */
    }

    /* 复制载荷数据到输出缓冲区 */
    if ((payload_out != NULL) && (payload_buf_size > 0U))
    {
        if (len >= (uint32_t)payload_buf_size)
        {
            /* 载荷超出输出缓冲区，丢弃该包 */
            ESP01S_DropPrefix((uint16_t)(consumed + len));
            return 0U;
        }

        memcpy(payload_out, s_rx_cache + consumed, (size_t)len);
        /* Ensure safe string printing for UDP/JSON path. */
        if (len < (uint32_t)payload_buf_size)
        {
            payload_out[len] = '\0';
        }
    }

    /* 填写输出参数 */
    if (payload_len != NULL)
    {
        *payload_len = (uint16_t)len;
    }
    if (link_id != NULL)
    {
        *link_id = out_link;
    }
    if ((src_ip != NULL) && (src_ip_size > 0U))
    {
        strncpy(src_ip, out_ip, src_ip_size - 1U);
        src_ip[src_ip_size - 1U] = '\0';
    }
    if (src_port != NULL)
    {
        *src_port = out_port;
    }

    /* 从 RX 缓存中移除已解析的整个 +IPD 报文 */
    ESP01S_DropPrefix((uint16_t)(consumed + len));
    return 1U;
}

/**
 * @brief 尝试读取一个 +IPD 数据包（公开接口）
 *
 * 优先从 pending 缓冲队列中取，若为空则从 RX 缓存中解析。
 * 全程持有 MQTT TX 互斥锁保护。
 *
 * @return 1=成功解析一个包，0=无可用数据
 */
uint8_t ESP01S_TryReadIpd(char *payload_out,
                          uint16_t payload_buf_size,
                          uint16_t *payload_len,
                          uint8_t *link_id,
                          char *src_ip,
                          uint16_t src_ip_size,
                          uint16_t *src_port)
{
    uint8_t ok;

    if (!ESP01S_MqttTxLock(50U))
    {
        return 0U;
    }

    /* 优先从 pending 缓冲队列中取（之前 MQTT 轮询时暂存的其他 link 数据） */
    if (ESP01S_PendingIpdPopAny((uint8_t *)payload_out,
                                payload_buf_size,
                                payload_len,
                                link_id,
                                src_ip,
                                src_ip_size,
                                src_port))
                                {
        ESP01S_MqttTxUnlock();
        return 1U;
    }

    /* pending 队列为空，从 RX 缓存中直接解析 */
    ok = ESP01S_TryReadIpdFromRx((uint8_t *)payload_out,
                                 payload_buf_size,
                                 payload_len,
                                 link_id,
                                 src_ip,
                                 src_ip_size,
                                 src_port);
    ESP01S_MqttTxUnlock();
    return ok;
}

/**
 * @brief 轮询指定链路上的 MQTT 报文
 *
 * 工作流程：
 *   1. 优先从 pending 队列中查找匹配 link_id 的数据
 *   2. 若未找到，从 RX 缓存中解析 +IPD
 *   3. 若解析到的数据属于其他 link_id，则压入 pending 队列
 *   4. 最多尝试 6 次（避免死循环）
 *
 * @param link_id    期望的 TCP 连接 ID
 * @param event_out  [out] 解析后的 MQTT 事件
 * @return 1=成功解析到指定链路的报文，0=无可用报文
 */
uint8_t ESP01S_MqttPoll(uint8_t link_id, ESP01S_MqttEvent_t *event_out)
{
    char src_ip[20];
    uint16_t src_port = 0U;
    uint16_t payload_len = 0U;
    uint8_t rx_link = 0U;
    uint8_t try_count;
    uint8_t ok = 0U;

    if (event_out == NULL)
    {
        return 0U;
    }
    if (!ESP01S_MqttTxLock(50U))
    {
        return 0U;
    }

    /* 优先从 pending 队列中查找匹配的 link_id */
    if (ESP01S_PendingIpdPopByLink(link_id,
                                   s_mqtt_rx_buf,
                                   sizeof(s_mqtt_rx_buf),
                                   &payload_len,
                                   &rx_link,
                                   src_ip,
                                   sizeof(src_ip),
                                   &src_port))
                                   {
        ok = ESP01S_MqttParsePacket(rx_link, s_mqtt_rx_buf, payload_len, event_out);
        ESP01S_MqttTxUnlock();
        return ok;
    }

    /* 从 RX 缓存中解析 +IPD，最多尝试 6 次 */
    for (try_count = 0U; try_count < 6U; try_count++)
    {
        if (!ESP01S_TryReadIpdFromRx(s_mqtt_rx_buf,
                                     sizeof(s_mqtt_rx_buf),
                                     &payload_len,
                                     &rx_link,
                                     src_ip,
                                     sizeof(src_ip),
                                     &src_port)) 
                                     {
            ESP01S_MqttTxUnlock();
            return 0U;
        }

        if (rx_link == link_id)
        {
            /* 找到匹配的 link_id，解析 MQTT 报文 */
            ok = ESP01S_MqttParsePacket(rx_link, s_mqtt_rx_buf, payload_len, event_out);
            ESP01S_MqttTxUnlock();
            return ok;
        }

        /* 不匹配的 link_id，暂存到 pending 队列 */
        (void)ESP01S_PendingIpdPush(s_mqtt_rx_buf,
                                    payload_len,
                                    rx_link,
                                    src_ip,
                                    src_port);
    }

    ESP01S_MqttTxUnlock();
    return 0U;
}

/**
 * @brief 等待指定类型的 MQTT 报文（带超时）
 *
 * 在超时时间内循环调用 ESP01S_MqttPoll()，
 * 直到收到期望类型的报文或超时。
 *
 * @param link_id    TCP 连接 ID
 * @param expected   期望的报文类型（如 CONNACK/SUBACK）
 * @param timeout_ms 超时时间（毫秒）
 * @param event_out  [out] 解析结果（可为 NULL）
 * @return 1=收到期望报文，0=超时
 */
static uint8_t ESP01S_MqttWaitEvent(uint8_t link_id,
                                    ESP01S_MqttPacketType_t expected,
                                    uint32_t timeout_ms,
                                    ESP01S_MqttEvent_t *event_out)
{
    TickType_t start_tick;
    TickType_t timeout_ticks;
    ESP01S_MqttEvent_t evt;

    start_tick = xTaskGetTickCount();
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0U) timeout_ticks = 1U;

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        if (ESP01S_MqttPoll(link_id, &evt))
        {
            if (evt.type == expected)
            {
                if (event_out != NULL)
                {
                    *event_out = evt;
                }
                return 1U;
            }
        }

        ESP01S_SleepMs(2U);
    }

    return 0U;
}

/**
 * @brief 发送 MQTT CONNECT 报文并等待 CONNACK
 *
 * MQTT CONNECT 报文结构：
 *   - 固定头: 0x10 + Remaining Length
 *   - 可变头: 协议名"MQTT" + 协议级别(0x04=3.1.1) + 连接标志 + KeepAlive
 *   - 载荷:  Client ID
 *
 * @param link_id           TCP 连接 ID
 * @param client_id         客户端标识（NULL 时默认 "gw001"）
 * @param clean             清除会话标志
 * @param keepalive_seconds 心跳间隔（秒）0 时使用默认值
 * @return 1=连接成功，0=失败
 */
uint8_t ESP01S_MqttConnect(uint8_t link_id,
                           const char *client_id,
                           uint8_t clean,
                           uint16_t keepalive_seconds)
{
    const char *cid = client_id;
    uint16_t cid_len;
    uint8_t rl[4];
    uint8_t rl_len;
    uint32_t remaining_len;
    uint16_t idx = 0U;
    ESP01S_MqttEvent_t connack;

    if ((cid == NULL) || (cid[0] == '\0'))
    {
        cid = "gw001";
    }
    if (!ESP01S_MqttTxLock(ESP01S_MQTT_TIMEOUT_MS))
    {
        return 0U;
    }

    cid_len = (uint16_t)strlen(cid);
    if (cid_len == 0U)
    {
        goto done_fail;
    }

    if (keepalive_seconds == 0U)
    {
        keepalive_seconds = ESP01S_MQTT_KEEPALIVE_S;
    }

    /* Remaining Length = 可变头(10字节) + Client ID 长度前缀(2字节) + Client ID 字符串 */
    remaining_len = 10U + 2U + (uint32_t)cid_len;
    rl_len = ESP01S_MqttEncodeRemainingLength(remaining_len, rl);
    if (rl_len == 0U)
    {
        goto done_fail;
    }

    if ((uint32_t)(1U + rl_len + remaining_len) > (uint32_t)sizeof(s_mqtt_tx_buf))
    {
        goto done_fail;
    }

    s_mqtt_tx_buf[idx++] = 0x10U; /* CONNECT 固定头: 类型=1, 标志=0 */
    memcpy(&s_mqtt_tx_buf[idx], rl, rl_len);
    idx = (uint16_t)(idx + rl_len);

    /* 可变头：协议名称 "MQTT" */
    s_mqtt_tx_buf[idx++] = 0x00U;  /* 协议名长度高字节 */
    s_mqtt_tx_buf[idx++] = 0x04U;  /* 协议名长度低字节 (4) */
    s_mqtt_tx_buf[idx++] = 'M';
    s_mqtt_tx_buf[idx++] = 'Q';
    s_mqtt_tx_buf[idx++] = 'T';
    s_mqtt_tx_buf[idx++] = 'T';
    s_mqtt_tx_buf[idx++] = 0x04U; /* 协议级别: MQTT 3.1.1 */
    s_mqtt_tx_buf[idx++] = (clean != 0U) ? 0x02U : 0x00U;  /* 连接标志: bit1=CleanSession */
    s_mqtt_tx_buf[idx++] = (uint8_t)((keepalive_seconds >> 8) & 0xFFU); /* KeepAlive 高字节 */
    s_mqtt_tx_buf[idx++] = (uint8_t)(keepalive_seconds & 0xFFU);        /* KeepAlive 低字节 */

    /* 载荷：Client ID（UTF-8 编码，前缀 2 字节长度） */
    s_mqtt_tx_buf[idx++] = (uint8_t)((cid_len >> 8) & 0xFFU);  /* Client ID 长度高字节 */
    s_mqtt_tx_buf[idx++] = (uint8_t)(cid_len & 0xFFU);         /* Client ID 长度低字节 */
    memcpy(&s_mqtt_tx_buf[idx], cid, cid_len);
    idx = (uint16_t)(idx + cid_len);

    /* 通过 TCP 链路发送 CONNECT 报文 */
    if (!ESP01S_SendLinkData(link_id, s_mqtt_tx_buf, idx))
    {
        goto done_fail;
    }

    /* 等待 CONNACK 响应 */
    if (!ESP01S_MqttWaitEvent(link_id, ESP01S_MQTT_PKT_CONNACK, ESP01S_MQTT_TIMEOUT_MS, &connack))
    {
        goto done_fail;
    }

    ESP01S_MqttTxUnlock();
    return (connack.return_code == 0U) ? 1U : 0U;  /* return_code=0 表示连接成功 */

done_fail:
    ESP01S_MqttTxUnlock();
    return 0U;
}

/**
 * @brief 发送 MQTT SUBSCRIBE 报文并等待 SUBACK
 *
 * MQTT SUBSCRIBE 报文结构：
 *   - 固定头: 0x82 (type=8, QoS1) + Remaining Length
 *   - Packet ID (2字节)
 *   - Topic Filter (UTF-8) + Requested QoS (1字节)
 *
 * @param link_id       TCP 连接 ID
 * @param topic_filter  订阅主题过滤器（NULL 时默认 "ws/gw001/cmd/#"）
 * @return 1=订阅成功，0=失败
 */
uint8_t ESP01S_MqttSubscribe(uint8_t link_id, const char *topic_filter)
{
    const char *topic = topic_filter;
    uint16_t topic_len;
    uint16_t packet_id;
    uint8_t rl[4];
    uint8_t rl_len;
    uint32_t remaining_len;
    uint16_t idx = 0U;
    ESP01S_MqttEvent_t suback;

    if ((topic == NULL) || (topic[0] == '\0'))
    {
        topic = "ws/gw001/cmd/#";
    }
    if (!ESP01S_MqttTxLock(ESP01S_MQTT_TIMEOUT_MS))
    {
        return 0U;
    }

    topic_len = (uint16_t)strlen(topic);
    if (topic_len == 0U)
    {
        goto done_fail;
    }

    packet_id = ESP01S_MqttNextPacketId();  /* 生成报文标识符 */
    /* Remaining Length = Packet ID(2) + Topic长度前缀(2) + Topic字符串 + QoS(1) */
    remaining_len = 2U + 2U + (uint32_t)topic_len + 1U;
    rl_len = ESP01S_MqttEncodeRemainingLength(remaining_len, rl);
    if (rl_len == 0U)
    {
        goto done_fail;
    }
    if ((uint32_t)(1U + rl_len + remaining_len) > (uint32_t)sizeof(s_mqtt_tx_buf))
    {
        goto done_fail;
    }

    s_mqtt_tx_buf[idx++] = 0x82U; /* SUBSCRIBE 固定头: type=8, QoS1 标志 */
    memcpy(&s_mqtt_tx_buf[idx], rl, rl_len);
    idx = (uint16_t)(idx + rl_len);
    s_mqtt_tx_buf[idx++] = (uint8_t)((packet_id >> 8) & 0xFFU); /* Packet ID 高字节 */
    s_mqtt_tx_buf[idx++] = (uint8_t)(packet_id & 0xFFU);        /* Packet ID 低字节 */
    s_mqtt_tx_buf[idx++] = (uint8_t)((topic_len >> 8) & 0xFFU); /* Topic 长度高字节 */
    s_mqtt_tx_buf[idx++] = (uint8_t)(topic_len & 0xFFU);        /* Topic 长度低字节 */
    memcpy(&s_mqtt_tx_buf[idx], topic, topic_len);  /* Topic 字符串 */
    idx = (uint16_t)(idx + topic_len);
    s_mqtt_tx_buf[idx++] = 0x00U; /* 请求的 QoS 等级: QoS0 */

    /* 通过 TCP 发送 SUBSCRIBE 报文 */
    if (!ESP01S_SendLinkData(link_id, s_mqtt_tx_buf, idx))
    {
        goto done_fail;
    }

    /* 等待 SUBACK 响应 */
    if (!ESP01S_MqttWaitEvent(link_id, ESP01S_MQTT_PKT_SUBACK, ESP01S_MQTT_TIMEOUT_MS, &suback))
    {
        goto done_fail;
    }

    /* 校验 Packet ID 匹配且返回码非 0x80（失败） */
    if ((suback.packet_id != packet_id) || (suback.return_code == 0x80U))
    {
        goto done_fail;
    }

    ESP01S_MqttTxUnlock();
    return 1U;

done_fail:
    ESP01S_MqttTxUnlock();
    return 0U;
}

/**
 * @brief 发送 MQTT PUBLISH 报文（QoS0，无需确认）
 *
 * MQTT PUBLISH 报文结构（QoS0）：
 *   - 固定头: 0x30 (type=3, QoS0, 不保留) + Remaining Length
 *   - Topic 长度(2字节) + Topic 字符串
 *   - Payload（无 Packet ID）
 *
 * @param link_id      TCP 连接 ID
 * @param topic        发布主题（NULL 时默认 "ws/gw001/tele/sensors"）
 * @param payload      载荷数据
 * @param payload_len  载荷长度
 * @return 1=发送成功，0=失败
 */
uint8_t ESP01S_MqttPublish(uint8_t link_id,
                           const char *topic,
                           const uint8_t *payload,
                           uint16_t payload_len)
{
    const char *pub_topic = topic;
    uint16_t topic_len;
    uint8_t rl[4];
    uint8_t rl_len;
    uint32_t remaining_len;
    uint16_t idx = 0U;

    if ((pub_topic == NULL) || (pub_topic[0] == '\0'))
    {
        pub_topic = "ws/gw001/tele/sensors";
    }
    if (payload == NULL)
    {
        ESP01S_SetLastError(ESP01S_ERR_INVALID_ARG);
        return 0U;
    }
    if (!ESP01S_MqttTxLock(ESP01S_MQTT_TIMEOUT_MS))
    {
        return 0U;
    }

    topic_len = (uint16_t)strlen(pub_topic);
    /* Remaining Length = Topic长度(2) + Topic字符串 + Payload */
    remaining_len = 2U + (uint32_t)topic_len + (uint32_t)payload_len;
    rl_len = ESP01S_MqttEncodeRemainingLength(remaining_len, rl);
    if (rl_len == 0U)
    {
        ESP01S_SetLastError(ESP01S_ERR_MQTT_RL_ENCODE_FAIL);
        goto done_fail;
    }
    if ((uint32_t)(1U + rl_len + remaining_len) > (uint32_t)sizeof(s_mqtt_tx_buf))
    {
        ESP01S_SetLastError(ESP01S_ERR_MQTT_FRAME_TOO_LARGE);
        goto done_fail;
    }

    s_mqtt_tx_buf[idx++] = 0x30U; /* PUBLISH 固定头: type=3, QoS0, DUP=0, Retain=0 */
    memcpy(&s_mqtt_tx_buf[idx], rl, rl_len);
    idx = (uint16_t)(idx + rl_len);
    s_mqtt_tx_buf[idx++] = (uint8_t)((topic_len >> 8) & 0xFFU); /* Topic 长度高字节 */
    s_mqtt_tx_buf[idx++] = (uint8_t)(topic_len & 0xFFU);        /* Topic 长度低字节 */
    memcpy(&s_mqtt_tx_buf[idx], pub_topic, topic_len);  /* Topic 字符串 */
    idx = (uint16_t)(idx + topic_len);
    /* 复制载荷数据 */
    if (payload_len > 0U)
    {
        memcpy(&s_mqtt_tx_buf[idx], payload, payload_len);
        idx = (uint16_t)(idx + payload_len);
    }

    /* 通过 TCP 发送 PUBLISH 报文 */
    if (!ESP01S_SendLinkData(link_id, s_mqtt_tx_buf, idx))
    {
        goto done_fail;
    }

    ESP01S_SetLastError(ESP01S_ERR_NONE);
    ESP01S_MqttTxUnlock();
    return 1U;  /* QoS0 无需等待 PUBACK */

done_fail:
    ESP01S_MqttTxUnlock();
    return 0U;
}

/**
 * @brief 发送 MQTT PINGREQ 心跳报文并等待 PINGRESP
 *
 * PINGREQ 固定为 2 字节: {0xC0, 0x00}
 *
 * @param link_id TCP 连接 ID
 * @return 1=收到 PINGRESP，0=超时
 */
uint8_t ESP01S_MqttPing(uint8_t link_id)
{
    ESP01S_MqttEvent_t evt;
    const uint8_t pingreq[2] = {0xC0U, 0x00U};  /* PINGREQ: type=12, remaining_len=0 */

    if (!ESP01S_MqttTxLock(ESP01S_MQTT_TIMEOUT_MS))
    {
        return 0U;
    }

    if (!ESP01S_SendLinkData(link_id, pingreq, sizeof(pingreq)))
    {
        ESP01S_MqttTxUnlock();
        return 0U;
    }

    if (!ESP01S_MqttWaitEvent(link_id, ESP01S_MQTT_PKT_PINGRESP, ESP01S_MQTT_TIMEOUT_MS, &evt))
    {
        ESP01S_MqttTxUnlock();
        return 0U;
    }

    ESP01S_MqttTxUnlock();
    return 1U;
}

/**
 * @brief 上电初始化序列（ESP01S_RuntimeInit 的兼容包装）
 * @param ssid       WiFi SSID
 * @param password   WiFi 密码
 * @param local_port 本地 UDP 监听端口
 * @return 1=成功，0=失败
 */
uint8_t ESP01S_PowerOnSequence(const char *ssid, const char *password, uint16_t local_port)
{
    return ESP01S_RuntimeInit(ssid, password, local_port);
}

/**
 * @brief 通过 AT 指令查询 WiFi 连接状态
 *
 * 优先使用 AT+CWJAP? 判断 STA 关联状态，
 * 若不支持则回退到 AT+CIPSTATUS 检查网络状态。
 *
 * @return 1=已连接到 AP，0=未连接
 */
uint8_t ESP01S_CheckWifiConnected(void)
{
    uint16_t mark;
    const char *view;
    uint8_t connected = 0U;

    /* 优先用 AT+CWJAP? 查询 STA 关联状态 */
    if (!ESP01S_MqttTxLock(ESP01S_MQTT_TIMEOUT_MS + 1000U))
    {
        return 0U;
    }

    ESP01S_FetchRx();
    if (s_rx_len > 512U)
    {
        ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
    }
    mark = s_rx_len;
    ESP01S_SendATLine("AT+CWJAP?");
    if (ESP01S_WaitForAnyAfter("OK", NULL, NULL, 5000U, mark))
    {
        view = ESP01S_RxFrom(mark);
        if (ESP01S_RxHasStrAfter("+CWJAP:\"", mark))
        {
            connected = 1U;  /* 响应中包含已连接的 AP 信息 */
        }
    }

    /*
       NOTE:
       AT+CWJAP? 只能说明 STA 与 AP 关联，并不必然表示“已获取 IP”。
       某些现场问题（例如 DHCP 租约/续租异常）会表现为仍关联但无 IP，
       进而导致网页端/MQTT 无法访问。
       因此当 CWJAP? 判断已关联时，也额外用 CIPSTATUS 验证 STATUS:2/3。
    */
    if (connected != 0U)
    {
        ESP01S_FetchRx();
        if (s_rx_len > 512U)
        {
            ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
        }
        mark = s_rx_len;
        ESP01S_SendATLine("AT+CIPSTATUS");
        if (ESP01S_WaitForAnyAfter("STATUS:", NULL, NULL, 5000U, mark))
        {
            view = ESP01S_RxFrom(mark);
            if ((ESP01S_RxHasStrAfter("STATUS:2", mark) == 0U) &&
                (ESP01S_RxHasStrAfter("STATUS:3", mark) == 0U))
            {
                connected = 0U;
            }
        }
        goto done;
    }

    /* 回退方案：使用 AT+CIPSTATUS 检查网络状态 */
    ESP01S_FetchRx();
    if (s_rx_len > 512U)
    {
        ESP01S_DropPrefix((uint16_t)(s_rx_len - 128U));
    }
    mark = s_rx_len;
    ESP01S_SendATLine("AT+CIPSTATUS");
    if (!ESP01S_WaitForAnyAfter("STATUS:", NULL, NULL, 5000U, mark))
    {
        goto done;
    }

    view = ESP01S_RxFrom(mark);
    /* STATUS:2=已获得IP, STATUS:3=已连接 */
    if (ESP01S_RxHasStrAfter("STATUS:2", mark) ||
        ESP01S_RxHasStrAfter("STATUS:3", mark))
        {
        connected = 1U;
    }

done:
    ESP01S_MqttTxUnlock();
    return connected;
}

/**
 * @brief 设置 WiFi 连接状态标志
 * @param connected 非0=已连接，0=未连接
 */
void ESP01S_SetConnected(uint8_t connected)
{
    s_connected = (connected != 0U) ? 1U : 0U;
}

/**
 * @brief 获取 WiFi 连接状态
 * @return 1=已连接，0=未连接
 */
uint8_t ESP01S_IsConnected(void)
{
    return s_connected;
}

/**
 * @brief 标记 Web 前端活跃（刷新心跳时间戳）
 *
 * 使用临界区保护对 s_web_alive_valid 和 s_web_alive_tick 的读写，
 * 确保与 ESP01S_IsWebConnected() 的并发安全。
 */
void ESP01S_MarkWebAlive(void)
{
    taskENTER_CRITICAL();
    s_web_alive_valid = 1U;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        s_web_alive_tick = xTaskGetTickCount();
    }
    else
    {
        s_web_alive_tick = 0U;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 判断 Web 前端是否在线
 *
 * 若距离上次心跳超过 90 秒，则认为前端已离线。
 *
 * @return 1=在线，0=离线或从未收到心跳
 */
uint8_t ESP01S_IsWebConnected(void)
{
    uint8_t valid;
    TickType_t last_tick;
    TickType_t now_tick;
    TickType_t timeout_ticks;

    taskENTER_CRITICAL();
    valid = s_web_alive_valid;
    last_tick = s_web_alive_tick;
    taskEXIT_CRITICAL();

    if (valid == 0U)
    {
        return 0U;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return 1U;
    }

    timeout_ticks = pdMS_TO_TICKS(ESP01S_WEB_ONLINE_TIMEOUT_MS);
    if (timeout_ticks == 0U)
    {
        timeout_ticks = 1U;
    }

    now_tick = xTaskGetTickCount();
    if ((now_tick - last_tick) <= timeout_ticks)
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 获取最近一次操作的错误码
 * @return 错误码（ESP01S_ERR_xxx，0=无错误）
 */
uint16_t ESP01S_GetLastErrorCode(void)
{
    return s_last_error_code;
}

/**
 * @brief 获取最近一次操作的错误描述文本
 * @return 错误描述字符串（英文）
 */
const char *ESP01S_GetLastErrorText(void)
{
    switch (s_last_error_code)
    {
    case ESP01S_ERR_NONE:
        return "ok";
    case ESP01S_ERR_INVALID_ARG:
        return "invalid_arg";
    case ESP01S_ERR_MUTEX_CREATE_FAIL:
        return "mutex_create_fail";
    case ESP01S_ERR_TX_LOCK_TIMEOUT:
        return "tx_lock_timeout";
    case ESP01S_ERR_CIPSEND_PROMPT_TIMEOUT:
        return "cipsend_prompt_timeout";
    case ESP01S_ERR_SEND_OK_TIMEOUT:
        return "send_ok_timeout";
    case ESP01S_ERR_MQTT_RL_ENCODE_FAIL:
        return "mqtt_remaining_len_encode_fail";
    case ESP01S_ERR_MQTT_FRAME_TOO_LARGE:
        return "mqtt_frame_too_large";
    case ESP01S_ERR_BUSY_RESPONSE:
        return "busy";
    case ESP01S_ERR_LINK_INVALID:
        return "link_invalid_or_closed";
    default:
        return "unknown";
    }
}
