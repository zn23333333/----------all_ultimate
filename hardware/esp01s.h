/**
 * @file    esp01s.h
 * @brief   ESP01S WiFi/MQTT 驱动模块头文件
 *
 * 本模块为 STM32F407 环境保障系统提供 ESP01S WiFi 模组的驱动接口。
 * 通过 USART2（PA2/PA3, 115200bps）使用 AT 指令与 ESP01S 通信，实现：
 *   - WiFi STA 模式连接与管理
 *   - 多路 UDP/TCP 网络通信（CIPMUX=1 模式）
 *   - 软件 MQTT 客户端（在 TCP 链路上手动编解码 MQTT 3.1.1 报文）
 *   - +IPD 数据接收与缓冲队列管理
 *   - Web 前端在线状态检测
 *
 * 依赖：uart2_esp01s（底层 UART 收发）、FreeRTOS（延时与互斥锁）
 */

#ifndef ESP01S_H
#define ESP01S_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 超时常量定义 ======================== */

#define ESP01S_CMD_TIMEOUT_MS        8000U   /**< 通用 AT 指令响应超时（毫秒） */
#define ESP01S_JOINAP_TIMEOUT_MS     30000U  /**< WiFi 连接（AT+CWJAP）超时（毫秒） */
#define ESP01S_UDP_OPEN_TIMEOUT_MS   15000U  /**< UDP 监听端口打开超时（毫秒） */
#define ESP01S_SEND_TIMEOUT_MS       2000U   /**< 数据发送（等待 SEND OK）超时（毫秒） */
#define ESP01S_MQTT_TIMEOUT_MS       8000U   /**< MQTT 操作（连接/订阅/发布）超时（毫秒） */
#define ESP01S_MQTT_KEEPALIVE_S      60U     /**< MQTT 心跳保活间隔（秒） */

/* ======================== MQTT 缓冲区大小 ======================== */

#define ESP01S_MQTT_MAX_TOPIC_LEN    128U    /**< MQTT 主题名最大长度（字节） */
#define ESP01S_MQTT_MAX_PAYLOAD_LEN  384U    /**< MQTT 载荷最大长度（字节） */

/* ======================== MQTT 报文类型枚举 ======================== */

/**
 * @brief MQTT 报文类型枚举
 *
 * 用于标识从 MQTT Broker 接收到的报文类型。
 * 本模块仅在 TCP 链路上手动编解码 MQTT 报文，不使用 AT+MQTT 指令。
 */
typedef enum
{
    ESP01S_MQTT_PKT_NONE = 0U,    /**< 无有效报文 / 初始状态 */
    ESP01S_MQTT_PKT_CONNACK,      /**< CONNACK — 连接确认报文（type=2） */
    ESP01S_MQTT_PKT_SUBACK,       /**< SUBACK  — 订阅确认报文（type=9） */
    ESP01S_MQTT_PKT_PINGRESP,     /**< PINGRESP — 心跳响应报文（type=13） */
    ESP01S_MQTT_PKT_PUBLISH,      /**< PUBLISH  — 服务器下发的发布报文（type=3） */
    ESP01S_MQTT_PKT_OTHER         /**< 其他未识别类型的 MQTT 报文 */
} ESP01S_MqttPacketType_t;

/* ======================== MQTT 事件结构体 ======================== */

/**
 * @brief MQTT 事件结构体
 *
 * 当通过 ESP01S_MqttPoll() 从 TCP 链路接收到一个完整的 MQTT 报文时，
 * 解析结果填充到此结构体中，供上层业务逻辑使用。
 */
typedef struct
{
    ESP01S_MqttPacketType_t type;  /**< 报文类型（CONNACK/SUBACK/PUBLISH 等） */
    uint8_t link_id;               /**< 该报文所属的 ESP 多路连接 ID（0~4） */
    uint8_t session_present;       /**< CONNACK: 会话存在标志（0 或 1） */
    uint8_t return_code;           /**< CONNACK/SUBACK: 返回码（0=成功） */
    uint16_t packet_id;            /**< SUBACK/PUBLISH(QoS>0): 报文标识符 */
    uint8_t qos;                   /**< PUBLISH: QoS 等级（0/1/2） */
    uint8_t retain;                /**< PUBLISH: 保留标志 */
    char topic[ESP01S_MQTT_MAX_TOPIC_LEN];       /**< PUBLISH: 主题名字符串 */
    uint16_t payload_len;                         /**< PUBLISH: 载荷实际长度 */
    uint8_t payload[ESP01S_MQTT_MAX_PAYLOAD_LEN]; /**< PUBLISH: 载荷数据缓冲区 */
} ESP01S_MqttEvent_t;

/* ======================== 底层 UART / AT 指令接口 ======================== */

/**
 * @brief 初始化 ESP01S 使用的 UART 外设（USART2）并清空接收缓存
 */
void ESP01S_UART_Init(void);

/**
 * @brief 清空 UART 接收缓冲区及内部 RX 缓存和 IPD 队列
 */
void ESP01S_ClearRx(void);

/**
 * @brief 发送一条 AT 指令（自动追加 \r\n）
 * @param cmd AT 指令字符串，如 "AT+GMR"
 */
void ESP01S_SendATLine(const char *cmd);

/**
 * @brief 从内部 RX 缓存中读取原始数据并清除已读部分
 * @param out       输出缓冲区
 * @param out_size  输出缓冲区大小
 * @return 实际读取的字节数
 */
uint16_t ESP01S_ReadRaw(char *out, uint16_t out_size);

/**
 * @brief 发送 AT 指令并等待期望的响应关键字
 * @param cmd         AT 指令字符串
 * @param expected    期望的响应关键字（NULL 时默认 "OK"）
 * @param timeout_ms  超时时间（毫秒，0 时使用默认超时）
 * @return 1=收到期望响应，0=超时或出错
 */
uint8_t ESP01S_SendLineWait(const char *cmd, const char *expected, uint32_t timeout_ms);

/**
 * @brief 发送 AT 指令并等待 "OK" 响应
 * @param cmd         AT 指令字符串
 * @param timeout_ms  超时时间（毫秒）
 * @return 1=成功，0=失败
 */
uint8_t ESP01S_SendLineWaitOK(const char *cmd, uint32_t timeout_ms);

/* ======================== WiFi 初始化与连接管理 ======================== */

/**
 * @brief 运行时初始化：AT 握手 → WiFi 连接 → 多路模式 → UDP 监听
 *
 * 完整初始化流程：
 *   1. 探测 AT 固件（含波特率回退扫描）
 *   2. 关闭回显（ATE0）
 *   3. 检查/建立 WiFi STA 连接（CWJAP）
 *   4. 启用多路连接（CIPMUX=1）
 *   5. 启用 +IPD 源地址信息（CIPDINFO=1）
 *   6. 在 link0 上打开 UDP 监听端口
 *
 * @param ssid        WiFi SSID
 * @param password    WiFi 密码
 * @param local_port  本地 UDP 监听端口号
 * @return 1=初始化成功，0=失败
 */
uint8_t ESP01S_RuntimeInit(const char *ssid, const char *password, uint16_t local_port);

/* ======================== UDP / TCP 网络操作 ======================== */

/**
 * @brief 在 link0 上打开 UDP 监听
 *
 * 发送 AT+CIPSTART=0,"UDP","0.0.0.0",0,<local_port>,2
 * mode=2 表示接收任意远端地址的数据。
 *
 * @param local_port 本地监听端口号
 * @return 1=成功，0=失败
 */
uint8_t ESP01S_OpenUdpListener(uint16_t local_port);

/**
 * @brief 在指定链路上建立 TCP 连接
 * @param link_id  多路连接 ID（0~4）
 * @param ip       目标服务器 IP 地址字符串
 * @param port     目标服务器端口号
 * @return 1=连接成功，0=失败
 */
uint8_t ESP01S_OpenTcp(uint8_t link_id, const char *ip, uint16_t port);

/**
 * @brief 通过指定链路向目标地址发送 UDP 数据
 *
 * 优先使用带目标地址的语法：AT+CIPSEND=<link>,<len>,"<dst_ip>",<dst_port>
 * 若失败则回退到基本语法：AT+CIPSEND=<link>,<len>
 *
 * @param link_id      多路连接 ID
 * @param dst_ip       目标 IP 地址字符串
 * @param dst_port     目标端口号
 * @param payload      待发送数据
 * @param payload_len  数据长度
 * @return 1=发送成功，0=失败
 */
uint8_t ESP01S_SendUdpTo(uint8_t link_id,
                         const char *dst_ip,
                         uint16_t dst_port,
                         const char *payload,
                         uint16_t payload_len);

/* ======================== +IPD 数据接收解析 ======================== */

/**
 * @brief 从内部缓冲区中尝试解析一个 +IPD 数据包
 *
 * 支持多种 +IPD 格式：
 *   - +IPD,<len>:<payload>              （单路模式，无源地址）
 *   - +IPD,<id>,<len>:<payload>         （多路模式，无源地址）
 *   - +IPD,<id>,<len>,<ip>,<port>:<payload> （多路模式+CIPDINFO）
 *
 * @param payload_out      载荷输出缓冲区
 * @param payload_buf_size 载荷缓冲区大小
 * @param payload_len      [out] 实际载荷长度
 * @param link_id          [out] 连接 ID
 * @param src_ip           [out] 源 IP 字符串
 * @param src_ip_size      源 IP 缓冲区大小
 * @param src_port         [out] 源端口号
 * @return 1=成功解析一个包，0=无可用数据
 */
uint8_t ESP01S_TryReadIpd(char *payload_out,
                          uint16_t payload_buf_size,
                          uint16_t *payload_len,
                          uint8_t *link_id,
                          char *src_ip,
                          uint16_t src_ip_size,
                          uint16_t *src_port);

/* ======================== MQTT 协议操作 ======================== */

/**
 * @brief 在指定 TCP 链路上发送 MQTT CONNECT 报文并等待 CONNACK
 * @param link_id           TCP 连接 ID
 * @param client_id         客户端标识符（NULL 时默认 "gw001"）
 * @param clean             清除会话标志（非0=清除旧会话）
 * @param keepalive_seconds 心跳保活间隔（秒，0 时使用默认值）
 * @return 1=连接成功（return_code=0），0=失败
 */
uint8_t ESP01S_MqttConnect(uint8_t link_id,
                           const char *client_id,
                           uint8_t clean,
                           uint16_t keepalive_seconds);

/**
 * @brief 在指定链路上发送 MQTT SUBSCRIBE 报文并等待 SUBACK
 * @param link_id       TCP 连接 ID
 * @param topic_filter  订阅主题过滤器（NULL 时默认 "ws/gw001/cmd/#"）
 * @return 1=订阅成功，0=失败
 */
uint8_t ESP01S_MqttSubscribe(uint8_t link_id, const char *topic_filter);

/**
 * @brief 在指定链路上发送 MQTT PUBLISH 报文（QoS0，无需 PUBACK）
 * @param link_id      TCP 连接 ID
 * @param topic        发布主题（NULL 时默认 "ws/gw001/tele/sensors"）
 * @param payload      载荷数据
 * @param payload_len  载荷长度
 * @return 1=发送成功，0=失败
 */
uint8_t ESP01S_MqttPublish(uint8_t link_id,
                           const char *topic,
                           const uint8_t *payload,
                           uint16_t payload_len);

/**
 * @brief 发送 MQTT PINGREQ 心跳报文并等待 PINGRESP
 * @param link_id TCP 连接 ID
 * @return 1=收到 PINGRESP，0=超时
 */
uint8_t ESP01S_MqttPing(uint8_t link_id);

/**
 * @brief 轮询指定链路上的 MQTT 报文（从共享 RX 流中消费一个 +IPD 帧）
 *
 * 若 +IPD 帧属于其他 link_id，则将其暂存到 pending 队列中。
 *
 * @param link_id    期望的 TCP 连接 ID
 * @param event_out  [out] 解析后的 MQTT 事件
 * @return 1=成功解析到指定链路的报文，0=无可用报文
 */
uint8_t ESP01S_MqttPoll(uint8_t link_id, ESP01S_MqttEvent_t *event_out);

/* ======================== 兼容接口与状态管理 ======================== */

/**
 * @brief 上电初始化序列（ESP01S_RuntimeInit 的兼容包装）
 * @param ssid       WiFi SSID
 * @param password   WiFi 密码
 * @param local_port 本地 UDP 监听端口
 * @return 1=成功，0=失败
 */
uint8_t ESP01S_PowerOnSequence(const char *ssid, const char *password, uint16_t local_port);

/**
 * @brief 设置 WiFi 连接状态标志
 * @param connected 非0=已连接，0=未连接
 */
void ESP01S_SetConnected(uint8_t connected);

/**
 * @brief 获取当前 WiFi 连接状态
 * @return 1=已连接，0=未连接
 */
uint8_t ESP01S_IsConnected(void);

/**
 * @brief 通过 AT 指令主动查询 WiFi 是否已连接到 AP
 * @return 1=已关联到 AP，0=未连接
 */
uint8_t ESP01S_CheckWifiConnected(void);

/**
 * @brief 标记 Web 前端活跃（刷新心跳时间戳）
 *
 * 每次收到 Web 前端的请求时调用此函数，用于判断前端是否在线。
 */
void ESP01S_MarkWebAlive(void);

/**
 * @brief 判断 Web 前端是否在线（90秒超时）
 * @return 1=在线，0=离线或超时
 */
uint8_t ESP01S_IsWebConnected(void);

/**
 * @brief 获取最近一次操作的错误码
 * @return 错误码（0=无错误）
 */
uint16_t ESP01S_GetLastErrorCode(void);

/**
 * @brief 获取最近一次操作的错误描述文本
 * @return 错误描述字符串（如 "ok", "busy", "tx_lock_timeout" 等）
 */
const char *ESP01S_GetLastErrorText(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP01S_H */
