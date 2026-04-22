/**
 * @file  mqtt_app.h
 * @brief MQTT 应用层模块头文件
 *
 * 本模块是环境保障系统的核心通信模块，负责 STM32F407 与云端/上位机之间的
 * 数据交互。通过 ESP01S WiFi 模块实现 MQTT 协议通信。
 *
 * 功能概述:
 *   - WiFi / MQTT 连接管理（自动重连、超时检测、PING 心跳保活）
 *   - MQTT 命令处理（阈值修改 cmd/threshold、手动控制 cmd/manual、
 *     查询 cmd/query、继电器 cmd/relay、风扇 cmd/fan、时间同步 tele/time）
 *   - 周期遥测发布（传感器数据 tele/sensors、设备状态 tele/status、
 *     阈值参数 tele/thresholds）
 *   - ACK 确认队列（延迟发送、失败重试、请求去重缓存）
 *   - UDP 设备发现协议（响应 gateway_discover 广播）
 *   - 设备动作事件上报（tele/action，如加热器开关、风扇档位变化等）
 *
 * MQTT Broker: 8.212.157.225:1883
 * 客户端 ID:   gw001
 * 订阅主题:    ws/gw001/cmd/#, ws/gw001/tele/time
 */
#ifndef MQTT_APP_H
#define MQTT_APP_H

#include <stdint.h>

/**
 * @brief MQTT 应用层 FreeRTOS 任务入口函数
 *
 * 该任务负责完整的 WiFi 初始化、MQTT 连接建立、消息轮询收发、
 * 周期遥测数据发布以及命令分发处理。是系统通信的主循环。
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void MqttApp_Task(void *arg);

/**
 * @brief 轮询继电器 Modbus 通信链路状态
 *
 * 由 ControlTask（main.c）周期调用，通过 Modbus 读取继电器模块状态，
 * 更新内部缓存的链路连接标志。连续失败 3 次则标记链路断开。
 */
void MqttApp_PollRelayLink(void);

/**
 * @brief 获取 MQTT 模块内部维护的继电器 Modbus 链路缓存状态
 *
 * 该状态由 MqttApp_PollRelayLink() 周期维护，采用“连续失败多次才判离线”的
 * 去抖策略，适合供屏幕等显示模块直接复用，避免重复占用 Modbus 总线。
 *
 * @return 1=继电器链路在线, 0=继电器链路离线
 */
uint8_t MqttApp_GetRelayLinkCached(void);

/**
 * @brief 获取 MQTT Broker 当前是否处于已连接可用状态
 *
 * 该状态由 MQTT 连接建立、PING 心跳成功/失败以及重连过程共同维护，
 * 用于屏幕或其他模块显示“EMQX/MQTT 服务器在线状态”。
 *
 * @return 1=Broker 已连接并可正常通信, 0=未连接/重连中
 */
uint8_t MqttApp_IsBrokerConnected(void);

#endif /* MQTT_APP_H */
