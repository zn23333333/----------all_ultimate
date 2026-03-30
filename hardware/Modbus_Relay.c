/**
 * @file    Modbus_Relay.c
 * @brief   Modbus RTU 继电器控制与传感器数据采集模块 - 实现文件
 * @details 本文件实现了以下核心功能：
 *          1. 预定义 Modbus RTU 命令表（写继电器、读继电器状态、读传感器等报文模板）
 *          2. Modbus 请求/响应完整流程（发送命令 → 等待响应 → CRC16校验 → 解析数据）
 *          3. 8路继电器独立控制（通过 Modbus 功能码 0x06 写单寄存器）
 *          4. 多种传感器数据读取：
 *             - 温湿度传感器（功能码 0x03，从站地址 0x01）
 *             - PM2.5/PM10 颗粒物传感器（功能码 0x03，从站地址 0x03）
 *             - 燃气传感器（功能码 0x04，从站地址 0x04）
 *          5. OLED 显示各传感器数据（SSD1306 驱动）
 *          6. FreeRTOS 互斥锁保护 Modbus 串口总线的并发访问
 *
 * @note    Modbus 总线上所有设备共享同一个 UART1 串口，
 *          因此必须通过 xSerialSemaphore 互斥锁保证同一时刻只有一个任务使用总线。
 */

#include "Modbus_Relay.h"    /* 模块头文件：类型定义、宏、函数声明 */
#include "uart1_modbus.h"    /* UART1 Modbus 底层驱动：发送/接收/CRC校验 */
#include "OLED.h"            /* SSD1306 OLED 显示驱动 */
#include "delay.h"           /* 延时函数（用于无RTOS环境的阻塞等待） */
#include <string.h>          /* memcpy, memset, memcmp */

/** @brief 全局设备状态实例，保存所有继电器与传感器的最新数据 */
DeviceStatus_t g_device_status = {0};

/** @brief Modbus 串口总线互斥锁句柄，FreeRTOS 环境下保护并发访问 */
SemaphoreHandle_t xSerialSemaphore;

/**
 * @brief Modbus RTU 预定义命令表（不含CRC，CRC由发送函数自动追加）
 * @details 每条命令6字节：[从站地址, 功能码, 寄存器地址高, 寄存器地址低, 数据高, 数据低]
 *          索引顺序与 ModbusCommand_t 枚举一一对应。
 *
 *  继电器控制命令（功能码0x06 写单寄存器）：
 *    寄存器地址 0x0000~0x0007 对应第1~8路继电器
 *    写入 0x0001 = 吸合(ON)，写入 0x0000 = 断开(OFF)
 *
 *  读取命令：
 *    CMD_READ_ALL:    功能码0x03，读寄存器0x0000起始，读8个寄存器
 *    CMD_READ_HUMIDITY_TEMPERATURE: 功能码0x03，读寄存器0x0000起始，读2个寄存器
 *    CMD_READ_PM25_PM10: 功能码0x03，读寄存器0x0004起始，读6个寄存器
 *    CMD_READ_GAS:    功能码0x04，读输入寄存器0x4700起始，读1个寄存器
 */
static const uint8_t s_modbus_commands[][6] = {
    /* ---- 继电器控制命令 (功能码0x06: 写单寄存器) ---- */
    /* 索引  命令含义                    从站  功能码  寄存器地址   写入值    */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x00, 0x00, 0x01},  /* [0]  CMD_OPEN_CH1:  打开第1路继电器，寄存器0x0000写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x00, 0x00, 0x00},  /* [1]  CMD_CLOSE_CH1: 关闭第1路继电器，寄存器0x0000写0x0000 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x01, 0x00, 0x01},  /* [2]  CMD_OPEN_CH2:  打开第2路继电器，寄存器0x0001写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x01, 0x00, 0x00},  /* [3]  CMD_CLOSE_CH2: 关闭第2路继电器，寄存器0x0001写0x0000 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x02, 0x00, 0x01},  /* [4]  CMD_OPEN_CH3:  打开第3路继电器，寄存器0x0002写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x02, 0x00, 0x00},  /* [5]  CMD_CLOSE_CH3: 关闭第3路继电器，寄存器0x0002写0x0000 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x03, 0x00, 0x01},  /* [6]  CMD_OPEN_CH4:  打开第4路继电器，寄存器0x0003写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x03, 0x00, 0x00},  /* [7]  CMD_CLOSE_CH4: 关闭第4路继电器，寄存器0x0003写0x0000 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x04, 0x00, 0x01},  /* [8]  CMD_OPEN_CH5:  打开第5路继电器，寄存器0x0004写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x04, 0x00, 0x00},  /* [9]  CMD_CLOSE_CH5: 关闭第5路继电器，寄存器0x0004写0x0000 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x05, 0x00, 0x01},  /* [10] CMD_OPEN_CH6:  打开第6路继电器，寄存器0x0005写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x05, 0x00, 0x00},  /* [11] CMD_CLOSE_CH6: 关闭第6路继电器，寄存器0x0005写0x0000 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x06, 0x00, 0x01},  /* [12] CMD_OPEN_CH7:  打开第7路继电器，寄存器0x0006写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x06, 0x00, 0x00},  /* [13] CMD_CLOSE_CH7: 关闭第7路继电器，寄存器0x0006写0x0000 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x07, 0x00, 0x01},  /* [14] CMD_OPEN_CH8:  打开第8路继电器，寄存器0x0007写0x0001 */
    {RELAY_SLAVE_ADDR, 0x06, 0x00, 0x07, 0x00, 0x00},  /* [15] CMD_CLOSE_CH8: 关闭第8路继电器，寄存器0x0007写0x0000 */
    /* ---- 读取命令 ---- */
    {RELAY_SLAVE_ADDR, 0x03, 0x00, 0x00, 0x00, 0x08},  /* [16] CMD_READ_ALL:  读8路继电器状态，功能码0x03，起始寄存器0x0000，数量8 */
    {TEMP_HUMI_SLAVE_ADDR, 0x03, 0x00, 0x00, 0x00, 0x02},  /* [17] CMD_READ_HUMIDITY_TEMPERATURE: 读温湿度，功能码0x03，起始寄存器0x0000，数量2 */
    {PM_SLAVE_ADDR, 0x03, 0x00, 0x04, 0x00, 0x06},  /* [18] CMD_READ_PM25_PM10: 读颗粒物浓度，功能码0x03，起始寄存器0x0004，数量6 */
    {GAS_SLAVE_ADDR, 0x04, 0x47, 0x00, 0x00, 0x01},  /* [19] CMD_READ_GAS: 读燃气浓度，功能码0x04(读输入寄存器)，起始寄存器0x4700，数量1 */
};

/** @brief 命令表总条数，由编译器自动计算 */
#define MODBUS_COMMAND_COUNT ((uint8_t)(sizeof(s_modbus_commands) / sizeof(s_modbus_commands[0])))

/**
 * @brief  初始化 Modbus 继电器模块
 * @details 完成以下初始化：
 *          1. 调用 Serial_Init() 初始化 UART1 串口外设（波特率、引脚等）
 *          2. 创建 FreeRTOS 互斥锁用于保护串口总线
 *          3. 清零全局设备状态结构体
 */
void Modbus_Relay_Init(void)
{
    Serial_Init();                                      /* 初始化 UART1 硬件 */
    xSerialSemaphore = xSemaphoreCreateMutex();         /* 创建串口总线互斥锁 */
    memset(&g_device_status, 0, sizeof(g_device_status)); /* 清零设备状态 */
}

/**
 * @brief  校验 Modbus 从站响应帧的合法性
 * @param  cmd      当前发送的命令类型，用于确定期望的响应格式
 * @param  response 接收到的响应帧数据指针
 * @param  length   响应帧的字节长度
 * @return ModbusError_t 校验结果错误码
 * @note   校验顺序：最小长度 → CRC16 → 异常响应标志 → 帧内容匹配
 */
static ModbusError_t ValidateResponse(ModbusCommand_t cmd, uint8_t *response, uint8_t length)
{
    /* 响应帧至少需要4字节：地址(1) + 功能码(1) + 数据(至少0) + CRC(2) */
    if (length < 4)
    {
        return MODBUS_ERROR_RESPONSE;
    }

    /* CRC16校验：验证数据完整性 */
    if (!Serial_ValidateCRC(response, length))
    {
        return MODBUS_ERROR_CRC;
    }

    /* 检查异常响应标志：功能码最高位为1表示从站返回了异常 */
    if (response[1] & 0x80)
    {
        return MODBUS_ERROR_RESPONSE;
    }

    /* ---- 继电器写入命令的响应校验（功能码0x06，回显整个请求帧） ---- */
    if (cmd <= CMD_CLOSE_CH8)
    {
        /* 验证从站地址和功能码 */
        if (response[0] != RELAY_SLAVE_ADDR || response[1] != 0x06)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        /* 写单寄存器的正常响应是原样回显请求帧前6字节 */
        if (memcmp(response, s_modbus_commands[cmd], 6) != 0)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        return MODBUS_SUCCESS;
    }

    /* ---- 读取命令的响应校验 ---- */
    switch (cmd)
    {
    case CMD_READ_ALL:
        /* 继电器状态读取：从站地址=0x02, 功能码=0x03 */
        if (response[0] != RELAY_SLAVE_ADDR || response[1] != 0x03)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        /* 字节数应为 8通道 × 2字节/通道 = 16字节 */
        if (response[2] != (RELAY_CHANNEL_COUNT * 2U))
        {
            return MODBUS_ERROR_RESPONSE;
        }
        break;
    case CMD_READ_HUMIDITY_TEMPERATURE:
        /* 温湿度读取：从站地址=0x01, 功能码=0x03 */
        if (response[0] != TEMP_HUMI_SLAVE_ADDR || response[1] != 0x03)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        break;
    case CMD_READ_PM25_PM10:
        /* PM2.5/PM10读取：从站地址=0x03, 功能码=0x03 */
        if (response[0] != PM_SLAVE_ADDR || response[1] != 0x03)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        /* 返回数据字节数应为0x0C（6个寄存器 × 2字节 = 12字节） */
        if (response[2] != 0x0C)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        break;
    case CMD_READ_GAS:
        /* 燃气传感器读取：从站地址=0x04, 功能码=0x04（读输入寄存器） */
        if (response[0] != GAS_SLAVE_ADDR || response[1] != 0x04)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        /* 返回数据字节数应为0x02（1个寄存器 × 2字节） */
        if (response[2] != 0x02)
        {
            return MODBUS_ERROR_RESPONSE;
        }
        break;
    default:
        return MODBUS_ERROR_RESPONSE;
    }

    return MODBUS_SUCCESS;
}

/**
 * @brief  发送 Modbus 命令并等待响应（无RTOS版本，阻塞式）
 * @details 在 FreeRTOS 调度器启动前使用，不依赖互斥锁和任务延时，
 *          使用 Delay_ms() 阻塞等待。用于上电初始化阶段。
 * @param  cmd         要发送的命令索引
 * @param  retry_count 最大重试次数
 * @param  timeout_ms  单次等待响应的超时时间（毫秒）
 * @return ModbusError_t 通信结果错误码
 */
static ModbusError_t SendModbusCommand_NoRTOS(ModbusCommand_t cmd, uint8_t retry_count, uint16_t timeout_ms)
{
    uint8_t packet[8] = {0};                    /* 发送缓冲区：6字节命令 + 2字节CRC */
    ModbusError_t result = MODBUS_ERROR_TIMEOUT; /* 默认结果为超时 */
    uint8_t retry;
    uint16_t wait_ms;

    /* 检查命令索引合法性 */
    if ((uint8_t)cmd >= MODBUS_COMMAND_COUNT)
    {
        return MODBUS_ERROR_RESPONSE;
    }

    /* 从命令表复制6字节报文体（CRC由Serial_SendPacket自动追加） */
    memcpy(packet, s_modbus_commands[cmd], 6U);

    /* 重试循环 */
    for (retry = 0U; retry < retry_count; retry++)
    {
        Serial_ClearRxBuffer();     /* 清空接收缓冲区，丢弃残留数据 */
        Serial_SendPacket(packet);  /* 发送Modbus请求帧（自动追加CRC16） */

        /* 轮询等待响应，每1ms检查一次 */
        for (wait_ms = 0U; wait_ms < timeout_ms; wait_ms++)
        {
            if (Serial_GetRxFlag() != 0U)
            {
                /* 收到响应帧 */
                uint8_t *response = Serial_GetRxBuffer();       /* 获取接收缓冲区指针 */
                uint8_t response_length = Serial_GetRxLength(); /* 获取接收数据长度 */
                ModbusError_t error = ValidateResponse(cmd, response, response_length); /* 校验响应 */

                if (error == MODBUS_SUCCESS)
                {
                    result = MODBUS_SUCCESS;
                    break;  /* 通信成功，退出等待循环 */
                }

                result = error; /* 记录错误类型，继续等待或重试 */
            }
            Delay_ms(1U);   /* 阻塞延时1ms */
        }

        if (result == MODBUS_SUCCESS)
        {
            break;  /* 通信成功，退出重试循环 */
        }

        Delay_ms(20U);  /* 重试间隔20ms，给从站留出恢复时间 */
    }

    return result;
}

/**
 * @brief  上电时强制关闭全部8路继电器（没用）
 * @details 在 FreeRTOS 调度器启动前调用。依次向8路继电器发送关闭命令，
 *          确保系统上电后所有继电器处于安全的断开状态。
 *          每路重试2次，单次超时180ms。
 * @return 1=全部关闭成功，0=至少有一路关闭失败
 */
uint8_t Modbus_Relay_ForceAllOffOnBoot(void)
{
    uint8_t ch;
    uint8_t ok = 1U;    /* 成功标志 */

    /* 遍历8路继电器，依次发送关闭命令 */
    for (ch = 0U; ch < RELAY_CHANNEL_COUNT; ch++)
    {
        /*
         * 关闭命令索引计算：CMD_CLOSE_CHn = CMD_CLOSE_CH1 + (通道号 × 2)
         * 因为命令表中每个通道占2个条目（打开+关闭），关闭命令在奇数索引位置
         */
        ModbusCommand_t cmd = (ModbusCommand_t)(CMD_CLOSE_CH1 + (ch * 2U));
        if (SendModbusCommand_NoRTOS(cmd, 2U, 180U) == MODBUS_SUCCESS)
        {
            g_device_status.relay_state[ch] = 0U;   /* 更新状态为断开 */
        }
        else
        {
            ok = 0U;    /* 标记失败 */
        }
    }

    return ok;
}

/**
 * @brief  发送 Modbus 命令并等待响应（FreeRTOS 环境，带互斥锁保护）
 * @details 完整的 Modbus RTU 通信流程：
 *          1. 获取串口互斥锁（最长等待1秒）
 *          2. 组装并发送请求帧
 *          3. 等待从站响应（继电器命令150ms，传感器命令300ms）
 *          4. 校验响应帧并解析数据到 g_device_status
 *          5. 释放互斥锁
 * @param  cmd         要发送的命令索引，见 ModbusCommand_t
 * @param  retry_count 最大重试次数
 * @return ModbusError_t 通信结果错误码
 */
ModbusError_t SendModbusCommand(ModbusCommand_t cmd, uint8_t retry_count)
{
    uint8_t packet[8] = {0};                    /* 发送缓冲区：6字节命令 + 2字节CRC */
    ModbusError_t result = MODBUS_ERROR_TIMEOUT; /* 默认结果为超时 */

    /* 检查命令索引合法性 */
    if ((uint8_t)cmd >= MODBUS_COMMAND_COUNT)
    {
        return MODBUS_ERROR_RESPONSE;
    }

    /* 获取串口互斥锁，最长等待1000ms，防止多任务并发访问总线 */
    if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        return MODBUS_ERROR_SERIAL;  /* 互斥锁获取超时 */
    }

    /* 从命令表复制6字节报文体 */
    memcpy(packet, s_modbus_commands[cmd], 6);

    /* 重试循环 */
    for (uint8_t retry = 0; retry < retry_count; retry++)
    {
        TickType_t timeout = pdMS_TO_TICKS(150);    /* 继电器命令默认超时150ms */
        TickType_t start_time;

        /* 传感器读取命令响应较慢，超时设为300ms */
        if (cmd == CMD_READ_HUMIDITY_TEMPERATURE || cmd == CMD_READ_PM25_PM10 || cmd == CMD_READ_GAS)
        {
            timeout = pdMS_TO_TICKS(300);
        }

        Serial_ClearRxBuffer();             /* 清空接收缓冲区 */
        Serial_SendPacket(packet);          /* 发送Modbus请求帧（自动追加CRC16） */
        start_time = xTaskGetTickCount();   /* 记录发送时刻，用于超时判断 */

        /* 轮询等待响应，每1ms检查一次，直到超时 */
        while ((xTaskGetTickCount() - start_time) < timeout)
        {
            if (Serial_GetRxFlag())
            {
                /* 收到响应帧 */
                uint8_t *response = Serial_GetRxBuffer();       /* 获取接收缓冲区指针 */
                uint8_t response_length = Serial_GetRxLength(); /* 获取实际接收长度 */
                ModbusError_t error = ValidateResponse(cmd, response, response_length); /* 校验响应帧 */

                if (error == MODBUS_SUCCESS)
                {
                    result = MODBUS_SUCCESS;

                    /* ===== 根据命令类型解析响应数据 ===== */
                    if (cmd <= CMD_CLOSE_CH8)
                    {
                        /* --- 继电器写入命令：从命令索引推算通道号和目标状态 --- */
                        uint8_t relay_index = ((uint8_t)cmd) / 2U;  /* 通道号 = 命令索引 / 2（每通道占2条命令） */
                        uint8_t relay_state = ((((uint8_t)cmd) & 0x01U) == 0U) ? 1U : 0U; /* 偶数索引=打开(1)，奇数索引=关闭(0) */
                        g_device_status.relay_state[relay_index] = relay_state; /* 更新全局状态 */
                    }
                    else
                    {
                        /* --- 读取命令：解析响应帧中的传感器数据 --- */
                        switch (cmd)
                        {
                        case CMD_READ_ALL:
                            /*
                             * 继电器状态响应帧格式：
                             * [地址(1)] [功能码0x03(1)] [字节数(1)] [CH1高(1)][CH1低(1)]...[CH8高(1)][CH8低(1)] [CRC(2)]
                             * 最小长度 = 3(头) + 16(8通道×2字节) + 2(CRC) = 21字节
                             */
                            if (response_length >= (uint8_t)(3U + (RELAY_CHANNEL_COUNT * 2U) + 2U))
                            {
                                for (uint8_t i = 0; i < RELAY_CHANNEL_COUNT; i++)
                                {
                                    /* 拼接大端序16位寄存器值：高字节在前，低字节在后 */
                                    uint16_t relay_raw = ((uint16_t)response[3 + (i * 2)] << 8) | response[4 + (i * 2)];
                                    g_device_status.relay_state[i] = (relay_raw == 0x0001U) ? 1U : 0U; /* 0x0001=ON, 其他=OFF */
                                }
                            }
                            break;
                        case CMD_READ_HUMIDITY_TEMPERATURE:
                            /*
                             * 温湿度响应帧格式：
                             * [地址(1)] [功能码0x03(1)] [字节数=4(1)] [湿度高(1)][湿度低(1)] [温度高(1)][温度低(1)] [CRC(2)]
                             * 湿度和温度均为原始值，需除以10.0得到实际值
                             * 温度可能为负数，需按int16_t处理
                             */
                            if (response_length >= 7 && response[2] == 4)
                            {
                                uint16_t humi_raw = ((uint16_t)response[3] << 8) | response[4]; /* 湿度原始值（×10） */
                                uint16_t temp_raw = ((uint16_t)response[5] << 8) | response[6]; /* 温度原始值（×10，有符号） */
                                g_device_status.humidity = humi_raw / 10.0f;          /* 换算为 %RH */
                                g_device_status.temperature = (int16_t)temp_raw / 10.0f; /* 换算为 ℃（支持负温度） */
                            }
                            break;
                        case CMD_READ_PM25_PM10:
                            /*
                             * PM传感器响应帧格式：
                             * [地址(1)] [功能码0x03(1)] [字节数(1)] [数据...] [CRC(2)]
                             * response[3-4]: PM2.5浓度（μg/m³）
                             * response[5-6]: PM10浓度（μg/m³）
                             */
                            if (response_length >= 9 && response[2] >= 4)
                            {
                                g_device_status.pm25 = ((uint16_t)response[3] << 8) | response[4]; /* PM2.5 大端序拼接 */
                                g_device_status.pm10 = ((uint16_t)response[5] << 8) | response[6]; /* PM10  大端序拼接 */
                            }
                            break;
                        case CMD_READ_GAS:
                            /*
                             * 燃气传感器响应帧格式：
                             * [地址(1)] [功能码0x04(1)] [字节数=2(1)] [气体高(1)][气体低(1)] [CRC(2)]
                             */
                            if (response_length >= 7 && response[2] == 2)
                            {
                                g_device_status.gas = ((uint16_t)response[3] << 8) | response[4]; /* 燃气浓度值 */
                            }
                            break;
                        default:
                            break;
                        }
                    }

                    g_device_status.last_update = xTaskGetTickCount(); /* 记录最近一次成功通信的时间戳 */
                    break;  /* 通信成功，退出等待循环 */
                }

                result = error; /* 记录错误类型 */
            }

            Delay_TaskMs(1U); /* 至少让出 1 tick，避免 1ms 在 100Hz tick 下被截断为 0 */
        }

        if (result == MODBUS_SUCCESS)
        {
            break;  /* 通信成功，退出重试循环 */
        }

        vTaskDelay(pdMS_TO_TICKS(20)); /* 重试间隔20ms */
    }

    xSemaphoreGive(xSerialSemaphore);  /* 释放串口互斥锁 */
    return result;
}

/**
 * @brief  读取全部继电器通道状态
 * @param  states       输出缓冲区，存储各通道状态（1=ON, 0=OFF）
 * @param  max_channels 输出缓冲区最大容量（防止越界）
 * @return 1=读取成功，0=参数无效或通信失败
 */
uint8_t ReadRelayStatus(uint8_t *states, uint8_t max_channels)
{
    uint8_t copy_count;

    /* 参数有效性检查 */
    if (states == NULL || max_channels == 0U)
    {
        return 0;
    }

    /* 发送读取全部继电器状态命令，重试2次 */
    if (SendModbusCommand(CMD_READ_ALL, 2) != MODBUS_SUCCESS)
    {
        return 0;
    }

    /* 复制状态数据，取缓冲区大小与实际通道数的较小值 */
    copy_count = (max_channels < RELAY_CHANNEL_COUNT) ? max_channels : RELAY_CHANNEL_COUNT;
    memcpy(states, g_device_status.relay_state, copy_count);
    return 1;
}

/**
 * @brief  读取温湿度传感器数据
 * @param  temp 输出参数，温度值（℃）
 * @param  humi 输出参数，湿度值（%RH）
 * @return 1=读取成功，0=通信失败
 */
uint8_t ReadTemperatureHumidity(float *temp, float *humi)
{
    if (SendModbusCommand(CMD_READ_HUMIDITY_TEMPERATURE, 2) == MODBUS_SUCCESS)
    {
        *temp = g_device_status.temperature; /* 从全局状态获取已解析的温度值 */
        *humi = g_device_status.humidity;     /* 从全局状态获取已解析的湿度值 */
        return 1;
    }
    return 0;
}

/**
 * @brief  读取 PM2.5/PM10 颗粒物传感器数据
 * @param  pm25 输出参数，PM2.5 浓度（μg/m³）
 * @param  pm10 输出参数，PM10 浓度（μg/m³）
 * @return 1=读取成功，0=通信失败
 */
uint8_t ReadPM25PM10(uint16_t *pm25, uint16_t *pm10)
{
    if (SendModbusCommand(CMD_READ_PM25_PM10, 2) == MODBUS_SUCCESS)
    {
        *pm25 = g_device_status.pm25; /* 从全局状态获取 PM2.5 值 */
        *pm10 = g_device_status.pm10; /* 从全局状态获取 PM10 值 */
        return 1;
    }
    return 0;
}

/**
 * @brief  在 OLED 上显示 PM2.5/PM10 颗粒物数据
 * @details 显示格式：
 *          第1行: "PM Sensor:"
 *          第2行: "PM2.5: xxxx ug/m3"
 *          第3行: "PM10:  xxxx ug/m3"
 *          读取失败时显示 "Read PM Failed"
 */
void DisplayPM25PM10(void)
{
    uint16_t pm25, pm10;

    if (ReadPM25PM10(&pm25, &pm10))
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "PM Sensor:");        /* 标题行 */
        OLED_Printf(0, 16, OLED_6X8, "PM2.5:");           /* PM2.5 标签 */
        OLED_ShowNum(50, 16, pm25, 4, OLED_6X8);           /* PM2.5 数值 */
        OLED_Printf(90, 16, OLED_6X8, "ug/m3");           /* 单位 */
        OLED_Printf(0, 32, OLED_6X8, "PM10:");            /* PM10 标签 */
        OLED_ShowNum(50, 32, pm10, 4, OLED_6X8);           /* PM10 数值 */
        OLED_Printf(90, 32, OLED_6X8, "ug/m3");           /* 单位 */
        OLED_Update();                                      /* 刷新OLED显示 */
    }
    else
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "Read PM Failed");    /* 读取失败提示 */
        OLED_Update();
    }
}

/**
 * @brief  读取燃气传感器数据
 * @param  gas 输出参数，燃气浓度值
 * @return 1=读取成功，0=通信失败
 */
uint8_t ReadGas(uint16_t *gas)
{
    if (SendModbusCommand(CMD_READ_GAS, 2) == MODBUS_SUCCESS)
    {
        *gas = g_device_status.gas;  /* 从全局状态获取燃气浓度值 */
        return 1;
    }
    return 0;
}

/**
 * @brief  在 OLED 上显示燃气传感器数据
 * @details 显示格式：
 *          第1行: "Gas Sensor:"
 *          第2行: "Gas: xxxx"
 *          读取失败时显示 "Read Gas Failed"
 */
void DisplayGas(void)
{
    uint16_t gas;

    if (ReadGas(&gas))
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "Gas Sensor:");       /* 标题行 */
        OLED_Printf(0, 16, OLED_6X8, "Gas:");             /* 燃气标签 */
        OLED_ShowNum(50, 16, gas, 4, OLED_6X8);            /* 燃气浓度数值 */
        OLED_Update();                                      /* 刷新OLED显示 */
    }
    else
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "Read Gas Failed");   /* 读取失败提示 */
        OLED_Update();
    }
}


/**
 * @brief  控制单路继电器的开/关
 * @param  relay_num 继电器通道号（0~7，对应第1~8路）
 * @param  state     目标状态，非0=打开（吸合），0=关闭（断开）
 * @return 1=操作成功，0=参数无效或通信失败
 */
uint8_t WriteRelay(uint8_t relay_num, uint8_t state)
{
    ModbusCommand_t cmd;
    uint8_t cmd_index;

    /* 通道号范围检查 */
    if (relay_num >= RELAY_CHANNEL_COUNT)
    {
        return 0;
    }

    /*
     * 计算命令索引:
     * 每个通道占2个命令（打开+关闭），打开在偶数索引，关闭在奇数索引
     * cmd_index = 通道号 × 2 + (state ? 0 : 1)
     * 例如：通道0打开→索引0(CMD_OPEN_CH1)，通道0关闭→索引1(CMD_CLOSE_CH1)
     */
    cmd_index = (uint8_t)(relay_num * 2U + (state ? 0U : 1U));
    cmd = (ModbusCommand_t)cmd_index;
    return (SendModbusCommand(cmd, 2) == MODBUS_SUCCESS);
}


/**
 * @brief  在 OLED 上显示8路继电器状态
 * @details 显示格式（双列布局）：
 *          第1行: "Relay Status:"
 *          第2行: "C1:ON   C5:OFF"
 *          第3行: "C2:OFF  C6:ON"
 *          第4行: "C3:ON   C7:OFF"
 *          第5行: "C4:OFF  C8:ON"
 *          读取失败时显示 "Read Relay Failed"
 */
void DisplayRelayStatus(void)
{
    uint8_t relay_states[RELAY_CHANNEL_COUNT] = {0};

    if (ReadRelayStatus(relay_states, RELAY_CHANNEL_COUNT))
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "Relay Status:");     /* 标题行 */
        /* 双列显示：左列CH1~4，右列CH5~8 */
        for (uint8_t row = 0; row < (RELAY_CHANNEL_COUNT / 2U); row++)
        {
            uint8_t left_idx = row;                             /* 左列通道索引：0~3 */
            uint8_t right_idx = row + (RELAY_CHANNEL_COUNT / 2U); /* 右列通道索引：4~7 */
            uint8_t y = (uint8_t)(16U + row * 12U);             /* Y坐标，每行间距12像素 */
            OLED_Printf(0, y, OLED_6X8, "C%u:%s", left_idx + 1U, relay_states[left_idx] ? "ON " : "OFF");
            OLED_Printf(64, y, OLED_6X8, "C%u:%s", right_idx + 1U, relay_states[right_idx] ? "ON " : "OFF");
        }
        OLED_Update();  /* 刷新OLED显示 */
    }
    else
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "Read Relay Failed"); /* 读取失败提示 */
        OLED_Update();
    }
}

/**
 * @brief  在 OLED 上显示温湿度传感器数据
 * @details 显示格式：
 *          第1行: "Temperature:"
 *          第2行: "xx.xx C"
 *          第3行: "Humidity:"
 *          第4行: "xx.xx %RH"
 *          读取失败时显示 "Read Sensor Failed"
 */
void DisplayTemperatureHumidity(void)
{
    float temp, humi;

    if (ReadTemperatureHumidity(&temp, &humi))
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "Temperature:");              /* 温度标题 */
        OLED_ShowFloatNum(0, 16, temp, 2, 2, 6);                   /* 温度值：2位整数+2位小数 */
        OLED_Printf(50, 16, OLED_6X8, "C");                       /* 单位：摄氏度 */

        OLED_Printf(0, 32, OLED_6X8, "Humidity:");                 /* 湿度标题 */
        OLED_ShowFloatNum(0, 48, humi, 2, 2, 6);                   /* 湿度值：2位整数+2位小数 */
        OLED_Printf(50, 48, OLED_6X8, "%%RH");                    /* 单位：相对湿度百分比 */
        OLED_Update();                                              /* 刷新OLED显示 */
    }
    else
    {
        OLED_Clear();
        OLED_Printf(0, 0, OLED_6X8, "Read Sensor Failed");        /* 读取失败提示 */
        OLED_Update();
    }
}
