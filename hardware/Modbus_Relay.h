/**
 * @file    Modbus_Relay.h
 * @brief   Modbus RTU 继电器控制与传感器数据采集模块 - 头文件
 * @details 本模块用于 STM32F407 环境保障系统，通过 Modbus RTU 协议
 *          控制8路继电器（写单寄存器 0x06）并读取多种环境传感器数据
 *          （温湿度、PM2.5/PM10、燃气），同时提供 OLED 显示功能。
 *          使用 FreeRTOS 互斥锁保护 Modbus 串口总线的并发访问。
 */

#ifndef MODBUS_RELAY_H
#define MODBUS_RELAY_H

#include <stdint.h>         /* 标准整数类型定义 */
#include "FreeRTOS.h"       /* FreeRTOS 内核头文件 */
#include "semphr.h"         /* FreeRTOS 信号量/互斥锁头文件 */

/* ======================== Modbus 从设备地址定义 ======================== */
#define RELAY_SLAVE_ADDR      0x02    /**< 继电器板 Modbus 从站地址 */
#define TEMP_HUMI_SLAVE_ADDR  0x01    /**< 温湿度传感器 Modbus 从站地址 */
#define PM_SLAVE_ADDR         0x03    /**< PM2.5/PM10 颗粒物传感器 Modbus 从站地址 */
#define GAS_SLAVE_ADDR        0x04    /**< 燃气传感器 Modbus 从站地址 */
#define RELAY_CHANNEL_COUNT   8       /**< 继电器总通道数（8路独立控制） */

/* ======================== Modbus 通信错误码定义 ======================== */
/**
 * @brief Modbus 通信返回错误码枚举
 */
typedef enum
{
    MODBUS_SUCCESS = 0,         /**< 通信成功 */
    MODBUS_ERROR_TIMEOUT,       /**< 通信超时，从站未在规定时间内响应 */
    MODBUS_ERROR_CRC,           /**< CRC16 校验失败，数据传输出错 */
    MODBUS_ERROR_RESPONSE,      /**< 响应帧格式错误或异常响应 */
    MODBUS_ERROR_SERIAL         /**< 串口资源获取失败（互斥锁超时） */
} ModbusError_t;



/* ======================== Modbus 命令索引枚举 ======================== */
/**
 * @brief Modbus 命令索引枚举
 * @details 每个枚举值对应 s_modbus_commands[] 命令表中的一条预定义报文。
 *          继电器控制命令按 "打开/关闭" 交替排列，索引规则：
 *          - 打开第N路: CMD_OPEN_CHn  = (N-1) * 2
 *          - 关闭第N路: CMD_CLOSE_CHn = (N-1) * 2 + 1
 */
typedef enum
{
    CMD_OPEN_CH1 = 0,               /**< 打开继电器第1路 */
    CMD_CLOSE_CH1,                  /**< 关闭继电器第1路 */
    CMD_OPEN_CH2,                   /**< 打开继电器第2路 */
    CMD_CLOSE_CH2,                  /**< 关闭继电器第2路 */
    CMD_OPEN_CH3,                   /**< 打开继电器第3路 */
    CMD_CLOSE_CH3,                  /**< 关闭继电器第3路 */
    CMD_OPEN_CH4,                   /**< 打开继电器第4路 */
    CMD_CLOSE_CH4,                  /**< 关闭继电器第4路 */
    CMD_OPEN_CH5,                   /**< 打开继电器第5路 */
    CMD_CLOSE_CH5,                  /**< 关闭继电器第5路 */
    CMD_OPEN_CH6,                   /**< 打开继电器第6路 */
    CMD_CLOSE_CH6,                  /**< 关闭继电器第6路 */
    CMD_OPEN_CH7,                   /**< 打开继电器第7路 */
    CMD_CLOSE_CH7,                  /**< 关闭继电器第7路 */
    CMD_OPEN_CH8,                   /**< 打开继电器第8路 */
    CMD_CLOSE_CH8,                  /**< 关闭继电器第8路 */
    CMD_READ_ALL,                   /**< 读取全部8路继电器状态（功能码0x03） */
    CMD_READ_HUMIDITY_TEMPERATURE,  /**< 读取温湿度传感器数据（功能码0x03） */
    CMD_READ_PM25_PM10,             /**< 读取PM2.5/PM10颗粒物传感器数据（功能码0x03） */
    CMD_READ_GAS                    /**< 读取燃气传感器数据（功能码0x04） */
} ModbusCommand_t;



/* ======================== 设备状态数据结构体 ======================== */
/**
 * @brief 设备状态汇总结构体
 * @details 保存所有继电器通道状态和各传感器最新读数，
 *          由 SendModbusCommand() 在成功通信后自动更新。
 */
typedef struct
{
    uint8_t relay_state[RELAY_CHANNEL_COUNT];  /**< 8路继电器状态数组，1=吸合/ON，0=断开/OFF */
    float temperature;      /**< 温度值，单位：摄氏度（°C），精度0.1 */
    float humidity;          /**< 湿度值，单位：%RH，精度0.1 */
    uint16_t pm25;           /**< PM2.5 浓度，单位：μg/m³ */
    uint16_t pm10;           /**< PM10 浓度，单位：μg/m³ */
    uint16_t gas;            /**< 燃气浓度原始值（ppm） */
    uint32_t last_update;    /**< 最后一次成功通信的时间戳（FreeRTOS Tick） */
} DeviceStatus_t;



/* ======================== 函数声明 ======================== */

/**
 * @brief  初始化 Modbus 继电器模块
 * @details 初始化串口外设，创建 FreeRTOS 互斥锁，清零设备状态
 */
void Modbus_Relay_Init(void);

/**
 * @brief  发送 Modbus 命令并等待响应（FreeRTOS 环境，带互斥锁保护）
 * @param  cmd         要发送的命令索引，见 ModbusCommand_t 枚举
 * @param  retry_count 通信失败时的最大重试次数
 * @return ModbusError_t 通信结果错误码
 */
ModbusError_t SendModbusCommand(ModbusCommand_t cmd, uint8_t retry_count);

/**
 * @brief  读取全部继电器通道状态
 * @param  states       输出缓冲区，存储各通道状态（1=ON, 0=OFF）
 * @param  max_channels 输出缓冲区最大容量（通道数）
 * @return 1=读取成功，0=读取失败
 */
uint8_t ReadRelayStatus(uint8_t *states, uint8_t max_channels);

/**
 * @brief  读取温湿度传感器数据
 * @param  temp 输出参数，温度值（°C）
 * @param  humi 输出参数，湿度值（%RH）
 * @return 1=读取成功，0=读取失败
 */
uint8_t ReadTemperatureHumidity(float *temp, float *humi);

/**
 * @brief  读取 PM2.5 和 PM10 颗粒物传感器数据
 * @param  pm25 输出参数，PM2.5 浓度（μg/m³）
 * @param  pm10 输出参数，PM10 浓度（μg/m³）
 * @return 1=读取成功，0=读取失败
 */
uint8_t ReadPM25PM10(uint16_t *pm25, uint16_t *pm10);

/**
 * @brief  读取燃气传感器数据
 * @param  gas 输出参数，燃气浓度值
 * @return 1=读取成功，0=读取失败
 */
uint8_t ReadGas(uint16_t *gas);

/**
 * @brief  控制单路继电器的开/关
 * @param  relay_num 继电器通道号（0~7，对应第1~8路）
 * @param  state     目标状态，1=打开（吸合），0=关闭（断开）
 * @return 1=操作成功，0=操作失败
 */
uint8_t WriteRelay(uint8_t relay_num, uint8_t state);

/**
 * @brief  在 OLED 上显示8路继电器状态
 */
void DisplayRelayStatus(void);

/**
 * @brief  在 OLED 上显示温湿度数据
 */
void DisplayTemperatureHumidity(void);

/**
 * @brief  在 OLED 上显示 PM2.5/PM10 数据
 */
void DisplayPM25PM10(void);

/**
 * @brief  在 OLED 上显示燃气传感器数据
 */
void DisplayGas(void);

/**
 * @brief  上电时强制关闭全部继电器（无RTOS环境，阻塞式）
 * @details 在 FreeRTOS 调度器启动前调用，依次关闭8路继电器，
 *          确保系统上电后所有继电器处于安全的断开状态。
 * @return 1=全部关闭成功，0=至少有一路关闭失败
 */
uint8_t Modbus_Relay_ForceAllOffOnBoot(void);


/* ======================== 全局变量声明 ======================== */
extern DeviceStatus_t g_device_status;      /**< 全局设备状态实例，保存最新的传感器和继电器数据 */
extern SemaphoreHandle_t xSerialSemaphore;  /**< Modbus 串口总线互斥锁，防止多任务并发访问 */

#endif /* MODBUS_RELAY_H */
