/**
 * @file    screen.c
 * @brief   串口屏控制模块 —— 核心实现
 *
 * 本文件实现了 STM32F407 环境保障系统与串口屏（UART4, 512000bps）之间的
 * 完整通信控制逻辑，涵盖以下子系统：
 *
 * 【架构概述】
 *   1. 协议编解码层
 *      - 自定义二进制帧格式：0x55(头) + 数据 + 0xAA(尾)
 *      - 滑动窗口解析器：3字节窗口(页面切换)、7字节窗口(阈值/自动恢复)、8字节窗口(手动控制/恢复出厂)
 *      - 小端字节序 32 位数值编解码
 *
 *   2. 页面管理系统
 *      - PAGE 0x00: 主页（连接状态汇总、日期时间）
 *      - PAGE 0x01: 传感器数据页（温湿度/PM2.5/气体/人体/光照/门磁）
 *      - PAGE 0x02~0x07: 阈值设置页 1~6
 *      - PAGE 0x08~0x11: 手动控制页 1~4（加热/制冷/加湿/除湿/报警/灯光/风扇）
 *      - PAGE 0x12: 继电器总览页
 *      - PAGE 0x13: 执行记录页
 *      - PAGE 0x14: 系统设置页
 *
 *   3. 文本缓存去重机制
 *      - 80 个缓存槽位，每个存储组件 ID 与对应文本值
 *      - 仅在文本内容变化时才通过 UART 发送，减少通信量
 *
 *   4. 本地时钟
 *      - 基于 FreeRTOS Tick 的秒级软时钟
 *      - 支持 NTP/MQTT 时间同步与延迟补偿
 *      - 含闰年、月天数计算
 *
 *   5. 动作记录队列
 *      - 环形队列（24 条），记录设备操作事件
 *      - 同时写入串口屏 recorder 控件和内存队列
 *      - 供 MQTT 模块异步弹出上报
 *
 *   6. Nextion 指令接口
 *      - 文本指令：component.txt="value" + 0xFF 0xFF 0xFF
 *      - 数值指令：component.val=N + 0xFF 0xFF 0xFF
 *      - 使用互斥锁保护多任务并发发送
 *
 * 所有 GB2312 中文字符串均以十六进制转义编码，避免源文件编码问题。
 */

#include "screen.h"

/* ======================== 头文件包含 ======================== */
#include "uart4_screen.h"       /* UART4 串口屏底层驱动 */
#include "all_sensor_data.h"    /* 传感器数据聚合接口 */
#include "Modbus_Relay.h"       /* Modbus 继电器通信接口 */
#include "esp01s.h"             /* ESP01S WiFi 模块状态查询 */
#include "mqtt_app.h"           /* MQTT Broker 连接状态查询 */
#include "logic.h"              /* 业务逻辑层（阈值/手动控制/设备状态） */
#include "runtime_config.h"     /* 运行时配置（恢复出厂设置） */
#include "delay.h"              /* 任务态/阻塞式延时工具 */
#include "FreeRTOS.h"           /* FreeRTOS 内核 */
#include "task.h"               /* FreeRTOS 任务管理 */
#include "semphr.h"             /* FreeRTOS 信号量/互斥锁 */

#include <stdio.h>
#include <string.h>

/* ======================== 协议帧常量 ======================== */
#define SCREEN_CMD_HEAD                 0x55U   /**< 帧头标识字节 */
#define SCREEN_CMD_INDEX_PAGE           0x00U   /**< 页面ID：主页（首页） */
#define SCREEN_CMD_SENSOR_PAGE          0x01U   /**< 页面ID：传感器数据页 */
#define SCREEN_CMD_CHANGE1_PAGE         0x02U   /**< 页面ID：阈值设置页1（温湿度阈值） */
#define SCREEN_CMD_CHANGE2_PAGE         0x03U   /**< 页面ID：阈值设置页2（PM2.5/气体报警） */
#define SCREEN_CMD_CHANGE3_PAGE         0x04U   /**< 页面ID：阈值设置页3（门磁/光照/人感/风扇1档） */
#define SCREEN_CMD_CHANGE4_PAGE         0x05U   /**< 页面ID：阈值设置页4（风扇2~5档转速） */
#define SCREEN_CMD_CHANGE5_PAGE         0x06U   /**< 页面ID：阈值设置页5（PM2.5 五档区间） */
#define SCREEN_CMD_CHANGE6_PAGE         0x07U   /**< 页面ID：阈值设置页6（气体 五档区间） */
#define SCREEN_CMD_HAND1_PAGE           0x08U   /**< 页面ID：手动控制页1（加热器/制冷器） */
#define SCREEN_CMD_HAND2_PAGE           0x09U   /**< 页面ID：手动控制页2（加湿器/除湿器） */
#define SCREEN_CMD_HAND3_PAGE           0x10U   /**< 页面ID：手动控制页3（报警器/灯光） */
#define SCREEN_CMD_HAND4_PAGE           0x11U   /**< 页面ID：手动控制页4（风扇） */
#define SCREEN_CMD_RELAY_PAGE           0x12U   /**< 页面ID：继电器总览页 */
#define SCREEN_CMD_RELAY_PAGE_ALT       0x13U   /**< 页面ID：执行记录页 */
#define SCREEN_CMD_SETTING_PAGE         0x14U   /**< 页面ID：系统设置页 */
#define SCREEN_CMD_SETTING_PAGE_ALT     0x15U   /**< 页面ID：无 */
#define SCREEN_CMD_NONE_PAGE            0xFFU   /**< 无效/未知页面ID */
#define SCREEN_CMD_TAIL                 0xAAU   /**< 帧尾标识字节 */

/* ======================== 帧长度与刷新参数 ======================== */
#define SCREEN_CHANGE1_FRAME_LEN        7U      /**< 阈值设置/自动恢复帧长度：头(1)+ID(1)+数据(4)+尾(1) */
#define SCREEN_MANUAL_SAVE_FRAME_LEN    8U      /**< 手动控制保存帧长度：头(1)+ID(1)+时长(4)+值(1)+尾(1) */
#define SCREEN_REFRESH_INTERVAL_MS      500U    /**< 允许周期刷新的页面刷新间隔（毫秒） */
#define SCREEN_ACTION_QUEUE_SIZE        24U     /**< 动作事件环形队列容量 */
#define SCREEN_TEXT_CACHE_SLOTS         80U     /**< 单页缓存控件数量（如果一页内需要更改的数据控件大于80就要增加，不增加也没太大问题） */
#define SCREEN_TEXT_CACHE_VALUE_MAX     384U    /**< 单个控件的最大文本长度 */
#define SCREEN_RELAY_POLL_INTERVAL_MS   3000U   /**< 继电器连接状态轮询间隔（毫秒） */
#define SCREEN_RESET_DEBOUNCE_MS        1500U   /**< 恢复出厂帧去重窗口（毫秒） */
#define SCREEN_SELF_TEST_SHORT_FIRST    0U      /**< 自检开关：1=启动时发送简短自检, 0=关闭 */
#define SCREEN_TIME_SYNC_SEC_THRESHOLD  1U      /**< NTP 校时秒数偏差阈值（超过则校准） */
#define SCREEN_TIME_RX_COMPENSATE_S     2U      /**< 接收延迟补偿秒数（补偿网络+处理延时） */

/* ======================== 手动控制设备ID常量 ======================== */
#define SCREEN_MANUAL_HEATER_ID         0x30U   /**< 手动控制帧设备ID：加热器 */
#define SCREEN_MANUAL_COOLER_ID         0x31U   /**< 手动控制帧设备ID：制冷器 */
#define SCREEN_MANUAL_HUMIDIFIER_ID     0x32U   /**< 手动控制帧设备ID：加湿器 */
#define SCREEN_MANUAL_DEHUMIDIFIER_ID   0x33U   /**< 手动控制帧设备ID：除湿器 */
#define SCREEN_MANUAL_ALARM_ID          0x34U   /**< 手动控制帧设备ID：声光报警器 */
#define SCREEN_MANUAL_LIGHT_ID          0x35U   /**< 手动控制帧设备ID：LED灯光 */
#define SCREEN_MANUAL_FAN_ID            0x36U   /**< 手动控制帧设备ID：风扇 */
#define SCREEN_MANUAL_AUTO_MAGIC_BYTE   0xF0U   /**< 自动恢复帧的魔术字节标识 */
#define SCREEN_RESET_DEFAULTS_ID        0x88U   /**< 恢复出厂设置帧的设备ID （屏幕会重复发送三次）*/
#define SCREEN_RESET_DEFAULTS_MARK       0xFFU  /**< 恢复出厂设置帧的标记字节（屏幕会重复发送三次） */

/* ======================== GB2312 中文字符串常量 ======================== */
/* 以十六进制转义形式编码，避免源文件编码问题。实际显示内容见右侧注释。 */
#define CN_CONNECTED                    "\xD2\xD1\xC1\xAC\xBD\xD3"               /* "已连接" */
#define CN_DISCONNECTED                 "\xCE\xB4\xC1\xAC\xBD\xD3"               /* "未连接" */
#define CN_TEMP_HUMI                    "\xCE\xC2\xCA\xAA\xB6\xC8\xB4\xAB\xB8\xD0\xC6\xF7"   /* "温湿度传感器" */
#define CN_PM                           "\xB7\xDB\xB3\xBE\xB4\xAB\xB8\xD0\xC6\xF7"           /* "粉尘传感器" */
#define CN_GAS                          "\xC6\xF8\xCC\xE5\xB4\xAB\xB8\xD0\xC6\xF7"           /* "气体传感器" */
#define CN_RELAY                        "\xBC\xCC\xB5\xE7\xC6\xF7"               /* "继电器" */
#define CN_WIFI                         "\x77\x69\x66\x69"                          /* "wifi" */
#define CN_MQTT_SERVER                  "\x6D\x71\x74\x74\xB7\xFE\xCE\xF1\xC6\xF7" /* "mqtt服务器" */
#define CN_WEB_CLIENT                   "\x77\x65\x62\xB6\xCB"                      /* "web端" */
#define CN_REMOTE_HUMAN                 "\xD4\xB6\xB3\xCC\xBA\xC1\xC3\xD7\xB2\xA8\xC0\xD7\xB4\xEF"   /* "远程毫米波雷达" */
#define CN_REMOTE_LIGHT                 "\xD4\xB6\xB3\xCC\xB9\xE2\xD5\xD5\xB4\xAB\xB8\xD0\xC6\xF7"   /* "远程光照传感器" */
#define CN_REMOTE_DOOR                  "\xD4\xB6\xB3\xCC\xC3\xC5\xB4\xC5\xBF\xAA\xB9\xD8"   /* "远程门磁开关" */
#define CN_ERROR                        "\x45\x52\x52"                             /* "ERR" */
#define CN_PRESENT                      "\xD3\xD0\xC8\xCB"                         /* "有人" */
#define CN_ABSENT                       "\xCE\xDE\xC8\xCB"                         /* "无人" */
#define CN_DOOR_OPEN                    "\xBF\xAA"                                 /* "开" */
#define CN_DOOR_CLOSE                   "\xB9\xD8"                                 /* "关" */
#define CN_STATUS_MANUAL_ON             "\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC\x3A\xCA\xD6\xB6\xAF\xBF\xAA"   /* "当前状态:手动开" */
#define CN_STATUS_MANUAL_OFF            "\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC\x3A\xCA\xD6\xB6\xAF\xB9\xD8"   /* "当前状态:手动关" */
#define CN_STATUS_AUTO_ON               "\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC\x3A\xD7\xD4\xB6\xAF\xBF\xAA"   /* "当前状态:自动开" */
#define CN_STATUS_AUTO_OFF              "\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC\x3A\xD7\xD4\xB6\xAF\xB9\xD8"   /* "当前状态:自动关" */
#define CN_STATUS_MANUAL_ON_PENDING     "\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC\x3A\xCA\xD6\xB6\xAF\xBF\xAA\x28\xD6\xB4\xD0\xD0\xD6\xD0\x29"   /* "当前状态:手动开(执行中)" */
#define CN_STATUS_MANUAL_OFF_PENDING    "\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC\x3A\xCA\xD6\xB6\xAF\xB9\xD8\x28\xD6\xB4\xD0\xD0\xD6\xD0\x29"   /* "当前状态:手动关(执行中)" */
#define CN_RELAY_MANUAL_ON              "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\xBF\xAA"           /* "当前:手动开" */
#define CN_RELAY_MANUAL_OFF             "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\xB9\xD8"           /* "当前:手动关" */
#define CN_RELAY_AUTO_ON                "\xB5\xB1\xC7\xB0\x3A\xD7\xD4\xB6\xAF\xBF\xAA"           /* "当前:自动开" */
#define CN_RELAY_AUTO_OFF               "\xB5\xB1\xC7\xB0\x3A\xD7\xD4\xB6\xAF\xB9\xD8"           /* "当前:自动关" */
#define CN_RELAY_MANUAL_ON_PENDING      "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\xBF\xAA\x28\xD6\xB4\xD0\xD0\xD6\xD0\x29"   /* "当前:手动开(执行中)" */
#define CN_RELAY_MANUAL_OFF_PENDING     "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\xB9\xD8\x28\xD6\xB4\xD0\xD0\xD6\xD0\x29"   /* "当前:手动关(执行中)" */
#define CN_RELAY_FAN_MANUAL_GEAR1       "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\x31\xB5\xB5"       /* "当前:手动1档" */
#define CN_RELAY_FAN_MANUAL_GEAR2       "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\x32\xB5\xB5"       /* "当前:手动2档" */
#define CN_RELAY_FAN_MANUAL_GEAR3       "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\x33\xB5\xB5"       /* "当前:手动3档" */
#define CN_RELAY_FAN_MANUAL_GEAR4       "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\x34\xB5\xB5"       /* "当前:手动4档" */
#define CN_RELAY_FAN_MANUAL_GEAR5       "\xB5\xB1\xC7\xB0\x3A\xCA\xD6\xB6\xAF\x35\xB5\xB5"       /* "当前:手动5档" */
#define CN_RELAY_FAN_AUTO_GEAR1         "\xB5\xB1\xC7\xB0\x3A\xD7\xD4\xB6\xAF\x31\xB5\xB5"       /* "当前:自动1档" */
#define CN_RELAY_FAN_AUTO_GEAR2         "\xB5\xB1\xC7\xB0\x3A\xD7\xD4\xB6\xAF\x32\xB5\xB5"       /* "当前:自动2档" */
#define CN_RELAY_FAN_AUTO_GEAR3         "\xB5\xB1\xC7\xB0\x3A\xD7\xD4\xB6\xAF\x33\xB5\xB5"       /* "当前:自动3档" */
#define CN_RELAY_FAN_AUTO_GEAR4         "\xB5\xB1\xC7\xB0\x3A\xD7\xD4\xB6\xAF\x34\xB5\xB5"       /* "当前:自动4档" */
#define CN_RELAY_FAN_AUTO_GEAR5         "\xB5\xB1\xC7\xB0\x3A\xD7\xD4\xB6\xAF\x35\xB5\xB5"       /* "当前:自动5档" */
#define CN_ACTION_MANUAL_ON             "\xCA\xD6\xB6\xAF\xBF\xAA"               /* "手动开" —— 动作记录用 */
#define CN_ACTION_MANUAL_OFF            "\xCA\xD6\xB6\xAF\xB9\xD8"               /* "手动关" */
#define CN_ACTION_AUTO_ON               "\xD7\xD4\xB6\xAF\xBF\xAA"               /* "自动开" */
#define CN_ACTION_AUTO_OFF              "\xD7\xD4\xB6\xAF\xB9\xD8"               /* "自动关" */
#define CN_DEVICE_HEATER                "\xD6\xC6\xC8\xC8\xC6\xF7"               /* "制热器" */
#define CN_DEVICE_COOLER                "\xD6\xC6\xC0\xE4\xC6\xF7"               /* "制冷器" */
#define CN_DEVICE_HUMIDIFIER            "\xBC\xD3\xCA\xAA\xC6\xF7"               /* "加湿器" */
#define CN_DEVICE_ALARM                 "\xC9\xF9\xB9\xE2\xB1\xA8\xBE\xAF\xC6\xF7"   /* "声光报警器" */
#define CN_DEVICE_LIGHT                 "Led\xB5\xC6\xB9\xE2"                     /* "Led灯光" */
#define CN_DEVICE_FAN                   "\xB7\xE7\xC9\xC8"                         /* "风扇" */

/* ======================== 传感器页 UI 组件 ID（GB2312）======================== */
#define UI_TEMP_ID                      "\xCE\xC2\xB6\xC8"                         /* "温度" —— 温度显示控件 */
#define UI_HUMI_ID                      "\xCA\xAA\xB6\xC8"                         /* "湿度" —— 湿度显示控件 */
#define UI_PM_ID                        "\xB7\xDB\xB3\xBE"                         /* "粉尘" —— PM2.5显示控件 */
#define UI_GAS_ID                       "\xC6\xF8\xCC\xE5"                         /* "气体" —— 气体浓度显示控件 */
#define UI_HUMAN_ID                     "\xC8\xCB\xD4\xB1\xBC\xE0\xB2\xE2"       /* "人员监测" —— 人体在离显示 */
#define UI_DIST_ID                      "\xBE\xE0\xC0\xEB"                         /* "距离" —— 人体距离显示 */
#define UI_LIGHT_ID                     "\xB9\xE2\xD5\xD5"                         /* "光照" —— 光照强度显示 */
#define UI_DOOR_ID                      "\xC3\xC5\xD7\xB4\xCC\xAC"               /* "门状态" —— 门磁开关显示 */
/* ======================== 主页 UI 组件 ID ======================== */
#define UI_INDEX_STATUS_ID              "\xC1\xAC\xBD\xD3\xD7\xB4\xCC\xAC"       /* "连接状态" —— 主页连接状态汇总 */
#define UI_INDEX_DATE_ID                "\xC8\xD5\xC6\xDA"                         /* "日期" —— 主页日期显示 */
#define UI_INDEX_TIME_ID                "\xCA\xB1\xBC\xE4"                         /* "时间" —— 主页时间显示 */
/* ======================== 阈值设置页 UI 组件 ID ======================== */
#define UI_HEATER_THRESHOLD_ID          "\xBC\xD3\xC8\xC8\xC6\xF7\xE3\xD0\xD6\xB5"   /* "加热器阈值" */
#define UI_COOLER_THRESHOLD_ID          "\xD6\xC6\xC0\xE4\xC6\xF7\xE3\xD0\xD6\xB5"   /* "制冷器阈值" */
#define UI_HUMID_THRESHOLD_ID           "\xBC\xD3\xCA\xAA\xC6\xF7\xE3\xD0\xD6\xB5"   /* "加湿器阈值" */
#define UI_DEHUMID_THRESHOLD_ID         "\xB3\xFD\xCA\xAA\xC6\xF7\xE3\xD0\xD6\xB5"   /* "除湿器阈值" */
#define UI_PM25_ALARM_DELAY_ID          "\xB7\xDB\xB3\xBE\xB1\xA8\xBE\xAF\xD1\xD3\xCA\xB1"   /* "粉尘报警延时" */
#define UI_GAS_ALARM_DELAY_ID           "\xC6\xF8\xCC\xE5\xB1\xA8\xBE\xAF\xD1\xD3\xCA\xB1"   /* "气体报警延时" */
#define UI_PM25_ALARM_THRESHOLD_ID      "\xB7\xDB\xB3\xBE\xB1\xA8\xBE\xAF\xE3\xD0\xD6\xB5"   /* "粉尘报警阈值" */
#define UI_GAS_ALARM_THRESHOLD_ID       "\xC6\xF8\xCC\xE5\xB1\xA8\xBE\xAF\xE3\xD0\xD6\xB5"   /* "气体报警阈值" */
#define UI_DOOR_ALARM_THRESHOLD_ID      "\xC3\xC5\xCE\xB4\xB9\xD8\xB1\xD5\xBE\xAF\xB1\xA8"   /* "门未关闭警报" */
#define UI_LIGHT_ON_THRESHOLD_ID        "\xB9\xE2\xB8\xD0\xC6\xF4\xB5\xC6\xE3\xD0\xD6\xB5"   /* "光感启灯阈值" */
#define UI_HUMAN_LIGHT_THRESHOLD_ID     "\xC8\xCB\xB8\xD0\xBF\xAA\xB5\xC6\xE3\xD0\xD6\xB5"   /* "人感开灯阈值" */
/* ======================== 风扇档位转速 UI 组件 ID ======================== */
#define UI_FAN_LEVEL1_SPEED_ID          "\xB7\xE7\xC9\xC8\x31\xB5\xB5\xD7\xAA\xCB\xD9"   /* "风扇1档转速" */
#define UI_FAN_LEVEL2_SPEED_ID          "\xB7\xE7\xC9\xC8\x32\xB5\xB5\xD7\xAA\xCB\xD9"   /* "风扇2档转速" */
#define UI_FAN_LEVEL3_SPEED_ID          "\xB7\xE7\xC9\xC8\x33\xB5\xB5\xD7\xAA\xCB\xD9"   /* "风扇3档转速" */
#define UI_FAN_LEVEL4_SPEED_ID          "\xB7\xE7\xC9\xC8\x34\xB5\xB5\xD7\xAA\xCB\xD9"   /* "风扇4档转速" */
#define UI_FAN_LEVEL5_SPEED_ID          "\xB7\xE7\xC9\xC8\x35\xB5\xB5\xD7\xAA\xCB\xD9"   /* "风扇5档转速" */
/* ======================== PM2.5 五档区间 UI 组件 ID ======================== */
#define UI_PM_L1_LOW_ID                 "\xB7\xDB\xB3\xBE\xD2\xBB\xB5\xB5\xCF\xC2\xCF\xDE"   /* "粉尘一档下限" */
#define UI_PM_L1_HIGH_ID                "\xB7\xDB\xB3\xBE\xD2\xBB\xB5\xB5\xC9\xCF\xCF\xDE"   /* "粉尘一档上限" */
#define UI_PM_L2_LOW_ID                 "\xB7\xDB\xB3\xBE\xB6\xFE\xB5\xB5\xCF\xC2\xCF\xDE"   /* "粉尘二档下限" */
#define UI_PM_L2_HIGH_ID                "\xB7\xDB\xB3\xBE\xB6\xFE\xB5\xB5\xC9\xCF\xCF\xDE"   /* "粉尘二档上限" */
#define UI_PM_L3_LOW_ID                 "\xB7\xDB\xB3\xBE\xC8\xFD\xB5\xB5\xCF\xC2\xCF\xDE"   /* "粉尘三档下限" */
#define UI_PM_L3_HIGH_ID                "\xB7\xDB\xB3\xBE\xC8\xFD\xB5\xB5\xC9\xCF\xCF\xDE"   /* "粉尘三档上限" */
#define UI_PM_L4_LOW_ID                 "\xB7\xDB\xB3\xBE\xCB\xC4\xB5\xB5\xCF\xC2\xCF\xDE"   /* "粉尘四档下限" */
#define UI_PM_L4_HIGH_ID                "\xB7\xDB\xB3\xBE\xCB\xC4\xB5\xB5\xC9\xCF\xCF\xDE"   /* "粉尘四档上限" */
#define UI_PM_L5_LOW_ID                 "\xB7\xDB\xB3\xBE\xCE\xE5\xB5\xB5\xCF\xC2\xCF\xDE"   /* "粉尘五档下限" */
#define UI_PM_L5_HIGH_ID                "\xB7\xDB\xB3\xBE\xCE\xE5\xB5\xB5\xC9\xCF\xCF\xDE"   /* "粉尘五档上限" */
/* ======================== 气体五档区间 UI 组件 ID ======================== */
#define UI_GAS_L1_LOW_ID                "\xC6\xF8\xCC\xE5\xD2\xBB\xB5\xB5\xCF\xC2\xCF\xDE"   /* "气体一档下限" */
#define UI_GAS_L1_HIGH_ID               "\xC6\xF8\xCC\xE5\xD2\xBB\xB5\xB5\xC9\xCF\xCF\xDE"   /* "气体一档上限" */
#define UI_GAS_L2_LOW_ID                "\xC6\xF8\xCC\xE5\xB6\xFE\xB5\xB5\xCF\xC2\xCF\xDE"   /* "气体二档下限" */
#define UI_GAS_L2_HIGH_ID               "\xC6\xF8\xCC\xE5\xB6\xFE\xB5\xB5\xC9\xCF\xCF\xDE"   /* "气体二档上限" */
#define UI_GAS_L3_LOW_ID                "\xC6\xF8\xCC\xE5\xC8\xFD\xB5\xB5\xCF\xC2\xCF\xDE"   /* "气体三档下限" */
#define UI_GAS_L3_HIGH_ID               "\xC6\xF8\xCC\xE5\xC8\xFD\xB5\xB5\xC9\xCF\xCF\xDE"   /* "气体三档上限" */
#define UI_GAS_L4_LOW_ID                "\xC6\xF8\xCC\xE5\xCB\xC4\xB5\xB5\xCF\xC2\xCF\xDE"   /* "气体四档下限" */
#define UI_GAS_L4_HIGH_ID               "\xC6\xF8\xCC\xE5\xCB\xC4\xB5\xB5\xC9\xCF\xCF\xDE"   /* "气体四档上限" */
#define UI_GAS_L5_LOW_ID                "\xC6\xF8\xCC\xE5\xCE\xE5\xB5\xB5\xCF\xC2\xCF\xDE"   /* "气体五档下限" */
#define UI_GAS_L5_HIGH_ID               "\xC6\xF8\xCC\xE5\xCE\xE5\xB5\xB5\xC9\xCF\xCF\xDE"   /* "气体五档上限" */
/* ======================== 手动控制页/继电器页/设置页 UI 组件 ID ======================== */
#define UI_HAND_HEATER_STATUS_ID        "t3"                                       /* 手动控制页——第一行设备状态显示 */
#define UI_HAND_COOLER_STATUS_ID        "t8"                                       /* 手动控制页——第二行设备状态显示 */
#define UI_RELAY_HEATER_STATUS_ID       "\xD6\xC6\xC8\xC8\xC6\xF7\xD7\xB4\xCC\xAC"   /* "制热器状态" —— 继电器总览页 */
#define UI_RELAY_COOLER_STATUS_ID       "\xD6\xC6\xC0\xE4\xC6\xF7\xD7\xB4\xCC\xAC"   /* "制冷器状态" */
#define UI_RELAY_HUMID_STATUS_ID        "\xBC\xD3\xCA\xAA\xC6\xF7\xD7\xB4\xCC\xAC"   /* "加湿器状态" */
#define UI_RELAY_ALARM_STATUS_ID        "\xB5\xC6\xB9\xE2\xB1\xA8\xBE\xAF\xC6\xF7\xD7\xB4\xCC\xAC"   /* "灯光报警器状态" */
#define UI_RELAY_LIGHT_STATUS_ID        "led\xB5\xC6\xB9\xE2\xD7\xB4\xCC\xAC"       /* "led灯光状态" */
#define UI_RELAY_FAN_STATUS_ID          "\xB7\xE7\xC9\xC8\xD7\xB4\xCC\xAC"           /* "风扇状态" */
#define UI_SYSTEM_UPTIME_ID             "\xCF\xB5\xCD\xB3\xD4\xCB\xD0\xD0\xCA\xB1\xBC\xE4"   /* "系统运行时间" —— 设置页 */

/* ======================== 静态变量（模块内部状态） ======================== */
static uint8_t s_frame_window[3] = {0U, 0U, 0U};          /**< 3字节滑动窗口，用于检测页面切换帧 [0x55, pageID, 0xAA] */
static uint8_t s_frame_window7[SCREEN_CHANGE1_FRAME_LEN] = {0U};  /**< 7字节滑动窗口，用于检测阈值设置帧和自动恢复帧 */
static uint8_t s_frame_window8[SCREEN_MANUAL_SAVE_FRAME_LEN] = {0U};  /**< 8字节滑动窗口，用于检测手动控制帧和恢复出厂帧 */
static ScreenActionEvent_t s_action_queue[SCREEN_ACTION_QUEUE_SIZE]; /**< 设备动作事件环形队列，供 MQTT 上报 */
static uint8_t s_action_queue_head = 0U;                   /**< 队列写入位置（下一个写入索引） */
static uint8_t s_action_queue_tail = 0U;                   /**< 队列读取位置（下一个读取索引） */
static uint8_t s_action_queue_count = 0U;                  /**< 队列中当前事件数量 */
static SemaphoreHandle_t s_screen_tx_mutex = NULL;         /**< UART4 发送互斥锁，保护多任务并发发送 */
static uint8_t s_active_page = SCREEN_CMD_INDEX_PAGE;      /**< 当前活动页面 ID，决定刷新内容 */
static uint8_t s_immediate_refresh_pending = 0U;           /**< 是否强制马上刷新标志 */
static uint8_t s_force_text_send = 0U;                     /**< 强制文本发送标志（跳过缓存去重，确保页面切换后刷新） */
static char s_time_local_str[20] = {0};                    /**< 本地时间完整字符串 "YYYY-MM-DD HH:MM:SS" */
static char s_time_tz_str[8] = {0};                        /**< 时区字符串, 如 "+08:00" */
static char s_time_date_str[20] = {'-', '-', '-', '-', '/', '-', '-', '/', '-', '-', '\0'}; /**< 格式化日期字符串(显示用，GB2312年月日) */
static char s_time_hm_str[6] = {'-', '-', ':', '-', '-', '\0'};  /**< 格式化时分字符串 "HH:MM" */
static uint32_t s_time_ts = 0U;                            /**< 最后收到的 UNIX 时间戳 */
static uint8_t s_time_has_ts = 0U;                         /**< UNIX 时间戳是否有效 */
static uint8_t s_time_valid = 0U;                          /**< 本地时钟是否已同步有效 */
static uint8_t s_relay_connected_cache = 0U;               /**< 继电器连接状态缓存（避免频繁 Modbus 查询） */
static TickType_t s_relay_poll_last_tick = 0U;              /**< 继电器状态上次轮询的 Tick 时间戳 */
static TickType_t s_reset_defaults_last_tick = 0U;          /**< 最近一次处理恢复出厂帧的 Tick 时间戳 */

/**
 * @brief 文本缓存槽位结构体
 *
 * 用于去重机制：缓存每个 UI 组件的最后发送值，
 * 只有当内容变化时才通过 UART 重新发送，减少通信负担。
 */
typedef struct
{
    uint8_t used;                                  /**< 槽位是否已使用: 1=占用, 0=空闲 */
    char id[64];                                   /**< UI 组件 ID 字符串（屏幕控件） */
    char value[SCREEN_TEXT_CACHE_VALUE_MAX];        /**< 上次发送的文本值 */
} ScreenTextCacheEntry_t;

static ScreenTextCacheEntry_t s_text_cache[SCREEN_TEXT_CACHE_SLOTS]; /**< 文本缓存数组 */

/**
 * @brief 本地时钟结构体
 *
 * 基于 FreeRTOS Tick 实现的软件时钟，每秒自增一次，
 * 支持 NTP 同步校准。用于屏幕日期时间显示和动作记录时间戳。
 */
typedef struct
{
    uint16_t year;                                 /**< 年 (e.g. 2026) */
    uint8_t month;                                 /**< 月 (1~12) */
    uint8_t day;                                   /**< 日 (1~31) */
    uint8_t hour;                                  /**< 时 (0~23) */
    uint8_t minute;                                /**< 分 (0~59) */
    uint8_t second;                                /**< 秒 (0~59) */
    TickType_t last_tick;                           /**< 上次更新时的 FreeRTOS Tick 计数 */
    TickType_t last_sync_tick;                      /**< 上次 NTP 同步时的 Tick 计数 */
    uint8_t valid;                                  /**< 时钟是否有效: 1=已同步, 0=未初始化 */
} ScreenLocalClock_t;

static ScreenLocalClock_t s_local_clock = {0};      /**< 本地时钟实例 */

/**
 * @brief   向动作事件队列推入一条记录
 * @param   device      被操作的设备类型
 * @param   is_manual    操作模式: 1=手动, 0=自动
 * @param   action_value 操作值: 开关类为0/1, 风扇为档位1~5
 * @param   date_text   日期字符串 (可为 NULL)
 * @param   time_text   时间字符串 (可为 NULL)
 *
 * 环形队列，队列满时自动覆盖最旧的事件。
 * 使用临界区保护，确保中断安全。
 */
static void Screen_ActionQueuePush(ScreenActionDevice_t device,
                                   uint8_t is_manual,
                                   uint8_t action_value,
                                   const char *date_text,
                                   const char *time_text)
{
    ScreenActionEvent_t *slot;

    taskENTER_CRITICAL();  /* 进入临界区，保护队列操作的原子性 */

    if (s_action_queue_count >= SCREEN_ACTION_QUEUE_SIZE)
    {
        /* 队列已满，丢弃最旧的一条记录（移动尾指针） */
        s_action_queue_tail = (uint8_t)((s_action_queue_tail + 1U) % SCREEN_ACTION_QUEUE_SIZE);
        s_action_queue_count--;
    }

    slot = &s_action_queue[s_action_queue_head];
    slot->device = device;
    slot->is_manual = (is_manual != 0U) ? 1U : 0U;
    slot->action_value = action_value;

    if (date_text != NULL)
    {
        strncpy(slot->date_text, date_text, sizeof(slot->date_text) - 1U);
        slot->date_text[sizeof(slot->date_text) - 1U] = '\0';
    }
    else
    {
        slot->date_text[0] = '\0';
    }

    if (time_text != NULL)
    {
        strncpy(slot->time_text, time_text, sizeof(slot->time_text) - 1U);
        slot->time_text[sizeof(slot->time_text) - 1U] = '\0';
    }
    else
    {
        slot->time_text[0] = '\0';
    }

    /* 写入指针前进并更新计数 */
    s_action_queue_head = (uint8_t)((s_action_queue_head + 1U) % SCREEN_ACTION_QUEUE_SIZE);
    s_action_queue_count++;

    taskEXIT_CRITICAL();   /* 退出临界区 */
}

/**
 * @brief   判断是否为闰年
 * @param   year  年份
 * @return  1=闰年, 0=非闰年
 */
static uint8_t Screen_IsLeapYear(uint16_t year)
{
    if ((year % 400U) == 0U)
    {
        return 1U;
    }
    if ((year % 100U) == 0U)
    {
        return 0U;
    }
    return ((year % 4U) == 0U) ? 1U : 0U;
}

/**
 * @brief   获取指定年月的天数
 * @param   year   年份
 * @param   month  月份 (1~12)
 * @return  该月的天数，参数无效返回 0
 */
static uint8_t Screen_DaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] = {
        31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U
    };

    if ((month < 1U) || (month > 12U))
    {
        return 0U;
    }

    if ((month == 2U) && (Screen_IsLeapYear(year) != 0U))
    {
        return 29U;
    }
    return days[month - 1U];
}

/**
 * @brief   解析字符串中的2位十进制数字
 * @param   s    指向字符串的指针（至少 2 个字符）
 * @param   out  解析结果输出
 * @return  1=解析成功, 0=解析失败
 */
static uint8_t Screen_Parse2Digits(const char *s, uint8_t *out)
{
    if ((s == NULL) || (out == NULL))
    {
        return 0U;
    }
    if ((s[0] < '0') || (s[0] > '9') || (s[1] < '0') || (s[1] > '9'))
    {
        return 0U;
    }
    *out = (uint8_t)((uint8_t)(s[0] - '0') * 10U + (uint8_t)(s[1] - '0'));
    return 1U;
}

/**
 * @brief   解析字符串中的4位十进制数字（用于年份）
 * @param   s    指向字符串的指针（至少 4 个字符）
 * @param   out  解析结果输出
 * @return  1=解析成功, 0=解析失败
 */
static uint8_t Screen_Parse4Digits(const char *s, uint16_t *out)
{
    uint16_t value;

    if ((s == NULL) || (out == NULL))
    {
        return 0U;
    }
    if ((s[0] < '0') || (s[0] > '9') ||
        (s[1] < '0') || (s[1] > '9') ||
        (s[2] < '0') || (s[2] > '9') ||
        (s[3] < '0') || (s[3] > '9'))
        {
        return 0U;
    }

    value = (uint16_t)(s[0] - '0');
    value = (uint16_t)(value * 10U + (uint16_t)(s[1] - '0'));
    value = (uint16_t)(value * 10U + (uint16_t)(s[2] - '0'));
    value = (uint16_t)(value * 10U + (uint16_t)(s[3] - '0'));
    *out = value;
    return 1U;
}

/**
 * @brief   解析 "YYYY-MM-DD HH:MM[:SS]" 格式的本地时间字符串
 * @param   local_str  时间字符串，最少 16 字符，秒可选
 * @param   out_clock  解析结果输出到此时钟结构体
 * @return  1=解析成功且日期时间合法, 0=失败
 */
static uint8_t Screen_ParseLocalDateTime(const char *local_str, ScreenLocalClock_t *out_clock)
{
    size_t len;
    uint8_t days;

    if ((local_str == NULL) || (out_clock == NULL))
    {
        return 0U;
    }

    len = strlen(local_str);
    if (len < 16U)
    {
        return 0U;
    }
    if ((local_str[4] != '-') ||
        (local_str[7] != '-') ||
        (local_str[10] != ' ') ||
        (local_str[13] != ':'))
        {
        return 0U;
    }

    memset(out_clock, 0, sizeof(*out_clock));
    if (!Screen_Parse4Digits(&local_str[0], &out_clock->year) ||
        !Screen_Parse2Digits(&local_str[5], &out_clock->month) ||
        !Screen_Parse2Digits(&local_str[8], &out_clock->day) ||
        !Screen_Parse2Digits(&local_str[11], &out_clock->hour) ||
        !Screen_Parse2Digits(&local_str[14], &out_clock->minute))
        {
        return 0U;
    }

    if ((len >= 19U) && (local_str[16] == ':'))
    {
        if (!Screen_Parse2Digits(&local_str[17], &out_clock->second))
        {
            return 0U;
        }
    }
    else
    {
        out_clock->second = 0U;
    }

    days = Screen_DaysInMonth(out_clock->year, out_clock->month);
    if ((days == 0U) ||
        (out_clock->day < 1U) ||
        (out_clock->day > days) ||
        (out_clock->hour > 23U) ||
        (out_clock->minute > 59U) ||
        (out_clock->second > 59U))
        {
        return 0U;
    }

    out_clock->valid = 1U;
    return 1U;
}

/**
 * @brief   格式化本地时钟到显示字符串（在临界区内调用）
 *
 * 将 s_local_clock 的年月日时分秒格式化到:
 *   - s_time_local_str: "YYYY-MM-DD HH:MM:SS"
 *   - s_time_date_str:  "YYYY年MM月DD日" (GB2312)
 *   - s_time_hm_str:    "HH:MM"
 */
static void Screen_FormatLocalClockLocked(void)
{
    if (s_local_clock.valid == 0U)
    {
        snprintf(s_time_date_str, sizeof(s_time_date_str), "----/--/--");
        memcpy(s_time_hm_str, "--:--", 6U);
        s_time_valid = 0U;
        return;
    }

    snprintf(s_time_local_str,
             sizeof(s_time_local_str),
             "%04u-%02u-%02u %02u:%02u:%02u",
             (unsigned int)s_local_clock.year,
             (unsigned int)s_local_clock.month,
             (unsigned int)s_local_clock.day,
             (unsigned int)s_local_clock.hour,
             (unsigned int)s_local_clock.minute,
             (unsigned int)s_local_clock.second);
    snprintf(s_time_date_str,
             sizeof(s_time_date_str),
             "%04u\xC4\xEA%02u\xD4\xC2%02u\xC8\xD5",  /* 格式: YYYY年MM月DD日 (GB2312编码) */
             (unsigned int)s_local_clock.year,
             (unsigned int)s_local_clock.month,
             (unsigned int)s_local_clock.day);
    snprintf(s_time_hm_str,
             sizeof(s_time_hm_str),
             "%02u:%02u",
             (unsigned int)s_local_clock.hour,
             (unsigned int)s_local_clock.minute);
    s_time_valid = 1U;
}

/**
 * @brief   本地时钟加一秒（在临界区内调用）
 *
 * 处理秒/分/时/日/月/年的进位和溢出，含闰年处理。
 */
static void Screen_LocalClockAddOneSecondLocked(void)
{
    uint8_t days;

    if (s_local_clock.valid == 0U)
    {
        return;
    }

    s_local_clock.second++;
    if (s_local_clock.second < 60U)
    {
        return;
    }
    s_local_clock.second = 0U;

    s_local_clock.minute++;
    if (s_local_clock.minute < 60U)
    {
        return;
    }
    s_local_clock.minute = 0U;

    s_local_clock.hour++;
    if (s_local_clock.hour < 24U)
    {
        return;
    }
    s_local_clock.hour = 0U;

    days = Screen_DaysInMonth(s_local_clock.year, s_local_clock.month);
    s_local_clock.day++;
    if (s_local_clock.day <= days)
    {
        return;
    }
    s_local_clock.day = 1U;

    s_local_clock.month++;
    if (s_local_clock.month <= 12U)
    {
        return;
    }
    s_local_clock.month = 1U;
    s_local_clock.year++;
}

/**
 * @brief   为时钟结构体批量加若干秒
 * @param   clock    目标时钟结构体
 * @param   seconds  要增加的秒数
 *
 * 用于 NTP 时间接收延迟补偿，逐10加秒以确保日期进位正确。
 */
static void Screen_LocalClockAddSeconds(ScreenLocalClock_t *clock, uint16_t seconds)
{
    uint16_t i;
    uint8_t days;

    if ((clock == NULL) || (clock->valid == 0U) || (seconds == 0U))
    {
        return;
    }

    for (i = 0U; i < seconds; i++)
    {
        clock->second++;
        if (clock->second < 60U)
        {
            continue;
        }
        clock->second = 0U;

        clock->minute++;
        if (clock->minute < 60U)
        {
            continue;
        }
        clock->minute = 0U;

        clock->hour++;
        if (clock->hour < 24U)
        {
            continue;
        }
        clock->hour = 0U;

        days = Screen_DaysInMonth(clock->year, clock->month);
        clock->day++;
        if (clock->day <= days)
        {
            continue;
        }
        clock->day = 1U;

        clock->month++;
        if (clock->month <= 12U)
        {
            continue;
        }
        clock->month = 1U;
        clock->year++;
    }
}

/**
 * @brief   基于 FreeRTOS Tick 更新本地时钟
 *
 * 计算距上次更新经过的秒数，逐秒递增本地时钟，
 * 然后重新格式化显示字符串。
 */
static void Screen_TimeUpdateFromTick(void)
{
    const TickType_t one_sec_ticks = (pdMS_TO_TICKS(1000U) == 0U) ? 1U : pdMS_TO_TICKS(1000U);
    TickType_t now;
    TickType_t elapsed;
    TickType_t add_secs;
    TickType_t i;

    now = xTaskGetTickCount();

    taskENTER_CRITICAL();
    if (s_local_clock.valid == 0U)
    {
        taskEXIT_CRITICAL();
        return;
    }

    elapsed = now - s_local_clock.last_tick;
    add_secs = elapsed / one_sec_ticks;
    if (add_secs == 0U)
    {
        taskEXIT_CRITICAL();
        return;
    }

    for (i = 0U; i < add_secs; i++)
    {
        Screen_LocalClockAddOneSecondLocked();
    }
    s_local_clock.last_tick += (add_secs * one_sec_ticks);
    Screen_FormatLocalClockLocked();
    taskEXIT_CRITICAL();
}

/**
 * @brief   发送原始 Nextion 指令（不加锁）
 * @param   cmd  Nextion 指令字符串
 *
 * 发送指令后追加 3 个 0xFF 作为 Nextion 协议结束标识。
 */
static void Screen_SendCommandRaw(const char *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    UART_Screen_SendString(cmd);
    UART_Screen_SendByte(0xFFU);
    UART_Screen_SendByte(0xFFU);
    UART_Screen_SendByte(0xFFU);
}

/**
 * @brief   发送 Nextion 指令（带互斥锁保护）
 * @param   cmd  Nextion 指令字符串
 *
 * 多任务并发安全，自动获取/释放 TX 互斥锁。
 */
static void Screen_SendCommand(const char *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    if ((s_screen_tx_mutex != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
        {
        (void)xSemaphoreTake(s_screen_tx_mutex, portMAX_DELAY);
    }

    Screen_SendCommandRaw(cmd);

    if ((s_screen_tx_mutex != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
        {
        (void)xSemaphoreGive(s_screen_tx_mutex);
    }
}

/**
 * @brief   重置所有文本缓存槽位
 *
 * 页面切换时调用，清除所有缓存确保新页面内容全部重新发送。
 */
static void Screen_TextCacheReset(void)
{
    memset(s_text_cache, 0, sizeof(s_text_cache));
}

/**
 * @brief   在文本缓存中查找指定组件 ID 的槽位
 * @param   component_id  UI 组件 ID 字符串
 * @return  匹配的槽位指针，或第一个空闲槽位，或 NULL（缓存已满且未命中）
 */
static ScreenTextCacheEntry_t *Screen_TextCacheFind(const char *component_id)
{
    uint8_t i;
    ScreenTextCacheEntry_t *free_slot = NULL;

    if (component_id == NULL)
    {
        return NULL;
    }

    for (i = 0U; i < SCREEN_TEXT_CACHE_SLOTS; i++)
    {
        if (s_text_cache[i].used == 0U)
        {
            if (free_slot == NULL)
            {
                free_slot = &s_text_cache[i];
            }
            continue;
        }

        if (strncmp(s_text_cache[i].id, component_id, sizeof(s_text_cache[i].id)) == 0)
        {
            return &s_text_cache[i];
        }
    }

    return free_slot;
}

/**
 * @brief   判断文本是否需要发送（去重逻辑核心）
 * @param   component_id  UI 组件 ID
 * @param   value         待发送的文本值
 * @return  1=内容已变化需发送, 0=与缓存相同无需发送
 *
 * 如果内容已变化或槽位未使用，会同时更新缓存内容。
 */
static uint8_t Screen_TextCacheNeedSend(const char *component_id, const char *value)
{
    ScreenTextCacheEntry_t *slot;

    if ((component_id == NULL) || (value == NULL))
    {
        return 0U;
    }

    slot = Screen_TextCacheFind(component_id);
    if (slot == NULL)
    {
        /* Cache full: keep sending to ensure screen state stays correct. */
        return 1U;
    }

    if ((slot->used != 0U) &&
        (strncmp(slot->value, value, sizeof(slot->value)) == 0))
        {
        return 0U;
    }

    strncpy(slot->id, component_id, sizeof(slot->id) - 1U);
    slot->id[sizeof(slot->id) - 1U] = '\0';
    strncpy(slot->value, value, sizeof(slot->value) - 1U);
    slot->value[sizeof(slot->value) - 1U] = '\0';
    slot->used = 1U;
    return 1U;
}

/**
 * @brief   获取继电器连接状态（带轮询间隔缓存）
 * @return  1=继电器已连接, 0=未连接
 *
 * 每隔 SCREEN_RELAY_POLL_INTERVAL_MS 才实际查询 Modbus，
 * 中间返回缓存值，避免频繁占用总线。
 */
static uint8_t Screen_GetRelayConnectedCached(void)
{
    TickType_t now_tick;
    TickType_t interval_tick;
    uint8_t relay_states[RELAY_CHANNEL_COUNT] = {0U};

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return s_relay_connected_cache;
    }

    now_tick = xTaskGetTickCount();
    interval_tick = pdMS_TO_TICKS(SCREEN_RELAY_POLL_INTERVAL_MS);
    if (interval_tick == 0U)
    {
        interval_tick = 1U;
    }

    if ((s_relay_poll_last_tick == 0U) ||
        ((now_tick - s_relay_poll_last_tick) >= interval_tick))
        {
        s_relay_connected_cache = ReadRelayStatus(relay_states, RELAY_CHANNEL_COUNT);
        s_relay_poll_last_tick = now_tick;
    }

    return s_relay_connected_cache;
}

/**
 * @brief   判断页面命令是否为已知页面 ID
 * @param   page_cmd  页面命令字节
 * @return  1=已知页面, 0=未知页面
 */
static uint8_t Screen_IsKnownPage(uint8_t page_cmd)
{
    if ((page_cmd == SCREEN_CMD_INDEX_PAGE) ||
        (page_cmd == SCREEN_CMD_SENSOR_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE1_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE2_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE3_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE4_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE5_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE6_PAGE) ||
        (page_cmd == SCREEN_CMD_HAND1_PAGE) ||
        (page_cmd == SCREEN_CMD_HAND2_PAGE) ||
        (page_cmd == SCREEN_CMD_HAND3_PAGE) ||
        (page_cmd == SCREEN_CMD_HAND4_PAGE) ||
        (page_cmd == SCREEN_CMD_RELAY_PAGE) ||
        (page_cmd == SCREEN_CMD_RELAY_PAGE_ALT) ||
        (page_cmd == SCREEN_CMD_SETTING_PAGE) ||
        (page_cmd == SCREEN_CMD_SETTING_PAGE_ALT))
        {
        return 1U;
    }
    return 0U;
}

/**
 * @brief   请求即时刷新屏幕（不等待周期定时器）
 */
static void Screen_RequestImmediateRefresh(void)
{
    if (s_immediate_refresh_pending < 255U)
    {
        s_immediate_refresh_pending++;
    }
}

/**
 * @brief   处理页面切换命令
 * @param   page_cmd  收到的页面ID字节
 *
 * 校验页面ID是否合法，处理备选页面ID映射，
 * 页面变化时清除文本缓存并触发即时刷新。
 */
static void Screen_ProcessPageCommand(uint8_t page_cmd)
{
    uint8_t old_page = s_active_page;

    if (!Screen_IsKnownPage(page_cmd))
    {
        /* Unknown page id: stop index periodic refresh to avoid wrong-page flooding. */
        s_active_page = SCREEN_CMD_NONE_PAGE;
        return;
    }

    if (page_cmd == SCREEN_CMD_RELAY_PAGE_ALT)
    {
        page_cmd = SCREEN_CMD_RELAY_PAGE;
    }
    else if (page_cmd == SCREEN_CMD_SETTING_PAGE_ALT)
    {
        page_cmd = SCREEN_CMD_SETTING_PAGE;
    }

    s_active_page = page_cmd;
    if (old_page != s_active_page)
    {
        Screen_TextCacheReset();
    }
    if (s_force_text_send < 255U)
    {
        s_force_text_send++;
    }
    Screen_RequestImmediateRefresh();
}

/**
 * @brief   小端字节序解码 4 字节为 32 位无符号整数
 * @param   bytes4  指向 4 字节数据的指针 (LSB first)
 * @return  解码后的 uint32_t 值
 */
static uint32_t Screen_DecodeU32LE(const uint8_t *bytes4)
{
    if (bytes4 == NULL)
    {
        return 0U;
    }

    return ((uint32_t)bytes4[0]) |
           ((uint32_t)bytes4[1] << 8) |
           ((uint32_t)bytes4[2] << 16) |
           ((uint32_t)bytes4[3] << 24);
}

/**
 * @brief   发送文本到串口屏组件（带缓存去重 + 互斥锁）
 * @param   component_id  UI 组件 ID 字符串
 * @param   value         要显示的文本值
 *
 * 如果文本与缓存相同且未强制发送，则跳过。
 * 生成 Nextion 指令: component_id.txt="value" + 0xFF 0xFF 0xFF
 */
static void Screen_SendText(const char *component_id, const char *value)
{
    char cmd[512];
    int len;
    uint8_t changed;

    if ((component_id == NULL) || (value == NULL))
    {
        return;
    }

    changed = Screen_TextCacheNeedSend(component_id, value);
    if ((changed == 0U) && (s_force_text_send == 0U))
    {
        return;
    }

    len = snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", component_id, value);
    if ((len > 0) && ((size_t)len < sizeof(cmd)))
    {
        Screen_SendCommand(cmd);
    }
}

/**
 * @brief   发送文本到串口屏组件（不加互斥锁、不缓存去重）
 * @param   component_id  UI 组件 ID
 * @param   value         文本值
 *
 * 用于已持有互斥锁的场景（如 Screen_RecordDeviceAction 内部）。
 */
static void Screen_SendTextNoLock(const char *component_id, const char *value)
{
    char cmd[512];
    int len;

    if ((component_id == NULL) || (value == NULL))
    {
        return;
    }

    len = snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", component_id, value);
    if ((len > 0) && ((size_t)len < sizeof(cmd)))
    {
        Screen_SendCommandRaw(cmd);
    }
}

/**
 * @brief   根据设备类型获取对应的中文名称字符串
 * @param   device  设备类型枚举
 * @return  GB2312 编码的设备名称字符串
 */
static const char *Screen_ActionDeviceText(ScreenActionDevice_t device)
{
    switch (device)
    {
    case SCREEN_ACTION_DEV_HEATER:
        return CN_DEVICE_HEATER;
    case SCREEN_ACTION_DEV_COOLER:
        return CN_DEVICE_COOLER;
    case SCREEN_ACTION_DEV_HUMIDIFIER:
        return CN_DEVICE_HUMIDIFIER;
    case SCREEN_ACTION_DEV_ALARM:
        return CN_DEVICE_ALARM;
    case SCREEN_ACTION_DEV_LIGHT:
        return CN_DEVICE_LIGHT;
    case SCREEN_ACTION_DEV_FAN:
        return CN_DEVICE_FAN;
    default:
        return "";
    }
}

/**
 * @brief   获取开关类设备的动作状态文本
 * @param   is_manual    1=手动, 0=自动
 * @param   on           1=开, 0=关
 * @return  GB2312 状态文本，如 "手动开"/"自动关" 等
 */
static const char *Screen_ActionRelayText(uint8_t is_manual, uint8_t on)
{
    if (is_manual != 0U)
    {
        return (on != 0U) ? CN_ACTION_MANUAL_ON : CN_ACTION_MANUAL_OFF;
    }
    return (on != 0U) ? CN_ACTION_AUTO_ON : CN_ACTION_AUTO_OFF;
}

/**
 * @brief   获取风扇动作的档位文本
 * @param   is_manual      1=手动, 0=自动
 * @param   gear_1_to_5    风扇档位 (1~5)
 * @return  GB2312 档位文本，如 "手动1档"/"自动3档"
 */
static const char *Screen_ActionFanText(uint8_t is_manual, uint8_t gear_1_to_5)
{
    uint8_t gear;
    static const char *manual_text[5] = {
        "\xCA\xD6\xB6\xAF\x31\xB5\xB5",
        "\xCA\xD6\xB6\xAF\x32\xB5\xB5",
        "\xCA\xD6\xB6\xAF\x33\xB5\xB5",
        "\xCA\xD6\xB6\xAF\x34\xB5\xB5",
        "\xCA\xD6\xB6\xAF\x35\xB5\xB5"
    };
    static const char *auto_text[5] = {
        "\xD7\xD4\xB6\xAF\x31\xB5\xB5",
        "\xD7\xD4\xB6\xAF\x32\xB5\xB5",
        "\xD7\xD4\xB6\xAF\x33\xB5\xB5",
        "\xD7\xD4\xB6\xAF\x34\xB5\xB5",
        "\xD7\xD4\xB6\xAF\x35\xB5\xB5"
    };

    gear = gear_1_to_5;
    if (gear < 1U)
    {
        gear = 1U;
    }
    else if (gear > 5U)
    {
        gear = 5U;
    }

    if (is_manual != 0U)
    {
        return manual_text[gear - 1U];
    }
    return auto_text[gear - 1U];
}

/**
 * @brief   获取当前日期和时分字符串（用于 UI 显示）
 * @param   date_out  输出日期字符串，至少 20 字节
 * @param   hm_out    输出时分字符串，至少 6 字节
 *
 * 先更新本地时钟，再在临界区内拷贝格式化后的字符串。
 */
static void Screen_GetDateTimeForUi(char date_out[20], char hm_out[6])
{
    uint8_t valid;

    Screen_TimeUpdateFromTick();

    taskENTER_CRITICAL();
    valid = s_time_valid;
    if (valid != 0U)
    {
        memcpy(date_out, s_time_date_str, 20U);
        memcpy(hm_out, s_time_hm_str, 6U);
    }
    else
    {
        snprintf(date_out, 20U, "----/--/--");
        memcpy(hm_out, "--:--", 6U);
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief   从 MQTT 时间消息同步本地时钟
 * @param   ts         UNIX 时间戳
 * @param   has_ts     时间戳是否有效
 * @param   local_str  本地时间字符串 "YYYY-MM-DD HH:MM[:SS]"
 * @param   tz_str     时区字符串
 *
 * 解析时间字符串后与本地时钟对比，仅在偏差超过阈值时才校准，
 * 含接收延迟补偿。校准后触发屏幕即时刷新。
 */
void Screen_SetTimeFromMqtt(uint32_t ts,
                            uint8_t has_ts,
                            const char *local_str,
                            const char *tz_str)
{
    char local_buf[20] = {0};
    char tz_buf[8] = {0};
    ScreenLocalClock_t parsed_clock;
    TickType_t now;
    uint8_t parsed_ok;
    uint8_t should_calibrate;
    uint8_t sec_diff;
    size_t local_len = 0U;

    if (local_str != NULL)
    {
        strncpy(local_buf, local_str, sizeof(local_buf) - 1U);
        local_buf[sizeof(local_buf) - 1U] = '\0';
        local_len = strlen(local_buf);
    }

    if (tz_str != NULL)
    {
        strncpy(tz_buf, tz_str, sizeof(tz_buf) - 1U);
        tz_buf[sizeof(tz_buf) - 1U] = '\0';
    }

    (void)local_len;
    parsed_ok = Screen_ParseLocalDateTime(local_buf, &parsed_clock);
    if ((parsed_ok != 0U) && (SCREEN_TIME_RX_COMPENSATE_S > 0U))
    {
        Screen_LocalClockAddSeconds(&parsed_clock, SCREEN_TIME_RX_COMPENSATE_S);
    }
    now = xTaskGetTickCount();

    taskENTER_CRITICAL();
    memcpy(s_time_tz_str, tz_buf, sizeof(s_time_tz_str));
    s_time_ts = ts;
    s_time_has_ts = (has_ts != 0U) ? 1U : 0U;

    should_calibrate = 0U;
    if (parsed_ok != 0U)
    {
        if (s_local_clock.valid == 0U)
        {
            /* 本地时钟未初始化，直接校准 */
            should_calibrate = 1U;
        }
        else if ((parsed_clock.year != s_local_clock.year) ||
                   (parsed_clock.month != s_local_clock.month) ||
                   (parsed_clock.day != s_local_clock.day) ||
                   (parsed_clock.hour != s_local_clock.hour) ||
                   (parsed_clock.minute != s_local_clock.minute))
        {
            /* 年/月/日/时/分 任一不同，直接校准 */
            should_calibrate = 1U;
        }
        else
        {
            sec_diff = (parsed_clock.second > s_local_clock.second)
                           ? (uint8_t)(parsed_clock.second - s_local_clock.second)
                           : (uint8_t)(s_local_clock.second - parsed_clock.second);
            if (sec_diff >= SCREEN_TIME_SYNC_SEC_THRESHOLD)
            {
                /* 秒数偏差超过阈值，需要校准 */
                should_calibrate = 1U;
            }
        }
    }

    if ((parsed_ok != 0U) && (should_calibrate != 0U))
    {
        /* 校准本地时钟: 用解析出的时间替换当前值 */
        s_local_clock.year = parsed_clock.year;
        s_local_clock.month = parsed_clock.month;
        s_local_clock.day = parsed_clock.day;
        s_local_clock.hour = parsed_clock.hour;
        s_local_clock.minute = parsed_clock.minute;
        s_local_clock.second = parsed_clock.second;
        s_local_clock.last_tick = now;
        s_local_clock.last_sync_tick = now;
        s_local_clock.valid = 1U;
    }

    Screen_FormatLocalClockLocked();
    taskEXIT_CRITICAL();

    /* 时间同步不应打断阈值设置页/手动控制页的本地编辑。 */
    if ((s_active_page != SCREEN_CMD_CHANGE1_PAGE) &&
        (s_active_page != SCREEN_CMD_CHANGE2_PAGE) &&
        (s_active_page != SCREEN_CMD_CHANGE3_PAGE) &&
        (s_active_page != SCREEN_CMD_CHANGE4_PAGE) &&
        (s_active_page != SCREEN_CMD_CHANGE5_PAGE) &&
        (s_active_page != SCREEN_CMD_CHANGE6_PAGE) &&
        (s_active_page != SCREEN_CMD_HAND1_PAGE) &&
        (s_active_page != SCREEN_CMD_HAND2_PAGE) &&
        (s_active_page != SCREEN_CMD_HAND3_PAGE) &&
        (s_active_page != SCREEN_CMD_HAND4_PAGE))
    {
        /* 其他状态页仍然可以立即更新日期时间显示。 */
        Screen_RequestImmediateRefresh();
    }
}

/**
 * @brief   记录一次设备操作事件
 * @param   device        被操作的设备类型
 * @param   is_manual     操作模式: 1=手动, 0=自动
 * @param   action_value  操作值: 开关类为0/1, 风扇为档位1~5
 *
 * 获取当前时间，向串口屏 recorder 控件写入操作记录，
 * 同时推入内存动作队列供 MQTT 上报。
 */
void Screen_RecordDeviceAction(ScreenActionDevice_t device,
                               uint8_t is_manual,
                               uint8_t action_value)
{
    uint16_t year = 0U;
    uint8_t month = 0U;
    uint8_t day = 0U;
    uint8_t hour = 0U;
    uint8_t minute = 0U;
    uint8_t time_valid = 0U;
    char date_text[24];
    char time_text[8];
    const char *device_text;
    const char *action_text;
    char merged_text[120];
    char insert_cmd[176];
    int len;
    uint8_t locked = 0U;

    Screen_TimeUpdateFromTick();

    taskENTER_CRITICAL();
    if (s_local_clock.valid != 0U)
    {
        year = s_local_clock.year;
        month = s_local_clock.month;
        day = s_local_clock.day;
        hour = s_local_clock.hour;
        minute = s_local_clock.minute;
        time_valid = 1U;
    }
    taskEXIT_CRITICAL();

    if (time_valid != 0U)
    {
        snprintf(date_text, sizeof(date_text), "%u/%u/%u", year, month, day);
        snprintf(time_text, sizeof(time_text), "%02u:%02u", hour, minute);
    }
    else
    {
        snprintf(date_text, sizeof(date_text), "----/--/--");
        snprintf(time_text, sizeof(time_text), "--:--");
    }

    device_text = Screen_ActionDeviceText(device);
    if (device == SCREEN_ACTION_DEV_FAN)
    {
        action_text = Screen_ActionFanText(is_manual, action_value);
    }
    else
    {
        action_text = Screen_ActionRelayText(is_manual, action_value);
    }

    if ((s_screen_tx_mutex != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
        {
        (void)xSemaphoreTake(s_screen_tx_mutex, portMAX_DELAY);
        locked = 1U;
    }

    /* 向串口屏 recorder 控件写入记录条目 (t0=日期, t1=时间, t2=设备, t3=动作) */
    Screen_SendTextNoLock("recoder.t0", date_text);
    Screen_SendTextNoLock("recoder.t1", time_text);
    Screen_SendTextNoLock("recoder.t2", device_text);
    Screen_SendTextNoLock("recoder.t3", action_text);

    /* 合并所有字段为单行字符串，用 '^' 分隔，存入 tmp 临时组件 */
    snprintf(merged_text,
             sizeof(merged_text),
             "%s^%s^%s^%s",
             date_text,
             time_text,
             device_text,
             action_text);
    Screen_SendTextNoLock("recoder.tmp", merged_text);

    /* 等待屏幕处理完成临时数据写入 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        Delay_TaskMs(8U);  /* 至少等待 1 tick，避免 8ms 在 100Hz tick 下被截断为 0 */
    }
    /* 将合并文本插入 data0 列表组件（历史记录列表） */
    len = snprintf(insert_cmd,
                   sizeof(insert_cmd),
                   "recoder.data0.insert(\"%s\")",
                   merged_text);
    if ((len > 0) && ((size_t)len < sizeof(insert_cmd)))
    {
        Screen_SendCommandRaw(insert_cmd);
    }
    else
    {
        /* 合并文本超出 insert 命令缓冲区，回退使用 tmp.txt 引用 */
        Screen_SendCommandRaw("recoder.data0.insert(recoder.tmp.txt)");
    }

    /* 同时推入内存动作队列，供 MQTT 上报模块弹出 */
    Screen_ActionQueuePush(device, is_manual, action_value, date_text, time_text);

    if (locked != 0U)
    {
        (void)xSemaphoreGive(s_screen_tx_mutex);
    }
}

/**
 * @brief   从动作队列中弹出一条事件
 * @param   out_event  输出参数，弹出的事件数据
 * @return  1=成功弹出, 0=队列为空
 *
 * 使用临界区保护，确保中断安全。按 FIFO 顺序取出。
 */
uint8_t Screen_PopActionEvent(ScreenActionEvent_t *out_event)
{
    if (out_event == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (s_action_queue_count == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    *out_event = s_action_queue[s_action_queue_tail];
    s_action_queue_tail = (uint8_t)((s_action_queue_tail + 1U) % SCREEN_ACTION_QUEUE_SIZE);
    s_action_queue_count--;
    taskEXIT_CRITICAL();

    return 1U;
}

/**
 * @brief   发送数值到串口屏组件的 val 属性
 * @param   component_id  UI 组件 ID
 * @param   value         要设置的数值
 *
 * 生成 Nextion 指令: component_id.val=N + 0xFF 0xFF 0xFF
 * 主要用于阈值设置页显示当前阈值。
 */
static void Screen_SendVal(const char *component_id, uint32_t value)
{
    char cmd[96];
    int len;

    if (component_id == NULL)
    {
        return;
    }

    len = snprintf(cmd, sizeof(cmd), "%s.val=%lu", component_id, (unsigned long)value);
    if ((len > 0) && ((size_t)len < sizeof(cmd)))
    {
        Screen_SendCommand(cmd);
    }
}

/**
 * @brief   将无符号整数转为字符串后发送到文本组件
 * @param   component_id  UI 组件 ID
 * @param   value         要显示的数值
 *
 * 内部调用 Screen_SendText，具备缓存去重能力。
 */
static void Screen_SendUintText(const char *component_id, uint32_t value)
{
    char value_str[16];

    snprintf(value_str, sizeof(value_str), "%lu", (unsigned long)value);
    Screen_SendText(component_id, value_str);
}

/**
 * @brief   将浮点数转换为十倍整数值（保留一位小数）
 * @param   value  浮点数值
 * @return  value * 10 四舍五入后的无符号整数
 *
 * 串口屏的滑动条控件只支持整数，用 ×10 的方式保留一位小数精度。
 */
static uint32_t Screen_FloatToTenthVal(float value)
{
    float scaled;

    if (value <= 0.0f)
    {
        return 0U;
    }

    scaled = value * 10.0f + 0.5f;
    if (scaled >= 4294967295.0f)
    {
        return 4294967295U;
    }

    return (uint32_t)scaled;
}

/**
 * @brief   发送阈值设置页1的当前阈值到屏幕
 *
 * 包含加热器启动温度、制冷器启动温度、
 * 加湿器启动湿度、除湿器启动湿度四个参数。
 */
static void Screen_SendChange1Thresholds(void)
{
    float heater_on;
    float cooler_on;
    float humidifier_on;
    float dehumidifier_on;

    Logic_GetChange1Thresholds(&heater_on, &cooler_on, &humidifier_on, &dehumidifier_on);

    Screen_SendVal(UI_HEATER_THRESHOLD_ID, Screen_FloatToTenthVal(heater_on));
    Screen_SendVal(UI_COOLER_THRESHOLD_ID, Screen_FloatToTenthVal(cooler_on));
    Screen_SendVal(UI_HUMID_THRESHOLD_ID, Screen_FloatToTenthVal(humidifier_on));
    Screen_SendVal(UI_DEHUMID_THRESHOLD_ID, Screen_FloatToTenthVal(dehumidifier_on));
}

/**
 * @brief   发送阈值设置页2的当前阈值到屏幕
 *
 * 包含 PM2.5报警延时、气体报警延时、
 * PM2.5报警阈值、气体报警阈值四个参数。
 */
static void Screen_SendChange2Thresholds(void)
{
    float pm25_alarm_delay_seconds;
    float gas_alarm_delay_seconds;
    float pm25_alarm_threshold;
    float gas_alarm_threshold;

    Logic_GetChange2Thresholds(&pm25_alarm_delay_seconds,
                               &gas_alarm_delay_seconds,
                               &pm25_alarm_threshold,
                               &gas_alarm_threshold);

    Screen_SendVal(UI_PM25_ALARM_DELAY_ID, Screen_FloatToTenthVal(pm25_alarm_delay_seconds));
    Screen_SendVal(UI_GAS_ALARM_DELAY_ID, Screen_FloatToTenthVal(gas_alarm_delay_seconds));
    Screen_SendVal(UI_PM25_ALARM_THRESHOLD_ID, Screen_FloatToTenthVal(pm25_alarm_threshold));
    Screen_SendVal(UI_GAS_ALARM_THRESHOLD_ID, Screen_FloatToTenthVal(gas_alarm_threshold));
}

/**
 * @brief   发送阈值设置页3的当前阈值到屏幕
 *
 * 包含门未关报警时间、光感启灯阈值、
 * 人感开灯距离阈值、风扇1档转速四个参数。
 */
static void Screen_SendChange3Thresholds(void)
{
    float door_open_alarm_seconds;
    float light_on_threshold;
    float human_light_distance_threshold;
    float fan_level1_speed;

    Logic_GetChange3Thresholds(&door_open_alarm_seconds,
                               &light_on_threshold,
                               &human_light_distance_threshold,
                               &fan_level1_speed);

    Screen_SendVal(UI_DOOR_ALARM_THRESHOLD_ID, Screen_FloatToTenthVal(door_open_alarm_seconds));
    Screen_SendVal(UI_LIGHT_ON_THRESHOLD_ID, Screen_FloatToTenthVal(light_on_threshold));
    Screen_SendVal(UI_HUMAN_LIGHT_THRESHOLD_ID, Screen_FloatToTenthVal(human_light_distance_threshold));
    Screen_SendVal(UI_FAN_LEVEL1_SPEED_ID, Screen_FloatToTenthVal(fan_level1_speed));
}

/**
 * @brief   发送阈值设置页4的当前阈值到屏幕
 *
 * 包含风扇2~5档转速四个参数。
 */
static void Screen_SendChange4Thresholds(void)
{
    float fan_level2_speed;
    float fan_level3_speed;
    float fan_level4_speed;
    float fan_level5_speed;

    Logic_GetChange4Thresholds(&fan_level2_speed,
                               &fan_level3_speed,
                               &fan_level4_speed,
                               &fan_level5_speed);

    Screen_SendVal(UI_FAN_LEVEL2_SPEED_ID, Screen_FloatToTenthVal(fan_level2_speed));
    Screen_SendVal(UI_FAN_LEVEL3_SPEED_ID, Screen_FloatToTenthVal(fan_level3_speed));
    Screen_SendVal(UI_FAN_LEVEL4_SPEED_ID, Screen_FloatToTenthVal(fan_level4_speed));
    Screen_SendVal(UI_FAN_LEVEL5_SPEED_ID, Screen_FloatToTenthVal(fan_level5_speed));
}

/**
 * @brief   发送阈值设置页5的当前阈值到屏幕
 *
 * PM2.5 浓度五档区间（每档包含上下限），用于风扇档位映射。
 */
static void Screen_SendChange5Thresholds(void)
{
    uint16_t lower_bounds[5];
    uint16_t upper_bounds[5];

    Logic_GetChange5Thresholds(lower_bounds, upper_bounds);

    Screen_SendUintText(UI_PM_L1_LOW_ID, lower_bounds[0]);
    Screen_SendUintText(UI_PM_L1_HIGH_ID, upper_bounds[0]);
    Screen_SendUintText(UI_PM_L2_LOW_ID, lower_bounds[1]);
    Screen_SendUintText(UI_PM_L2_HIGH_ID, upper_bounds[1]);
    Screen_SendUintText(UI_PM_L3_LOW_ID, lower_bounds[2]);
    Screen_SendUintText(UI_PM_L3_HIGH_ID, upper_bounds[2]);
    Screen_SendUintText(UI_PM_L4_LOW_ID, lower_bounds[3]);
    Screen_SendUintText(UI_PM_L4_HIGH_ID, upper_bounds[3]);
    Screen_SendUintText(UI_PM_L5_LOW_ID, lower_bounds[4]);
    Screen_SendUintText(UI_PM_L5_HIGH_ID, upper_bounds[4]);
}

/**
 * @brief   发送阈值设置页6的当前阈值到屏幕
 *
 * 气体浓度五档区间（每档包含上下限），用于风扇档位映射。
 */
static void Screen_SendChange6Thresholds(void)
{
    uint16_t lower_bounds[5];
    uint16_t upper_bounds[5];

    Logic_GetChange6Thresholds(lower_bounds, upper_bounds);

    Screen_SendUintText(UI_GAS_L1_LOW_ID, lower_bounds[0]);
    Screen_SendUintText(UI_GAS_L1_HIGH_ID, upper_bounds[0]);
    Screen_SendUintText(UI_GAS_L2_LOW_ID, lower_bounds[1]);
    Screen_SendUintText(UI_GAS_L2_HIGH_ID, upper_bounds[1]);
    Screen_SendUintText(UI_GAS_L3_LOW_ID, lower_bounds[2]);
    Screen_SendUintText(UI_GAS_L3_HIGH_ID, upper_bounds[2]);
    Screen_SendUintText(UI_GAS_L4_LOW_ID, lower_bounds[3]);
    Screen_SendUintText(UI_GAS_L4_HIGH_ID, upper_bounds[3]);
    Screen_SendUintText(UI_GAS_L5_LOW_ID, lower_bounds[4]);
    Screen_SendUintText(UI_GAS_L5_HIGH_ID, upper_bounds[4]);
}

/**
 * @brief   手动控制页用 —— 获取设备状态显示文本 ("\u5f53\u524d\u72b6\u6001:xxx")
 * @param   status  逻辑层设备状态枚举
 * @return  GB2312 状态文本字符串
 */
static const char *Screen_ManualStatusText(LogicManualStatus_t status)
{
    switch (status)
    {
    case LOGIC_MANUAL_STATUS_MANUAL_ON:
        return CN_STATUS_MANUAL_ON;
    case LOGIC_MANUAL_STATUS_MANUAL_OFF:
        return CN_STATUS_MANUAL_OFF;
    case LOGIC_MANUAL_STATUS_MANUAL_ON_PENDING:
        return CN_STATUS_MANUAL_ON_PENDING;
    case LOGIC_MANUAL_STATUS_MANUAL_OFF_PENDING:
        return CN_STATUS_MANUAL_OFF_PENDING;
    case LOGIC_MANUAL_STATUS_AUTO_ON:
        return CN_STATUS_AUTO_ON;
    case LOGIC_MANUAL_STATUS_AUTO_OFF:
    default:
        return CN_STATUS_AUTO_OFF;
    }
}

/**
 * @brief   继电器总览页用 —— 获取设备状态显示文本 ("\u5f53\u524d:xxx")
 * @param   status  逻辑层设备状态枚举
 * @return  GB2312 状态文本字符串
 */
static const char *Screen_RelayStatusText(LogicManualStatus_t status)
{
    switch (status)
    {
    case LOGIC_MANUAL_STATUS_MANUAL_ON:
        return CN_RELAY_MANUAL_ON;
    case LOGIC_MANUAL_STATUS_MANUAL_OFF:
        return CN_RELAY_MANUAL_OFF;
    case LOGIC_MANUAL_STATUS_MANUAL_ON_PENDING:
        return CN_RELAY_MANUAL_ON_PENDING;
    case LOGIC_MANUAL_STATUS_MANUAL_OFF_PENDING:
        return CN_RELAY_MANUAL_OFF_PENDING;
    case LOGIC_MANUAL_STATUS_AUTO_ON:
        return CN_RELAY_AUTO_ON;
    case LOGIC_MANUAL_STATUS_AUTO_OFF:
    default:
        return CN_RELAY_AUTO_OFF;
    }
}

/**
 * @brief   获取风扇模式+档位的显示文本
 * @param   is_manual        1=手动, 0=自动
 * @param   fan_gear_1_to_5  风扇档位 (1~5)
 * @return  GB2312 状态文本, 如 "当前:手动3档" / "当前:自动2档"
 */
static const char *Screen_FanModeGearText(uint8_t is_manual, uint8_t fan_gear_1_to_5)
{
    if (is_manual != 0U)
    {
        switch (fan_gear_1_to_5)
        {
        case 1U:
            return CN_RELAY_FAN_MANUAL_GEAR1;
        case 2U:
            return CN_RELAY_FAN_MANUAL_GEAR2;
        case 3U:
            return CN_RELAY_FAN_MANUAL_GEAR3;
        case 4U:
            return CN_RELAY_FAN_MANUAL_GEAR4;
        case 5U:
        default:
            return CN_RELAY_FAN_MANUAL_GEAR5;
        }
    }

    switch (fan_gear_1_to_5)
    {
    case 1U:
        return CN_RELAY_FAN_AUTO_GEAR1;
    case 2U:
        return CN_RELAY_FAN_AUTO_GEAR2;
    case 3U:
        return CN_RELAY_FAN_AUTO_GEAR3;
    case 4U:
        return CN_RELAY_FAN_AUTO_GEAR4;
    case 5U:
    default:
        return CN_RELAY_FAN_AUTO_GEAR5;
    }
}

/**
 * @brief   发送手动控制页1的设备状态（加热器 + 制冷器）
 */
static void Screen_SendHand1Status(void)
{
    LogicManualStatus_t heater_status = Logic_GetDeviceManualStatus(LOGIC_DEV_HEATER);
    LogicManualStatus_t cooler_status = Logic_GetDeviceManualStatus(LOGIC_DEV_COOLER);

    Screen_SendText(UI_HAND_HEATER_STATUS_ID, Screen_ManualStatusText(heater_status));
    Screen_SendText(UI_HAND_COOLER_STATUS_ID, Screen_ManualStatusText(cooler_status));
}

/**
 * @brief   发送手动控制页2的设备状态（加湿器 + 除湿器）
 */
static void Screen_SendHand2Status(void)
{
    LogicManualStatus_t humidifier_status = Logic_GetDeviceManualStatus(LOGIC_DEV_HUMIDIFIER);
    LogicManualStatus_t dehumidifier_status = Logic_GetDeviceManualStatus(LOGIC_DEV_DEHUMIDIFIER);

    Screen_SendText(UI_HAND_HEATER_STATUS_ID, Screen_ManualStatusText(humidifier_status));
    Screen_SendText(UI_HAND_COOLER_STATUS_ID, Screen_ManualStatusText(dehumidifier_status));
}

/**
 * @brief   发送手动控制页3的设备状态（报警器 + 灯光）
 */
static void Screen_SendHand3Status(void)
{
    LogicManualStatus_t alarm_status = Logic_GetDeviceManualStatus(LOGIC_DEV_ALARM);
    LogicManualStatus_t light_status = Logic_GetDeviceManualStatus(LOGIC_DEV_LIGHT);

    Screen_SendText(UI_HAND_HEATER_STATUS_ID, Screen_ManualStatusText(alarm_status));
    Screen_SendText(UI_HAND_COOLER_STATUS_ID, Screen_ManualStatusText(light_status));
}

/**
 * @brief   发送手动控制页4的设备状态（风扇模式 + 档位）
 */
static void Screen_SendHand4Status(void)
{
    uint8_t fan_gear = 1U;
    uint8_t is_manual = 0U;

    Logic_GetFanModeAndGear(&is_manual, &fan_gear);
    Screen_SendText(UI_HAND_HEATER_STATUS_ID, Screen_FanModeGearText(is_manual, fan_gear));
}

/**
 * @brief   发送继电器总览页所有设备状态
 *
 * 显示加热器/制冷器/加湿器/报警器/灯光/风扇的当前模式和状态。
 */
static void Screen_SendRelayStatus(void)
{
    LogicManualStatus_t heater_status = Logic_GetDeviceManualStatus(LOGIC_DEV_HEATER);
    LogicManualStatus_t cooler_status = Logic_GetDeviceManualStatus(LOGIC_DEV_COOLER);
    LogicManualStatus_t humidifier_status = Logic_GetDeviceManualStatus(LOGIC_DEV_HUMIDIFIER);
    LogicManualStatus_t alarm_status = Logic_GetDeviceManualStatus(LOGIC_DEV_ALARM);
    LogicManualStatus_t light_status = Logic_GetDeviceManualStatus(LOGIC_DEV_LIGHT);
    uint8_t fan_gear = 1U;
    uint8_t fan_is_manual = 0U;

    Logic_GetFanModeAndGear(&fan_is_manual, &fan_gear);

    Screen_SendText(UI_RELAY_HEATER_STATUS_ID, Screen_RelayStatusText(heater_status));
    Screen_SendText(UI_RELAY_COOLER_STATUS_ID, Screen_RelayStatusText(cooler_status));
    Screen_SendText(UI_RELAY_HUMID_STATUS_ID, Screen_RelayStatusText(humidifier_status));
    Screen_SendText(UI_RELAY_ALARM_STATUS_ID, Screen_RelayStatusText(alarm_status));
    Screen_SendText(UI_RELAY_LIGHT_STATUS_ID, Screen_RelayStatusText(light_status));
    Screen_SendText(UI_RELAY_FAN_STATUS_ID, Screen_FanModeGearText(fan_is_manual, fan_gear));
}

/**
 * @brief   发送系统设置页信息（系统运行时间）
 *
 * 将 FreeRTOS Tick 计数转换为 "天/时/分" 格式显示。
 */
static void Screen_SendSettingStatus(void)
{
    TickType_t now_ticks;
    uint64_t total_minutes;
    uint32_t days;
    uint32_t hours;
    uint32_t minutes;
    char value[48];

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        now_ticks = 0U;
    }
    else
    {
        now_ticks = xTaskGetTickCount();
    }

    total_minutes = ((uint64_t)now_ticks) / ((uint64_t)configTICK_RATE_HZ * 60ULL);
    days = (uint32_t)(total_minutes / (24ULL * 60ULL));
    hours = (uint32_t)((total_minutes % (24ULL * 60ULL)) / 60ULL);
    minutes = (uint32_t)(total_minutes % 60ULL);

    snprintf(value,
             sizeof(value),
             "%lu\xCC\xEC%lu\xCA\xB1%lu\xB7\xD6",  /* 格式: N天N时N分 (GB2312编码) */
             (unsigned long)days,
             (unsigned long)hours,
             (unsigned long)minutes);
    Screen_SendText(UI_SYSTEM_UPTIME_ID, value);
}

/**
 * @brief   处理阈值设置帧
 * @param   frame  7字节帧数据 [HEAD, ID, data0, data1, data2, data3, TAIL]
 * @param   group  阈值组编号 (1~6)
 *
 * 解码小端序 32 位数值，组 1~4 解释为十分之一浮点数，
 * 组 5~6 直接作为整数使用。设置成功后更新当前活动页面。
 */
static void Screen_ProcessChangeThresholdFrame(const uint8_t frame[SCREEN_CHANGE1_FRAME_LEN],
                                               uint8_t group)
{
    uint8_t threshold_id;
    uint32_t raw;
    float real_value;
    uint8_t set_ok = 0U;

    if (frame == NULL)
    {
        return;
    }

    threshold_id = frame[1];
    raw = Screen_DecodeU32LE(&frame[2]);

    if (group <= 4U)
    {
        /* 组1~4: 值以十倍整数传输，除以10还原为浮点数 */
        real_value = (float)raw / 10.0f;
        switch (group)
        {
        case 1U: set_ok = Logic_SetChange1ThresholdById(threshold_id, real_value); break;
        case 2U: set_ok = Logic_SetChange2ThresholdById(threshold_id, real_value); break;
        case 3U: set_ok = Logic_SetChange3ThresholdById(threshold_id, real_value); break;
        case 4U: set_ok = Logic_SetChange4ThresholdById(threshold_id, real_value); break;
        default: break;
        }
    }
    else
    {
        switch (group)
        {
        case 5U: set_ok = Logic_SetChange5ThresholdById(threshold_id, raw); break;
        case 6U: set_ok = Logic_SetChange6ThresholdById(threshold_id, raw); break;
        default: break;
        }
    }

    if (set_ok)
    {
        /* 设置成功，更新当前活动页面为对应的阈值设置页 */
        /* SCREEN_CMD_CHANGE1_PAGE (0x02) 起始， group 1~6 顺序对应 */
        s_active_page = (uint8_t)(SCREEN_CMD_CHANGE1_PAGE + group - 1U);
    }
}

/**
 * @brief   判断设备 ID 是否为手动控制帧中的合法设备 ID
 * @param   id  设备 ID 字节 (0x30~0x36)
 * @return  1=合法, 0=不合法
 */
static uint8_t Screen_IsManualDeviceId(uint8_t id)
{
    return ((id == SCREEN_MANUAL_HEATER_ID) ||
            (id == SCREEN_MANUAL_COOLER_ID) ||
            (id == SCREEN_MANUAL_HUMIDIFIER_ID) ||
            (id == SCREEN_MANUAL_DEHUMIDIFIER_ID) ||
            (id == SCREEN_MANUAL_ALARM_ID) ||
            (id == SCREEN_MANUAL_LIGHT_ID) ||
            (id == SCREEN_MANUAL_FAN_ID)) ? 1U : 0U;
}

/**
 * @brief   验证手动控制帧的负载值是否合法
 * @param   id       设备 ID
 * @param   payload  负载值（风扇: 1~5 档位, 其他: 0/1 开关）
 * @return  1=合法, 0=不合法
 */
static uint8_t Screen_IsValidManualPayload(uint8_t id, uint8_t payload)
{
    if (id == SCREEN_MANUAL_FAN_ID)
    {
        return ((payload >= 1U) && (payload <= 5U)) ? 1U : 0U;
    }
    return ((payload == 0x00U) || (payload == 0x01U)) ? 1U : 0U;
}

/**
 * @brief   判断 8 字节帧是否为恢复出厂设置帧
 * @param   frame  8字节帧数据
 * @return  1=是恢复出厂帧, 0=不是
 *
 * 帧格式: [0x55, 0x88, 0xFF, 0x88, 0xFF, 0x88, 0xFF, 0xAA]
 * 使用重复标记字节确认，避免误触发。
 */
static uint8_t Screen_IsResetDefaultsFrame(const uint8_t frame[SCREEN_MANUAL_SAVE_FRAME_LEN])
{
    if (frame == NULL)
    {
        return 0U;
    }

    return ((frame[0] == SCREEN_CMD_HEAD) &&
            (frame[1] == SCREEN_RESET_DEFAULTS_ID) &&
            (frame[2] == SCREEN_RESET_DEFAULTS_MARK) &&
            (frame[3] == SCREEN_RESET_DEFAULTS_ID) &&
            (frame[4] == SCREEN_RESET_DEFAULTS_MARK) &&
            (frame[5] == SCREEN_RESET_DEFAULTS_ID) &&
            (frame[6] == SCREEN_RESET_DEFAULTS_MARK) &&
            (frame[7] == SCREEN_CMD_TAIL)) ? 1U : 0U;
}

/**
 * @brief   执行恢复出厂设置操作
 *
 * 调用运行时配置模块将所有参数重置为默认值并保存，
 * 然后切换到系统设置页并触发即时刷新。
 */
static void Screen_ProcessResetDefaultsFrame(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        TickType_t now = xTaskGetTickCount();

        if ((s_reset_defaults_last_tick != 0U) &&
            ((now - s_reset_defaults_last_tick) < pdMS_TO_TICKS(SCREEN_RESET_DEBOUNCE_MS)))
        {
            return;
        }

        s_reset_defaults_last_tick = now;
    }

    RuntimeConfig_ResetToFactoryAndRequestSave();
    s_active_page = SCREEN_CMD_SETTING_PAGE;
    Screen_RequestImmediateRefresh();
}

/* 手动控制设备ID (0x30~0x36) 到对应手动控制页面的映射表 */
static const uint8_t s_manual_id_to_page[LOGIC_DEV_COUNT] = {
    SCREEN_CMD_HAND1_PAGE,  /* HEATER      → 手动控制页1 */
    SCREEN_CMD_HAND1_PAGE,  /* COOLER      → 手动控制页1 */
    SCREEN_CMD_HAND2_PAGE,  /* HUMIDIFIER  → 手动控制页2 */
    SCREEN_CMD_HAND2_PAGE,  /* DEHUMIDIFIER→ 手动控制页2 */
    SCREEN_CMD_HAND3_PAGE,  /* ALARM       → 手动控制页3 */
    SCREEN_CMD_HAND3_PAGE,  /* LIGHT       → 手动控制页3 */
    SCREEN_CMD_HAND4_PAGE   /* FAN         → 手动控制页4 */
};

/**
 * @brief   处理手动控制保存帧
 * @param   frame  8字节帧 [HEAD, devID, dur0..dur3, value, TAIL]
 *
 * 解析设备ID、手动控制时长(秒)、操作值，
 * 调用逻辑层设置手动模式，并切换到对应页面。
 */
static void Screen_ProcessManualSaveFrame(const uint8_t frame[SCREEN_MANUAL_SAVE_FRAME_LEN])
{
    uint8_t id;
    uint32_t duration_seconds;
    uint8_t value;
    uint8_t dev;

    if (frame == NULL)
    {
        return;
    }

    id = frame[1];
    value = frame[6];
    duration_seconds = Screen_DecodeU32LE(&frame[2]);

    if (!Screen_IsManualDeviceId(id))
    {
        return;
    }

    dev = id - SCREEN_MANUAL_HEATER_ID;  /* 转换为 LOGIC_DEV_xxx 索引 (0x30基址偏移) */
    Logic_SetDeviceManual(dev, duration_seconds, value);
    s_active_page = s_manual_id_to_page[dev]; /* 切换到对应手动控制页 */
    Screen_RequestImmediateRefresh();
}

/**
 * @brief   处理手动控制自动恢复帧
 * @param   frame  7字节帧 [HEAD, devID, 0xF0, 0xF0, 0xF0, 0xF0, TAIL]
 *
 * 当数据位全为 0xF0 魔术字节时，表示恢复自动模式。
 */
static void Screen_ProcessManualAutoRestoreFrame(const uint8_t frame[SCREEN_CHANGE1_FRAME_LEN])
{
    uint8_t id;
    uint8_t dev;

    if (frame == NULL)
    {
        return;
    }

    id = frame[1];
    if (!Screen_IsManualDeviceId(id))
    {
        return;
    }

    if ((frame[2] != SCREEN_MANUAL_AUTO_MAGIC_BYTE) ||
        (frame[3] != SCREEN_MANUAL_AUTO_MAGIC_BYTE) ||
        (frame[4] != SCREEN_MANUAL_AUTO_MAGIC_BYTE) ||
        (frame[5] != SCREEN_MANUAL_AUTO_MAGIC_BYTE))
        {
        return;
    }

    dev = id - SCREEN_MANUAL_HEATER_ID;  /* 转换为 LOGIC_DEV_xxx 索引 */
    Logic_RestoreDeviceAuto(dev);
    s_active_page = s_manual_id_to_page[dev]; /* 切换到对应手动控制页 */
    Screen_RequestImmediateRefresh();
}

/**
 * @brief   处理串口屏接收到的单个字节（滑动窗口帧解析器）
 * @param   byte  接收到的字节
 *
 * 将新字节推入 3/7/8 字节的滑动窗口，依次检查：
 *   1. 8字节窗口: 恢复出厂帧 → 手动控制保存帧
 *   2. 7字节窗口: 自动恢复帧 → 阈值设置帧
 *   3. 3字节窗口: 页面切换帧
 * 按优先级顺序匹配，匹配成功后立即返回。
 */
static void Screen_ProcessByte(uint8_t byte)
{
    uint8_t i;

    /* --- 滑动窗口更新: 将新字节推入各窗口末尾，旧字节左移 --- */
    for (i = 0U; i < (SCREEN_MANUAL_SAVE_FRAME_LEN - 1U); i++)
    {
        s_frame_window8[i] = s_frame_window8[i + 1U];
    }
    s_frame_window8[SCREEN_MANUAL_SAVE_FRAME_LEN - 1U] = byte;

    for (i = 0U; i < (SCREEN_CHANGE1_FRAME_LEN - 1U); i++)
    {
        s_frame_window7[i] = s_frame_window7[i + 1U];
    }
    s_frame_window7[SCREEN_CHANGE1_FRAME_LEN - 1U] = byte;

    s_frame_window[0] = s_frame_window[1];
    s_frame_window[1] = s_frame_window[2];
    s_frame_window[2] = byte;

    /* --- 优先级最高: 检查 8 字节窗口是否为恢复出厂帧 --- */
    if (Screen_IsResetDefaultsFrame(s_frame_window8))
    {
        Screen_ProcessResetDefaultsFrame();
        return;
    }

    /* --- 检查 8 字节窗口是否为手动控制保存帧 --- */
    if ((s_frame_window8[0] == SCREEN_CMD_HEAD) &&
        (s_frame_window8[7] == SCREEN_CMD_TAIL) &&
        Screen_IsManualDeviceId(s_frame_window8[1]) &&
        Screen_IsValidManualPayload(s_frame_window8[1], s_frame_window8[6]))
    {
        Screen_ProcessManualSaveFrame(s_frame_window8);  /* 处理手动控制帧 */
        return;
    }

    /* --- 检查 7 字节窗口是否为自动恢复帧 --- */
    if ((s_frame_window7[0] == SCREEN_CMD_HEAD) &&
        (s_frame_window7[6] == SCREEN_CMD_TAIL) &&
        Screen_IsManualDeviceId(s_frame_window7[1]))
    {
        Screen_ProcessManualAutoRestoreFrame(s_frame_window7);  /* 处理自动恢复帧 */
        return;
    }

    /* --- 检查 7 字节窗口是否为阈值设置帧 (ID 范围 0x00~0x2A) --- */
    if ((s_frame_window7[0] == SCREEN_CMD_HEAD) &&
        (s_frame_window7[6] == SCREEN_CMD_TAIL) &&
        (s_frame_window7[1] <= 0x2AU))
    {
        /* 根据阈值 ID 范围分派到对应的阈值组 */
        if (s_frame_window7[1] <= 0x03U)
        {
            Screen_ProcessChangeThresholdFrame(s_frame_window7, 1U);  /* 组1: 温湿度阈值 */
        }
        else if (s_frame_window7[1] <= 0x07U)
        {
            Screen_ProcessChangeThresholdFrame(s_frame_window7, 2U);  /* 组2: PM2.5/气体报警 */
        }
        else if (s_frame_window7[1] <= 0x0BU)
        {
            Screen_ProcessChangeThresholdFrame(s_frame_window7, 3U);  /* 组3: 门磁/光照/人感/风扇1档 */
        }
        else if (s_frame_window7[1] <= 0x0FU)
        {
            Screen_ProcessChangeThresholdFrame(s_frame_window7, 4U);  /* 组4: 风扇2~5档转速 */
        }
        else if (s_frame_window7[1] <= 0x1AU)
        {
            Screen_ProcessChangeThresholdFrame(s_frame_window7, 5U);  /* 组5: PM2.5 五档区间 */
        }
        else
        {
            Screen_ProcessChangeThresholdFrame(s_frame_window7, 6U);  /* 组6: 气体 五档区间 */
        }
        return;
    }

    /* --- 最后检查 3 字节窗口是否为页面切换帧 [0x55, pageID, 0xAA] --- */
    if ((s_frame_window[0] == SCREEN_CMD_HEAD) &&
        (s_frame_window[2] == SCREEN_CMD_TAIL))
    {
        Screen_ProcessPageCommand(s_frame_window[1]);
    }
}

/**
 * @brief   发送主页状态信息
 *
 * 发送内容包括：
 *   - 所有传感器/继电器/WiFi/Web 端的连接状态汇总
 *   - 当前日期和时间
 */
static void Screen_SendIndexStatus(void)
{
    AllSensorConnectionStatus_t conn;
    uint8_t relay_connected;
    uint8_t wifi_connected;
    uint8_t mqtt_connected;
    uint8_t web_connected;
    char date_text[20];
    char hm_text[6];
    char status_text[320];
    int len;

    AllSensorData_GetConnectionStatus(&conn);
    relay_connected = Screen_GetRelayConnectedCached();
    wifi_connected = ESP01S_IsConnected();
    mqtt_connected = MqttApp_IsBrokerConnected();
    web_connected = ESP01S_IsWebConnected();
    Screen_GetDateTimeForUi(date_text, hm_text);

#if SCREEN_SELF_TEST_SHORT_FIRST
    Screen_SendCommand("t2.txt=\"OK\"");
    vTaskDelay(pdMS_TO_TICKS(20));
#endif

    /* 拼接所有连接状态为一个多行字符串，用 \\r 分行 */
    len = snprintf(status_text,
                   sizeof(status_text),
                   CN_TEMP_HUMI "%s\\r"
                   CN_PM "%s\\r"
                   CN_GAS "%s\\r"
                   CN_RELAY "%s\\r"
                   CN_WIFI "%s\\r"
                   CN_MQTT_SERVER "%s\\r"
                   CN_WEB_CLIENT "%s\\r"
                   CN_REMOTE_HUMAN "%s\\r"
                   CN_REMOTE_LIGHT "%s\\r"
                   CN_REMOTE_DOOR "%s",
                   conn.temp_humi_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   conn.pm_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   conn.gas_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   relay_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   wifi_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   mqtt_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   web_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   conn.human_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   conn.light_connected ? CN_CONNECTED : CN_DISCONNECTED,
                   conn.door_connected ? CN_CONNECTED : CN_DISCONNECTED);

    if ((len > 0) && ((size_t)len < sizeof(status_text)))
    {
        Screen_SendText(UI_INDEX_STATUS_ID, status_text);
    }

    Screen_SendText(UI_INDEX_DATE_ID, date_text);
    Screen_SendText(UI_INDEX_TIME_ID, hm_text);
}

/**
 * @brief   发送传感器数据页内容
 *
 * 根据各传感器连接状态，发送实时数据或 "ERR" 错误标识：
 *   - 温度/湿度、PM2.5、气体浓度
 *   - 人体在离、人体距离
 *   - 光照强度、门磁状态
 */
static void Screen_SendSensorStatus(void)
{
    AllSensorConnectionStatus_t conn;
    AllSensorData_t data;
    char value[32];

    AllSensorData_GetConnectionStatus(&conn);
    AllSensorData_GetSnapshot(&data);

    if (conn.temp_humi_connected)
    {
        snprintf(value, sizeof(value), "%.1f", (double)data.temperature);
        Screen_SendText(UI_TEMP_ID, value);
        snprintf(value, sizeof(value), "%.1f", (double)data.humidity);
        Screen_SendText(UI_HUMI_ID, value);
    }
    else
    {
        Screen_SendText(UI_TEMP_ID, CN_ERROR);
        Screen_SendText(UI_HUMI_ID, CN_ERROR);
    }

    if (conn.pm_connected)
    {
        snprintf(value, sizeof(value), "%u", (unsigned int)data.pm25);
        Screen_SendText(UI_PM_ID, value);
    }
    else
    {
        Screen_SendText(UI_PM_ID, CN_ERROR);
    }

    if (conn.gas_connected)
    {
        snprintf(value, sizeof(value), "%u", (unsigned int)data.gas);
        Screen_SendText(UI_GAS_ID, value);
    }
    else
    {
        Screen_SendText(UI_GAS_ID, CN_ERROR);
    }

    if (conn.human_connected)
    {
        Screen_SendText(UI_HUMAN_ID, (data.human_presence != 0U) ? CN_PRESENT : CN_ABSENT);
        snprintf(value, sizeof(value), "%u", (unsigned int)data.human_distance);
        Screen_SendText(UI_DIST_ID, value);
    }
    else
    {
        Screen_SendText(UI_HUMAN_ID, CN_ERROR);
        Screen_SendText(UI_DIST_ID, CN_ERROR);
    }

    if (conn.light_connected)
    {
        snprintf(value, sizeof(value), "%u", (unsigned int)data.light_lux);
        Screen_SendText(UI_LIGHT_ID, value);
    }
    else
    {
        Screen_SendText(UI_LIGHT_ID, CN_ERROR);
    }

    if (conn.door_connected)
    {
        Screen_SendText(UI_DOOR_ID, (data.door_closed != 0U) ? CN_DOOR_CLOSE : CN_DOOR_OPEN);
    }
    else
    {
        Screen_SendText(UI_DOOR_ID, CN_ERROR);
    }
}

/**
 * @brief   初始化串口屏模块
 *
 * 执行顺序：
 *   1. 初始化 UART4 底层硬件
 *   2. 发送 bkcmd=0 关闭屏幕返回码以减少 RX 噪声
 *   3. 创建 TX 互斥锁
 *   4. 清零所有帧解析窗口、动作队列、时间变量、文本缓存
 *   5. 设置默认页面为主页，并触发一次即时刷新
 */
void Screen_Init(void)
{
    UART_Screen_Init();                         /* 初始化 UART4 硬件 */
    /* 发送 bkcmd=0 关闭执行结果返回码，减少 RX 上的无用响应数据 */
    Screen_SendCommandRaw("bkcmd=0");
    if (s_screen_tx_mutex == NULL)
    {
        s_screen_tx_mutex = xSemaphoreCreateMutex();
    }

    memset(s_frame_window, 0, sizeof(s_frame_window));
    memset(s_frame_window7, 0, sizeof(s_frame_window7));
    memset(s_frame_window8, 0, sizeof(s_frame_window8));
    memset(s_action_queue, 0, sizeof(s_action_queue));
    s_action_queue_head = 0U;
    s_action_queue_tail = 0U;
    s_action_queue_count = 0U;
    memset(s_time_local_str, 0, sizeof(s_time_local_str));
    memset(s_time_tz_str, 0, sizeof(s_time_tz_str));
    snprintf(s_time_date_str, sizeof(s_time_date_str), "----/--/--");
    memcpy(s_time_hm_str, "--:--", 6U);
    s_time_ts = 0U;
    s_time_has_ts = 0U;
    s_time_valid = 0U;
    memset(&s_local_clock, 0, sizeof(s_local_clock));
    Screen_TextCacheReset();
    s_relay_connected_cache = 0U;
    s_relay_poll_last_tick = 0U;
    s_active_page = SCREEN_CMD_INDEX_PAGE;
    s_force_text_send = 0U;
    s_immediate_refresh_pending = 1U;
}

static void Screen_SendActivePageContent(void)
{
    if (s_active_page == SCREEN_CMD_SENSOR_PAGE)
    {
        Screen_SendSensorStatus();
    }
    else if (s_active_page == SCREEN_CMD_INDEX_PAGE)
    {
        Screen_SendIndexStatus();
    }
    else if (s_active_page == SCREEN_CMD_CHANGE1_PAGE)
    {
        Screen_SendChange1Thresholds();
    }
    else if (s_active_page == SCREEN_CMD_CHANGE2_PAGE)
    {
        Screen_SendChange2Thresholds();
    }
    else if (s_active_page == SCREEN_CMD_CHANGE3_PAGE)
    {
        Screen_SendChange3Thresholds();
    }
    else if (s_active_page == SCREEN_CMD_CHANGE4_PAGE)
    {
        Screen_SendChange4Thresholds();
    }
    else if (s_active_page == SCREEN_CMD_CHANGE5_PAGE)
    {
        Screen_SendChange5Thresholds();
    }
    else if (s_active_page == SCREEN_CMD_CHANGE6_PAGE)
    {
        Screen_SendChange6Thresholds();
    }
    else if (s_active_page == SCREEN_CMD_HAND1_PAGE)
    {
        Screen_SendHand1Status();
    }
    else if (s_active_page == SCREEN_CMD_HAND2_PAGE)
    {
        Screen_SendHand2Status();
    }
    else if (s_active_page == SCREEN_CMD_HAND3_PAGE)
    {
        Screen_SendHand3Status();
    }
    else if (s_active_page == SCREEN_CMD_HAND4_PAGE)
    {
        Screen_SendHand4Status();
    }
    else if (s_active_page == SCREEN_CMD_RELAY_PAGE)
    {
        Screen_SendRelayStatus();
    }
    else if ((s_active_page == SCREEN_CMD_SETTING_PAGE) ||
             (s_active_page == SCREEN_CMD_SETTING_PAGE_ALT))
    {
        Screen_SendSettingStatus();
    }
}

static uint8_t Screen_PageNeedsPeriodicRefresh(uint8_t page_cmd)
{
    if ((page_cmd == SCREEN_CMD_CHANGE1_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE2_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE3_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE4_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE5_PAGE) ||
        (page_cmd == SCREEN_CMD_CHANGE6_PAGE))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief   串口屏主任务函数 (FreeRTOS 任务入口)
 * @param   arg  任务参数（未使用）
 *
 * 无限循环中执行以下操作：
 *   1. 从 UART4 接收缓冲区读取所有待处理字节，逐字节解析帧
 *   2. 更新本地时钟
 *   3. 根据当前活动页面，处理即时刷新或周期定时刷新
 *   4. 每次循环延时 10ms 让出 CPU
 *
 * 阈值设置页仅在进入页面时即时刷新一次，
 * 手动控制页和其余常规状态页保持 500ms 周期刷新。
 */
void Screen_Task(void *arg)
{
    TickType_t last_refresh = xTaskGetTickCount();
    uint8_t rx_byte;

    (void)arg;

    for (;;)
    {
        /* 步骤1: 消费 UART4 接收缓冲区中的所有字节 */
        while (UART_Screen_ReadByte(&rx_byte))
        {
            Screen_ProcessByte(rx_byte);  /* 滑动窗口帧解析 */
        }

        /* 步骤2: 更新本地软时钟 */
        Screen_TimeUpdateFromTick();

        {
            TickType_t now = xTaskGetTickCount();
            TickType_t refresh_ticks = pdMS_TO_TICKS(SCREEN_REFRESH_INTERVAL_MS);

            if (s_immediate_refresh_pending > 0U)
            {
                /* 有即时刷新请求: 页面切换/时间同步/手动控制等触发 */
                Screen_SendActivePageContent();
                last_refresh = now;
                s_immediate_refresh_pending--;
                if (s_force_text_send > 0U)
                {
                    s_force_text_send--;
                }
            }
            else if (Screen_PageNeedsPeriodicRefresh(s_active_page) &&
                     ((now - last_refresh) >= refresh_ticks))
            {
                /* 周期定时刷新: 保持显示内容与实际数据同步 */
                /* 手动控制页强制重发文本，确保状态实时反映 */
                if ((s_active_page == SCREEN_CMD_HAND1_PAGE) ||
                    (s_active_page == SCREEN_CMD_HAND2_PAGE) ||
                    (s_active_page == SCREEN_CMD_HAND3_PAGE) ||
                    (s_active_page == SCREEN_CMD_HAND4_PAGE))
                {
                    s_force_text_send = 1U;
                }

                Screen_SendActivePageContent();
                s_force_text_send = 0U;
                last_refresh = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
