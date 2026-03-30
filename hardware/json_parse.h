/**
 * @file    json_parse.h
 * @brief   轻量级JSON值提取器（头文件）
 *
 * 提供从平坦JSON字符串中提取指定键值的函数。
 * 不依赖外部JSON库，适合嵌入式资源受限环境。
 *
 * 支持类型：字符串、整数、浮点数、布尔值
 * 匹配方式：自动给键名加引号，避免部分匹配问题
 */
#ifndef JSON_PARSE_H
#define JSON_PARSE_H

#include <stdint.h>

/**
 * @brief   从 JSON 字符串中提取字符串值
 * @param   json      JSON字符串
 * @param   key       裸键名（不含引号，如 "type"）
 * @param   out       输出缓冲区
 * @param   out_size  输出缓冲区大小
 * @return  1=成功, 0=失败
 */
uint8_t JsonParse_GetString(const char *json, const char *key, char *out, uint16_t out_size);

/**
 * @brief   从 JSON 字符串中提取整数值
 * @param   json       JSON字符串
 * @param   key        裸键名
 * @param   out_value  输出整数值
 * @return  1=成功, 0=失败
 */
uint8_t JsonParse_GetInt(const char *json, const char *key, int *out_value);

/**
 * @brief   从 JSON 字符串中提取浮点数值
 * @param   json       JSON字符串
 * @param   key        裸键名
 * @param   out_value  输出浮点值
 * @return  1=成功, 0=失败
 */
uint8_t JsonParse_GetFloat(const char *json, const char *key, float *out_value);

/**
 * @brief   从 JSON 字符串中提取布尔值
 * @param   json       JSON字符串
 * @param   key        裸键名
 * @param   out_value  输出布尔值（1=true, 0=false）
 * @return  1=成功, 0=失败
 */
uint8_t JsonParse_GetBool(const char *json, const char *key, uint8_t *out_value);

#endif /* JSON_PARSE_H */
