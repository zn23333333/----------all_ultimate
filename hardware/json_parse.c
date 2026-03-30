/**
 * @file    json_parse.c
 * @brief   轻量级JSON值提取器实现
 *
 * 在平坦JSON字符串中按键名搜索值，支持字符串/整数/浮点/布尔。
 * 内部用 \"key\" 格式匹配，避免部分键名冲突（如 type 与 subtype）。
 *
 * 限制：不支持嵌套JSON、数组、转义字符。
 */
#include "json_parse.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * @brief   内部函数：定位指定键的值起始位置
 *
 * 将裸键名包装为 "key" 格式进行搜索，找到后跳过冒号和空白，
 * 返回值的起始位置指针。
 *
 * @param   json       JSON字符串
 * @param   key        裸键名
 * @param   out_start  输出：值起始位置指针
 * @return  1=找到, 0=未找到
 */
static uint8_t JsonParse_FindValue(const char *json, const char *key, const char **out_start)
{
    char pattern[48];
    int pattern_len;
    const char *p;

    if ((json == NULL) || (key == NULL) || (out_start == NULL))
    {
        return 0U;
    }

    pattern_len = snprintf(pattern, sizeof(pattern), "\"%s\"", key);  /* 组装 "key" 格式匹配串 */
    if ((pattern_len <= 0) || (pattern_len >= (int)sizeof(pattern)))
    {
        return 0U;
    }

    p = strstr(json, pattern);  /* 在JSON中搜索 "key" */
    if (p == NULL)
    {
        return 0U;
    }

    p = strchr(p + pattern_len, ':');  /* 定位冒号 */
    if (p == NULL)
    {
        return 0U;
    }
    p++;  /* 跳过冒号 */

    while ((*p == ' ') || (*p == '\t'))
    {  /* 跳过空白字符 */
        p++;
    }

    *out_start = p;
    return 1U;
}

/**
 * @brief   提取字符串值（去掉引号，截取到输出缓冲区）
 */
uint8_t JsonParse_GetString(const char *json, const char *key, char *out, uint16_t out_size)
{
    const char *start;
    const char *end;
    uint16_t len;

    if ((out == NULL) || (out_size < 2U))
    {
        return 0U;
    }

    if (!JsonParse_FindValue(json, key, &start))
    {
        return 0U;
    }

    if (*start != '"')
    {
        return 0U;
    }
    start++;

    end = strchr(start, '"');
    if (end == NULL)
    {
        return 0U;
    }

    len = (uint16_t)(end - start);
    if (len >= out_size)
    {
        len = (uint16_t)(out_size - 1U);
    }

    memcpy(out, start, len);
    out[len] = '\0';
    return 1U;
}

/**
 * @brief   提取整数值（支持负数，支持引号包装的数字）
 */
uint8_t JsonParse_GetInt(const char *json, const char *key, int *out_value)
{
    const char *p;
    int value = 0;
    int sign = 1;
    uint8_t has_digit = 0U;

    if (out_value == NULL)
    {
        return 0U;
    }

    if (!JsonParse_FindValue(json, key, &p))
    {
        return 0U;
    }

    if (*p == '"')
    {
        p++;
    }

    if (*p == '-') { sign = -1; p++; }  /* 处理负号 */

    while ((*p >= '0') && (*p <= '9'))
    {
        has_digit = 1U;
        value = value * 10 + (*p - '0');
        p++;
    }
    if (!has_digit) return 0U;

    *out_value = value * sign;
    return 1U;
}

/**
 * @brief   提取浮点数值（先提取数字字符串再用atof转换）
 */
uint8_t JsonParse_GetFloat(const char *json, const char *key, float *out_value)
{
    const char *p;
    char num_buf[24];
    uint16_t n = 0U;
    char ch;

    if (out_value == NULL)
    {
        return 0U;
    }

    if (!JsonParse_FindValue(json, key, &p))
    {
        return 0U;
    }

    if (*p == '"')
    {
        p++;
    }

    while ((n + 1U) < sizeof(num_buf))
    {
        ch = *p;
        if (((ch >= '0') && (ch <= '9')) || (ch == '-') || (ch == '+') || (ch == '.'))
        {
            num_buf[n++] = ch;
            p++;
        }
        else break;
    }
    if (n == 0U) return 0U;

    num_buf[n] = '\0';
    *out_value = (float)atof(num_buf);
    return 1U;
}

/**
 * @brief   提取布尔值
 *
 * 支持多种格式：true/false、"1"/"0"、1/0
 */
uint8_t JsonParse_GetBool(const char *json, const char *key, uint8_t *out_value)
{
    const char *p;

    if (out_value == NULL)
    {
        return 0U;
    }

    if (!JsonParse_FindValue(json, key, &p))
    {
        return 0U;
    }

    if ((strncmp(p, "true", 4U) == 0) || (*p == '1') ||
        ((p[0] == '"') && (p[1] == '1')))
        {
        *out_value = 1U;
        return 1U;
    }
    if ((strncmp(p, "false", 5U) == 0) || (*p == '0') ||
        ((p[0] == '"') && (p[1] == '0')))
        {
        *out_value = 0U;
        return 1U;
    }

    return 0U;
}
