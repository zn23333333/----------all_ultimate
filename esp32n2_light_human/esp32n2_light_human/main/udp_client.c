/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include <lwip/netdb.h>

#define UDP_REPORT_PORT 4001
#define UDP_DISCOVERY_PORT UDP_REPORT_PORT
#define UDP_DISCOVERY_RETRY_MS 2000
#define UDP_DISCOVERY_REFRESH_MS 30000
#define UDP_DISCOVERY_TIMEOUT_MS 1200
#define UDP_GATEWAY_ID_MAX_LEN 32

#define LD2410S_UART_NUM UART_NUM_2
#define LD2410S_UART_TXD GPIO_NUM_17
#define LD2410S_UART_RXD GPIO_NUM_16
#define LD2410S_UART_BAUD 115200
#define LD2410S_UART_BUF_SIZE 256

#define LD2410S_FRAME_LEN 5
#define LD2410S_FRAME_HEADER 0x6E
#define LD2410S_FRAME_TAIL 0x62

#define LD2410S_USE_OUT_PIN 0
#define LD2410S_OUT_GPIO GPIO_NUM_27

#define GY302_I2C_NUM I2C_NUM_0
#define GY302_I2C_SCL GPIO_NUM_22
#define GY302_I2C_SDA GPIO_NUM_21
#define GY302_I2C_FREQ_HZ 100000

#define BH1750_ADDR 0x23
#define BH1750_CMD_POWER_ON 0x01
#define BH1750_CMD_H_RES 0x10

static const char *TAG = "example";

/* ---- WiFi configuration ---- */
#define WIFI_SSID      CONFIG_EXAMPLE_WIFI_SSID
#define WIFI_PASS      CONFIG_EXAMPLE_WIFI_PASSWORD
#define WIFI_MAX_RETRY 0   /* 0 = retry forever */

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static int s_retry_num = 0;

typedef struct {
    bool presence;
    uint8_t distance;
} ld2410s_data_t;

typedef struct {
    uint32_t lux;
} bh1750_data_t;

static QueueHandle_t s_ld2410s_queue;
static QueueHandle_t s_bh1750_queue;
static SemaphoreHandle_t s_gateway_mutex;
static void gateway_invalidate(void);

typedef struct {
    bool valid;
    struct sockaddr_in addr;
    char id[UDP_GATEWAY_ID_MAX_LEN];
} udp_gateway_info_t;

static udp_gateway_info_t s_gateway_info;
static TickType_t s_next_discovery_tick;

static const char *skip_json_spaces(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        p++;
    }
    return p;
}

static bool json_extract_string_value(const char *json, const char *key, char *out, size_t out_size)
{
    if (out_size == 0) {
        return false;
    }

    char pattern[40];
    const int n = snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    if (n <= 0 || n >= (int)sizeof(pattern)) {
        return false;
    }

    const char *p = strstr(json, pattern);
    if (p == NULL) {
        return false;
    }

    p += n;
    p = skip_json_spaces(p);
    if (*p != ':') {
        return false;
    }
    p = skip_json_spaces(p + 1);
    if (*p != '"') {
        return false;
    }
    p++;

    const char *end = strchr(p, '"');
    if (end == NULL) {
        return false;
    }

    const size_t len = (size_t)(end - p);
    if (len == 0 || len >= out_size) {
        return false;
    }

    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool json_extract_u16_value(const char *json, const char *key, uint16_t *out)
{
    char pattern[40];
    const int n = snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    if (n <= 0 || n >= (int)sizeof(pattern)) {
        return false;
    }

    const char *p = strstr(json, pattern);
    if (p == NULL) {
        return false;
    }

    p += n;
    p = skip_json_spaces(p);
    if (*p != ':') {
        return false;
    }
    p = skip_json_spaces(p + 1);

    char *end_ptr = NULL;
    const long v = strtol(p, &end_ptr, 10);
    if (end_ptr == p || v <= 0 || v > UINT16_MAX) {
        return false;
    }

    *out = (uint16_t)v;
    return true;
}

static bool try_get_broadcast_addr(uint32_t *broadcast_addr)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        netif = esp_netif_get_handle_from_ifkey("ETH_DEF");
    }
    if (netif == NULL) {
        return false;
    }

    esp_netif_ip_info_t ip_info = { 0 };
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return false;
    }

    *broadcast_addr = (ip_info.ip.addr & ip_info.netmask.addr) | (~ip_info.netmask.addr);
    return true;
}

static void gateway_store(const struct sockaddr_in *addr, const char *id)
{
    if (xSemaphoreTake(s_gateway_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Gateway mutex timeout (store)");
        return;
    }

    s_gateway_info.addr = *addr;
    s_gateway_info.valid = true;
    strncpy(s_gateway_info.id, id, sizeof(s_gateway_info.id) - 1);
    s_gateway_info.id[sizeof(s_gateway_info.id) - 1] = '\0';
    s_next_discovery_tick = xTaskGetTickCount() + pdMS_TO_TICKS(UDP_DISCOVERY_REFRESH_MS);

    xSemaphoreGive(s_gateway_mutex);
}

static bool gateway_get(struct sockaddr_in *addr)
{
    bool ok = false;
    if (xSemaphoreTake(s_gateway_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_gateway_info.valid) {
            *addr = s_gateway_info.addr;
            ok = true;
        }
        xSemaphoreGive(s_gateway_mutex);
    }
    return ok;
}

static void gateway_invalidate(void)
{
    if (xSemaphoreTake(s_gateway_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Gateway mutex timeout (invalidate)");
        return;
    }
    s_gateway_info.valid = false;
    s_next_discovery_tick = 0;
    xSemaphoreGive(s_gateway_mutex);
}

static bool gateway_parse_announce(const char *payload, char *id_out, size_t id_out_size, uint16_t *port_out)
{
    // Expected reply: {"type":"gateway_announce","id":"<gateway_id>","port":<udp_port>}
    char type[32] = { 0 };
    if (!json_extract_string_value(payload, "type", type, sizeof(type))) {
        return false;
    }
    if (strcmp(type, "gateway_announce") != 0) {
        return false;
    }
    if (!json_extract_string_value(payload, "id", id_out, id_out_size)) {
        return false;
    }
    if (!json_extract_u16_value(payload, "port", port_out)) {
        return false;
    }
    return true;
}

static bool gateway_discover_once(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Discovery socket create failed: errno %d", errno);
        return false;
    }

    int yes = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) < 0) {
        ESP_LOGE(TAG, "Discovery setsockopt(SO_BROADCAST) failed: errno %d", errno);
        close(sock);
        return false;
    }

    struct timeval timeout = {
        .tv_sec = UDP_DISCOVERY_TIMEOUT_MS / 1000,
        .tv_usec = (UDP_DISCOVERY_TIMEOUT_MS % 1000) * 1000,
    };
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        ESP_LOGE(TAG, "Discovery setsockopt(SO_RCVTIMEO) failed: errno %d", errno);
        close(sock);
        return false;
    }

    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char request[160];
    const int request_len = snprintf(
        request, sizeof(request),
        "{\"type\":\"gateway_discover\",\"id\":\"esp32-%02X%02X%02X%02X%02X%02X\",\"want\":\"report_endpoint\"}\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if (request_len <= 0 || request_len >= (int)sizeof(request)) {
        close(sock);
        return false;
    }

    struct sockaddr_in bcast_addr = { 0 };
    bcast_addr.sin_family = AF_INET;
    bcast_addr.sin_port = htons(UDP_DISCOVERY_PORT);
    bcast_addr.sin_addr.s_addr = INADDR_BROADCAST;
    (void)try_get_broadcast_addr(&bcast_addr.sin_addr.s_addr);

    int sent = sendto(sock, request, request_len, 0, (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));
    if (sent < 0) {
        ESP_LOGE(TAG, "Discovery broadcast send failed: errno %d", errno);
        close(sock);
        return false;
    }

    char rx_buf[192];
    struct sockaddr_in from_addr = { 0 };
    socklen_t from_len = sizeof(from_addr);
    const int len = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0, (struct sockaddr *)&from_addr, &from_len);
    if (len <= 0) {
        close(sock);
        return false;
    }
    rx_buf[len] = '\0';

    char gateway_id[UDP_GATEWAY_ID_MAX_LEN] = { 0 };
    uint16_t report_port = 0;
    if (!gateway_parse_announce(rx_buf, gateway_id, sizeof(gateway_id), &report_port)) {
        ESP_LOGW(TAG, "Discovery reply ignored: %s", rx_buf);
        close(sock);
        return false;
    }

    from_addr.sin_port = htons(report_port);
    gateway_store(&from_addr, gateway_id);

    char ip_buf[INET_ADDRSTRLEN];
    inet_ntoa_r(from_addr.sin_addr, ip_buf, sizeof(ip_buf));
    ESP_LOGI(TAG, "Gateway discovered: id=%s addr=%s:%u", gateway_id, ip_buf, (unsigned)report_port);

    close(sock);
    return true;
}

static bool gateway_ensure(void)
{
    bool allow_discovery = false;
    bool has_gateway = false;
    TickType_t next_interval = pdMS_TO_TICKS(UDP_DISCOVERY_RETRY_MS);
    const TickType_t now = xTaskGetTickCount();

    if (xSemaphoreTake(s_gateway_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        has_gateway = s_gateway_info.valid;
        if (has_gateway) {
            next_interval = pdMS_TO_TICKS(UDP_DISCOVERY_REFRESH_MS);
        }

        if ((int32_t)(now - s_next_discovery_tick) >= 0) {
            s_next_discovery_tick = now + next_interval;
            allow_discovery = true;
        }
        xSemaphoreGive(s_gateway_mutex);
    }

    if (!allow_discovery) {
        return has_gateway;
    }

    if (gateway_discover_once()) {
        return true;
    }

    struct sockaddr_in addr = { 0 };
    return gateway_get(&addr);
}

/* ---- WiFi STA with auto-reconnect ---- */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        gateway_invalidate();
        if (WIFI_MAX_RETRY == 0 || s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "WiFi disconnected, retry #%d ...", s_retry_num);
        } else {
            ESP_LOGE(TAG, "WiFi connect failed after %d retries", s_retry_num);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected, IP=" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any_id;
    esp_event_handler_instance_t inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished, waiting for connection ...");

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected, starting tasks");
}

static void ld2410s_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = LD2410S_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(LD2410S_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LD2410S_UART_NUM, LD2410S_UART_TXD, LD2410S_UART_RXD,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LD2410S_UART_NUM, LD2410S_UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    if (LD2410S_USE_OUT_PIN) {
        gpio_config_t io_conf = { 0 };
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL << LD2410S_OUT_GPIO;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
}

static bool ld2410s_parse_frame(const uint8_t *frame, ld2410s_data_t *out)
{
    if (frame[0] != LD2410S_FRAME_HEADER || frame[4] != LD2410S_FRAME_TAIL) {
        return false;
    }

    out->presence = (frame[1] == 0x02);
    out->distance = frame[2];

    if (LD2410S_USE_OUT_PIN) {
        out->presence = gpio_get_level(LD2410S_OUT_GPIO);
    }

    if (!out->presence) {
        out->distance = 0;
    }

    return true;
}

static void ld2410s_uart_task(void *pvParameters)
{
    uint8_t rx_buf[64];
    uint8_t frame[LD2410S_FRAME_LEN] = { 0 };
    int frame_idx = 0;
    ld2410s_data_t data = { 0 };

    while (1) {
        const int len = uart_read_bytes(LD2410S_UART_NUM, rx_buf, sizeof(rx_buf),
                                        pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; i++) {
            const uint8_t b = rx_buf[i];

            if (frame_idx == 0) {
                if (b != LD2410S_FRAME_HEADER) {
                    continue;
                }
            }

            frame[frame_idx++] = b;
            if (frame_idx == LD2410S_FRAME_LEN) {
                if (ld2410s_parse_frame(frame, &data)) {
                    xQueueOverwrite(s_ld2410s_queue, &data);
                } else if (frame[LD2410S_FRAME_LEN - 1] == LD2410S_FRAME_HEADER) {
                    frame[0] = LD2410S_FRAME_HEADER;
                    frame_idx = 1;
                    continue;
                }
                frame_idx = 0;
            }
        }
    }
}

static void gy302_i2c_init(void)
{
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GY302_I2C_SDA,
        .scl_io_num = GY302_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = GY302_I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(GY302_I2C_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(GY302_I2C_NUM, conf.mode, 0, 0, 0));
}

static esp_err_t bh1750_write_cmd(uint8_t cmd)
{
    return i2c_master_write_to_device(GY302_I2C_NUM, BH1750_ADDR, &cmd, 1, pdMS_TO_TICKS(100));
}

static esp_err_t bh1750_read_raw(uint16_t *raw)
{
    uint8_t data[2] = { 0 };
    esp_err_t err = i2c_master_read_from_device(GY302_I2C_NUM, BH1750_ADDR, data, sizeof(data),
                                                pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        return err;
    }
    *raw = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

static void bh1750_task(void *pvParameters)
{
    uint16_t raw = 0;
    bh1750_data_t data = { 0 };

    gy302_i2c_init();
    vTaskDelay(pdMS_TO_TICKS(10));

    if (bh1750_write_cmd(BH1750_CMD_POWER_ON) != ESP_OK) {
        ESP_LOGW(TAG, "BH1750 power on failed");
    }
    if (bh1750_write_cmd(BH1750_CMD_H_RES) != ESP_OK) {
        ESP_LOGW(TAG, "BH1750 mode set failed");
    }

    while (1) {
        if (bh1750_write_cmd(BH1750_CMD_H_RES) != ESP_OK) {
            ESP_LOGW(TAG, "BH1750 measure start failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(180));
        if (bh1750_read_raw(&raw) == ESP_OK) {
            data.lux = (uint32_t)((float)raw / 1.2f + 0.5f);
            xQueueOverwrite(s_bh1750_queue, &data);
        } else {
            ESP_LOGW(TAG, "BH1750 read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void udp_client_task(void *pvParameters)
{
    const int addr_family = AF_INET;
    const int ip_protocol = IPPROTO_IP;
    ld2410s_data_t data = { 0 };
    uint32_t seq = 0;
    bool waiting_gateway_logged = false;

    while (1) {
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Human report socket created");

        TickType_t last_wake = xTaskGetTickCount();
        while (1) {
            /* Wait for WiFi to be connected before doing anything */
            if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT)) {
                ESP_LOGW(TAG, "WiFi not connected, pausing human report");
                xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                    pdFALSE, pdFALSE, portMAX_DELAY);
                last_wake = xTaskGetTickCount();
                gateway_invalidate();
                ESP_LOGI(TAG, "WiFi reconnected, resuming human report");
            }

            xQueueReceive(s_ld2410s_queue, &data, 0);

            struct sockaddr_in dest_addr = { 0 };
            if (!gateway_ensure() || !gateway_get(&dest_addr)) {
                if (!waiting_gateway_logged) {
                    ESP_LOGW(TAG, "Human report waiting for gateway discovery");
                    waiting_gateway_logged = true;
                }
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
                continue;
            }
            waiting_gateway_logged = false;

            char payload[96];
            const int len = snprintf(payload, sizeof(payload),
                                     "{\"type\":\"human\",\"presence\":%s,\"distance\":%u,\"seq\":%u}\n",
                                     data.presence ? "true" : "false",
                                     (unsigned)data.distance,
                                     (unsigned)seq);
            if (len <= 0 || len >= (int)sizeof(payload)) {
                ESP_LOGW(TAG, "Payload format error");
                continue;
            }

            int err = sendto(sock, payload, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                gateway_invalidate();
                break;
            }
            ESP_LOGI(TAG, "Sent: %s", payload);
            seq++;

            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void udp_light_task(void *pvParameters)
{
    const int addr_family = AF_INET;
    const int ip_protocol = IPPROTO_IP;
    bh1750_data_t data = { 0 };
    uint32_t seq = 0;
    bool waiting_gateway_logged = false;

    while (1) {
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Light report socket created");

        TickType_t last_wake = xTaskGetTickCount();
        while (1) {
            /* Wait for WiFi to be connected before doing anything */
            if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT)) {
                ESP_LOGW(TAG, "WiFi not connected, pausing light report");
                xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                    pdFALSE, pdFALSE, portMAX_DELAY);
                last_wake = xTaskGetTickCount();
                gateway_invalidate();
                ESP_LOGI(TAG, "WiFi reconnected, resuming light report");
            }

            xQueueReceive(s_bh1750_queue, &data, 0);

            struct sockaddr_in dest_addr = { 0 };
            if (!gateway_ensure() || !gateway_get(&dest_addr)) {
                if (!waiting_gateway_logged) {
                    ESP_LOGW(TAG, "Light report waiting for gateway discovery");
                    waiting_gateway_logged = true;
                }
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
                continue;
            }
            waiting_gateway_logged = false;

            char payload[96];
            const int len = snprintf(payload, sizeof(payload),
                                     "{\"type\":\"light\",\"lux\":%u,\"seq\":%u}\n",
                                     (unsigned)data.lux,
                                     (unsigned)seq);
            if (len <= 0 || len >= (int)sizeof(payload)) {
                ESP_LOGW(TAG, "Payload format error");
                continue;
            }

            int err = sendto(sock, payload, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                gateway_invalidate();
                break;
            }
            ESP_LOGI(TAG, "Sent: %s", payload);
            seq++;

            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* WiFi STA with auto-reconnect */
    wifi_init_sta();

    gpio_config_t d23_conf = { 0 };
    d23_conf.intr_type = GPIO_INTR_DISABLE;
    d23_conf.mode = GPIO_MODE_OUTPUT;
    d23_conf.pin_bit_mask = 1ULL << GPIO_NUM_23;
    d23_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    d23_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&d23_conf));
    gpio_set_level(GPIO_NUM_23, 0);

    ld2410s_uart_init();
    s_ld2410s_queue = xQueueCreate(1, sizeof(ld2410s_data_t));
    if (s_ld2410s_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor queue");
        return;
    }

    s_bh1750_queue = xQueueCreate(1, sizeof(bh1750_data_t));
    if (s_bh1750_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create light queue");
        return;
    }

    s_gateway_mutex = xSemaphoreCreateMutex();
    if (s_gateway_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create gateway mutex");
        return;
    }
    memset(&s_gateway_info, 0, sizeof(s_gateway_info));
    s_next_discovery_tick = 0;

    xTaskCreate(ld2410s_uart_task, "ld2410s_uart", 4096, NULL, 6, NULL);
    xTaskCreate(bh1750_task, "bh1750", 4096, NULL, 6, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(udp_light_task, "udp_light", 4096, NULL, 5, NULL);
}
