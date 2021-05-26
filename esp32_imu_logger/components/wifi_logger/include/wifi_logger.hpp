#ifndef WIFI_LOGGER_H
#define WIFI_LOGGER_H

#include "utils.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "udp_handler.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define MESSAGE_QUEUE_SIZE 100
#define BUFFER_SIZE 512

#define wifi_log_e(TAG, fmt, ...) generate_log_message(ESP_LOG_ERROR, TAG, __LINE__, __func__, fmt, __VA_ARGS__);
#define wifi_log_w(TAG, fmt, ...) generate_log_message(ESP_LOG_WARN, TAG, __LINE__, __func__, fmt, __VA_ARGS__);
#define wifi_log_i(TAG, fmt, ...) generate_log_message(ESP_LOG_INFO, TAG, __LINE__, __func__, fmt, __VA_ARGS__);
#define wifi_log_d(TAG, fmt, ...) generate_log_message(ESP_LOG_DEBUG, TAG, __LINE__, __func__, fmt, __VA_ARGS__);
#define wifi_log_v(TAG, fmt, ...) generate_log_message(ESP_LOG_VERBOSE, TAG, __LINE__, __func__, fmt, __VA_ARGS__);

typedef struct 
{
    char *ip;
    uint16_t port;
} HostAddress_t;

esp_err_t init_queue(void);
void init_wifi(void);
esp_err_t send_to_queue(char* log_message);
char* receive_from_queue(void);
void generate_log_message(esp_log_level_t level, const char *TAG, int line, const char *func, const char *fmt, ...);
int system_log_message_route(const char* fmt, va_list tag);
void wifi_logger(void *xLogHost);
void start_wifi_logger(char* host_ip, uint16_t host_port);

#ifdef __cplusplus
}
#endif

#endif