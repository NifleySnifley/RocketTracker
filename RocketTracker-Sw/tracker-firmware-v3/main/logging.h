#ifndef LOGGING_H
#define LOGGING_H

#include "esp_partition.h"
#include "esp_flash.h"
#include "nvs.h"

#define NVS_LOG_OFFSET "log_offset"

typedef struct logger_t {
    esp_flash_t* ext_flash;
    nvs_handle_t log_nvs;
    size_t log_address;
    const esp_partition_t* log_partition;
    uint8_t* log_buffer;
} logger_t;

esp_err_t logger_init(logger_t* logger, esp_flash_t* flash);
esp_err_t logger_write_data_now(logger_t* logger, uint16_t data_type, uint8_t* data, int data_length);
int logger_get_num_queued_logs(logger_t* logger);
esp_err_t logger_flush(logger_t* logger);

#endif