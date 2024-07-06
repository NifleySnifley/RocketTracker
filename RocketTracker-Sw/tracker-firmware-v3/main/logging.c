#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "logging.h"
#include "defs.h"
#include "memory.h"
#include "rtc.h"
#include "time.h"

// int64_t util_time_us() {
//     struct timeval tv_now;
//     gettimeofday(&tv_now, NULL);
//     int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
//     return time_us;
// }

esp_err_t logger_init(logger_t* logger, esp_flash_t* flash) {
    logger->ext_flash = flash;

    esp_err_t e = esp_partition_register_external(
        logger->ext_flash, 0, logger->ext_flash->size,
        LOG_PARTITION_LABEL, ESP_PARTITION_TYPE_DATA_LOG,
        ESP_PARTITION_SUBTYPE_DATA_LOG, (&logger->log_partition)
    );

    if (e != ESP_OK)
        return ESP_ERR_NOT_FOUND;

    ESP_LOGI("LOGGER", "Successfully created log partition");

    // int64_t start = util_time_us;
    // esp_partition_erase_range()

    e = nvs_open("log", NVS_READWRITE, &logger->log_nvs);
    if (e != ESP_OK) {
        ESP_LOGI("LOGGER", "Error (%s) opening log NVS handle!\n", esp_err_to_name(e));
    }

    // log_start_offset = nvs_find_key
    e = nvs_find_key(logger->log_nvs, "log_offset", NULL);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_u32(logger->log_nvs, "log_offset", 0);
    } else if (e != ESP_OK) {
        ESP_LOGE("LOGGER", "Error getting `log_offset`");
    } else {
        ESP_ERROR_CHECK(nvs_get_u32(logger->log_nvs, "log_offset", &logger->log_address));
    }

    ESP_LOGI("LOGGER", "log_address: %d", logger->log_address);

    ESP_LOGI("FLASH", "Erase size: %" PRIu32 ", Address: %" PRIu32 ", readonly: %s", logger->log_partition->erase_size, logger->log_partition->address, logger->log_partition->readonly ? "true" : "false");
    return ESP_OK;
}

esp_err_t logger_write_data_now(logger_t* logger, uint16_t data_type, uint8_t* data, int data_length) {


    return ESP_OK;
}

int logger_get_num_queued_logs(logger_t* logger) {


    return -1;
}

esp_err_t logger_flush(logger_t* logger) {


    return ESP_OK;
}