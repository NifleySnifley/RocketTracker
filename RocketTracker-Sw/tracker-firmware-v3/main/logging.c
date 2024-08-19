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
#include "math.h"
#include <stdlib.h>
#include "sys/time.h"
#include "configuration.h"
#include "esp_check.h"


int32_t LOG_HZ_AUTO_ARMED = 10;
int32_t LOG_HZ_AUTO_LIFTOFF = 200;
int32_t LOG_HZ_AUTO_FLIGHT = 60;
int32_t LOG_HZ_AUTO_LANDED = 1;
float LOG_AUTO_LIFTOFF_DURATION = 10;
float LIFTOFF_ACC_THRESHOLD_G = 4.0f;
float LIFTOFF_DURATION = 10.0f;

log_data_default_t log_data = { 0 };

size_t min(size_t a, size_t b) {
    return (a < b) ? a : b;
}

int64_t util_time_us() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    return time_us;
}

uint8_t logger_tmp_buffer[256] = { 0 };

void logger_task(void* arg) {
    logger_t* logger = (logger_t*)arg;
    ESP_LOGI("LOGGER", "Logger task starting");

    while (1) {
        log_request_t req = { 0 };
        // Do logger IO things
        if (xQueueReceive(logger->log_queue, &req, portMAX_DELAY) == pdPASS) {
            logger_log_data(logger, req.typeid, req.data, req.data_length, req.timestamp);
            // ESP_LOGI("LOGGER", "Logged %d bytes from queue.", req.data_length);
            // THIS IS IMPORTANT!!!
            free(req.data);
        }
    }
}

esp_err_t logger_init(logger_t* logger, esp_flash_t* flash, SemaphoreHandle_t sem) {
    logger->log_flash_sem = sem;
    logger->ext_flash = flash;

    esp_err_t e = esp_partition_register_external(
        logger->ext_flash, 0, logger->ext_flash->size,
        LOG_PARTITION_LABEL, ESP_PARTITION_TYPE_DATA_LOG,
        ESP_PARTITION_SUBTYPE_DATA_LOG, (&logger->log_partition)
    );

    if (e != ESP_OK)
        return ESP_ERR_NOT_FOUND;

    ESP_LOGI("LOGGER", "Successfully created log partition");

    e = nvs_open("log", NVS_READWRITE, &logger->log_nvs);
    if (e != ESP_OK) {
        ESP_LOGI("LOGGER", "Error (%s) opening log NVS handle!\n", esp_err_to_name(e));
    }

    e = nvs_find_key(logger->log_nvs, NVS_LOG_START, NULL);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_u32(logger->log_nvs, NVS_LOG_START, 0);
    } else if (e != ESP_OK) {
        ESP_LOGE("LOGGER", "Error getting NVS key '%s'", NVS_LOG_START);
    } else {
        uint32_t start_addr;
        ESP_ERROR_CHECK(nvs_get_u32(logger->log_nvs, NVS_LOG_START, &start_addr));
        logger->log_start_address = start_addr;
    }

    e = nvs_find_key(logger->log_nvs, NVS_LOG_END, NULL);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_u32(logger->log_nvs, NVS_LOG_END, 0);
    } else if (e != ESP_OK) {
        ESP_LOGE("LOGGER", "Error getting NVS key '%s'", NVS_LOG_END);
    } else {
        uint32_t end_addr;
        ESP_ERROR_CHECK(nvs_get_u32(logger->log_nvs, NVS_LOG_END, &end_addr));
        logger->log_end_address = end_addr;
    }

    // DEBUG!!!
    // logger->log_end_address = 0;
    // logger->log_start_address = 0;
    // logger_flush(logger);
    // esp_flash_erase_region(logger->ext_flash, 0, LOG_FLASH_SECTOR_SIZE);

    ESP_LOGI("LOGGER", "log_start_address: %d, log_end_address: %d", logger->log_start_address, logger->log_end_address);

    ESP_LOGI("FLASH", "Erase size: %" PRIu32 ", Address: %" PRIu32 ", readonly: %s", logger->log_partition->erase_size, logger->log_partition->address, logger->log_partition->readonly ? "true" : "false");

    e = logger_refresh(logger);
    if (e != ESP_OK) {
        ESP_LOGE("LOGGER", "Error '%s' while refreshing logger.", esp_err_to_name(e));
    }

    // Create the queue
    logger->log_queue = xQueueCreate(LOGGER_QUEUE_SIZE, sizeof(log_request_t));
    if (logger->log_queue == NULL) {
        ESP_LOGE("LOGGER", "Error initializing logger queue");
        return ESP_ERR_NOT_FINISHED;
    }

    // Create the logger task
    xTaskCreate(logger_task, "logger_task", 1024 * 4, logger, 18, &logger->log_task);

    config_get_int(CONFIG_LOGGING_LIFTOFF_LOGRATE_KEY, &LOG_HZ_AUTO_LIFTOFF);
    config_get_int(CONFIG_LOGGING_FLIGHT_LOGRATE_KEY, &LOG_HZ_AUTO_FLIGHT);
    config_get_float(CONFIG_LOGGING_LIFTOFF_DURATION_KEY, &LOG_AUTO_LIFTOFF_DURATION);
    config_get_float(CONFIG_LOGGING_LAUNCH_THRESHOLD_KEY, &LIFTOFF_ACC_THRESHOLD_G);
    LOG_HZ_AUTO_ARMED = 10;
    LOG_HZ_AUTO_LANDED = 1;

    return ESP_OK;
}

esp_err_t logger_read_bytes_raw(logger_t* logger, size_t start_address, size_t size, uint8_t* buffer) {
    if (start_address + size > LOG_MEMORY_SIZE_B) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_FALSE(xSemaphoreTakeRecursive(logger->log_flash_sem, portMAX_DELAY) == pdPASS, ESP_ERR_TIMEOUT, "LOGGER", "Error acquiring flash lock for log raw read");
    esp_err_t e = esp_partition_read_raw(logger->log_partition, start_address, buffer, size);
    xSemaphoreGive(logger->log_flash_sem);

    return e;
}

esp_err_t logger_write_bytes_raw(logger_t* logger, size_t start_address, size_t data_length, uint8_t* data) {
    if (start_address + data_length > LOG_MEMORY_SIZE_B) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_FALSE(xSemaphoreTakeRecursive(logger->log_flash_sem, portMAX_DELAY) == pdPASS, ESP_ERR_TIMEOUT, "LOGGER", "Error acquiring flash lock for log raw write");
    esp_err_t e = esp_partition_write_raw(logger->log_partition, start_address, data, data_length);
    xSemaphoreGive(logger->log_flash_sem);

    return e;
}

size_t logger_get_current_log_size(logger_t* logger) {
    if (logger->log_start_address <= logger->log_end_address) {
        return logger->log_end_address - logger->log_start_address;
    } else {
        // Log is wrapped around the end of the buffer
        return (LOG_MEMORY_SIZE_B - logger->log_start_address) + (logger->log_end_address);
    }
}

esp_err_t logger_erase_log(logger_t* logger) {
    if (logger_get_current_log_size(logger) <= 0) {
        return ESP_ERR_NOT_ALLOWED;
    }

    ESP_RETURN_ON_FALSE(xSemaphoreTakeRecursive(logger->log_flash_sem, portMAX_DELAY) == pdPASS, ESP_ERR_TIMEOUT, "LOGGER", "Error acquiring flash lock for log erase");

    esp_err_t e = ESP_OK;

    if (logger->log_start_address < logger->log_end_address) {
        size_t start_addr = (logger->log_start_address / LOG_FLASH_SECTOR_SIZE) * LOG_FLASH_SECTOR_SIZE;
        size_t end_addr = ((logger->log_end_address / LOG_FLASH_SECTOR_SIZE) + 1) * LOG_FLASH_SECTOR_SIZE;
        e |= esp_partition_erase_range(logger->log_partition, start_addr, end_addr - start_addr);
    } else {
        ESP_LOGI("LOGGER", "Erasing wrapped log!");
        // Wrapped
        size_t start_addr = (logger->log_start_address / LOG_FLASH_SECTOR_SIZE) * LOG_FLASH_SECTOR_SIZE;

        size_t end_addr = ((logger->log_end_address / LOG_FLASH_SECTOR_SIZE) + 1) * LOG_FLASH_SECTOR_SIZE;
        e |= esp_partition_erase_range(logger->log_partition, 0, end_addr);
        e |= esp_partition_erase_range(logger->log_partition, start_addr, ((LOG_MEMORY_SIZE_B / LOG_FLASH_SECTOR_SIZE) + 1) * LOG_FLASH_SECTOR_SIZE);
    }

    if (e != ESP_OK) {
        ESP_LOGE("LOGGER", "Error (%s) erasing flash!", esp_err_to_name(e));
    }

    // Start new ringbuffer where it last ended
    logger->log_start_address = logger->log_end_address;

    xSemaphoreGive(logger->log_flash_sem);
    logger_flush(logger);

    return e;
}

esp_err_t logger_clean_log(logger_t* logger) {
    ESP_RETURN_ON_FALSE(xSemaphoreTakeRecursive(logger->log_flash_sem, portMAX_DELAY) == pdPASS, ESP_ERR_TIMEOUT, "LOGGER", "Error acquiring flash lock for log clean");

    esp_err_t e = esp_partition_erase_range(logger->log_partition, 0, (LOG_MEMORY_SIZE_B / LOG_FLASH_SECTOR_SIZE + 1) * LOG_FLASH_SECTOR_SIZE);
    if (e != ESP_OK) {
        xSemaphoreGive(logger->log_flash_sem);
        return e;
    }
    logger->log_start_address = 0;
    logger->log_end_address = 0;
    e = logger_flush(logger);

    xSemaphoreGive(logger->log_flash_sem);
    return e;
}

esp_err_t logger_flush(logger_t* logger) {
    esp_err_t e = nvs_set_u32(logger->log_nvs, NVS_LOG_START, logger->log_start_address);
    e |= nvs_set_u32(logger->log_nvs, NVS_LOG_END, logger->log_end_address);
    return e;
}

esp_err_t logger_refresh(logger_t* logger) {
    size_t cur_end_address = logger->log_end_address - 1;
    if (logger->log_end_address == 0) {
        cur_end_address = LOG_MEMORY_SIZE_B;
    }

    ESP_RETURN_ON_FALSE(xSemaphoreTakeRecursive(logger->log_flash_sem, portMAX_DELAY) == pdPASS, ESP_ERR_TIMEOUT, "LOGGER", "Error acquiring flash lock for log raw read");

    uint8_t lastb;
    bool lastb_set = false;
    bool done = false;
    bool wrapped = false;

    while (!done) {
        int tmp_len = min(sizeof(logger_tmp_buffer), LOG_MEMORY_SIZE_B - cur_end_address);
        esp_err_t e = logger_read_bytes_raw(logger, cur_end_address, tmp_len, logger_tmp_buffer);
        if (e != ESP_OK) {
            xSemaphoreGive(logger->log_flash_sem);
            return e;
        }
        // ESP_LOGI("LOGGER", "Refresh read %d bytes starting at address %d.", tmp_len, cur_end_address);
        // ESP_LOG_BUFFER_HEX("LOGREFRESH", logger_tmp_buffer, tmp_len);

        for (int i = 0; i < tmp_len; ++i) {
            if (!lastb_set) {
                lastb = logger_tmp_buffer[i];
                lastb_set = true;
            } else {
                if (logger_tmp_buffer[i] == 0xFF && lastb == 0xFF) {
                    // Done, found end of log!
                    cur_end_address += i;
                    done = true;
                    break;
                }
                lastb = logger_tmp_buffer[i];
            }
        }
        if (done) {
            break;
        }

        cur_end_address += tmp_len;
        cur_end_address %= LOG_MEMORY_SIZE_B;
        wrapped |= cur_end_address < logger->log_end_address;
        ESP_LOGI("LOGGER", "Refresh state: cur_end_address = %d, wrapped = %s", cur_end_address, wrapped ? "true" : "false");

        if (wrapped && cur_end_address > logger->log_end_address) {
            done = false;
            break;
        }
        // Just to reset the WDT!
        vTaskDelay(1);
    }


    if (done) {
        logger->log_end_address = cur_end_address;
        ESP_LOGI("LOGGER", "Refreshed log status, log_end_address = %d", logger->log_end_address);
    } else {
        ESP_LOGW("LOGGER", "No result from refresh!");
    }

    xSemaphoreGive(logger->log_flash_sem);

    return logger_flush(logger);
}

esp_err_t logger_queue_log_data_now(logger_t* logger, log_datatype_t typeid, uint8_t* data_allocd, size_t data_length) {
    int64_t timestamp = util_time_us();
    return logger_queue_log_data(logger, typeid, data_allocd, data_length, timestamp);
}

esp_err_t logger_queue_log_data(logger_t* logger, log_datatype_t typeid, uint8_t* data_allocd, size_t data_length, int64_t timestamp) {
    log_request_t req = {
        .data_length = data_length,
        .timestamp = timestamp,
        .data = data_allocd,
        .typeid = typeid
    };

    if (xQueueSend(logger->log_queue, &req, pdMS_TO_TICKS(5)) != pdPASS) {
        ESP_LOGE("LOGGER", "Unable to write queue!");
        free(req.data);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}


// TODO: Use a static sized buffer, but write the data not-raw
// uint8_t log_cobs_buf[sizeof(log_data_t) * 3];
uint8_t* log_cobs_buf;
size_t log_cobs_buf_size = 0;

esp_err_t logger_log_data_now(logger_t* logger, log_datatype_t typeid, uint8_t* data, size_t data_length) {
    int64_t timestamp = util_time_us();
    return logger_log_data(logger, typeid, data, data_length, timestamp);
}

ssize_t cobs_write_data(uint8_t* data_ptr, size_t data_len, uint8_t* buffer_ptr, size_t buffer_len) {
    ssize_t buf_idx = 0;
    for (int i = 0; i < data_len; ++i) {
        uint8_t b = data_ptr[i];
        if (buf_idx + 2 > buffer_len) {
            return -1;
        }

        if (b == 0xFF) {
            buffer_ptr[buf_idx++] = LOGGER_COBS_ESC;
            buffer_ptr[buf_idx++] = LOGGER_COBS_ESC_FF;
        } else if (b == 0xAA) {
            buffer_ptr[buf_idx++] = LOGGER_COBS_ESC;
            buffer_ptr[buf_idx++] = LOGGER_COBS_ESC_AA;
        } else {
            buffer_ptr[buf_idx++] = b;
        }
    }
    return buf_idx;
}

esp_err_t logger_log_data(logger_t* logger, log_datatype_t typeid, uint8_t* data, size_t data_length, int64_t timestamp) {
    ssize_t buf_idx = 0;
    // uint8_t* ts_bytes = (uint8_t*)(&timestamp);
    if (typeid < 0 || typeid > UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t typeid_u16 = typeid;

    // DONE: Detect log full/unable to write!

    // This will eventually settle to be a buffer big enough to store regular log data
    size_t num_bytes_alloc = (data_length + sizeof(int64_t) + sizeof(uint16_t) + 1) * 2;
    if (num_bytes_alloc > log_cobs_buf_size || log_cobs_buf == NULL) {
        if (log_cobs_buf != NULL)
            free(log_cobs_buf);

        log_cobs_buf = (uint8_t*)malloc(num_bytes_alloc);
        log_cobs_buf_size = num_bytes_alloc;
    }

    buf_idx += cobs_write_data((uint8_t*)(&timestamp), sizeof(timestamp), &log_cobs_buf[buf_idx], log_cobs_buf_size - buf_idx);
    if (buf_idx < 0) {
        return ESP_ERR_NO_MEM;
    }

    buf_idx += cobs_write_data((uint8_t*)(&typeid_u16), sizeof(typeid_u16), &log_cobs_buf[buf_idx], log_cobs_buf_size - buf_idx);
    if (buf_idx < 0) {
        return ESP_ERR_NO_MEM;
    }

    buf_idx += cobs_write_data(data, data_length, &log_cobs_buf[buf_idx], log_cobs_buf_size - buf_idx);
    if (buf_idx < 0) {
        return ESP_ERR_NO_MEM;
    }

    // Frame spacer!
    log_cobs_buf[buf_idx++] = 0xFF;

    // DONE: Write to flash "ring buffer"
    size_t write_len = buf_idx;


    ESP_RETURN_ON_FALSE(xSemaphoreTakeRecursive(logger->log_flash_sem, portMAX_DELAY) == pdPASS, ESP_ERR_TIMEOUT, "LOGGER", "Error acquiring flash lock for log write");
    size_t end_ptr = (logger->log_end_address + write_len + 1) % LOG_MEMORY_SIZE_B;
    bool wrapped = end_ptr < logger->log_end_address;
    bool unable_to_write = false;

    // If log is wrapped
    if (logger->log_start_address > logger->log_end_address) {
        unable_to_write = (end_ptr >= logger->log_start_address) || wrapped;
    } else {
        unable_to_write = wrapped && (end_ptr >= logger->log_start_address);
    }
    if (unable_to_write) {
        ESP_LOGW("LOGGER", "Error, unable to write!");
        xSemaphoreGive(logger->log_flash_sem);
        return ESP_ERR_NO_MEM;
    }

    int bytes_to_end = min(LOG_MEMORY_SIZE_B - logger->log_end_address, write_len);
    int bytes_from_start = (logger->log_end_address + write_len - LOG_MEMORY_SIZE_B);

    // ESP_LOGI("LOGGER", "Writing log bytes, %d from start, %d to end (log_end_address = %d)", bytes_from_start, bytes_to_end, logger->log_end_address);

    if (bytes_to_end > 0) {
        esp_err_t e = logger_write_bytes_raw(logger, logger->log_end_address, bytes_to_end, log_cobs_buf);
        if (e != ESP_OK) {
            ESP_LOGE("LOGGER", "Error writing bytes_to_end");
        }
        logger->log_end_address += bytes_to_end;
    }

    if (bytes_from_start > 0) {
        esp_err_t e = logger_write_bytes_raw(logger, 0, bytes_from_start, &log_cobs_buf[bytes_to_end]);
        if (e != ESP_OK) {
            ESP_LOGE("LOGGER", "Error writing bytes_from_start");
        }
        logger->log_end_address = bytes_from_start;
    }

    xSemaphoreGive(logger->log_flash_sem);

    return ESP_OK;
}