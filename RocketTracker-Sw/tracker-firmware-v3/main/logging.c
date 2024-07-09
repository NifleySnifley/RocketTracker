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
        ESP_ERROR_CHECK(nvs_get_u32(logger->log_nvs, NVS_LOG_START, &logger->log_start_address));
    }

    e = nvs_find_key(logger->log_nvs, NVS_LOG_END, NULL);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_u32(logger->log_nvs, NVS_LOG_END, 0);
    } else if (e != ESP_OK) {
        ESP_LOGE("LOGGER", "Error getting NVS key '%s'", NVS_LOG_END);
    } else {
        ESP_ERROR_CHECK(nvs_get_u32(logger->log_nvs, NVS_LOG_END, &logger->log_end_address));
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
    return ESP_OK;
}

esp_err_t logger_read_bytes_raw(logger_t* logger, size_t start_address, size_t size, uint8_t* buffer) {
    if (start_address + size > LOG_MEMORY_SIZE_B) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_partition_read_raw(logger->log_partition, start_address, buffer, size);
}

esp_err_t logger_write_bytes_raw(logger_t* logger, size_t start_address, size_t data_length, uint8_t* data) {
    if (start_address + data_length > LOG_MEMORY_SIZE_B) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_partition_write_raw(logger->log_partition, start_address, data, data_length);
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
    logger_flush(logger);

    return e;
}

esp_err_t logger_clean_log(logger_t* logger) {
    esp_err_t e = esp_partition_erase_range(logger->log_partition, 0, (LOG_MEMORY_SIZE_B / LOG_FLASH_SECTOR_SIZE + 1) * LOG_FLASH_SECTOR_SIZE);
    if (e != ESP_OK) {
        return e;
    }
    logger->log_start_address = 0;
    logger->log_end_address = 0;
    e = logger_flush(logger);
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

    uint8_t lastb;
    bool lastb_set = false;
    bool done = false;
    bool wrapped = false;

    while (!done) {
        int tmp_len = min(sizeof(logger_tmp_buffer), LOG_MEMORY_SIZE_B - cur_end_address);
        esp_err_t e = logger_read_bytes_raw(logger, cur_end_address, tmp_len, logger_tmp_buffer);
        if (e != ESP_OK) return e;
        // ESP_LOGI("LOGGER", "Refresh read %d bytes starting at address %d.", tmp_len, cur_end_address);

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
    }

    if (done) {
        logger->log_end_address = cur_end_address;
        ESP_LOGI("LOGGER", "Refreshed log status, log_end_address = %d", logger->log_end_address);
    } else {
        ESP_LOGW("LOGGER", "No result from refresh!");
    }

    return logger_flush(logger);
    return ESP_OK;
}

uint8_t log_cobs_buf[sizeof(log_data_t) * 3];
esp_err_t logger_log_data_now(logger_t* logger, uint8_t* data, size_t data_length) {
    size_t buf_idx = 0;
    int64_t timestamp = util_time_us();
    uint8_t* ts_bytes = (uint8_t*)(&timestamp);

    // TODO: Detect log full/unable to write!

    // Encode timestamp
    for (int i = 0; i < sizeof(timestamp); ++i) {
        uint8_t b = ts_bytes[i];
        if (buf_idx + 2 > sizeof(log_cobs_buf)) {
            return ESP_ERR_NO_MEM;
        }

        if (b == 0xFF) {
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC;
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC_FF;
        } else if (b == 0xAA) {
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC;
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC_AA;
        } else {
            log_cobs_buf[buf_idx++] = b;
        }
    }

    // Encode data payload
    for (int i = 0; i < data_length; ++i) {
        uint8_t b = data[i];
        if (buf_idx + 2 > sizeof(log_cobs_buf)) {
            return ESP_ERR_NO_MEM;
        }

        if (b == 0xFF) {
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC;
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC_FF;
        } else if (b == 0xAA) {
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC;
            log_cobs_buf[buf_idx++] = LOGGER_COBS_ESC_AA;
        } else {
            log_cobs_buf[buf_idx++] = b;
        }
    }

    // Frame spacer!
    log_cobs_buf[buf_idx++] = 0xFF;

    // TODO: Write to flash "ring buffer"
    size_t write_len = buf_idx;

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

    // logger_flush(logger);

    // Don't flush!

    return ESP_OK;
}